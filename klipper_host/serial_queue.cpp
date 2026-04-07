#include "serial_queue.h"
#include "serial_port.h"

#include <iostream>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <cassert>

// ======================================================================
// SerialQueue — background-threaded serial command scheduler
// Port of Klipper's serialqueue.c
// ======================================================================

SerialQueue::SerialQueue() {}

SerialQueue::~SerialQueue() {
    stop();
    for (auto* cq : m_allocatedCQs)
        delete cq;
}

// ---- Public API ----

bool SerialQueue::start(SerialPort& port, double baud_adjust,
                        uint8_t initial_send_seq, uint8_t initial_receive_seq) {
    if (m_running.load()) return false;
    m_port = &port;
    m_bittimeAdjust = baud_adjust;

    // Reset state
    m_inputPos = 0;
    m_needSync = true;
    // Sequence numbers must continue from KlipperMCU's current state.
    // The MCU remembers its expected host seq; starting from 1 would cause
    // a NAK storm because the MCU rejects frames with wrong sequence.
    m_sendSeq = initial_send_seq;
    m_receiveSeq = initial_send_seq;   // all prior sends are ACK'd
    m_ignoreNakSeq = 0;
    m_lastAckSeq = initial_send_seq > 0 ? initial_send_seq - 1 : 0;
    m_retransmitSeq = 0;
    m_rttSampleSeq = 0;
    m_srtt = 0.0;
    m_rttvar = 0.0;
    m_rto = SQ_MIN_RTO;
    m_readyBytes = 0;
    m_needAckBytes = 0;
    m_lastAckBytes = 0;
    m_upcomingBytes = 0;
    m_minReleaseClock = SQ_MAX_CLOCK;
    m_idleTime = 0.0;
    m_lastReceiveSentTime = 0.0;
    m_nextRetransmitTime = 0.0;
    m_lastWriteFailTime = 0.0;

    m_stopRequested.store(false);
    m_running.store(true, std::memory_order_release);

    bool ok = m_thread.start(threadEntry, this);
    if (!ok) {
        m_running.store(false);
        return false;
    }
    return true;
}

void SerialQueue::stop() {
    if (!m_running.load()) return;
    m_stopRequested.store(true, std::memory_order_release);
    kickThread();
    m_thread.join();
    m_running.store(false, std::memory_order_release);
}

void SerialQueue::send(CommandQueue* cq, const uint8_t* msg, int len,
                       uint64_t min_clock, uint64_t req_clock,
                       uint64_t notify_id) {
    QueueMessage qm;
    std::memcpy(qm.msg, msg, len);
    qm.len = len;
    qm.min_clock = min_clock;
    qm.req_clock = req_clock;
    qm.notify_id = notify_id;

    {
        PlatformLockGuard lk(m_upcomingLock);
        cq->upcoming.push_back(std::move(qm));
        cq->upcoming_bytes += len;
        m_upcomingBytes += len;

        // Track minimum release clock for quick rejection
        if (min_clock < m_minReleaseClock)
            m_minReleaseClock = min_clock;

        if (!cq->in_upcoming_list) {
            m_upcomingQueues.push_back(cq);
            cq->in_upcoming_list = true;
        }
        m_needKick = true;
    }
    kickThread();
}

bool SerialQueue::sendAndWait(CommandQueue* cq, const uint8_t* msg, int len,
                              uint64_t min_clock, uint64_t req_clock,
                              uint32_t timeoutMs) {
    // Use a unique notify_id
    static std::atomic<uint64_t> s_nextNotifyId{1};
    uint64_t myId = s_nextNotifyId.fetch_add(1, std::memory_order_relaxed);

    // Setup wait state
    std::atomic<int> state{0}; // 0=waiting, 1=done, -1=fail
    SQNotifyCallback prevCb;
    {
        PlatformLockGuard lk(m_lock);
        prevCb = m_notifyCallback;
        auto prevCbRef = prevCb;
        m_notifyCallback = [myId, &state, prevCbRef](uint64_t id, bool ok) {
            if (id == myId) {
                state.store(ok ? 1 : -1, std::memory_order_release);
            }
            if (prevCbRef) prevCbRef(id, ok);
        };
    }

    send(cq, msg, len, min_clock, req_clock, myId);

    // Wait
    bool result = false;
    {
        PlatformLockGuard lk(m_lock);
        while (state.load(std::memory_order_acquire) == 0) {
            if (!m_notifyCond.waitFor(m_lock, timeoutMs)) {
                break; // timeout
            }
        }
        result = (state.load(std::memory_order_acquire) == 1);
        m_notifyCallback = prevCb; // restore
    }
    return result;
}

CommandQueue* SerialQueue::allocCommandQueue() {
    auto* cq = new CommandQueue();
    m_allocatedCQs.push_back(cq);
    return cq;
}

void SerialQueue::updateClockEstimate(const SQClockEstimate& ce) {
    PlatformLockGuard lk(m_lock);
    m_ce = ce;
}

SQClockEstimate SerialQueue::getClockEstimate() const {
    // Read under lock (const_cast because PlatformLock is not const-friendly)
    auto& self = const_cast<SerialQueue&>(*this);
    PlatformLockGuard lk(self.m_lock);
    return self.m_ce;
}

void SerialQueue::setReceiveCallback(SQReceiveCallback cb) {
    PlatformLockGuard lk(m_lock);
    m_receiveCallback = std::move(cb);
}

void SerialQueue::setNotifyCallback(SQNotifyCallback cb) {
    PlatformLockGuard lk(m_lock);
    m_notifyCallback = std::move(cb);
}

bool SerialQueue::flush(uint32_t timeoutMs) {
    PlatformLockGuard lk(m_lock);
    auto deadline = MonotonicClock::now() + timeoutMs / 1000.0;
    while (m_readyBytes > 0 || !m_sentQueue.empty() || m_upcomingBytes > 0) {
        uint32_t remainMs = 0;
        if (timeoutMs > 0) {
            double remain = deadline - MonotonicClock::now();
            if (remain <= 0) return false;
            remainMs = static_cast<uint32_t>(remain * 1000);
        } else {
            remainMs = 1000;
        }
        m_notifyCond.waitFor(m_lock, remainMs);
        if (!m_running.load()) return false;
    }
    return true;
}

SerialQueue::Stats SerialQueue::getStats() const {
    Stats s;
    s.bytes_write = m_bytesWrite.load(std::memory_order_relaxed);
    s.bytes_read = m_bytesRead.load(std::memory_order_relaxed);
    s.bytes_retransmit = m_bytesRetransmit.load(std::memory_order_relaxed);
    s.bytes_invalid = m_bytesInvalid.load(std::memory_order_relaxed);
    s.retransmit_count = m_retransmitCount.load(std::memory_order_relaxed);
    auto& self = const_cast<SerialQueue&>(*this);
    PlatformLockGuard lk(self.m_lock);
    s.send_seq = static_cast<uint32_t>(self.m_sendSeq);
    s.receive_seq = static_cast<uint32_t>(self.m_receiveSeq);
    return s;
}

int SerialQueue::getReceiveWindow() const {
    auto& self = const_cast<SerialQueue&>(*this);
    PlatformLockGuard lk(self.m_lock);
    return self.m_receiveWindow;
}

void SerialQueue::setReceiveWindow(int window) {
    PlatformLockGuard lk(m_lock);
    m_receiveWindow = window;
}

// ---- Background thread ----

#ifdef _WIN32
unsigned __stdcall SerialQueue::threadEntry(void* arg) {
    static_cast<SerialQueue*>(arg)->threadMain();
    return 0;
}
#else
void* SerialQueue::threadEntry(void* arg) {
    static_cast<SerialQueue*>(arg)->threadMain();
    return nullptr;
}
#endif

void SerialQueue::threadMain() {
    while (!m_stopRequested.load(std::memory_order_acquire)) {
        double now = MonotonicClock::now();

        // 1. Read incoming data (non-blocking)
        processInput();

        // 2. Command event: move upcoming→ready, build+send commands
        double nextWake = commandEvent(now);

        // 3. Retransmit check
        if (m_nextRetransmitTime > 0.0 && now >= m_nextRetransmitTime) {
            double rt = retransmitEvent(now);
            if (rt > 0.0 && (nextWake <= 0.0 || rt < nextWake))
                nextWake = rt;
        }

        // 4. Sleep until next event or kick
        {
            PlatformLockGuard lk(m_lock);

            // Check if there's pending work
            bool hasWork = m_readyBytes > 0 || m_needKick;
            if (hasWork) {
                m_needKick = false;
                continue; // no sleep, process immediately
            }

            // Calculate sleep time
            uint32_t sleepMs = 25; // default wake interval
            if (nextWake > 0.0) {
                double delta = nextWake - MonotonicClock::now();
                if (delta > 0.0 && delta * 1000.0 < sleepMs)
                    sleepMs = static_cast<uint32_t>(delta * 1000.0 + 0.5);
                if (sleepMs < 1) sleepMs = 1;
            }

            m_wakeCond.waitFor(m_lock, sleepMs);
        }
    }
}

void SerialQueue::kickThread() {
    PlatformLockGuard lk(m_lock);
    m_needKick = true;
    m_wakeCond.notifyOne();
}

// ---- Input Processing ----

void SerialQueue::processInput() {
    if (!m_port || !m_port->isOpen()) return;

    // Non-blocking read
    int space = sizeof(m_inputBuf) - m_inputPos;
    if (space <= 0) {
        // Buffer full, force sync
        m_inputPos = 0;
        m_needSync = true;
        space = sizeof(m_inputBuf);
    }

    int n = m_port->read(m_inputBuf + m_inputPos, space, 0);
    if (n > 0) {
        m_inputPos += n;
        m_bytesRead.fetch_add(n, std::memory_order_relaxed);
    }

    // Parse messages
    while (m_inputPos >= MESSAGE_MIN) {
        int ret = check_message(m_needSync, m_inputBuf, m_inputPos);
        if (ret == 0) break; // need more data
        if (ret < 0) {
            // Skip bytes
            int skip = -ret;
            if (skip > m_inputPos) skip = m_inputPos;
            std::memmove(m_inputBuf, m_inputBuf + skip, m_inputPos - skip);
            m_inputPos -= skip;
            m_bytesInvalid.fetch_add(skip, std::memory_order_relaxed);
            continue;
        }

        // Valid message of length ret
        double eventtime = MonotonicClock::now();
        int msgLen = ret;

        // Extract sequence number
        uint8_t recv_seq_byte = m_inputBuf[MESSAGE_HEADER_SIZE - 1];
        uint8_t seq_nibble = recv_seq_byte & MESSAGE_SEQ_MASK;

        // Collect callback data under m_lock, then invoke callback
        // OUTSIDE m_lock to avoid recursive lock deadlock (SRWLOCK
        // is non-recursive; the callback may call updateClockEstimate
        // which also takes m_lock).
        bool hasPayload = false;
        uint8_t payloadBuf[MESSAGE_MAX];
        int payloadLen = 0;
        double cb_sent_time = 0.0, cb_receive_time = 0.0;

        {
            PlatformLockGuard lk(m_lock);

            // Handle ACK (sequence number in response acknowledges our sends)
            handleAck(seq_nibble);

            // Dispatch payload (skip header, trim trailer)
            int payloadStart = MESSAGE_HEADER_SIZE;
            int payloadEnd = msgLen - MESSAGE_TRAILER_SIZE;
            if (payloadEnd > payloadStart) {
                payloadLen = payloadEnd - payloadStart;
                std::memcpy(payloadBuf, m_inputBuf + payloadStart, payloadLen);
                cb_sent_time = m_lastReceiveSentTime;
                cb_receive_time = eventtime;
                hasPayload = true;

                handleMessage(m_inputBuf + payloadStart, payloadLen, eventtime);
            }
        }

        // Invoke user callback outside m_lock
        if (hasPayload && m_receiveCallback) {
            m_receiveCallback(payloadBuf, payloadLen, cb_sent_time, cb_receive_time);
        }

        // Consume message from buffer
        std::memmove(m_inputBuf, m_inputBuf + msgLen, m_inputPos - msgLen);
        m_inputPos -= msgLen;
    }
}

void SerialQueue::handleAck(uint8_t recv_seq) {
    // The MCU ACKs by echoing back the next expected sequence number.
    // Any sent blocks with seq < recv_seq are confirmed delivered.

    // Extend to 64-bit
    uint64_t ackSeq = m_receiveSeq;
    // Align to the received nibble
    int seqDelta = (recv_seq - static_cast<int>(ackSeq & MESSAGE_SEQ_MASK)) & MESSAGE_SEQ_MASK;
    if (seqDelta == 0) return; // not a new ACK, probably our own message
    ackSeq += seqDelta;

    if (ackSeq > m_sendSeq) {
        // Future ACK — ignore (shouldn't happen in normal operation)
        return;
    }

    double eventtime = MonotonicClock::now();

    // Free acknowledged sent blocks
    while (!m_sentQueue.empty()) {
        auto& front = m_sentQueue.front();
        uint64_t front_seq = m_lastAckSeq + 1;
        if (front_seq >= ackSeq) break;

        m_needAckBytes -= front.len;
        m_lastAckBytes += front.len;

        // RTT sample (only from the sequence we're tracking)
        if (m_rttSampleSeq && front_seq == m_rttSampleSeq - 1) {
            double rtt = eventtime - front.sent_time;
            if (m_srtt == 0.0) {
                m_srtt = rtt;
                m_rttvar = rtt / 2.0;
            } else {
                m_rttvar = (3.0 * m_rttvar + std::abs(rtt - m_srtt)) / 4.0;
                m_srtt = (7.0 * m_srtt + rtt) / 8.0;
            }
            m_rto = m_srtt + 4.0 * m_rttvar;
            if (m_rto < SQ_MIN_RTO) m_rto = SQ_MIN_RTO;
            if (m_rto > SQ_MAX_RTO) m_rto = SQ_MAX_RTO;
            m_rttSampleSeq = 0;
        }

        m_sentQueue.pop_front();
        m_lastAckSeq = front_seq;
    }

    m_receiveSeq = ackSeq;

    // Update last_receive_sent_time
    if (!m_sentQueue.empty()) {
        m_lastReceiveSentTime = m_sentQueue.front().sent_time;
    } else {
        m_lastReceiveSentTime = eventtime;
    }

    // Check for pending notifications
    auto it = m_notifyQueue.begin();
    while (it != m_notifyQueue.end()) {
        // notify_id stores the sequence number at time of send
        if (it->req_clock < ackSeq) {
            if (m_notifyCallback) {
                m_notifyCallback(it->notify_id, true);
            }
            it = m_notifyQueue.erase(it);
        } else {
            ++it;
        }
    }

    // Retransmit timer update
    if (m_sentQueue.empty()) {
        m_nextRetransmitTime = 0.0;
    } else {
        m_nextRetransmitTime = m_sentQueue.front().receive_time + m_rto;
    }

    // Wake flush/sendAndWait waiters
    m_notifyCond.notifyAll();
}

void SerialQueue::handleMessage(const uint8_t* msg, int len, double eventtime) {
    // The main thread callback handles dispatching to response handlers.
    // SerialQueue itself only needs to handle the sequence ACK mechanism,
    // which is done in handleAck() via the frame header.
    (void)msg; (void)len; (void)eventtime;
}

// ---- Command Sending ----

double SerialQueue::calculateBittime(int bytes) const {
    return (bytes * 10.0 + m_receiveWindow) * m_bittimeAdjust;
}

void SerialQueue::doWrite(const uint8_t* buf, int len) {
    int ret = m_port->write(buf, len);
    if (ret < 0) {
        double now = MonotonicClock::now();
        if (now > m_lastWriteFailTime + 5.0) {
            m_lastWriteFailTime = now;
            std::cerr << "[SerialQueue] Write failed" << std::endl;
        }
    }
}

double SerialQueue::checkSendCommand(int buflen, double eventtime) {
    // Check if we can send more data
    // MAX_PENDING_BLOCKS limits total outstanding data
    if (m_needAckBytes + buflen > SQ_MAX_PENDING_BLOCKS * MESSAGE_MAX)
        return eventtime + 0.001; // try again soon

    // Need ready bytes to send
    if (m_readyBytes == 0)
        return 0.0;  // nothing to send, no wake needed

    // If a full frame's worth of data is ready, check if the commands
    // are close enough to send immediately.  Commands with req_clocks
    // far in the future must still be held by the MIN_REQTIME_DELTA gate
    // to prevent 32-bit MCU clock wrapping issues.
    static constexpr int MESSAGE_PAYLOAD_MAX = MESSAGE_MAX - MESSAGE_MIN;
    if (m_readyBytes >= MESSAGE_PAYLOAD_MAX && m_ce.est_freq > 0.0) {
        // Find minimum req_clock across all ready queues
        uint64_t minReqClockFF = SQ_MAX_CLOCK;
        for (auto* cq : m_readyQueues) {
            if (!cq->ready.empty()) {
                uint64_t rc = cq->ready.front().req_clock;
                if (rc < minReqClockFF) minReqClockFF = rc;
            }
        }
        if (minReqClockFF < SQ_MAX_CLOCK) {
            double dt = eventtime - m_ce.conv_time;
            uint64_t ack_clock_ff = static_cast<uint64_t>(
                m_ce.conv_clock + static_cast<int64_t>(dt * m_ce.est_freq));
            uint64_t maxAhead = static_cast<uint64_t>(2.0 * m_ce.est_freq);
            if (minReqClockFF <= ack_clock_ff + maxAhead)
                return -1.0; // PR_NOW — close enough, send immediately
        } else {
            return -1.0; // No clock-gated commands, send immediately
        }
        // Fall through to normal MIN_REQTIME_DELTA gate
    } else if (m_readyBytes >= MESSAGE_PAYLOAD_MAX) {
        return -1.0; // No freq info, send immediately
    }

    // MIN_REQTIME_DELTA gate: don't send commands until their req_clock
    // is within ~100ms of the current estimated MCU clock.  This prevents
    // flooding the MCU with step commands far ahead of execution,
    // matching Python Klipper's serialqueue.c behavior.
    if (m_ce.est_freq > 0.0) {
        // Find minimum req_clock across all ready queues
        uint64_t minReqClock = SQ_MAX_CLOCK;
        for (auto* cq : m_readyQueues) {
            if (!cq->ready.empty()) {
                uint64_t rc = cq->ready.front().req_clock;
                if (rc < minReqClock) minReqClock = rc;
            }
        }
        if (minReqClock < SQ_MAX_CLOCK) {
            double dt = eventtime - m_ce.conv_time;
            uint64_t ack_clock = static_cast<uint64_t>(
                m_ce.conv_clock + static_cast<int64_t>(dt * m_ce.est_freq));
            if (m_needAckBytes > 0)
                ack_clock += static_cast<uint64_t>(calculateBittime(m_needAckBytes) * m_ce.est_freq);
            uint64_t reqDelta = static_cast<uint64_t>(SQ_MIN_REQTIME_DELTA * m_ce.est_freq);
            if (minReqClock > ack_clock + reqDelta) {
                // Command is too far in the future — sleep until it's within range
                double sleepTime = static_cast<double>(
                    static_cast<int64_t>(minReqClock - ack_clock - reqDelta)) / m_ce.est_freq;
                return eventtime + (std::max)(0.001, sleepTime);
            }
        }
    }

    return -1.0; // PR_NOW equivalent — send now
}

int SerialQueue::buildAndSendCommand(uint8_t* buf, int pending, double eventtime) {
    int len = MESSAGE_HEADER_SIZE;

    while (m_readyBytes > 0) {
        // Find highest priority message (lowest req_clock) across all ready queues
        uint64_t minClock = SQ_MAX_CLOCK;
        CommandQueue* bestCQ = nullptr;
        for (auto* cq : m_readyQueues) {
            if (cq->ready.empty()) continue;
            auto& front = cq->ready.front();
            if (front.req_clock < minClock) {
                minClock = front.req_clock;
                bestCQ = cq;
            }
        }
        if (!bestCQ) break;

        auto& qm = bestCQ->ready.front();

        // Don't include commands with req_clock too far in the future.
        // Sending 32-bit clock values > 2^31 ticks ahead of the MCU
        // causes "Timer too close" because the MCU's signed comparison
        // interprets them as being in the past.
        if (m_ce.est_freq > 0.0 && qm.req_clock < SQ_MAX_CLOCK) {
            double dt = eventtime - m_ce.conv_time;
            uint64_t ack_ck = static_cast<uint64_t>(
                m_ce.conv_clock + static_cast<int64_t>(dt * m_ce.est_freq));
            // Max 5 seconds ahead (~1.5B ticks at 300MHz, well under 2^31)
            uint64_t maxAhead = static_cast<uint64_t>(5.0 * m_ce.est_freq);
            if (qm.req_clock > ack_ck + maxAhead)
                break;  // stop filling frame — remaining commands too far ahead
        }

        // Check if it fits in this frame
        if (len + qm.len > MESSAGE_MAX - MESSAGE_TRAILER_SIZE)
            break;

        // Move from ready → frame buffer
        std::memcpy(&buf[len], qm.msg, qm.len);
        len += qm.len;
        m_readyBytes -= qm.len;

        // Handle notification
        if (qm.notify_id) {
            QueueMessage notifyMsg = qm;
            notifyMsg.req_clock = m_sendSeq; // store seq for ACK tracking
            m_notifyQueue.push_back(std::move(notifyMsg));
        }

        bestCQ->ready.pop_front();

        // Remove from readyQueues if empty
        if (bestCQ->ready.empty()) {
            bestCQ->in_ready_list = false;
            m_readyQueues.remove(bestCQ);
        }
    }

    // Build frame header + trailer
    len += MESSAGE_TRAILER_SIZE;
    buf[0] = static_cast<uint8_t>(len); // MESSAGE_POS_LEN
    buf[1] = MESSAGE_DEST | static_cast<uint8_t>(m_sendSeq & MESSAGE_SEQ_MASK);

    uint16_t crc = crc16_ccitt(buf, len - MESSAGE_TRAILER_SIZE);
    buf[len - 3] = static_cast<uint8_t>(crc >> 8);
    buf[len - 2] = static_cast<uint8_t>(crc & 0xff);
    buf[len - 1] = MESSAGE_SYNC;

    // Store in sent queue for retransmission
    QueueMessage sentMsg;
    std::memcpy(sentMsg.msg, buf, len);
    sentMsg.len = len;
    sentMsg.sent_time = eventtime;
    double idletime = (eventtime > m_idleTime) ? eventtime : m_idleTime;
    idletime += calculateBittime(pending + len);
    sentMsg.receive_time = idletime;

    if (m_sentQueue.empty())
        m_nextRetransmitTime = idletime + m_rto;

    if (!m_rttSampleSeq)
        m_rttSampleSeq = m_sendSeq;

    m_sendSeq++;
    m_needAckBytes += len;
    m_sentQueue.push_back(std::move(sentMsg));

    return len;
}

uint64_t SerialQueue::checkUpcomingQueues(uint64_t ack_clock) {
    PlatformLockGuard lk(m_upcomingLock);
    m_needKick = false;

    if (ack_clock < m_minReleaseClock) {
        return m_minReleaseClock;
    }

    uint64_t minStalledClock = SQ_MAX_CLOCK;

    auto it = m_upcomingQueues.begin();
    while (it != m_upcomingQueues.end()) {
        CommandQueue* cq = *it;
        bool notInReady = cq->ready.empty();

        // Move messages from upcoming to ready where ack_clock >= min_clock
        auto msgIt = cq->upcoming.begin();
        while (msgIt != cq->upcoming.end()) {
            if (ack_clock < msgIt->min_clock) {
                if (msgIt->min_clock < minStalledClock)
                    minStalledClock = msgIt->min_clock;
                break;
            }
            m_readyBytes += msgIt->len;
            m_upcomingBytes -= msgIt->len;
            cq->upcoming_bytes -= msgIt->len;

            cq->ready.push_back(std::move(*msgIt));
            msgIt = cq->upcoming.erase(msgIt);
        }

        // Remove from upcoming list if empty
        if (cq->upcoming.empty()) {
            cq->in_upcoming_list = false;
            it = m_upcomingQueues.erase(it);
        } else {
            ++it;
        }

        // Add to ready queues if it has messages
        if (notInReady && !cq->ready.empty()) {
            cq->in_ready_list = true;
            {
                // readyQueues is protected by m_lock, but we hold m_upcomingLock.
                // This is safe because the background thread holds m_lock only
                // briefly and never acquires m_upcomingLock while holding m_lock
                // (lock ordering: m_upcomingLock → m_lock is fine here because
                // buildAndSendCommand holds m_lock and never touches upcoming).
                // Actually, we need to be careful — readyQueues is on the main
                // lock. We'll push the CQ to ready under m_lock separately.
                // For now, since the background thread is the only one calling
                // this function, and it's the only one reading readyQueues,
                // this is safe without m_lock.
                m_readyQueues.push_back(cq);
            }
        }
    }

    m_minReleaseClock = minStalledClock;
    return minStalledClock;
}

double SerialQueue::commandEvent(double eventtime) {
    PlatformLockGuard lk(m_lock);

    // Calculate ack_clock: the MCU clock at which the MCU will have
    // finished receiving our outstanding data.
    // This is: current_mcu_clock_at(now + bittime_of_pending_data)
    double ack_time = eventtime;
    if (m_needAckBytes > 0)
        ack_time += calculateBittime(m_needAckBytes);

    uint64_t ack_clock;
    if (m_ce.est_freq > 0.0) {
        double dt = ack_time - m_ce.conv_time;
        ack_clock = static_cast<uint64_t>(
            m_ce.conv_clock + static_cast<int64_t>(dt * m_ce.est_freq));
    } else {
        ack_clock = 0; // no clock estimate yet — release all untimed commands
    }

    // Move upcoming → ready based on ack_clock
    uint64_t minStalledClock = checkUpcomingQueues(ack_clock);

    // Build and send command frames
    uint8_t buf[MESSAGE_MAX * SQ_MAX_PENDING_BLOCKS];
    int buflen = 0;
    double waketime = 0.0;

    for (;;) {
        double check = checkSendCommand(buflen, eventtime);
        if (check >= 0.0) {
            // Not ready to send (too much pending or nothing ready)
            if (check > 0.0 && (waketime <= 0.0 || check < waketime))
                waketime = check;
            break;
        }
        buflen += buildAndSendCommand(&buf[buflen], buflen, eventtime);
        if (buflen + MESSAGE_MAX > static_cast<int>(sizeof(buf)))
            break;
    }

    if (buflen > 0) {
        // Write all built frames at once
        doWrite(buf, buflen);
        m_bytesWrite.fetch_add(buflen, std::memory_order_relaxed);

        double idletime = (eventtime > m_idleTime) ? eventtime : m_idleTime;
        m_idleTime = idletime + calculateBittime(buflen);
        waketime = -1.0; // PR_NOW — check again immediately
    }

    // Calculate next wake time based on stalled clock
    if (minStalledClock < SQ_MAX_CLOCK && m_ce.est_freq > 0.0) {
        double stalledTime = m_ce.conv_time +
            static_cast<double>(static_cast<int64_t>(minStalledClock) - m_ce.conv_clock) / m_ce.est_freq;
        double stalledWake = stalledTime - SQ_MIN_REQTIME_DELTA;
        if (stalledWake > 0.0 && (waketime <= 0.0 || stalledWake < waketime))
            waketime = stalledWake;
    }

    return waketime;
}

double SerialQueue::retransmitEvent(double eventtime) {
    PlatformLockGuard lk(m_lock);

    if (m_sentQueue.empty()) {
        m_nextRetransmitTime = 0.0;
        return 0.0;
    }

    auto& oldest = m_sentQueue.front();
    double timeout = oldest.receive_time + m_rto;
    if (eventtime < timeout) {
        m_nextRetransmitTime = timeout;
        return timeout;
    }

    // Retransmit all outstanding blocks
    uint8_t buf[MESSAGE_MAX * SQ_MAX_PENDING_BLOCKS];
    int buflen = 0;

    for (auto& sent : m_sentQueue) {
        if (buflen + sent.len > static_cast<int>(sizeof(buf)))
            break;
        std::memcpy(&buf[buflen], sent.msg, sent.len);
        buflen += sent.len;
        sent.sent_time = eventtime;
    }

    if (buflen > 0) {
        doWrite(buf, buflen);
        m_bytesRetransmit.fetch_add(buflen, std::memory_order_relaxed);
        m_retransmitCount.fetch_add(1, std::memory_order_relaxed);
    }

    // Exponential backoff
    m_rto *= 2.0;
    if (m_rto > SQ_MAX_RTO) m_rto = SQ_MAX_RTO;
    m_rttSampleSeq = 0;  // Karn's algorithm: don't sample retransmitted segments
    m_ignoreNakSeq = m_sendSeq;

    m_nextRetransmitTime = eventtime + m_rto;
    return m_nextRetransmitTime;
}
