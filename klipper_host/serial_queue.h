#pragma once

// SerialQueue: background-threaded serial command scheduler with clock-gating.
// Port of Klipper's serialqueue.c to C++ with platform-abstracted sync.
//
// Architecture:
//   - Each command stream (e.g. per-stepper) gets a CommandQueue.
//   - Commands are submitted via send() with min_clock/req_clock.
//   - Background thread moves commands from upcoming→ready when ack_clock >= min_clock.
//   - Ready commands are packed into frames ordered by req_clock (priority).
//   - Retransmission on timeout (RFC6298 adaptive RTO).
//   - ACKs update clock estimate for clock-gating decisions.

#include "platform_sync.h"
#include "klipper_proto.h"

#include <cstdint>
#include <vector>
#include <list>
#include <atomic>
#include <functional>

// ---- Constants (matching serialqueue.c) ----

static constexpr int64_t  SQ_MAX_CLOCK = 0x7fffffffffffffffLL;
static constexpr int64_t  SQ_BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000LL;
static constexpr double   SQ_MIN_RTO = 0.025;
static constexpr double   SQ_MAX_RTO = 5.000;
static constexpr int      SQ_MAX_PENDING_BLOCKS = 12;
static constexpr double   SQ_MIN_REQTIME_DELTA = 0.100;   // 100ms min ahead for scheduled cmds
static constexpr double   SQ_MIN_BACKGROUND_DELTA = 0.005;
static constexpr double   SQ_IDLE_QUERY_TIME = 1.0;

// ---- Queue Message ----

struct QueueMessage {
    uint8_t msg[MESSAGE_MAX];
    int len = 0;

    // Filled when on a command queue (upcoming/ready)
    uint64_t min_clock = 0;   // earliest MCU clock to send at
    uint64_t req_clock = 0;   // requested priority clock (lower = higher priority)

    // Filled when in sent queue (for retransmit tracking)
    double sent_time = 0.0;
    double receive_time = 0.0;  // estimated time MCU finishes receiving

    // Notification support (0 = no notification needed)
    uint64_t notify_id = 0;
};

// ---- Command Queue (per-stepper or per-command-stream) ----

struct CommandQueue {
    std::list<QueueMessage> upcoming;   // clock-gated, waiting for ack_clock
    std::list<QueueMessage> ready;      // released, ready to send
    int upcoming_bytes = 0;

    // Linked into SerialQueue lists
    bool in_ready_list = false;
    bool in_upcoming_list = false;
};

// ---- Clock Estimate (shared between SerialQueue and ClockSync) ----

struct SQClockEstimate {
    int64_t  last_clock = 0;
    int64_t  conv_clock = 0;
    double   conv_time = 0.0;
    double   est_freq = 1.0;
};

// ---- Callback for received responses ----
using SQReceiveCallback = std::function<void(const uint8_t* msg, int len,
                                              double sent_time, double receive_time)>;

// ---- Notify callback (for send_wait_ack) ----
using SQNotifyCallback = std::function<void(uint64_t notify_id, bool success)>;

// Forward declaration
class SerialPort;

// ---- SerialQueue ----

class SerialQueue : NonCopyable {
public:
    SerialQueue();
    ~SerialQueue();

    // Initialize with an open serial port handle (takes ownership of I/O).
    // The SerialPort object must remain valid for the lifetime of SerialQueue.
    bool start(SerialPort& port, double baud_adjust = 0.0);

    // Stop the background thread and clean up.
    void stop();

    // Submit a command to a specific command queue.
    // Non-blocking: adds to upcoming queue and kicks the background thread.
    // min_clock: earliest MCU clock the command may be sent (clock-gating)
    // req_clock: priority clock (lower = sent first among ready commands)
    // notify_id: if non-zero, SQNotifyCallback fires when ACK'd
    void send(CommandQueue* cq, const uint8_t* msg, int len,
              uint64_t min_clock = 0, uint64_t req_clock = 0,
              uint64_t notify_id = 0);

    // Send a command and block until ACK'd or timeout.
    // Returns true if ACK'd, false on timeout.
    bool sendAndWait(CommandQueue* cq, const uint8_t* msg, int len,
                     uint64_t min_clock, uint64_t req_clock,
                     uint32_t timeoutMs = 2000);

    // Allocate a new CommandQueue (caller owns lifetime, must outlive SerialQueue usage).
    CommandQueue* allocCommandQueue();

    // Get the default (non-timed) command queue.
    CommandQueue* getDefaultCommandQueue() { return &m_defaultCQ; }

    // Update clock estimate (called from ClockSync after processing get_clock responses).
    void updateClockEstimate(const SQClockEstimate& ce);

    // Get current clock estimate (thread-safe).
    SQClockEstimate getClockEstimate() const;

    // Set callback for received (non-ACK) messages.
    void setReceiveCallback(SQReceiveCallback cb);

    // Set callback for notify_id completion.
    void setNotifyCallback(SQNotifyCallback cb);

    // Flush: block until all currently-queued commands have been sent and ACK'd.
    // timeoutMs=0 means wait indefinitely.
    bool flush(uint32_t timeoutMs = 5000);

    // Check if background thread is running.
    bool isRunning() const { return m_running.load(std::memory_order_acquire); }

    // Get stats
    struct Stats {
        uint32_t bytes_write = 0;
        uint32_t bytes_read = 0;
        uint32_t bytes_retransmit = 0;
        uint32_t bytes_invalid = 0;
        uint32_t send_seq = 0;
        uint32_t receive_seq = 0;
        uint64_t retransmit_count = 0;
    };
    Stats getStats() const;

    // Get/set receive window (for baud rate calculations)
    int getReceiveWindow() const;
    void setReceiveWindow(int window);

private:
    // Background thread entry point
#ifdef _WIN32
    static unsigned __stdcall threadEntry(void* arg);
#else
    static void* threadEntry(void* arg);
#endif
    void threadMain();

    // Event loop phases
    void processInput();                        // read + parse incoming messages
    void handleAck(uint8_t recv_seq);           // process sequence ACK
    void handleMessage(const uint8_t* msg, int len, double eventtime);

    double checkSendCommand(int buflen, double eventtime);  // check if we should send
    int    buildAndSendCommand(uint8_t* buf, int pending, double eventtime);
    uint64_t checkUpcomingQueues(uint64_t ack_clock);       // upcoming → ready
    double commandEvent(double eventtime);                  // main send loop
    double retransmitEvent(double eventtime);               // retransmit timeout

    // Internal helpers
    double calculateBittime(int bytes) const;
    void doWrite(const uint8_t* buf, int len);
    void kickThread();  // wake up background thread

    // Serial port reference
    SerialPort* m_port = nullptr;

    // Background thread
    PlatformThread m_thread;
    std::atomic<bool> m_running{false};
    std::atomic<bool> m_stopRequested{false};

    // Main lock (protects all mutable state below)
    PlatformMutex m_lock;
    PlatformCondVar m_wakeCond;     // kick the background thread
    PlatformCondVar m_notifyCond;   // notify waiters (sendAndWait, flush)

    // Input buffer
    uint8_t m_inputBuf[4096];
    int m_inputPos = 0;
    bool m_needSync = true;

    // Clock tracking
    SQClockEstimate m_ce;
    int m_receiveWindow = 1;
    double m_bittimeAdjust = 0.0;
    double m_idleTime = 0.0;
    double m_lastReceiveSentTime = 0.0;

    // Sequence tracking
    uint64_t m_sendSeq = 1;    // 1-based like Python Klipper
    uint64_t m_receiveSeq = 1;
    uint64_t m_ignoreNakSeq = 0;
    uint64_t m_lastAckSeq = 0;
    uint64_t m_retransmitSeq = 0;
    uint64_t m_rttSampleSeq = 0;

    // Sent queue (for retransmission)
    std::list<QueueMessage> m_sentQueue;

    // RTT / RTO (RFC6298)
    double m_srtt = 0.0;
    double m_rttvar = 0.0;
    double m_rto = SQ_MIN_RTO;

    // Ready command queues (clock-gated, ready to send)
    std::list<CommandQueue*> m_readyQueues;
    int m_readyBytes = 0;
    int m_needAckBytes = 0;
    int m_lastAckBytes = 0;

    // Upcoming command queues (waiting for clock gate)
    PlatformMutex m_upcomingLock;   // separate lock for submit path
    std::list<CommandQueue*> m_upcomingQueues;
    int m_upcomingBytes = 0;
    uint64_t m_minReleaseClock = SQ_MAX_CLOCK;
    bool m_needKick = false;

    // Notify queue (messages awaiting ACK notification)
    std::list<QueueMessage> m_notifyQueue;

    // Default command queue
    CommandQueue m_defaultCQ;

    // Allocated command queues (for cleanup)
    std::vector<CommandQueue*> m_allocatedCQs;

    // Callbacks
    SQReceiveCallback m_receiveCallback;
    SQNotifyCallback m_notifyCallback;

    // Timer state
    double m_nextRetransmitTime = 0.0;  // 0 = disabled
    double m_lastWriteFailTime = 0.0;

    // Stats
    std::atomic<uint32_t> m_bytesWrite{0};
    std::atomic<uint32_t> m_bytesRead{0};
    std::atomic<uint32_t> m_bytesRetransmit{0};
    std::atomic<uint32_t> m_bytesInvalid{0};
    std::atomic<uint64_t> m_retransmitCount{0};
};
