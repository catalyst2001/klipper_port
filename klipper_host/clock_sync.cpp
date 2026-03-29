#include "clock_sync.h"
#include "klipper_mcu.h"
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>

// Host monotonic time in seconds (steady_clock)
static double hostTime() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

ClockSync::ClockSync() = default;
ClockSync::~ClockSync() = default;

bool ClockSync::connect(KlipperMCU& mcu) {
    // Get MCU frequency from config
    auto& cfg = mcu.getConfig();
    auto it = cfg.find("CLOCK_FREQ");
    if (it != cfg.end()) {
        m_mcuFreq = static_cast<double>(it->second);
    }
    else {
        m_mcuFreq = 1.0;
        return false;
    }

    // Step 1: get_uptime to get initial 64-bit clock
    std::map<std::string, int64_t> intP;
    std::map<std::string, std::vector<uint8_t>> bufP;
    double sentTime = hostTime();
    if (!mcu.sendWithResponse("get_uptime", "uptime", intP, bufP)) {
        return false;
    }
    double recvTime = hostTime();

    uint32_t high = static_cast<uint32_t>(intP["high"]);
    uint32_t clk = static_cast<uint32_t>(intP["clock"]);
    m_lastClock = (static_cast<int64_t>(high) << 32) | clk;

    double midTime = (sentTime + recvTime) * 0.5;
    m_clockAvg = static_cast<double>(m_lastClock);
    m_timeAvg = midTime;
    m_clockEst = {m_timeAvg, m_clockAvg, m_mcuFreq};
    m_predictionVariance = (0.001 * m_mcuFreq) * (0.001 * m_mcuFreq);

    // Step 2: Send 8 get_clock calls at 50ms intervals to seed regression
    for (int i = 0; i < 8; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        m_lastPredictionTime = -9999.0; // Force acceptance during init

        sentTime = hostTime();
        intP.clear();
        bufP.clear();
        if (!mcu.sendWithResponse("get_clock", "clock", intP, bufP)) {
            return false;
        }
        recvTime = hostTime();

        uint32_t clock32 = static_cast<uint32_t>(intP["clock"]);
        handleClockResponse(clock32, sentTime, recvTime);
    }

    std::cout << "[ClockSync] Initialized: freq=" << std::fixed
              << getEstimatedFreq() << " Hz, last_clock=" << m_lastClock << std::endl;
    return true;
}

int64_t ClockSync::extendClock(uint32_t clock32) {
    // Unsigned forward extension (like Klipper's _handle_clock)
    uint32_t lastLow = static_cast<uint32_t>(m_lastClock & 0xFFFFFFFF);
    uint32_t delta = clock32 - lastLow; // unsigned wrap handles overflow
    m_lastClock += delta;
    return m_lastClock;
}

void ClockSync::handleClockResponse(uint32_t clock32, double sentTime, double receiveTime) {
    // Step 1: Extend 32-bit to 64-bit clock
    int64_t clock = extendClock(clock32);

    // Step 2: Track minimum RTT
    if (sentTime == 0.0) return;
    double halfRtt = 0.5 * (receiveTime - sentTime);
    double agedRtt = (sentTime - m_minRttTime) * RTT_AGE;
    if (halfRtt < m_minHalfRtt + agedRtt) {
        m_minHalfRtt = halfRtt;
        m_minRttTime = sentTime;
    }

    // Use midpoint of sent/receive as the "sample time"
    double sampleTime = sentTime;

    // Step 3: Outlier rejection
    double clockD = static_cast<double>(clock);
    double expClock = (sampleTime - m_timeAvg) * m_clockEst.freq + m_clockAvg;
    double clockDiff2 = (clockD - expClock) * (clockD - expClock);

    double freqThreshold = 0.000500 * m_mcuFreq;
    if (clockDiff2 > 25.0 * m_predictionVariance &&
        clockDiff2 > freqThreshold * freqThreshold) {
        if (clockD > expClock && sampleTime < m_lastPredictionTime + 10.0) {
            // Clock ahead + recent good data → discard sample
            return;
        }
        // Reset prediction variance
        m_predictionVariance = (0.001 * m_mcuFreq) * (0.001 * m_mcuFreq);
    }
    else {
        m_lastPredictionTime = sampleTime;
        m_predictionVariance = (1.0 - CLOCK_DECAY) *
            (m_predictionVariance + clockDiff2 * CLOCK_DECAY);
    }

    // Step 4: Online linear regression (EMA)
    double diffSentTime = sampleTime - m_timeAvg;
    m_timeAvg += CLOCK_DECAY * diffSentTime;
    m_timeVariance = (1.0 - CLOCK_DECAY) *
        (m_timeVariance + diffSentTime * diffSentTime * CLOCK_DECAY);

    double diffClock = clockD - m_clockAvg;
    m_clockAvg += CLOCK_DECAY * diffClock;
    m_clockCovariance = (1.0 - CLOCK_DECAY) *
        (m_clockCovariance + diffSentTime * diffClock * CLOCK_DECAY);

    // Step 5: Compute new frequency and publish estimate
    double newFreq = m_mcuFreq; // fallback
    if (m_timeVariance > 0.0) {
        newFreq = m_clockCovariance / m_timeVariance;
    }

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        // Python-side estimate: time_avg + min_half_rtt, clock_avg, new_freq
        m_clockEst = {m_timeAvg + m_minHalfRtt, m_clockAvg, newFreq};
    }

    // Reset pending counter (we got a response)
    m_queriesPending = 0;
}

int64_t ClockSync::getClock(double eventTime) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return static_cast<int64_t>(
        m_clockEst.clock + (eventTime - m_clockEst.sampleTime) * m_clockEst.freq);
}

double ClockSync::estimateClockSystime(int64_t reqClock) const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return static_cast<double>(reqClock - m_clockEst.clock) / m_clockEst.freq
           + m_clockEst.sampleTime;
}

int64_t ClockSync::printTimeToClock(double printTime) const {
    return static_cast<int64_t>(printTime * m_mcuFreq);
}

double ClockSync::clockToPrintTime(int64_t clock) const {
    return static_cast<double>(clock) / m_mcuFreq;
}

double ClockSync::estimatedPrintTime(double eventTime) const {
    return clockToPrintTime(getClock(eventTime));
}

double ClockSync::estimatedPrintTime() const {
    return estimatedPrintTime(hostTime());
}

int64_t ClockSync::clock32ToClock64(uint32_t clock32) const {
    // Signed extension (handles both forward and backward deltas)
    int64_t lastClock = m_lastClock;
    uint32_t lastLow = static_cast<uint32_t>(lastClock & 0xFFFFFFFF);
    uint32_t diff = clock32 - lastLow;
    // Convert to signed: if bit 31 set, treat as negative
    int32_t signedDiff = static_cast<int32_t>(diff);
    return lastClock + signedDiff;
}

double ClockSync::getEstimatedFreq() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_clockEst.freq;
}

ClockSync::DebugInfo ClockSync::getDebugInfo() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return {
        m_timeAvg,
        m_clockAvg,
        m_clockEst.freq,
        m_minHalfRtt,
        m_predictionVariance,
        m_lastClock,
        m_queriesPending.load()
    };
}
