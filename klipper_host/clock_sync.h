#pragma once

#include <cstdint>
#include <atomic>
#include <mutex>
#include <tuple>

class KlipperMCU; // forward declaration

// Constants matching Klipper Python clocksync.py
constexpr double RTT_AGE = 0.000010 / (60.0 * 60.0);  // 10us per hour aging
constexpr double CLOCK_DECAY = 1.0 / 30.0;              // EMA weight
constexpr double TRANSMIT_EXTRA = 0.001;                 // 1ms safety margin

// Clock synchronization between host time and MCU clock.
// Uses exponential-decay weighted linear regression, matching Klipper's clocksync.py.
class ClockSync {
public:
    ClockSync();
    ~ClockSync();

    // Initialize clock sync after identify. Sends get_uptime + 8x get_clock to seed regression.
    bool connect(KlipperMCU& mcu);

    // Process a get_clock response. Called from the periodic poll or manually.
    // sentTime/receiveTime are host monotonic times (seconds since epoch).
    void handleClockResponse(uint32_t clock32, double sentTime, double receiveTime);

    // Convert host monotonic time to MCU clock value
    int64_t getClock(double eventTime) const;

    // Convert MCU clock to host time
    double estimateClockSystime(int64_t reqClock) const;

    // Convert print_time <-> MCU clock (using nominal mcu_freq)
    int64_t printTimeToClock(double printTime) const;
    double clockToPrintTime(int64_t clock) const;

    // Convert host time to print_time
    double estimatedPrintTime(double eventTime) const;

    // Extend 32-bit MCU clock to 64-bit (signed extension, for response timestamps)
    int64_t clock32ToClock64(uint32_t clock32) const;

    // Get MCU frequency
    double getMcuFreq() const { return m_mcuFreq; }

    // Is clock sync healthy? (fewer than 5 unanswered queries)
    bool isActive() const { return m_queriesPending.load() < 5; }

    // Get estimated frequency from regression
    double getEstimatedFreq() const;

    // Get last known 64-bit clock
    int64_t getLastClock() const { return m_lastClock; }

    // Debug info
    struct DebugInfo {
        double timeAvg;
        double clockAvg;
        double freq;
        double minHalfRtt;
        double predictionVariance;
        int64_t lastClock;
        int queriesPending;
    };
    DebugInfo getDebugInfo() const;

    // Increment/reset pending query counter (for periodic polling)
    void incrementPending() { m_queriesPending++; }
    void resetPending() { m_queriesPending = 0; }

private:
    // MCU clock frequency (from CLOCK_FREQ config)
    double m_mcuFreq = 1.0;

    // 64-bit extended clock
    int64_t m_lastClock = 0;

    // Clock estimate tuple: (sample_time, clock, freq)
    // Used for Python-side time conversions
    struct ClockEstimate {
        double sampleTime = 0.0;
        double clock = 0.0;
        double freq = 1.0;
    };
    mutable std::mutex m_mutex;
    ClockEstimate m_clockEst;

    // RTT tracking
    double m_minHalfRtt = 999999999.9;
    double m_minRttTime = 0.0;

    // Exponential moving average state (linear regression)
    double m_timeAvg = 0.0;
    double m_timeVariance = 0.0;
    double m_clockAvg = 0.0;
    double m_clockCovariance = 0.0;
    double m_predictionVariance = 0.0;
    double m_lastPredictionTime = 0.0;

    // Periodic query tracking
    std::atomic<int> m_queriesPending{0};

    // Extend 32-bit clock forward (unsigned wrap)
    int64_t extendClock(uint32_t clock32);
};
