#pragma once

#include "clock_sync.h"
#include "trapq.h"
#include "stepper.h"
#include "input_shaper.h"

#include <vector>
#include <deque>
#include <functional>
#include <mutex>

class KlipperMCU;

// ToolHead: manages motion planning with lookahead.
// Corresponds to Klipper's ToolHead (toolhead.py).
class ToolHead {
public:
    explicit ToolHead(KlipperMCU& mcu);

    // Configuration
    void setMaxVelocity(double maxVel);
    void setMaxAccel(double maxAccel);
    void setJunctionDeviation(double jd);
    void setSquareCornerVelocity(double scv);

    // Add axis stepper (0=X, 1=Y, 2=Z)
    void addStepper(int axis, MCU_stepper* stepper);

    // Absolute position move
    void moveAbsolute(const Vec3& pos, double speed);

    // Relative move
    void moveRelative(const Vec3& delta, double speed);

    // Flush all pending moves (force lookahead flush + step generation)
    void flush();

    // Get current position (after all moves)
    Vec3 getPosition() const { return m_pos; }
    void setPosition(const Vec3& pos) { m_pos = pos; }

    // Get the trapezoid queue (for external step generation)
    TrapQ& getTrapQ() { return m_trapq; }

    // Get config
    double getMaxVelocity() const { return m_maxVel; }
    double getMaxAccel() const { return m_maxAccel; }
    double getJunctionDeviation() const { return m_junctionDeviation; }

    // Get/set next print time for scheduling
    double getNextPrintTime() const { return m_nextPrintTime; }
    void setNextPrintTime(double t) { m_nextPrintTime = t; }

    // Stats
    size_t getQueueSize() const { return m_queue.size(); }

    // Generate and send queue_step commands for all steppers from flushed TrapMoves
    bool generateSteps();

    // Reset sync state — call at actual print start/end, not at every flush
    void resetSyncState() { m_needStartSync = true; m_needCheckPause = -1.0; m_stepClockSnapValid = false; }

    // Input shaper access
    InputShaper& getInputShaper() { return m_inputShaper; }
    const InputShaper& getInputShaper() const { return m_inputShaper; }

private:
    KlipperMCU& m_mcu;

    // Motion parameters
    double m_maxVel = 100.0;      // mm/s
    double m_maxAccel = 1000.0;   // mm/s^2
    double m_junctionDeviation = 0.02;  // mm (from square_corner_velocity)
    double m_squareCornerVelocity = 5.0; // mm/s
    double m_minCruiseRatio = 0.5;      // minimum cruise ratio [0, 1)
    double m_mcrPseudoAccel = 0.0;      // max_accel * (1 - min_cruise_ratio)

    // Current position
    Vec3 m_pos;

    // Print time tracking
    double m_nextPrintTime = 0.0;

    // Backpressure state (Klipper's special_queuing_state equivalent)
    bool m_needStartSync = true;   // true = idle, need to anchor print_time on first move
    double m_needCheckPause = -1.0; // print_time threshold for next pause check

    // Lookahead queue + time-based flush tracking
    std::deque<Move> m_queue;
    double m_junctionFlush = LOOKAHEAD_FLUSH_TIME; // countdown (seconds)

    // Axis steppers [X, Y, Z]
    MCU_stepper* m_steppers[3] = {nullptr, nullptr, nullptr};

    // Trapezoid move queue
    TrapQ m_trapq;

    // Input shaper
    InputShaper m_inputShaper;

    // Persistent clock snapshot for step clock computation.
    // Taken once at the start of step generation and reused for ALL batches
    // to avoid inter-call drift from changing frequency estimates.
    ClockSync::ClockSnapshot m_stepClockSnap{};
    bool m_stepClockSnapValid = false;

    // Lookahead: reverse + forward pass, then flush
    // lazy=true: only flush moves up to confirmed velocity peak (partial flush)
    // lazy=false: flush all moves (force complete stop at end)
    void lookaheadFlush(bool lazy);

    // Backpressure: sleep if host is too far ahead of MCU
    void checkPause();

    // Sync print_time to MCU clock on first move after idle
    void syncPrintTime();

    // Generate steps for a single axis from a TrapMove (analytical, no shaper)
    // needsReset: if true, send reset_step_clock before queue_step
    void generateAxisSteps(int axis, const TrapMove& tm, bool needsReset);

    // Generate steps for a single axis across all TrapMoves using input shaping
    // Uses secant/bisection method (Klipper's itersolve approach)
    void generateShapedAxisSteps(int axis, const std::vector<TrapMove>& moves,
                                  const ClockSync::ClockSnapshot& snap);

    // Clock-gate: wait until targetClock is within safe MCU timer range.
    // Returns false if MCU disconnected/shutdown (caller should abort).
    bool waitForClockGate(int64_t targetClock, double mcuFreq);

    // Helper: get axis position at any absolute print time across TrapMoves
    static double getAxisPositionAtTime(int axis, const std::vector<TrapMove>& moves,
                                        double printTime);
};
