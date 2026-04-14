#pragma once

#include "clock_sync.h"
#include "trapq.h"
#include "stepper.h"
#include "input_shaper.h"

#include <vector>
#include <deque>
#include <functional>
#include <algorithm>
#include <mutex>
#include <atomic>

class KlipperMCU;
class Reactor;

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
    void moveAbsolute(const Vec3& pos, double speed, double extruderPos);
    void moveAbsolute(const Vec3& pos, double speed) {
        moveAbsolute(pos, speed, m_ePos);
    }

    // Relative move
    void moveRelative(const Vec3& delta, double speed);

    // Flush all pending moves (force lookahead flush + step generation)
    void flush();

    // Get current position (after all moves)
    Vec3 getPosition() const { return m_pos; }
    void setPosition(const Vec3& pos) { m_pos = pos; }
    double getExtruderPosition() const { return m_ePos; }
    void setExtruderPosition(double e) { m_ePos = e; }

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

    // Generate and send queue_step commands for all steppers from flushed TrapMoves.
    // By default, only process a near-future time window (Python-style flush).
    bool generateSteps(bool flushAll = false);

    // Port of Python motion_queuing._advance_flush_time().
    bool advanceFlushTime(double wantFlushTime, double wantStepGenTime = 0.0);

    // Python motion_queuing-style tracking of pending MCU move queue activity.
    void noteMcuMovequeueActivity(double mqTime, bool isStepGen = true);
    double getLastFlushTime() const {
        return m_lastFlushTime.load(std::memory_order_acquire);
    }
    double getNeedFlushTime() const {
        return m_needFlushTime.load(std::memory_order_acquire);
    }
    double getNeedStepGenTime() const {
        return m_needStepGenTime.load(std::memory_order_acquire);
    }
    double calcStepGenRestart(double estPrintTime) const;
    bool consumeFlushKickRequest();
    void armFlushKickTimer() {
        m_doKickFlushTimer = true;
    }

    void setPressureAdvance(double advance, double smoothTime = 0.04) {
        m_pressureAdvance = (std::max)(0.0, advance);
        m_pressureAdvanceSmoothTime = (std::max)(0.0, smoothTime);
    }
    double getPressureAdvance() const { return m_pressureAdvance; }
    double getPressureAdvanceSmoothTime() const { return m_pressureAdvanceSmoothTime; }

    // Pause/resume step generation (for homing coordination)
    void pauseStepGen();
    void resumeStepGen();

    // Reset sync state — call at actual print start/end, not at every flush
    void resetSyncState() {
        m_needStartSync = true;
        m_needCheckPause = -1.0;
        m_stepClockSnapValid = false;
        m_needFlushTime.store(0.0, std::memory_order_release);
        m_needStepGenTime.store(0.0, std::memory_order_release);
        m_lastFlushTime.store(0.0, std::memory_order_release);
        m_stepGenPrintTime.store(0.0, std::memory_order_release);
        for (int i = 0; i < 4; i++)
            if (m_steppers[i]) m_steppers[i]->resetClockInitialized();
    }

    // Get the last print_time processed by stepGen (for backpressure)
    double getStepGenPrintTime() const {
        return m_stepGenPrintTime.load(std::memory_order_acquire);
    }

    // Input shaper access
    InputShaper& getInputShaper() { return m_inputShaper; }
    const InputShaper& getInputShaper() const { return m_inputShaper; }

    // Reactor support (for pause-based backpressure)
    void setReactor(Reactor* r) { m_reactor = r; }

    // Backpressure: pause if host is too far ahead of MCU
    void checkPause();

private:
    KlipperMCU& m_mcu;
    Reactor* m_reactor = nullptr;

    // Motion parameters
    double m_maxVel = 100.0;      // mm/s
    double m_maxAccel = 1000.0;   // mm/s^2
    double m_junctionDeviation = 0.02;  // mm (from square_corner_velocity)
    double m_squareCornerVelocity = 5.0; // mm/s
    double m_minCruiseRatio = 0.5;      // minimum cruise ratio [0, 1)
    double m_mcrPseudoAccel = 0.0;      // max_accel * (1 - min_cruise_ratio)

    // Current position
    Vec3 m_pos;
    double m_ePos = 0.0;

    // Print time tracking
    double m_nextPrintTime = 0.0;

    // Backpressure state (Klipper's special_queuing_state equivalent)
    bool m_needStartSync = true;   // true = idle, need to anchor print_time on first move
    double m_needCheckPause = -1.0; // print_time threshold for next pause check

    // Lookahead queue + time-based flush tracking
    std::deque<Move> m_queue;
    double m_junctionFlush = LOOKAHEAD_FLUSH_TIME; // countdown (seconds)

    // Axis steppers [X, Y, Z, E]
    MCU_stepper* m_steppers[4] = {nullptr, nullptr, nullptr, nullptr};

    // Pressure advance settings.
    double m_pressureAdvance = 0.0;
    double m_pressureAdvanceSmoothTime = 0.04;

    // Trapezoid move queue
    TrapQ m_trapq;

    // Input shaper
    InputShaper m_inputShaper;

    // Persistent clock snapshot for step clock computation.
    // Taken once at the start of step generation and reused for ALL batches
    // to avoid inter-call drift from changing frequency estimates.
    ClockSync::ClockSnapshot m_stepClockSnap{};
    bool m_stepClockSnapValid = false;


    // Step gen pause flag (for homing)
    std::atomic<bool> m_stepGenPaused{false};
    std::atomic<bool> m_stepGenRunning{false};

    // Python-style flush tracking.
    std::atomic<double> m_needFlushTime{0.0};
    std::atomic<double> m_needStepGenTime{0.0};
    std::atomic<double> m_lastFlushTime{0.0};
    bool m_doKickFlushTimer = true;
    bool m_flushKickRequested = false;

    // Backpressure: last print_time that stepGen has actually sent to MCU.
    // Equivalent to Python's last_step_gen_time.
    std::atomic<double> m_stepGenPrintTime{0.0};

    // Lookahead: reverse + forward pass, then flush
    // lazy=true: only flush moves up to confirmed velocity peak (partial flush)
    // lazy=false: flush all moves (force complete stop at end)
    void lookaheadFlush(bool lazy);

    // Sync print_time to MCU clock on first move after idle
    void syncPrintTime();

    // Generate steps for a single axis from a TrapMove (analytical, no shaper)
    // needsReset: if true, send reset_step_clock before queue_step
    void generateAxisSteps(int axis, const TrapMove& tm, bool needsReset);

    // Generate steps for a single axis across all TrapMoves using input shaping
    // Uses secant/bisection method (Klipper's itersolve approach)
    void generateShapedAxisSteps(int axis, const std::vector<TrapMove>& moves,
                                  const ClockSync::ClockSnapshot& snap);

    // Generate extruder steps with pressure-advance smoothing semantics.
    void generatePressureAdvanceExtruderSteps(const std::vector<TrapMove>& moves,
                                              const ClockSync::ClockSnapshot& snap);

    // Pressure-advance position helpers (mirroring kin_extruder integration model).
    double calcExtruderPaPositionAtTime(const std::vector<TrapMove>& moves,
                                        double printTime) const;
    double calcExtruderPaPositionSmooth(const std::vector<TrapMove>& moves,
                                        size_t moveIdx, double moveTime) const;

    // Helper: get axis position at any absolute print time across TrapMoves
    static double getAxisPositionAtTime(int axis, const std::vector<TrapMove>& moves,
                                        double printTime);
};
