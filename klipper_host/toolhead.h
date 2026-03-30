#pragma once

#include "trapq.h"
#include "stepper.h"

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

private:
    KlipperMCU& m_mcu;

    // Motion parameters
    double m_maxVel = 100.0;      // mm/s
    double m_maxAccel = 1000.0;   // mm/s^2
    double m_junctionDeviation = 0.02;  // mm (from square_corner_velocity)
    double m_squareCornerVelocity = 5.0; // mm/s

    // Current position
    Vec3 m_pos;

    // Print time tracking
    double m_nextPrintTime = 0.1;

    // Lookahead queue
    std::deque<Move> m_queue;

    // Axis steppers [X, Y, Z]
    MCU_stepper* m_steppers[3] = {nullptr, nullptr, nullptr};

    // Trapezoid move queue
    TrapQ m_trapq;

    // Lookahead: reverse + forward pass, then flush
    void lookaheadFlush(bool forceFlush);

    // Generate steps for a single axis from a TrapMove
    // needsReset: if true, send reset_step_clock before queue_step
    void generateAxisSteps(int axis, const TrapMove& tm, bool needsReset);
};
