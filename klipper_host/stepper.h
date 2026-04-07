#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include <atomic>

class KlipperMCU;

// MCU_stepper: OID-based stepper driver.
// Wraps config_stepper, queue_step, set_next_step_dir MCU commands.
// Corresponds to Klipper's MCU_stepper (stepper.py).
class MCU_stepper {
public:
    explicit MCU_stepper(KlipperMCU& mcu);

    // Setup before finalize
    void setupPin(const std::string& stepPin, const std::string& dirPin);
    void setupStepPulseTicks(int stepPulseTicks);
    void setupInvertDir(bool invert);
    void setupStepDist(double rotationDist, int fullSteps, int microsteps,
                       double gearRatio = 1.0);

    // Build config commands (called during config finalization)
    bool buildConfig();

    // Runtime: send queue_step command
    bool queueStep(int64_t interval, int64_t count, int64_t add);
    bool queueStepBatched(int64_t interval, int64_t count, int64_t add);

    // Runtime: set next step direction (true = forward)
    bool setNextStepDir(bool forward);
    bool setNextStepDirBatched(bool forward);

    // Runtime: reset step clock
    bool resetStepClock(int64_t clock);
    bool resetStepClockBatched(int64_t clock);

    // Getters
    int getOid() const { return m_oid; }
    const std::string& getStepPinName() const { return m_stepPin; }
    const std::string& getDirPinName() const { return m_dirPin; }
    double getStepDist() const { return m_stepDist; }
    int64_t getCommandedPosition() const { return m_commandedPos; }
    void setCommandedPosition(int64_t pos) { m_commandedPos = pos; }
    bool getCurrentDir() const { return m_curDir; }

    // Synchronization: send stepper_get_position to MCU and wait for response.
    // Acts as a command-level barrier — MCU won't respond until all prior
    // commands/events have been processed. Returns current position, or
    // INT64_MIN on timeout.
    int64_t syncPosition(int timeoutMs = 2000);

    // Step clock tracking for inter-move continuity
    int64_t getLastStepClock() const { return m_lastStepClock; }
    void setLastStepClock(int64_t clock) { m_lastStepClock = clock; m_clockInitialized = true; }
    bool isClockInitialized() const { return m_clockInitialized; }
    void resetClockInitialized() { m_clockInitialized = false; }

private:
    KlipperMCU& m_mcu;
    std::string m_stepPin;
    std::string m_dirPin;
    int m_oid = -1;
    bool m_invertDir = false;
    int m_stepPulseTicks = 0;
    double m_stepDist = 0.0;  // mm per step
    int64_t m_commandedPos = 0;
    bool m_curDir = true;
    int64_t m_lastStepClock = 0;  // absolute clock of last queued step
    bool m_clockInitialized = false;
};

// MCU_endstop: OID-based endstop with trsync support for homing.
// Wraps config_endstop, endstop_home, trsync commands.
// Corresponds to Klipper's MCU_endstop (mcu.py).
class MCU_endstop {
public:
    explicit MCU_endstop(KlipperMCU& mcu);

    void setupPin(const std::string& pin, bool pullUp = true);

    bool buildConfig();

    // Home: trigger endstop homing sequence.
    // Returns true if endstop was triggered, false on timeout.
    bool home(int64_t homeClock, double sampleTime, int sampleCount,
              double restTime, bool pinValue,
              const std::vector<MCU_stepper*>& steppers,
              double triggerClearTime = 0.0);

    // Query endstop state (returns pin value)
    bool queryState(bool& triggered);

    int getOid() const { return m_oid; }
    int getTrsyncOid() const { return m_trsyncOid; }

private:
    KlipperMCU& m_mcu;
    std::string m_pin;
    bool m_pullUp = true;
    int m_oid = -1;
    int m_trsyncOid = -1;
};

// PrinterRail: bundles stepper(s) + endstop(s) + position limits.
// Corresponds to Klipper's PrinterRail (stepper.py).
class PrinterRail {
public:
    PrinterRail(MCU_stepper& stepper, MCU_endstop& endstop);

    void setPositionLimits(double posMin, double posMax);
    void setHomingSpeed(double speed);
    void setHomingRetractDist(double dist);
    void setSecondHomingSpeed(double speed);
    void setPositionEndstop(double pos);

    MCU_stepper& getStepper() { return m_stepper; }
    MCU_endstop& getEndstop() { return m_endstop; }

    double getPosMin() const { return m_posMin; }
    double getPosMax() const { return m_posMax; }
    double getHomingSpeed() const { return m_homingSpeed; }
    double getHomingRetractDist() const { return m_homingRetractDist; }
    double getSecondHomingSpeed() const { return m_secondHomingSpeed; }
    double getPositionEndstop() const { return m_posEndstop; }

    // Three-phase homing: fast home -> retract -> slow home
    bool homeAxis(KlipperMCU& mcu);

    void setHomingAccel(double accel) { m_homingAccel = accel; }
    double getHomingAccel() const { return m_homingAccel; }

private:
    MCU_stepper& m_stepper;
    MCU_endstop& m_endstop;
    double m_posMin = 0.0;
    double m_posMax = 200.0;
    double m_homingSpeed = 5.0;
    double m_homingRetractDist = 5.0;
    double m_secondHomingSpeed = 2.5;
    double m_posEndstop = 0.0;
    double m_homingAccel = 500.0;  // mm/s^2, acceleration ramp for homing

    // Helper: queue steps with acceleration ramp from rest to target speed,
    // then cruise. Used for homing phases.
    void queueHomingSteps(KlipperMCU& mcu, double speed, int64_t maxSteps,
                          int64_t startClock);
};
