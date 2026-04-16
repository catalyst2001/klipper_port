#pragma once

#include "toolhead.h"
#include "stepper.h"

#include <string>
#include <map>
#include <functional>
#include <vector>

class KlipperMCU;

// GCodeParser: parses and dispatches G-code commands.
// Corresponds to Klipper's GCodeDispatch (gcode.py).
class GCodeParser {
public:
    explicit GCodeParser(ToolHead& toolhead, KlipperMCU& mcu);

    // Register axis rails for homing
    void addRail(int axis, PrinterRail* rail);

    // Set absolute/relative mode
    void setAbsoluteMode(bool absolute) { m_absoluteMode = absolute; }
    bool isAbsoluteMode() const { return m_absoluteMode; }
    bool isAbsoluteExtruderMode() const { return m_absoluteExtruderMode; }

    // Parse and execute a single line of G-code
    // Returns true if the command was recognized and executed
    bool executeLine(const std::string& line);

    // Parse and execute a sequence of lines (e.g. from a file)
    int executeBlock(const std::string& gcode);

    // Get current position as reported by toolhead
    Vec3 getPosition() const { return m_toolhead.getPosition(); }

    // Get the base position (G92 offset)
    Vec3 getBasePosition() const { return m_basePos; }

    // Get last error/status message
    const std::string& getLastMessage() const { return m_lastMsg; }

    // Check if any axis is homed
    bool isHomed(int axis) const;
    bool isAllHomed() const;

    // Feedrate in mm/s
    double getFeedrate() const { return m_feedrate; }
    void setFeedrate(double f) { m_feedrate = f; }

    // Speed factor (1.0 = normal, 2.0 = double speed)
    void setSpeedFactor(double f) { m_speedFactor = f; }
    double getSpeedFactor() const { return m_speedFactor; }
    double getExtrudeFactor() const { return m_extrudeFactor; }

    // Pressure advance settings (compatibility state).
    double getPressureAdvance() const { return m_pressureAdvance; }
    double getPressureAdvanceSmoothTime() const { return m_pressureAdvanceSmoothTime; }

    // Register custom command handler
    using CommandHandler = std::function<bool(const std::map<char, double>& params)>;
    void registerCommand(const std::string& cmd, CommandHandler handler);

private:
    ToolHead& m_toolhead;
    KlipperMCU& m_mcu;

    // Coordinate system
    Vec3 m_basePos;         // G92 offset
    double m_baseEPos = 0.0; // G92 offset for extruder
    bool m_absoluteMode = true;
    bool m_absoluteExtruderMode = true;
    double m_feedrate = 25.0;  // mm/s (default)
    double m_speedFactor = 1.0;  // speed multiplier (M220 equivalent)
    double m_extrudeFactor = 1.0;  // extrusion multiplier (M221 equivalent)
    double m_gcodeEPos = 0.0;      // logical E position from gcode stream
    double m_pressureAdvance = 0.0;
    double m_pressureAdvanceSmoothTime = 0.04;
    bool m_pressureAdvanceWarned = false;
    bool m_homed[3] = {false, false, false};

    // Axis rails for homing
    PrinterRail* m_rails[3] = {nullptr, nullptr, nullptr};

    std::string m_lastMsg;

    // Custom command handlers
    std::map<std::string, CommandHandler> m_customHandlers;

    // Parse parameters from a G-code line: "G1 X10.5 Y20 F300"
    // Returns command string "G1" and params map {'X':10.5, 'Y':20, 'F':300}
    struct ParsedLine {
        std::string command;      // "G0", "G1", "G28", "M82", etc.
        std::map<char, double> params;
    };
    ParsedLine parseLine(const std::string& line);

    // Built-in command handlers
    bool cmdG0G1(const std::map<char, double>& params);
    bool cmdG28(const std::map<char, double>& params);
    bool cmdG90(const std::map<char, double>& params);
    bool cmdG91(const std::map<char, double>& params);
    bool cmdG92(const std::map<char, double>& params);
    bool cmdM114(const std::map<char, double>& params);
    bool cmdM82(const std::map<char, double>& params);
    bool cmdM83(const std::map<char, double>& params);
    bool cmdM204(const std::map<char, double>& params);
    bool cmdM205(const std::map<char, double>& params);
    bool cmdM220(const std::map<char, double>& params);
    bool cmdM221(const std::map<char, double>& params);
    bool cmdM84(const std::map<char, double>& params);
    bool cmdM112(const std::map<char, double>& params);
    bool cmdM400(const std::map<char, double>& params);

    // Extended Klipper-style tuning commands (e.g. SET_VELOCITY_LIMIT).
    bool tryExecuteExtendedCommand(const std::string& line);
    bool cmdG2G3(bool clockwise, const std::map<char, double>& params);
};
