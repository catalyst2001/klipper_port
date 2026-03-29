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

    // Register custom command handler
    using CommandHandler = std::function<bool(const std::map<char, double>& params)>;
    void registerCommand(const std::string& cmd, CommandHandler handler);

private:
    ToolHead& m_toolhead;
    KlipperMCU& m_mcu;

    // Coordinate system
    Vec3 m_basePos;         // G92 offset
    bool m_absoluteMode = true;
    double m_feedrate = 25.0;  // mm/s (default)
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
    bool cmdM84(const std::map<char, double>& params);
    bool cmdM112(const std::map<char, double>& params);
    bool cmdM400(const std::map<char, double>& params);
};
