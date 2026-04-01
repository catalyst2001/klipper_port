#include "gcode.h"
#include "klipper_mcu.h"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ========== GCodeParser ==========

GCodeParser::GCodeParser(ToolHead& toolhead, KlipperMCU& mcu)
    : m_toolhead(toolhead), m_mcu(mcu)
{
}

void GCodeParser::addRail(int axis, PrinterRail* rail) {
    if (axis >= 0 && axis < 3)
        m_rails[axis] = rail;
}

bool GCodeParser::isHomed(int axis) const {
    return (axis >= 0 && axis < 3) ? m_homed[axis] : false;
}

bool GCodeParser::isAllHomed() const {
    return m_homed[0] && m_homed[1] && m_homed[2];
}

void GCodeParser::registerCommand(const std::string& cmd, CommandHandler handler) {
    m_customHandlers[cmd] = std::move(handler);
}

GCodeParser::ParsedLine GCodeParser::parseLine(const std::string& line) {
    ParsedLine result;

    // Strip comments (everything after ';')
    std::string cleaned = line;
    auto commentPos = cleaned.find(';');
    if (commentPos != std::string::npos)
        cleaned = cleaned.substr(0, commentPos);

    // Trim whitespace
    while (!cleaned.empty() && std::isspace(static_cast<unsigned char>(cleaned.front())))
        cleaned.erase(cleaned.begin());
    while (!cleaned.empty() && std::isspace(static_cast<unsigned char>(cleaned.back())))
        cleaned.pop_back();

    if (cleaned.empty()) return result;

    // Parse command letter + number: "G1", "M114", etc.
    // Then parse parameters: letter + number pairs
    size_t pos = 0;

    // Get command: first letter + digits
    if (pos < cleaned.size() && std::isalpha(static_cast<unsigned char>(cleaned[pos]))) {
        char cmdLetter = static_cast<char>(std::toupper(static_cast<unsigned char>(cleaned[pos])));
        pos++;
        size_t numStart = pos;
        while (pos < cleaned.size() && std::isdigit(static_cast<unsigned char>(cleaned[pos])))
            pos++;
        // Normalize: strip leading zeros so G00->G0, G01->G1, etc.
        std::string numPart = cleaned.substr(numStart, pos - numStart);
        // Convert to integer and back to remove leading zeros
        if (!numPart.empty()) {
            try { numPart = std::to_string(std::stoi(numPart)); } catch (...) {}
        }
        result.command = std::string(1, cmdLetter) + numPart;
    }

    // Skip whitespace
    while (pos < cleaned.size() && std::isspace(static_cast<unsigned char>(cleaned[pos])))
        pos++;

    // Parse parameters: X10.5 Y-20 F300
    while (pos < cleaned.size()) {
        if (std::isalpha(static_cast<unsigned char>(cleaned[pos]))) {
            char paramLetter = static_cast<char>(std::toupper(static_cast<unsigned char>(cleaned[pos])));
            pos++;
            size_t numStart = pos;
            // Allow negative sign and decimal point
            if (pos < cleaned.size() && (cleaned[pos] == '-' || cleaned[pos] == '+'))
                pos++;
            while (pos < cleaned.size() &&
                   (std::isdigit(static_cast<unsigned char>(cleaned[pos])) || cleaned[pos] == '.'))
                pos++;
            if (pos > numStart) {
                std::string numStr = cleaned.substr(numStart, pos - numStart);
                // Guard against strings like "-", "+", "." that aren't valid numbers
                try {
                    size_t idx = 0;
                    double val = std::stod(numStr, &idx);
                    if (idx > 0)
                        result.params[paramLetter] = val;
                } catch (...) {
                    // Skip unparseable parameter value
                }
            }
        } else {
            pos++;
        }
    }

    return result;
}

bool GCodeParser::executeLine(const std::string& line) {
    auto parsed = parseLine(line);
    if (parsed.command.empty()) return true; // empty line is OK

    // Check custom handlers first
    auto customIt = m_customHandlers.find(parsed.command);
    if (customIt != m_customHandlers.end()) {
        return customIt->second(parsed.params);
    }

    // Built-in commands
    if (parsed.command == "G0" || parsed.command == "G1") {
        return cmdG0G1(parsed.params);
    } else if (parsed.command == "G2" || parsed.command == "G3") {
        return cmdG2G3(parsed.command == "G2", parsed.params);
    } else if (parsed.command == "G28") {
        return cmdG28(parsed.params);
    } else if (parsed.command == "G90") {
        return cmdG90(parsed.params);
    } else if (parsed.command == "G91") {
        return cmdG91(parsed.params);
    } else if (parsed.command == "G92") {
        return cmdG92(parsed.params);
    } else if (parsed.command == "M114") {
        return cmdM114(parsed.params);
    } else if (parsed.command == "M84") {
        return cmdM84(parsed.params);
    } else if (parsed.command == "M112") {
        return cmdM112(parsed.params);
    } else if (parsed.command == "M400") {
        return cmdM400(parsed.params);
    }

    // Silently ignore common non-motion commands (M-codes for temp, fan, etc.)
    if (parsed.command[0] == 'M' || parsed.command == "G4" || parsed.command == "G21" || parsed.command == "G20") {
        return true; // silently skip
    }

    m_lastMsg = "Unknown command: " + parsed.command;
    return false;
}

int GCodeParser::executeBlock(const std::string& gcode) {
    std::istringstream ss(gcode);
    std::string line;
    int count = 0;
    while (std::getline(ss, line)) {
        if (executeLine(line))
            count++;
    }
    // Flush remaining moves
    m_toolhead.flush();
    m_toolhead.generateSteps();
    return count;
}

// G0/G1: Linear move
bool GCodeParser::cmdG0G1(const std::map<char, double>& params) {
    // Handle feedrate
    auto fIt = params.find('F');
    if (fIt != params.end()) {
        m_feedrate = fIt->second / 60.0;  // F is in mm/min, convert to mm/s
        if (m_feedrate <= 0) m_feedrate = 1.0;
    }

    Vec3 curPos = m_toolhead.getPosition();
    Vec3 target = curPos;

    if (m_absoluteMode) {
        auto xIt = params.find('X');
        if (xIt != params.end()) target.x = xIt->second + m_basePos.x;
        auto yIt = params.find('Y');
        if (yIt != params.end()) target.y = yIt->second + m_basePos.y;
        auto zIt = params.find('Z');
        if (zIt != params.end()) target.z = zIt->second + m_basePos.z;
    } else {
        auto xIt = params.find('X');
        if (xIt != params.end()) target.x += xIt->second;
        auto yIt = params.find('Y');
        if (yIt != params.end()) target.y += yIt->second;
        auto zIt = params.find('Z');
        if (zIt != params.end()) target.z += zIt->second;
    }

    m_toolhead.moveAbsolute(target, m_feedrate);
    return true;
}

// G2/G3: Arc move (clockwise / counterclockwise)
// Linearizes arc into small line segments
bool GCodeParser::cmdG2G3(bool clockwise, const std::map<char, double>& params) {
    // Handle feedrate
    auto fIt = params.find('F');
    if (fIt != params.end()) {
        m_feedrate = fIt->second / 60.0;
        if (m_feedrate <= 0) m_feedrate = 1.0;
    }

    Vec3 curPos = m_toolhead.getPosition();
    double startX = curPos.x;
    double startY = curPos.y;

    // Target position
    double endX = startX, endY = startY;
    double endZ = curPos.z;

    if (m_absoluteMode) {
        auto xIt = params.find('X');
        if (xIt != params.end()) endX = xIt->second + m_basePos.x;
        auto yIt = params.find('Y');
        if (yIt != params.end()) endY = yIt->second + m_basePos.y;
        auto zIt = params.find('Z');
        if (zIt != params.end()) endZ = zIt->second + m_basePos.z;
    } else {
        auto xIt = params.find('X');
        if (xIt != params.end()) endX += xIt->second;
        auto yIt = params.find('Y');
        if (yIt != params.end()) endY += yIt->second;
        auto zIt = params.find('Z');
        if (zIt != params.end()) endZ += zIt->second;
    }

    // I, J are always relative offsets from current position to arc center
    double i = 0.0, j = 0.0;
    auto iIt = params.find('I');
    if (iIt != params.end()) i = iIt->second;
    auto jIt = params.find('J');
    if (jIt != params.end()) j = jIt->second;

    double centerX = startX + i;
    double centerY = startY + j;

    double r1 = std::hypot(startX - centerX, startY - centerY);
    double r2 = std::hypot(endX - centerX, endY - centerY);
    double radius = (r1 + r2) / 2.0;

    if (radius < 1e-6) {
        // Degenerate arc — treat as linear move
        Vec3 target{endX, endY, endZ};
        m_toolhead.moveAbsolute(target, m_feedrate);
        return true;
    }

    double startAngle = std::atan2(startY - centerY, startX - centerX);
    double endAngle = std::atan2(endY - centerY, endX - centerX);

    double sweep;
    if (clockwise) {
        sweep = startAngle - endAngle;
        if (sweep <= 0) sweep += 2.0 * M_PI;
        sweep = -sweep; // negative for CW
    } else {
        sweep = endAngle - startAngle;
        if (sweep <= 0) sweep += 2.0 * M_PI;
    }

    // Number of segments: ~1mm per segment or at least 8
    int segments = static_cast<int>(std::abs(sweep) * radius / 1.0);
    if (segments < 8) segments = 8;
    if (segments > 360) segments = 360;

    double zStep = (endZ - curPos.z) / segments;

    for (int s = 1; s <= segments; s++) {
        double frac = static_cast<double>(s) / segments;
        double angle = startAngle + sweep * frac;
        Vec3 pt;
        pt.x = centerX + radius * std::cos(angle);
        pt.y = centerY + radius * std::sin(angle);
        pt.z = curPos.z + zStep * s;
        m_toolhead.moveAbsolute(pt, m_feedrate);
    }

    // Ensure we end exactly at the target
    Vec3 finalPt{endX, endY, endZ};
    m_toolhead.moveAbsolute(finalPt, m_feedrate);
    return true;
}

// G28: Home axes
bool GCodeParser::cmdG28(const std::map<char, double>& params) {
    bool homeX = params.count('X') > 0 || params.empty();
    bool homeY = params.count('Y') > 0 || params.empty();
    bool homeZ = params.count('Z') > 0 || params.empty();

    // Flush pending moves first
    m_toolhead.flush();
    m_toolhead.generateSteps();

    int axes[] = {0, 1, 2};
    bool homeFlags[] = {homeX, homeY, homeZ};
    const char* axisNames[] = {"X", "Y", "Z"};

    for (int i = 0; i < 3; ++i) {
        if (!homeFlags[i] || !m_rails[i]) continue;

        std::cout << "[GCode] Homing " << axisNames[i] << " axis..." << std::endl;
        if (m_rails[i]->homeAxis(m_mcu)) {
            m_homed[i] = true;
            std::cout << "[GCode] " << axisNames[i] << " homed OK" << std::endl;
        } else {
            m_lastMsg = std::string("Failed to home ") + axisNames[i] + " axis";
            std::cout << "[GCode] " << m_lastMsg << std::endl;
            return false;
        }
    }

    // Update toolhead position to endstop positions
    Vec3 pos = m_toolhead.getPosition();
    if (homeX && m_rails[0]) pos.x = m_rails[0]->getPositionEndstop();
    if (homeY && m_rails[1]) pos.y = m_rails[1]->getPositionEndstop();
    if (homeZ && m_rails[2]) pos.z = m_rails[2]->getPositionEndstop();
    m_toolhead.setPosition(pos);

    m_lastMsg = "ok";
    return true;
}

// G90: Absolute positioning
bool GCodeParser::cmdG90(const std::map<char, double>&) {
    m_absoluteMode = true;
    m_lastMsg = "ok";
    return true;
}

// G91: Relative positioning
bool GCodeParser::cmdG91(const std::map<char, double>&) {
    m_absoluteMode = false;
    m_lastMsg = "ok";
    return true;
}

// G92: Set position (coordinate offset)
bool GCodeParser::cmdG92(const std::map<char, double>& params) {
    Vec3 curPos = m_toolhead.getPosition();

    auto xIt = params.find('X');
    if (xIt != params.end()) m_basePos.x = curPos.x - xIt->second;
    auto yIt = params.find('Y');
    if (yIt != params.end()) m_basePos.y = curPos.y - yIt->second;
    auto zIt = params.find('Z');
    if (zIt != params.end()) m_basePos.z = curPos.z - zIt->second;

    m_lastMsg = "ok";
    return true;
}

// M114: Report position
bool GCodeParser::cmdM114(const std::map<char, double>&) {
    Vec3 pos = m_toolhead.getPosition();
    Vec3 gpos = {pos.x - m_basePos.x, pos.y - m_basePos.y, pos.z - m_basePos.z};

    std::ostringstream ss;
    ss << "X:" << gpos.x << " Y:" << gpos.y << " Z:" << gpos.z;
    m_lastMsg = ss.str();
    std::cout << "[GCode] " << m_lastMsg << std::endl;
    return true;
}

// M84: Disable steppers (no-op for now, clears homed state)
bool GCodeParser::cmdM84(const std::map<char, double>&) {
    m_homed[0] = m_homed[1] = m_homed[2] = false;
    m_lastMsg = "ok";
    return true;
}

// M112: Emergency stop
bool GCodeParser::cmdM112(const std::map<char, double>&) {
    m_mcu.sendCommand("emergency_stop");
    m_lastMsg = "Emergency stop triggered";
    return true;
}

// M400: Wait for moves to finish
bool GCodeParser::cmdM400(const std::map<char, double>&) {
    m_toolhead.flush();
    m_toolhead.generateSteps();
    m_lastMsg = "ok";
    return true;
}
