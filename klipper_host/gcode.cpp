#include "gcode.h"
#include "klipper_mcu.h"

#include <sstream>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <regex>
#include <thread>
#include <unordered_map>

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
    if (tryExecuteExtendedCommand(line)) {
        return true;
    }

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
    } else if (parsed.command == "M82") {
        return cmdM82(parsed.params);
    } else if (parsed.command == "M83") {
        return cmdM83(parsed.params);
    } else if (parsed.command == "M204") {
        return cmdM204(parsed.params);
    } else if (parsed.command == "M205") {
        return cmdM205(parsed.params);
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

bool GCodeParser::tryExecuteExtendedCommand(const std::string& line) {
    std::string cleaned = line;
    auto commentPos = cleaned.find(';');
    if (commentPos != std::string::npos)
        cleaned = cleaned.substr(0, commentPos);

    while (!cleaned.empty() && std::isspace(static_cast<unsigned char>(cleaned.front())))
        cleaned.erase(cleaned.begin());
    while (!cleaned.empty() && std::isspace(static_cast<unsigned char>(cleaned.back())))
        cleaned.pop_back();
    if (cleaned.empty())
        return false;

    std::istringstream ss(cleaned);
    std::string cmd;
    ss >> cmd;
    if (cmd.empty())
        return false;

    std::transform(cmd.begin(), cmd.end(), cmd.begin(),
        [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    std::unordered_map<std::string, std::string> kv;
    std::string token;
    while (ss >> token) {
        auto eq = token.find('=');
        if (eq == std::string::npos)
            continue;
        std::string key = token.substr(0, eq);
        std::string val = token.substr(eq + 1);
        if (key.empty() || val.empty())
            continue;
        std::transform(key.begin(), key.end(), key.begin(),
            [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
        kv[key] = val;
    }

    auto getFloat = [&](const char* key, double& out) -> bool {
        auto it = kv.find(key);
        if (it == kv.end())
            return false;
        try {
            out = std::stod(it->second);
            return true;
        } catch (...) {
            return false;
        }
    };

    if (cmd == "SET_VELOCITY_LIMIT") {
        double v;
        if (getFloat("VELOCITY", v) || getFloat("MAX_VELOCITY", v))
            m_toolhead.setMaxVelocity(v);

        double a;
        if (getFloat("ACCEL", a) || getFloat("MAX_ACCEL", a))
            m_toolhead.setMaxAccel(a);

        double scv;
        if (getFloat("SQUARE_CORNER_VELOCITY", scv))
            m_toolhead.setSquareCornerVelocity(scv);

        m_lastMsg = "ok";
        return true;
    }

    if (cmd == "SET_INPUT_SHAPER") {
        auto& shaper = m_toolhead.getInputShaper();

        auto setAxis = [&](int axis, const char* typeKey, const char* freqKey, const char* dampingKey) {
            ShaperType curType = shaper.getAxisType(axis);
            double curFreq = shaper.getAxisFrequency(axis);
            double curDamping = shaper.getAxisDampingRatio(axis);

            auto typeIt = kv.find(typeKey);
            if (typeIt != kv.end()) {
                curType = InputShaper::parseType(typeIt->second);
            } else {
                typeIt = kv.find("SHAPER_TYPE");
                if (typeIt != kv.end())
                    curType = InputShaper::parseType(typeIt->second);
            }

            double val;
            if (getFloat(freqKey, val))
                curFreq = val;
            else if (getFloat("SHAPER_FREQ", val))
                curFreq = val;

            if (getFloat(dampingKey, val))
                curDamping = val;
            else if (getFloat("DAMPING_RATIO", val))
                curDamping = val;

            shaper.setAxisShaper(axis, curType, curFreq, curDamping);
        };

        setAxis(0, "SHAPER_TYPE_X", "SHAPER_FREQ_X", "DAMPING_RATIO_X");
        setAxis(1, "SHAPER_TYPE_Y", "SHAPER_FREQ_Y", "DAMPING_RATIO_Y");

        m_lastMsg = "ok";
        return true;
    }

    if (cmd == "SET_PRESSURE_ADVANCE") {
        double advance;
        if (getFloat("ADVANCE", advance)) {
            m_pressureAdvance = (std::max)(0.0, advance);
        }

        double smoothTime;
        if (getFloat("SMOOTH_TIME", smoothTime)) {
            m_pressureAdvanceSmoothTime = (std::max)(0.0, smoothTime);
        }

        m_toolhead.setPressureAdvance(m_pressureAdvance, m_pressureAdvanceSmoothTime);

        if (!m_pressureAdvanceWarned) {
            std::cout << "[GCode] Pressure advance updated: advance="
                      << m_pressureAdvance << " smooth_time="
                      << m_pressureAdvanceSmoothTime << std::endl;
            m_pressureAdvanceWarned = true;
        }

        m_lastMsg = "ok";
        return true;
    }

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
    m_toolhead.generateSteps(true);
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
    double curE = m_toolhead.getExtruderPosition();
    double targetE = curE;

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

    auto eIt = params.find('E');
    if (eIt != params.end()) {
        if (m_absoluteExtruderMode)
            targetE = eIt->second + m_baseEPos;
        else
            targetE += eIt->second;
    }

    m_toolhead.moveAbsolute(target, m_feedrate * m_speedFactor, targetE);
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
    double curE = m_toolhead.getExtruderPosition();
    double startX = curPos.x;
    double startY = curPos.y;

    // Target position
    double endX = startX, endY = startY;
    double endZ = curPos.z;
    double endE = curE;

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

    auto eIt = params.find('E');
    if (eIt != params.end()) {
        if (m_absoluteExtruderMode)
            endE = eIt->second + m_baseEPos;
        else
            endE += eIt->second;
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
        m_toolhead.moveAbsolute(target, m_feedrate * m_speedFactor);
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
    double eStep = (endE - curE) / segments;

    for (int s = 1; s <= segments; s++) {
        double frac = static_cast<double>(s) / segments;
        double angle = startAngle + sweep * frac;
        Vec3 pt;
        pt.x = centerX + radius * std::cos(angle);
        pt.y = centerY + radius * std::sin(angle);
        pt.z = curPos.z + zStep * s;
        double pe = curE + eStep * s;
        m_toolhead.moveAbsolute(pt, m_feedrate * m_speedFactor, pe);
    }

    // Ensure we end exactly at the target
    Vec3 finalPt{endX, endY, endZ};
    m_toolhead.moveAbsolute(finalPt, m_feedrate * m_speedFactor, endE);
    return true;
}

// G28: Home axes
bool GCodeParser::cmdG28(const std::map<char, double>& params) {
    bool homeX = params.count('X') > 0 || params.empty();
    bool homeY = params.count('Y') > 0 || params.empty();
    bool homeZ = params.count('Z') > 0 || params.empty();

    // Flush pending moves and pause step gen thread during homing
    // (homing sends stepper commands directly, which conflicts with
    // concurrent step generation)
    m_toolhead.flush();
    m_toolhead.pauseStepGen();  // waits for in-flight generateSteps() to finish

    // Stop SerialQueue so homing can use the direct serial path.
    // Must happen before syncPosition: the SQ thread would otherwise
    // read responses from the serial port, causing syncPosition to timeout.
    // pauseStepGen guarantees no in-flight generateSteps, so no commands
    // are being submitted to SQ; stopping it is safe.
    m_mcu.stopSerialQueue();

    // Sync barrier: round-trip to MCU confirms all scheduled steps are settled
    for (int a = 0; a < 3; ++a) {
        if (m_rails[a])
            m_rails[a]->getStepper().syncPosition();
    }

    int axes[] = {0, 1, 2};
    bool homeFlags[] = {homeX, homeY, homeZ};
    const char* axisNames[] = {"X", "Y", "Z"};

    for (int i = 0; i < 3; ++i) {
        if (!homeFlags[i] || !m_rails[i]) continue;

        std::cout << "[GCode] Homing " << axisNames[i] << " axis..." << std::endl;
        if (m_rails[i]->homeAxis(m_mcu)) {
            m_homed[i] = true;
            // Send trsync_trigger to cleanly stop the trsync instance
            // (matches Python Klipper's trsync.stop() behavior)
            int trsyncOid = m_rails[i]->getEndstop().getTrsyncOid();
            std::map<std::string, int64_t> trParams = {
                {"oid", trsyncOid},
                {"reason", 2}  // REASON_HOST_REQUEST
            };
            m_mcu.sendCommand("trsync_trigger", trParams);
            // Drain any remaining trsync_state reports
            m_mcu.processIncoming(50);
            // Clear SF_NEED_RESET on MCU stepper (matches Python note_homing_end)
            m_rails[i]->getStepper().resetStepClock(0);
            // Sync barrier: round-trip confirms MCU finished this axis
            m_rails[i]->getStepper().syncPosition();
            std::cout << "[GCode] " << axisNames[i] << " homed OK" << std::endl;
        } else {
            m_lastMsg = std::string("Failed to home ") + axisNames[i] + " axis";
            std::cout << "[GCode] " << m_lastMsg << std::endl;
            m_mcu.startSerialQueue();
            m_toolhead.resumeStepGen();
            return false;
        }
    }

    // Update toolhead position to endstop positions
    Vec3 pos = m_toolhead.getPosition();
    if (homeX && m_rails[0]) pos.x = m_rails[0]->getPositionEndstop();
    if (homeY && m_rails[1]) pos.y = m_rails[1]->getPositionEndstop();
    if (homeZ && m_rails[2]) pos.z = m_rails[2]->getPositionEndstop();
    m_toolhead.setPosition(pos);

    // Reset step clock state and resume step gen thread.
    // Re-base print time with a sufficient buffer — homing consumed real time
    // but no print moves, so the old printTime is now too close to the MCU clock.
    m_toolhead.resetSyncState();
    m_mcu.stepSyncReset();

    // Re-sync clocks after long direct-path homing while SerialQueue is still
    // stopped (safe sendWithResponse path). This reduces stale estPrintTime
    // risk before scheduling the first post-homing print moves.
    if (!m_mcu.initClockSync()) {
        m_lastMsg = "Clock sync re-init failed after homing";
        m_toolhead.resumeStepGen();
        return false;
    }

    // Restart SerialQueue for clock-gated step delivery.
    // Must happen after resetSyncState (which resets stepper clock tracking)
    // and before resumeStepGen (which begins submitting commands to SQ).
    m_mcu.startSerialQueue();

    double newPrintTime = m_mcu.getClockSync().estimatedPrintTime() + 4.0;
    m_toolhead.setNextPrintTime(newPrintTime);
    m_toolhead.resumeStepGen();

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
    double curE = m_toolhead.getExtruderPosition();

    auto xIt = params.find('X');
    if (xIt != params.end()) m_basePos.x = curPos.x - xIt->second;
    auto yIt = params.find('Y');
    if (yIt != params.end()) m_basePos.y = curPos.y - yIt->second;
    auto zIt = params.find('Z');
    if (zIt != params.end()) m_basePos.z = curPos.z - zIt->second;
    auto eIt = params.find('E');
    if (eIt != params.end()) m_baseEPos = curE - eIt->second;

    m_lastMsg = "ok";
    return true;
}

// M114: Report position
bool GCodeParser::cmdM114(const std::map<char, double>&) {
    Vec3 pos = m_toolhead.getPosition();
    double ePos = m_toolhead.getExtruderPosition();
    Vec3 gpos = {pos.x - m_basePos.x, pos.y - m_basePos.y, pos.z - m_basePos.z};
    double ge = ePos - m_baseEPos;

    std::ostringstream ss;
    ss << "X:" << gpos.x << " Y:" << gpos.y << " Z:" << gpos.z << " E:" << ge;
    m_lastMsg = ss.str();
    std::cout << "[GCode] " << m_lastMsg << std::endl;
    return true;
}

bool GCodeParser::cmdM82(const std::map<char, double>&) {
    m_absoluteExtruderMode = true;
    m_lastMsg = "ok";
    return true;
}

bool GCodeParser::cmdM83(const std::map<char, double>&) {
    m_absoluteExtruderMode = false;
    m_lastMsg = "ok";
    return true;
}

// M204: Set acceleration (supports S, P, T with Klipper-like behavior)
bool GCodeParser::cmdM204(const std::map<char, double>& params) {
    auto sIt = params.find('S');
    auto pIt = params.find('P');
    auto tIt = params.find('T');

    if (sIt != params.end()) {
        m_toolhead.setMaxAccel((std::max)(1.0, sIt->second));
    } else {
        double accel = 0.0;
        if (pIt != params.end()) accel = (std::max)(accel, pIt->second);
        if (tIt != params.end()) accel = (std::max)(accel, tIt->second);
        if (accel > 0.0)
            m_toolhead.setMaxAccel(accel);
    }

    m_lastMsg = "ok";
    return true;
}

// M205: Set advanced motion parameters; currently maps X/Y to SCV.
bool GCodeParser::cmdM205(const std::map<char, double>& params) {
    auto xIt = params.find('X');
    auto yIt = params.find('Y');
    double scv = -1.0;
    if (xIt != params.end()) scv = xIt->second;
    if (yIt != params.end()) scv = (std::max)(scv, yIt->second);
    if (scv > 0.0)
        m_toolhead.setSquareCornerVelocity(scv);

    m_lastMsg = "ok";
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
    m_toolhead.generateSteps(true);
    m_lastMsg = "ok";
    return true;
}
