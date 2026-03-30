#include "klipper_config.h"
#include "klipper_mcu.h"
#include "mcu_objects.h"
#include "stepper.h"
#include "toolhead.h"
#include "gcode.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <iostream>
#include <cctype>

// ========== ConfigSection helpers ==========

std::string ConfigSection::get(const std::string& key, const std::string& def) const {
    auto it = values.find(key);
    return (it != values.end()) ? it->second : def;
}

int ConfigSection::getInt(const std::string& key, int def) const {
    auto it = values.find(key);
    if (it == values.end()) return def;
    try { return std::stoi(it->second); }
    catch (...) { return def; }
}

double ConfigSection::getFloat(const std::string& key, double def) const {
    auto it = values.find(key);
    if (it == values.end()) return def;
    try { return std::stod(it->second); }
    catch (...) { return def; }
}

bool ConfigSection::getBool(const std::string& key, bool def) const {
    auto it = values.find(key);
    if (it == values.end()) return def;
    std::string v = it->second;
    std::transform(v.begin(), v.end(), v.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (v == "true" || v == "1" || v == "yes") return true;
    if (v == "false" || v == "0" || v == "no") return false;
    return def;
}

bool ConfigSection::has(const std::string& key) const {
    return values.count(key) > 0;
}

// ========== Pin helpers ==========

std::string KlipperConfig::cleanPin(const std::string& raw,
                                     bool& invert, bool& pullUp) {
    invert = false;
    pullUp = false;
    std::string pin = raw;

    // Trim whitespace
    while (!pin.empty() && std::isspace(static_cast<unsigned char>(pin.front())))
        pin.erase(pin.begin());
    while (!pin.empty() && std::isspace(static_cast<unsigned char>(pin.back())))
        pin.pop_back();

    // Strip comment after pin name
    auto spacePos = pin.find(' ');
    if (spacePos != std::string::npos)
        pin = pin.substr(0, spacePos);
    auto hashPos = pin.find('#');
    if (hashPos != std::string::npos)
        pin = pin.substr(0, hashPos);

    // Process prefixes
    std::string result;
    for (char c : pin) {
        if (c == '!') invert = true;
        else if (c == '^') pullUp = true;
        else if (c == '~') { /* pulldown, ignore */ }
        else result += c;
    }
    return result;
}

std::string KlipperConfig::stripPinPrefix(const std::string& raw) {
    // Remove "vref_scaled:" or other prefixes
    auto colonPos = raw.find(':');
    if (colonPos != std::string::npos)
        return raw.substr(colonPos + 1);
    return raw;
}

// ========== Parser ==========

bool KlipperConfig::parseFile(const std::string& path,
                               std::vector<ConfigSection>& sections,
                               std::string& error) {
    std::ifstream file(path);
    if (!file.is_open()) {
        error = "Cannot open config file: " + path;
        return false;
    }

    ConfigSection* current = nullptr;
    std::string line;
    int lineNum = 0;

    while (std::getline(file, line)) {
        lineNum++;

        // Strip trailing \r
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        // Strip comments
        auto hashPos = line.find('#');
        // Only strip if # is not inside a value (simple heuristic: strip if # appears after `:`)
        // Actually, for Klipper configs, inline comments with # after value are common
        // e.g. "endstop_pin: PD25 #IO0"
        // We need to be careful: only strip # if preceded by whitespace
        for (size_t i = 0; i < line.size(); i++) {
            if (line[i] == '#') {
                // If at start of line, or preceded by whitespace, it's a comment
                if (i == 0 || std::isspace(static_cast<unsigned char>(line[i - 1]))) {
                    line = line.substr(0, i);
                    break;
                }
            }
        }

        // Trim trailing whitespace
        while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back())))
            line.pop_back();

        if (line.empty()) continue;

        // Section header: [section_name]
        if (line.front() == '[') {
            auto endBracket = line.find(']');
            if (endBracket == std::string::npos) {
                error = "Malformed section header at line " + std::to_string(lineNum);
                return false;
            }
            std::string sectionName = line.substr(1, endBracket - 1);
            sections.push_back({sectionName, {}});
            current = &sections.back();
            continue;
        }

        // Key: value pair
        if (current) {
            auto colonPos = line.find(':');
            if (colonPos != std::string::npos) {
                std::string key = line.substr(0, colonPos);
                std::string val = line.substr(colonPos + 1);

                // Trim key
                while (!key.empty() && std::isspace(static_cast<unsigned char>(key.back())))
                    key.pop_back();
                while (!key.empty() && std::isspace(static_cast<unsigned char>(key.front())))
                    key.erase(key.begin());

                // Trim value
                while (!val.empty() && std::isspace(static_cast<unsigned char>(val.front())))
                    val.erase(val.begin());
                while (!val.empty() && std::isspace(static_cast<unsigned char>(val.back())))
                    val.pop_back();

                current->values[key] = val;
            }
        }
    }

    return true;
}

// ========== Object builders ==========

ConfigResult::StepperInfo KlipperConfig::buildStepper(
    KlipperMCU& mcu, const ConfigSection& section,
    std::vector<std::string>& warnings)
{
    ConfigResult::StepperInfo info;
    info.name = section.type;

    // Step/dir pins
    bool invertStep = false, pu = false;
    std::string stepPin = cleanPin(section.get("step_pin"), invertStep, pu);
    bool invertDir = false;
    std::string dirPin = cleanPin(section.get("dir_pin"), invertDir, pu);

    if (stepPin.empty() || dirPin.empty()) {
        warnings.push_back(info.name + ": missing step_pin or dir_pin");
        return info;
    }

    // Create stepper
    info.stepper = std::make_unique<MCU_stepper>(mcu);
    info.stepper->setupPin(stepPin, dirPin);
    info.stepper->setupInvertDir(invertDir);

    // Microsteps and rotation distance
    int microsteps = section.getInt("microsteps", 16);
    double rotationDist = section.getFloat("rotation_distance", 40.0);
    int fullSteps = section.getInt("full_steps_per_rotation", 200);
    double gearRatio = section.getFloat("gear_ratio", 1.0);

    info.stepper->setupStepDist(rotationDist, fullSteps, microsteps, gearRatio);
    info.stepper->buildConfig();

    // Enable pin → digital out
    if (section.has("enable_pin")) {
        bool invertEn = false, pullUpEn = false;
        std::string enPin = cleanPin(section.get("enable_pin"), invertEn, pullUpEn);
        // Enable pins are typically active-low (! prefix means invert)
        // We create it as a digital out, start with enabled state = !invert
        // (most Klipper configs use "!PA9" meaning the enable is active low)
    }

    // Endstop
    if (section.has("endstop_pin")) {
        bool invertEnd = false, pullUpEnd = false;
        std::string endPin = cleanPin(section.get("endstop_pin"), invertEnd, pullUpEnd);

        info.endstop = std::make_unique<MCU_endstop>(mcu);
        info.endstop->setupPin(endPin, pullUpEnd);
        info.endstop->buildConfig();

        // Create rail with position limits
        double posEndstop = section.getFloat("position_endstop", 0.0);
        double posMin = section.getFloat("position_min", 0.0);
        double posMax = section.getFloat("position_max", 200.0);
        double homingSpeed = section.getFloat("homing_speed", 5.0);
        double homingRetract = section.getFloat("homing_retract_dist", 5.0);
        double secondHomingSpeed = section.getFloat("second_homing_speed", homingSpeed / 2.0);

        info.rail = std::make_unique<PrinterRail>(
            *info.stepper, *info.endstop);
        info.rail->setPositionLimits(posMin, posMax);
        info.rail->setPositionEndstop(posEndstop);
        info.rail->setHomingSpeed(homingSpeed);
        info.rail->setHomingRetractDist(homingRetract);
        info.rail->setSecondHomingSpeed(secondHomingSpeed);
    }

    return info;
}

ConfigResult KlipperConfig::buildObjects(KlipperMCU& mcu,
                                          const std::vector<ConfigSection>& sections)
{
    ConfigResult result;

    for (auto& section : sections) {
        const std::string& type = section.type;

        // Stepper sections: stepper_x, stepper_y, stepper_z
        if (type.substr(0, 8) == "stepper_") {
            auto info = buildStepper(mcu, section, result.warnings);
            if (info.stepper) {
                std::cout << "[Config] Created " << info.name
                          << ": step=" << info.stepper->getStepPinName()
                          << " dir=" << info.stepper->getDirPinName()
                          << " step_dist=" << info.stepper->getStepDist()
                          << "mm" << std::endl;
                result.steppers.push_back(std::move(info));
            }
        }
        // Extruder: has stepper + heater + sensor
        else if (type == "extruder" || type.substr(0, 9) == "extruder_") {
            // Stepper part
            if (section.has("step_pin")) {
                auto info = buildStepper(mcu, section, result.warnings);
                if (info.stepper) {
                    info.name = type;
                    std::cout << "[Config] Created " << info.name
                              << " stepper: step=" << info.stepper->getStepPinName()
                              << " step_dist=" << info.stepper->getStepDist()
                              << "mm" << std::endl;
                    result.steppers.push_back(std::move(info));
                }
            }
            // Heater pin → PWM
            if (section.has("heater_pin")) {
                bool inv = false, pu = false;
                std::string pin = cleanPin(section.get("heater_pin"), inv, pu);

                auto pwm = std::make_unique<MCU_pwm>(mcu);
                pwm->setupPin(pin, inv);
                pwm->setupCycleTime(0.100, true);
                pwm->setupMaxDuration(5.0);
                pwm->setupStartValue(0.0, 0.0);
                pwm->buildConfig();

                std::cout << "[Config] Created " << type << " heater PWM: pin="
                          << pin << std::endl;
                result.pwmOutputs.push_back({type + "_heater", pin, std::move(pwm)});
            }
            // Sensor pin → ADC
            if (section.has("sensor_pin")) {
                std::string rawPin = section.get("sensor_pin");
                std::string pin = stripPinPrefix(rawPin);
                bool inv = false, pu = false;
                pin = cleanPin(pin, inv, pu);

                auto adc = std::make_unique<MCU_adc>(mcu);
                adc->setupPin(pin);
                adc->setupAdcSample(0.5, 0.001, 8, 0.0, 1.0, 0);
                adc->buildConfig();

                std::cout << "[Config] Created " << type << " sensor ADC: pin="
                          << pin << std::endl;
                result.adcInputs.push_back({type + "_sensor", pin, std::move(adc)});
            }
        }
        // Heater bed
        else if (type == "heater_bed") {
            if (section.has("heater_pin")) {
                bool inv = false, pu = false;
                std::string pin = cleanPin(section.get("heater_pin"), inv, pu);

                auto pwm = std::make_unique<MCU_pwm>(mcu);
                pwm->setupPin(pin, inv);
                pwm->setupCycleTime(0.100, true);
                pwm->setupMaxDuration(10.0);
                pwm->setupStartValue(0.0, 0.0);
                pwm->buildConfig();

                std::cout << "[Config] Created heater_bed PWM: pin=" << pin << std::endl;
                result.pwmOutputs.push_back({"heater_bed", pin, std::move(pwm)});
            }
            if (section.has("sensor_pin")) {
                std::string rawPin = section.get("sensor_pin");
                std::string pin = stripPinPrefix(rawPin);
                bool inv = false, pu = false;
                pin = cleanPin(pin, inv, pu);

                auto adc = std::make_unique<MCU_adc>(mcu);
                adc->setupPin(pin);
                adc->setupAdcSample(0.5, 0.001, 8, 0.0, 1.0, 0);
                adc->buildConfig();

                std::cout << "[Config] Created heater_bed sensor ADC: pin=" << pin << std::endl;
                result.adcInputs.push_back({"heater_bed_sensor", pin, std::move(adc)});
            }
        }
        // Fan
        else if (type == "fan" || type.substr(0, 11) == "heater_fan ") {
            if (section.has("pin")) {
                bool inv = false, pu = false;
                std::string pin = cleanPin(section.get("pin"), inv, pu);

                auto pwm = std::make_unique<MCU_pwm>(mcu);
                pwm->setupPin(pin, inv);
                pwm->setupCycleTime(0.100, false);
                pwm->setupMaxDuration(0.0);
                pwm->setupStartValue(0.0, 0.0);
                pwm->buildConfig();

                std::cout << "[Config] Created fan PWM (" << type << "): pin="
                          << pin << std::endl;
                result.pwmOutputs.push_back({type, pin, std::move(pwm)});
            }
        }
        // Printer section
        else if (type == "printer") {
            result.kinematics = section.get("kinematics", "cartesian");
            result.maxVelocity = section.getFloat("max_velocity", 300.0);
            result.maxAccel = section.getFloat("max_accel", 3000.0);
            result.squareCornerVelocity = section.getFloat("square_corner_velocity", 5.0);
            std::cout << "[Config] Printer: " << result.kinematics
                      << " vel=" << result.maxVelocity
                      << " accel=" << result.maxAccel << std::endl;
        }
        // TMC5160 sections (informational only for now)
        else if (type.substr(0, 7) == "tmc5160" || type.substr(0, 7) == "tmc2209") {
            std::cout << "[Config] Noted driver config: " << type << std::endl;
        }
        // Other sections: log as skipped
        else {
            if (type != "mcu" && type.substr(0, 10) != "adc_scaled")
                result.warnings.push_back("Skipped section: [" + type + "]");
        }
    }

    return result;
}

ConfigResult KlipperConfig::load(KlipperMCU& mcu, const std::string& path) {
    std::vector<ConfigSection> sections;
    std::string parseError;

    if (!parseFile(path, sections, parseError)) {
        ConfigResult r;
        r.lastError = parseError;
        return r;
    }

    std::cout << "[Config] Parsed " << sections.size() << " sections from "
              << path << std::endl;

    return buildObjects(mcu, sections);
}
