#pragma once

#include <string>
#include <map>
#include <vector>
#include <memory>

class KlipperMCU;
class MCU_stepper;
class MCU_endstop;
class MCU_digital_out;
class MCU_pwm;
class MCU_adc;
class PrinterRail;
class ToolHead;
class GCodeParser;
enum class ShaperType;

// Parsed section from a Klipper .cfg file
struct ConfigSection {
    std::string type;    // e.g. "stepper_x", "extruder", "printer"
    std::map<std::string, std::string> values;

    std::string get(const std::string& key, const std::string& def = "") const;
    int getInt(const std::string& key, int def = 0) const;
    double getFloat(const std::string& key, double def = 0.0) const;
    bool getBool(const std::string& key, bool def = false) const;
    bool has(const std::string& key) const;
};

// Result of loading a config: all created objects.
struct ConfigResult {
    // Steppers in order (X, Y, Z, extruder, ...)
    struct StepperInfo {
        std::string name;          // "stepper_x", "stepper_y", etc.
        std::unique_ptr<MCU_stepper> stepper;
        std::unique_ptr<MCU_endstop> endstop;
        std::unique_ptr<PrinterRail> rail;
        std::string enablePinRaw;  // raw enable_pin from config (e.g. "!PA9")
    };
    std::vector<StepperInfo> steppers;

    // PWM outputs (heaters, fans)
    struct PwmInfo {
        std::string name;
        std::string pin;
        std::unique_ptr<MCU_pwm> pwm;
    };
    std::vector<PwmInfo> pwmOutputs;

    // ADC inputs (temperature sensors)
    struct AdcInfo {
        std::string name;
        std::string pin;
        std::unique_ptr<MCU_adc> adc;
    };
    std::vector<AdcInfo> adcInputs;

    // Digital outputs (enable pins, etc.)
    struct DigitalInfo {
        std::string name;
        std::string pin;
        std::unique_ptr<MCU_digital_out> dout;
    };
    std::vector<DigitalInfo> digitalOuts;

    // Printer config
    std::string kinematics;
    double maxVelocity = 300;
    double maxAccel = 3000;
    double squareCornerVelocity = 5.0;
    double pressureAdvance = 0.0;
    double pressureAdvanceSmoothTime = 0.04;

    // TMC5160 driver configurations
    struct TMC5160Config {
        std::string name;              // e.g. "stepper_x"
        std::string csPin;
        std::string spiBus;
        int chainPosition = 0;
        int chainLength = 0;
        double runCurrent = 1.0;
        double holdCurrent = 0.5;
        double senseResistor = 0.075;
        int microsteps = 256;
        bool interpolate = true;
        bool stealthChop = true;
    };
    std::vector<TMC5160Config> tmcConfigs;

    // Input shaper configuration
    struct InputShaperConfig {
        std::string shaperTypeX = "none";
        std::string shaperTypeY = "none";
        double shaperFreqX = 0.0;
        double shaperFreqY = 0.0;
        double dampingRatioX = 0.1;
        double dampingRatioY = 0.1;
    };
    InputShaperConfig inputShaper;

    // Error messages
    std::vector<std::string> warnings;
    std::string lastError;

    bool ok() const { return lastError.empty(); }
};

// KlipperConfig: parses a Klipper .cfg file and creates MCU objects.
class KlipperConfig {
public:
    // Parse a .cfg file into sections
    static bool parseFile(const std::string& path,
                          std::vector<ConfigSection>& sections,
                          std::string& error);

    // Build all MCU objects from parsed sections
    static ConfigResult buildObjects(KlipperMCU& mcu,
                                     const std::vector<ConfigSection>& sections);

    // Convenience: parse + build in one call
    static ConfigResult load(KlipperMCU& mcu, const std::string& path);

private:
    // Strip a pin name: remove '!' prefix (invert), '^' (pullup), '~' (pulldown)
    // Returns cleaned pin name, sets invert/pullup flags
    static std::string cleanPin(const std::string& raw,
                                bool& invert, bool& pullUp);

    // Strip "vref_scaled:" or similar prefixes from sensor pins
    static std::string stripPinPrefix(const std::string& raw);

    // Build a stepper + endstop + rail from a [stepper_*] section
    static ConfigResult::StepperInfo buildStepper(
        KlipperMCU& mcu, const ConfigSection& section,
        std::vector<std::string>& warnings);
};
