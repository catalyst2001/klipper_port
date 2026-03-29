#include <iostream>
#include <string>
#include <iomanip>
#include "klipper_mcu.h"
#include "mcu_objects.h"
#include "bus_objects.h"
#include "stepper.h"
#include "toolhead.h"
#include "gcode.h"

int main() {
    std::cout << "=== Klipper Host C++ Test ===" << std::endl;
    std::cout << "Connecting to Duet 3 6HC on COM3..." << std::endl;

    KlipperMCU mcu;

    // Connect
    if (!mcu.connect("COM3", 250000)) {
        std::cerr << "ERROR: " << mcu.getLastError() << std::endl;
        return 1;
    }
    std::cout << "Connected!" << std::endl;

    // Identify
    std::cout << "\n--- Running identify handshake ---" << std::endl;
    if (!mcu.identify()) {
        std::cerr << "ERROR: " << mcu.getLastError() << std::endl;
        return 1;
    }

    // Print results
    std::cout << "\n=== MCU Info ===" << std::endl;
    std::cout << "Version: " << mcu.getVersion() << std::endl;
    std::cout << "Build: " << mcu.getBuildVersions() << std::endl;

    std::cout << "\n=== Available Commands ===" << std::endl;
    for (auto& [name, fmt] : mcu.getCommands()) {
        std::cout << "  [" << fmt.msgId << "] " << fmt.formatStr << std::endl;
    }

    std::cout << "\n=== Available Responses ===" << std::endl;
    for (auto& [name, fmt] : mcu.getResponses()) {
        std::cout << "  [" << fmt.msgId << "] " << fmt.formatStr << std::endl;
    }

    std::cout << "\n=== Config ===" << std::endl;
    for (auto& [key, val] : mcu.getConfig()) {
        std::cout << "  " << key << " = " << val << std::endl;
    }

    std::cout << "\n=== Enumerations ===" << std::endl;
    for (auto& [enumName, values] : mcu.getEnumerations()) {
        std::cout << "  " << enumName << ": ";
        for (auto& [vname, ev] : values) {
            if (ev.isRange())
                std::cout << vname << "=[" << ev.value << ".." << (ev.value + ev.count - 1) << "] ";
            else
                std::cout << vname << "=" << ev.value << " ";
        }
        std::cout << std::endl;
    }

    // Test: get_clock
    std::cout << "\n--- Testing get_clock ---" << std::endl;
    std::map<std::string, int64_t> clockIntParams;
    std::map<std::string, std::vector<uint8_t>> clockBufParams;
    if (mcu.sendWithResponse("get_clock", "clock", clockIntParams, clockBufParams)) {
        // Clock is uint32, display as unsigned
        uint32_t clock = static_cast<uint32_t>(clockIntParams["clock"]);
        std::cout << "MCU Clock: " << clock << std::endl;
    }
    else {
        std::cerr << "get_clock failed: " << mcu.getLastError() << std::endl;
    }

    // Test: get_uptime
    std::cout << "\n--- Testing get_uptime ---" << std::endl;
    std::map<std::string, int64_t> uptimeInt;
    std::map<std::string, std::vector<uint8_t>> uptimeBuf;
    if (mcu.sendWithResponse("get_uptime", "uptime", uptimeInt, uptimeBuf)) {
        uint32_t high = static_cast<uint32_t>(uptimeInt["high"]);
        uint32_t clk = static_cast<uint32_t>(uptimeInt["clock"]);
        uint64_t totalClocks = (static_cast<uint64_t>(high) << 32) | clk;
        double uptimeSeconds = static_cast<double>(totalClocks) / 300000000.0; // 300MHz
        std::cout << "Uptime: " << uptimeSeconds << " seconds ("
                  << (uptimeSeconds / 3600.0) << " hours)" << std::endl;
    }
    else {
        std::cerr << "get_uptime failed: " << mcu.getLastError() << std::endl;
    }

    // Test: Clock Synchronization
    std::cout << "\n--- Testing Clock Sync ---" << std::endl;
    if (mcu.initClockSync()) {
        auto& cs = mcu.getClockSync();
        auto dbg = cs.getDebugInfo();
        std::cout << "Clock Sync OK:" << std::endl;
        std::cout << "  MCU freq: " << std::fixed << std::setprecision(0) << cs.getMcuFreq() << " Hz" << std::endl;
        std::cout << "  Estimated freq: " << std::setprecision(1) << dbg.freq << " Hz" << std::endl;
        std::cout << "  Min half RTT: " << std::setprecision(6) << dbg.minHalfRtt * 1000.0 << " ms" << std::endl;
        std::cout << "  Last clock: " << dbg.lastClock << std::endl;

        // Test clock sync poll
        if (mcu.clockSyncPoll()) {
            dbg = cs.getDebugInfo();
            std::cout << "  After poll - freq: " << std::setprecision(1) << dbg.freq << " Hz" << std::endl;
        }
    }
    else {
        std::cerr << "Clock sync failed: " << mcu.getLastError() << std::endl;
    }

    // Test: Pin Resolution
    std::cout << "\n--- Testing Pin Resolution ---" << std::endl;
    std::vector<std::string> testPins = {"PA0", "PA15", "PB0", "PC5", "PD0", "PD5", "PE0", "ADC_TEMPERATURE"};
    for (auto& pin : testPins) {
        int num = mcu.resolvePin(pin);
        std::cout << "  " << pin << " = " << num << std::endl;
    }

    // Test: Enum Resolution
    std::cout << "\n--- Testing Enum Resolution ---" << std::endl;
    std::cout << "  spi_bus spi0 = " << mcu.resolveEnum("spi_bus", "spi0") << std::endl;
    std::cout << "  i2c_bus twihs0 = " << mcu.resolveEnum("i2c_bus", "twihs0") << std::endl;

    // Test: Shutdown detection (register callback)
    mcu.setShutdownCallback([](const std::string& reason) {
        std::cerr << "*** SHUTDOWN CALLBACK: " << reason << " ***" << std::endl;
    });

    // Test: MCU constants
    std::cout << "\n--- MCU Constants ---" << std::endl;
    std::cout << "  ADC_MAX = " << mcu.getConstantInt("ADC_MAX") << std::endl;
    std::cout << "  PWM_MAX = " << mcu.getConstantInt("PWM_MAX") << std::endl;
    std::cout << "  CLOCK_FREQ = " << mcu.getConstantInt("CLOCK_FREQ") << std::endl;
    std::cout << "  MCU = " << mcu.getConfigStrings().count("MCU") << std::endl;

    // Test: MCU_digital_out (build config but don't finalize - just verify API)
    std::cout << "\n--- Testing MCU_digital_out API ---" << std::endl;
    {
        MCU_digital_out dout(mcu);
        dout.setupPin("PD0", false);
        dout.setupMaxDuration(0.0);
        dout.setupStartValue(false, false);
        if (dout.buildConfig()) {
            std::cout << "  Digital out OID=" << dout.getOid()
                      << " pin=" << dout.getPinName() << " OK" << std::endl;
        } else {
            std::cout << "  Digital out build failed" << std::endl;
        }
    }

    // Test: MCU_pwm (hardware PWM)
    std::cout << "\n--- Testing MCU_pwm API ---" << std::endl;
    {
        MCU_pwm pwm(mcu);
        pwm.setupPin("PD0", false);
        pwm.setupCycleTime(0.001, true);  // 1kHz hardware PWM
        pwm.setupMaxDuration(0.0);
        pwm.setupStartValue(0.0, 0.0);
        if (pwm.buildConfig()) {
            std::cout << "  PWM OID=" << pwm.getOid()
                      << " pin=" << pwm.getPinName()
                      << " pwm_max=" << pwm.getPwmMax() << " OK" << std::endl;
        } else {
            std::cout << "  PWM build failed" << std::endl;
        }
    }

    // Test: MCU_adc
    std::cout << "\n--- Testing MCU_adc API ---" << std::endl;
    {
        MCU_adc adc(mcu);
        adc.setupPin("ADC_TEMPERATURE");
        adc.setupAdcSample(0.5, 0.001, 8, 0.0, 1.0, 0);
        adc.setupAdcCallback([](double readTime, double value) {
            std::cout << "  ADC callback: time=" << readTime
                      << " value=" << value << std::endl;
        });
        if (adc.buildConfig()) {
            std::cout << "  ADC OID=" << adc.getOid()
                      << " pin=" << adc.getPinName() << " OK" << std::endl;
        } else {
            std::cout << "  ADC build failed" << std::endl;
        }
    }

    // Test: MCU_SPI (build config API test)
    std::cout << "\n--- Testing MCU_SPI API ---" << std::endl;
    {
        MCU_SPI spi(mcu);
        spi.setupPin("PA5", false);
        spi.setupBus("spi0", 0, 4000000);
        if (spi.buildConfig()) {
            std::cout << "  SPI OID=" << spi.getOid() << " OK" << std::endl;
        } else {
            std::cout << "  SPI build failed" << std::endl;
        }
    }

    // Test: MCU_I2C (build config API test)
    std::cout << "\n--- Testing MCU_I2C API ---" << std::endl;
    {
        MCU_I2C i2c(mcu);
        i2c.setupBus("twihs0", 100000, 0x48);
        if (i2c.buildConfig()) {
            std::cout << "  I2C OID=" << i2c.getOid() << " OK" << std::endl;
        } else {
            std::cout << "  I2C build failed" << std::endl;
        }
    }

    // Test: TMC UART CRC and bit-framing
    std::cout << "\n--- Testing TMC UART CRC ---" << std::endl;
    {
        uint8_t testData[] = {0xF5, 0x00, 0x06};
        uint8_t crc = MCU_TMC_uart::calcCrc8(testData, 3);
        std::cout << "  CRC8 of [F5,00,06] = 0x" << std::hex << (int)crc << std::dec << std::endl;
    }

    // Test: Thermocouple temperature conversion
    std::cout << "\n--- Testing Thermocouple Conversions ---" << std::endl;
    {
        // MAX31855: 25.0C = 0x00190000 (100 << 18)
        double t1 = MCU_Thermocouple::convertTemperature(
            MCU_Thermocouple::SensorType::MAX31855, 100 << 18);
        std::cout << "  MAX31855 raw 0x01900000 = " << t1 << " C" << std::endl;

        // MAX6675: 25.0C = (100 << 3)
        double t2 = MCU_Thermocouple::convertTemperature(
            MCU_Thermocouple::SensorType::MAX6675, 100 << 3);
        std::cout << "  MAX6675 raw 0x0320 = " << t2 << " C" << std::endl;
    }

    // ---- Phase 4: Stepper, Toolhead, G-code ----

    // Test: MCU_stepper (build config API)
    std::cout << "\n--- Testing MCU_stepper API ---" << std::endl;
    MCU_stepper stepperX(mcu);
    stepperX.setupPin("PD6", "PD11");
    stepperX.setupStepDist(40.0, 200, 16); // 40mm belt, 200 steps, 16 microsteps
    stepperX.setupInvertDir(false);
    stepperX.buildConfig();
    std::cout << "  Stepper X: OID=" << stepperX.getOid()
              << " step_dist=" << stepperX.getStepDist() << " mm" << std::endl;

    MCU_stepper stepperY(mcu);
    stepperY.setupPin("PD7", "PD12");
    stepperY.setupStepDist(40.0, 200, 16);
    stepperY.setupInvertDir(false);
    stepperY.buildConfig();
    std::cout << "  Stepper Y: OID=" << stepperY.getOid()
              << " step_dist=" << stepperY.getStepDist() << " mm" << std::endl;

    MCU_stepper stepperZ(mcu);
    stepperZ.setupPin("PD8", "PD13");
    stepperZ.setupStepDist(8.0, 200, 16); // 8mm lead screw
    stepperZ.setupInvertDir(false);
    stepperZ.buildConfig();
    std::cout << "  Stepper Z: OID=" << stepperZ.getOid()
              << " step_dist=" << stepperZ.getStepDist() << " mm" << std::endl;

    // Test: MCU_endstop
    std::cout << "\n--- Testing MCU_endstop API ---" << std::endl;
    MCU_endstop endstopX(mcu);
    endstopX.setupPin("PC16", true);
    endstopX.buildConfig();
    std::cout << "  Endstop X: OID=" << endstopX.getOid()
              << " trsync=" << endstopX.getTrsyncOid() << std::endl;

    // Test: PrinterRail
    std::cout << "\n--- Testing PrinterRail ---" << std::endl;
    PrinterRail railX(stepperX, endstopX);
    railX.setPositionLimits(0, 300);
    railX.setHomingSpeed(25.0);
    railX.setPositionEndstop(0);
    std::cout << "  Rail X: range [" << railX.getPosMin() << ", " << railX.getPosMax()
              << "] homing_speed=" << railX.getHomingSpeed() << std::endl;

    // Test: Toolhead + Trapezoid Planner
    std::cout << "\n--- Testing Toolhead ---" << std::endl;
    ToolHead toolhead(mcu);
    toolhead.setMaxVelocity(100);
    toolhead.setMaxAccel(1000);
    toolhead.setSquareCornerVelocity(5.0);
    toolhead.addStepper(0, &stepperX);
    toolhead.addStepper(1, &stepperY);
    toolhead.addStepper(2, &stepperZ);
    std::cout << "  maxVel=" << toolhead.getMaxVelocity()
              << " maxAccel=" << toolhead.getMaxAccel()
              << " junctionDev=" << toolhead.getJunctionDeviation() << std::endl;

    // Test: Move/TrapMove generation (offline, no MCU send)
    std::cout << "\n--- Testing Move + TrapMove ---" << std::endl;
    {
        Move move(Vec3(0, 0, 0), Vec3(10, 0, 0), 50.0, 1000.0);
        move.setJunction(0, 50.0 * 50.0, 0);
        auto trapMoves = move.toTrapMoves();
        std::cout << "  Move 10mm @ 50mm/s: " << trapMoves.size() << " trap moves" << std::endl;
        for (size_t i = 0; i < trapMoves.size(); ++i) {
            auto& tm = trapMoves[i];
            std::cout << "    [" << i << "] t=" << tm.print_time
                      << " dt=" << tm.move_t
                      << " v0=" << tm.start_v
                      << " a/2=" << tm.half_accel << std::endl;
        }
    }

    // Test: G-code parser (offline)
    std::cout << "\n--- Testing GCodeParser ---" << std::endl;
    GCodeParser gcode(toolhead, mcu);
    gcode.addRail(0, &railX);

    gcode.executeLine("G90");
    std::cout << "  G90: absolute=" << gcode.isAbsoluteMode() << std::endl;
    gcode.executeLine("G91");
    std::cout << "  G91: absolute=" << gcode.isAbsoluteMode() << std::endl;
    gcode.executeLine("G90");

    gcode.executeLine("G92 X0 Y0 Z0");
    std::cout << "  G92 X0 Y0 Z0: basePos=("
              << gcode.getBasePosition().x << ","
              << gcode.getBasePosition().y << ","
              << gcode.getBasePosition().z << ")" << std::endl;

    // Parse test without sending to MCU (toolhead queues but doesn't send)
    gcode.executeLine("G1 X10 Y5 F3000");
    gcode.executeLine("G1 X20 Y10");
    std::cout << "  After G1 moves: queue=" << toolhead.getQueueSize()
              << " pos=(" << toolhead.getPosition().x
              << "," << toolhead.getPosition().y
              << "," << toolhead.getPosition().z << ")" << std::endl;

    gcode.executeLine("M114");
    std::cout << "  M114: " << gcode.getLastMessage() << std::endl;

    // Test: Config finalization (sends all config to MCU)
    std::cout << "\n--- Testing Config Finalization ---" << std::endl;
    if (mcu.finalizeConfig()) {
        std::cout << "  Config finalized OK! OIDs=" << mcu.getOidCount() << std::endl;

        // Poll for a few seconds to receive ADC data
        std::cout << "\n--- Polling for ADC data (3 seconds) ---" << std::endl;
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::seconds(3)) {
            mcu.processIncoming(100);
        }
    } else {
        std::cout << "  Config finalize failed: " << mcu.getLastError() << std::endl;
    }

    std::cout << "\n=== Done ===" << std::endl;
    mcu.disconnect();
    return 0;
}