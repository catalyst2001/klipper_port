// ============================================================================
// Klipper Host C++ Test Tool
// Console-based test harness for MCU communication, motion testing, and
// gcode file execution with full timestamped logging.
//
// Usage:
//   klipper_host.exe gcode <file>           - run gcode file
//   klipper_host.exe move "<gcode>"         - execute single gcode block
//   klipper_host.exe monitor [seconds]      - monitor MCU stats (default 30s)
//   klipper_host.exe info                   - connect, identify, show config
// ============================================================================

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <iomanip>
#include <algorithm>
#include <csignal>
#include <map>

#include "klipper_mcu.h"
#include "klipper_config.h"
#include "mcu_objects.h"
#include "bus_objects.h"
#include "stepper.h"
#include "toolhead.h"
#include "gcode.h"
#include "tmc5160.h"
#include "input_shaper.h"

// ---- Global state ----
static std::mutex g_mcuMutex;
static std::atomic<bool> g_running{true};
static std::atomic<bool> g_shutdown{false};
static std::atomic<int> g_clockSyncFailures{0};
static std::mutex g_logMutex;

// ---- Timestamped logging ----
static std::string timestamp() {
    auto now = std::chrono::system_clock::now();
    auto t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    struct tm lt;
    localtime_s(&lt, &t);
    std::ostringstream ss;
    ss << std::setfill('0')
       << std::setw(2) << lt.tm_hour << ":"
       << std::setw(2) << lt.tm_min << ":"
       << std::setw(2) << lt.tm_sec << "."
       << std::setw(3) << ms.count();
    return ss.str();
}

static void Log(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_logMutex);
    std::cout << "[" << timestamp() << "] " << msg << std::endl;
}

static void LogError(const std::string& msg) {
    std::lock_guard<std::mutex> lock(g_logMutex);
    std::cerr << "[" << timestamp() << "] ERROR: " << msg << std::endl;
}

// ---- Signal handler for Ctrl-C ----
static void signalHandler(int) {
    g_running = false;
}

// ---- Background: ProcessIncoming + response logging ----
static void pollThread(KlipperMCU& mcu) {
    while (g_running) {
        {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            if (!mcu.isConnected()) {
                Log("POLL: MCU disconnected (port closed)");
                g_running = false;
                break;
            }
            auto responses = mcu.processIncoming(5);
            for (auto& resp : responses) {
                if (resp.name == "analog_in_state" || resp.name == "thermocouple_state")
                    continue;
                std::ostringstream ss;
                ss << "<< [" << resp.msgId << "] " << resp.name;
                for (auto& [k, v] : resp.intParams)
                    ss << " " << k << "=" << v;
                for (auto& [k, v] : resp.bufParams)
                    ss << " " << k << "=[" << v.size() << " bytes]";
                Log(ss.str());
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// ---- Background: Clock sync polling ----
static void clockSyncThread(KlipperMCU& mcu) {
    while (g_running) {
        {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            if (!mcu.isConnected()) break;
            if (!mcu.clockSyncPoll()) {
                int fails = ++g_clockSyncFailures;
                Log("CLOCKSYNC: poll failed (consecutive=" + std::to_string(fails) + ")");
                if (fails >= 3) {
                    Log("CLOCKSYNC: 3 consecutive failures - MCU likely rebooted!");
                    g_running = false;
                    break;
                }
            } else {
                g_clockSyncFailures = 0;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(984));
    }
}

// ---- Initialization sequence (matches GUI flow) ----
struct TestContext {
    KlipperMCU mcu;
    std::unique_ptr<ConfigResult> config;
    std::unique_ptr<ToolHead> toolhead;
    std::unique_ptr<GCodeParser> gcode;
    std::vector<std::unique_ptr<MCU_SPI>> spiObjects;
    std::vector<std::unique_ptr<TMC5160>> tmcDrivers;
    std::thread pollTh;
    std::thread clockSyncTh;

    bool init(const std::string& port, const std::string& configPath) {
        // 1. Connect
        Log("Connecting to " + port + "...");
        if (!mcu.connect(port, 250000)) {
            LogError("Connect failed: " + mcu.getLastError());
            return false;
        }
        Log("Connected!");

        // 2. Identify
        Log("Running identify handshake...");
        if (!mcu.identify()) {
            LogError("Identify failed: " + mcu.getLastError());
            return false;
        }
        Log("Identified! Version: " + mcu.getVersion());
        Log("Build: " + mcu.getBuildVersions());
        Log("Commands: " + std::to_string(mcu.getCommands().size()) +
            ", Responses: " + std::to_string(mcu.getResponses().size()));
        for (auto& [key, val] : mcu.getConfig()) {
            Log("  Config: " + key + " = " + std::to_string(val));
        }

        // 3. Init clock sync
        Log("Initializing clock synchronization...");
        if (!mcu.initClockSync()) {
            LogError("Clock sync init failed: " + mcu.getLastError());
            return false;
        }
        {
            auto& cs = mcu.getClockSync();
            auto dbg = cs.getDebugInfo();
            std::ostringstream ss;
            ss << "Clock sync initialized: freq=" << std::fixed << std::setprecision(0)
               << dbg.freq << " Hz, RTT=" << std::setprecision(3)
               << dbg.minHalfRtt * 2000.0 << " ms";
            Log(ss.str());
        }

        // 3.5. Clear shutdown if MCU is in shutdown from a previous session
        if (mcu.isShutdown()) {
            Log("MCU is in shutdown state: " + mcu.getShutdownMsg());
            Log("Clearing shutdown...");
            if (mcu.clearShutdown()) {
                Log("Shutdown cleared successfully");
            } else {
                Log("clearShutdown failed (will retry during finalize): " + mcu.getLastError());
            }
        }

        // 4. Load config (no MCU communication — just queues config commands)
        Log("Loading config: " + configPath);
        config = std::make_unique<ConfigResult>(KlipperConfig::load(mcu, configPath));
        if (!config->ok()) {
            LogError("Config load failed: " + config->lastError);
            return false;
        }
        for (auto& w : config->warnings)
            Log("  Warning: " + w);
        for (auto& si : config->steppers) {
            std::ostringstream ss;
            ss << "  [" << si.name << "] stepper OID=" << si.stepper->getOid()
               << " dist=" << std::setprecision(5) << si.stepper->getStepDist() << "mm";
            Log(ss.str());
        }
        for (auto& ai : config->adcInputs)
            Log("  [" + ai.name + "] ADC pin=" + ai.pin);
        for (auto& di : config->digitalOuts)
            Log("  [" + di.name + "] digital out pin=" + di.pin);
        for (auto& tc : config->tmcConfigs) {
            std::ostringstream ss;
            ss << "  [" << tc.name << "] TMC5160 run=" << tc.runCurrent
               << "A hold=" << tc.holdCurrent << "A ms=" << tc.microsteps;
            Log(ss.str());
        }
        {
            std::ostringstream ss;
            ss << "  Printer: " << config->kinematics
               << " vel=" << config->maxVelocity
               << " accel=" << config->maxAccel
               << " scv=" << config->squareCornerVelocity;
            Log(ss.str());
        }

        // 4b. Create TMC5160 SPI objects (must be before finalize to register OIDs)
        std::map<std::string, MCU_SPI*> tmcBusMap;
        if (!config->tmcConfigs.empty()) {
            Log("Creating TMC5160 SPI bus objects...");
            for (auto& tc : config->tmcConfigs) {
                if (tmcBusMap.find(tc.spiBus) == tmcBusMap.end()) {
                    auto spi = std::make_unique<MCU_SPI>(mcu);
                    spi->setupPin(tc.csPin, false);
                    spi->setupBus(tc.spiBus, 3, 4000000);
                    spi->buildConfig();
                    tmcBusMap[tc.spiBus] = spi.get();
                    spiObjects.push_back(std::move(spi));
                }
            }
        }

        // 5. Finalize MCU config (no background threads yet — exclusive serial access)
        Log("Finalizing MCU configuration...");
        if (!mcu.finalizeConfig()) {
            LogError("Finalize failed: " + mcu.getLastError());
            return false;
        }
        Log("Config finalized! OIDs=" + std::to_string(mcu.getOidCount()));

        // 6. Re-init clock sync after finalize
        mcu.initClockSync();
        {
            auto& cs = mcu.getClockSync();
            auto dbg = cs.getDebugInfo();
            std::ostringstream ss;
            ss << "Clock sync re-initialized: freq=" << std::fixed << std::setprecision(0)
               << dbg.freq << " Hz, RTT=" << std::setprecision(3)
               << dbg.minHalfRtt * 2000.0 << " ms";
            Log(ss.str());
        }

        // 7. Start background threads (AFTER finalize — finalize may reset MCU)
        mcu.setShutdownCallback([](const std::string& reason) {
            Log("!!! MCU SHUTDOWN: " + reason + " !!!");
            g_shutdown = true;
            g_running = false;
        });
        pollTh = std::thread(pollThread, std::ref(mcu));
        clockSyncTh = std::thread(clockSyncThread, std::ref(mcu));

        // 8. Create ToolHead
        {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            toolhead = std::make_unique<ToolHead>(mcu);
            double basePrintTime = mcu.getClockSync().estimatedPrintTime() + 0.25;
            toolhead->setNextPrintTime(basePrintTime);
            toolhead->setMaxVelocity(config->maxVelocity);
            toolhead->setMaxAccel(config->maxAccel);
            toolhead->setSquareCornerVelocity(config->squareCornerVelocity);
            for (size_t i = 0; i < config->steppers.size() && i < 3; ++i)
                toolhead->addStepper(static_cast<int>(i), config->steppers[i].stepper.get());
        }
        Log("Toolhead initialized");

        // 9. Create G-code parser
        gcode = std::make_unique<GCodeParser>(*toolhead, mcu);
        for (size_t i = 0; i < config->steppers.size() && i < 3; ++i) {
            if (config->steppers[i].rail)
                gcode->addRail(static_cast<int>(i), config->steppers[i].rail.get());
        }
        Log("G-code parser initialized");

        // 10. Initialize TMC5160 driver registers (SPI objects created in step 4b)
        if (!config->tmcConfigs.empty()) {
            Log("Initializing TMC5160 drivers...");
            for (auto& tc : config->tmcConfigs) {
                auto driver = std::make_unique<TMC5160>(tc.name);
                driver->setSpi(tmcBusMap[tc.spiBus], tc.chainPosition, tc.chainLength);
                driver->setCurrent(tc.runCurrent, tc.holdCurrent, tc.senseResistor);
                driver->setMicrosteps(tc.microsteps, tc.interpolate);
                driver->setStealthChop(tc.stealthChop);
                {
                    std::lock_guard<std::mutex> lock(g_mcuMutex);
                    if (driver->initRegisters()) {
                        auto status = driver->readStatus();
                        Log("  [" + tc.name + "] Init OK - " + TMC5160::formatStatus(status));
                    } else {
                        LogError("  [" + tc.name + "] Init FAILED");
                    }
                }
                tmcDrivers.push_back(std::move(driver));
            }
        }

        Log("=== Initialization complete ===");
        return true;
    }

    void shutdown() {
        g_running = false;
        if (pollTh.joinable()) pollTh.join();
        if (clockSyncTh.joinable()) clockSyncTh.join();
        mcu.disconnect();
        Log("Disconnected.");
    }
};

// ---- Mode: run gcode file ----
static int runGcodeFile(TestContext& ctx, const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        LogError("Cannot open file: " + filePath);
        return 1;
    }
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(file, line))
        lines.push_back(line);
    Log("Loaded gcode: " + filePath + " (" + std::to_string(lines.size()) + " lines)");

    constexpr double BUFFER_TIME_START = 4.0;
    constexpr double BUFFER_TIME_HIGH  = 4.0;
    constexpr double BUFFER_TIME_LOW   = 1.0;
    constexpr size_t FLUSH_BATCH = 10;
    size_t linesSinceFlush = 0;
    size_t errorCount = 0;

    // Init print state
    {
        std::lock_guard<std::mutex> lock(g_mcuMutex);
        ctx.toolhead->resetSyncState();
        double initialTime = ctx.mcu.getClockSync().estimatedPrintTime() + BUFFER_TIME_START;
        ctx.toolhead->setNextPrintTime(initialTime);
        auto& cs = ctx.mcu.getClockSync();
        std::ostringstream ss;
        ss << "Print init: estPrintTime=" << std::fixed << std::setprecision(3)
           << cs.estimatedPrintTime() << " initialTime=" << initialTime
           << " clock=" << cs.getDebugInfo().lastClock
           << " estFreq=" << std::setprecision(0) << cs.getEstimatedFreq()
           << " nomFreq=" << cs.getMcuFreq();
        Log(ss.str());
    }
    Log("Starting print...");

    auto startTime = std::chrono::steady_clock::now();
    size_t contentLines = 0;
    size_t flushCount = 0;

    // Step generation thread: decouples gcode processing from serial I/O.
    // The main thread builds moves (trapq); this thread sends them to MCU.
    // generateSteps self-paces via clock-gating (waits until MCU clock is
    // close enough before sending each step batch).
    std::atomic<bool> stepGenDone{false};
    std::thread stepGenThread([&]() {
        while (ctx.mcu.isConnected() && !ctx.mcu.isShutdown()) {
            auto t0 = std::chrono::steady_clock::now();
            bool ok = ctx.toolhead->generateSteps();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - t0).count();
            if (ms > 500) {
                std::cerr << "[StepGen] " << ms << "ms" << std::endl;
            }
            if (!ok) break; // MCU error
            if (stepGenDone.load(std::memory_order_acquire)) {
                // Final drain: process any remaining TrapMoves
                ctx.toolhead->generateSteps();
                break;
            }
            // Brief sleep when trapq was empty to avoid busy-waiting
            if (ms < 2)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });

    for (size_t i = 0; i < lines.size(); ++i) {
        if (!g_running) break;

        // Health check
        {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            if (!ctx.mcu.isConnected()) {
                LogError("MCU disconnected at line " + std::to_string(i + 1)
                         + " content=" + std::to_string(contentLines));
                break;
            }
            if (ctx.mcu.isShutdown()) {
                std::string shutMsg = ctx.mcu.getShutdownMsg();
                double ahead = ctx.toolhead->getNextPrintTime()
                             - ctx.mcu.getClockSync().estimatedPrintTime();
                std::ostringstream ss;
                ss << "MCU SHUTDOWN at line " << (i + 1) << "/" << lines.size()
                   << " content=" << contentLines
                   << " ahead=" << std::fixed << std::setprecision(3) << ahead << "s"
                   << " reason=\"" << shutMsg << "\"";
                LogError(ss.str());
                break;
            }
        }

        const std::string& cmd = lines[i];
        // Skip empty/comment lines
        {
            std::string trimmed = cmd;
            while (!trimmed.empty() && (trimmed[0] == ' ' || trimmed[0] == '\t'))
                trimmed.erase(trimmed.begin());
            if (trimmed.empty() || trimmed[0] == ';' || trimmed[0] == '%' || trimmed[0] == '(')
                continue;
        }

        // Execute
        bool execOk;
        {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            double minTime = ctx.mcu.getClockSync().estimatedPrintTime() + 0.25;
            if (minTime > ctx.toolhead->getNextPrintTime())
                ctx.toolhead->setNextPrintTime(minTime);
            execOk = ctx.gcode->executeLine(cmd);
        }
        if (!execOk) {
            std::string msg = ctx.gcode->getLastMessage();
            if (!msg.empty() && msg.find("Unknown") == std::string::npos) {
                LogError("Line " + std::to_string(i + 1) + ": " + msg + " [" + cmd + "]");
                errorCount++;
            }
        }

        contentLines++;
        linesSinceFlush++;

        // Periodic line logging: first 5, then every 100 content lines
        if (contentLines <= 5 || contentLines % 100 == 0) {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            auto p = ctx.toolhead->getPosition();
            auto elapsed = std::chrono::steady_clock::now() - startTime;
            double sec = std::chrono::duration<double>(elapsed).count();
            double ahead = ctx.toolhead->getNextPrintTime()
                         - ctx.mcu.getClockSync().estimatedPrintTime();
            std::ostringstream ss;
            ss << "L" << (i + 1) << "/" << lines.size()
               << " #" << contentLines
               << " pos=(" << std::fixed << std::setprecision(2)
               << p.x << "," << p.y << "," << p.z << ")"
               << " ahead=" << std::setprecision(3) << ahead << "s"
               << " t=" << std::setprecision(1) << sec << "s"
               << " [" << cmd << "]";
            if (ctx.mcu.isShutdown())
                ss << " !! MCU SHUTDOWN !!";
            Log(ss.str());
        }

        // Flush periodically to push moves into TrapQ for the step gen thread
        if (linesSinceFlush >= FLUSH_BATCH || i == lines.size() - 1) {
            linesSinceFlush = 0;
            flushCount++;

            {
                std::lock_guard<std::mutex> lock(g_mcuMutex);
                ctx.toolhead->flush();
            }

            // Log some flushes for diagnostics
            auto elapsed = std::chrono::steady_clock::now() - startTime;
            double sec = std::chrono::duration<double>(elapsed).count();
            double ahead;
            double printTime;
            {
                std::lock_guard<std::mutex> lock(g_mcuMutex);
                printTime = ctx.toolhead->getNextPrintTime();
                ahead = printTime - ctx.mcu.getClockSync().estimatedPrintTime();
            }
            if (flushCount <= 5 || flushCount % 10 == 0 || i == lines.size() - 1) {
                std::ostringstream ss;
                ss << "Flush #" << flushCount << " at line " << (i + 1)
                   << "/" << lines.size()
                   << " elapsed=" << std::fixed << std::setprecision(1) << sec << "s"
                   << " ahead=" << std::setprecision(3) << ahead << "s"
                   << " printTime=" << std::setprecision(3) << printTime;
                Log(ss.str());
            }

            // Backpressure: wait if host is too far ahead of MCU.
            // The step gen thread drains the trapq in parallel, so the
            // MCU clock advances while we wait here.
            for (;;) {
                if (!g_running) break;
                {
                    std::lock_guard<std::mutex> lock(g_mcuMutex);
                    if (!ctx.mcu.isConnected() || ctx.mcu.isShutdown()) break;
                    ahead = ctx.toolhead->getNextPrintTime()
                          - ctx.mcu.getClockSync().estimatedPrintTime();
                }
                if (ahead < BUFFER_TIME_HIGH) break;
                double waitSec = ahead - BUFFER_TIME_LOW;
                int waitMs = (std::max)(10, (std::min)(500, static_cast<int>(waitSec * 1000)));
                std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
            }
        }
    }

    // Final flush + stop step generation thread
    Log("Flushing remaining moves...");
    {
        std::lock_guard<std::mutex> lock(g_mcuMutex);
        if (ctx.mcu.isConnected() && !ctx.mcu.isShutdown())
            ctx.toolhead->flush();
    }
    stepGenDone.store(true, std::memory_order_release);
    stepGenThread.join();
    {
        std::lock_guard<std::mutex> lock(g_mcuMutex);
        ctx.toolhead->resetSyncState();
    }

    auto elapsed = std::chrono::steady_clock::now() - startTime;
    double totalSec = std::chrono::duration<double>(elapsed).count();
    {
        auto p = ctx.toolhead->getPosition();
        std::ostringstream ps;
        ps << "Final position: (" << std::fixed << std::setprecision(3)
           << p.x << "," << p.y << "," << p.z << ")"
           << " contentLines=" << contentLines << " flushes=" << flushCount;
        Log(ps.str());
    }
    std::ostringstream ss;
    ss << "Print finished: " << lines.size() << " lines, "
       << std::fixed << std::setprecision(1) << totalSec << "s, "
       << errorCount << " errors";
    Log(ss.str());

    // Wait for MCU to finish executing remaining queued steps
    Log("Waiting for MCU to finish executing steps...");
    for (int w = 0; w < 300 && g_running; ++w) {
        double ahead;
        {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            if (!ctx.mcu.isConnected() || ctx.mcu.isShutdown()) break;
            ahead = ctx.toolhead->getNextPrintTime()
                  - ctx.mcu.getClockSync().estimatedPrintTime();
        }
        if (ahead <= 0.2) break;
        if (w % 20 == 0) {
            std::ostringstream ss;
            ss << "  Waiting: ahead=" << std::fixed << std::setprecision(2) << ahead << "s";
            Log(ss.str());
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    Log("Done.");

    return (errorCount > 0 || g_shutdown) ? 1 : 0;
}

// ---- Mode: execute gcode block ----
static int runGcodeBlock(TestContext& ctx, const std::string& gcodeBlock) {
    Log("Executing: " + gcodeBlock);

    {
        std::lock_guard<std::mutex> lock(g_mcuMutex);
        ctx.toolhead->resetSyncState();
        double now = ctx.mcu.getClockSync().estimatedPrintTime() + 0.25;
        ctx.toolhead->setNextPrintTime(now);
    }

    int count;
    {
        std::lock_guard<std::mutex> lock(g_mcuMutex);
        count = ctx.gcode->executeBlock(gcodeBlock);
    }
    if (count <= 0) {
        LogError("Execute failed: " + ctx.gcode->getLastMessage());
        return 1;
    }

    {
        std::lock_guard<std::mutex> lock(g_mcuMutex);
        ctx.toolhead->flush();
    }
    ctx.toolhead->generateSteps();

    Log("Waiting for motion to complete...");
    for (int w = 0; w < 100 && g_running; ++w) {
        double ahead;
        {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            if (!ctx.mcu.isConnected() || ctx.mcu.isShutdown()) break;
            ahead = ctx.toolhead->getNextPrintTime()
                  - ctx.mcu.getClockSync().estimatedPrintTime();
        }
        if (ahead <= 0.2) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    {
        std::lock_guard<std::mutex> lock(g_mcuMutex);
        ctx.toolhead->resetSyncState();
    }

    Vec3 pos = ctx.toolhead->getPosition();
    std::ostringstream posStr;
    posStr << "Position: X=" << std::fixed << std::setprecision(3) << pos.x
           << " Y=" << pos.y << " Z=" << pos.z;
    Log(posStr.str());
    Log("Done.");
    return 0;
}

// ---- Mode: monitor MCU stats ----
static int runMonitor(TestContext& ctx, int seconds) {
    Log("Monitoring MCU for " + std::to_string(seconds) + " seconds...");
    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        auto elapsed = std::chrono::steady_clock::now() - start;
        if (std::chrono::duration<double>(elapsed).count() >= seconds) break;
        {
            std::lock_guard<std::mutex> lock(g_mcuMutex);
            if (!ctx.mcu.isConnected()) {
                LogError("MCU disconnected!");
                return 1;
            }
            if (ctx.mcu.isShutdown()) {
                LogError("MCU shutdown!");
                return 1;
            }
            auto& cs = ctx.mcu.getClockSync();
            auto dbg = cs.getDebugInfo();
            std::ostringstream ss;
            ss << "ClockSync: freq=" << std::fixed << std::setprecision(0) << dbg.freq
               << " rtt=" << std::setprecision(3) << dbg.minHalfRtt * 2000.0 << "ms"
               << " clock=" << dbg.lastClock
               << " est_print_time=" << std::setprecision(3) << cs.estimatedPrintTime();
            Log(ss.str());
        }
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    Log("Monitor done.");
    return 0;
}

// ---- Mode: info only ----
static int runInfo(TestContext& ctx) {
    Log("MCU info displayed above. Monitoring for 5 seconds...");
    std::this_thread::sleep_for(std::chrono::seconds(5));
    Log("Done.");
    return 0;
}

// ---- Usage ----
static void printUsage(const char* exe) {
    std::cerr << "Usage:\n"
              << "  " << exe << " gcode <file.gcode>    - run gcode file\n"
              << "  " << exe << " move \"<gcode>\"        - execute gcode block\n"
              << "  " << exe << " monitor [seconds]      - monitor MCU stats\n"
              << "  " << exe << " info                   - show MCU info\n\n"
              << "Options:\n"
              << "  --port <COMx>     - serial port (default COM3)\n"
              << "  --config <file>   - config file (default configs/generic-duet3-6hc.cfg)\n";
}

// ---- Main ----
int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Parse args
    std::string mode;
    std::string modeArg;
    std::string port = "COM3";
    std::string configPath = "configs/generic-duet3-6hc.cfg";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (mode.empty()) {
            mode = arg;
        } else if (modeArg.empty()) {
            modeArg = arg;
        }
    }

    if (mode.empty()) {
        printUsage(argv[0]);
        return 1;
    }

    Log("=== Klipper Host C++ Test Tool ===");
    Log("Mode: " + mode + (modeArg.empty() ? "" : " " + modeArg));

    // Initialize
    TestContext ctx;
    if (!ctx.init(port, configPath)) {
        LogError("Initialization failed!");
        ctx.shutdown();
        return 1;
    }

    // Run selected mode
    int result = 0;
    if (mode == "gcode") {
        if (modeArg.empty()) {
            LogError("gcode mode requires a file path");
            result = 1;
        } else {
            result = runGcodeFile(ctx, modeArg);
        }
    } else if (mode == "move") {
        if (modeArg.empty()) {
            LogError("move mode requires a gcode string");
            result = 1;
        } else {
            result = runGcodeBlock(ctx, modeArg);
        }
    } else if (mode == "monitor") {
        int seconds = modeArg.empty() ? 30 : std::stoi(modeArg);
        result = runMonitor(ctx, seconds);
    } else if (mode == "info") {
        result = runInfo(ctx);
    } else {
        LogError("Unknown mode: " + mode);
        printUsage(argv[0]);
        result = 1;
    }

    // Shutdown
    ctx.shutdown();

    if (g_shutdown) {
        LogError("Session ended due to MCU shutdown!");
        return 2;
    }

    return result;
}

