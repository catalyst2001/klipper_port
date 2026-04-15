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
#include <filesystem>
#include <cstdlib>

#include "klipper_mcu.h"
#include "klipper_config.h"
#include "mcu_objects.h"
#include "bus_objects.h"
#include "stepper.h"
#include "toolhead.h"
#include "gcode.h"
#include "tmc5160.h"
#include "input_shaper.h"
#include "reactor.h"
#include "moonraker_api.h"

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
// (Not used with reactor — serial queue handles I/O.  Kept for non-reactor
// modes like monitor/info where serial queue is active.)
static void pollThread(KlipperMCU& mcu) {
    while (g_running) {
        if (mcu.isSerialQueueActive()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
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
        if (mcu.isSerialQueueActive()) {
            mcu.clockSyncPollAsync();
        } else {
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
    Reactor reactor;
    std::unique_ptr<ConfigResult> config;
    std::unique_ptr<ToolHead> toolhead;
    std::unique_ptr<GCodeParser> gcode;
    std::vector<std::unique_ptr<MCU_SPI>> spiObjects;
    std::vector<std::unique_ptr<TMC5160>> tmcDrivers;
    // Legacy threads for non-reactor modes (monitor, info)
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

        // 6. Re-init clock sync after finalizeConfig (MCU may have been reset,
        //    which resets its clock to 0 — the step-3 sync data would be stale)
        if (!mcu.initClockSync()) {
            LogError("Clock sync re-init failed: " + mcu.getLastError());
            return false;
        }
        {
            auto& cs = mcu.getClockSync();
            auto dbg = cs.getDebugInfo();
            std::ostringstream ss;
            ss << "Clock sync re-initialized: freq=" << std::fixed << std::setprecision(0)
               << dbg.freq << " Hz, RTT=" << std::setprecision(3)
               << dbg.minHalfRtt * 2000.0 << " ms";
            Log(ss.str());
        }

        // 7. Set shutdown callback
        mcu.setShutdownCallback([](const std::string& reason) {
            Log("!!! MCU SHUTDOWN: " + reason + " !!!");
            g_shutdown = true;
            g_running = false;
        });
        // NOTE: Legacy poll/clockSync threads are NOT started here.
        // For reactor-based gcode mode, the reactor handles clock sync.
        // For monitor/info modes, call startLegacyThreads() explicitly.

        // 8. Create ToolHead with reactor support
        {
            toolhead = std::make_unique<ToolHead>(mcu);
            toolhead->setReactor(&reactor);
            double basePrintTime = mcu.getClockSync().estimatedPrintTime() + 0.25;
            toolhead->setNextPrintTime(basePrintTime);
            toolhead->setMaxVelocity(config->maxVelocity);
            toolhead->setMaxAccel(config->maxAccel);
            toolhead->setSquareCornerVelocity(config->squareCornerVelocity);
            toolhead->setPressureAdvance(config->pressureAdvance,
                                         config->pressureAdvanceSmoothTime);

            auto findStepperByName = [&](const std::string& exact,
                                         const std::string& prefix = "") -> MCU_stepper* {
                for (auto& s : config->steppers) {
                    if (s.name == exact)
                        return s.stepper.get();
                    if (!prefix.empty() && s.name.rfind(prefix, 0) == 0)
                        return s.stepper.get();
                }
                return nullptr;
            };

            if (auto* sx = findStepperByName("stepper_x")) toolhead->addStepper(0, sx);
            if (auto* sy = findStepperByName("stepper_y")) toolhead->addStepper(1, sy);
            if (auto* sz = findStepperByName("stepper_z")) toolhead->addStepper(2, sz);
            if (auto* se = findStepperByName("extruder", "extruder")) toolhead->addStepper(3, se);
        }
        Log("Toolhead initialized");

        // 9. Create G-code parser
        gcode = std::make_unique<GCodeParser>(*toolhead, mcu);
        for (auto& s : config->steppers) {
            if (!s.rail) continue;
            if (s.name == "stepper_x") gcode->addRail(0, s.rail.get());
            else if (s.name == "stepper_y") gcode->addRail(1, s.rail.get());
            else if (s.name == "stepper_z") gcode->addRail(2, s.rail.get());
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
                if (driver->initRegisters()) {
                    auto status = driver->readStatus();
                    Log("  [" + tc.name + "] Init OK - " + TMC5160::formatStatus(status));
                } else {
                    LogError("  [" + tc.name + "] Init FAILED");
                }
                tmcDrivers.push_back(std::move(driver));
            }
        }

        // 11. Start SerialQueue (after TMC init — SerialQueue takes over serial I/O)
        Log("Starting SerialQueue...");
        if (!mcu.startSerialQueue()) {
            LogError("Failed to start SerialQueue: " + mcu.getLastError());
            return false;
        }
        Log("SerialQueue started");

        Log("=== Initialization complete ===");
        return true;
    }

    void startLegacyThreads() {
        pollTh = std::thread(pollThread, std::ref(mcu));
        clockSyncTh = std::thread(clockSyncThread, std::ref(mcu));
    }

    void shutdown() {
        g_running = false;
        reactor.end();
        if (pollTh.joinable()) pollTh.join();
        if (clockSyncTh.joinable()) clockSyncTh.join();
        mcu.stopSerialQueue();
        mcu.disconnect();
        Log("Disconnected.");
    }
};

struct GcodeRunHooks {
    std::function<bool()> shouldPause;
    std::function<bool()> shouldCancel;
    std::function<void(size_t filePosition, size_t fileSize,
                       size_t contentLines, size_t totalLines)> onProgress;
};

// ---- Mode: run gcode file (reactor-based) ----
static int runGcodeFile(TestContext& ctx, const std::string& filePath,
                        size_t startLine = 0, double speedFactor = 1.0,
                        const GcodeRunHooks* hooks = nullptr) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        LogError("Cannot open file: " + filePath);
        return 1;
    }
    std::vector<std::string> lines;
    std::vector<size_t> lineOffsets;
    std::string line;
    size_t fileSize = 0;
    try {
        if (std::filesystem::exists(filePath))
            fileSize = static_cast<size_t>(std::filesystem::file_size(filePath));
    } catch (...) {
    }
    while (std::getline(file, line)) {
        lines.push_back(line);
        std::streampos pos = file.tellg();
        lineOffsets.push_back(pos >= 0 ? static_cast<size_t>(pos) : fileSize);
    }
    Log("Loaded gcode: " + filePath + " (" + std::to_string(lines.size()) + " lines)");
    if (speedFactor != 1.0) {
        Log("Speed factor: " + std::to_string(speedFactor) + "x");
        ctx.gcode->setSpeedFactor(speedFactor);
    }

    // --start-line: skip to the specified line, but execute setup commands
    if (startLine > 0 && startLine < lines.size()) {
        Log("Skipping to line " + std::to_string(startLine) + " (executing setup commands)...");
        std::vector<std::string> preamble;
        for (size_t i = 0; i < startLine; ++i) {
            std::string t = lines[i];
            while (!t.empty() && (t[0] == ' ' || t[0] == '\t')) t.erase(t.begin());
            if (t.empty() || t[0] == ';' || t[0] == '%' || t[0] == '(') continue;
            if (t[0] == 'M' || t[0] == 'm' ||
                t.substr(0, 3) == "G28" || t.substr(0, 3) == "G90" ||
                t.substr(0, 3) == "G91" || t.substr(0, 3) == "G92") {
                preamble.push_back(lines[i]);
            }
        }
        Log("Preamble: " + std::to_string(preamble.size()) + " setup commands");
        std::vector<std::string> newLines;
        std::vector<size_t> newOffsets;
        newLines.insert(newLines.end(), preamble.begin(), preamble.end());
        newLines.insert(newLines.end(), lines.begin() + startLine, lines.end());
        for (size_t i = 0; i < startLine; ++i) {
            std::string t = lines[i];
            while (!t.empty() && (t[0] == ' ' || t[0] == '\t')) t.erase(t.begin());
            if (t.empty() || t[0] == ';' || t[0] == '%' || t[0] == '(') continue;
            if (t[0] == 'M' || t[0] == 'm' ||
                t.substr(0, 3) == "G28" || t.substr(0, 3) == "G90" ||
                t.substr(0, 3) == "G91" || t.substr(0, 3) == "G92") {
                newOffsets.push_back(i < lineOffsets.size() ? lineOffsets[i] : 0);
            }
        }
        newOffsets.insert(newOffsets.end(), lineOffsets.begin() + startLine, lineOffsets.end());
        lines = std::move(newLines);
        lineOffsets = std::move(newOffsets);
        Log("Effective lines: " + std::to_string(lines.size()));
    }

    constexpr size_t FLUSH_BATCH = 10;
    size_t errorCount = 0;

    // Init print state (uses BUFFER_TIME_START from trapq.h = 0.250)
    ctx.toolhead->resetSyncState();
    double initialTime = ctx.mcu.getClockSync().estimatedPrintTime() + BUFFER_TIME_START;
    ctx.toolhead->setNextPrintTime(initialTime);
    {
        auto& cs = ctx.mcu.getClockSync();
        std::ostringstream ss;
        ss << "Print init: estPrintTime=" << std::fixed << std::setprecision(3)
           << cs.estimatedPrintTime() << " initialTime=" << initialTime
           << " clock=" << cs.getDebugInfo().lastClock
           << " estFreq=" << std::setprecision(0) << cs.getEstimatedFreq()
           << " nomFreq=" << cs.getMcuFreq();
        Log(ss.str());
    }
    Log("Starting print (reactor-based)...");

    auto startTime = std::chrono::steady_clock::now();
    size_t contentLines = 0;
    size_t flushCount = 0;
    size_t gcodeIdx = 0;
    size_t linesSinceFlush = 0;

    // ---- Reactor timer: clock sync (every ~984ms) ----
    auto clockSyncTimer = ctx.reactor.registerTimer([&](double eventtime) -> double {
        if (!g_running) return REACTOR_NEVER;
        if (ctx.mcu.isSerialQueueActive()) {
            ctx.mcu.clockSyncPollAsync();
        }
        return eventtime + 0.984;
    }, ctx.reactor.monotonic() + 0.984);

    // ---- Reactor timer: step generation / flush handler ----
    ReactorTimerPtr stepGenTimer;
    stepGenTimer = ctx.reactor.registerTimer([&](double eventtime) -> double {
        if (!g_running) return REACTOR_NEVER;
        if (!ctx.mcu.isConnected() || ctx.mcu.isShutdown())
            return eventtime + 0.050;

        // Port of Python Klipper's motion_queuing._flush_handler():
        // while actively stepping, wake earlier and flush a tighter window;
        // when idle / draining remnants, fall back to the relaxed horizon.
        double estPrintTime = ctx.mcu.getClockSync().estimatedPrintTime();
        double needStepGenTime = ctx.toolhead->getNeedStepGenTime();
        double needFlushTime = ctx.toolhead->getNeedFlushTime();
        double lastStepGenTime = ctx.toolhead->getStepGenPrintTime();
        double lastFlushTime = ctx.toolhead->getLastFlushTime();
        double aggrSgTime = needStepGenTime - 2.0 * SDS_CHECK_TIME;

        if (lastStepGenTime < aggrSgTime) {
            // Actively stepping - aggressive batching (Python _flush_handler).
            double wantSgTime = estPrintTime + BGFLUSH_SG_HIGH_TIME;
            double batchTime = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME;
            double nextBatchTime = lastStepGenTime + batchTime;
            if (nextBatchTime > estPrintTime) {
                if (nextBatchTime > wantSgTime + 0.005)
                    nextBatchTime = lastStepGenTime;
                wantSgTime = nextBatchTime;
            }
            wantSgTime = (std::min)(wantSgTime, aggrSgTime);
            if (wantSgTime > lastStepGenTime)
                ctx.toolhead->advanceFlushTime(0.0, wantSgTime);

            // Re-schedule timer
            lastStepGenTime = ctx.toolhead->getStepGenPrintTime();
            double waketime = lastStepGenTime - BGFLUSH_SG_LOW_TIME;
            double delay = waketime - estPrintTime;
            return eventtime + (std::max)(0.001, delay);
        }

        // Not stepping (or only remnants) - relaxed flush horizon.
        double maxFlushTime = needFlushTime + BGFLUSH_EXTRA_TIME;
        double wantFlushTime = (std::min)(estPrintTime + BGFLUSH_HIGH_TIME, maxFlushTime);
        if (wantFlushTime > lastFlushTime)
            ctx.toolhead->advanceFlushTime(wantFlushTime, 0.0);

        lastFlushTime = ctx.toolhead->getLastFlushTime();
        if (lastFlushTime >= maxFlushTime) {
            // Python: do_kick_flush_timer = True before sleeping forever.
            ctx.toolhead->armFlushKickTimer();
            return REACTOR_NEVER;
        }

        double waketime = lastFlushTime - BGFLUSH_LOW_TIME;
        double delay = waketime - estPrintTime;
        return eventtime + (std::max)(0.001, delay);
    }, REACTOR_NOW);

    // ---- Reactor timer: gcode processing ----
    auto gcodeTimer = ctx.reactor.registerTimer([&](double eventtime) -> double {
        // Health check (before g_running so shutdown diagnostics are logged)
        if (ctx.mcu.isShutdown()) {
            std::string shutMsg = ctx.mcu.getShutdownMsg();
            double ahead = ctx.toolhead->getNextPrintTime()
                         - ctx.mcu.getClockSync().estimatedPrintTime();
            std::ostringstream ss;
            ss << "MCU SHUTDOWN at line " << (gcodeIdx + 1) << "/" << lines.size()
               << " content=" << contentLines
               << " ahead=" << std::fixed << std::setprecision(3) << ahead << "s"
               << " reason=\"" << shutMsg << "\""
               << " stepSync(total=" << ctx.mcu.getStepSyncTotal()
               << " gated=" << ctx.mcu.getStepSyncGated() << ")";
            LogError(ss.str());
            ctx.reactor.end();
            return REACTOR_NEVER;
        }
        if (hooks && hooks->shouldCancel && hooks->shouldCancel()) {
            if (ctx.mcu.isConnected() && !ctx.mcu.isShutdown()) {
                ctx.toolhead->flush();
                ctx.toolhead->generateSteps(true);
            }
            if (hooks->onProgress) {
                size_t filePosition = gcodeIdx > 0 && !lineOffsets.empty()
                    ? lineOffsets[(std::min)(gcodeIdx, lineOffsets.size()) - 1]
                    : 0;
                hooks->onProgress(filePosition, fileSize, contentLines, lines.size());
            }
            ctx.reactor.end();
            return REACTOR_NEVER;
        }
        if (hooks && hooks->shouldPause && hooks->shouldPause()) {
            if (hooks->onProgress) {
                size_t filePosition = gcodeIdx > 0 && !lineOffsets.empty()
                    ? lineOffsets[(std::min)(gcodeIdx, lineOffsets.size()) - 1]
                    : 0;
                hooks->onProgress(filePosition, fileSize, contentLines, lines.size());
            }
            return eventtime + 0.100;
        }
        if (!g_running || gcodeIdx >= lines.size()) {
            // Final flush + step gen
            if (ctx.mcu.isConnected() && !ctx.mcu.isShutdown()) {
                ctx.toolhead->flush();
                ctx.toolhead->generateSteps(true);
            }
            if (hooks && hooks->onProgress)
                hooks->onProgress(fileSize, fileSize, contentLines, lines.size());
            ctx.reactor.end();
            return REACTOR_NEVER;
        }

        if (!ctx.mcu.isConnected()) {
            LogError("MCU disconnected at line " + std::to_string(gcodeIdx + 1)
                     + " content=" + std::to_string(contentLines));
            ctx.reactor.end();
            return REACTOR_NEVER;
        }

        // Process a batch of gcode lines
        for (size_t batch = 0; batch < FLUSH_BATCH && gcodeIdx < lines.size() && g_running;
             ++gcodeIdx) {
            const std::string& cmd = lines[gcodeIdx];
            // Skip empty/comment lines
            {
                std::string trimmed = cmd;
                while (!trimmed.empty() && (trimmed[0] == ' ' || trimmed[0] == '\t'))
                    trimmed.erase(trimmed.begin());
                if (trimmed.empty() || trimmed[0] == ';' || trimmed[0] == '%' || trimmed[0] == '(')
                    continue;
            }

            // Execute gcode line (moveAbsolute → checkPause → reactor.pause
            // handles backpressure automatically)
            bool execOk = ctx.gcode->executeLine(cmd);
            if (!execOk) {
                std::string msg = ctx.gcode->getLastMessage();
                if (!msg.empty() && msg.find("Unknown") == std::string::npos) {
                    LogError("Line " + std::to_string(gcodeIdx + 1) + ": " + msg + " [" + cmd + "]");
                    errorCount++;
                }
            }

            contentLines++;
            batch++;

            // Periodic logging: first 5, then every 100 content lines
            if (contentLines <= 5 || contentLines % 100 == 0) {
                auto p = ctx.toolhead->getPosition();
                auto elapsed = std::chrono::steady_clock::now() - startTime;
                double sec = std::chrono::duration<double>(elapsed).count();
                double ahead = ctx.toolhead->getNextPrintTime()
                             - ctx.mcu.getClockSync().estimatedPrintTime();
                std::ostringstream ss;
                ss << "L" << (gcodeIdx + 1) << "/" << lines.size()
                   << " #" << contentLines
                   << " pos=(" << std::fixed << std::setprecision(2)
                   << p.x << "," << p.y << "," << p.z << ")"
                   << " ahead=" << std::setprecision(3) << ahead << "s"
                   << " t=" << std::setprecision(1) << sec << "s"
                   << " [" << cmd << "]";
                Log(ss.str());
            }
        }

        // Flush after each batch
        ctx.toolhead->flush();
        if (ctx.toolhead->consumeFlushKickRequest())
            ctx.reactor.updateTimer(stepGenTimer, REACTOR_NOW);
        flushCount++;

        // Backpressure: pause gcode coroutine if too far ahead of MCU.
        // This yields back to the reactor, allowing stepGen and clockSync
        // timers to run, matching Python Klipper's ToolHead._check_pause().
        ctx.toolhead->checkPause();

        // Log flush diagnostics
        {
            auto elapsed = std::chrono::steady_clock::now() - startTime;
            double sec = std::chrono::duration<double>(elapsed).count();
            double printTime = ctx.toolhead->getNextPrintTime();
            double ahead = printTime - ctx.mcu.getClockSync().estimatedPrintTime();
            if (flushCount <= 5 || flushCount % 10 == 0 || gcodeIdx >= lines.size()) {
                std::ostringstream ss;
                ss << "Flush #" << flushCount << " at line " << gcodeIdx
                   << "/" << lines.size()
                   << " elapsed=" << std::fixed << std::setprecision(1) << sec << "s"
                   << " ahead=" << std::setprecision(3) << ahead << "s"
                   << " printTime=" << std::setprecision(3) << printTime
                   << " steps=" << ctx.mcu.getStepSyncTotal()
                   << " gated=" << ctx.mcu.getStepSyncGated();
                Log(ss.str());
            }
        }
        if (hooks && hooks->onProgress) {
            size_t filePosition = gcodeIdx > 0 && !lineOffsets.empty()
                ? lineOffsets[(std::min)(gcodeIdx, lineOffsets.size()) - 1]
                : 0;
            hooks->onProgress(filePosition, fileSize, contentLines, lines.size());
        }

        return REACTOR_NOW;  // process next batch immediately (checkPause limits rate)
    }, REACTOR_NOW);

    // ---- Run reactor event loop (blocks until end() is called) ----
    ctx.reactor.run();

    // Cleanup
    ctx.reactor.unregisterTimer(clockSyncTimer);
    ctx.reactor.unregisterTimer(stepGenTimer);
    ctx.reactor.unregisterTimer(gcodeTimer);

    ctx.toolhead->resetSyncState();

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
    {
        std::ostringstream ss;
        ss << "Print finished: " << lines.size() << " lines, "
           << std::fixed << std::setprecision(1) << totalSec << "s, "
           << errorCount << " errors";
        Log(ss.str());
    }

    // Wait for MCU to finish executing remaining queued steps
    Log("Waiting for MCU to finish executing steps...");
    for (int w = 0; w < 300 && g_running; ++w) {
        if (!ctx.mcu.isConnected() || ctx.mcu.isShutdown()) break;
        double ahead = ctx.toolhead->getNextPrintTime()
                     - ctx.mcu.getClockSync().estimatedPrintTime();
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

    ctx.toolhead->resetSyncState();
    double now = ctx.mcu.getClockSync().estimatedPrintTime() + 0.25;
    ctx.toolhead->setNextPrintTime(now);

    int count = ctx.gcode->executeBlock(gcodeBlock);
    if (count <= 0) {
        LogError("Execute failed: " + ctx.gcode->getLastMessage());
        return 1;
    }

    ctx.toolhead->flush();
    ctx.toolhead->generateSteps();

    Log("Waiting for motion to complete...");
    for (int w = 0; w < 100 && g_running; ++w) {
        if (!ctx.mcu.isConnected() || ctx.mcu.isShutdown()) break;
        double ahead = ctx.toolhead->getNextPrintTime()
                     - ctx.mcu.getClockSync().estimatedPrintTime();
        if (ahead <= 0.2) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    ctx.toolhead->resetSyncState();

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

static int runApiServer(TestContext& ctx, int httpPort) {
    ctx.startLegacyThreads();
    using json = nlohmann::json;

    auto defaultObjects = []() {
        return std::vector<std::string>{
            "webhooks", "toolhead", "gcode_move", "motion_report",
            "print_stats", "extruder", "heater_bed", "virtual_sdcard", "configfile"
        };
    };

    auto parseRequestedObjects = [&](const std::string& query) {
        std::vector<std::string> objects;
        std::istringstream ss(query);
        std::string token;
        while (std::getline(ss, token, '&')) {
            if (token.empty())
                continue;
            auto eq = token.find('=');
            std::string name = (eq == std::string::npos) ? token : token.substr(0, eq);
            if (!name.empty())
                objects.push_back(name);
        }
        if (objects.empty())
            objects = defaultObjects();
        return objects;
    };

    namespace fs = std::filesystem;

    struct PrintJobState {
        std::mutex mutex;
        std::thread worker;
        std::atomic<bool> active{false};
        std::atomic<bool> pauseRequested{false};
        std::atomic<bool> cancelRequested{false};
        std::string state = "standby";
        std::string jobId;
        std::string filename;
        std::string message;
        json metadata = json::object();
        size_t filePosition = 0;
        size_t fileSize = 0;
        double progress = 0.0;
        double startTime = 0.0;
        double endTime = 0.0;
        double filamentUsed = 0.0;
        double printDuration = 0.0;
        double totalDuration = 0.0;
        std::chrono::steady_clock::time_point startedAt{};
    } printJob;

    struct HistoryState {
        std::mutex mutex;
        uint64_t nextJobId = 1;
        std::vector<json> jobs;
        json totals = {
            {"total_jobs", 0},
            {"total_time", 0.0},
            {"total_print_time", 0.0},
            {"total_filament_used", 0.0},
            {"longest_job", 0.0},
            {"longest_print", 0.0}
        };
    } history;

    auto joinFinishedWorker = [&]() {
        if (printJob.worker.joinable() && !printJob.active.load(std::memory_order_acquire))
            printJob.worker.join();
    };

    auto getHostName = []() -> std::string {
        char* buf = nullptr;
        size_t len = 0;
        std::string host = "localhost";
        if (_dupenv_s(&buf, &len, "COMPUTERNAME") == 0 && buf) {
            host = buf;
            free(buf);
        }
        return host;
    };

    auto normalizeFilename = [](std::string filename) {
        std::replace(filename.begin(), filename.end(), '\\', '/');
        const std::string prefix = "gcode/";
        if (filename.rfind(prefix, 0) == 0)
            filename = filename.substr(prefix.size());
        return filename;
    };

    auto gcodeRoot = []() -> fs::path {
        return fs::path("gcode");
    };

    auto configRoot = []() -> fs::path {
        return fs::path("configs");
    };

    auto logsRoot = []() -> fs::path {
        return fs::path("logs");
    };

    auto resolveRootPath = [&](const std::string& root) -> fs::path {
        if (root == "config")
            return configRoot();
        if (root == "logs")
            return logsRoot();
        return gcodeRoot();
    };

    auto unixNow = []() -> double {
        using namespace std::chrono;
        return duration<double>(system_clock::now().time_since_epoch()).count();
    };

    auto fileTimeToUnixSeconds = [&](fs::file_time_type value) -> double {
        using namespace std::chrono;
        auto systemNow = system_clock::now();
        auto fileNow = fs::file_time_type::clock::now();
        auto translated = systemNow + duration_cast<system_clock::duration>(value - fileNow);
        return duration<double>(translated.time_since_epoch()).count();
    };

    auto formatJobId = [](uint64_t value) -> std::string {
        std::ostringstream ss;
        ss << std::uppercase << std::hex << std::setw(16) << std::setfill('0') << value;
        return ss.str();
    };

    auto tryParseFirstNumber = [](const std::string& text, double& value) -> bool {
        size_t start = 0;
        while (start < text.size()) {
            char ch = text[start];
            if ((ch >= '0' && ch <= '9') || ch == '-' || ch == '+')
                break;
            ++start;
        }
        if (start >= text.size())
            return false;
        try {
            size_t parsed = 0;
            value = std::stod(text.substr(start), &parsed);
            return parsed > 0;
        } catch (...) {
            return false;
        }
    };

    auto scanGcodeMetadata = [&](const std::string& filename) -> json {
        std::string rel = normalizeFilename(filename);
        fs::path path = gcodeRoot() / fs::path(rel);
        json meta = {
            {"size", 0},
            {"modified", 0.0},
            {"uuid", rel},
            {"slicer", "unknown"},
            {"object_height", nullptr},
            {"layer_height", nullptr},
            {"first_layer_height", nullptr},
            {"filament_total", nullptr},
            {"estimated_time", nullptr},
            {"thumbnails", json::array()},
            {"gcode_start_byte", 0},
            {"gcode_end_byte", 0},
            {"filename", rel}
        };
        if (!fs::exists(path) || !fs::is_regular_file(path))
            return meta;

        meta["size"] = static_cast<int64_t>(fs::file_size(path));
        meta["gcode_end_byte"] = meta["size"];
        try {
            meta["modified"] = fileTimeToUnixSeconds(fs::last_write_time(path));
        } catch (...) {
        }

        std::ifstream file(path);
        std::string line;
        double minZ = 0.0;
        double maxZ = 0.0;
        bool haveMinZ = false;
        bool haveMaxZ = false;
        while (std::getline(file, line)) {
            auto setMetaNumber = [&](const std::string& prefix, const char* field) {
                if (line.rfind(prefix, 0) != 0)
                    return false;
                double value = 0.0;
                if (!tryParseFirstNumber(line.substr(prefix.size()), value))
                    return false;
                meta[field] = value;
                return true;
            };

            if (line.rfind("; generated by ", 0) == 0) {
                meta["slicer"] = line.substr(15);
            } else if (line.rfind(";GENERATOR.NAME:", 0) == 0) {
                meta["slicer"] = line.substr(16);
            } else if (line.rfind(";TIME:", 0) == 0) {
                setMetaNumber(";TIME:", "estimated_time");
            } else if (line.rfind(";Layer height:", 0) == 0) {
                setMetaNumber(";Layer height:", "layer_height");
            } else if (line.rfind(";First layer height:", 0) == 0) {
                setMetaNumber(";First layer height:", "first_layer_height");
            } else if (line.rfind(";MINZ:", 0) == 0) {
                haveMinZ = tryParseFirstNumber(line.substr(6), minZ);
            } else if (line.rfind(";MAXZ:", 0) == 0) {
                haveMaxZ = tryParseFirstNumber(line.substr(6), maxZ);
            } else if (line.rfind(";Filament used", 0) == 0 || line.rfind("; filament used", 0) == 0) {
                double filament = 0.0;
                if (tryParseFirstNumber(line, filament)) {
                    if (line.find("mm") == std::string::npos && line.find('m') != std::string::npos)
                        filament *= 1000.0;
                    meta["filament_total"] = filament;
                }
            }
        }
        if (haveMinZ && haveMaxZ && maxZ >= minZ)
            meta["object_height"] = maxZ - minZ;
        return meta;
    };

    MoonrakerApiCallbacks callbacks;
    callbacks.getServerInfo = [&]() -> json {
        bool connected = ctx.mcu.isConnected() && !ctx.mcu.isShutdown();
        return {
            {"klippy_connected", connected},
            {"klippy_state", connected ? "ready" : (ctx.mcu.isShutdown() ? "shutdown" : "disconnected")},
            {"components", {"application", "klippy_connection", "machine", "file_manager", "job_queue", "history", "authorization", "database", "announcements", "webcam"}},
            {"failed_components", json::array()},
            {"registered_directories", {"config", "gcodes", "logs"}},
            {"warnings", json::array()},
            {"websocket_count", 0},
            {"moonraker_version", "klipper_host_cpp-dev"},
            {"api_version", {1, 0, 0}},
            {"api_version_string", "1.0.0"}
        };
    };

    callbacks.getServerConfig = [&]() -> json {
        return {
            {"config", {
                {"server", {
                    {"host", "0.0.0.0"},
                    {"port", httpPort},
                    {"ssl_port", 7130},
                    {"enable_debug_logging", false},
                    {"enable_asyncio_debug", false},
                    {"klippy_uds_address", nullptr},
                    {"max_upload_size", 210},
                    {"ssl_certificate_path", nullptr},
                    {"ssl_key_path", nullptr}
                }},
                {"file_manager", {
                    {"config_path", configRoot().generic_string()},
                    {"log_path", logsRoot().generic_string()},
                    {"enable_object_processing", false},
                    {"queue_gcode_uploads", true}
                }},
                {"data_store", {
                    {"temperature_store_size", 600},
                    {"gcode_store_size", 1000}
                }},
                {"authorization", {
                    {"force_logins", false},
                    {"cors_domains", json::array({"*://localhost", "*://app.fluidd.xyz"})},
                    {"trusted_clients", json::array({"127.0.0.0/8", "192.168.0.0/16"})}
                }},
                {"history", json::object()},
                {"job_queue", {
                    {"load_on_startup", true},
                    {"automatic_transition", false},
                    {"job_transition_delay", 2},
                    {"job_transition_gcode", ""}
                }},
                {"announcements", {{"subscriptions", json::array({"fluidd"})}}},
                {"octoprint_compat", json::object()},
                {"template", json::object()}
            }},
            {"orig", {
                {"server", {{"host", "0.0.0.0"}, {"port", std::to_string(httpPort)}}},
                {"file_manager", {{"config_path", configRoot().generic_string()}, {"log_path", logsRoot().generic_string()}}},
                {"authorization", {{"force_logins", "False"}}},
                {"history", json::object()},
                {"job_queue", json::object()},
                {"announcements", {{"subscriptions", "fluidd"}}}
            }},
            {"files", json::array({
                {{"filename", "moonraker.conf"}, {"sections", json::array({"server", "file_manager", "data_store", "authorization", "history", "job_queue", "announcements", "octoprint_compat"})}}
            })}
        };
    };

    callbacks.getPrinterInfo = [&]() -> json {
        bool shutdown = ctx.mcu.isShutdown();
        return {
            {"state", shutdown ? "error" : "ready"},
            {"state_message", shutdown ? ctx.mcu.getShutdownMsg() : "Printer is ready"},
            {"hostname", getHostName()},
            {"software_version", "klipper_host_cpp"}
        };
    };

    callbacks.getSystemInfo = [&]() -> json {
        return {
            {"system_info", {
                {"provider", "klipper_host_cpp"},
                {"platform", "windows"},
                {"hostname", getHostName()},
                {"cpu_info", {
                    {"cpu_count", std::thread::hardware_concurrency()},
                    {"bits", "64"},
                    {"processor", "generic"},
                    {"cpu_desc", "Host CPU"},
                    {"serial_number", "unknown"},
                    {"hardware_desc", "windows-pc"},
                    {"model", "Generic"},
                    {"total_memory", 0},
                    {"memory_units", "kB"}
                }},
                {"sd_info", {
                    {"manufacturer_id", "unknown"},
                    {"manufacturer", "unknown"},
                    {"oem_id", "unknown"},
                    {"product_name", "hostfs"},
                    {"product_revision", "1.0"},
                    {"serial_number", "unknown"},
                    {"manufacturer_date", "unknown"},
                    {"capacity", "unknown"},
                    {"total_bytes", 0}
                }},
                {"distribution", {
                    {"name", "Windows"},
                    {"id", "windows"},
                    {"like", json::array()},
                    {"version", "unknown"},
                    {"version_parts", {{"major", 0}, {"minor", 0}, {"build_number", 0}}},
                    {"codename", ""},
                    {"release_info", {{"description", "Windows"}, {"release_id", "windows"}, {"version_id", "unknown"}, {"codename", ""}}}
                }},
                {"available_services", json::array({"klipper_host"})},
                {"instance_ids", {{"moonraker", getHostName()}, {"klipper", "klipper_host_cpp"}}},
                {"service_state", {{"klipper_host", {{"active_state", "active"}, {"sub_state", "running"}}}}},
                {"python", {{"version_string", "n/a"}, {"version", json::array({0,0,0})}}},
                {"network", json::object()},
                {"canbus", json::object()},
                {"virtualization", {{"virt_type", "none"}, {"virt_identifier", "host"}}}
            }}
        };
    };

    callbacks.getAccessInfo = [&]() -> json {
        return {
            {"default_source", "moonraker"},
            {"available_sources", json::array({"moonraker"})},
            {"login_required", false},
            {"trusted", true}
        };
    };

    callbacks.getCurrentUser = [&]() -> json {
        return {
            {"username", nullptr},
            {"source", nullptr},
            {"created_on", nullptr}
        };
    };

    callbacks.listUsers = [&]() -> json {
        return {{"users", json::array()}};
    };

    callbacks.getApiKey = [&]() -> std::string {
        return "dev-token";
    };

    callbacks.queryObjects = [&](const std::string& query) -> json {
        Vec3 pos = ctx.toolhead->getPosition();
        double epos = ctx.toolhead->getExtruderPosition();
        double eventtime = ctx.mcu.getClockSync().estimatedPrintTime();
        auto requested = parseRequestedObjects(query);
        auto hasObj = [&](const std::string& name) {
            return std::find(requested.begin(), requested.end(), name) != requested.end();
        };

        json status = json::object();
        if (hasObj("webhooks")) {
            status["webhooks"] = {
                {"state", ctx.mcu.isShutdown() ? "shutdown" : "ready"},
                {"state_message", ctx.mcu.isShutdown() ? ctx.mcu.getShutdownMsg() : "ready"}
            };
        }
        if (hasObj("toolhead")) {
            status["toolhead"] = {
                {"position", {pos.x, pos.y, pos.z, epos}},
                {"max_velocity", ctx.toolhead->getMaxVelocity()},
                {"max_accel", ctx.toolhead->getMaxAccel()},
                {"print_time", ctx.toolhead->getNextPrintTime()},
                {"estimated_print_time", eventtime},
                {"stalls", 0}
            };
        }
        if (hasObj("gcode_move")) {
            status["gcode_move"] = {
                {"gcode_position", {pos.x, pos.y, pos.z, epos}},
                {"position", {pos.x, pos.y, pos.z, epos}},
                {"speed", ctx.gcode->getFeedrate()},
                {"speed_factor", ctx.gcode->getSpeedFactor()},
                {"extrude_factor", 1.0},
                {"absolute_coordinates", ctx.gcode->isAbsoluteMode()},
                {"absolute_extrude", ctx.gcode->isAbsoluteExtruderMode()},
                {"homing_origin", {0.0, 0.0, 0.0, 0.0}}
            };
        }
        if (hasObj("motion_report")) {
            status["motion_report"] = {
                {"live_position", {pos.x, pos.y, pos.z, epos}},
                {"live_velocity", 0.0},
                {"live_extruder_velocity", 0.0}
            };
        }
        if (hasObj("print_stats")) {
            std::lock_guard<std::mutex> lock(printJob.mutex);
            status["print_stats"] = {
                {"state", printJob.state},
                {"filename", printJob.filename},
                {"message", printJob.message},
                {"info", {{"total_layer", nullptr}, {"current_layer", nullptr}}},
                {"print_duration", printJob.printDuration},
                {"total_duration", printJob.totalDuration}
            };
        }
        if (hasObj("extruder")) {
            status["extruder"] = {
                {"temperature", 0.0},
                {"target", 0.0},
                {"power", 0.0},
                {"can_extrude", true},
                {"pressure_advance", ctx.toolhead->getPressureAdvance()},
                {"smooth_time", ctx.toolhead->getPressureAdvanceSmoothTime()}
            };
        }
        if (hasObj("heater_bed")) {
            status["heater_bed"] = {
                {"temperature", 0.0},
                {"target", 0.0},
                {"power", 0.0}
            };
        }
        if (hasObj("virtual_sdcard")) {
            std::lock_guard<std::mutex> lock(printJob.mutex);
            status["virtual_sdcard"] = {
                {"is_active", printJob.active.load(std::memory_order_acquire)},
                {"progress", printJob.progress},
                {"file_position", static_cast<int64_t>(printJob.filePosition)},
                {"file_size", static_cast<int64_t>(printJob.fileSize)}
            };
        }
        if (hasObj("configfile")) {
            status["configfile"] = {
                {"save_config_pending", false},
                {"warnings", json::array()},
                {"config", {
                    {"printer", {
                        {"max_velocity", ctx.toolhead->getMaxVelocity()},
                        {"max_accel", ctx.toolhead->getMaxAccel()}
                    }}
                }}
            };
        }

        return {
            {"eventtime", eventtime},
            {"status", status}
        };
    };

    callbacks.listFiles = [&](const std::string& root) -> json {
        json files = json::array();
        fs::path base = resolveRootPath(root);
        if (!fs::exists(base))
            return files;

        for (const auto& entry : fs::recursive_directory_iterator(base)) {
            if (!entry.is_regular_file())
                continue;
            std::string rel = fs::relative(entry.path(), base).generic_string();
            double modified = 0.0;
            try {
                modified = fileTimeToUnixSeconds(entry.last_write_time());
            } catch (...) {
            }
            std::string dirname = fs::relative(entry.path().parent_path(), base).generic_string();
            if (dirname == ".")
                dirname.clear();
            files.push_back({
                {"path", rel},
                {"filename", rel},
                {"dirname", dirname},
                {"modified", modified},
                {"size", static_cast<int64_t>(entry.file_size())},
                {"permissions", "rw"}
            });
        }
        return files;
    };

    callbacks.listFileRoots = [&]() -> json {
        return json::array({
            {{"name", "config"}, {"path", configRoot().generic_string()}, {"permissions", "rw"}},
            {{"name", "logs"}, {"path", logsRoot().generic_string()}, {"permissions", "rw"}},
            {{"name", "gcodes"}, {"path", gcodeRoot().generic_string()}, {"permissions", "rw"}}
        });
    };

    callbacks.getFileMetadata = [&](const std::string& filename) -> json {
        return scanGcodeMetadata(filename);
    };

    callbacks.getHistoryList = [&]() -> json {
        std::lock_guard<std::mutex> lock(history.mutex);
        return {
            {"count", static_cast<int>(history.jobs.size())},
            {"jobs", history.jobs}
        };
    };

    callbacks.getHistoryTotals = [&]() -> json {
        std::lock_guard<std::mutex> lock(history.mutex);
        return {
            {"job_totals", history.totals},
            {"auxiliary_totals", json::array()}
        };
    };

    callbacks.getGcodeStore = [&]() -> json {
        return {{"gcode_store", json::array()}};
    };

    callbacks.getAnnouncements = [&]() -> json {
        return {
            {"entries", json::array()},
            {"feeds", json::array()}
        };
    };

    callbacks.getJobQueueStatus = [&]() -> json {
        std::lock_guard<std::mutex> lock(printJob.mutex);
        std::string queueState = "ready";
        if (printJob.active.load(std::memory_order_acquire))
            queueState = printJob.pauseRequested.load(std::memory_order_acquire) ? "paused" : "loading";
        return {
            {"queued_jobs", json::array()},
            {"queue_state", queueState}
        };
    };

    callbacks.executeGcode = [&](const std::string& script, std::string& message) -> bool {
        if (script.empty()) {
            message = "Empty script";
            return false;
        }
        if (printJob.active.load(std::memory_order_acquire)) {
            message = "Print is in progress";
            return false;
        }
        std::lock_guard<std::mutex> lock(g_mcuMutex);
        int executed = ctx.gcode->executeBlock(script);
        if (executed <= 0) {
            message = ctx.gcode->getLastMessage();
            if (message.empty())
                message = "No commands executed";
            return false;
        }
        message = "ok";
        return true;
    };

    callbacks.startPrint = [&](const std::string& filename, std::string& message) -> bool {
        joinFinishedWorker();
        if (printJob.active.load(std::memory_order_acquire)) {
            message = "A print is already running";
            return false;
        }

        std::string rel = normalizeFilename(filename);
        fs::path path = gcodeRoot() / fs::path(rel);
        if (!fs::exists(path) || !fs::is_regular_file(path)) {
            message = "File not found: " + rel;
            return false;
        }

        json metadata = scanGcodeMetadata(rel);
        std::string jobId;
        {
            std::lock_guard<std::mutex> lock(history.mutex);
            jobId = formatJobId(history.nextJobId++);
        }

        {
            std::lock_guard<std::mutex> lock(printJob.mutex);
            printJob.active.store(true, std::memory_order_release);
            printJob.pauseRequested.store(false, std::memory_order_release);
            printJob.cancelRequested.store(false, std::memory_order_release);
            printJob.state = "printing";
            printJob.jobId = jobId;
            printJob.filename = rel;
            printJob.message.clear();
            printJob.metadata = metadata;
            printJob.filePosition = 0;
            printJob.fileSize = metadata.value("size", static_cast<int64_t>(fs::file_size(path)));
            printJob.progress = 0.0;
            printJob.startTime = unixNow();
            printJob.endTime = 0.0;
            printJob.filamentUsed = metadata.contains("filament_total") && metadata["filament_total"].is_number()
                ? metadata["filament_total"].get<double>() : 0.0;
            printJob.printDuration = 0.0;
            printJob.totalDuration = 0.0;
            printJob.startedAt = std::chrono::steady_clock::now();
        }

        printJob.worker = std::thread([&, path, rel]() {
            GcodeRunHooks hooks;
            hooks.shouldPause = [&]() {
                return printJob.pauseRequested.load(std::memory_order_acquire);
            };
            hooks.shouldCancel = [&]() {
                return printJob.cancelRequested.load(std::memory_order_acquire);
            };
            hooks.onProgress = [&](size_t filePosition, size_t fileSize, size_t, size_t) {
                std::lock_guard<std::mutex> lock(printJob.mutex);
                printJob.filePosition = filePosition;
                printJob.fileSize = fileSize;
                if (fileSize > 0) {
                    printJob.progress = (std::min)(1.0,
                        static_cast<double>(filePosition) / static_cast<double>(fileSize));
                }
                auto elapsed = std::chrono::steady_clock::now() - printJob.startedAt;
                double elapsedSec = std::chrono::duration<double>(elapsed).count();
                printJob.totalDuration = elapsedSec;
                printJob.printDuration = elapsedSec;
                if (printJob.pauseRequested.load(std::memory_order_acquire))
                    printJob.state = "paused";
                else if (!printJob.cancelRequested.load(std::memory_order_acquire))
                    printJob.state = "printing";
            };

            int rc = runGcodeFile(ctx, path.string(), 0, 1.0, &hooks);

            json historyEntry;
            {
                std::lock_guard<std::mutex> lock(printJob.mutex);
                printJob.endTime = unixNow();
                if (printJob.cancelRequested.load(std::memory_order_acquire)) {
                    printJob.state = "cancelled";
                    printJob.message = "Print cancelled";
                } else if (g_shutdown || ctx.mcu.isShutdown()) {
                    printJob.state = "error";
                    printJob.message = ctx.mcu.getShutdownMsg().empty() ? "MCU shutdown" : ctx.mcu.getShutdownMsg();
                } else if (rc != 0) {
                    printJob.state = "error";
                    if (printJob.message.empty())
                        printJob.message = ctx.gcode->getLastMessage().empty() ? "Print failed" : ctx.gcode->getLastMessage();
                } else {
                    printJob.state = "complete";
                    printJob.message = "Print finished";
                    printJob.progress = 1.0;
                    printJob.filePosition = printJob.fileSize;
                }
                historyEntry = {
                    {"job_id", printJob.jobId},
                    {"user", nullptr},
                    {"filename", printJob.filename},
                    {"exists", true},
                    {"status", printJob.state == "complete" ? "completed" : printJob.state},
                    {"start_time", printJob.startTime},
                    {"end_time", printJob.endTime},
                    {"print_duration", printJob.printDuration},
                    {"total_duration", printJob.totalDuration},
                    {"filament_used", printJob.filamentUsed},
                    {"metadata", printJob.metadata},
                    {"auxiliary_data", json::array()}
                };
                printJob.active.store(false, std::memory_order_release);
            }

            {
                std::lock_guard<std::mutex> lock(history.mutex);
                history.jobs.insert(history.jobs.begin(), historyEntry);
                history.totals["total_jobs"] = history.totals.value("total_jobs", 0) + 1;
                history.totals["total_time"] = history.totals.value("total_time", 0.0) + historyEntry.value("total_duration", 0.0);
                history.totals["total_print_time"] = history.totals.value("total_print_time", 0.0) + historyEntry.value("print_duration", 0.0);
                history.totals["total_filament_used"] = history.totals.value("total_filament_used", 0.0) + historyEntry.value("filament_used", 0.0);
                history.totals["longest_job"] = (std::max)(history.totals.value("longest_job", 0.0), historyEntry.value("total_duration", 0.0));
                history.totals["longest_print"] = (std::max)(history.totals.value("longest_print", 0.0), historyEntry.value("print_duration", 0.0));
            }
        });

        message = "ok";
        return true;
    };

    callbacks.pausePrint = [&](std::string& message) -> bool {
        if (!printJob.active.load(std::memory_order_acquire)) {
            message = "No active print";
            return false;
        }
        printJob.pauseRequested.store(true, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(printJob.mutex);
            printJob.state = "paused";
            printJob.message = "Print paused";
        }
        message = "ok";
        return true;
    };

    callbacks.resumePrint = [&](std::string& message) -> bool {
        if (!printJob.active.load(std::memory_order_acquire)) {
            message = "No active print";
            return false;
        }
        printJob.pauseRequested.store(false, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(printJob.mutex);
            printJob.state = "printing";
            printJob.message = "Print resumed";
        }
        message = "ok";
        return true;
    };

    callbacks.cancelPrint = [&](std::string& message) -> bool {
        if (!printJob.active.load(std::memory_order_acquire)) {
            message = "No active print";
            return false;
        }
        printJob.pauseRequested.store(false, std::memory_order_release);
        printJob.cancelRequested.store(true, std::memory_order_release);
        {
            std::lock_guard<std::mutex> lock(printJob.mutex);
            printJob.state = "cancelled";
            printJob.message = "Cancellation requested";
        }
        message = "ok";
        return true;
    };

    MoonrakerApiServer server(std::move(callbacks));
    std::string error;
    if (!server.start(static_cast<uint16_t>(httpPort), error)) {
        LogError("Moonraker API start failed: " + error);
        return 1;
    }

    Log("Moonraker-compatible API listening on http://0.0.0.0:" + std::to_string(httpPort));
    Log("Press Ctrl-C to stop the API server");

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    if (printJob.active.load(std::memory_order_acquire)) {
        printJob.cancelRequested.store(true, std::memory_order_release);
    }
    if (printJob.worker.joinable())
        printJob.worker.join();

    server.stop();
    return 0;
}

// ---- Usage ----
static void printUsage(const char* exe) {
    std::cerr << "Usage:\n"
              << "  " << exe << " gcode <file.gcode>    - run gcode file\n"
              << "  " << exe << " move \"<gcode>\"        - execute gcode block\n"
              << "  " << exe << " monitor [seconds]      - monitor MCU stats\n"
              << "  " << exe << " info                   - show MCU info\n"
              << "  " << exe << " api [http_port]        - run Moonraker-style HTTP API\n\n"
              << "Options:\n"
              << "  --port <COMx>       - serial port (default COM3)\n"
              << "  --config <file>     - config file (default configs/generic-duet3-6hc.cfg)\n"
              << "  --start-line <N>    - skip to line N (skip non-move lines before it)\n"
              << "  --speed-factor <X>  - speed multiplier (e.g. 2.0 for 2x speed)\n"
              << "  --http-port <N>     - HTTP API port for api mode (default 7125)\n";
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
    size_t g_startLine = 0;
    double g_speedFactor = 1.0;
    int httpPort = 7125;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--port" && i + 1 < argc) {
            port = argv[++i];
        } else if (arg == "--config" && i + 1 < argc) {
            configPath = argv[++i];
        } else if (arg == "--start-line" && i + 1 < argc) {
            g_startLine = std::stoull(argv[++i]);
        } else if (arg == "--speed-factor" && i + 1 < argc) {
            g_speedFactor = std::stod(argv[++i]);
        } else if (arg == "--http-port" && i + 1 < argc) {
            httpPort = std::stoi(argv[++i]);
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
        // Reactor-based: no legacy threads needed
        if (modeArg.empty()) {
            LogError("gcode mode requires a file path");
            result = 1;
        } else {
            result = runGcodeFile(ctx, modeArg, g_startLine, g_speedFactor);
        }
    } else if (mode == "move") {
        ctx.startLegacyThreads();
        if (modeArg.empty()) {
            LogError("move mode requires a gcode string");
            result = 1;
        } else {
            result = runGcodeBlock(ctx, modeArg);
        }
    } else if (mode == "monitor") {
        ctx.startLegacyThreads();
        int seconds = modeArg.empty() ? 30 : std::stoi(modeArg);
        result = runMonitor(ctx, seconds);
    } else if (mode == "info") {
        ctx.startLegacyThreads();
        result = runInfo(ctx);
    } else if (mode == "api") {
        if (!modeArg.empty())
            httpPort = std::stoi(modeArg);
        result = runApiServer(ctx, httpPort);
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

