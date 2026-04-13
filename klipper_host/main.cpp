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
#include "reactor.h"

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

// ---- Mode: run gcode file (reactor-based) ----
static int runGcodeFile(TestContext& ctx, const std::string& filePath, size_t startLine = 0, double speedFactor = 1.0) {
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
        newLines.insert(newLines.end(), preamble.begin(), preamble.end());
        newLines.insert(newLines.end(), lines.begin() + startLine, lines.end());
        lines = std::move(newLines);
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
        if (!g_running || gcodeIdx >= lines.size()) {
            // Final flush + step gen
            if (ctx.mcu.isConnected() && !ctx.mcu.isShutdown()) {
                ctx.toolhead->flush();
                ctx.toolhead->generateSteps(true);
            }
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

// ---- Usage ----
static void printUsage(const char* exe) {
    std::cerr << "Usage:\n"
              << "  " << exe << " gcode <file.gcode>    - run gcode file\n"
              << "  " << exe << " move \"<gcode>\"        - execute gcode block\n"
              << "  " << exe << " monitor [seconds]      - monitor MCU stats\n"
              << "  " << exe << " info                   - show MCU info\n\n"
              << "Options:\n"
              << "  --port <COMx>     - serial port (default COM3)\n"
              << "  --config <file>   - config file (default configs/generic-duet3-6hc.cfg)\n"
              << "  --start-line <N>  - skip to line N (skip non-move lines before it)\n"
              << "  --speed-factor <X> - speed multiplier (e.g. 2.0 for 2x speed)\n";
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

