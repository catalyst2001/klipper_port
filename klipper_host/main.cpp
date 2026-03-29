#include <iostream>
#include <string>
#include <iomanip>
#include "klipper_mcu.h"

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

    std::cout << "\n=== Done ===" << std::endl;
    mcu.disconnect();
    return 0;
}