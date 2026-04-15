#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <map>
#include <vector>

#include "nlohmann/json.hpp"

struct MoonrakerApiCallbacks {
    std::function<nlohmann::json()> getServerInfo;
    std::function<nlohmann::json()> getPrinterInfo;
    std::function<nlohmann::json()> getSystemInfo;
    std::function<nlohmann::json(const std::string& query)> queryObjects;
    std::function<nlohmann::json()> listFiles;
    std::function<nlohmann::json(const std::string& filename)> getFileMetadata;
    std::function<bool(const std::string& script, std::string& message)> executeGcode;
    std::function<bool(const std::string& filename, std::string& message)> startPrint;
    std::function<bool(std::string& message)> pausePrint;
    std::function<bool(std::string& message)> resumePrint;
    std::function<bool(std::string& message)> cancelPrint;
};

class MoonrakerApiServer {
public:
    explicit MoonrakerApiServer(MoonrakerApiCallbacks callbacks = {});
    ~MoonrakerApiServer();

    bool start(uint16_t port, std::string& error);
    void stop();

    bool isRunning() const { return m_running.load(std::memory_order_acquire); }
    uint16_t port() const { return m_port; }

private:
    struct ClientSession;

    void acceptLoop();
    void notifyLoop();
    void pruneClosedClients();
    void closeClientSocket(const std::shared_ptr<ClientSession>& session);
    bool sendJsonToClient(const std::shared_ptr<ClientSession>& session,
                          const nlohmann::json& payload,
                          uint8_t opcode = 0x1);
    void handleClient(const std::shared_ptr<ClientSession>& session);
    void handleWebSocketClient(const std::shared_ptr<ClientSession>& session);
    nlohmann::json dispatchJsonRpc(const nlohmann::json& message,
                                   bool& sendStatusNotify,
                                   nlohmann::json& notifyPayload,
                                   std::string& subscribedQuery);

    MoonrakerApiCallbacks m_callbacks;
    std::atomic<bool> m_running{false};
    std::thread m_acceptThread;
    std::thread m_notifyThread;
    uintptr_t m_listenSocket = 0;
    uint16_t m_port = 0;
    std::mutex m_clientsMutex;
    std::vector<std::shared_ptr<ClientSession>> m_clients;
    std::mutex m_stateMutex;
    std::map<std::string, nlohmann::json> m_database;
};
