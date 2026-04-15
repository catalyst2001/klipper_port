#include "moonraker_api.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#define NOMINMAX
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")

using json = nlohmann::json;

namespace {
struct HttpRequest {
    std::string method;
    std::string target;
    std::string path;
    std::string query;
    std::map<std::string, std::string> headers;
    std::string body;
};

static std::string trim(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start])))
        ++start;
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1])))
        --end;
    return s.substr(start, end - start);
}

static std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

static std::string urlDecode(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '%' && i + 2 < s.size()) {
            char hex[3] = { s[i + 1], s[i + 2], 0 };
            out.push_back(static_cast<char>(std::strtoul(hex, nullptr, 16)));
            i += 2;
        } else if (s[i] == '+') {
            out.push_back(' ');
        } else {
            out.push_back(s[i]);
        }
    }
    return out;
}

static std::map<std::string, std::string> parseQuery(const std::string& query) {
    std::map<std::string, std::string> result;
    std::istringstream ss(query);
    std::string token;
    while (std::getline(ss, token, '&')) {
        if (token.empty())
            continue;
        auto eq = token.find('=');
        if (eq == std::string::npos) {
            result[urlDecode(token)] = "";
        } else {
            result[urlDecode(token.substr(0, eq))] = urlDecode(token.substr(eq + 1));
        }
    }
    return result;
}

static bool parseRequest(const std::string& raw, HttpRequest& req) {
    auto headerEnd = raw.find("\r\n\r\n");
    if (headerEnd == std::string::npos)
        return false;

    std::istringstream ss(raw.substr(0, headerEnd));
    std::string line;
    if (!std::getline(ss, line))
        return false;
    if (!line.empty() && line.back() == '\r')
        line.pop_back();

    std::istringstream rl(line);
    std::string version;
    if (!(rl >> req.method >> req.target >> version))
        return false;

    auto qpos = req.target.find('?');
    if (qpos == std::string::npos) {
        req.path = req.target;
        req.query.clear();
    } else {
        req.path = req.target.substr(0, qpos);
        req.query = req.target.substr(qpos + 1);
    }

    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        if (line.empty())
            break;
        auto colon = line.find(':');
        if (colon == std::string::npos)
            continue;
        req.headers[toLower(trim(line.substr(0, colon)))] = trim(line.substr(colon + 1));
    }

    req.body = raw.substr(headerEnd + 4);
    return true;
}

static std::string jsonResponse(int code, const json& payload) {
    std::string status = "200 OK";
    if (code == 204) status = "204 No Content";
    else if (code == 400) status = "400 Bad Request";
    else if (code == 404) status = "404 Not Found";
    else if (code == 500) status = "500 Internal Server Error";
    std::string body = (code == 204) ? "" : payload.dump(2);

    std::ostringstream ss;
    ss << "HTTP/1.1 " << status << "\r\n"
       << "Content-Type: application/json\r\n"
       << "Access-Control-Allow-Origin: *\r\n"
       << "Access-Control-Allow-Headers: Content-Type, X-Api-Key\r\n"
       << "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n"
       << "Connection: close\r\n"
       << "Content-Length: " << body.size() << "\r\n\r\n"
       << body;
    return ss.str();
}

static json makeError(int code, const std::string& message) {
    return json{{"error", {{"code", code}, {"message", message}}}};
}

static std::string recvRequest(SOCKET client) {
    std::string data;
    char buf[4096];
    int contentLength = -1;
    size_t headerEnd = std::string::npos;

    for (;;) {
        int got = recv(client, buf, sizeof(buf), 0);
        if (got <= 0)
            break;
        data.append(buf, buf + got);

        if (headerEnd == std::string::npos) {
            headerEnd = data.find("\r\n\r\n");
            if (headerEnd != std::string::npos) {
                std::string headers = data.substr(0, headerEnd);
                auto pos = toLower(headers).find("content-length:");
                if (pos != std::string::npos) {
                    pos += std::strlen("content-length:");
                    while (pos < headers.size() && std::isspace(static_cast<unsigned char>(headers[pos])))
                        ++pos;
                    size_t end = pos;
                    while (end < headers.size() && std::isdigit(static_cast<unsigned char>(headers[end])))
                        ++end;
                    contentLength = std::stoi(headers.substr(pos, end - pos));
                } else {
                    contentLength = 0;
                }
            }
        }

        if (headerEnd != std::string::npos && contentLength >= 0) {
            size_t totalNeeded = headerEnd + 4 + static_cast<size_t>(contentLength);
            if (data.size() >= totalNeeded)
                break;
        }

        if (data.size() > 1024 * 1024)
            break;
    }
    return data;
}

static bool sendAll(SOCKET client, const void* data, size_t len) {
    const char* ptr = static_cast<const char*>(data);
    while (len > 0) {
        int sent = send(client, ptr, static_cast<int>(len), 0);
        if (sent <= 0)
            return false;
        ptr += sent;
        len -= static_cast<size_t>(sent);
    }
    return true;
}

static uint32_t rol32(uint32_t value, int bits) {
    return (value << bits) | (value >> (32 - bits));
}

static std::array<uint8_t, 20> sha1Digest(const std::string& input) {
    std::vector<uint8_t> data(input.begin(), input.end());
    const uint64_t bitLen = static_cast<uint64_t>(data.size()) * 8ULL;
    data.push_back(0x80);
    while ((data.size() % 64) != 56)
        data.push_back(0x00);
    for (int i = 7; i >= 0; --i)
        data.push_back(static_cast<uint8_t>((bitLen >> (i * 8)) & 0xFF));

    uint32_t h0 = 0x67452301;
    uint32_t h1 = 0xEFCDAB89;
    uint32_t h2 = 0x98BADCFE;
    uint32_t h3 = 0x10325476;
    uint32_t h4 = 0xC3D2E1F0;

    for (size_t chunk = 0; chunk < data.size(); chunk += 64) {
        uint32_t w[80] = {};
        for (int i = 0; i < 16; ++i) {
            size_t off = chunk + i * 4;
            w[i] = (static_cast<uint32_t>(data[off]) << 24)
                 | (static_cast<uint32_t>(data[off + 1]) << 16)
                 | (static_cast<uint32_t>(data[off + 2]) << 8)
                 | static_cast<uint32_t>(data[off + 3]);
        }
        for (int i = 16; i < 80; ++i)
            w[i] = rol32(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);

        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; ++i) {
            uint32_t f = 0, k = 0;
            if (i < 20) {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }
            uint32_t temp = rol32(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = rol32(b, 30);
            b = a;
            a = temp;
        }

        h0 += a; h1 += b; h2 += c; h3 += d; h4 += e;
    }

    std::array<uint8_t, 20> out{};
    const uint32_t words[5] = { h0, h1, h2, h3, h4 };
    for (int i = 0; i < 5; ++i) {
        out[i * 4 + 0] = static_cast<uint8_t>((words[i] >> 24) & 0xFF);
        out[i * 4 + 1] = static_cast<uint8_t>((words[i] >> 16) & 0xFF);
        out[i * 4 + 2] = static_cast<uint8_t>((words[i] >> 8) & 0xFF);
        out[i * 4 + 3] = static_cast<uint8_t>(words[i] & 0xFF);
    }
    return out;
}

static std::string base64Encode(const uint8_t* data, size_t len) {
    static constexpr char alphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    out.reserve(((len + 2) / 3) * 4);
    for (size_t i = 0; i < len; i += 3) {
        uint32_t triple = static_cast<uint32_t>(data[i]) << 16;
        bool have2 = (i + 1 < len);
        bool have3 = (i + 2 < len);
        if (have2) triple |= static_cast<uint32_t>(data[i + 1]) << 8;
        if (have3) triple |= static_cast<uint32_t>(data[i + 2]);
        out.push_back(alphabet[(triple >> 18) & 0x3F]);
        out.push_back(alphabet[(triple >> 12) & 0x3F]);
        out.push_back(have2 ? alphabet[(triple >> 6) & 0x3F] : '=');
        out.push_back(have3 ? alphabet[triple & 0x3F] : '=');
    }
    return out;
}

static std::string websocketAcceptKey(const std::string& clientKey) {
    const std::string magic = clientKey + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    auto digest = sha1Digest(magic);
    return base64Encode(digest.data(), digest.size());
}

static bool sendWsFrame(SOCKET client, uint8_t opcode, const std::string& payload) {
    std::vector<uint8_t> frame;
    frame.reserve(payload.size() + 16);
    frame.push_back(static_cast<uint8_t>(0x80 | (opcode & 0x0F)));
    if (payload.size() < 126) {
        frame.push_back(static_cast<uint8_t>(payload.size()));
    } else if (payload.size() <= 0xFFFF) {
        frame.push_back(126);
        frame.push_back(static_cast<uint8_t>((payload.size() >> 8) & 0xFF));
        frame.push_back(static_cast<uint8_t>(payload.size() & 0xFF));
    } else {
        frame.push_back(127);
        uint64_t len64 = static_cast<uint64_t>(payload.size());
        for (int i = 7; i >= 0; --i)
            frame.push_back(static_cast<uint8_t>((len64 >> (i * 8)) & 0xFF));
    }
    frame.insert(frame.end(), payload.begin(), payload.end());
    return sendAll(client, frame.data(), frame.size());
}

static bool recvExact(SOCKET client, void* buf, size_t len) {
    char* ptr = static_cast<char*>(buf);
    while (len > 0) {
        int got = recv(client, ptr, static_cast<int>(len), 0);
        if (got <= 0)
            return false;
        ptr += got;
        len -= static_cast<size_t>(got);
    }
    return true;
}

static bool recvWsFrame(SOCKET client, uint8_t& opcode, std::string& payload) {
    uint8_t hdr[2] = {};
    if (!recvExact(client, hdr, sizeof(hdr)))
        return false;

    opcode = static_cast<uint8_t>(hdr[0] & 0x0F);
    const bool masked = (hdr[1] & 0x80) != 0;
    uint64_t len = hdr[1] & 0x7F;
    if (len == 126) {
        uint8_t ext[2] = {};
        if (!recvExact(client, ext, sizeof(ext)))
            return false;
        len = (static_cast<uint64_t>(ext[0]) << 8) | static_cast<uint64_t>(ext[1]);
    } else if (len == 127) {
        uint8_t ext[8] = {};
        if (!recvExact(client, ext, sizeof(ext)))
            return false;
        len = 0;
        for (int i = 0; i < 8; ++i)
            len = (len << 8) | static_cast<uint64_t>(ext[i]);
    }

    uint8_t maskKey[4] = {};
    if (masked && !recvExact(client, maskKey, sizeof(maskKey)))
        return false;

    payload.assign(static_cast<size_t>(len), '\0');
    if (len > 0 && !recvExact(client, payload.data(), static_cast<size_t>(len)))
        return false;

    if (masked) {
        for (size_t i = 0; i < payload.size(); ++i)
            payload[i] = static_cast<char>(payload[i] ^ maskKey[i % 4]);
    }
    return true;
}

static std::string objectsQueryStringFromJson(const json& objects) {
    if (!objects.is_object())
        return std::string();
    std::ostringstream ss;
    bool first = true;
    for (auto it = objects.begin(); it != objects.end(); ++it) {
        if (!first)
            ss << '&';
        ss << it.key();
        first = false;
    }
    return ss.str();
}

static json makeJsonRpcResult(const json& id, const json& result) {
    return json{{"jsonrpc", "2.0"}, {"result", result}, {"id", id}};
}

static json makeJsonRpcError(const json& id, int code, const std::string& message) {
    return json{{"jsonrpc", "2.0"}, {"error", {{"code", code}, {"message", message}}}, {"id", id}};
}
} // namespace

MoonrakerApiServer::MoonrakerApiServer(MoonrakerApiCallbacks callbacks)
    : m_callbacks(std::move(callbacks)) {}

struct MoonrakerApiServer::ClientSession {
    std::atomic<uintptr_t> socketHandle{0};
    std::thread thread;
    std::mutex sendMutex;
    std::mutex stateMutex;
    std::atomic<bool> websocket{false};
    std::atomic<bool> closing{false};
    std::atomic<bool> finished{false};
    std::string subscriptionQuery;
    json lastStatus = json::object();
};

MoonrakerApiServer::~MoonrakerApiServer() {
    stop();
}

bool MoonrakerApiServer::start(uint16_t port, std::string& error) {
    if (m_running.load(std::memory_order_acquire)) {
        error = "Server already running";
        return false;
    }

    WSADATA wsaData{};
    int rc = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (rc != 0) {
        error = "WSAStartup failed";
        return false;
    }

    SOCKET listenSock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listenSock == INVALID_SOCKET) {
        error = "socket() failed";
        WSACleanup();
        return false;
    }

    BOOL opt = 1;
    setsockopt(listenSock, SOL_SOCKET, SO_REUSEADDR,
               reinterpret_cast<const char*>(&opt), sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(listenSock, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == SOCKET_ERROR) {
        error = "bind() failed";
        closesocket(listenSock);
        WSACleanup();
        return false;
    }
    if (listen(listenSock, 8) == SOCKET_ERROR) {
        error = "listen() failed";
        closesocket(listenSock);
        WSACleanup();
        return false;
    }

    m_port = port;
    m_listenSocket = static_cast<uintptr_t>(listenSock);
    m_running.store(true, std::memory_order_release);
    m_acceptThread = std::thread(&MoonrakerApiServer::acceptLoop, this);
    m_notifyThread = std::thread(&MoonrakerApiServer::notifyLoop, this);
    return true;
}

void MoonrakerApiServer::stop() {
    if (!m_running.exchange(false, std::memory_order_acq_rel))
        return;

    SOCKET listenSock = static_cast<SOCKET>(m_listenSocket);
    if (listenSock != INVALID_SOCKET && listenSock != 0) {
        shutdown(listenSock, SD_BOTH);
        closesocket(listenSock);
    }
    m_listenSocket = 0;

    if (m_acceptThread.joinable())
        m_acceptThread.join();

    std::vector<std::shared_ptr<ClientSession>> clients;
    {
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        clients = m_clients;
    }
    for (const auto& session : clients)
        closeClientSocket(session);

    if (m_notifyThread.joinable())
        m_notifyThread.join();

    for (const auto& session : clients) {
        if (session->thread.joinable())
            session->thread.join();
    }

    {
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        m_clients.clear();
    }

    WSACleanup();
}

void MoonrakerApiServer::acceptLoop() {
    SOCKET listenSock = static_cast<SOCKET>(m_listenSocket);
    while (m_running.load(std::memory_order_acquire)) {
        sockaddr_in clientAddr{};
        int addrLen = sizeof(clientAddr);
        SOCKET client = accept(listenSock, reinterpret_cast<sockaddr*>(&clientAddr), &addrLen);
        if (client == INVALID_SOCKET) {
            if (!m_running.load(std::memory_order_acquire))
                break;
            continue;
        }
        auto session = std::make_shared<ClientSession>();
        session->socketHandle.store(static_cast<uintptr_t>(client), std::memory_order_release);
        session->thread = std::thread(&MoonrakerApiServer::handleClient, this, session);
        {
            std::lock_guard<std::mutex> lock(m_clientsMutex);
            m_clients.push_back(session);
        }
        pruneClosedClients();
    }
}

void MoonrakerApiServer::notifyLoop() {
    while (m_running.load(std::memory_order_acquire)) {
        std::vector<std::shared_ptr<ClientSession>> clients;
        {
            std::lock_guard<std::mutex> lock(m_clientsMutex);
            clients = m_clients;
        }

        for (const auto& session : clients) {
            if (!session->websocket.load(std::memory_order_acquire)
                || session->closing.load(std::memory_order_acquire)) {
                continue;
            }

            std::string query;
            json lastStatus;
            {
                std::lock_guard<std::mutex> lock(session->stateMutex);
                query = session->subscriptionQuery;
                lastStatus = session->lastStatus;
            }
            if (query.empty() || !m_callbacks.queryObjects)
                continue;

            json result = m_callbacks.queryObjects(query);
            if (!result.contains("status") || !result.contains("eventtime"))
                continue;

            json status = result["status"];
            if (status == lastStatus)
                continue;

            {
                std::lock_guard<std::mutex> lock(session->stateMutex);
                if (session->subscriptionQuery != query)
                    continue;
                session->lastStatus = status;
            }

            sendJsonToClient(session, {
                {"jsonrpc", "2.0"},
                {"method", "notify_status_update"},
                {"params", json::array({status, result["eventtime"]})}
            });
        }

        pruneClosedClients();
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
}

void MoonrakerApiServer::pruneClosedClients() {
    std::vector<std::shared_ptr<ClientSession>> finished;
    {
        std::lock_guard<std::mutex> lock(m_clientsMutex);
        auto it = m_clients.begin();
        while (it != m_clients.end()) {
            if ((*it)->finished.load(std::memory_order_acquire)) {
                finished.push_back(*it);
                it = m_clients.erase(it);
            } else {
                ++it;
            }
        }
    }

    for (const auto& session : finished) {
        if (session->thread.joinable()
            && session->thread.get_id() != std::this_thread::get_id()) {
            session->thread.join();
        }
    }
}

void MoonrakerApiServer::closeClientSocket(const std::shared_ptr<ClientSession>& session) {
    uintptr_t handle = session->socketHandle.exchange(0, std::memory_order_acq_rel);
    if (handle == 0)
        return;

    SOCKET client = static_cast<SOCKET>(handle);
    session->closing.store(true, std::memory_order_release);
    shutdown(client, SD_BOTH);
    closesocket(client);
}

bool MoonrakerApiServer::sendJsonToClient(const std::shared_ptr<ClientSession>& session,
                                         const json& payload,
                                         uint8_t opcode) {
    uintptr_t handle = session->socketHandle.load(std::memory_order_acquire);
    if (handle == 0)
        return false;

    std::lock_guard<std::mutex> lock(session->sendMutex);
    handle = session->socketHandle.load(std::memory_order_acquire);
    if (handle == 0)
        return false;

    const std::string wirePayload = (opcode == 0x1) ? payload.dump() : std::string();
    if (!sendWsFrame(static_cast<SOCKET>(handle), opcode, wirePayload)) {
        closeClientSocket(session);
        return false;
    }
    return true;
}

json MoonrakerApiServer::dispatchJsonRpc(const json& message,
                                         bool& sendStatusNotify,
                                         json& notifyPayload,
                                         std::string& subscribedQuery) {
    sendStatusNotify = false;
    notifyPayload = json::array();
    subscribedQuery.clear();

    const json id = message.contains("id") ? message["id"] : json(nullptr);
    const std::string method = message.value("method", "");
    json params = message.contains("params") ? message["params"] : json::object();
    if (!params.is_object())
        params = json::object();

    if (method.empty())
        return makeJsonRpcError(id, -32600, "Missing method");

    if (method == "server.connection.identify") {
        return makeJsonRpcResult(id, {
            {"connection_id", 1},
            {"state", "ready"},
            {"moonraker_version", "klipper_host_cpp-dev"}
        });
    }
    if (method == "server.info") {
        return makeJsonRpcResult(id,
            m_callbacks.getServerInfo ? m_callbacks.getServerInfo() : json::object());
    }
    if (method == "printer.info") {
        return makeJsonRpcResult(id,
            m_callbacks.getPrinterInfo ? m_callbacks.getPrinterInfo() : json::object());
    }
    if (method == "machine.system_info") {
        return makeJsonRpcResult(id,
            m_callbacks.getSystemInfo ? m_callbacks.getSystemInfo() : json::object());
    }
    if (method == "printer.objects.list") {
        return makeJsonRpcResult(id, {
            {"objects", {"webhooks", "toolhead", "gcode_move", "motion_report",
                           "print_stats", "extruder", "heater_bed", "virtual_sdcard", "configfile"}}
        });
    }
    if (method == "printer.objects.query") {
        std::string query = objectsQueryStringFromJson(params.value("objects", json::object()));
        return makeJsonRpcResult(id,
            m_callbacks.queryObjects ? m_callbacks.queryObjects(query) : json::object());
    }
    if (method == "printer.objects.subscribe") {
        std::string query = objectsQueryStringFromJson(params.value("objects", json::object()));
        subscribedQuery = query;
        json result = m_callbacks.queryObjects ? m_callbacks.queryObjects(query) : json::object();
        if (result.contains("status") && result.contains("eventtime")) {
            sendStatusNotify = true;
            notifyPayload = json::array({result["status"], result["eventtime"]});
        }
        return makeJsonRpcResult(id, result);
    }
    if (method == "printer.gcode.script") {
        std::string script = params.value("script", "");
        std::string msg;
        if (!m_callbacks.executeGcode || !m_callbacks.executeGcode(script, msg))
            return makeJsonRpcError(id, -32000, msg.empty() ? "G-code execution failed" : msg);
        return makeJsonRpcResult(id, "ok");
    }
    if (method == "printer.print.start") {
        std::string filename = params.value("filename", "");
        std::string msg;
        if (!m_callbacks.startPrint || !m_callbacks.startPrint(filename, msg))
            return makeJsonRpcError(id, -32000, msg.empty() ? "Unable to start print" : msg);
        return makeJsonRpcResult(id, "ok");
    }
    if (method == "printer.print.pause") {
        std::string msg;
        if (!m_callbacks.pausePrint || !m_callbacks.pausePrint(msg))
            return makeJsonRpcError(id, -32000, msg.empty() ? "Unable to pause print" : msg);
        return makeJsonRpcResult(id, "ok");
    }
    if (method == "printer.print.resume") {
        std::string msg;
        if (!m_callbacks.resumePrint || !m_callbacks.resumePrint(msg))
            return makeJsonRpcError(id, -32000, msg.empty() ? "Unable to resume print" : msg);
        return makeJsonRpcResult(id, "ok");
    }
    if (method == "printer.print.cancel") {
        std::string msg;
        if (!m_callbacks.cancelPrint || !m_callbacks.cancelPrint(msg))
            return makeJsonRpcError(id, -32000, msg.empty() ? "Unable to cancel print" : msg);
        return makeJsonRpcResult(id, "ok");
    }
    if (method == "access.oneshot_token") {
        return makeJsonRpcResult(id, "dev-token");
    }
    if (method == "server.files.list") {
        return makeJsonRpcResult(id, m_callbacks.listFiles ? m_callbacks.listFiles() : json::array());
    }
    if (method == "server.files.metadata") {
        std::string filename = params.value("filename", "");
        return makeJsonRpcResult(id, m_callbacks.getFileMetadata ? m_callbacks.getFileMetadata(filename) : json::object());
    }
    if (method == "server.files.roots") {
        return makeJsonRpcResult(id, {
            {"roots", json::array({{{"name", "gcodes"}, {"path", "gcode"}, {"permissions", "rw"}}})}
        });
    }
    if (method == "server.webcams.list") {
        return makeJsonRpcResult(id, {{"webcams", json::array()}});
    }
    if (method == "server.history.list") {
        return makeJsonRpcResult(id, {{"jobs", json::array()}, {"count", 0}});
    }
    if (method == "server.job_queue.status") {
        return makeJsonRpcResult(id, {{"queued_jobs", json::array()}, {"queue_state", "ready"}});
    }
    if (method == "server.extensions.list") {
        return makeJsonRpcResult(id, {{"extensions", json::array()}});
    }
    if (method == "machine.device_power.devices") {
        return makeJsonRpcResult(id, {{"devices", json::array()}});
    }
    if (method == "machine.proc_stats") {
        return makeJsonRpcResult(id, {
            {"system_cpu_usage", {{"cpu", 0.0}}},
            {"system_memory", {{"total", 0}, {"used", 0}, {"available", 0}}}
        });
    }
    if (method == "server.database.list") {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        json namespaces = json::array();
        std::map<std::string, bool> seen;
        for (const auto& kv : m_database) {
            std::string ns = kv.first.substr(0, kv.first.find(':'));
            if (!seen.emplace(ns, true).second)
                continue;
            namespaces.push_back(ns);
        }
        return makeJsonRpcResult(id, {{"namespaces", namespaces}});
    }
    if (method == "server.database.get_item") {
        std::string ns = params.value("namespace", "fluidd");
        std::string key = params.value("key", "");
        std::lock_guard<std::mutex> lock(m_stateMutex);
        json value = json::object();
        auto it = m_database.find(ns + ":" + key);
        if (it != m_database.end())
            value = it->second;
        return makeJsonRpcResult(id, {{"namespace", ns}, {"key", key}, {"value", value}});
    }
    if (method == "server.database.post_item") {
        std::string ns = params.value("namespace", "fluidd");
        std::string key = params.value("key", "");
        json value = params.contains("value") ? params["value"] : json::object();
        {
            std::lock_guard<std::mutex> lock(m_stateMutex);
            m_database[ns + ":" + key] = value;
        }
        return makeJsonRpcResult(id, {{"namespace", ns}, {"key", key}, {"value", value}});
    }

    return makeJsonRpcError(id, -32601, "Method not found: " + method);
}

void MoonrakerApiServer::handleWebSocketClient(const std::shared_ptr<ClientSession>& session) {
    uintptr_t handle = session->socketHandle.load(std::memory_order_acquire);
    if (handle == 0)
        return;

    SOCKET client = static_cast<SOCKET>(handle);

    sendJsonToClient(session,
        json{{"jsonrpc", "2.0"}, {"method", "notify_klippy_ready"}, {"params", json::array()}});

    while (m_running.load(std::memory_order_acquire)) {
        uint8_t opcode = 0;
        std::string payload;
        if (!recvWsFrame(client, opcode, payload))
            break;

        if (opcode == 0x8) {
            sendJsonToClient(session, json(), 0x8);
            break;
        }
        if (opcode == 0x9) {
            uintptr_t pingHandle = session->socketHandle.load(std::memory_order_acquire);
            if (pingHandle == 0)
                break;
            std::lock_guard<std::mutex> lock(session->sendMutex);
            if (!sendWsFrame(static_cast<SOCKET>(pingHandle), 0xA, payload)) {
                closeClientSocket(session);
                break;
            }
            continue;
        }
        if (opcode != 0x1)
            continue;

        try {
            json incoming = json::parse(payload);
            if (incoming.is_array()) {
                for (const auto& msg : incoming) {
                    bool sendNotify = false;
                    json notifyPayload;
                    std::string subscribedQuery;
                    json response = dispatchJsonRpc(msg, sendNotify, notifyPayload, subscribedQuery);
                    if (!subscribedQuery.empty() && notifyPayload.is_array() && notifyPayload.size() >= 1) {
                        std::lock_guard<std::mutex> lock(session->stateMutex);
                        session->subscriptionQuery = subscribedQuery;
                        session->lastStatus = notifyPayload[0];
                    }
                    if (msg.contains("id"))
                        sendJsonToClient(session, response);
                    if (sendNotify)
                        sendJsonToClient(session, {{"jsonrpc", "2.0"}, {"method", "notify_status_update"}, {"params", notifyPayload}});
                }
            } else {
                bool sendNotify = false;
                json notifyPayload;
                std::string subscribedQuery;
                json response = dispatchJsonRpc(incoming, sendNotify, notifyPayload, subscribedQuery);
                if (!subscribedQuery.empty() && notifyPayload.is_array() && notifyPayload.size() >= 1) {
                    std::lock_guard<std::mutex> lock(session->stateMutex);
                    session->subscriptionQuery = subscribedQuery;
                    session->lastStatus = notifyPayload[0];
                }
                if (incoming.contains("id"))
                    sendJsonToClient(session, response);
                if (sendNotify)
                    sendJsonToClient(session, {{"jsonrpc", "2.0"}, {"method", "notify_status_update"}, {"params", notifyPayload}});
            }
        } catch (...) {
            sendJsonToClient(session, makeJsonRpcError(nullptr, -32700, "Invalid JSON"));
        }
    }

    closeClientSocket(session);
}

void MoonrakerApiServer::handleClient(const std::shared_ptr<ClientSession>& session) {
    uintptr_t handle = session->socketHandle.load(std::memory_order_acquire);
    if (handle == 0) {
        session->finished.store(true, std::memory_order_release);
        return;
    }

    SOCKET client = static_cast<SOCKET>(handle);
    std::string raw = recvRequest(client);
    HttpRequest req;
    std::string out;

    if (!parseRequest(raw, req)) {
        out = jsonResponse(400, makeError(400, "Malformed HTTP request"));
        sendAll(client, out.data(), out.size());
        closeClientSocket(session);
        session->finished.store(true, std::memory_order_release);
        return;
    }

    auto upgradeIt = req.headers.find("upgrade");
    if (upgradeIt != req.headers.end() && toLower(upgradeIt->second) == "websocket") {
        auto keyIt = req.headers.find("sec-websocket-key");
        if (keyIt == req.headers.end()) {
            out = jsonResponse(400, makeError(400, "Missing Sec-WebSocket-Key"));
            sendAll(client, out.data(), out.size());
            closeClientSocket(session);
            session->finished.store(true, std::memory_order_release);
            return;
        }

        std::ostringstream hs;
        hs << "HTTP/1.1 101 Switching Protocols\r\n"
           << "Upgrade: websocket\r\n"
           << "Connection: Upgrade\r\n"
           << "Sec-WebSocket-Accept: " << websocketAcceptKey(keyIt->second) << "\r\n\r\n";
        if (!sendAll(client, hs.str().data(), hs.str().size())) {
            closeClientSocket(session);
            session->finished.store(true, std::memory_order_release);
            return;
        }
        session->websocket.store(true, std::memory_order_release);
        handleWebSocketClient(session);
        session->finished.store(true, std::memory_order_release);
        return;
    }

    json result;
    int code = 200;

    if (req.method == "OPTIONS") {
        code = 204;
        result = json::object();
    } else if (req.path == "/server/info") {
        result = json{{"result", m_callbacks.getServerInfo ? m_callbacks.getServerInfo() : json::object()}};
    } else if (req.path == "/server/config") {
        result = json{{"result", {{"host", "0.0.0.0"}, {"port", m_port}}}};
    } else if (req.path == "/server/version") {
        result = json{{"result", {{"moonraker_version", "cpp-host-dev"}, {"api_version", {1, 0, 0}}}}};
    } else if (req.path == "/printer/info") {
        result = json{{"result", m_callbacks.getPrinterInfo ? m_callbacks.getPrinterInfo() : json::object()}};
    } else if (req.path == "/machine/system_info") {
        result = json{{"result", m_callbacks.getSystemInfo ? m_callbacks.getSystemInfo() : json::object()}};
    } else if (req.path == "/machine/device_power/devices") {
        result = json{{"result", {{"devices", json::array()}}}};
    } else if (req.path == "/machine/proc_stats") {
        result = json{{"result", {{"system_cpu_usage", {{"cpu", 0.0}}}, {"system_memory", {{"total", 0}, {"used", 0}, {"available", 0}}}}}};
    } else if (req.path == "/printer/objects/list") {
        result = json{{"result", {{"objects", {"webhooks", "toolhead", "gcode_move", "motion_report", "print_stats", "extruder", "heater_bed", "virtual_sdcard", "configfile"}}}}};
    } else if (req.path == "/printer/objects/query") {
        if (req.body.empty()) {
            result = json{{"result", m_callbacks.queryObjects ? m_callbacks.queryObjects(req.query) : json::object()}};
        } else {
            try {
                auto j = json::parse(req.body);
                std::string query = objectsQueryStringFromJson(j.value("objects", json::object()));
                result = json{{"result", m_callbacks.queryObjects ? m_callbacks.queryObjects(query) : json::object()}};
            } catch (...) {
                code = 400;
                result = makeError(400, "Invalid JSON body");
            }
        }
    } else if (req.path == "/access/oneshot_token" || req.path == "/access/api_key") {
        result = json{{"result", "dev-token"}};
    } else if (req.path == "/server/files/roots") {
        result = json{{"result", {{"roots", json::array({{{"name", "gcodes"}, {"path", "gcode"}, {"permissions", "rw"}}})}}}};
    } else if (req.path == "/server/files/list") {
        result = json{{"result", m_callbacks.listFiles ? m_callbacks.listFiles() : json::array()}};
    } else if (req.path == "/server/files/metadata") {
        auto query = parseQuery(req.query);
        std::string filename = query.count("filename") ? query["filename"] : "";
        result = json{{"result", m_callbacks.getFileMetadata ? m_callbacks.getFileMetadata(filename) : json::object()}};
    } else if (req.path == "/server/history/list") {
        result = json{{"result", {{"jobs", json::array()}, {"count", 0}}}};
    } else if (req.path == "/server/job_queue/status") {
        result = json{{"result", {{"queued_jobs", json::array()}, {"queue_state", "ready"}}}};
    } else if (req.path == "/server/webcams/list") {
        result = json{{"result", {{"webcams", json::array()}}}};
    } else if (req.path == "/server/extensions/list") {
        result = json{{"result", {{"extensions", json::array()}}}};
    } else if (req.path == "/server/database/list") {
        std::lock_guard<std::mutex> lock(m_stateMutex);
        json namespaces = json::array();
        std::map<std::string, bool> seen;
        for (const auto& kv : m_database) {
            std::string ns = kv.first.substr(0, kv.first.find(':'));
            if (!seen.emplace(ns, true).second)
                continue;
            namespaces.push_back(ns);
        }
        result = json{{"result", {{"namespaces", namespaces}}}};
    } else if (req.path == "/server/database/item") {
        auto query = parseQuery(req.query);
        std::string ns = query.count("namespace") ? query["namespace"] : "fluidd";
        std::string key = query.count("key") ? query["key"] : "";
        if (req.method == "GET") {
            std::lock_guard<std::mutex> lock(m_stateMutex);
            json value = json::object();
            auto it = m_database.find(ns + ":" + key);
            if (it != m_database.end())
                value = it->second;
            result = json{{"result", {{"namespace", ns}, {"key", key}, {"value", value}}}};
        } else {
            try {
                auto j = req.body.empty() ? json::object() : json::parse(req.body);
                json value = j.contains("value") ? j["value"] : json::object();
                {
                    std::lock_guard<std::mutex> lock(m_stateMutex);
                    m_database[ns + ":" + key] = value;
                }
                result = json{{"result", {{"namespace", ns}, {"key", key}, {"value", value}}}};
            } catch (...) {
                code = 400;
                result = makeError(400, "Invalid JSON body");
            }
        }
    } else if (req.path == "/printer/gcode/script") {
        std::string script;
        auto query = parseQuery(req.query);
        auto it = query.find("script");
        if (it != query.end())
            script = it->second;
        if (script.empty() && !req.body.empty()) {
            try {
                auto j = json::parse(req.body);
                if (j.contains("script") && j["script"].is_string())
                    script = j["script"].get<std::string>();
            } catch (...) {
            }
        }

        if (script.empty()) {
            code = 400;
            result = makeError(400, "Missing gcode script");
        } else if (!m_callbacks.executeGcode) {
            code = 500;
            result = makeError(500, "No gcode executor registered");
        } else {
            std::string msg;
            bool ok = m_callbacks.executeGcode(script, msg);
            if (ok)
                result = json{{"result", "ok"}};
            else {
                code = 500;
                result = makeError(500, msg.empty() ? "G-code execution failed" : msg);
            }
        }
    } else if (req.path == "/printer/print/start") {
        std::string filename;
        auto query = parseQuery(req.query);
        auto it = query.find("filename");
        if (it != query.end())
            filename = it->second;
        if (filename.empty() && !req.body.empty()) {
            try {
                auto j = json::parse(req.body);
                if (j.contains("filename") && j["filename"].is_string())
                    filename = j["filename"].get<std::string>();
            } catch (...) {
            }
        }
        std::string msg;
        if (!m_callbacks.startPrint || !m_callbacks.startPrint(filename, msg)) {
            code = 500;
            result = makeError(500, msg.empty() ? "Unable to start print" : msg);
        } else {
            result = json{{"result", "ok"}};
        }
    } else if (req.path == "/printer/print/pause") {
        std::string msg;
        if (!m_callbacks.pausePrint || !m_callbacks.pausePrint(msg)) {
            code = 500;
            result = makeError(500, msg.empty() ? "Unable to pause print" : msg);
        } else {
            result = json{{"result", "ok"}};
        }
    } else if (req.path == "/printer/print/resume") {
        std::string msg;
        if (!m_callbacks.resumePrint || !m_callbacks.resumePrint(msg)) {
            code = 500;
            result = makeError(500, msg.empty() ? "Unable to resume print" : msg);
        } else {
            result = json{{"result", "ok"}};
        }
    } else if (req.path == "/printer/print/cancel") {
        std::string msg;
        if (!m_callbacks.cancelPrint || !m_callbacks.cancelPrint(msg)) {
            code = 500;
            result = makeError(500, msg.empty() ? "Unable to cancel print" : msg);
        } else {
            result = json{{"result", "ok"}};
        }
    } else {
        code = 404;
        result = makeError(404, "Endpoint not found");
    }

    out = jsonResponse(code, result);
    sendAll(client, out.data(), out.size());
    closeClientSocket(session);
    session->finished.store(true, std::memory_order_release);
}
