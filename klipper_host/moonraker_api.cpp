#include "moonraker_api.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <map>
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
} // namespace

MoonrakerApiServer::MoonrakerApiServer(MoonrakerApiCallbacks callbacks)
    : m_callbacks(std::move(callbacks)) {}

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
    m_thread = std::thread(&MoonrakerApiServer::acceptLoop, this);
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

    if (m_thread.joinable())
        m_thread.join();

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
        handleClient(static_cast<uintptr_t>(client));
    }
}

void MoonrakerApiServer::handleClient(uintptr_t clientHandle) {
    SOCKET client = static_cast<SOCKET>(clientHandle);
    std::string raw = recvRequest(client);
    HttpRequest req;
    std::string out;

    if (!parseRequest(raw, req)) {
        out = jsonResponse(400, makeError(400, "Malformed HTTP request"));
        send(client, out.c_str(), static_cast<int>(out.size()), 0);
        shutdown(client, SD_BOTH);
        closesocket(client);
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
    } else if (req.path == "/printer/objects/list") {
        result = json{{"result", {{"objects", {"webhooks", "toolhead", "gcode_move", "motion_report", "print_stats", "extruder", "heater_bed", "virtual_sdcard", "configfile"}}}}};
    } else if (req.path == "/printer/objects/query") {
        result = json{{"result", m_callbacks.queryObjects ? m_callbacks.queryObjects(req.query) : json::object()}};
    } else if (req.path == "/access/oneshot_token") {
        result = json{{"result", "dev-token"}};
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
    } else {
        code = 404;
        result = makeError(404, "Endpoint not found");
    }

    out = jsonResponse(code, result);
    send(client, out.c_str(), static_cast<int>(out.size()), 0);
    shutdown(client, SD_BOTH);
    closesocket(client);
}
