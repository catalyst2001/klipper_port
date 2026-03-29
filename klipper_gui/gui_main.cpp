#include <wx/wx.h>
#include <wx/listctrl.h>
#include <wx/timer.h>
#include <wx/textctrl.h>
#include <wx/splitter.h>
#include <wx/notebook.h>
#include <wx/stattext.h>
#include <wx/grid.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <sstream>

#include "../klipper_host/klipper_mcu.h"
#include "../klipper_host/mcu_objects.h"

#include <memory>

// ---- Log Message for passing between threads ----
struct LogEntry {
    wxString text;
    wxColour color;
};

// ---- Main Application ----
class KlipperGuiApp : public wxApp {
public:
    bool OnInit() override;
};

// ---- Main Frame ----
class KlipperFrame : public wxFrame {
public:
    KlipperFrame();
    ~KlipperFrame();

private:
    // MCU
    KlipperMCU m_mcu;
    std::atomic<bool> m_connected{false};
    std::atomic<bool> m_pollRunning{false};
    std::thread m_pollThread;
    std::mutex m_mcuMutex;

    // Thread-safe log queue
    std::mutex m_logMutex;
    std::queue<LogEntry> m_logQueue;

    // Clock sync polling
    std::atomic<bool> m_clockSyncRunning{false};
    std::thread m_clockSyncThread;

    // UI Controls
    wxTextCtrl* m_logText = nullptr;
    wxButton* m_btnConnect = nullptr;
    wxButton* m_btnIdentify = nullptr;
    wxButton* m_btnGetClock = nullptr;
    wxButton* m_btnGetUptime = nullptr;
    wxButton* m_btnGetConfig = nullptr;
    wxButton* m_btnSetPin = nullptr;
    wxButton* m_btnEmergencyStop = nullptr;
    wxButton* m_btnReset = nullptr;
    wxTextCtrl* m_pinNumCtrl = nullptr;
    wxChoice* m_pinValueCtrl = nullptr;
    wxStaticText* m_statusLabel = nullptr;
    wxStaticText* m_clockSyncLabel = nullptr;
    wxListCtrl* m_cmdList = nullptr;
    wxListCtrl* m_respList = nullptr;
    wxTimer m_uiTimer;

    // GPIO/ADC objects
    std::vector<std::unique_ptr<MCU_digital_out>> m_digitalOuts;
    std::vector<std::unique_ptr<MCU_adc>> m_adcInputs;
    struct AdcReading { double time = 0; double value = 0; };
    std::mutex m_adcMutex;
    std::vector<AdcReading> m_adcReadings;  // latest reading per ADC (indexed by order)

    // GPIO/ADC UI
    wxListCtrl* m_gpioList = nullptr;
    wxListCtrl* m_adcList = nullptr;
    wxTextCtrl* m_cfgPinCtrl = nullptr;
    wxButton* m_btnAddDigitalOut = nullptr;
    wxButton* m_btnAddAdc = nullptr;
    wxButton* m_btnFinalize = nullptr;
    wxTextCtrl* m_adcPinCtrl = nullptr;
    wxTextCtrl* m_adcReportCtrl = nullptr;

    // Pin management
    int m_nextOid = 0;

    void CreateUI();
    void Log(const wxString& msg, const wxColour& color = *wxBLACK);
    void LogFromThread(const wxString& msg, const wxColour& color = *wxBLACK);

    // Event handlers
    void OnConnect(wxCommandEvent& evt);
    void OnIdentify(wxCommandEvent& evt);
    void OnGetClock(wxCommandEvent& evt);
    void OnGetUptime(wxCommandEvent& evt);
    void OnGetConfig(wxCommandEvent& evt);
    void OnSetPin(wxCommandEvent& evt);
    void OnEmergencyStop(wxCommandEvent& evt);
    void OnReset(wxCommandEvent& evt);
    void OnUITimer(wxTimerEvent& evt);
    void OnClose(wxCloseEvent& evt);

    void StopPolling();
    void StartPolling();
    void PollThread();

    void StopClockSync();
    void StartClockSync();
    void ClockSyncThread();

    void PopulateCommandList();
    void PopulateResponseList();
    int ResolvePinNumber(const wxString& pinStr);
    void OnAddDigitalOut(wxCommandEvent& evt);
    void OnAddAdc(wxCommandEvent& evt);
    void OnFinalizeConfig(wxCommandEvent& evt);
    void OnToggleGpio(wxListEvent& evt);
};

// ---- App Implementation ----
wxIMPLEMENT_APP(KlipperGuiApp);

bool KlipperGuiApp::OnInit() {
    auto* frame = new KlipperFrame();
    frame->Show(true);
    return true;
}

// ---- Frame Implementation ----
enum {
    ID_CONNECT = wxID_HIGHEST + 1,
    ID_IDENTIFY,
    ID_GET_CLOCK,
    ID_GET_UPTIME,
    ID_GET_CONFIG,
    ID_SET_PIN,
    ID_EMERGENCY_STOP,
    ID_RESET,
    ID_UI_TIMER,
    ID_ADD_DIGITAL_OUT,
    ID_ADD_ADC,
    ID_FINALIZE_CONFIG,
    ID_GPIO_LIST,
};

KlipperFrame::KlipperFrame()
    : wxFrame(nullptr, wxID_ANY, "Klipper Host - Duet 3 6HC", wxDefaultPosition, wxSize(1100, 750)),
      m_uiTimer(this, ID_UI_TIMER)
{
    CreateUI();

    Bind(wxEVT_BUTTON, &KlipperFrame::OnConnect, this, ID_CONNECT);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnIdentify, this, ID_IDENTIFY);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnGetClock, this, ID_GET_CLOCK);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnGetUptime, this, ID_GET_UPTIME);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnGetConfig, this, ID_GET_CONFIG);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnSetPin, this, ID_SET_PIN);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnEmergencyStop, this, ID_EMERGENCY_STOP);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnReset, this, ID_RESET);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnAddDigitalOut, this, ID_ADD_DIGITAL_OUT);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnAddAdc, this, ID_ADD_ADC);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnFinalizeConfig, this, ID_FINALIZE_CONFIG);
    Bind(wxEVT_LIST_ITEM_ACTIVATED, &KlipperFrame::OnToggleGpio, this, ID_GPIO_LIST);
    Bind(wxEVT_TIMER, &KlipperFrame::OnUITimer, this, ID_UI_TIMER);
    Bind(wxEVT_CLOSE_WINDOW, &KlipperFrame::OnClose, this);

    m_uiTimer.Start(100); // 10 Hz UI update
    Log("Klipper Host C++ GUI started. Click 'Connect' to begin.", wxColour(0, 0, 160));
}

KlipperFrame::~KlipperFrame() {
    StopPolling();
}

void KlipperFrame::CreateUI() {
    auto* mainPanel = new wxPanel(this);
    auto* mainSizer = new wxBoxSizer(wxVERTICAL);

    // Status bar
    m_statusLabel = new wxStaticText(mainPanel, wxID_ANY, "Status: Disconnected");
    m_statusLabel->SetForegroundColour(*wxRED);
    auto statusFont = m_statusLabel->GetFont();
    statusFont.SetWeight(wxFONTWEIGHT_BOLD);
    m_statusLabel->SetFont(statusFont);
    mainSizer->Add(m_statusLabel, 0, wxALL | wxEXPAND, 5);

    // Clock sync status
    m_clockSyncLabel = new wxStaticText(mainPanel, wxID_ANY, "Clock Sync: inactive");
    m_clockSyncLabel->SetForegroundColour(wxColour(80, 80, 80));
    m_clockSyncLabel->SetFont(wxFont(8, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL));
    mainSizer->Add(m_clockSyncLabel, 0, wxLEFT | wxRIGHT | wxBOTTOM | wxEXPAND, 5);

    // Top toolbar
    auto* toolSizer = new wxBoxSizer(wxHORIZONTAL);
    m_btnConnect = new wxButton(mainPanel, ID_CONNECT, "Connect COM3");
    m_btnIdentify = new wxButton(mainPanel, ID_IDENTIFY, "Identify");
    m_btnGetClock = new wxButton(mainPanel, ID_GET_CLOCK, "Get Clock");
    m_btnGetUptime = new wxButton(mainPanel, ID_GET_UPTIME, "Get Uptime");
    m_btnGetConfig = new wxButton(mainPanel, ID_GET_CONFIG, "Get Config");
    m_btnEmergencyStop = new wxButton(mainPanel, ID_EMERGENCY_STOP, "EMERGENCY STOP");
    m_btnEmergencyStop->SetBackgroundColour(*wxRED);
    m_btnEmergencyStop->SetForegroundColour(*wxWHITE);
    m_btnReset = new wxButton(mainPanel, ID_RESET, "Reset MCU");

    toolSizer->Add(m_btnConnect, 0, wxALL, 3);
    toolSizer->Add(m_btnIdentify, 0, wxALL, 3);
    toolSizer->Add(m_btnGetClock, 0, wxALL, 3);
    toolSizer->Add(m_btnGetUptime, 0, wxALL, 3);
    toolSizer->Add(m_btnGetConfig, 0, wxALL, 3);
    toolSizer->AddStretchSpacer();
    toolSizer->Add(m_btnReset, 0, wxALL, 3);
    toolSizer->Add(m_btnEmergencyStop, 0, wxALL, 3);
    mainSizer->Add(toolSizer, 0, wxEXPAND);

    // Pin control row
    auto* pinSizer = new wxBoxSizer(wxHORIZONTAL);
    pinSizer->Add(new wxStaticText(mainPanel, wxID_ANY, "  Pin:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_pinNumCtrl = new wxTextCtrl(mainPanel, wxID_ANY, "PD0", wxDefaultPosition, wxSize(80, -1));
    pinSizer->Add(m_pinNumCtrl, 0, wxALL, 3);
    pinSizer->Add(new wxStaticText(mainPanel, wxID_ANY, "Value:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_pinValueCtrl = new wxChoice(mainPanel, wxID_ANY);
    m_pinValueCtrl->Append("LOW (0)");
    m_pinValueCtrl->Append("HIGH (1)");
    m_pinValueCtrl->SetSelection(1);
    pinSizer->Add(m_pinValueCtrl, 0, wxALL, 3);
    m_btnSetPin = new wxButton(mainPanel, ID_SET_PIN, "Set Pin");
    pinSizer->Add(m_btnSetPin, 0, wxALL, 3);
    mainSizer->Add(pinSizer, 0, wxEXPAND);

    // Splitter: top = notebook (commands/responses), bottom = log
    auto* splitter = new wxSplitterWindow(mainPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxSP_3D);
    auto* notebook = new wxNotebook(splitter, wxID_ANY);

    // Commands tab
    auto* cmdPanel = new wxPanel(notebook);
    auto* cmdSizer = new wxBoxSizer(wxVERTICAL);
    m_cmdList = new wxListCtrl(cmdPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT);
    m_cmdList->AppendColumn("ID", wxLIST_FORMAT_LEFT, 50);
    m_cmdList->AppendColumn("Name", wxLIST_FORMAT_LEFT, 200);
    m_cmdList->AppendColumn("Format", wxLIST_FORMAT_LEFT, 500);
    cmdSizer->Add(m_cmdList, 1, wxEXPAND | wxALL, 2);
    cmdPanel->SetSizer(cmdSizer);
    notebook->AddPage(cmdPanel, "Commands");

    // Responses tab
    auto* respPanel = new wxPanel(notebook);
    auto* respSizer = new wxBoxSizer(wxVERTICAL);
    m_respList = new wxListCtrl(respPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT);
    m_respList->AppendColumn("ID", wxLIST_FORMAT_LEFT, 50);
    m_respList->AppendColumn("Name", wxLIST_FORMAT_LEFT, 200);
    m_respList->AppendColumn("Format", wxLIST_FORMAT_LEFT, 500);
    respSizer->Add(m_respList, 1, wxEXPAND | wxALL, 2);
    respPanel->SetSizer(respSizer);
    notebook->AddPage(respPanel, "Responses");

    // GPIO tab
    auto* gpioPanel = new wxPanel(notebook);
    auto* gpioSizer = new wxBoxSizer(wxVERTICAL);
    auto* gpioAddSizer = new wxBoxSizer(wxHORIZONTAL);
    gpioAddSizer->Add(new wxStaticText(gpioPanel, wxID_ANY, "Pin:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_cfgPinCtrl = new wxTextCtrl(gpioPanel, wxID_ANY, "PD0", wxDefaultPosition, wxSize(80, -1));
    gpioAddSizer->Add(m_cfgPinCtrl, 0, wxALL, 3);
    m_btnAddDigitalOut = new wxButton(gpioPanel, ID_ADD_DIGITAL_OUT, "Add Digital Out");
    gpioAddSizer->Add(m_btnAddDigitalOut, 0, wxALL, 3);
    m_btnFinalize = new wxButton(gpioPanel, ID_FINALIZE_CONFIG, "Finalize Config");
    gpioAddSizer->Add(m_btnFinalize, 0, wxALL, 3);
    gpioSizer->Add(gpioAddSizer, 0, wxEXPAND);
    m_gpioList = new wxListCtrl(gpioPanel, ID_GPIO_LIST, wxDefaultPosition, wxDefaultSize, wxLC_REPORT);
    m_gpioList->AppendColumn("OID", wxLIST_FORMAT_LEFT, 50);
    m_gpioList->AppendColumn("Pin", wxLIST_FORMAT_LEFT, 100);
    m_gpioList->AppendColumn("State", wxLIST_FORMAT_LEFT, 80);
    m_gpioList->AppendColumn("Status", wxLIST_FORMAT_LEFT, 200);
    gpioSizer->Add(m_gpioList, 1, wxEXPAND | wxALL, 2);
    gpioPanel->SetSizer(gpioSizer);
    notebook->AddPage(gpioPanel, "GPIO");

    // ADC tab
    auto* adcPanel = new wxPanel(notebook);
    auto* adcSizer = new wxBoxSizer(wxVERTICAL);
    auto* adcAddSizer = new wxBoxSizer(wxHORIZONTAL);
    adcAddSizer->Add(new wxStaticText(adcPanel, wxID_ANY, "Pin:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_adcPinCtrl = new wxTextCtrl(adcPanel, wxID_ANY, "ADC_TEMPERATURE", wxDefaultPosition, wxSize(140, -1));
    adcAddSizer->Add(m_adcPinCtrl, 0, wxALL, 3);
    adcAddSizer->Add(new wxStaticText(adcPanel, wxID_ANY, "Report(s):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_adcReportCtrl = new wxTextCtrl(adcPanel, wxID_ANY, "0.5", wxDefaultPosition, wxSize(60, -1));
    adcAddSizer->Add(m_adcReportCtrl, 0, wxALL, 3);
    m_btnAddAdc = new wxButton(adcPanel, ID_ADD_ADC, "Add ADC");
    adcAddSizer->Add(m_btnAddAdc, 0, wxALL, 3);
    adcSizer->Add(adcAddSizer, 0, wxEXPAND);
    m_adcList = new wxListCtrl(adcPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT);
    m_adcList->AppendColumn("OID", wxLIST_FORMAT_LEFT, 50);
    m_adcList->AppendColumn("Pin", wxLIST_FORMAT_LEFT, 140);
    m_adcList->AppendColumn("Value", wxLIST_FORMAT_LEFT, 100);
    m_adcList->AppendColumn("Voltage", wxLIST_FORMAT_LEFT, 100);
    m_adcList->AppendColumn("Last Read", wxLIST_FORMAT_LEFT, 150);
    adcSizer->Add(m_adcList, 1, wxEXPAND | wxALL, 2);
    adcPanel->SetSizer(adcSizer);
    notebook->AddPage(adcPanel, "ADC");

    // Log panel
    m_logText = new wxTextCtrl(splitter, wxID_ANY, "", wxDefaultPosition, wxDefaultSize,
        wxTE_MULTILINE | wxTE_READONLY | wxTE_RICH2 | wxHSCROLL);
    auto logFont = wxFont(9, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL);
    m_logText->SetFont(logFont);

    splitter->SplitHorizontally(notebook, m_logText, 300);
    splitter->SetMinimumPaneSize(100);

    mainSizer->Add(splitter, 1, wxEXPAND | wxALL, 3);

    mainPanel->SetSizer(mainSizer);

    // Initial button states
    m_btnIdentify->Enable(false);
    m_btnGetClock->Enable(false);
    m_btnGetUptime->Enable(false);
    m_btnGetConfig->Enable(false);
    m_btnSetPin->Enable(false);
    m_btnEmergencyStop->Enable(false);
    m_btnReset->Enable(false);
    m_btnAddDigitalOut->Enable(false);
    m_btnAddAdc->Enable(false);
    m_btnFinalize->Enable(false);
}

void KlipperFrame::Log(const wxString& msg, const wxColour& color) {
    auto now = wxDateTime::Now();
    wxString timestamp = now.Format("[%H:%M:%S] ");

    m_logText->SetDefaultStyle(wxTextAttr(wxColour(128, 128, 128)));
    m_logText->AppendText(timestamp);
    m_logText->SetDefaultStyle(wxTextAttr(color));
    m_logText->AppendText(msg + "\n");
    m_logText->ShowPosition(m_logText->GetLastPosition());
}

void KlipperFrame::LogFromThread(const wxString& msg, const wxColour& color) {
    std::lock_guard<std::mutex> lock(m_logMutex);
    m_logQueue.push({msg, color});
}

void KlipperFrame::OnUITimer(wxTimerEvent&) {
    // Process log messages from other threads
    {
        std::lock_guard<std::mutex> lock(m_logMutex);
        while (!m_logQueue.empty()) {
            auto& entry = m_logQueue.front();
            Log(entry.text, entry.color);
            m_logQueue.pop();
        }
    }

    // Update clock sync display
    if (m_connected && m_clockSyncRunning) {
        auto info = m_mcu.getClockSync().getDebugInfo();
        m_clockSyncLabel->SetLabel(wxString::Format(
            "Clock Sync: freq=%.0f Hz | RTT=%.3f ms | clock=%lld | var=%.1f",
            info.freq, info.minHalfRtt * 2000.0, info.lastClock, info.predictionVariance));
        m_clockSyncLabel->SetForegroundColour(wxColour(0, 100, 0));
    } else if (!m_connected) {
        m_clockSyncLabel->SetLabel("Clock Sync: inactive");
        m_clockSyncLabel->SetForegroundColour(wxColour(80, 80, 80));
    }

    // Check shutdown state
    if (m_connected && m_mcu.isShutdown()) {
        m_statusLabel->SetLabel(wxString::Format("Status: MCU SHUTDOWN - %s", m_mcu.getShutdownMsg()));
        m_statusLabel->SetForegroundColour(*wxRED);
    }

    // Update ADC readings display
    {
        std::lock_guard<std::mutex> lock(m_adcMutex);
        for (size_t i = 0; i < m_adcReadings.size() && i < static_cast<size_t>(m_adcList->GetItemCount()); i++) {
            auto& r = m_adcReadings[i];
            if (r.time > 0) {
                m_adcList->SetItem(static_cast<int>(i), 2, wxString::Format("%.4f", r.value));
                m_adcList->SetItem(static_cast<int>(i), 3, wxString::Format("%.3f V", r.value * 3.3));
                m_adcList->SetItem(static_cast<int>(i), 4, wxString::Format("t=%.3f", r.time));
            }
        }
    }
}

void KlipperFrame::OnConnect(wxCommandEvent&) {
    if (m_connected) {
        StopClockSync();
        StopPolling();
        m_mcu.disconnect();
        m_connected = false;
        m_statusLabel->SetLabel("Status: Disconnected");
        m_statusLabel->SetForegroundColour(*wxRED);
        m_btnConnect->SetLabel("Connect COM3");
        m_btnIdentify->Enable(false);
        m_btnGetClock->Enable(false);
        m_btnGetUptime->Enable(false);
        m_btnGetConfig->Enable(false);
        m_btnSetPin->Enable(false);
        m_btnEmergencyStop->Enable(false);
        m_btnReset->Enable(false);
        m_btnAddDigitalOut->Enable(false);
        m_btnAddAdc->Enable(false);
        m_btnFinalize->Enable(false);
        m_cmdList->DeleteAllItems();
        m_respList->DeleteAllItems();
        m_gpioList->DeleteAllItems();
        m_adcList->DeleteAllItems();
        m_digitalOuts.clear();
        m_adcInputs.clear();
        m_adcReadings.clear();
        Log("Disconnected.", *wxRED);
    }
    else {
        Log("Connecting to COM3...");
        if (m_mcu.connect("COM3", 250000)) {
            m_connected = true;
            m_statusLabel->SetLabel("Status: Connected (not identified)");
            m_statusLabel->SetForegroundColour(wxColour(200, 150, 0));
            m_btnConnect->SetLabel("Disconnect");
            m_btnIdentify->Enable(true);
            Log("Connected to COM3!", wxColour(0, 128, 0));
        }
        else {
            Log("Failed to connect: " + wxString(m_mcu.getLastError()), *wxRED);
        }
    }
}

void KlipperFrame::OnIdentify(wxCommandEvent&) {
    if (!m_connected) return;
    Log("Running identify handshake...");

    wxBusyCursor wait;
    if (m_mcu.identify()) {
        m_statusLabel->SetLabel(wxString::Format("Status: Connected - %s | MCU: %s",
            m_mcu.getVersion(), wxString(m_mcu.getConfig().count("MCU") ? "" : "unknown")));
        m_statusLabel->SetForegroundColour(wxColour(0, 128, 0));

        Log(wxString::Format("Identified! Version: %s", m_mcu.getVersion()), wxColour(0, 128, 0));
        Log(wxString::Format("Build: %s", m_mcu.getBuildVersions()));
        Log(wxString::Format("Commands: %zu, Responses: %zu",
            m_mcu.getCommands().size(), m_mcu.getResponses().size()));

        // Show config
        for (auto& [key, val] : m_mcu.getConfig()) {
            Log(wxString::Format("  Config: %s = %d", key, val), wxColour(80, 80, 80));
        }

        PopulateCommandList();
        PopulateResponseList();

        m_btnGetClock->Enable(true);
        m_btnGetUptime->Enable(true);
        m_btnGetConfig->Enable(true);
        m_btnSetPin->Enable(true);
        m_btnEmergencyStop->Enable(true);
        m_btnReset->Enable(true);
        m_btnAddDigitalOut->Enable(true);
        m_btnAddAdc->Enable(true);
        m_btnFinalize->Enable(true);

        StartPolling();

        // Initialize clock sync
        Log("Initializing clock synchronization...");
        if (m_mcu.initClockSync()) {
            auto info = m_mcu.getClockSync().getDebugInfo();
            Log(wxString::Format("Clock sync initialized: freq=%.0f Hz, RTT=%.3f ms",
                info.freq, info.minHalfRtt * 2000.0), wxColour(0, 128, 0));
            StartClockSync();
        } else {
            Log("Clock sync init failed: " + wxString(m_mcu.getLastError()), *wxRED);
        }

        // Register shutdown callback
        m_mcu.setShutdownCallback([this](const std::string& reason) {
            LogFromThread(wxString::Format("!!! MCU SHUTDOWN: %s !!!", reason), *wxRED);
        });
    }
    else {
        Log("Identify failed: " + wxString(m_mcu.getLastError()), *wxRED);
    }
}

void KlipperFrame::OnGetClock(wxCommandEvent&) {
    if (!m_connected) return;

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    std::map<std::string, int64_t> intP;
    std::map<std::string, std::vector<uint8_t>> bufP;
    if (m_mcu.sendWithResponse("get_clock", "clock", intP, bufP)) {
        uint32_t clock = static_cast<uint32_t>(intP["clock"]);
        Log(wxString::Format("MCU Clock: %u (%.3f seconds @ 300MHz)",
            clock, static_cast<double>(clock) / 300000000.0), wxColour(0, 0, 160));
    }
    else {
        Log("get_clock failed: " + wxString(m_mcu.getLastError()), *wxRED);
    }
}

void KlipperFrame::OnGetUptime(wxCommandEvent&) {
    if (!m_connected) return;

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    std::map<std::string, int64_t> intP;
    std::map<std::string, std::vector<uint8_t>> bufP;
    if (m_mcu.sendWithResponse("get_uptime", "uptime", intP, bufP)) {
        uint32_t high = static_cast<uint32_t>(intP["high"]);
        uint32_t clk = static_cast<uint32_t>(intP["clock"]);
        uint64_t total = (static_cast<uint64_t>(high) << 32) | clk;
        double secs = static_cast<double>(total) / 300000000.0;
        Log(wxString::Format("Uptime: %.1f seconds (%.2f hours)", secs, secs / 3600.0), wxColour(0, 0, 160));
    }
    else {
        Log("get_uptime failed: " + wxString(m_mcu.getLastError()), *wxRED);
    }
}

void KlipperFrame::OnGetConfig(wxCommandEvent&) {
    if (!m_connected) return;

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    std::map<std::string, int64_t> intP;
    std::map<std::string, std::vector<uint8_t>> bufP;
    if (m_mcu.sendWithResponse("get_config", "config", intP, bufP)) {
        Log(wxString::Format("Config: is_config=%lld crc=%u is_shutdown=%lld move_count=%lld",
            intP["is_config"], static_cast<uint32_t>(intP["crc"]),
            intP["is_shutdown"], intP["move_count"]), wxColour(0, 0, 160));
    }
    else {
        Log("get_config failed: " + wxString(m_mcu.getLastError()), *wxRED);
    }
}

int KlipperFrame::ResolvePinNumber(const wxString& pinStr) {
    // Try direct number
    long pinNum;
    if (pinStr.ToLong(&pinNum)) {
        return static_cast<int>(pinNum);
    }

    // Resolve from enumerations (e.g., "PA0" -> 0, "PD0" -> 96)
    auto& enums = m_mcu.getEnumerations();
    auto pinIt = enums.find("pin");
    if (pinIt != enums.end()) {
        std::string pinName = pinStr.ToStdString();
        // Check exact match first
        auto it = pinIt->second.find(pinName);
        if (it != pinIt->second.end()) {
            return it->second.value;
        }

        // Check range match: e.g., "PD5" -> find "PD0" with range [96, 32], return 96+5=101
        if (pinName.size() >= 3 && pinName[0] == 'P') {
            char port = pinName[1];
            std::string numStr = pinName.substr(2);
            long offset;
            if (wxString(numStr).ToLong(&offset)) {
                std::string basePin = std::string("P") + port + "0";
                auto baseIt = pinIt->second.find(basePin);
                if (baseIt != pinIt->second.end() && baseIt->second.isRange()) {
                    if (offset < baseIt->second.count) {
                        return baseIt->second.value + static_cast<int>(offset);
                    }
                }
            }
        }
    }

    return -1; // Not found
}

void KlipperFrame::OnSetPin(wxCommandEvent&) {
    if (!m_connected) return;

    wxString pinStr = m_pinNumCtrl->GetValue();
    int pinNum = ResolvePinNumber(pinStr);
    if (pinNum < 0) {
        Log("Invalid pin: " + pinStr, *wxRED);
        return;
    }

    int value = m_pinValueCtrl->GetSelection(); // 0 = LOW, 1 = HIGH

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    // Use set_digital_out for immediate pin control (no OID needed)
    std::map<std::string, int64_t> params = {{"pin", pinNum}, {"value", value}};
    if (m_mcu.sendCommand("set_digital_out", params)) {
        Log(wxString::Format("Set pin %s (#%d) = %s", pinStr, pinNum, value ? "HIGH" : "LOW"),
            wxColour(0, 128, 0));
    }
    else {
        Log("set_digital_out failed: " + wxString(m_mcu.getLastError()), *wxRED);
    }
}

void KlipperFrame::OnEmergencyStop(wxCommandEvent&) {
    if (!m_connected) return;

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (m_mcu.sendCommand("emergency_stop")) {
        Log("!!! EMERGENCY STOP SENT !!!", *wxRED);
        m_statusLabel->SetLabel("Status: EMERGENCY STOP");
        m_statusLabel->SetForegroundColour(*wxRED);
    }
    else {
        Log("emergency_stop failed: " + wxString(m_mcu.getLastError()), *wxRED);
    }
}

void KlipperFrame::OnReset(wxCommandEvent&) {
    if (!m_connected) return;

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (m_mcu.sendCommand("reset")) {
        Log("Reset command sent to MCU", wxColour(200, 100, 0));
    }
    else {
        Log("reset failed: " + wxString(m_mcu.getLastError()), *wxRED);
    }
}

void KlipperFrame::PopulateCommandList() {
    m_cmdList->DeleteAllItems();
    int row = 0;
    for (auto& [name, fmt] : m_mcu.getCommands()) {
        m_cmdList->InsertItem(row, wxString::Format("%d", fmt.msgId));
        m_cmdList->SetItem(row, 1, fmt.name);
        m_cmdList->SetItem(row, 2, fmt.formatStr);
        row++;
    }
}

void KlipperFrame::PopulateResponseList() {
    m_respList->DeleteAllItems();
    int row = 0;
    for (auto& [name, fmt] : m_mcu.getResponses()) {
        m_respList->InsertItem(row, wxString::Format("%d", fmt.msgId));
        m_respList->SetItem(row, 1, fmt.name);
        m_respList->SetItem(row, 2, fmt.formatStr);
        row++;
    }
}

void KlipperFrame::StartPolling() {
    if (m_pollRunning) return;
    m_pollRunning = true;
    m_pollThread = std::thread(&KlipperFrame::PollThread, this);
}

void KlipperFrame::StopPolling() {
    m_pollRunning = false;
    if (m_pollThread.joinable()) {
        m_pollThread.join();
    }
}

void KlipperFrame::PollThread() {
    while (m_pollRunning && m_connected) {
        std::lock_guard<std::mutex> lock(m_mcuMutex);
        auto responses = m_mcu.processIncoming(50);
        for (auto& resp : responses) {
            std::ostringstream ss;
            ss << "<< [" << resp.msgId << "] " << resp.name;
            for (auto& [k, v] : resp.intParams) {
                ss << " " << k << "=" << v;
            }
            for (auto& [k, v] : resp.bufParams) {
                ss << " " << k << "=[" << v.size() << " bytes]";
            }
            LogFromThread(wxString(ss.str()), wxColour(100, 0, 100));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void KlipperFrame::StartClockSync() {
    if (m_clockSyncRunning) return;
    m_clockSyncRunning = true;
    m_clockSyncThread = std::thread(&KlipperFrame::ClockSyncThread, this);
}

void KlipperFrame::StopClockSync() {
    m_clockSyncRunning = false;
    if (m_clockSyncThread.joinable()) {
        m_clockSyncThread.join();
    }
}

void KlipperFrame::ClockSyncThread() {
    while (m_clockSyncRunning && m_connected) {
        {
            std::lock_guard<std::mutex> lock(m_mcuMutex);
            m_mcu.clockSyncPoll();
        }
        // Poll at ~1 Hz (matching Klipper's QUERY_FREQ)
        std::this_thread::sleep_for(std::chrono::milliseconds(984));
    }
}

void KlipperFrame::OnAddDigitalOut(wxCommandEvent&) {
    if (!m_connected) return;
    if (m_mcu.isConfigFinalized()) {
        Log("Config already finalized. Disconnect and reconnect to reconfigure.", *wxRED);
        return;
    }

    wxString pinStr = m_cfgPinCtrl->GetValue().Trim();
    if (pinStr.empty()) return;

    auto dout = std::make_unique<MCU_digital_out>(m_mcu);
    dout->setupPin(pinStr.ToStdString(), false);
    dout->setupMaxDuration(0.0);
    dout->setupStartValue(false, false);

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (dout->buildConfig()) {
        int row = m_gpioList->GetItemCount();
        m_gpioList->InsertItem(row, wxString::Format("%d", dout->getOid()));
        m_gpioList->SetItem(row, 1, pinStr);
        m_gpioList->SetItem(row, 2, "LOW");
        m_gpioList->SetItem(row, 3, "pending finalize");

        Log(wxString::Format("Added digital out: OID=%d pin=%s", dout->getOid(), pinStr),
            wxColour(0, 128, 0));
        m_digitalOuts.push_back(std::move(dout));
    } else {
        Log(wxString::Format("Failed to add digital out for pin %s", pinStr), *wxRED);
    }
}

void KlipperFrame::OnAddAdc(wxCommandEvent&) {
    if (!m_connected) return;
    if (m_mcu.isConfigFinalized()) {
        Log("Config already finalized. Disconnect and reconnect to reconfigure.", *wxRED);
        return;
    }

    wxString pinStr = m_adcPinCtrl->GetValue().Trim();
    if (pinStr.empty()) return;

    double reportTime = 0.5;
    m_adcReportCtrl->GetValue().ToDouble(&reportTime);
    if (reportTime < 0.01) reportTime = 0.01;

    auto adc = std::make_unique<MCU_adc>(m_mcu);
    adc->setupPin(pinStr.ToStdString());
    adc->setupAdcSample(reportTime, 0.001, 8, 0.0, 1.0, 0);

    size_t adcIdx = m_adcInputs.size();
    {
        std::lock_guard<std::mutex> lock(m_adcMutex);
        m_adcReadings.push_back({0.0, 0.0});
    }

    adc->setupAdcCallback([this, adcIdx](double readTime, double value) {
        std::lock_guard<std::mutex> lock(m_adcMutex);
        if (adcIdx < m_adcReadings.size()) {
            m_adcReadings[adcIdx] = {readTime, value};
        }
    });

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (adc->buildConfig()) {
        int row = m_adcList->GetItemCount();
        m_adcList->InsertItem(row, wxString::Format("%d", adc->getOid()));
        m_adcList->SetItem(row, 1, pinStr);
        m_adcList->SetItem(row, 2, "---");
        m_adcList->SetItem(row, 3, "---");
        m_adcList->SetItem(row, 4, "pending finalize");

        Log(wxString::Format("Added ADC: OID=%d pin=%s report=%.2fs", adc->getOid(), pinStr, reportTime),
            wxColour(0, 128, 0));
        m_adcInputs.push_back(std::move(adc));
    } else {
        Log(wxString::Format("Failed to add ADC for pin %s", pinStr), *wxRED);
        std::lock_guard<std::mutex> adcLock(m_adcMutex);
        m_adcReadings.pop_back();
    }
}

void KlipperFrame::OnFinalizeConfig(wxCommandEvent&) {
    if (!m_connected) return;
    if (m_mcu.isConfigFinalized()) {
        Log("Config already finalized.", wxColour(200, 100, 0));
        return;
    }

    if (m_digitalOuts.empty() && m_adcInputs.empty()) {
        Log("No GPIO or ADC objects configured. Add some first.", wxColour(200, 100, 0));
        return;
    }

    Log("Finalizing MCU configuration...");
    wxBusyCursor wait;

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (m_mcu.finalizeConfig()) {
        Log(wxString::Format("Config finalized! OIDs=%d", m_mcu.getOidCount()), wxColour(0, 128, 0));

        // Update GPIO list status
        for (int i = 0; i < m_gpioList->GetItemCount(); i++) {
            m_gpioList->SetItem(i, 3, "active");
        }
        // Update ADC list status
        for (int i = 0; i < m_adcList->GetItemCount(); i++) {
            m_adcList->SetItem(i, 4, "active");
        }

        m_btnAddDigitalOut->Enable(false);
        m_btnAddAdc->Enable(false);
        m_btnFinalize->Enable(false);
    } else {
        Log("Config finalization failed: " + wxString(m_mcu.getLastError()), *wxRED);
    }
}

void KlipperFrame::OnToggleGpio(wxListEvent& evt) {
    if (!m_connected || !m_mcu.isConfigFinalized()) return;

    int idx = evt.GetIndex();
    if (idx < 0 || idx >= static_cast<int>(m_digitalOuts.size())) return;

    auto& dout = m_digitalOuts[idx];
    bool newVal = !dout->getLastValue();

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    double printTime = m_mcu.getClockSync().estimatedPrintTime();
    if (dout->setDigital(printTime + 0.05, newVal)) {
        m_gpioList->SetItem(idx, 2, newVal ? "HIGH" : "LOW");
        Log(wxString::Format("GPIO OID=%d -> %s", dout->getOid(), newVal ? "HIGH" : "LOW"),
            wxColour(0, 128, 0));
    } else {
        Log(wxString::Format("Failed to toggle GPIO OID=%d", dout->getOid()), *wxRED);
    }
}

void KlipperFrame::OnClose(wxCloseEvent& evt) {
    StopClockSync();
    StopPolling();
    m_mcu.disconnect();
    evt.Skip();
}
