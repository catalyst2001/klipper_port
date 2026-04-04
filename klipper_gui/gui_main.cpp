#include <wx/wx.h>
#include <wx/listctrl.h>
#include <wx/timer.h>
#include <wx/textctrl.h>
#include <wx/splitter.h>
#include <wx/notebook.h>
#include <wx/stattext.h>
#include <wx/spinctrl.h>
#include <wx/grid.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <queue>
#include <sstream>
#include <fstream>

#include "../klipper_host/klipper_mcu.h"
#include "../klipper_host/mcu_objects.h"
#include "../klipper_host/bus_objects.h"
#include "../klipper_host/stepper.h"
#include "../klipper_host/toolhead.h"
#include "../klipper_host/gcode.h"
#include "../klipper_host/klipper_config.h"
#include "../klipper_host/tmc5160.h"

#include <wx/filedlg.h>
#include <wx/filename.h>
#include <wx/gauge.h>

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

    // SPI/I2C/Thermocouple objects
    std::vector<std::unique_ptr<MCU_SPI>> m_spiDevices;
    std::vector<std::unique_ptr<MCU_I2C>> m_i2cDevices;
    std::vector<std::unique_ptr<MCU_Thermocouple>> m_thermocouples;
    struct TcReading { double temp = 0; uint8_t fault = 0; };

    // TMC5160 driver objects
    std::vector<std::unique_ptr<TMC5160>> m_tmcDrivers;
    int m_tmcPollCounter = 0;
    bool m_tmcUnpoweredLogged = false;  // avoid spamming "VMot off"

    // TMC5160 register UI: per-driver checkboxes and labels
    wxNotebook* m_tmcNotebook = nullptr;  // sub-notebook for driver tabs

    struct TmcBitCheckbox {
        wxCheckBox* cb;
        uint8_t reg;   // register address
        uint8_t bit;   // bit position
        bool writable;
    };
    struct TmcFieldLabel {
        wxStaticText* label;  // read-only display
        uint8_t reg;
        uint8_t bitLow;  // low bit
        uint8_t bitHigh; // high bit (inclusive)
    };
    struct TmcFieldSpin {
        wxSpinCtrl* spin;     // writable control
        uint8_t reg;
        uint8_t bitLow;
        uint8_t bitHigh;
    };
    struct TmcDriverUI {
        std::vector<TmcBitCheckbox> bits;
        std::vector<TmcFieldLabel> fields;      // read-only fields
        std::vector<TmcFieldSpin> writeFields;  // writable fields
        wxStaticText* statusLabel = nullptr;
    };
    std::vector<TmcDriverUI> m_tmcUI;
    std::mutex m_tcMutex;
    std::vector<TcReading> m_tcReadings;

    // Bus UI
    wxListCtrl* m_busList = nullptr;
    wxTextCtrl* m_spiCsCtrl = nullptr;
    wxTextCtrl* m_spiBusCtrl = nullptr;
    wxButton* m_btnAddSpi = nullptr;
    wxListCtrl* m_tcList = nullptr;
    wxChoice* m_tcTypeCtrl = nullptr;
    wxButton* m_btnAddThermocouple = nullptr;

    // Stepper / motion objects
    std::vector<std::unique_ptr<MCU_stepper>> m_stepperObjs;
    std::vector<std::unique_ptr<MCU_endstop>> m_endstopObjs;
    std::unique_ptr<ToolHead> m_toolhead;
    std::unique_ptr<GCodeParser> m_gcode;

    // PrinterRail objects (for homing + endstop queries)
    std::vector<std::unique_ptr<PrinterRail>> m_rails;
    int m_endstopPollCounter = 0;

    // Motion UI
    wxListCtrl* m_stepperList = nullptr;
    wxTextCtrl* m_stepPinCtrl = nullptr;
    wxTextCtrl* m_dirPinCtrl = nullptr;
    wxButton* m_btnAddStepper = nullptr;
    wxTextCtrl* m_gcodeInput = nullptr;
    wxButton* m_btnSendGcode = nullptr;
    wxButton* m_btnHomeAll = nullptr;
    wxStaticText* m_motionStatus = nullptr;

    // Jog controls
    wxTextCtrl* m_jogDistXY = nullptr;
    wxTextCtrl* m_jogDistZ = nullptr;
    wxTextCtrl* m_jogSpeed = nullptr;

    // Config loading
    wxButton* m_btnLoadConfig = nullptr;
    wxStaticText* m_configPathLabel = nullptr;
    std::unique_ptr<ConfigResult> m_configResult;

    // Endstop status display
    wxStaticText* m_endstopStatus = nullptr;

    // Print tab
    wxStaticText* m_printFileLabel = nullptr;
    wxButton* m_btnPrintLoad = nullptr;
    wxButton* m_btnPrintStart = nullptr;
    wxButton* m_btnPrintPause = nullptr;
    wxButton* m_btnPrintStop = nullptr;
    wxGauge* m_printProgress = nullptr;
    wxStaticText* m_printStatus = nullptr;
    wxTextCtrl* m_printGcodeView = nullptr;
    std::string m_printFilePath;
    std::vector<std::string> m_printLines;
    std::atomic<bool> m_printing{false};
    std::atomic<bool> m_printPaused{false};
    std::atomic<bool> m_printStop{false};
    std::thread m_printThread;
    std::atomic<size_t> m_printCurrentLine{0};
    size_t m_printLastHighlightLine = 0;  // for UI highlighting
    std::atomic<size_t> m_printErrorCount{0};

    // Driver error ignore
    wxCheckBox* m_cbIgnoreTmcErrors = nullptr;

    // Pin management
    int m_nextOid = 0;

    void CreateUI();
    void buildTmcDriverTabs();
    bool updateTmcRegisterUI(bool ignoreErrors = false);  // returns true if any driver unpowered
    void writeTmcRegisterFromUI(size_t driverIdx, uint8_t reg);
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
    void OnAddSpi(wxCommandEvent& evt);
    void OnAddThermocouple(wxCommandEvent& evt);
    void OnAddStepper(wxCommandEvent& evt);
    void OnSendGcode(wxCommandEvent& evt);
    void OnHomeAll(wxCommandEvent& evt);
    void OnLoadConfig(wxCommandEvent& evt);
    void OnJog(wxCommandEvent& evt);
    void OnPrintLoad(wxCommandEvent& evt);
    void OnPrintStart(wxCommandEvent& evt);
    void OnPrintPause(wxCommandEvent& evt);
    void OnPrintStop(wxCommandEvent& evt);
    void PrintThread();
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
    ID_ADD_SPI,
    ID_ADD_THERMOCOUPLE,
    ID_ADD_STEPPER,
    ID_SEND_GCODE,
    ID_HOME_ALL,
    ID_LOAD_CONFIG,
    ID_JOG_XP,
    ID_JOG_XN,
    ID_JOG_YP,
    ID_JOG_YN,
    ID_JOG_ZP,
    ID_JOG_ZN,
    ID_PRINT_LOAD,
    ID_PRINT_START,
    ID_PRINT_PAUSE,
    ID_PRINT_STOP,
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
    Bind(wxEVT_BUTTON, &KlipperFrame::OnAddSpi, this, ID_ADD_SPI);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnAddThermocouple, this, ID_ADD_THERMOCOUPLE);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnAddStepper, this, ID_ADD_STEPPER);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnSendGcode, this, ID_SEND_GCODE);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnHomeAll, this, ID_HOME_ALL);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnLoadConfig, this, ID_LOAD_CONFIG);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnJog, this, ID_JOG_XP);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnJog, this, ID_JOG_XN);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnJog, this, ID_JOG_YP);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnJog, this, ID_JOG_YN);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnJog, this, ID_JOG_ZP);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnJog, this, ID_JOG_ZN);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnPrintLoad, this, ID_PRINT_LOAD);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnPrintStart, this, ID_PRINT_START);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnPrintPause, this, ID_PRINT_PAUSE);
    Bind(wxEVT_BUTTON, &KlipperFrame::OnPrintStop, this, ID_PRINT_STOP);
    Bind(wxEVT_TIMER, &KlipperFrame::OnUITimer, this, ID_UI_TIMER);
    Bind(wxEVT_CLOSE_WINDOW, &KlipperFrame::OnClose, this);

    m_uiTimer.Start(100); // 10 Hz UI update
    Log("Klipper Host C++ GUI started. Click 'Connect' to begin.", wxColour(0, 0, 160));
}

KlipperFrame::~KlipperFrame() {
    // Stop any active print thread before destruction
    if (m_printing) {
        m_printStop = true;
        m_printPaused = false;
    }
    if (m_printThread.joinable()) m_printThread.join();
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

    m_btnLoadConfig = new wxButton(mainPanel, ID_LOAD_CONFIG, "Load Config...");

    toolSizer->Add(m_btnConnect, 0, wxALL, 3);
    toolSizer->Add(m_btnIdentify, 0, wxALL, 3);
    toolSizer->Add(m_btnLoadConfig, 0, wxALL, 3);
    toolSizer->Add(m_btnGetClock, 0, wxALL, 3);
    toolSizer->Add(m_btnGetUptime, 0, wxALL, 3);
    toolSizer->Add(m_btnGetConfig, 0, wxALL, 3);
    toolSizer->AddStretchSpacer();
    toolSizer->Add(m_btnReset, 0, wxALL, 3);
    toolSizer->Add(m_btnEmergencyStop, 0, wxALL, 3);
    mainSizer->Add(toolSizer, 0, wxEXPAND);

    // Config path label
    m_configPathLabel = new wxStaticText(mainPanel, wxID_ANY, "Config: (none)");
    m_configPathLabel->SetForegroundColour(wxColour(80, 80, 80));
    mainSizer->Add(m_configPathLabel, 0, wxLEFT | wxRIGHT | wxEXPAND, 5);

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

    // SPI/I2C Buses tab
    auto* busPanel = new wxPanel(notebook);
    auto* busSizer = new wxBoxSizer(wxVERTICAL);
    auto* busAddSizer = new wxBoxSizer(wxHORIZONTAL);
    busAddSizer->Add(new wxStaticText(busPanel, wxID_ANY, "CS Pin:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_spiCsCtrl = new wxTextCtrl(busPanel, wxID_ANY, "PA5", wxDefaultPosition, wxSize(60, -1));
    busAddSizer->Add(m_spiCsCtrl, 0, wxALL, 3);
    busAddSizer->Add(new wxStaticText(busPanel, wxID_ANY, "SPI Bus:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_spiBusCtrl = new wxTextCtrl(busPanel, wxID_ANY, "spi0", wxDefaultPosition, wxSize(60, -1));
    busAddSizer->Add(m_spiBusCtrl, 0, wxALL, 3);
    m_btnAddSpi = new wxButton(busPanel, ID_ADD_SPI, "Add SPI");
    busAddSizer->Add(m_btnAddSpi, 0, wxALL, 3);
    busSizer->Add(busAddSizer, 0, wxEXPAND);
    m_busList = new wxListCtrl(busPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT);
    m_busList->AppendColumn("Type", wxLIST_FORMAT_LEFT, 60);
    m_busList->AppendColumn("OID", wxLIST_FORMAT_LEFT, 50);
    m_busList->AppendColumn("Bus", wxLIST_FORMAT_LEFT, 80);
    m_busList->AppendColumn("Pin/Addr", wxLIST_FORMAT_LEFT, 100);
    m_busList->AppendColumn("Status", wxLIST_FORMAT_LEFT, 150);
    busSizer->Add(m_busList, 1, wxEXPAND | wxALL, 2);
    busPanel->SetSizer(busSizer);
    notebook->AddPage(busPanel, "Buses");

    // Thermocouple tab
    auto* tcPanel = new wxPanel(notebook);
    auto* tcSizer = new wxBoxSizer(wxVERTICAL);
    auto* tcAddSizer = new wxBoxSizer(wxHORIZONTAL);
    tcAddSizer->Add(new wxStaticText(tcPanel, wxID_ANY, "Type:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_tcTypeCtrl = new wxChoice(tcPanel, wxID_ANY);
    m_tcTypeCtrl->Append("MAX31855");
    m_tcTypeCtrl->Append("MAX31856");
    m_tcTypeCtrl->Append("MAX6675");
    m_tcTypeCtrl->Append("MAX31865");
    m_tcTypeCtrl->SetSelection(0);
    tcAddSizer->Add(m_tcTypeCtrl, 0, wxALL, 3);
    m_btnAddThermocouple = new wxButton(tcPanel, ID_ADD_THERMOCOUPLE, "Add Thermocouple (uses last SPI)");
    tcAddSizer->Add(m_btnAddThermocouple, 0, wxALL, 3);
    tcSizer->Add(tcAddSizer, 0, wxEXPAND);
    m_tcList = new wxListCtrl(tcPanel, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_REPORT);
    m_tcList->AppendColumn("OID", wxLIST_FORMAT_LEFT, 50);
    m_tcList->AppendColumn("Type", wxLIST_FORMAT_LEFT, 100);
    m_tcList->AppendColumn("Temperature", wxLIST_FORMAT_LEFT, 120);
    m_tcList->AppendColumn("Fault", wxLIST_FORMAT_LEFT, 80);
    m_tcList->AppendColumn("Status", wxLIST_FORMAT_LEFT, 150);
    tcSizer->Add(m_tcList, 1, wxEXPAND | wxALL, 2);
    tcPanel->SetSizer(tcSizer);
    notebook->AddPage(tcPanel, "Thermocouple");

    // Motion tab
    auto* motionPanel = new wxPanel(notebook);
    auto* motionSizer = new wxBoxSizer(wxVERTICAL);

    // Add stepper row
    auto* stepAddSizer = new wxBoxSizer(wxHORIZONTAL);
    stepAddSizer->Add(new wxStaticText(motionPanel, wxID_ANY, "Step Pin:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_stepPinCtrl = new wxTextCtrl(motionPanel, wxID_ANY, "PD6", wxDefaultPosition, wxSize(60, -1));
    stepAddSizer->Add(m_stepPinCtrl, 0, wxALL, 3);
    stepAddSizer->Add(new wxStaticText(motionPanel, wxID_ANY, "Dir Pin:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_dirPinCtrl = new wxTextCtrl(motionPanel, wxID_ANY, "PD11", wxDefaultPosition, wxSize(60, -1));
    stepAddSizer->Add(m_dirPinCtrl, 0, wxALL, 3);
    m_btnAddStepper = new wxButton(motionPanel, ID_ADD_STEPPER, "Add Stepper");
    stepAddSizer->Add(m_btnAddStepper, 0, wxALL, 3);
    m_btnHomeAll = new wxButton(motionPanel, ID_HOME_ALL, "Home All (G28)");
    stepAddSizer->Add(m_btnHomeAll, 0, wxALL, 3);
    motionSizer->Add(stepAddSizer, 0, wxEXPAND);

    // Stepper list
    m_stepperList = new wxListCtrl(motionPanel, wxID_ANY, wxDefaultPosition, wxSize(-1, 100), wxLC_REPORT);
    m_stepperList->AppendColumn("OID", wxLIST_FORMAT_LEFT, 50);
    m_stepperList->AppendColumn("Step Pin", wxLIST_FORMAT_LEFT, 80);
    m_stepperList->AppendColumn("Dir Pin", wxLIST_FORMAT_LEFT, 80);
    m_stepperList->AppendColumn("Step Dist", wxLIST_FORMAT_LEFT, 90);
    m_stepperList->AppendColumn("Status", wxLIST_FORMAT_LEFT, 120);
    motionSizer->Add(m_stepperList, 1, wxEXPAND | wxALL, 2);

    // G-code input
    auto* gcodeSizer = new wxBoxSizer(wxHORIZONTAL);
    gcodeSizer->Add(new wxStaticText(motionPanel, wxID_ANY, "G-code:"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_gcodeInput = new wxTextCtrl(motionPanel, wxID_ANY, "G1 X10 Y5 F3000",
        wxDefaultPosition, wxSize(350, -1), wxTE_PROCESS_ENTER);
    gcodeSizer->Add(m_gcodeInput, 1, wxALL, 3);
    m_btnSendGcode = new wxButton(motionPanel, ID_SEND_GCODE, "Send");
    gcodeSizer->Add(m_btnSendGcode, 0, wxALL, 3);
    motionSizer->Add(gcodeSizer, 0, wxEXPAND);

    // ---- Jog Controls ----
    auto* jogBox = new wxStaticBoxSizer(wxVERTICAL, motionPanel, "Jog Controls");

    // Distance / speed inputs row
    auto* jogParamSizer = new wxBoxSizer(wxHORIZONTAL);
    jogParamSizer->Add(new wxStaticText(motionPanel, wxID_ANY, "XY dist (mm):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_jogDistXY = new wxTextCtrl(motionPanel, wxID_ANY, "10", wxDefaultPosition, wxSize(50, -1));
    jogParamSizer->Add(m_jogDistXY, 0, wxALL, 3);
    jogParamSizer->Add(new wxStaticText(motionPanel, wxID_ANY, "Z dist (mm):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_jogDistZ = new wxTextCtrl(motionPanel, wxID_ANY, "1", wxDefaultPosition, wxSize(50, -1));
    jogParamSizer->Add(m_jogDistZ, 0, wxALL, 3);
    jogParamSizer->Add(new wxStaticText(motionPanel, wxID_ANY, "Speed (mm/min):"), 0, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    m_jogSpeed = new wxTextCtrl(motionPanel, wxID_ANY, "3000", wxDefaultPosition, wxSize(60, -1));
    jogParamSizer->Add(m_jogSpeed, 0, wxALL, 3);
    jogBox->Add(jogParamSizer, 0, wxEXPAND);

    // Buttons row: XY pad + Z pad
    auto* jogBtnSizer = new wxBoxSizer(wxHORIZONTAL);

    // XY pad using wxGridSizer 3x3
    auto* xyGrid = new wxGridSizer(3, 3, 2, 2);
    wxSize jogBtnSize(50, 35);
    xyGrid->Add(0, 0);  // top-left empty
    xyGrid->Add(new wxButton(motionPanel, ID_JOG_YP, "+Y", wxDefaultPosition, jogBtnSize), 0, wxEXPAND);
    xyGrid->Add(0, 0);  // top-right empty
    xyGrid->Add(new wxButton(motionPanel, ID_JOG_XN, "-X", wxDefaultPosition, jogBtnSize), 0, wxEXPAND);
    xyGrid->Add(0, 0);  // center empty
    xyGrid->Add(new wxButton(motionPanel, ID_JOG_XP, "+X", wxDefaultPosition, jogBtnSize), 0, wxEXPAND);
    xyGrid->Add(0, 0);  // bottom-left empty
    xyGrid->Add(new wxButton(motionPanel, ID_JOG_YN, "-Y", wxDefaultPosition, jogBtnSize), 0, wxEXPAND);
    xyGrid->Add(0, 0);  // bottom-right empty
    jogBtnSizer->Add(xyGrid, 0, wxALL, 5);

    jogBtnSizer->AddSpacer(20);

    // Z column
    auto* zCol = new wxBoxSizer(wxVERTICAL);
    zCol->Add(new wxButton(motionPanel, ID_JOG_ZP, "+Z", wxDefaultPosition, jogBtnSize), 0, wxALL, 2);
    zCol->Add(new wxButton(motionPanel, ID_JOG_ZN, "-Z", wxDefaultPosition, jogBtnSize), 0, wxALL, 2);
    jogBtnSizer->Add(zCol, 0, wxALIGN_CENTER_VERTICAL);

    jogBox->Add(jogBtnSizer, 0, wxALL, 2);
    motionSizer->Add(jogBox, 0, wxEXPAND | wxALL, 5);

    // Motion status
    m_motionStatus = new wxStaticText(motionPanel, wxID_ANY, "Motion: idle");
    m_motionStatus->SetForegroundColour(wxColour(80, 80, 80));
    motionSizer->Add(m_motionStatus, 0, wxALL, 5);

    // Endstop status
    m_endstopStatus = new wxStaticText(motionPanel, wxID_ANY, "Endstops: (not configured)");
    m_endstopStatus->SetForegroundColour(wxColour(80, 80, 80));
    motionSizer->Add(m_endstopStatus, 0, wxLEFT | wxRIGHT | wxBOTTOM, 5);

    motionPanel->SetSizer(motionSizer);
    notebook->AddPage(motionPanel, "Motion");

    // ---- Stepper Drivers tab (TMC5160 registers) ----
    auto* driversPanel = new wxPanel(notebook);
    auto* driversSizer = new wxBoxSizer(wxVERTICAL);
    m_cbIgnoreTmcErrors = new wxCheckBox(driversPanel, wxID_ANY, "Ignore driver errors (suppress warnings/errors)");
    driversSizer->Add(m_cbIgnoreTmcErrors, 0, wxALL, 5);
    m_tmcNotebook = new wxNotebook(driversPanel, wxID_ANY);
    driversSizer->Add(m_tmcNotebook, 1, wxEXPAND | wxALL, 2);
    driversPanel->SetSizer(driversSizer);
    notebook->AddPage(driversPanel, "Stepper Drivers");

    // ---- Print tab (G-code file execution) ----
    auto* printPanel = new wxPanel(notebook);
    auto* printSizer = new wxBoxSizer(wxVERTICAL);

    // File selection row
    auto* printFileSizer = new wxBoxSizer(wxHORIZONTAL);
    m_btnPrintLoad = new wxButton(printPanel, ID_PRINT_LOAD, "Load G-code...");
    printFileSizer->Add(m_btnPrintLoad, 0, wxALL, 3);
    m_printFileLabel = new wxStaticText(printPanel, wxID_ANY, "File: (none)");
    m_printFileLabel->SetForegroundColour(wxColour(80, 80, 80));
    printFileSizer->Add(m_printFileLabel, 1, wxALIGN_CENTER_VERTICAL | wxALL, 3);
    printSizer->Add(printFileSizer, 0, wxEXPAND);

    // Control buttons row
    auto* printCtrlSizer = new wxBoxSizer(wxHORIZONTAL);
    m_btnPrintStart = new wxButton(printPanel, ID_PRINT_START, "Start");
    m_btnPrintStart->SetBackgroundColour(wxColour(200, 255, 200));
    m_btnPrintPause = new wxButton(printPanel, ID_PRINT_PAUSE, "Pause");
    m_btnPrintStop = new wxButton(printPanel, ID_PRINT_STOP, "Stop");
    m_btnPrintStop->SetBackgroundColour(wxColour(255, 200, 200));
    printCtrlSizer->Add(m_btnPrintStart, 0, wxALL, 3);
    printCtrlSizer->Add(m_btnPrintPause, 0, wxALL, 3);
    printCtrlSizer->Add(m_btnPrintStop, 0, wxALL, 3);
    printSizer->Add(printCtrlSizer, 0, wxEXPAND);

    // Progress bar
    m_printProgress = new wxGauge(printPanel, wxID_ANY, 100, wxDefaultPosition, wxSize(-1, 20));
    printSizer->Add(m_printProgress, 0, wxEXPAND | wxALL, 5);

    // Print status
    m_printStatus = new wxStaticText(printPanel, wxID_ANY, "Print: idle");
    m_printStatus->SetForegroundColour(wxColour(80, 80, 80));
    printSizer->Add(m_printStatus, 0, wxALL, 5);

    // G-code preview
    m_printGcodeView = new wxTextCtrl(printPanel, wxID_ANY, "", wxDefaultPosition, wxDefaultSize,
        wxTE_MULTILINE | wxTE_READONLY | wxTE_RICH2 | wxHSCROLL);
    m_printGcodeView->SetFont(wxFont(8, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL));
    printSizer->Add(m_printGcodeView, 1, wxEXPAND | wxALL, 2);

    printPanel->SetSizer(printSizer);
    notebook->AddPage(printPanel, "Print");

    // Initial print button states
    m_btnPrintLoad->Enable(false);
    m_btnPrintStart->Enable(false);
    m_btnPrintPause->Enable(false);
    m_btnPrintStop->Enable(false);

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
    m_btnAddSpi->Enable(false);
    m_btnAddThermocouple->Enable(false);
    m_btnAddStepper->Enable(false);
    m_btnSendGcode->Enable(false);
    m_btnHomeAll->Enable(false);
    m_btnLoadConfig->Enable(false);
}

void KlipperFrame::buildTmcDriverTabs() {
    if (!m_tmcNotebook) return;

    // Clear any previous tabs
    m_tmcNotebook->DeleteAllPages();
    m_tmcUI.clear();

    // Register bit definitions: { register_addr, bit_pos, name }
    struct BitDef { uint8_t reg; uint8_t bit; const char* name; };
    // Register multi-bit field definitions: { register_addr, low_bit, high_bit, name }
    struct FieldDef { uint8_t reg; uint8_t lo; uint8_t hi; const char* name; };

    // GCONF bits
    static const BitDef gconfBits[] = {
        {TMC5160Reg::GCONF, 0, "recalibrate"},
        {TMC5160Reg::GCONF, 1, "faststandstill"},
        {TMC5160Reg::GCONF, 2, "en_pwm_mode"},
        {TMC5160Reg::GCONF, 3, "multistep_filt"},
        {TMC5160Reg::GCONF, 4, "shaft"},
        {TMC5160Reg::GCONF, 5, "diag0_error"},
        {TMC5160Reg::GCONF, 6, "diag0_otpw"},
        {TMC5160Reg::GCONF, 7, "diag0_stall"},
        {TMC5160Reg::GCONF, 8, "diag1_stall"},
        {TMC5160Reg::GCONF, 9, "diag1_index"},
        {TMC5160Reg::GCONF, 10, "diag1_onstate"},
        {TMC5160Reg::GCONF, 12, "diag0_pushpull"},
        {TMC5160Reg::GCONF, 13, "diag1_pushpull"},
        {TMC5160Reg::GCONF, 14, "small_hysteresis"},
        {TMC5160Reg::GCONF, 15, "stop_enable"},
        {TMC5160Reg::GCONF, 16, "direct_mode"},
    };
    // GSTAT bits
    static const BitDef gstatBits[] = {
        {TMC5160Reg::GSTAT, 0, "reset"},
        {TMC5160Reg::GSTAT, 1, "drv_err"},
        {TMC5160Reg::GSTAT, 2, "uv_cp"},
    };
    // DRV_STATUS bits
    static const BitDef drvStatusBits[] = {
        {TMC5160Reg::DRV_STATUS, 12, "s2vsA"},
        {TMC5160Reg::DRV_STATUS, 13, "s2vsB"},
        {TMC5160Reg::DRV_STATUS, 14, "stealth"},
        {TMC5160Reg::DRV_STATUS, 24, "stallGuard"},
        {TMC5160Reg::DRV_STATUS, 25, "ot"},
        {TMC5160Reg::DRV_STATUS, 26, "otpw"},
        {TMC5160Reg::DRV_STATUS, 27, "s2gA"},
        {TMC5160Reg::DRV_STATUS, 28, "s2gB"},
        {TMC5160Reg::DRV_STATUS, 29, "olA"},
        {TMC5160Reg::DRV_STATUS, 30, "olB"},
        {TMC5160Reg::DRV_STATUS, 31, "stst"},
    };
    static const FieldDef drvStatusFields[] = {
        {TMC5160Reg::DRV_STATUS, 0, 9, "SG_RESULT"},
        {TMC5160Reg::DRV_STATUS, 16, 20, "CS_ACTUAL"},
    };
    // CHOPCONF bits
    static const BitDef chopconfBits[] = {
        {TMC5160Reg::CHOPCONF, 28, "intpol"},
        {TMC5160Reg::CHOPCONF, 29, "dedge"},
        {TMC5160Reg::CHOPCONF, 30, "diss2g"},
        {TMC5160Reg::CHOPCONF, 31, "diss2vs"},
    };
    static const FieldDef chopconfFields[] = {
        {TMC5160Reg::CHOPCONF, 0, 3, "toff"},
        {TMC5160Reg::CHOPCONF, 4, 6, "hstrt"},
        {TMC5160Reg::CHOPCONF, 7, 10, "hend"},
        {TMC5160Reg::CHOPCONF, 15, 16, "tbl"},
        {TMC5160Reg::CHOPCONF, 20, 23, "tpfd"},
        {TMC5160Reg::CHOPCONF, 24, 27, "mres"},
    };
    // IHOLD_IRUN fields
    static const FieldDef iholdIrunFields[] = {
        {TMC5160Reg::IHOLD_IRUN, 0, 4, "IHOLD"},
        {TMC5160Reg::IHOLD_IRUN, 8, 12, "IRUN"},
        {TMC5160Reg::IHOLD_IRUN, 16, 19, "IHOLDDELAY"},
    };
    // PWMCONF bits
    static const BitDef pwmconfBits[] = {
        {TMC5160Reg::PWMCONF, 18, "pwm_autoscale"},
        {TMC5160Reg::PWMCONF, 19, "pwm_autograd"},
    };
    static const FieldDef pwmconfFields[] = {
        {TMC5160Reg::PWMCONF, 0, 7, "pwm_ofs"},
        {TMC5160Reg::PWMCONF, 8, 15, "pwm_grad"},
        {TMC5160Reg::PWMCONF, 16, 17, "pwm_freq"},
        {TMC5160Reg::PWMCONF, 20, 21, "freewheel"},
        {TMC5160Reg::PWMCONF, 24, 27, "pwm_reg"},
        {TMC5160Reg::PWMCONF, 28, 31, "pwm_lim"},
    };
    // IOIN bits
    static const BitDef ioinBits[] = {
        {TMC5160Reg::IOIN, 0, "refl_step"},
        {TMC5160Reg::IOIN, 1, "refr_dir"},
        {TMC5160Reg::IOIN, 2, "encb_dcen"},
        {TMC5160Reg::IOIN, 3, "enca_dcin"},
        {TMC5160Reg::IOIN, 4, "drv_enn"},
        {TMC5160Reg::IOIN, 5, "enc_n"},
        {TMC5160Reg::IOIN, 6, "sd_mode"},
        {TMC5160Reg::IOIN, 7, "swcomp_in"},
    };
    static const FieldDef ioinFields[] = {
        {TMC5160Reg::IOIN, 24, 31, "VERSION"},
    };

    // Register group descriptor
    struct RegGroup {
        const char* name;
        const BitDef* bits;
        size_t numBits;
        const FieldDef* fields;
        size_t numFields;
        bool writable;
    };

    static const RegGroup groups[] = {
        {"GCONF",      gconfBits,     std::size(gconfBits),     nullptr, 0,                          true},
        {"GSTAT",      gstatBits,     std::size(gstatBits),     nullptr, 0,                          false},
        {"IOIN",       ioinBits,      std::size(ioinBits),      ioinFields, std::size(ioinFields),   false},
        {"IHOLD_IRUN", nullptr,       0,                        iholdIrunFields, std::size(iholdIrunFields), true},
        {"CHOPCONF",   chopconfBits,  std::size(chopconfBits),  chopconfFields, std::size(chopconfFields), true},
        {"DRV_STATUS", drvStatusBits, std::size(drvStatusBits), drvStatusFields, std::size(drvStatusFields), false},
        {"PWMCONF",    pwmconfBits,   std::size(pwmconfBits),   pwmconfFields, std::size(pwmconfFields), true},
    };

    for (size_t di = 0; di < m_tmcDrivers.size(); ++di) {
        auto& tmc = m_tmcDrivers[di];
        TmcDriverUI dui;

        auto* scrollWin = new wxScrolledWindow(m_tmcNotebook, wxID_ANY);
        scrollWin->SetScrollRate(10, 10);

        auto* hSizer = new wxBoxSizer(wxHORIZONTAL);

        for (auto& grp : groups) {
            auto* box = new wxStaticBoxSizer(wxVERTICAL, scrollWin, grp.name);

            // Single-bit checkboxes
            for (size_t bi = 0; bi < grp.numBits; ++bi) {
                auto& bd = grp.bits[bi];
                auto* cb = new wxCheckBox(box->GetStaticBox(), wxID_ANY, bd.name);
                if (!grp.writable) {
                    cb->Disable();  // read-only
                } else {
                    // Bind checkbox toggle → write register
                    cb->Bind(wxEVT_CHECKBOX, [this, di, reg = bd.reg](wxCommandEvent&) {
                        writeTmcRegisterFromUI(di, reg);
                    });
                }
                box->Add(cb, 0, wxLEFT | wxRIGHT, 3);
                dui.bits.push_back({cb, bd.reg, bd.bit, grp.writable});
            }

            // Multi-bit fields
            for (size_t fi = 0; fi < grp.numFields; ++fi) {
                auto& fd = grp.fields[fi];
                if (grp.writable) {
                    // Writable: use wxSpinCtrl
                    int maxVal = (1 << (fd.hi - fd.lo + 1)) - 1;
                    auto* fieldSizer = new wxBoxSizer(wxHORIZONTAL);
                    auto* lbl = new wxStaticText(box->GetStaticBox(), wxID_ANY, wxString(fd.name) + ":");
                    auto lblFont = lbl->GetFont();
                    lblFont.SetPointSize(8);
                    lbl->SetFont(lblFont);
                    auto* spin = new wxSpinCtrl(box->GetStaticBox(), wxID_ANY, "0",
                        wxDefaultPosition, wxSize(60, -1), wxSP_ARROW_KEYS, 0, maxVal, 0);
                    spin->Bind(wxEVT_SPINCTRL, [this, di, reg = fd.reg](wxSpinEvent&) {
                        writeTmcRegisterFromUI(di, reg);
                    });
                    fieldSizer->Add(lbl, 0, wxALIGN_CENTER_VERTICAL | wxRIGHT, 3);
                    fieldSizer->Add(spin, 0, 0, 0);
                    box->Add(fieldSizer, 0, wxLEFT | wxRIGHT | wxTOP, 3);
                    dui.writeFields.push_back({spin, fd.reg, fd.lo, fd.hi});
                } else {
                    // Read-only: use wxStaticText
                    auto* label = new wxStaticText(box->GetStaticBox(), wxID_ANY,
                        wxString::Format("%s: ---", fd.name));
                    auto font = label->GetFont();
                    font.SetPointSize(8);
                    label->SetFont(font);
                    box->Add(label, 0, wxLEFT | wxRIGHT | wxTOP, 3);
                    dui.fields.push_back({label, fd.reg, fd.lo, fd.hi});
                }
            }

            hSizer->Add(box, 0, wxALL | wxEXPAND, 3);
        }

        // Status summary label at end
        dui.statusLabel = new wxStaticText(scrollWin, wxID_ANY, "Status: waiting...");
        auto* vSizer = new wxBoxSizer(wxVERTICAL);
        vSizer->Add(hSizer, 0, wxEXPAND);
        vSizer->Add(dui.statusLabel, 0, wxALL, 5);

        scrollWin->SetSizer(vSizer);
        vSizer->FitInside(scrollWin);

        m_tmcNotebook->AddPage(scrollWin, tmc->getName());
        m_tmcUI.push_back(std::move(dui));
    }
}

bool KlipperFrame::updateTmcRegisterUI(bool ignoreErrors) {
    if (m_tmcDrivers.empty() || m_tmcUI.empty()) return false;

    bool anyUnpowered = false;

    // Map register address to RegisterDump field
    auto getRegValue = [](const TMC5160::RegisterDump& d, uint8_t reg) -> uint32_t {
        switch (reg) {
            case TMC5160Reg::GCONF:      return d.gconf;
            case TMC5160Reg::GSTAT:       return d.gstat;
            case TMC5160Reg::IOIN:        return d.ioin;
            case TMC5160Reg::IHOLD_IRUN:  return d.ihold_irun;
            case TMC5160Reg::CHOPCONF:    return d.chopconf;
            case TMC5160Reg::DRV_STATUS:  return d.drv_status;
            case TMC5160Reg::PWMCONF:     return d.pwmconf;
            case TMC5160Reg::PWM_SCALE:   return d.pwm_scale;
            default: return 0;
        }
    };

    for (size_t di = 0; di < m_tmcDrivers.size() && di < m_tmcUI.size(); ++di) {
        auto dump = m_tmcDrivers[di]->readAllRegisters();
        auto& dui = m_tmcUI[di];

        if (!dump.valid) {
            if (dui.statusLabel)
                dui.statusLabel->SetLabel("Status: SPI read failed");
            continue;
        }

        // GSTAT bit 2 (uv_cp): charge pump undervoltage = VMot off or too low
        bool unpowered = dump.uvCp();
        if (unpowered) anyUnpowered = true;

        // Update single-bit checkboxes
        for (auto& bc : dui.bits) {
            uint32_t regVal = getRegValue(dump, bc.reg);
            bc.cb->SetValue((regVal >> bc.bit) & 1);
        }

        // Update read-only multi-bit field labels
        for (auto& fl : dui.fields) {
            uint32_t regVal = getRegValue(dump, fl.reg);
            uint32_t mask = ((1U << (fl.bitHigh - fl.bitLow + 1)) - 1);
            uint32_t fieldVal = (regVal >> fl.bitLow) & mask;
            wxString current = fl.label->GetLabel();
            wxString name = current.BeforeFirst(':');
            fl.label->SetLabel(wxString::Format("%s: %u", name, fieldVal));
        }

        // Update writable spin controls (skip if user is editing)
        for (auto& wf : dui.writeFields) {
            if (wf.spin->HasFocus()) continue;
            uint32_t regVal = getRegValue(dump, wf.reg);
            uint32_t mask = ((1U << (wf.bitHigh - wf.bitLow + 1)) - 1);
            uint32_t fieldVal = (regVal >> wf.bitLow) & mask;
            wf.spin->SetValue(static_cast<int>(fieldVal));
        }

        // Update status summary + log errors
        if (dui.statusLabel) {
            if (unpowered) {
                dui.statusLabel->SetLabel("Status: Charge pump undervoltage (GSTAT.uv_cp) - VMot off?");
                dui.statusLabel->SetForegroundColour(wxColour(200, 100, 0));
            } else {
                // Parse DRV_STATUS bits directly from dump
                TMC5160::DriverStatus status;
                status.raw       = dump.drv_status;
                status.sgResult  = dump.drv_status & 0x3FF;
                status.s2vsA     = (dump.drv_status >> 12) & 1;
                status.s2vsB     = (dump.drv_status >> 13) & 1;
                status.stealthChop = (dump.drv_status >> 14) & 1;
                status.csActual  = (dump.drv_status >> 16) & 0x1F;
                status.stallGuard = (dump.drv_status >> 24) & 1;
                status.ot        = (dump.drv_status >> 25) & 1;
                status.otpw      = (dump.drv_status >> 26) & 1;
                status.s2gA      = (dump.drv_status >> 27) & 1;
                status.s2gB      = (dump.drv_status >> 28) & 1;
                status.olA       = (dump.drv_status >> 29) & 1;
                status.olB       = (dump.drv_status >> 30) & 1;
                status.stst      = (dump.drv_status >> 31) & 1;

                if (ignoreErrors) {
                    dui.statusLabel->SetLabel("Status: " + TMC5160::formatStatus(status) + " [errors ignored]");
                    dui.statusLabel->SetForegroundColour(wxColour(120, 120, 120));
                } else if (status.hasError()) {
                    dui.statusLabel->SetLabel("Status: ERROR - " + TMC5160::formatErrors(status));
                    dui.statusLabel->SetForegroundColour(*wxRED);
                    Log(wxString::Format("[TMC5160:%s] ERROR: %s",
                        m_tmcDrivers[di]->getName(), TMC5160::formatErrors(status)), *wxRED);
                } else if (status.hasWarning()) {
                    dui.statusLabel->SetLabel("Status: WARNING - " + TMC5160::formatErrors(status));
                    dui.statusLabel->SetForegroundColour(wxColour(200, 100, 0));
                    Log(wxString::Format("[TMC5160:%s] Warning: %s",
                        m_tmcDrivers[di]->getName(), TMC5160::formatErrors(status)), wxColour(200, 100, 0));
                } else {
                    dui.statusLabel->SetLabel("Status: OK - " + TMC5160::formatStatus(status));
                    dui.statusLabel->SetForegroundColour(wxColour(0, 128, 0));
                }
            }
        }
    }

    return anyUnpowered;
}

void KlipperFrame::writeTmcRegisterFromUI(size_t driverIdx, uint8_t reg) {
    if (driverIdx >= m_tmcDrivers.size() || driverIdx >= m_tmcUI.size()) return;

    auto& dui = m_tmcUI[driverIdx];
    uint32_t value = 0;

    // Collect single-bit checkbox values for this register
    for (auto& bc : dui.bits) {
        if (bc.reg != reg || !bc.writable) continue;
        if (bc.cb->GetValue())
            value |= (1U << bc.bit);
    }

    // Collect multi-bit field values from writable spin controls
    for (auto& wf : dui.writeFields) {
        if (wf.reg != reg) continue;
        uint32_t fieldVal = static_cast<uint32_t>(wf.spin->GetValue());
        value |= (fieldVal << wf.bitLow);
    }

    // Write to hardware
    {
        std::lock_guard<std::mutex> lock(m_mcuMutex);
        m_tmcDrivers[driverIdx]->writeRegister(reg, value);
    }

    Log(wxString::Format("[TMC5160:%s] Write reg 0x%02X = 0x%08X",
        m_tmcDrivers[driverIdx]->getName(), reg, value),
        wxColour(0, 0, 180));
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

    // Update thermocouple readings display
    {
        std::lock_guard<std::mutex> lock(m_tcMutex);
        for (size_t i = 0; i < m_tcReadings.size() && i < static_cast<size_t>(m_tcList->GetItemCount()); i++) {
            auto& r = m_tcReadings[i];
            m_tcList->SetItem(static_cast<int>(i), 2, wxString::Format("%.1f C", r.temp));
            m_tcList->SetItem(static_cast<int>(i), 3, r.fault ? wxString::Format("0x%02X", r.fault) : wxString("OK"));
        }
    }

    // TMC5160 status polling (~every 2 seconds at 10Hz timer)
    if (m_connected && m_mcu.isConfigFinalized() && !m_tmcDrivers.empty()) {
        m_tmcPollCounter++;
        if (m_tmcPollCounter >= 20) {
            m_tmcPollCounter = 0;
            bool ignoreTmc = m_cbIgnoreTmcErrors && m_cbIgnoreTmcErrors->GetValue();
            std::lock_guard<std::mutex> lock(m_mcuMutex);

            // Update register UI and get unpowered state
            bool anyUnpowered = updateTmcRegisterUI(ignoreTmc);

            if (!ignoreTmc) {
                if (anyUnpowered && !m_tmcUnpoweredLogged) {
                    Log("TMC5160: Charge pump undervoltage (GSTAT.uv_cp set) - VMot off or too low",
                        wxColour(200, 100, 0));
                    m_tmcUnpoweredLogged = true;
                } else if (!anyUnpowered && m_tmcUnpoweredLogged) {
                    Log("TMC5160: Charge pump OK (GSTAT.uv_cp cleared) - VMot restored",
                        wxColour(0, 128, 0));
                    m_tmcUnpoweredLogged = false;
                }
            }
        }
    }

    // Endstop polling (~every 500ms at 10Hz timer)
    if (m_connected && m_mcu.isConfigFinalized() && !m_endstopObjs.empty()) {
        m_endstopPollCounter++;
        if (m_endstopPollCounter >= 5) {
            m_endstopPollCounter = 0;
            std::lock_guard<std::mutex> lock(m_mcuMutex);

            wxString endstopStr = "Endstops:";
            const char* axisNames[] = {"X", "Y", "Z"};
            for (size_t i = 0; i < m_endstopObjs.size() && i < 3; ++i) {
                bool triggered = false;
                if (m_endstopObjs[i]->queryState(triggered)) {
                    endstopStr += wxString::Format("  %s=%s", axisNames[i],
                        triggered ? "TRIGGERED" : "open");
                } else {
                    endstopStr += wxString::Format("  %s=?", axisNames[i]);
                }
            }
            if (m_endstopStatus) {
                m_endstopStatus->SetLabel(endstopStr);
                m_endstopStatus->SetForegroundColour(
                    endstopStr.Contains("TRIGGERED") ? *wxRED : wxColour(0, 128, 0));
            }
        }
    }

    // Print progress update + line highlighting
    if (m_printing) {
        size_t current = m_printCurrentLine;
        size_t total = m_printLines.size();
        if (total > 0) {
            int pct = static_cast<int>((current * 100) / total);
            m_printProgress->SetValue(pct);

            size_t errorCount = m_printErrorCount;
            wxString statusText = wxString::Format("Print: line %zu / %zu (%d%%)", current, total, pct);
            if (errorCount > 0)
                statusText += wxString::Format(" [%zu errors]", errorCount);
            if (m_printPaused.load())
                statusText += " [PAUSED]";
            m_printStatus->SetLabel(statusText);
            m_printStatus->SetForegroundColour(
                m_printPaused.load() ? wxColour(200, 100, 0) : wxColour(0, 128, 0));

            // Highlight current line yellow, clear previous
            if (m_printGcodeView && current != m_printLastHighlightLine && current > 0) {
                // Clear previous highlight
                if (m_printLastHighlightLine > 0) {
                    long prevStart = m_printGcodeView->XYToPosition(0, static_cast<long>(m_printLastHighlightLine - 1));
                    long prevEnd = m_printGcodeView->GetLineLength(static_cast<long>(m_printLastHighlightLine - 1)) + prevStart;
                    m_printGcodeView->SetStyle(prevStart, prevEnd, wxTextAttr(*wxBLACK, *wxWHITE));
                }
                // Set new highlight
                long lineIdx = static_cast<long>(current - 1);
                long startPos = m_printGcodeView->XYToPosition(0, lineIdx);
                long endPos = m_printGcodeView->GetLineLength(lineIdx) + startPos;
                m_printGcodeView->SetStyle(startPos, endPos, wxTextAttr(*wxBLACK, wxColour(255, 255, 150)));
                m_printGcodeView->ShowPosition(startPos);
                m_printLastHighlightLine = current;
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
        m_btnAddSpi->Enable(false);
        m_btnAddThermocouple->Enable(false);
        m_btnAddStepper->Enable(false);
        m_btnSendGcode->Enable(false);
        m_btnHomeAll->Enable(false);
        m_btnLoadConfig->Enable(false);
        m_cmdList->DeleteAllItems();
        m_respList->DeleteAllItems();
        m_gpioList->DeleteAllItems();
        m_adcList->DeleteAllItems();
        m_busList->DeleteAllItems();
        m_tcList->DeleteAllItems();
        m_stepperList->DeleteAllItems();
        m_digitalOuts.clear();
        m_adcInputs.clear();
        m_adcReadings.clear();
        m_spiDevices.clear();
        m_i2cDevices.clear();
        m_thermocouples.clear();
        m_tcReadings.clear();
        m_tmcDrivers.clear();
        m_tmcUnpoweredLogged = false;
        m_tmcUI.clear();
        if (m_tmcNotebook) m_tmcNotebook->DeleteAllPages();
        m_stepperObjs.clear();
        m_endstopObjs.clear();
        m_rails.clear();
        m_toolhead.reset();
        m_gcode.reset();
        m_configResult.reset();
        m_configPathLabel->SetLabel("Config: (none)");
        // Stop any active print
        if (m_printing) {
            m_printStop = true;
            if (m_printThread.joinable()) m_printThread.join();
            m_printing = false;
            m_printStop = false;
        }
        m_printLines.clear();
        m_printFilePath.clear();
        m_printCurrentLine = 0;
        if (m_printProgress) m_printProgress->SetValue(0);
        if (m_printStatus) m_printStatus->SetLabel("Print: idle");
        if (m_printFileLabel) m_printFileLabel->SetLabel("File: (none)");
        if (m_btnPrintLoad) m_btnPrintLoad->Enable(false);
        if (m_btnPrintStart) m_btnPrintStart->Enable(false);
        if (m_btnPrintPause) m_btnPrintPause->Enable(false);
        if (m_btnPrintStop) m_btnPrintStop->Enable(false);
        if (m_endstopStatus) m_endstopStatus->SetLabel("Endstops: (not configured)");
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
        m_btnAddSpi->Enable(true);
        m_btnAddThermocouple->Enable(true);
        m_btnAddStepper->Enable(true);

        m_btnLoadConfig->Enable(true);

        // Initialize clock sync BEFORE starting poll thread
        // (poll thread would steal get_clock responses via processIncoming)
        Log("Initializing clock synchronization...");
        if (m_mcu.initClockSync()) {
            auto info = m_mcu.getClockSync().getDebugInfo();
            Log(wxString::Format("Clock sync initialized: freq=%.0f Hz, RTT=%.3f ms",
                info.freq, info.minHalfRtt * 2000.0), wxColour(0, 128, 0));
        } else {
            Log("Clock sync init failed: " + wxString(m_mcu.getLastError()), *wxRED);
        }

        StartPolling();
        StartClockSync();

        // Register shutdown callback — auto-disconnect on MCU shutdown/restart
        m_mcu.setShutdownCallback([this](const std::string& reason) {
            LogFromThread(wxString::Format("!!! MCU SHUTDOWN: %s !!!", reason), *wxRED);
            // Stop any active print
            if (m_printing) {
                m_printStop = true;
            }
            // Schedule auto-disconnect on UI thread
            CallAfter([this]() {
                if (m_connected) {
                    LogFromThread("Auto-disconnecting due to MCU shutdown...", *wxRED);
                    wxCommandEvent evt;
                    OnConnect(evt); // triggers disconnect branch
                }
            });
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
    int consecutiveErrors = 0;
    while (m_pollRunning && m_connected) {
        {
            std::lock_guard<std::mutex> lock(m_mcuMutex);

            // Check serial device health (detects USB disconnect)
            if (!m_mcu.isConnected()) {
                LogFromThread("PollThread: MCU disconnected (port closed)", *wxRED);
                break;
            }

            auto responses = m_mcu.processIncoming(5);

            // processIncoming returns empty on read error or timeout.
            // Track consecutive empty reads to detect device loss.
            if (responses.empty()) {
                // Check if the serial port is still alive
                if (!m_mcu.isConnected()) {
                    LogFromThread("PollThread: serial device lost", *wxRED);
                    break;
                }
                // Normal timeout — not an error
            } else {
                consecutiveErrors = 0;
            }

            for (auto& resp : responses) {
                // Skip high-frequency sensor responses (already shown in ADC/TC lists)
                if (resp.name == "analog_in_state" || resp.name == "thermocouple_state")
                    continue;

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
        } // mutex released here
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // If we broke out due to device loss, trigger auto-disconnect on UI thread
    if (m_connected && !m_pollRunning) {
        // Normal stop — do nothing
    } else if (m_connected) {
        LogFromThread("Auto-disconnecting due to device loss...", *wxRED);
        CallAfter([this]() {
            if (m_connected) {
                wxCommandEvent evt;
                OnConnect(evt); // triggers disconnect branch
            }
        });
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

    if (m_digitalOuts.empty() && m_adcInputs.empty() && m_stepperObjs.empty()) {
        Log("No GPIO, ADC, or stepper objects configured. Add some first.", wxColour(200, 100, 0));
        return;
    }

    Log("Finalizing MCU configuration...");
    wxBusyCursor wait;

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (m_mcu.finalizeConfig()) {
        Log(wxString::Format("Config finalized! OIDs=%d", m_mcu.getOidCount()), wxColour(0, 128, 0));

        // Re-initialize clock sync (may have been invalidated by MCU reset during finalization)
        if (m_mcu.initClockSync()) {
            auto csInfo = m_mcu.getClockSync().getDebugInfo();
            Log(wxString::Format("Clock sync re-initialized: freq=%.0f Hz, RTT=%.3f ms",
                csInfo.freq, csInfo.minHalfRtt * 2000.0), wxColour(0, 128, 0));
        } else {
            Log("Clock sync re-init failed (non-fatal): " + wxString(m_mcu.getLastError()),
                wxColour(200, 100, 0));
        }

        // Update GPIO list status
        for (int i = 0; i < m_gpioList->GetItemCount(); i++) {
            m_gpioList->SetItem(i, 3, "active");
        }
        // Update ADC list status
        for (int i = 0; i < m_adcList->GetItemCount(); i++) {
            m_adcList->SetItem(i, 4, "active");
        }
        // Update bus list status
        for (int i = 0; i < m_busList->GetItemCount(); i++) {
            m_busList->SetItem(i, 4, "active");
        }
        // Update thermocouple list status
        for (int i = 0; i < m_tcList->GetItemCount(); i++) {
            m_tcList->SetItem(i, 4, "active - sampling");
        }

        m_btnAddDigitalOut->Enable(false);
        m_btnAddAdc->Enable(false);
        m_btnFinalize->Enable(false);
        m_btnAddSpi->Enable(false);
        m_btnAddThermocouple->Enable(false);
        m_btnAddStepper->Enable(false);
        m_btnLoadConfig->Enable(false);

        // Update stepper list status
        for (int i = 0; i < m_stepperList->GetItemCount(); i++) {
            m_stepperList->SetItem(i, 4, "active");
        }

        // Create toolhead + gcode parser after finalization
        if (!m_stepperObjs.empty()) {
            m_toolhead = std::make_unique<ToolHead>(m_mcu);

            // Initialize print_time base from actual MCU clock
            double basePrintTime = m_mcu.getClockSync().estimatedPrintTime() + 0.25;
            m_toolhead->setNextPrintTime(basePrintTime);

            // Use config settings if loaded, otherwise defaults
            if (m_configResult) {
                m_toolhead->setMaxVelocity(m_configResult->maxVelocity);
                m_toolhead->setMaxAccel(m_configResult->maxAccel);
                m_toolhead->setSquareCornerVelocity(m_configResult->squareCornerVelocity);
            } else {
                m_toolhead->setMaxVelocity(100);
                m_toolhead->setMaxAccel(1000);
                m_toolhead->setSquareCornerVelocity(5.0);
            }

            for (size_t i = 0; i < m_stepperObjs.size() && i < 3; ++i) {
                m_toolhead->addStepper(static_cast<int>(i), m_stepperObjs[i].get());
            }

            m_gcode = std::make_unique<GCodeParser>(*m_toolhead, m_mcu);

            // Register rails for homing (G28) support
            for (size_t i = 0; i < m_rails.size() && i < 3; ++i) {
                m_gcode->addRail(static_cast<int>(i), m_rails[i].get());
            }

            m_btnSendGcode->Enable(true);
            m_btnHomeAll->Enable(true);
            m_btnPrintLoad->Enable(true);
            Log("Toolhead + G-code parser initialized", wxColour(0, 128, 0));

            if (!m_rails.empty()) {
                Log(wxString::Format("  %zu axis rail(s) registered for homing", m_rails.size()),
                    wxColour(0, 128, 0));
            }
        }

        // Initialize TMC5160 drivers (after finalize, SPI is now operational)
        if (!m_tmcDrivers.empty()) {
            Log(wxString::Format("Initializing %zu TMC5160 driver(s)...", m_tmcDrivers.size()));
            for (auto& tmc : m_tmcDrivers) {
                if (tmc->initRegisters()) {
                    auto status = tmc->readStatus();
                    wxString msg = wxString::Format("  [TMC5160:%s] Init OK - %s",
                        tmc->getName(), TMC5160::formatStatus(status));
                    if (status.hasError()) {
                        msg += " ERRORS: " + TMC5160::formatErrors(status);
                        Log(msg, *wxRED);
                    } else if (status.hasWarning()) {
                        msg += " WARN: " + TMC5160::formatErrors(status);
                        Log(msg, wxColour(200, 100, 0));
                    } else {
                        Log(msg, wxColour(0, 128, 0));
                    }
                } else {
                    Log(wxString::Format("  [TMC5160:%s] Init FAILED", tmc->getName()), *wxRED);
                }
            }
        }
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

void KlipperFrame::OnAddSpi(wxCommandEvent&) {
    if (!m_connected) return;
    if (m_mcu.isConfigFinalized()) {
        Log("Config already finalized.", *wxRED);
        return;
    }

    wxString csPin = m_spiCsCtrl->GetValue().Trim();
    wxString spiBus = m_spiBusCtrl->GetValue().Trim();

    auto spi = std::make_unique<MCU_SPI>(m_mcu);
    spi->setupPin(csPin.ToStdString(), false);
    spi->setupBus(spiBus.ToStdString(), 0, 4000000);

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (spi->buildConfig()) {
        int row = m_busList->GetItemCount();
        m_busList->InsertItem(row, "SPI");
        m_busList->SetItem(row, 1, wxString::Format("%d", spi->getOid()));
        m_busList->SetItem(row, 2, spiBus);
        m_busList->SetItem(row, 3, csPin);
        m_busList->SetItem(row, 4, "pending finalize");

        Log(wxString::Format("Added SPI: OID=%d bus=%s cs=%s", spi->getOid(), spiBus, csPin),
            wxColour(0, 128, 0));
        m_spiDevices.push_back(std::move(spi));
    } else {
        Log("Failed to add SPI device", *wxRED);
    }
}

void KlipperFrame::OnAddThermocouple(wxCommandEvent&) {
    if (!m_connected) return;
    if (m_mcu.isConfigFinalized()) {
        Log("Config already finalized.", *wxRED);
        return;
    }
    if (m_spiDevices.empty()) {
        Log("Add an SPI device first (CS pin for the thermocouple chip).", *wxRED);
        return;
    }

    int typeIdx = m_tcTypeCtrl->GetSelection();
    auto sensorType = static_cast<MCU_Thermocouple::SensorType>(typeIdx);
    wxString typeName = m_tcTypeCtrl->GetString(typeIdx);

    auto tc = std::make_unique<MCU_Thermocouple>(m_mcu);
    tc->setupSpi(*m_spiDevices.back());
    tc->setupSensor(sensorType);
    tc->setupReportTime(0.300);

    size_t tcIdx = m_thermocouples.size();
    {
        std::lock_guard<std::mutex> lock(m_tcMutex);
        m_tcReadings.push_back({0.0, 0});
    }

    tc->setCallback([this, tcIdx](double temp, uint8_t fault) {
        std::lock_guard<std::mutex> lock(m_tcMutex);
        if (tcIdx < m_tcReadings.size()) {
            m_tcReadings[tcIdx] = {temp, fault};
        }
    });

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    tc->initSensor();
    if (tc->buildConfig()) {
        int row = m_tcList->GetItemCount();
        m_tcList->InsertItem(row, wxString::Format("%d", tc->getOid()));
        m_tcList->SetItem(row, 1, typeName);
        m_tcList->SetItem(row, 2, "---");
        m_tcList->SetItem(row, 3, "---");
        m_tcList->SetItem(row, 4, "pending finalize");

        Log(wxString::Format("Added thermocouple: OID=%d type=%s", tc->getOid(), typeName),
            wxColour(0, 128, 0));
        m_thermocouples.push_back(std::move(tc));
    } else {
        Log("Failed to add thermocouple", *wxRED);
        std::lock_guard<std::mutex> tcLock(m_tcMutex);
        m_tcReadings.pop_back();
    }
}

void KlipperFrame::OnAddStepper(wxCommandEvent&) {
    if (!m_connected) return;
    if (m_mcu.isConfigFinalized()) {
        Log("Config already finalized.", *wxRED);
        return;
    }

    wxString stepPin = m_stepPinCtrl->GetValue().Trim();
    wxString dirPin = m_dirPinCtrl->GetValue().Trim();
    if (stepPin.empty() || dirPin.empty()) return;

    auto stepper = std::make_unique<MCU_stepper>(m_mcu);
    stepper->setupPin(stepPin.ToStdString(), dirPin.ToStdString());
    stepper->setupStepDist(40.0, 200, 16); // defaults: 40mm belt, 200 steps, 16 microsteps
    stepper->setupInvertDir(false);

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (stepper->buildConfig()) {
        int row = m_stepperList->GetItemCount();
        m_stepperList->InsertItem(row, wxString::Format("%d", stepper->getOid()));
        m_stepperList->SetItem(row, 1, stepPin);
        m_stepperList->SetItem(row, 2, dirPin);
        m_stepperList->SetItem(row, 3, wxString::Format("%.5f mm", stepper->getStepDist()));
        m_stepperList->SetItem(row, 4, "pending finalize");

        Log(wxString::Format("Added stepper: OID=%d step=%s dir=%s",
            stepper->getOid(), stepPin, dirPin), wxColour(0, 128, 0));
        m_stepperObjs.push_back(std::move(stepper));
    } else {
        Log(wxString::Format("Failed to add stepper %s/%s", stepPin, dirPin), *wxRED);
    }
}

void KlipperFrame::OnSendGcode(wxCommandEvent&) {
    if (!m_connected || !m_gcode) return;

    wxString cmd = m_gcodeInput->GetValue().Trim();
    if (cmd.empty()) return;

    Log(wxString::Format("> %s", cmd), wxColour(0, 0, 160));

    std::lock_guard<std::mutex> lock(m_mcuMutex);

    // Advance print_time base to current MCU time so steps are in the future
    if (m_toolhead) {
        double now = m_mcu.getClockSync().estimatedPrintTime() + 0.25;
        if (now > m_toolhead->getNextPrintTime())
            m_toolhead->setNextPrintTime(now);
    }

    if (m_gcode->executeLine(cmd.ToStdString())) {
        // Flush and generate steps
        m_toolhead->flush();
        m_toolhead->generateSteps();

        wxString msg = m_gcode->getLastMessage();
        if (!msg.empty()) {
            Log(wxString::Format("  %s", msg), wxColour(0, 128, 0));
        }

        Vec3 pos = m_toolhead->getPosition();
        m_motionStatus->SetLabel(wxString::Format(
            "Motion: X=%.3f Y=%.3f Z=%.3f | F=%.0f mm/min",
            pos.x, pos.y, pos.z, m_gcode->getFeedrate() * 60.0));
    } else {
        Log(wxString::Format("  Error: %s", m_gcode->getLastMessage()), *wxRED);
    }
}

void KlipperFrame::OnHomeAll(wxCommandEvent&) {
    if (!m_connected || !m_gcode) return;

    Log("Sending G28 (Home All)...", wxColour(0, 0, 160));

    std::lock_guard<std::mutex> lock(m_mcuMutex);
    if (m_gcode->executeLine("G28")) {
        Log("Homing complete", wxColour(0, 128, 0));
        Vec3 pos = m_toolhead->getPosition();
        m_motionStatus->SetLabel(wxString::Format(
            "Motion: X=%.3f Y=%.3f Z=%.3f | Homed",
            pos.x, pos.y, pos.z));
    } else {
        Log(wxString::Format("Homing failed: %s", m_gcode->getLastMessage()), *wxRED);
    }
}

void KlipperFrame::OnJog(wxCommandEvent& evt) {
    if (!m_connected || !m_gcode) return;

    double distXY = 0, distZ = 0, speed = 0;
    if (!m_jogDistXY->GetValue().ToDouble(&distXY) || distXY <= 0) {
        Log("Invalid XY distance", *wxRED); return;
    }
    if (!m_jogDistZ->GetValue().ToDouble(&distZ) || distZ <= 0) {
        Log("Invalid Z distance", *wxRED); return;
    }
    if (!m_jogSpeed->GetValue().ToDouble(&speed) || speed <= 0) {
        Log("Invalid speed", *wxRED); return;
    }

    std::string axis;
    double dist = 0;
    switch (evt.GetId()) {
        case ID_JOG_XP: axis = "X"; dist =  distXY; break;
        case ID_JOG_XN: axis = "X"; dist = -distXY; break;
        case ID_JOG_YP: axis = "Y"; dist =  distXY; break;
        case ID_JOG_YN: axis = "Y"; dist = -distXY; break;
        case ID_JOG_ZP: axis = "Z"; dist =  distZ;  break;
        case ID_JOG_ZN: axis = "Z"; dist = -distZ;  break;
        default: return;
    }

    // Use G91 (relative) → G1 → G90 (absolute) sequence
    std::string gcode = "G91\nG1 " + axis + std::to_string(dist)
                        + " F" + std::to_string(static_cast<int>(speed)) + "\nG90";

    Log(wxString::Format("> Jog %s%+.3f mm @ F%d", axis, dist, static_cast<int>(speed)),
        wxColour(0, 0, 160));

    std::lock_guard<std::mutex> lock(m_mcuMutex);

    // Check endstop before moving: refuse to move toward a triggered endstop
    {
        int axisIdx = (axis == "X") ? 0 : (axis == "Y") ? 1 : 2;
        if (axisIdx < static_cast<int>(m_endstopObjs.size())) {
            bool triggered = false;
            if (m_endstopObjs[axisIdx]->queryState(triggered) && triggered) {
                // Check if moving toward endstop (toward position_endstop)
                double endstopPos = 0.0;
                if (axisIdx < static_cast<int>(m_rails.size())) {
                    endstopPos = m_rails[axisIdx]->getPositionEndstop();
                }
                Vec3 curPos = m_toolhead->getPosition();
                double curAxisPos = (axisIdx == 0) ? curPos.x : (axisIdx == 1) ? curPos.y : curPos.z;
                bool movingToward = (endstopPos <= curAxisPos) ? (dist < 0) : (dist > 0);
                if (movingToward) {
                    Log(wxString::Format("Jog blocked: %s endstop is triggered! Move away first.", axis),
                        *wxRED);
                    return;
                }
            }
        }
    }

    // Advance print_time base to current MCU time so steps are in the future
    double now = m_mcu.getClockSync().estimatedPrintTime() + 0.25;
    if (now > m_toolhead->getNextPrintTime())
        m_toolhead->setNextPrintTime(now);

    if (m_gcode->executeBlock(gcode) > 0) {
        m_toolhead->flush();
        m_toolhead->generateSteps();

        Vec3 pos = m_toolhead->getPosition();
        m_motionStatus->SetLabel(wxString::Format(
            "Motion: X=%.3f Y=%.3f Z=%.3f | F=%.0f mm/min",
            pos.x, pos.y, pos.z, speed));
    } else {
        Log(wxString::Format("  Jog failed: %s", m_gcode->getLastMessage()), *wxRED);
    }
}

void KlipperFrame::OnLoadConfig(wxCommandEvent&) {
    if (!m_connected) return;
    if (m_mcu.isConfigFinalized()) {
        Log("Config already finalized. Disconnect and reconnect to load a new config.", *wxRED);
        return;
    }

    wxFileDialog dlg(this, "Open Klipper Config", "", "",
                     "Klipper Config (*.cfg)|*.cfg|All Files (*.*)|*.*",
                     wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;

    std::string path = dlg.GetPath().ToStdString();
    Log(wxString::Format("Loading config: %s", path), wxColour(0, 0, 160));
    wxBusyCursor wait;

    std::lock_guard<std::mutex> lock(m_mcuMutex);

    auto result = std::make_unique<ConfigResult>(KlipperConfig::load(m_mcu, path));
    if (!result->ok()) {
        Log(wxString::Format("Config load failed: %s", result->lastError), *wxRED);
        return;
    }

    // Log warnings
    for (auto& w : result->warnings) {
        Log(wxString::Format("  Warning: %s", w), wxColour(200, 100, 0));
    }

    // Transfer steppers to GUI lists
    for (auto& si : result->steppers) {
        if (!si.stepper) continue;

        int row = m_stepperList->GetItemCount();
        m_stepperList->InsertItem(row, wxString::Format("%d", si.stepper->getOid()));
        m_stepperList->SetItem(row, 1, si.stepper->getStepPinName());
        m_stepperList->SetItem(row, 2, si.stepper->getDirPinName());
        m_stepperList->SetItem(row, 3, wxString::Format("%.5f mm", si.stepper->getStepDist()));
        m_stepperList->SetItem(row, 4, "pending finalize");

        Log(wxString::Format("  [%s] stepper OID=%d step=%s dir=%s dist=%.5fmm",
            si.name, si.stepper->getOid(),
            si.stepper->getStepPinName(), si.stepper->getDirPinName(),
            si.stepper->getStepDist()), wxColour(0, 128, 0));

        if (si.endstop) {
            m_endstopObjs.push_back(std::move(si.endstop));
        }
        if (si.rail) {
            m_rails.push_back(std::move(si.rail));
        }
        m_stepperObjs.push_back(std::move(si.stepper));
    }

    // Transfer ADC inputs
    for (auto& ai : result->adcInputs) {
        if (!ai.adc) continue;

        size_t adcIdx = m_adcInputs.size();
        {
            std::lock_guard<std::mutex> adcLock(m_adcMutex);
            m_adcReadings.push_back({0.0, 0.0});
        }

        ai.adc->setupAdcCallback([this, adcIdx](double readTime, double value) {
            std::lock_guard<std::mutex> adcLock(m_adcMutex);
            if (adcIdx < m_adcReadings.size()) {
                m_adcReadings[adcIdx] = {readTime, value};
            }
        });

        int row = m_adcList->GetItemCount();
        m_adcList->InsertItem(row, wxString::Format("%d", ai.adc->getOid()));
        m_adcList->SetItem(row, 1, ai.pin);
        m_adcList->SetItem(row, 2, "---");
        m_adcList->SetItem(row, 3, "---");
        m_adcList->SetItem(row, 4, "pending finalize");

        Log(wxString::Format("  [%s] ADC OID=%d pin=%s", ai.name, ai.adc->getOid(), ai.pin),
            wxColour(0, 128, 0));
        m_adcInputs.push_back(std::move(ai.adc));
    }

    // Transfer digital outputs
    for (auto& di : result->digitalOuts) {
        if (!di.dout) continue;

        int row = m_gpioList->GetItemCount();
        m_gpioList->InsertItem(row, wxString::Format("%d", di.dout->getOid()));
        m_gpioList->SetItem(row, 1, di.pin);
        m_gpioList->SetItem(row, 2, "LOW");
        m_gpioList->SetItem(row, 3, "pending finalize");

        Log(wxString::Format("  [%s] digital out OID=%d pin=%s", di.name, di.dout->getOid(), di.pin),
            wxColour(0, 128, 0));
        m_digitalOuts.push_back(std::move(di.dout));
    }

    // Create TMC5160 drivers (group by SPI bus for daisy chain sharing)
    if (!result->tmcConfigs.empty()) {
        struct SpiBusEntry { std::string csPin; std::string spiBus; MCU_SPI* spi; };
        std::vector<SpiBusEntry> spiBuses;

        for (auto& tc : result->tmcConfigs) {
            // Find or create SPI for this (csPin, spiBus) combination
            MCU_SPI* spi = nullptr;
            for (auto& entry : spiBuses) {
                if (entry.csPin == tc.csPin && entry.spiBus == tc.spiBus) {
                    spi = entry.spi;
                    break;
                }
            }

            if (!spi) {
                auto newSpi = std::make_unique<MCU_SPI>(m_mcu);
                newSpi->setupPin(tc.csPin, false);
                newSpi->setupBus(tc.spiBus, 3, 4000000);  // SPI mode 3, 4MHz
                if (!newSpi->buildConfig()) {
                    Log(wxString::Format("Failed to create SPI for TMC bus=%s cs=%s",
                        tc.spiBus, tc.csPin), *wxRED);
                    continue;
                }
                spi = newSpi.get();
                spiBuses.push_back({tc.csPin, tc.spiBus, spi});

                int row = m_busList->GetItemCount();
                m_busList->InsertItem(row, "SPI-TMC");
                m_busList->SetItem(row, 1, wxString::Format("%d", newSpi->getOid()));
                m_busList->SetItem(row, 2, tc.spiBus);
                m_busList->SetItem(row, 3, tc.csPin);
                m_busList->SetItem(row, 4, "pending finalize");

                m_spiDevices.push_back(std::move(newSpi));
            }

            auto driver = std::make_unique<TMC5160>(tc.name);
            driver->setSpi(spi, tc.chainPosition, tc.chainLength);
            driver->setCurrent(tc.runCurrent, tc.holdCurrent, tc.senseResistor);
            driver->setMicrosteps(tc.microsteps, tc.interpolate);
            driver->setStealthChop(tc.stealthChop);

            Log(wxString::Format("  [TMC5160:%s] pos=%d/%d run=%.1fA hold=%.1fA ms=%d%s",
                tc.name, tc.chainPosition, tc.chainLength,
                tc.runCurrent, tc.holdCurrent, tc.microsteps,
                tc.stealthChop ? " StealthChop" : " SpreadCycle"),
                wxColour(0, 128, 0));

            m_tmcDrivers.push_back(std::move(driver));
        }

        Log(wxString::Format("  TMC5160: %zu driver(s) on %zu SPI bus(es)",
            m_tmcDrivers.size(), spiBuses.size()), wxColour(0, 128, 0));

        // Build register UI for each driver
        buildTmcDriverTabs();
    }

    // Store printer settings
    Log(wxString::Format("  Printer: %s vel=%.0f accel=%.0f scv=%.1f",
        result->kinematics, result->maxVelocity, result->maxAccel,
        result->squareCornerVelocity), wxColour(0, 128, 0));

    m_configResult = std::move(result);
    m_configPathLabel->SetLabel(wxString::Format("Config: %s", path));
    m_configPathLabel->SetForegroundColour(wxColour(0, 100, 0));

    Log(wxString::Format("Config loaded: %zu steppers, %zu ADCs, %zu digital outs, %zu TMC drivers",
        m_configResult->steppers.size(), m_configResult->adcInputs.size(),
        m_configResult->digitalOuts.size(), m_configResult->tmcConfigs.size()),
        wxColour(0, 128, 0));

    // Disable load config after loading (can only load once before finalize)
    m_btnLoadConfig->Enable(false);
}

void KlipperFrame::OnPrintLoad(wxCommandEvent&) {
    if (!m_connected || !m_gcode) return;

    wxFileDialog dlg(this, "Open G-code File", "", "",
                     "G-code Files (*.gcode;*.gco;*.g)|*.gcode;*.gco;*.g|All Files (*.*)|*.*",
                     wxFD_OPEN | wxFD_FILE_MUST_EXIST);
    if (dlg.ShowModal() == wxID_CANCEL)
        return;

    std::string path = dlg.GetPath().ToStdString();

    std::ifstream file(path);
    if (!file.is_open()) {
        Log("Failed to open G-code file: " + wxString(path), *wxRED);
        return;
    }

    m_printLines.clear();
    std::string line;
    while (std::getline(file, line)) {
        // Strip \r from Windows line endings
        if (!line.empty() && line.back() == '\r')
            line.pop_back();
        m_printLines.push_back(line);
    }

    m_printFilePath = path;
    m_printCurrentLine = 0;

    // Update UI
    wxString filename = wxFileName(path).GetFullName();
    m_printFileLabel->SetLabel(wxString::Format("File: %s (%zu lines)", filename, m_printLines.size()));
    m_printFileLabel->SetForegroundColour(wxColour(0, 100, 0));
    m_printProgress->SetValue(0);
    m_printStatus->SetLabel(wxString::Format("Print: loaded %zu lines, ready to start", m_printLines.size()));
    m_printStatus->SetForegroundColour(wxColour(0, 0, 160));

    // Show preview (first 200 lines)
    m_printGcodeView->Clear();
    size_t previewLines = std::min(m_printLines.size(), size_t(200));
    for (size_t i = 0; i < previewLines; ++i) {
        m_printGcodeView->AppendText(wxString::Format("%5zu: %s\n", i + 1, m_printLines[i]));
    }
    if (m_printLines.size() > 200) {
        m_printGcodeView->AppendText(wxString::Format("... (%zu more lines)\n",
            m_printLines.size() - 200));
    }

    m_btnPrintStart->Enable(true);
    m_btnPrintPause->Enable(false);
    m_btnPrintStop->Enable(false);

    Log(wxString::Format("Loaded G-code: %s (%zu lines)", filename, m_printLines.size()),
        wxColour(0, 128, 0));
}

void KlipperFrame::OnPrintStart(wxCommandEvent&) {
    if (!m_connected || !m_gcode || m_printLines.empty()) return;

    if (m_printing) {
        // Resume from pause
        m_printPaused = false;
        m_btnPrintPause->SetLabel("Pause");
        Log("Print resumed", wxColour(0, 128, 0));
        return;
    }

    m_printing = true;
    m_printPaused = false;
    m_printStop = false;
    m_printCurrentLine = 0;
    m_printLastHighlightLine = 0;
    m_printErrorCount = 0;

    m_btnPrintStart->Enable(false);
    m_btnPrintPause->Enable(true);
    m_btnPrintStop->Enable(true);
    m_btnPrintLoad->Enable(false);

    Log(wxString::Format("Starting print: %s (%zu lines)", m_printFilePath, m_printLines.size()),
        wxColour(0, 128, 0));

    // Start print thread
    if (m_printThread.joinable()) m_printThread.join();
    m_printThread = std::thread(&KlipperFrame::PrintThread, this);
}

void KlipperFrame::OnPrintPause(wxCommandEvent&) {
    if (!m_printing) return;

    m_printPaused = !m_printPaused;
    m_btnPrintPause->SetLabel(m_printPaused ? "Resume" : "Pause");
    m_btnPrintStart->Enable(m_printPaused.load());
    Log(m_printPaused ? "Print paused" : "Print resumed", wxColour(200, 100, 0));
}

void KlipperFrame::OnPrintStop(wxCommandEvent&) {
    if (!m_printing) return;

    m_printStop = true;
    m_printPaused = false;  // unblock if paused
    Log("Print stop requested...", *wxRED);
}

void KlipperFrame::PrintThread() {
    // Klipper-style flow control with hysteresis:
    // Fill buffer up to BUFFER_TIME_HIGH, then wait until it drains to BUFFER_TIME_LOW.
    // This gives the MCU a healthy scheduling window and prevents "stepper too far in past".
    constexpr double BUFFER_TIME_START = 1.0;  // initial print_time ahead of MCU
    constexpr double BUFFER_TIME_HIGH  = 2.0;  // pause scheduling when this far ahead
    constexpr double BUFFER_TIME_LOW   = 1.0;  // resume scheduling when buffer drains to this
    constexpr size_t FLUSH_BATCH = 10;
    size_t linesSinceFlush = 0;
    m_printErrorCount = 0;

    // Set initial print_time well ahead of MCU time ONCE
    {
        std::lock_guard<std::mutex> lock(m_mcuMutex);
        m_toolhead->resetSyncState(); // start from idle — first lookaheadFlush will call syncPrintTime
        double initialTime = m_mcu.getClockSync().estimatedPrintTime() + BUFFER_TIME_START;
        m_toolhead->setNextPrintTime(initialTime);
    }

    for (size_t i = 0; i < m_printLines.size(); ++i) {
        // --- Check stop/pause (no mutex needed, these are atomics) ---
        if (m_printStop) break;
        while (m_printPaused && !m_printStop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (m_printStop) break;

        // --- Health check: connection and shutdown (quick lock) ---
        {
            std::lock_guard<std::mutex> lock(m_mcuMutex);
            if (!m_mcu.isConnected()) {
                LogFromThread("Print aborted: MCU disconnected!", *wxRED);
                break;
            }
            if (m_mcu.isShutdown()) {
                LogFromThread(wxString::Format("Print aborted: MCU shutdown - %s",
                    m_mcu.getShutdownMsg()), *wxRED);
                break;
            }
        }

        const auto& line = m_printLines[i];
        m_printCurrentLine = i + 1;

        // Skip empty lines and comments
        if (line.empty() || line[0] == ';' || line[0] == '(') continue;
        auto commentPos = line.find(';');
        std::string cmd = (commentPos != std::string::npos) ? line.substr(0, commentPos) : line;
        while (!cmd.empty() && (cmd.back() == ' ' || cmd.back() == '\t'))
            cmd.pop_back();
        if (cmd.empty()) continue;

        // --- Execute G-code line (short lock) ---
        bool execOk = false;
        {
            std::lock_guard<std::mutex> lock(m_mcuMutex);

            // Safety net: only bump print_time if it has fallen behind MCU time
            // (can happen after a long pause or many non-move commands).
            // Normal flow: print_time advances naturally via move durations.
            double minTime = m_mcu.getClockSync().estimatedPrintTime() + 0.25;
            if (minTime > m_toolhead->getNextPrintTime())
                m_toolhead->setNextPrintTime(minTime);

            execOk = m_gcode->executeLine(cmd);
        }

        if (!execOk) {
            m_printErrorCount++;
            // Only log first 10 errors to avoid flooding
            if (m_printErrorCount <= 10) {
                LogFromThread(wxString::Format("Print error at line %zu: %s",
                    i + 1, m_gcode->getLastMessage()), *wxRED);
            } else if (m_printErrorCount == 11) {
                LogFromThread("(suppressing further print errors...)", wxColour(180, 0, 0));
            }
            continue;
        }

        linesSinceFlush++;

        // --- Flush and generate steps periodically ---
        if (linesSinceFlush >= FLUSH_BATCH || i == m_printLines.size() - 1) {
            linesSinceFlush = 0;
            {
                std::lock_guard<std::mutex> lock(m_mcuMutex);
                m_toolhead->flush();
            }

            // generateSteps() now self-paces via internal clock-gating:
            // it waits (sleeping without mutex) until each step's target clock
            // is within safe MCU 32-bit timer range before sending. This mirrors
            // Python Klipper's serialqueue MIN_REQTIME_DELTA gating.
            // Called WITHOUT mcuMutex so clock sync thread can update during waits.
            m_toolhead->generateSteps();

            // --- Flow control with hysteresis ---
            // Wait until buffer drains to BUFFER_TIME_LOW before sending more.
            for (;;) {
                if (m_printStop) break;

                double ahead;
                {
                    std::lock_guard<std::mutex> lock(m_mcuMutex);
                    if (!m_mcu.isConnected() || m_mcu.isShutdown()) break;
                    ahead = m_toolhead->getNextPrintTime()
                          - m_mcu.getClockSync().estimatedPrintTime();
                }

                if (ahead < BUFFER_TIME_HIGH) break;

                // Buffer is full — wait for MCU to catch up.
                double waitSec = ahead - BUFFER_TIME_LOW;
                int waitMs = std::max(10, std::min(500, static_cast<int>(waitSec * 1000)));
                std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
            }
        }
    }

    // Final flush
    {
        std::lock_guard<std::mutex> lock(m_mcuMutex);
        if (m_mcu.isConnected() && !m_mcu.isShutdown()) {
            m_toolhead->flush();
        }
    }
    // generateSteps handles its own pacing (clock-gating waits internally)
    m_toolhead->generateSteps();
    {
        std::lock_guard<std::mutex> lock(m_mcuMutex);
        m_toolhead->resetSyncState(); // back to idle
    }

    bool stopped = m_printStop.load();
    size_t errorCount = m_printErrorCount.load();
    m_printing = false;
    m_printStop = false;

    if (stopped) {
        LogFromThread("Print stopped by user", *wxRED);
    } else {
        wxString msg = wxString::Format("Print complete! %zu lines", m_printLines.size());
        if (errorCount > 0)
            msg += wxString::Format(" (%zu errors skipped)", errorCount);
        LogFromThread(msg, wxColour(0, 128, 0));
    }

    CallAfter([this, stopped]() {
        m_btnPrintStart->Enable(false);
        m_btnPrintPause->Enable(false);
        m_btnPrintStop->Enable(false);
        m_btnPrintLoad->Enable(true);
        m_btnPrintPause->SetLabel("Pause");
        m_printProgress->SetValue(stopped ? m_printProgress->GetValue() : 100);
        m_printStatus->SetLabel(stopped ? "Print: stopped" : "Print: complete");
        m_printStatus->SetForegroundColour(stopped ? *wxRED : wxColour(0, 128, 0));
    });
}

void KlipperFrame::OnClose(wxCloseEvent& evt) {
    // Stop any active print
    if (m_printing) {
        m_printStop = true;
        m_printPaused = false;
        if (m_printThread.joinable()) m_printThread.join();
    }
    StopClockSync();
    StopPolling();
    m_mcu.disconnect();
    evt.Skip();
}
