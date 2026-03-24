#include <Arduino.h>
#include <JC_Button.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <Updater.h>
#include <WiFiClientSecureBearSSL.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "pins.h"
#if LED_COUNT > 0
extern "C"
{
#include "esp8266_peri.h"
}
#endif

// -----------------------------------------------------------------------------
// VacUBear Firmware
// -----------------------------------------------------------------------------
// Architektur in Kurzform:
// - Eine zeitbasierte Show-Statemachine steuert Pumpen und Ventil.
// - Optionales RGBW-Licht (SK6812) wird synchron zur Show mit Soft-Fade gefahren.
// - Konfiguration liegt persistent in LittleFS (/config.json).
// - Steuerung/Monitoring via MQTT (inkl. Home Assistant Discovery).
// - Lokales Web-Interface fuer WiFi/MQTT/Show/OTA-Parameter.
// - OTA-Update ueber Manifest (Boot + periodischer Hintergrund-Check).
//
// Wichtiger Grundsatz:
// Diese Datei ist der zentrale "Orchestrator". Einzelne Features sind als
// logisch getrennte Funktionsbloecke organisiert, laufen aber im selben loop().

#ifndef ENABLE_SERIAL_LOG
#define ENABLE_SERIAL_LOG 0
#endif

#if ENABLE_SERIAL_LOG
#define LOGI(fmt, ...) Serial.printf("[INFO] " fmt "\n", ##__VA_ARGS__)
#define LOGW(fmt, ...) Serial.printf("[WARN] " fmt "\n", ##__VA_ARGS__)
#define LOGD(fmt, ...) Serial.printf("[DBG] " fmt "\n", ##__VA_ARGS__)
#else
#define LOGI(...)
#define LOGW(...)
#define LOGD(...)
#endif

#define SHOW_LENGTH 10000UL
#define SHOW_NACHLAUF 20000UL

#ifndef FW_VERSION
#define FW_VERSION "0.0.0-dev"
#endif

#ifndef OTA_MANIFEST_URL
#define OTA_MANIFEST_URL ""
#endif

#ifndef LED_COUNT
#define LED_COUNT 0
#endif

#ifndef TELEMETRY_INTERVAL_MS_CFG
#define TELEMETRY_INTERVAL_MS_CFG 300000UL
#endif

// -----------------------------------------------------------------------------
// Timing / Betriebsparameter
// -----------------------------------------------------------------------------
// Hinweis:
// - *_CFG-Werte kommen aus Build-Flags (falls gesetzt).
// - Die restlichen Konstanten sind bewusst fest im Code hinterlegt, damit
//   Hardware-Schutz und Show-Charakter reproduzierbar bleiben.
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000UL;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 30000UL;
static const uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000UL;
static const uint32_t CONFIG_SAVE_DELAY_MS = 2000UL;
static const uint32_t OTA_CHECK_INTERVAL_MS = 86400000UL;
static const uint32_t OTA_HTTP_TIMEOUT_MS = 30000UL;
static const uint32_t OTA_RESTART_DELAY_MS = 2000UL;
// Developer limits (not user editable)
static const uint32_t SHOW_LENGTH_MIN_S = 10UL;
static const uint32_t SHOW_LENGTH_MAX_S = 60UL;
static const uint32_t SHOW_NACHLAUF_MIN_S = 5UL;
static const uint32_t SHOW_LENGTH_MIN_MS = SHOW_LENGTH_MIN_S * 1000UL;
static const uint32_t SHOW_LENGTH_MAX_MS = SHOW_LENGTH_MAX_S * 1000UL;
static const uint32_t SHOW_NACHLAUF_MIN_MS = SHOW_NACHLAUF_MIN_S * 1000UL;
static const uint16_t PUMP_PWM_MAX = 1023;
static const uint16_t PUMP_PWM_FREQ_HZ = 1000;
static const uint32_t PUMP_SOFTSTART_MS = 2500UL;
static const uint32_t PUMP_SOFTSTOP_MS = 1200UL;
static const uint32_t PUMP_PWM_UPDATE_MS = 20UL;
static const uint32_t LED_FADE_IN_MS = 1200UL;
static const uint32_t LED_FADE_OUT_MS = 1200UL;
static const uint32_t LED_AFTER_BELUEFTEN_MS = 5000UL;
static const uint32_t LIGHT_PREVIEW_MS = 5000UL;
static const uint16_t LIGHT_LEVEL_MAX = 1023;
static const uint32_t LIGHT_UPDATE_MS = 20UL;
static const uint32_t BUTTON_LIGHT_TOGGLE_MS = 3000UL;
static const uint32_t BUTTON_AP_MODE_MS = 10000UL;
static const uint32_t BUTTON_BOOT_GUARD_MS = 1200UL;
static const uint32_t BOOT_PULSE_PERIOD_MS = 1600UL;
static const uint16_t BOOT_PULSE_LEVEL_MIN = 32;
static const uint16_t BOOT_PULSE_LEVEL_MAX = 768;
static const uint32_t LIGHT_FEEDBACK_ON_MS = 160UL;
static const uint32_t LIGHT_FEEDBACK_OFF_MS = 140UL;
static const uint8_t LIGHT_FEEDBACK_BLINK_COUNT = 3;
static const uint32_t SK6812_BIT_TOTAL_NS = 1250UL;
static const uint32_t SK6812_T0H_NS = 350UL;
static const uint32_t SK6812_T1H_NS = 700UL;
static const uint32_t SK6812_RESET_US = 90UL;
static const uint32_t TELEMETRY_INTERVAL_MS = TELEMETRY_INTERVAL_MS_CFG;

static const char *CONFIG_FILE = "/config.json";
static const char *DEVICE_PREFIX = "vacubear";
#if LED_COUNT > 0
static const bool HAS_LED_OUTPUT = true;
#else
static const bool HAS_LED_OUTPUT = false;
#endif

// Persistente Anwender-Konfiguration (LittleFS JSON).
struct AppConfig
{
  String wifiSsid;
  String wifiPassword;
  String mqttHost;
  uint16_t mqttPort;
  String mqttUser;
  String mqttPassword;
  String mqttTopic;
  String otaManifestUrl;
  uint32_t showLengthMs;
  uint32_t showNachlaufMs;
  bool lightEnabled;
  uint8_t lightR;
  uint8_t lightG;
  uint8_t lightB;
  uint8_t lightW;

  AppConfig()
      : wifiSsid(""),
        wifiPassword(""),
        mqttHost(""),
        mqttPort(1883),
        mqttUser(""),
        mqttPassword(""),
        mqttTopic(""),
        otaManifestUrl(OTA_MANIFEST_URL),
        showLengthMs(SHOW_LENGTH),
        showNachlaufMs(SHOW_NACHLAUF),
        lightEnabled(true),
        lightR(255),
        lightG(255),
        lightB(255),
        lightW(0)
  {
  }
};

// Laufzeitstatus der Show.
//
// Begriffe:
// - now < fadeInDoneAt -> "FadeIn"      (nur Licht blendet ein)
// - fadeInDoneAt .. endAt        -> "Vakuumieren" (Pumpen AN, Ventil geschlossen)
// - endAt .. openValveAt         -> "Haltezeit"   (Pumpen AUS, Ventil geschlossen)
// - openValveAt .. finishAt      -> "FadeOut"     (Ventil offen, Licht blendet aus)
class ShowStatus
{
public:
  bool isRunning;
  unsigned long fadeInDoneAt;
  unsigned long endAt;
  unsigned long openValveAt;
  unsigned long finishAt;
  bool shouldStart;
  unsigned long showDuration;
  unsigned long showNachlauf;

  ShowStatus()
      : isRunning(false),
        fadeInDoneAt(0),
        endAt(0),
        openValveAt(0),
        finishAt(0),
        shouldStart(false),
        showDuration(SHOW_LENGTH),
        showNachlauf(SHOW_NACHLAUF)
  {
  }
};

// Softstart/Softstop-Regelung fuer Pumpen per PWM.
struct PumpSoftControl
{
  uint16_t currentPwm;
  uint16_t targetPwm;
  unsigned long lastUpdateAt;

  PumpSoftControl()
      : currentPwm(0), targetPwm(0), lastUpdateAt(0)
  {
  }
};

// Soft-Fade-Regelung fuer LED-Ausgang.
struct LightSoftControl
{
  uint16_t currentLevel;
  uint16_t targetLevel;
  unsigned long onUntilAt;
  unsigned long previewUntilAt;
  unsigned long lastUpdateAt;
  uint8_t lastR;
  uint8_t lastG;
  uint8_t lastB;
  uint8_t lastW;
  bool outputInitialized;

  LightSoftControl()
      : currentLevel(0),
        targetLevel(0),
        onUntilAt(0),
        previewUntilAt(0),
        lastUpdateAt(0),
        lastR(0),
        lastG(0),
        lastB(0),
        lastW(0),
        outputInitialized(false)
  {
  }
};

struct LightFeedbackState
{
  bool active;
  bool outputOn;
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t w;
  uint8_t togglesRemaining;
  unsigned long nextToggleAt;

  LightFeedbackState()
      : active(false),
        outputOn(false),
        r(0),
        g(0),
        b(0),
        w(0),
        togglesRemaining(0),
        nextToggleAt(0)
  {
  }
};

// OTA-Zustand inkl. letzter Pruefung und Ergebnis.
struct OtaStatus
{
  String currentVersion;
  String latestVersion;
  String firmwareUrl;
  String releaseNotes;
  String lastError;
  String phase;
  String source;
  String statusText;
  bool updateAvailable;
  bool checkInProgress;
  bool updateInProgress;
  bool rebootPending;
  uint32_t progressBytes;
  uint32_t progressTotal;
  uint8_t progressPercent;
  unsigned long lastCheckAt;
  unsigned long rebootAt;

  OtaStatus()
      : currentVersion(FW_VERSION),
        latestVersion(""),
        firmwareUrl(""),
        releaseNotes(""),
        lastError(""),
        phase("idle"),
        source(""),
        statusText("Bereit"),
        updateAvailable(false),
        checkInProgress(false),
        updateInProgress(false),
        rebootPending(false),
        progressBytes(0),
        progressTotal(0),
        progressPercent(0),
        lastCheckAt(0),
        rebootAt(0)
  {
  }
};

struct ParsedHttpUrl
{
  bool valid;
  bool https;
  String host;
  uint16_t port;
  String path;

  ParsedHttpUrl()
      : valid(false),
        https(false),
        host(""),
        port(0),
        path("/")
  {
  }
};

// -----------------------------------------------------------------------------
// Globale Laufzeitobjekte
// -----------------------------------------------------------------------------
Button button(PIN_TASTER, 25, true);
ShowStatus showStatus;
AppConfig config;
ESP8266WebServer server(80);
DNSServer dnsServer;
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

String apSsid;
String deviceId;
bool captivePortalEnabled = false;
unsigned long lastWifiRetryAt = 0;
unsigned long lastMqttReconnectAt = 0;
unsigned long lastTelemetryAt = 0;
bool lastShowRunningState = false;
bool configSavePending = false;
unsigned long configSaveAt = 0;
String lastPhaseLog = "";

String topicBase;
String topicShowSet;
String topicShowState;
String topicLightSet;
String topicLightState;
String topicLightRgbSet;
String topicLightRgbState;
String topicLightRgbwSet;
String topicLightRgbwState;
String topicCfgShowLengthSet;
String topicCfgShowLengthState;
String topicCfgNachlaufSet;
String topicCfgNachlaufState;
String topicOtaInstall;
String topicOtaState;
String topicAvailability;
String topicTeleState;
PumpSoftControl pumpControl;
LightSoftControl lightControl;
LightFeedbackState lightFeedback;
OtaStatus otaStatus;
bool otaCheckRequested = false;
bool otaUpdateRequested = false;
bool otaBootCheckPending = true;
uint8_t otaLastPublishedProgressPercent = 255;
bool wifiStartupPending = false;
unsigned long wifiStartupAt = 0;
bool bootIndicatorActive = false;
bool apModeIndicatorActive = false;
unsigned long buttonPressedAt = 0;
bool buttonApModeTriggered = false;
bool buttonLightToggleArmed = false;
bool buttonInputUnlocked = false;
unsigned long buttonUnlockAt = 0;
#if LED_COUNT > 0
uint32_t lightPinMask = (1UL << PIN_LED);
#endif

// -----------------------------------------------------------------------------
// Funktionsprototypen
// -----------------------------------------------------------------------------
void startShow(void);
void stopShow(void);
void handleButtonInput(void);
void handleShow(void);
void setupPumpControl(void);
void setPumpTarget(bool enabled);
void updatePumpControl(void);
void applyPumpPwm(uint16_t pwm);
void setupLightControl(void);
void updateLightControl(void);
void setLightTargetLevel(uint16_t level);
void lightDriverBegin(void);
void lightDriverShow(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void applyIndicatorColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w, bool force = false);
void applyLightOutput(bool force = false);
void applyBootIndicator(void);
void requestLightPreview(uint32_t durationMs = LIGHT_PREVIEW_MS);
bool isLightPreviewActive(unsigned long now);
void requestLightBlinkFeedback(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t blinkCount = LIGHT_FEEDBACK_BLINK_COUNT);
bool updateLightFeedback(void);
void otaLoop(void);
bool otaCheckForUpdate(const char *reason);
bool otaApplyUpdate(void);
void setSafeOutputState(void);
int compareVersionStrings(const String &lhs, const String &rhs);
void updateOtaDerivedState(const String &latestVersion, const String &firmwareUrl, const String &releaseNotes);
void otaResetProgress(const String &phase, const String &source, const String &statusText = "");
void otaSetPhase(const String &phase, const String &statusText = "");
void otaSetProgress(size_t current, size_t total);
void otaScheduleRestart(const String &statusText);
void otaFinalizeFailure(const String &errorText);
String formatHttpClientError(int httpCode, BearSSL::WiFiClientSecure *secureClient = nullptr);
bool parseHttpUrl(const String &url, ParsedHttpUrl &parsed);
bool configureSecureClientForUrl(BearSSL::WiFiClientSecure &secureClient, const String &url, String &tlsNote);
void setupWiFi(void);
void setupWebServer(void);
void setupMqtt(void);
void handleWiFiStartup(void);
void mqttLoop(void);
void mqttEnsureConnected(void);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void updateMqttTopics(void);
void clearStaleCommandTopics(void);
void publishAvailability(const char *state, bool retained = true);
void publishShowState(bool retained = true);
void publishLightState(bool retained = true);
void publishConfigState(bool retained = true);
void publishOtaUpdateState(bool retained = true);
void publishDiscovery(void);
void publishUpdateDiscovery(void);
void publishSensorDiscovery(void);
void publishTelemetry(bool force = false);
const char *getShowPhase(void);

void loadConfig(void);
bool saveConfig(void);
void loadLegacyConfig(const String &raw);
void enterAccessPointMode(const char *reason);
void startAccessPoint(void);
void setupCaptivePortal(void);
void handleRoot(void);
void handleSave(void);
void handleAppJs(void);
void handleCaptiveProbe(void);
void handleApiConfigGet(void);
void handleApiConfigPost(void);
void handleApiWifiScan(void);
void handleApiShowGet(void);
void handleApiShowPost(void);
void handleApiLightGet(void);
void handleLightConfig(void);
void handleNotFound(void);
void handleStatus(void);
void handleOtaStatus(void);
void handleOtaCheck(void);
void handleOtaUpdate(void);
void handleOtaUpload(void);
void handleOtaUploadData(void);
bool handleCaptivePortalRedirect(void);
String htmlEscape(const String &input);
String buildHtmlPage(const String &message = "");
void sendHtmlPage(const String &message = "", int statusCode = 200);
bool isIpAddress(const String &host);
bool isLegacyOtaManifestUrl(const String &url);
String defaultApSsid(void);
String defaultDeviceId(void);
String rgbToHex(uint8_t r, uint8_t g, uint8_t b);
void sanitizeConfig(void);
void applyConfigToRuntime(bool wifiChanged, bool mqttChanged, bool publishStates = true);
void buildLightStatusJson(JsonDocument &doc);
void buildStatusJson(JsonDocument &doc);
void buildOtaStatusJson(JsonDocument &doc);
void buildConfigJson(JsonObject root);
bool parseJsonBody(JsonDocument &doc, String &errorText);
void sendJsonDocument(int statusCode, JsonDocument &doc);
void sendJsonError(int statusCode, const String &errorText);
uint32_t clampU32(uint32_t value, uint32_t minValue, uint32_t maxValue);
uint32_t clampMinU32(uint32_t value, uint32_t minValue);
uint32_t secToMs(uint32_t seconds);
uint32_t msToSec(uint32_t milliseconds);
uint8_t clampU8(int value);
void scheduleConfigSave(void);

void sanitizeConfig()
{
  config.wifiSsid.trim();
  config.wifiPassword.trim();
  config.mqttHost.trim();
  config.mqttUser.trim();
  config.mqttPassword.trim();
  config.mqttTopic.trim();
  config.otaManifestUrl.trim();
  if (config.otaManifestUrl.length() == 0 || isLegacyOtaManifestUrl(config.otaManifestUrl))
  {
    config.otaManifestUrl = String(OTA_MANIFEST_URL);
  }

  if (config.mqttPort == 0)
  {
    config.mqttPort = 1883;
  }
  if (config.mqttTopic.length() == 0 || config.mqttTopic == "vacubear" || config.mqttTopic.startsWith("vacubear/"))
  {
    config.mqttTopic = deviceId;
  }

  config.showLengthMs = clampU32(config.showLengthMs, SHOW_LENGTH_MIN_MS, SHOW_LENGTH_MAX_MS);
  config.showNachlaufMs = clampMinU32(config.showNachlaufMs, SHOW_NACHLAUF_MIN_MS);
}

void applyConfigToRuntime(bool wifiChanged, bool mqttChanged, bool publishStates)
{
  showStatus.showDuration = config.showLengthMs;
  showStatus.showNachlauf = config.showNachlaufMs;

  if (wifiChanged)
  {
    if (config.wifiSsid.length() == 0)
    {
      startAccessPoint();
      if (!captivePortalEnabled)
      {
        setupCaptivePortal();
      }
    }
    else
    {
      WiFi.mode(captivePortalEnabled ? WIFI_AP_STA : WIFI_STA);
      WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
      wifiStartupPending = true;
      wifiStartupAt = millis();
      lastWifiRetryAt = wifiStartupAt;
      bootIndicatorActive = HAS_LED_OUTPUT;
      apModeIndicatorActive = false;
    }
  }

  if (mqttChanged)
  {
    updateMqttTopics();
    if (mqttClient.connected())
    {
      mqttClient.disconnect();
    }
    lastMqttReconnectAt = 0;
  }

  if (publishStates)
  {
    publishConfigState(true);
    if (HAS_LED_OUTPUT)
    {
      publishLightState(true);
    }
    publishTelemetry(true);
  }
}

void buildLightStatusJson(JsonDocument &doc)
{
  unsigned long now = millis();
  doc["enabled"] = config.lightEnabled;
  doc["preview_remaining_ms"] = isLightPreviewActive(now) ? (uint32_t)(lightControl.previewUntilAt - now) : 0;
  doc["show_running"] = showStatus.isRunning;
  JsonObject color = doc["color"].to<JsonObject>();
  color["r"] = config.lightR;
  color["g"] = config.lightG;
  color["b"] = config.lightB;
  color["w"] = config.lightW;
}

void buildStatusJson(JsonDocument &status)
{
  status["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  status["sta_ip"] = WiFi.localIP().toString();
  status["ap_enabled"] = captivePortalEnabled;
  status["ap_ip"] = WiFi.softAPIP().toString();
  status["show_running"] = showStatus.isRunning;
  status["show_length"] = config.showLengthMs;
  status["show_nachlauf"] = config.showNachlaufMs;
  status["pump_pwm"] = pumpControl.currentPwm;
  status["pump_target_pwm"] = pumpControl.targetPwm;
  if (HAS_LED_OUTPUT)
  {
    unsigned long now = millis();
    status["light_enabled"] = config.lightEnabled;
    status["led_count"] = LED_COUNT;
    status["light_level"] = lightControl.currentLevel;
    status["light_target_level"] = lightControl.targetLevel;
    status["light_on_until_at"] = lightControl.onUntilAt;
    status["light_preview_remaining_ms"] = isLightPreviewActive(now) ? (uint32_t)(lightControl.previewUntilAt - now) : 0;

    JsonObject color = status["light"].to<JsonObject>();
    color["r"] = config.lightR;
    color["g"] = config.lightG;
    color["b"] = config.lightB;
    color["w"] = config.lightW;
  }

  JsonObject ota = status["ota"].to<JsonObject>();
  ota["current_version"] = otaStatus.currentVersion;
  ota["latest_version"] = otaStatus.latestVersion;
  ota["update_available"] = otaStatus.updateAvailable;
  ota["check_in_progress"] = otaStatus.checkInProgress;
  ota["update_in_progress"] = otaStatus.updateInProgress;
  ota["phase"] = otaStatus.phase;
  ota["source"] = otaStatus.source;
  ota["status_text"] = otaStatus.statusText;
  ota["reboot_pending"] = otaStatus.rebootPending;
  ota["progress_bytes"] = otaStatus.progressBytes;
  ota["progress_total"] = otaStatus.progressTotal;
  ota["progress_percent"] = otaStatus.progressPercent;
  ota["last_error"] = otaStatus.lastError;
  ota["last_check_ms"] = otaStatus.lastCheckAt;
  ota["reboot_at_ms"] = otaStatus.rebootAt;
}

void buildOtaStatusJson(JsonDocument &status)
{
  status["configured"] = config.otaManifestUrl.length() > 0;
  status["manifest_url"] = config.otaManifestUrl;
  status["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  status["current_version"] = otaStatus.currentVersion;
  status["latest_version"] = otaStatus.latestVersion;
  status["firmware_url"] = otaStatus.firmwareUrl;
  status["release_notes"] = otaStatus.releaseNotes;
  status["update_available"] = otaStatus.updateAvailable;
  status["check_in_progress"] = otaStatus.checkInProgress;
  status["update_in_progress"] = otaStatus.updateInProgress;
  status["phase"] = otaStatus.phase;
  status["source"] = otaStatus.source;
  status["status_text"] = otaStatus.statusText;
  status["reboot_pending"] = otaStatus.rebootPending;
  status["progress_bytes"] = otaStatus.progressBytes;
  status["progress_total"] = otaStatus.progressTotal;
  status["progress_percent"] = otaStatus.progressPercent;
  status["last_error"] = otaStatus.lastError;
  status["last_check_ms"] = otaStatus.lastCheckAt;
  status["reboot_at_ms"] = otaStatus.rebootAt;
  status["interval_ms"] = OTA_CHECK_INTERVAL_MS;
}

void publishOtaUpdateState(bool retained)
{
  if (!mqttClient.connected())
  {
    return;
  }

  JsonDocument doc;
  const bool installBusy = otaStatus.updateInProgress || otaStatus.rebootPending;
  const bool hasPendingUpdate = otaStatus.updateAvailable || installBusy;
  const String effectiveLatestVersion = hasPendingUpdate && otaStatus.latestVersion.length() > 0
                                            ? otaStatus.latestVersion
                                            : otaStatus.currentVersion;

  doc["installed_version"] = otaStatus.currentVersion;
  doc["latest_version"] = effectiveLatestVersion;
  doc["title"] = "VacUBear Firmware";
  doc["release_summary"] = otaStatus.releaseNotes;
  doc["in_progress"] = installBusy;
  if (installBusy)
  {
    doc["update_percentage"] = otaStatus.rebootPending ? 100 : otaStatus.progressPercent;
  }

  String payload;
  serializeJson(doc, payload);
  mqttClient.publish(topicOtaState.c_str(), payload.c_str(), retained);
  LOGD("MQTT TX ota_update_state=%s", payload.c_str());
}

void buildConfigJson(JsonObject root)
{
  root["device_id"] = deviceId;
  JsonObject wifi = root["wifi"].to<JsonObject>();
  wifi["ssid"] = config.wifiSsid;
  wifi["password_set"] = config.wifiPassword.length() > 0;

  JsonObject mqtt = root["mqtt"].to<JsonObject>();
  mqtt["host"] = config.mqttHost;
  mqtt["port"] = config.mqttPort;
  mqtt["user"] = config.mqttUser;
  mqtt["password_set"] = config.mqttPassword.length() > 0;
  mqtt["topic"] = config.mqttTopic;

  JsonObject ota = root["ota"].to<JsonObject>();
  ota["manifest_url"] = config.otaManifestUrl;

  JsonObject show = root["show"].to<JsonObject>();
  show["length_ms"] = config.showLengthMs;
  show["nachlauf_ms"] = config.showNachlaufMs;
  show["length_s"] = msToSec(config.showLengthMs);
  show["nachlauf_s"] = msToSec(config.showNachlaufMs);

  JsonObject light = root["light"].to<JsonObject>();
  light["enabled"] = config.lightEnabled;
  light["r"] = config.lightR;
  light["g"] = config.lightG;
  light["b"] = config.lightB;
  light["w"] = config.lightW;
}

bool parseJsonBody(JsonDocument &doc, String &errorText)
{
  String raw = server.arg("plain");
  if (raw.length() == 0)
  {
    errorText = "Leerer JSON-Body";
    return false;
  }

  DeserializationError err = deserializeJson(doc, raw);
  if (err)
  {
    errorText = String("JSON ungueltig: ") + err.c_str();
    return false;
  }
  return true;
}

bool requestBodyLooksLikeJson()
{
  String raw = server.arg("plain");
  for (size_t i = 0; i < raw.length(); i++)
  {
    char c = raw[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n')
    {
      continue;
    }
    return c == '{' || c == '[';
  }
  return false;
}

void sendJsonDocument(int statusCode, JsonDocument &doc)
{
  String out;
  serializeJson(doc, out);
  server.send(statusCode, "application/json", out);
}

void sendJsonError(int statusCode, const String &errorText)
{
  JsonDocument doc;
  doc["ok"] = false;
  doc["error"] = errorText;
  sendJsonDocument(statusCode, doc);
}

void setup()
{
  // setup() initialisiert nur. Die eigentliche Ablaufsteuerung passiert in loop().
  Serial.begin(115200);
  LOGI("Booting VacUBear firmware");
  otaStatus.currentVersion = String(FW_VERSION);

  pinMode(PIN_PUMPE1, OUTPUT);
  pinMode(PIN_PUMPE2, OUTPUT);
  pinMode(PIN_VENTIL, OUTPUT);
  setupPumpControl();
  setupLightControl();
  digitalWrite(PIN_VENTIL, LOW);

  button.begin();
  buttonInputUnlocked = false;
  buttonUnlockAt = millis() + BUTTON_BOOT_GUARD_MS;

  if (!LittleFS.begin())
  {
    LOGW("LittleFS mount failed");
  }

  deviceId = defaultDeviceId();
  LOGI("Device ID: %s", deviceId.c_str());
  loadConfig();
  setupWiFi();
  setupWebServer();
  setupMqtt();

  lastShowRunningState = showStatus.isRunning;
  LOGI("Setup complete");
}

void loop()
{
  // Hauptreihenfolge:
  // 1) lokale Eingaben einlesen
  // 2) Show/Pumpen/Licht zeitbasiert fortschreiben
  // 3) Netzwerkdienste (HTTP/DNS/WiFi/MQTT/OTA) bedienen
  // 4) verzögertes Speichern ausfuehren
  handleButtonInput();

  handleShow();
  updatePumpControl();
  updateLightControl();

#if ENABLE_SERIAL_LOG
  const char *phase = getShowPhase();
  if (String(phase) != lastPhaseLog)
  {
    lastPhaseLog = phase;
    LOGI("Show phase changed -> %s", phase);
  }
#endif

  if (showStatus.isRunning != lastShowRunningState)
  {
    lastShowRunningState = showStatus.isRunning;
    if (mqttClient.connected())
    {
      publishShowState(true);
      if (HAS_LED_OUTPUT)
      {
        publishLightState(true);
      }
      publishTelemetry(true);
    }
  }

  server.handleClient();

  if (captivePortalEnabled)
  {
    dnsServer.processNextRequest();
  }

  handleWiFiStartup();

  if (!wifiStartupPending &&
      WiFi.status() != WL_CONNECTED &&
      config.wifiSsid.length() > 0)
  {
    if (millis() - lastWifiRetryAt > WIFI_RETRY_INTERVAL_MS)
    {
      lastWifiRetryAt = millis();
      WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
      LOGI("Retrying WiFi STA connection...");
    }
  }

  mqttLoop();

  if (mqttClient.connected())
  {
    publishTelemetry(false);
  }

  otaLoop();

  if (configSavePending && millis() >= configSaveAt)
  {
    saveConfig();
    configSavePending = false;
  }
}

void handleButtonInput()
{
  button.read();

  if (!buttonInputUnlocked)
  {
    if (millis() < buttonUnlockAt || button.isPressed())
    {
      return;
    }
    buttonInputUnlocked = true;
    buttonPressedAt = 0;
    buttonApModeTriggered = false;
    buttonLightToggleArmed = false;
    LOGI("Button input unlocked after boot guard");
    return;
  }

  if (button.wasPressed())
  {
    buttonPressedAt = button.lastChange();
    buttonApModeTriggered = false;
    buttonLightToggleArmed = false;
    LOGI("Button pressed");
  }

  if (button.isPressed() &&
      !buttonApModeTriggered &&
      !buttonLightToggleArmed &&
      button.pressedFor(BUTTON_LIGHT_TOGGLE_MS))
  {
    buttonLightToggleArmed = true;
    LOGI("Button long press armed for light toggle");
  }

  if (button.isPressed() &&
      !buttonApModeTriggered &&
      button.pressedFor(BUTTON_AP_MODE_MS))
  {
    buttonApModeTriggered = true;
    enterAccessPointMode("button-longpress");
    return;
  }

  if (!button.wasReleased())
  {
    return;
  }

  unsigned long pressDurationMs = (buttonPressedAt > 0) ? (button.lastChange() - buttonPressedAt) : 0;
  LOGI("Button released after %lu ms", pressDurationMs);
  (void)pressDurationMs;

  if (buttonApModeTriggered)
  {
    return;
  }

  if (buttonLightToggleArmed)
  {
    buttonLightToggleArmed = false;
    if (HAS_LED_OUTPUT && showStatus.isRunning)
    {
      config.lightEnabled = !config.lightEnabled;
      scheduleConfigSave();
      publishLightState(true);
      publishTelemetry(true);
      if (config.lightEnabled)
      {
        requestLightBlinkFeedback(0, 255, 0, 0);
      }
      else
      {
        requestLightBlinkFeedback(255, 0, 0, 0);
      }
      LOGI("Show lighting toggled via long press -> %s", config.lightEnabled ? "on" : "off");
    }
    else
    {
      LOGI("Long press ignored: show not running or LED output disabled");
    }
    return;
  }

  if (showStatus.isRunning)
  {
    stopShow();
  }
  else
  {
    startShow();
  }
}

void startShow()
{
  // Kein direkter Start hier: wir setzen nur ein Start-Flag.
  // handleShow() uebernimmt dann atomar die Zeitstempelberechnung.
  if (showStatus.isRunning)
  {
    LOGD("startShow ignored: already running");
    return;
  }
  LOGI("startShow requested");
  showStatus.shouldStart = true;
}

void stopShow()
{
  // Stop erzwingt den fruehen Uebergang in die Belueften-/FadeOut-Phase.
  if (!showStatus.isRunning)
  {
    LOGD("stopShow ignored: not running");
    return;
  }
  LOGI("stopShow requested");
  unsigned long stopAt = millis() + 10;
  unsigned long fadeOutMs = (HAS_LED_OUTPUT && config.lightEnabled) ? LED_FADE_OUT_MS : 0UL;
  showStatus.fadeInDoneAt = stopAt;
  showStatus.endAt = stopAt;
  showStatus.openValveAt = stopAt;
  showStatus.finishAt = stopAt + fadeOutMs;
  lightControl.onUntilAt = showStatus.openValveAt;
  LOGD("Show stop transitions: openValveAt=%lu finishAt=%lu", showStatus.openValveAt, showStatus.finishAt);
}

void handleShow()
{
  // Zentrale Show-Statemachine, die in jedem loop()-Durchlauf getaktet wird.
  // Alle Aktoren werden hier konsistent aus "Zeitfenster + Status" abgeleitet.
  if (showStatus.shouldStart)
  {
    unsigned long now = millis();
    unsigned long fadeInMs = (HAS_LED_OUTPUT && config.lightEnabled) ? LED_FADE_IN_MS : 0UL;
    unsigned long fadeOutMs = (HAS_LED_OUTPUT && config.lightEnabled) ? LED_FADE_OUT_MS : 0UL;
    showStatus.fadeInDoneAt = now + fadeInMs;
    showStatus.endAt = showStatus.fadeInDoneAt + showStatus.showDuration;
    showStatus.openValveAt = showStatus.endAt + showStatus.showNachlauf;
    showStatus.finishAt = showStatus.openValveAt + fadeOutMs;
    showStatus.shouldStart = false;
    showStatus.isRunning = true;
    lightControl.onUntilAt = showStatus.openValveAt;
    setLightTargetLevel(LIGHT_LEVEL_MAX);
    LOGI("Show started: FadeIn=%lu ms, Vakuumieren=%lu ms, Haltezeit=%lu ms, FadeOut=%lu ms",
         fadeInMs, showStatus.showDuration, showStatus.showNachlauf, fadeOutMs);
    LOGD("Show timeline: fadeInDoneAt=%lu endAt=%lu openValveAt=%lu finishAt=%lu",
         showStatus.fadeInDoneAt, showStatus.endAt, showStatus.openValveAt, showStatus.finishAt);
  }

  if (showStatus.isRunning)
  {
    unsigned long now = millis();
    if (now < showStatus.fadeInDoneAt)
    {
      setPumpTarget(false);
      digitalWrite(PIN_VENTIL, LOW);
    }
    else if (now < showStatus.endAt)
    {
      setPumpTarget(true);
      digitalWrite(PIN_VENTIL, HIGH);
    }
    else if (now < showStatus.openValveAt)
    {
      setPumpTarget(false);
      digitalWrite(PIN_VENTIL, HIGH);
    }
    else if (now < showStatus.finishAt)
    {
      setPumpTarget(false);
      digitalWrite(PIN_VENTIL, LOW);
    }
    else
    {
      setPumpTarget(false);
      digitalWrite(PIN_VENTIL, LOW);
      showStatus.isRunning = false;
      LOGI("Show finished");
    }
  }
  else
  {
    setPumpTarget(false);
    digitalWrite(PIN_VENTIL, LOW);
    showStatus.isRunning = false;
  }
}

void setupPumpControl()
{
  // Einheitliche PWM-Konfiguration fuer beide Pumpen.
  analogWriteRange(PUMP_PWM_MAX);
  analogWriteFreq(PUMP_PWM_FREQ_HZ);
  applyPumpPwm(0);
  LOGI("Pump PWM configured: range=%u freq=%u", PUMP_PWM_MAX, PUMP_PWM_FREQ_HZ);
}

void setPumpTarget(bool enabled)
{
  uint16_t newTarget = enabled ? PUMP_PWM_MAX : 0;
  if (newTarget != pumpControl.targetPwm)
  {
    pumpControl.targetPwm = newTarget;
    LOGD("Pump target PWM -> %u", pumpControl.targetPwm);
  }
}

void updatePumpControl()
{
  // Zeitdiskrete Rampe:
  // - Update nur alle PUMP_PWM_UPDATE_MS
  // - step-Berechnung so, dass Rampenzeiten auch bei ganzzahligen Schritten
  //   moeglichst nah an den Sollwerten bleiben.
  unsigned long now = millis();
  if (now - pumpControl.lastUpdateAt < PUMP_PWM_UPDATE_MS)
  {
    return;
  }
  pumpControl.lastUpdateAt = now;

  if (pumpControl.currentPwm == pumpControl.targetPwm)
  {
    return;
  }

  uint16_t upStep = (uint16_t)(((uint32_t)PUMP_PWM_MAX * PUMP_PWM_UPDATE_MS + PUMP_SOFTSTART_MS - 1) / PUMP_SOFTSTART_MS);
  if (upStep == 0)
  {
    upStep = 1;
  }

  uint16_t downStep = (uint16_t)(((uint32_t)PUMP_PWM_MAX * PUMP_PWM_UPDATE_MS + PUMP_SOFTSTOP_MS - 1) / PUMP_SOFTSTOP_MS);
  if (downStep == 0)
  {
    downStep = 1;
  }

  if (pumpControl.currentPwm < pumpControl.targetPwm)
  {
    uint32_t next = pumpControl.currentPwm + upStep;
    if (next > pumpControl.targetPwm)
    {
      next = pumpControl.targetPwm;
    }
    pumpControl.currentPwm = (uint16_t)next;
  }
  else
  {
    int32_t next = (int32_t)pumpControl.currentPwm - downStep;
    if (next < (int32_t)pumpControl.targetPwm)
    {
      next = pumpControl.targetPwm;
    }
    pumpControl.currentPwm = (uint16_t)next;
  }

  applyPumpPwm(pumpControl.currentPwm);
}

void applyPumpPwm(uint16_t pwm)
{
  analogWrite(PIN_PUMPE1, pwm);
  analogWrite(PIN_PUMPE2, pwm);
}

#if LED_COUNT > 0
static inline uint32_t IRAM_ATTR lightCyclesForNs(uint32_t ns)
{
  return (clockCyclesPerMicrosecond() * ns + 999UL) / 1000UL;
}

static inline void IRAM_ATTR lightWaitUntil(uint32_t startCycle, uint32_t targetCycles)
{
  while ((int32_t)(esp_get_cycle_count() - startCycle) < (int32_t)targetCycles)
  {
  }
}

static inline void IRAM_ATTR lightWriteByte(uint8_t value,
                                            uint32_t pinMask,
                                            uint32_t bitTotalCycles,
                                            uint32_t t0hCycles,
                                            uint32_t t1hCycles)
{
  for (uint8_t mask = 0x80; mask != 0; mask >>= 1)
  {
    GPOS = pinMask;
    uint32_t bitStart = esp_get_cycle_count();
    lightWaitUntil(bitStart, (value & mask) ? t1hCycles : t0hCycles);
    GPOC = pinMask;
    lightWaitUntil(bitStart, bitTotalCycles);
  }
}
#endif

void lightDriverBegin(void)
{
#if LED_COUNT > 0
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
#endif
}

void lightDriverShow(uint8_t r, uint8_t g, uint8_t b, uint8_t w)
{
#if LED_COUNT > 0
  const uint32_t bitTotalCycles = lightCyclesForNs(SK6812_BIT_TOTAL_NS);
  const uint32_t t0hCycles = lightCyclesForNs(SK6812_T0H_NS);
  const uint32_t t1hCycles = lightCyclesForNs(SK6812_T1H_NS);

  noInterrupts();
  for (uint16_t i = 0; i < LED_COUNT; i++)
  {
    // SK6812 RGBW verwendet in dieser Hardware die Byte-Reihenfolge GRBW.
    lightWriteByte(g, lightPinMask, bitTotalCycles, t0hCycles, t1hCycles);
    lightWriteByte(r, lightPinMask, bitTotalCycles, t0hCycles, t1hCycles);
    lightWriteByte(b, lightPinMask, bitTotalCycles, t0hCycles, t1hCycles);
    lightWriteByte(w, lightPinMask, bitTotalCycles, t0hCycles, t1hCycles);
  }
  interrupts();
  delayMicroseconds(SK6812_RESET_US);
#else
  (void)r;
  (void)g;
  (void)b;
  (void)w;
#endif
}

void setupLightControl()
{
  // Lichtzustand immer definiert resetten, auch wenn LED_COUNT=0 ist.
  lightControl.currentLevel = 0;
  lightControl.targetLevel = 0;
  lightControl.onUntilAt = 0;
  lightControl.previewUntilAt = 0;
  lightControl.lastUpdateAt = 0;
  lightFeedback.active = false;
  lightFeedback.outputOn = false;
  lightFeedback.togglesRemaining = 0;
  lightFeedback.nextToggleAt = 0;
  lightControl.outputInitialized = false;

#if LED_COUNT > 0
  lightDriverBegin();
  lightDriverShow(0, 0, 0, 0);
  lightControl.outputInitialized = true;
  LOGI("Light output configured: count=%u pin=%d", (unsigned int)LED_COUNT, (int)PIN_LED);
#else
  LOGI("Light output disabled: LED_COUNT=0");
#endif
}

void requestLightPreview(uint32_t durationMs)
{
  if (!HAS_LED_OUTPUT || durationMs == 0)
  {
    return;
  }
  lightControl.previewUntilAt = millis() + durationMs;
  LOGD("Light preview requested: until=%lu", lightControl.previewUntilAt);
}

bool isLightPreviewActive(unsigned long now)
{
  if (lightControl.previewUntilAt == 0)
  {
    return false;
  }
  if ((int32_t)(lightControl.previewUntilAt - now) > 0)
  {
    return true;
  }
  lightControl.previewUntilAt = 0;
  return false;
}

void requestLightBlinkFeedback(uint8_t r, uint8_t g, uint8_t b, uint8_t w, uint8_t blinkCount)
{
  if (!HAS_LED_OUTPUT || blinkCount == 0)
  {
    return;
  }
  lightFeedback.active = true;
  lightFeedback.outputOn = false;
  lightFeedback.r = r;
  lightFeedback.g = g;
  lightFeedback.b = b;
  lightFeedback.w = w;
  // LED-Schreibzugriffe nur im regulaeren Loop-Pfad ausfuehren.
  // Das vermeidet spontane Show()-Transfers aus Button-/MQTT-/Web-Callbacks.
  lightFeedback.togglesRemaining = (uint8_t)(blinkCount * 2U);
  lightFeedback.nextToggleAt = millis();
  LOGD("Light feedback blink started: rgbw=%u,%u,%u,%u blinks=%u", r, g, b, w, blinkCount);
}

void setLightTargetLevel(uint16_t level)
{
  if (level > LIGHT_LEVEL_MAX)
  {
    level = LIGHT_LEVEL_MAX;
  }
  if (level != lightControl.targetLevel)
  {
    lightControl.targetLevel = level;
    LOGD("Light target level -> %u", lightControl.targetLevel);
  }
}

void applyIndicatorColor(uint8_t r, uint8_t g, uint8_t b, uint8_t w, bool force)
{
#if LED_COUNT > 0
  if (!HAS_LED_OUTPUT || !lightControl.outputInitialized)
  {
    return;
  }

  if (!force &&
      r == lightControl.lastR &&
      g == lightControl.lastG &&
      b == lightControl.lastB &&
      w == lightControl.lastW)
  {
    return;
  }

  lightDriverShow(r, g, b, w);

  lightControl.lastR = r;
  lightControl.lastG = g;
  lightControl.lastB = b;
  lightControl.lastW = w;
#else
  (void)r;
  (void)g;
  (void)b;
  (void)w;
  (void)force;
#endif
}

void applyLightOutput(bool force)
{
#if LED_COUNT > 0
  // RGBW aus Konfiguration wird mit aktuellem Soft-Fade-Level skaliert.
  // Dadurch bleibt die eingestellte Farbe erhalten, waehrend nur die
  // Helligkeit zeitlich ein-/ausgeblendet wird.
  if (!HAS_LED_OUTPUT || !lightControl.outputInitialized)
  {
    return;
  }

  uint8_t scaledR = (uint8_t)(((uint32_t)config.lightR * lightControl.currentLevel + (LIGHT_LEVEL_MAX / 2)) / LIGHT_LEVEL_MAX);
  uint8_t scaledG = (uint8_t)(((uint32_t)config.lightG * lightControl.currentLevel + (LIGHT_LEVEL_MAX / 2)) / LIGHT_LEVEL_MAX);
  uint8_t scaledB = (uint8_t)(((uint32_t)config.lightB * lightControl.currentLevel + (LIGHT_LEVEL_MAX / 2)) / LIGHT_LEVEL_MAX);
  uint8_t scaledW = (uint8_t)(((uint32_t)config.lightW * lightControl.currentLevel + (LIGHT_LEVEL_MAX / 2)) / LIGHT_LEVEL_MAX);

  if (!force &&
      scaledR == lightControl.lastR &&
      scaledG == lightControl.lastG &&
      scaledB == lightControl.lastB &&
      scaledW == lightControl.lastW)
  {
    return;
  }

  lightDriverShow(scaledR, scaledG, scaledB, scaledW);

  lightControl.lastR = scaledR;
  lightControl.lastG = scaledG;
  lightControl.lastB = scaledB;
  lightControl.lastW = scaledW;
#else
  (void)force;
#endif
}

bool updateLightFeedback()
{
#if LED_COUNT > 0
  if (!lightFeedback.active)
  {
    return false;
  }

  unsigned long now = millis();
  if ((int32_t)(now - lightFeedback.nextToggleAt) < 0)
  {
    return true;
  }

  if (lightFeedback.togglesRemaining == 0)
  {
    lightFeedback.active = false;
    lightControl.lastUpdateAt = now;
    applyLightOutput(true);
    return false;
  }

  lightFeedback.outputOn = !lightFeedback.outputOn;
  if (lightFeedback.outputOn)
  {
    applyIndicatorColor(lightFeedback.r, lightFeedback.g, lightFeedback.b, lightFeedback.w, true);
    lightFeedback.nextToggleAt = now + LIGHT_FEEDBACK_ON_MS;
  }
  else
  {
    applyIndicatorColor(0, 0, 0, 0, true);
    lightFeedback.nextToggleAt = now + LIGHT_FEEDBACK_OFF_MS;
  }
  lightFeedback.togglesRemaining--;
  if (lightFeedback.togglesRemaining == 0 && !lightFeedback.outputOn)
  {
    lightFeedback.active = false;
  }
  return lightFeedback.active;
#else
  return false;
#endif
}

void applyBootIndicator()
{
#if LED_COUNT > 0
  if (!HAS_LED_OUTPUT || !lightControl.outputInitialized)
  {
    return;
  }

  unsigned long now = millis();
  if (now - lightControl.lastUpdateAt < LIGHT_UPDATE_MS)
  {
    return;
  }
  lightControl.lastUpdateAt = now;

  uint32_t phase = now % BOOT_PULSE_PERIOD_MS;
  uint32_t halfPeriod = BOOT_PULSE_PERIOD_MS / 2UL;
  uint16_t level = BOOT_PULSE_LEVEL_MIN;
  uint16_t levelRange = BOOT_PULSE_LEVEL_MAX - BOOT_PULSE_LEVEL_MIN;

  if (halfPeriod > 0)
  {
    if (phase < halfPeriod)
    {
      level = BOOT_PULSE_LEVEL_MIN + (uint16_t)((levelRange * phase) / halfPeriod);
    }
    else
    {
      uint32_t downPhase = BOOT_PULSE_PERIOD_MS - phase;
      level = BOOT_PULSE_LEVEL_MIN + (uint16_t)((levelRange * downPhase) / halfPeriod);
    }
  }

  uint8_t scaledR = (uint8_t)(((uint32_t)255U * level + (LIGHT_LEVEL_MAX / 2)) / LIGHT_LEVEL_MAX);
  if (scaledR == lightControl.lastR &&
      lightControl.lastG == 0 &&
      lightControl.lastB == 0 &&
      lightControl.lastW == 0)
  {
    return;
  }

  applyIndicatorColor(scaledR, 0, 0, 0, true);
#endif
}

void updateLightControl()
{
  // Licht ist aktiv:
  // - waehrend FadeIn, Vakuumieren und Haltezeit
  // - nicht waehrend FadeOut; dort blendet das Licht kontrolliert aus
  if (updateLightFeedback())
  {
    return;
  }

  if (bootIndicatorActive || apModeIndicatorActive)
  {
    applyBootIndicator();
    return;
  }

  unsigned long now = millis();
  bool lightWindowActive = config.lightEnabled && showStatus.isRunning && (int32_t)(showStatus.openValveAt - now) > 0;
  if (!lightWindowActive && config.lightEnabled && lightControl.onUntilAt > 0)
  {
    lightWindowActive = (int32_t)(lightControl.onUntilAt - now) > 0;
  }
  bool previewActive = isLightPreviewActive(now);

  setLightTargetLevel((lightWindowActive || previewActive) ? LIGHT_LEVEL_MAX : 0);

  if (now - lightControl.lastUpdateAt < LIGHT_UPDATE_MS)
  {
    return;
  }
  lightControl.lastUpdateAt = now;

  if (lightControl.currentLevel == lightControl.targetLevel)
  {
    applyLightOutput(false);
    return;
  }

  uint16_t upStep = (uint16_t)(((uint32_t)LIGHT_LEVEL_MAX * LIGHT_UPDATE_MS + LED_FADE_IN_MS - 1) / LED_FADE_IN_MS);
  if (upStep == 0)
  {
    upStep = 1;
  }

  uint16_t downStep = (uint16_t)(((uint32_t)LIGHT_LEVEL_MAX * LIGHT_UPDATE_MS + LED_FADE_OUT_MS - 1) / LED_FADE_OUT_MS);
  if (downStep == 0)
  {
    downStep = 1;
  }

  if (lightControl.currentLevel < lightControl.targetLevel)
  {
    uint32_t next = lightControl.currentLevel + upStep;
    if (next > lightControl.targetLevel)
    {
      next = lightControl.targetLevel;
    }
    lightControl.currentLevel = (uint16_t)next;
  }
  else
  {
    int32_t next = (int32_t)lightControl.currentLevel - downStep;
    if (next < (int32_t)lightControl.targetLevel)
    {
      next = lightControl.targetLevel;
    }
    lightControl.currentLevel = (uint16_t)next;
  }

  applyLightOutput(false);
}

void setSafeOutputState()
{
  // Wird z. B. vor OTA aufgerufen, um alle Aktoren in sicheren Zustand
  // zu bringen (Pumpen AUS, Ventil definiert, Licht AUS).
  bootIndicatorActive = false;
  showStatus.shouldStart = false;
  showStatus.isRunning = false;
  showStatus.fadeInDoneAt = 0;
  showStatus.endAt = 0;
  showStatus.openValveAt = 0;
  showStatus.finishAt = 0;
  setPumpTarget(false);
  pumpControl.currentPwm = 0;
  applyPumpPwm(0);
  lightControl.onUntilAt = 0;
  lightControl.previewUntilAt = 0;
  lightFeedback.active = false;
  lightControl.currentLevel = 0;
  lightControl.targetLevel = 0;
  applyLightOutput(true);
  digitalWrite(PIN_VENTIL, LOW);
}

static bool isDigitChar(char c)
{
  return c >= '0' && c <= '9';
}

static size_t extractVersionNumbers(const String &version, uint32_t *outParts, size_t maxParts)
{
  // Extrahiert numerische Teile aus Strings wie:
  // "1.2.3-dev" -> [1,2,3]
  size_t count = 0;
  int index = 0;
  int len = (int)version.length();

  while (index < len && count < maxParts)
  {
    while (index < len && !isDigitChar(version[index]))
    {
      index++;
    }
    if (index >= len)
    {
      break;
    }

    uint32_t value = 0;
    while (index < len && isDigitChar(version[index]))
    {
      value = (value * 10UL) + (uint32_t)(version[index] - '0');
      index++;
    }
    outParts[count++] = value;
  }

  return count;
}

static int versionQualifierRank(const String &version)
{
  // Reihenfolge fuer Prae-Release-Tags:
  // dev < alpha < beta < rc < stable/custom-release-name
  String lower = version;
  lower.toLowerCase();

  int separatorPos = lower.indexOf('-');
  if (separatorPos < 0)
  {
    return 4;
  }

  if (lower.indexOf("rc", separatorPos) >= 0)
  {
    return 3;
  }
  if (lower.indexOf("beta", separatorPos) >= 0)
  {
    return 2;
  }
  if (lower.indexOf("alpha", separatorPos) >= 0)
  {
    return 1;
  }
  if (lower.indexOf("dev", separatorPos) >= 0 || lower.indexOf("snapshot", separatorPos) >= 0)
  {
    return 0;
  }

  // Unbekannte Suffixe gelten als benannte Stable-Releases.
  return 4;
}

int compareVersionStrings(const String &lhs, const String &rhs)
{
  // Vergleich in 2 Schritten:
  // 1) numerische Segmente (SemVer-aehnlich)
  // 2) Qualifier-Rang (dev/beta/rc/stable)
  const size_t MAX_PARTS = 8;
  uint32_t leftParts[MAX_PARTS] = {0};
  uint32_t rightParts[MAX_PARTS] = {0};
  size_t leftCount = extractVersionNumbers(lhs, leftParts, MAX_PARTS);
  size_t rightCount = extractVersionNumbers(rhs, rightParts, MAX_PARTS);
  size_t count = leftCount > rightCount ? leftCount : rightCount;

  for (size_t i = 0; i < count; i++)
  {
    uint32_t leftValue = i < leftCount ? leftParts[i] : 0;
    uint32_t rightValue = i < rightCount ? rightParts[i] : 0;
    if (leftValue < rightValue)
    {
      return -1;
    }
    if (leftValue > rightValue)
    {
      return 1;
    }
  }

  int leftRank = versionQualifierRank(lhs);
  int rightRank = versionQualifierRank(rhs);
  if (leftRank < rightRank)
  {
    return -1;
  }
  if (leftRank > rightRank)
  {
    return 1;
  }

  int textCmp = lhs.compareTo(rhs);
  if (textCmp < 0)
  {
    return -1;
  }
  if (textCmp > 0)
  {
    return 1;
  }
  return 0;
}

void otaResetProgress(const String &phase, const String &source, const String &statusText)
{
  otaStatus.phase = phase;
  otaStatus.source = source;
  otaStatus.statusText = statusText.length() > 0 ? statusText : phase;
  otaStatus.progressBytes = 0;
  otaStatus.progressTotal = 0;
  otaStatus.progressPercent = 0;
  otaLastPublishedProgressPercent = 255;
  otaStatus.rebootPending = false;
  otaStatus.rebootAt = 0;
  if (mqttClient.connected())
  {
    publishOtaUpdateState(true);
  }
}

void otaSetPhase(const String &phase, const String &statusText)
{
  otaStatus.phase = phase;
  otaStatus.statusText = statusText.length() > 0 ? statusText : phase;
  if (mqttClient.connected())
  {
    publishOtaUpdateState(true);
  }
}

void otaSetProgress(size_t current, size_t total)
{
  otaStatus.progressBytes = (uint32_t)current;
  otaStatus.progressTotal = (uint32_t)total;
  if (total > 0)
  {
    size_t percent = (current * 100U) / total;
    if (percent > 100U)
    {
      percent = 100U;
    }
    otaStatus.progressPercent = (uint8_t)percent;
  }
  else
  {
    otaStatus.progressPercent = 0;
  }
  if (mqttClient.connected())
  {
    if (otaStatus.progressPercent == 100 ||
        otaLastPublishedProgressPercent == 255 ||
        otaStatus.progressPercent == 0 ||
        otaStatus.progressPercent >= (uint8_t)(otaLastPublishedProgressPercent + 5))
    {
      otaLastPublishedProgressPercent = otaStatus.progressPercent;
      publishOtaUpdateState(true);
    }
  }
}

void otaScheduleRestart(const String &statusText)
{
  otaStatus.currentVersion = otaStatus.latestVersion.length() > 0 ? otaStatus.latestVersion : otaStatus.currentVersion;
  otaStatus.updateAvailable = false;
  otaStatus.updateInProgress = false;
  otaStatus.rebootPending = true;
  otaStatus.rebootAt = millis() + OTA_RESTART_DELAY_MS;
  otaSetPhase("rebooting", statusText);
  otaSetProgress(otaStatus.progressTotal > 0 ? otaStatus.progressTotal : otaStatus.progressBytes,
                 otaStatus.progressTotal);
}

void otaFinalizeFailure(const String &errorText)
{
  if (Update.isRunning())
  {
    Update.end();
  }
  otaStatus.updateInProgress = false;
  otaStatus.rebootPending = false;
  otaStatus.rebootAt = 0;
  otaStatus.lastError = errorText;
  otaSetPhase("failed", errorText.length() > 0 ? errorText : "OTA-Update fehlgeschlagen");
  if (mqttClient.connected())
  {
    publishOtaUpdateState(true);
    publishTelemetry(true);
  }
}

String formatHttpClientError(int httpCode, BearSSL::WiFiClientSecure *secureClient)
{
  String message = "HTTP-Fehler " + String(httpCode);
  String detail = HTTPClient::errorToString(httpCode);
  if (detail.length() > 0)
  {
    message += " (" + detail + ")";
  }

  if (secureClient != nullptr)
  {
    char sslError[96] = {0};
    int sslCode = secureClient->getLastSSLError(sslError, sizeof(sslError));
    if (sslCode != 0)
    {
      message += " / SSL " + String(sslCode);
      if (sslError[0] != '\0')
      {
        message += " (" + String(sslError) + ")";
      }
    }
  }

  return message;
}

bool parseHttpUrl(const String &url, ParsedHttpUrl &parsed)
{
  parsed = ParsedHttpUrl();

  int schemeEnd = url.indexOf("://");
  if (schemeEnd <= 0)
  {
    return false;
  }

  String scheme = url.substring(0, schemeEnd);
  scheme.toLowerCase();
  if (scheme == "https")
  {
    parsed.https = true;
    parsed.port = 443;
  }
  else if (scheme == "http")
  {
    parsed.https = false;
    parsed.port = 80;
  }
  else
  {
    return false;
  }

  int hostStart = schemeEnd + 3;
  int pathStart = url.indexOf('/', hostStart);
  String authority = pathStart >= 0 ? url.substring(hostStart, pathStart) : url.substring(hostStart);
  parsed.path = pathStart >= 0 ? url.substring(pathStart) : "/";

  int atPos = authority.lastIndexOf('@');
  if (atPos >= 0)
  {
    authority = authority.substring(atPos + 1);
  }

  int colonPos = authority.lastIndexOf(':');
  if (colonPos >= 0 && authority.indexOf(']') < 0)
  {
    parsed.host = authority.substring(0, colonPos);
    parsed.port = (uint16_t)authority.substring(colonPos + 1).toInt();
  }
  else
  {
    parsed.host = authority;
  }

  parsed.host.trim();
  if (parsed.host.length() == 0 || parsed.port == 0)
  {
    return false;
  }

  parsed.valid = true;
  return true;
}

bool configureSecureClientForUrl(BearSSL::WiFiClientSecure &secureClient, const String &url, String &tlsNote)
{
  tlsNote = "";
  secureClient.setInsecure();
  secureClient.setTimeout(OTA_HTTP_TIMEOUT_MS);

  ParsedHttpUrl parsed;
  if (!parseHttpUrl(url, parsed) || !parsed.https)
  {
    return true;
  }

  const uint16_t recvCandidates[] = {512, 1024, 2048, 4096};
  for (size_t i = 0; i < (sizeof(recvCandidates) / sizeof(recvCandidates[0])); i++)
  {
    uint16_t recvSize = recvCandidates[i];
    if (BearSSL::WiFiClientSecure::probeMaxFragmentLength(parsed.host, parsed.port, recvSize))
    {
      secureClient.setBufferSizes(recvSize, 512);
      tlsNote = "TLS-MFLN aktiv (" + String(recvSize) + "/512)";
      return true;
    }
  }

  tlsNote = "TLS-MFLN nicht verfuegbar";
  return true;
}

void updateOtaDerivedState(const String &latestVersion, const String &firmwareUrl, const String &releaseNotes)
{
  // Leitet "updateAvailable" ausschließlich aus Version + URL ab.
  otaStatus.latestVersion = latestVersion;
  otaStatus.firmwareUrl = firmwareUrl;
  otaStatus.releaseNotes = releaseNotes;
  int versionCmp = compareVersionStrings(otaStatus.currentVersion, latestVersion);
  otaStatus.updateAvailable = latestVersion.length() > 0 &&
                              firmwareUrl.length() > 0 &&
                              versionCmp < 0;
  LOGI("OTA compare: current=%s latest=%s cmp=%d",
       otaStatus.currentVersion.c_str(),
       latestVersion.c_str(),
       versionCmp);
}

bool otaCheckForUpdate(const char *reason)
{
  // Holt manifest.json, validiert Pflichtfelder und aktualisiert otaStatus.
  // reason ist nur fuer Logging/Diagnose (boot/manual/daily ...).
  if (otaStatus.checkInProgress || otaStatus.updateInProgress)
  {
    return false;
  }

  otaStatus.lastCheckAt = millis();
  otaStatus.lastError = "";

  if (config.otaManifestUrl.length() == 0)
  {
    otaStatus.lastError = "OTA-Manifest-URL nicht gesetzt";
    updateOtaDerivedState("", "", "");
    otaSetPhase("failed", otaStatus.lastError);
    LOGW("OTA check skipped (%s): manifest URL missing", reason);
    return false;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    otaStatus.lastError = "WLAN nicht verbunden";
    otaSetPhase("failed", otaStatus.lastError);
    LOGW("OTA check skipped (%s): WiFi not connected", reason);
    return false;
  }

  otaStatus.checkInProgress = true;
  otaResetProgress("checking", "manifest", "Pruefe auf neue Firmware");
  String requestUrl = config.otaManifestUrl;
  requestUrl += requestUrl.indexOf('?') >= 0 ? "&nocache=" : "?nocache=";
  requestUrl += String(millis());
  ParsedHttpUrl requestTarget;
  if (!parseHttpUrl(requestUrl, requestTarget))
  {
    otaStatus.lastError = "OTA-Manifest-URL ungueltig";
    otaStatus.checkInProgress = false;
    otaSetPhase("failed", otaStatus.lastError);
    return false;
  }
  bool mqttWasConnected = mqttClient.connected();
  if (mqttWasConnected)
  {
    mqttClient.disconnect();
    delay(50);
  }

  BearSSL::WiFiClientSecure secureClient;
  WiFiClient plainClient;
  String transportNote = requestTarget.https ? "HTTPS" : "HTTP";
  String tlsNote;
  if (requestTarget.https)
  {
    configureSecureClientForUrl(secureClient, requestUrl, tlsNote);
    transportNote += " / " + tlsNote;
  }
  LOGI("OTA check (%s): %s heap=%u transport=%s", reason, requestUrl.c_str(), ESP.getFreeHeap(), transportNote.c_str());

  HTTPClient http;
  http.setTimeout(OTA_HTTP_TIMEOUT_MS);
  http.setReuse(false);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  bool beginOk = requestTarget.https ? http.begin(secureClient, requestUrl) : http.begin(plainClient, requestUrl);
  if (!beginOk)
  {
    otaStatus.lastError = "Manifest-Request konnte nicht gestartet werden";
    otaStatus.checkInProgress = false;
    otaSetPhase("failed", otaStatus.lastError);
    if (mqttWasConnected)
    {
      publishTelemetry(true);
    }
    LOGW("OTA check failed: begin() failed transport=%s", transportNote.c_str());
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK)
  {
    otaStatus.lastError = formatHttpClientError(httpCode, requestTarget.https ? &secureClient : nullptr);
    if (requestTarget.https &&
        httpCode == HTTPC_ERROR_CONNECTION_FAILED &&
        tlsNote == "TLS-MFLN nicht verfuegbar")
    {
      otaStatus.lastError += " / Zielserver unterstuetzt keine kleinen TLS-Fragmente";
    }
    updateOtaDerivedState("", "", "");
    otaStatus.checkInProgress = false;
    otaSetPhase("failed", otaStatus.lastError);
    http.end();
    if (mqttWasConnected)
    {
      publishTelemetry(true);
    }
    LOGW("OTA check failed: http=%d err=%s heap=%u transport=%s", httpCode, otaStatus.lastError.c_str(), ESP.getFreeHeap(), transportNote.c_str());
    return false;
  }

  String payload = http.getString();
  http.end();
  String payloadPreview = payload;
  payloadPreview.replace('\n', ' ');
  if (payloadPreview.length() > 220)
  {
    payloadPreview = payloadPreview.substring(0, 220) + "...";
  }
  LOGI("OTA manifest preview: %s", payloadPreview.c_str());

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err)
  {
    otaStatus.lastError = "Manifest JSON ungueltig";
    updateOtaDerivedState("", "", "");
    otaStatus.checkInProgress = false;
    otaSetPhase("failed", otaStatus.lastError);
    if (mqttWasConnected)
    {
      publishTelemetry(true);
    }
    LOGW("OTA check failed: invalid JSON");
    return false;
  }

  String latestVersion = doc["version"] | "";
  String firmwareUrl = doc["firmware_url"] | "";
  if (firmwareUrl.length() == 0)
  {
    firmwareUrl = doc["url"] | "";
  }
  String releaseNotes = doc["notes"] | "";
  latestVersion.trim();
  firmwareUrl.trim();
  releaseNotes.trim();

  if (latestVersion.length() == 0 || firmwareUrl.length() == 0)
  {
    otaStatus.lastError = "Manifest unvollstaendig (version/url)";
    updateOtaDerivedState("", "", "");
    otaStatus.checkInProgress = false;
    otaSetPhase("failed", otaStatus.lastError);
    if (mqttWasConnected)
    {
      publishTelemetry(true);
    }
    LOGW("OTA check failed: manifest missing fields");
    return false;
  }

  updateOtaDerivedState(latestVersion, firmwareUrl, releaseNotes);
  otaStatus.checkInProgress = false;
  otaStatus.lastError = "";
  otaSetPhase("idle", otaStatus.updateAvailable ? "Neue Firmware verfuegbar" : "Firmware ist aktuell");

  LOGI("OTA check done: current=%s latest=%s fw=%s available=%s",
       otaStatus.currentVersion.c_str(),
       otaStatus.latestVersion.c_str(),
       otaStatus.firmwareUrl.c_str(),
       otaStatus.updateAvailable ? "yes" : "no");

  if (mqttClient.connected())
  {
    publishTelemetry(true);
  }
  else if (mqttWasConnected)
  {
    // mqttEnsureConnected() baut die Verbindung in loop() wieder auf.
  }

  return true;
}

bool otaApplyUpdate()
{
  // Fuehrt das eigentliche OTA-Flash aus.
  // Voraussetzung: WiFi online, Show gestoppt, gueltige Firmware-URL.
  if (otaStatus.updateInProgress)
  {
    return false;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    otaStatus.lastError = "WLAN nicht verbunden";
    otaSetPhase("failed", otaStatus.lastError);
    return false;
  }
  if (showStatus.isRunning)
  {
    otaStatus.lastError = "Show laeuft noch";
    otaSetPhase("failed", otaStatus.lastError);
    return false;
  }
  if (otaStatus.firmwareUrl.length() == 0)
  {
    otaStatus.lastError = "Keine Firmware-URL verfuegbar";
    otaSetPhase("failed", otaStatus.lastError);
    return false;
  }

  otaStatus.updateInProgress = true;
  otaStatus.lastError = "";
  otaResetProgress("downloading", "manifest", "Firmware wird geladen");
  LOGI("Starting OTA update from %s", otaStatus.firmwareUrl.c_str());
  ParsedHttpUrl firmwareTarget;
  if (!parseHttpUrl(otaStatus.firmwareUrl, firmwareTarget))
  {
    otaFinalizeFailure("Firmware-URL ungueltig");
    return false;
  }

  setSafeOutputState();
  if (mqttClient.connected())
  {
    publishOtaUpdateState(true);
    publishTelemetry(true);
  }

  BearSSL::WiFiClientSecure secureClient;
  WiFiClient plainClient;
  String transportNote = firmwareTarget.https ? "HTTPS" : "HTTP";
  String tlsNote;
  if (firmwareTarget.https)
  {
    configureSecureClientForUrl(secureClient, otaStatus.firmwareUrl, tlsNote);
    transportNote += " / " + tlsNote;
  }
  LOGI("OTA apply heap=%u transport=%s", ESP.getFreeHeap(), transportNote.c_str());

  ESPhttpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  ESPhttpUpdate.setClientTimeout(OTA_HTTP_TIMEOUT_MS);
  ESPhttpUpdate.rebootOnUpdate(false);
  ESPhttpUpdate.onStart([]() {
    otaSetPhase("flashing", "Firmware wird geschrieben");
    publishTelemetry(true);
  });
  ESPhttpUpdate.onProgress([](int current, int total) {
    otaSetProgress(current, total);
    otaSetPhase("flashing", total > 0 ? "Firmware wird geschrieben" : "Firmware-Stream aktiv");
  });
  ESPhttpUpdate.onError([](int error) {
    LOGW("OTA progress callback error=%d", error);
  });
  ESPhttpUpdate.onEnd([]() {
    otaSetProgress(otaStatus.progressTotal > 0 ? otaStatus.progressTotal : otaStatus.progressBytes,
                   otaStatus.progressTotal);
  });
  t_httpUpdate_return result = firmwareTarget.https
                                   ? ESPhttpUpdate.update(secureClient, otaStatus.firmwareUrl)
                                   : ESPhttpUpdate.update(plainClient, otaStatus.firmwareUrl);

  if (result == HTTP_UPDATE_OK)
  {
    otaStatus.lastError = "";
    otaScheduleRestart("Firmware erfolgreich installiert, Neustart laeuft");
    publishTelemetry(true);
    return true;
  }

  if (result == HTTP_UPDATE_NO_UPDATES)
  {
    otaFinalizeFailure("Kein neues Update verfuegbar");
    LOGW("OTA update returned: no updates");
  }
  else
  {
    String errorText = ESPhttpUpdate.getLastErrorString();
    if (firmwareTarget.https &&
        errorText.indexOf("SSL structures and buffers") >= 0 &&
        tlsNote == "TLS-MFLN nicht verfuegbar")
    {
      errorText += " / Zielserver unterstuetzt keine kleinen TLS-Fragmente";
    }
    otaFinalizeFailure(errorText);
    LOGW("OTA update failed (%d): %s", ESPhttpUpdate.getLastError(), errorText.c_str());
  }

  return false;
}

void otaLoop()
{
  // OTA-Scheduler:
  // - Boot-Check (sobald WLAN steht)
  // - manuell angeforderte Checks/Updates
  // - periodischer Check (Intervallkonstante)
  if (otaBootCheckPending && WiFi.status() == WL_CONNECTED)
  {
    otaBootCheckPending = false;
    otaCheckForUpdate("boot-delayed");
  }

  if (otaCheckRequested)
  {
    otaCheckRequested = false;
    otaCheckForUpdate("manual");
  }

  if (otaUpdateRequested)
  {
    otaUpdateRequested = false;
    otaApplyUpdate();
  }

  if (otaStatus.rebootPending && millis() >= otaStatus.rebootAt)
  {
    LOGI("Restarting after OTA update");
    delay(100);
    ESP.restart();
    return;
  }

  if (WiFi.status() != WL_CONNECTED || otaStatus.lastCheckAt == 0)
  {
    return;
  }

  if (!otaStatus.checkInProgress &&
      !otaStatus.updateInProgress &&
      (millis() - otaStatus.lastCheckAt) >= OTA_CHECK_INTERVAL_MS)
  {
    otaCheckForUpdate("daily");
  }
}

void setupMqtt()
{
  // Nur Client-Grundkonfiguration. Verbindungsaufbau passiert in mqttEnsureConnected().
  mqttClient.setServer(config.mqttHost.c_str(), config.mqttPort);
  mqttClient.setBufferSize(1024);
  mqttClient.setCallback(mqttCallback);
  updateMqttTopics();
  LOGI("MQTT setup: broker=%s:%u", config.mqttHost.c_str(), config.mqttPort);
}

void handleWiFiStartup()
{
  if (!wifiStartupPending)
  {
    return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    wifiStartupPending = false;
    bootIndicatorActive = false;
    if (!captivePortalEnabled)
    {
      apModeIndicatorActive = false;
    }
    LOGI("STA connected. IP=%s", WiFi.localIP().toString().c_str());
    return;
  }

  if ((millis() - wifiStartupAt) < WIFI_CONNECT_TIMEOUT_MS)
  {
    return;
  }

  wifiStartupPending = false;
  bootIndicatorActive = false;
  LOGW("Initial WiFi connect timed out, enabling AP fallback");
  startAccessPoint();
  setupCaptivePortal();
}

void mqttLoop()
{
  // MQTT nur bedienen, wenn STA-Link aktiv ist.
  if (WiFi.status() != WL_CONNECTED)
  {
    return;
  }

  if (!mqttClient.connected())
  {
    mqttEnsureConnected();
    return;
  }

  mqttClient.loop();
}

void clearStaleCommandTopics()
{
  // Command-Topics sollten nicht retained sein.
  // Loescht alte retained Kommandos, damit beim Power-On keine stale Befehle
  // (z. B. show/set=ON) direkt nach dem Subscribe wieder zugestellt werden.
  mqttClient.publish(topicShowSet.c_str(), "", true);
  if (HAS_LED_OUTPUT)
  {
    mqttClient.publish(topicLightSet.c_str(), "", true);
    mqttClient.publish(topicLightRgbSet.c_str(), "", true);
    mqttClient.publish(topicLightRgbwSet.c_str(), "", true);
  }
  mqttClient.publish(topicCfgShowLengthSet.c_str(), "", true);
  mqttClient.publish(topicCfgNachlaufSet.c_str(), "", true);
  mqttClient.publish(topicOtaInstall.c_str(), "", true);
}

void mqttEnsureConnected()
{
  // Nicht blockierender Reconnect mit Zeitabstand, damit loop() reaktiv bleibt.
  if (config.mqttHost.length() == 0)
  {
    LOGW("MQTT disabled: broker host empty");
    return;
  }

  if (millis() - lastMqttReconnectAt < MQTT_RECONNECT_INTERVAL_MS)
  {
    return;
  }
  lastMqttReconnectAt = millis();

  String clientId = deviceId;
  bool connected = false;

  if (config.mqttUser.length() > 0)
  {
    connected = mqttClient.connect(
        clientId.c_str(),
        config.mqttUser.c_str(),
        config.mqttPassword.c_str(),
        topicAvailability.c_str(),
        0,
        true,
        "offline");
  }
  else
  {
    connected = mqttClient.connect(
        clientId.c_str(),
        topicAvailability.c_str(),
        0,
        true,
        "offline");
  }

  if (!connected)
  {
    LOGW("MQTT connect failed, rc=%d", mqttClient.state());
    return;
  }

  LOGI("MQTT connected");
  clearStaleCommandTopics();

  mqttClient.subscribe(topicShowSet.c_str());
  if (HAS_LED_OUTPUT)
  {
    mqttClient.subscribe(topicLightSet.c_str());
    mqttClient.subscribe(topicLightRgbSet.c_str());
    mqttClient.subscribe(topicLightRgbwSet.c_str());
  }
  mqttClient.subscribe(topicCfgShowLengthSet.c_str());
  mqttClient.subscribe(topicCfgNachlaufSet.c_str());
  mqttClient.subscribe(topicOtaInstall.c_str());
  publishAvailability("online", true);
  publishDiscovery();
  publishShowState(true);
  if (HAS_LED_OUTPUT)
  {
    publishLightState(true);
  }
  publishConfigState(true);
  publishOtaUpdateState(true);
  publishTelemetry(true);
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  // Zentraler MQTT-Command-Dispatcher.
  // Erwartete Payloads:
  // - show/set: ON|OFF
  // - light/set: ON|OFF oder JSON
  // - light/rgb/set: "r,g,b"
  // - light/rgbw/set: "r,g,b,w"
  // - config/*/set: Sekundenwerte
  String topicStr(topic);
  String payloadStr;
  payloadStr.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++)
  {
    payloadStr += static_cast<char>(payload[i]);
  }
  payloadStr.trim();
  LOGI("MQTT RX topic=%s payload=%s", topicStr.c_str(), payloadStr.c_str());

  if (topicStr == topicShowSet)
  {
    if (payloadStr == "ON")
    {
      startShow();
    }
    else if (payloadStr == "OFF")
    {
      stopShow();
    }
    publishShowState(true);
    publishTelemetry(true);
    return;
  }

  if (!HAS_LED_OUTPUT && (topicStr == topicLightSet || topicStr == topicLightRgbSet || topicStr == topicLightRgbwSet))
  {
    LOGD("Ignoring light MQTT command (LED_COUNT=0)");
    return;
  }

  if (HAS_LED_OUTPUT && topicStr == topicLightSet)
  {
    bool changed = false;
    bool colorChanged = false;

    if (payloadStr == "ON" || payloadStr == "OFF")
    {
      bool newEnabled = payloadStr == "ON";
      if (config.lightEnabled != newEnabled)
      {
        config.lightEnabled = newEnabled;
        changed = true;
      }
      publishLightState(true);
      publishTelemetry(true);
      if (changed)
      {
        if (!config.lightEnabled && !showStatus.isRunning)
        {
          lightControl.previewUntilAt = 0;
        }
        scheduleConfigSave();
      }
      return;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, payloadStr);
    if (!err)
    {
      if (!doc["state"].isNull())
      {
        String state = doc["state"].as<String>();
        state.trim();
        state.toUpperCase();
        if (state == "ON" || state == "OFF")
        {
          bool newEnabled = state == "ON";
          if (config.lightEnabled != newEnabled)
          {
            config.lightEnabled = newEnabled;
            changed = true;
          }
        }
      }
      if (doc["color"].is<JsonObject>())
      {
        JsonObject color = doc["color"].as<JsonObject>();
        if (!color["r"].isNull())
        {
          config.lightR = clampU8(color["r"].as<int>());
          changed = true;
          colorChanged = true;
        }
        if (!color["g"].isNull())
        {
          config.lightG = clampU8(color["g"].as<int>());
          changed = true;
          colorChanged = true;
        }
        if (!color["b"].isNull())
        {
          config.lightB = clampU8(color["b"].as<int>());
          changed = true;
          colorChanged = true;
        }
        if (!color["w"].isNull())
        {
          config.lightW = clampU8(color["w"].as<int>());
          changed = true;
          colorChanged = true;
        }
      }

      if (!doc["white_value"].isNull())
      {
        config.lightW = clampU8(doc["white_value"].as<int>());
        changed = true;
        colorChanged = true;
      }
      if (!doc["white"].isNull())
      {
        config.lightW = clampU8(doc["white"].as<int>());
        changed = true;
        colorChanged = true;
      }
    }

    if (changed)
    {
      LOGI("Beleuchtung updated via JSON command");
      if (colorChanged && !showStatus.isRunning)
      {
        requestLightPreview();
      }
      else if (!config.lightEnabled && !showStatus.isRunning)
      {
        lightControl.previewUntilAt = 0;
      }
      scheduleConfigSave();
    }
    publishLightState(true);
    publishTelemetry(true);
    return;
  }

  if (HAS_LED_OUTPUT && topicStr == topicLightRgbSet)
  {
    int r = -1;
    int g = -1;
    int b = -1;
    int parsed = sscanf(payloadStr.c_str(), "%d,%d,%d", &r, &g, &b);
    if (parsed == 3)
    {
      config.lightR = clampU8(r);
      config.lightG = clampU8(g);
      config.lightB = clampU8(b);
      LOGI("Beleuchtung updated via RGB topic: %u,%u,%u", config.lightR, config.lightG, config.lightB);
      if (!showStatus.isRunning)
      {
        requestLightPreview();
      }
      scheduleConfigSave();
      publishLightState(true);
      publishTelemetry(true);
    }
    else
    {
      LOGW("Invalid RGB payload on %s: %s", topicStr.c_str(), payloadStr.c_str());
    }
    return;
  }

  if (HAS_LED_OUTPUT && topicStr == topicLightRgbwSet)
  {
    int r = -1;
    int g = -1;
    int b = -1;
    int w = -1;
    int parsed = sscanf(payloadStr.c_str(), "%d,%d,%d,%d", &r, &g, &b, &w);
    if (parsed == 4)
    {
      config.lightR = clampU8(r);
      config.lightG = clampU8(g);
      config.lightB = clampU8(b);
      config.lightW = clampU8(w);
      LOGI("Beleuchtung updated via RGBW topic: %u,%u,%u,%u", config.lightR, config.lightG, config.lightB, config.lightW);
      if (!showStatus.isRunning)
      {
        requestLightPreview();
      }
      scheduleConfigSave();
      publishLightState(true);
      publishTelemetry(true);
    }
    else
    {
      LOGW("Invalid RGBW payload: %s", payloadStr.c_str());
    }
    return;
  }

  if (topicStr == topicCfgShowLengthSet)
  {
    long valueSec = payloadStr.toInt();
    if (valueSec <= 0)
    {
      LOGW("Invalid show length payload: %s", payloadStr.c_str());
      return;
    }

    uint32_t clampedSec = clampU32((uint32_t)valueSec, SHOW_LENGTH_MIN_S, SHOW_LENGTH_MAX_S);
    config.showLengthMs = secToMs(clampedSec);
    showStatus.showDuration = config.showLengthMs;
    LOGI("Show length updated via MQTT: %u s (%u ms)", clampedSec, config.showLengthMs);
    scheduleConfigSave();
    publishConfigState(true);
    publishTelemetry(true);
    return;
  }

  if (topicStr == topicCfgNachlaufSet)
  {
    long valueSec = payloadStr.toInt();
    if (valueSec < 0)
    {
      LOGW("Invalid nachlauf payload: %s", payloadStr.c_str());
      return;
    }

    uint32_t minClampedSec = clampMinU32((uint32_t)valueSec, SHOW_NACHLAUF_MIN_S);
    config.showNachlaufMs = secToMs(minClampedSec);
    showStatus.showNachlauf = config.showNachlaufMs;
    LOGI("Nachlauf updated via MQTT: %u s (%u ms)", minClampedSec, config.showNachlaufMs);
    scheduleConfigSave();
    publishConfigState(true);
    publishTelemetry(true);
    return;
  }

  if (topicStr == topicOtaInstall)
  {
    if (payloadStr != "install")
    {
      LOGW("Invalid OTA install payload: %s", payloadStr.c_str());
      return;
    }

    if (otaStatus.checkInProgress || otaStatus.updateInProgress)
    {
      otaStatus.lastError = "OTA bereits aktiv";
      publishTelemetry(true);
      return;
    }
    if (WiFi.status() != WL_CONNECTED)
    {
      otaStatus.lastError = "WLAN nicht verbunden";
      publishTelemetry(true);
      return;
    }
    if (showStatus.isRunning)
    {
      otaStatus.lastError = "Show laeuft noch";
      publishTelemetry(true);
      return;
    }
    if (!otaStatus.updateAvailable || otaStatus.firmwareUrl.length() == 0)
    {
      otaStatus.lastError = "Kein Update verfuegbar";
      publishTelemetry(true);
      return;
    }

    otaUpdateRequested = true;
    otaStatus.lastError = "";
    publishTelemetry(true);
  }
}

void publishAvailability(const char *state, bool retained)
{
  mqttClient.publish(topicAvailability.c_str(), state, retained);
  LOGD("MQTT TX availability=%s", state);
}

void publishShowState(bool retained)
{
  // Show-Entity in HA: ON nur waehrend showStatus.isRunning.
  mqttClient.publish(topicShowState.c_str(), showStatus.isRunning ? "ON" : "OFF", retained);
  LOGD("MQTT TX show_state=%s", showStatus.isRunning ? "ON" : "OFF");
}

void publishLightState(bool retained)
{
  // Light-Entity bildet die aktivierte Show-Beleuchtung ab.
  // Farbe wird persistent konfiguriert, ON/OFF schaltet die Show-Beleuchtung frei.
  if (!HAS_LED_OUTPUT)
  {
    return;
  }
  mqttClient.publish(topicLightState.c_str(), config.lightEnabled ? "ON" : "OFF", retained);
  String rgb = String(config.lightR) + "," + String(config.lightG) + "," + String(config.lightB);
  mqttClient.publish(topicLightRgbState.c_str(), rgb.c_str(), retained);
  String rgbw = String(config.lightR) + "," + String(config.lightG) + "," + String(config.lightB) + "," + String(config.lightW);
  mqttClient.publish(topicLightRgbwState.c_str(), rgbw.c_str(), retained);
  LOGD("MQTT TX light_state=%s rgb=%s rgbw=%s", config.lightEnabled ? "ON" : "OFF", rgb.c_str(), rgbw.c_str());
}

void publishConfigState(bool retained)
{
  if (!mqttClient.connected())
  {
    return;
  }
  String showSec = String(msToSec(config.showLengthMs));
  String nachlaufSec = String(msToSec(config.showNachlaufMs));
  mqttClient.publish(topicCfgShowLengthState.c_str(), showSec.c_str(), retained);
  mqttClient.publish(topicCfgNachlaufState.c_str(), nachlaufSec.c_str(), retained);
  LOGD("MQTT TX config show=%s s nachlauf=%s s", showSec.c_str(), nachlaufSec.c_str());
}

void publishDiscovery()
{
  // Veroeffentlicht HA Discovery fuer Steuer-Entities.
  // Bei LED_COUNT=0 wird die Light-Discovery absichtlich geloescht.
  LOGI("Publishing Home Assistant discovery");
  JsonDocument switchCfg;
  switchCfg["name"] = "Vakuumieren";
  switchCfg["object_id"] = "show";
  switchCfg["unique_id"] = deviceId + "-show";
  switchCfg["command_topic"] = topicShowSet;
  switchCfg["state_topic"] = topicShowState;
  switchCfg["payload_on"] = "ON";
  switchCfg["payload_off"] = "OFF";
  switchCfg["availability_topic"] = topicAvailability;
  switchCfg["payload_available"] = "online";
  switchCfg["payload_not_available"] = "offline";
  JsonObject swDevice = switchCfg["device"].to<JsonObject>();
  JsonArray swIds = swDevice["identifiers"].to<JsonArray>();
  swIds.add(deviceId);
  swDevice["name"] = "VacUBear";
  swDevice["manufacturer"] = "KingBEAR";
  swDevice["model"] = "Frame-25";

  String switchPayload;
  serializeJson(switchCfg, switchPayload);
  String switchDiscoveryTopic = "homeassistant/switch/" + deviceId + "/show/config";
  mqttClient.publish(switchDiscoveryTopic.c_str(), switchPayload.c_str(), true);

  String lightDiscoveryTopic = "homeassistant/light/" + deviceId + "/light/config";
  if (HAS_LED_OUTPUT)
  {
    JsonDocument lightCfg;
    lightCfg["name"] = "Beleuchtung";
    lightCfg["object_id"] = "light";
    lightCfg["unique_id"] = deviceId + "-light";
    lightCfg["command_topic"] = topicLightSet;
    lightCfg["state_topic"] = topicLightState;
    lightCfg["payload_on"] = "ON";
    lightCfg["payload_off"] = "OFF";
    lightCfg["rgb_command_topic"] = topicLightRgbSet;
    lightCfg["rgb_state_topic"] = topicLightRgbState;
    JsonArray colorModes = lightCfg["supported_color_modes"].to<JsonArray>();
    colorModes.add("rgb");
    lightCfg["availability_topic"] = topicAvailability;
    lightCfg["payload_available"] = "online";
    lightCfg["payload_not_available"] = "offline";
    JsonObject lightDevice = lightCfg["device"].to<JsonObject>();
    JsonArray lightIds = lightDevice["identifiers"].to<JsonArray>();
    lightIds.add(deviceId);
    lightDevice["name"] = "VacUBear";
    lightDevice["manufacturer"] = "KingBEAR";
    lightDevice["model"] = "Frame-25";

    if (lightCfg.overflowed())
    {
      LOGW("Light discovery payload overflowed");
    }

    String lightPayload;
    serializeJson(lightCfg, lightPayload);
    mqttClient.publish(lightDiscoveryTopic.c_str(), lightPayload.c_str(), true);
    LOGD("MQTT TX light discovery: %s", lightPayload.c_str());
  }
  else
  {
    mqttClient.publish(lightDiscoveryTopic.c_str(), "", true);
    mqttClient.publish(topicLightState.c_str(), "", true);
    mqttClient.publish(topicLightRgbwState.c_str(), "", true);
    LOGI("Light discovery removed (LED_COUNT=0)");
  }

  publishUpdateDiscovery();
  publishSensorDiscovery();
}

void publishUpdateDiscovery()
{
  JsonDocument cfg;
  cfg["name"] = "Firmware";
  cfg["object_id"] = "firmware";
  cfg["unique_id"] = deviceId + "-firmware-update";
  cfg["device_class"] = "firmware";
  cfg["command_topic"] = topicOtaInstall;
  cfg["payload_install"] = "install";
  cfg["state_topic"] = topicOtaState;
  cfg["availability_topic"] = topicAvailability;
  cfg["payload_available"] = "online";
  cfg["payload_not_available"] = "offline";

  JsonObject dev = cfg["device"].to<JsonObject>();
  JsonArray ids = dev["identifiers"].to<JsonArray>();
  ids.add(deviceId);
  dev["name"] = "VacUBear";
  dev["manufacturer"] = "KingBEAR";
  dev["model"] = "Frame-25";

  String payload;
  serializeJson(cfg, payload);
  mqttClient.publish((String("homeassistant/update/") + deviceId + "/firmware/config").c_str(), payload.c_str(), true);
}

void publishSensorDiscovery()
{
  // Veroeffentlicht Diagnose-/Status-Entities (Sensoren/Binary-Sensoren/Number).
  auto addDevice = [&](JsonObject obj)
  {
    JsonObject dev = obj["device"].to<JsonObject>();
    JsonArray ids = dev["identifiers"].to<JsonArray>();
    ids.add(deviceId);
    dev["name"] = "VacUBear";
    dev["manufacturer"] = "KingBEAR";
    dev["model"] = "Frame-25";
  };

  {
    JsonDocument cfg;
    cfg["name"] = "Show aktiv";
    cfg["object_id"] = "show_aktiv";
    cfg["unique_id"] = deviceId + "-show-aktiv";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ 'ON' if value_json.Show.Aktiv else 'OFF' }}";
    cfg["payload_on"] = "ON";
    cfg["payload_off"] = "OFF";
    cfg["availability_topic"] = topicAvailability;
    cfg["enabled_by_default"] = false;
    cfg["entity_category"] = "diagnostic";
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/binary_sensor/") + deviceId + "/show_aktiv/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Show Phase";
    cfg["object_id"] = "show_phase";
    cfg["unique_id"] = deviceId + "-show-phase";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.Show.Phase }}";
    cfg["icon"] = "mdi:state-machine";
    cfg["availability_topic"] = topicAvailability;
    cfg["enabled_by_default"] = false;
    cfg["entity_category"] = "diagnostic";
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/show_phase/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "IP-Adresse";
    cfg["object_id"] = "ip_adresse";
    cfg["unique_id"] = deviceId + "-ip-adresse";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.WiFi.IP }}";
    cfg["icon"] = "mdi:ip-network";
    cfg["availability_topic"] = topicAvailability;
    cfg["entity_category"] = "diagnostic";
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/ip_adresse/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "WLAN RSSI";
    cfg["object_id"] = "wifi_rssi";
    cfg["unique_id"] = deviceId + "-wifi-rssi";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.WiFi.RSSI }}";
    cfg["unit_of_measurement"] = "dBm";
    cfg["icon"] = "mdi:wifi";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/wifi_rssi/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Uptime";
    cfg["object_id"] = "uptime";
    cfg["unique_id"] = deviceId + "-uptime";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.UptimeSec }}";
    cfg["unit_of_measurement"] = "s";
    cfg["device_class"] = "duration";
    cfg["state_class"] = "measurement";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/uptime/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Pumpen PWM";
    cfg["object_id"] = "pumpen_pwm";
    cfg["unique_id"] = deviceId + "-pumpen-pwm";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.Pumpen.PWM }}";
    cfg["unit_of_measurement"] = "raw";
    cfg["icon"] = "mdi:fan";
    cfg["availability_topic"] = topicAvailability;
    cfg["enabled_by_default"] = false;
    cfg["entity_category"] = "diagnostic";
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/pumpen_pwm/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Show Länge";
    cfg["object_id"] = "show_laenge";
    cfg["unique_id"] = deviceId + "-show-laenge";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.Show.LaengeS }}";
    cfg["unit_of_measurement"] = "s";
    cfg["icon"] = "mdi:timer-sand";
    cfg["availability_topic"] = topicAvailability;
    cfg["enabled_by_default"] = false;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/show_laenge/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Nachlaufzeit";
    cfg["object_id"] = "nachlaufzeit";
    cfg["unique_id"] = deviceId + "-nachlaufzeit";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.Show.NachlaufS }}";
    cfg["unit_of_measurement"] = "s";
    cfg["icon"] = "mdi:timer-outline";
    cfg["availability_topic"] = topicAvailability;
    cfg["enabled_by_default"] = false;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/nachlaufzeit/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Vakuumier-Zeit";
    cfg["object_id"] = "show_laenge_setzen";
    cfg["unique_id"] = deviceId + "-show-laenge-setzen";
    cfg["command_topic"] = topicCfgShowLengthSet;
    cfg["state_topic"] = topicCfgShowLengthState;
    cfg["unit_of_measurement"] = "s";
    cfg["min"] = SHOW_LENGTH_MIN_S;
    cfg["max"] = SHOW_LENGTH_MAX_S;
    cfg["step"] = 1;
    cfg["mode"] = "box";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/number/") + deviceId + "/show_laenge_setzen/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Nachlaufzeit";
    cfg["object_id"] = "nachlaufzeit_setzen";
    cfg["unique_id"] = deviceId + "-nachlaufzeit-setzen";
    cfg["command_topic"] = topicCfgNachlaufSet;
    cfg["state_topic"] = topicCfgNachlaufState;
    cfg["unit_of_measurement"] = "s";
    cfg["min"] = SHOW_NACHLAUF_MIN_S;
    cfg["step"] = 1;
    cfg["mode"] = "box";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/number/") + deviceId + "/nachlaufzeit_setzen/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Firmware Version";
    cfg["object_id"] = "firmware_version";
    cfg["unique_id"] = deviceId + "-firmware-version";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.OTA.CurrentVersion }}";
    cfg["icon"] = "mdi:chip";
    cfg["availability_topic"] = topicAvailability;
    cfg["enabled_by_default"] = false;
    cfg["entity_category"] = "diagnostic";
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/firmware_version/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Firmware Verfuegbar";
    cfg["object_id"] = "firmware_verfuegbar";
    cfg["unique_id"] = deviceId + "-firmware-verfuegbar";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.OTA.LatestVersion }}";
    cfg["icon"] = "mdi:update";
    cfg["availability_topic"] = topicAvailability;
    cfg["enabled_by_default"] = false;
    cfg["entity_category"] = "diagnostic";
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/firmware_verfuegbar/config").c_str(), payload.c_str(), true);
  }

  {
    JsonDocument cfg;
    cfg["name"] = "Update verfuegbar";
    cfg["object_id"] = "update_verfuegbar";
    cfg["unique_id"] = deviceId + "-update-verfuegbar";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ 'ON' if value_json.OTA.UpdateAvailable else 'OFF' }}";
    cfg["payload_on"] = "ON";
    cfg["payload_off"] = "OFF";
    cfg["icon"] = "mdi:download";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/binary_sensor/") + deviceId + "/update_verfuegbar/config").c_str(), payload.c_str(), true);
  }

  LOGI("Published Home Assistant sensor discovery");
}

const char *getShowPhase()
{
  // Menschlich lesbare Show-Phase fuer Telemetrie/UI.
  unsigned long now = millis();
  if (showStatus.isRunning)
  {
    if (now < showStatus.fadeInDoneAt)
    {
      return "FadeIn";
    }
    if (now < showStatus.endAt)
    {
      return "Vakuumieren";
    }
    if (now < showStatus.openValveAt)
    {
      return "Haltezeit";
    }
    return "FadeOut";
  }

  if (showStatus.finishAt > 0 && (now - showStatus.finishAt) < 2000UL)
  {
    return "FadeOut";
  }
  return "Pause";
}

void publishTelemetry(bool force)
{
  // Tasmota-aehnliche Sammeltelemetrie auf tele/<deviceId>/STATE.
  // Wird periodisch und bei relevanten Events veroeffentlicht.
  if (!mqttClient.connected())
  {
    return;
  }

  unsigned long now = millis();
  if (!force && (now - lastTelemetryAt) < TELEMETRY_INTERVAL_MS)
  {
    return;
  }
  lastTelemetryAt = now;

  JsonDocument doc;
  doc["UptimeSec"] = now / 1000;

  JsonObject wifi = doc["WiFi"].to<JsonObject>();
  wifi["RSSI"] = WiFi.RSSI();
  wifi["SSID"] = WiFi.SSID();
  wifi["IP"] = WiFi.localIP().toString();
  wifi["Host"] = deviceId;
  wifi["Status"] = WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected";

  JsonObject show = doc["Show"].to<JsonObject>();
  show["Aktiv"] = showStatus.isRunning;
  show["Status"] = showStatus.isRunning ? "ON" : "OFF";
  show["Phase"] = getShowPhase();
  show["LaengeMs"] = config.showLengthMs;
  show["NachlaufMs"] = config.showNachlaufMs;
  show["LaengeS"] = msToSec(config.showLengthMs);
  show["NachlaufS"] = msToSec(config.showNachlaufMs);
  show["FadeInDoneAt"] = showStatus.fadeInDoneAt;
  show["EndAt"] = showStatus.endAt;
  show["OpenValveAt"] = showStatus.openValveAt;
  show["FinishAt"] = showStatus.finishAt;

  JsonObject pumpen = doc["Pumpen"].to<JsonObject>();
  pumpen["PWM"] = pumpControl.currentPwm;
  pumpen["TargetPWM"] = pumpControl.targetPwm;

  if (HAS_LED_OUTPUT)
  {
    uint32_t previewRemainingMs = isLightPreviewActive(now)
                                      ? (uint32_t)(lightControl.previewUntilAt - now)
                                      : 0;
    JsonObject licht = doc["Lichtfarbe"].to<JsonObject>();
    licht["Enabled"] = config.lightEnabled;
    licht["R"] = config.lightR;
    licht["G"] = config.lightG;
    licht["B"] = config.lightB;
    licht["W"] = config.lightW;
    licht["PreviewRemainingMs"] = previewRemainingMs;

    JsonObject led = doc["LED"].to<JsonObject>();
    led["Enabled"] = config.lightEnabled;
    led["Count"] = LED_COUNT;
    led["Level"] = lightControl.currentLevel;
    led["TargetLevel"] = lightControl.targetLevel;
    led["OnUntilAt"] = lightControl.onUntilAt;
    led["PreviewUntilAt"] = lightControl.previewUntilAt;
  }

  JsonObject ota = doc["OTA"].to<JsonObject>();
  ota["CurrentVersion"] = otaStatus.currentVersion;
  ota["LatestVersion"] = otaStatus.latestVersion;
  ota["ReleaseNotes"] = otaStatus.releaseNotes;
  ota["UpdateAvailable"] = otaStatus.updateAvailable;
  ota["CheckInProgress"] = otaStatus.checkInProgress;
  ota["UpdateInProgress"] = otaStatus.updateInProgress;
  ota["Phase"] = otaStatus.phase;
  ota["Source"] = otaStatus.source;
  ota["StatusText"] = otaStatus.statusText;
  ota["RebootPending"] = otaStatus.rebootPending;
  ota["ProgressBytes"] = otaStatus.progressBytes;
  ota["ProgressTotal"] = otaStatus.progressTotal;
  ota["ProgressPercent"] = otaStatus.progressPercent;
  ota["LastError"] = otaStatus.lastError;
  ota["LastCheckMs"] = otaStatus.lastCheckAt;
  ota["RebootAtMs"] = otaStatus.rebootAt;

  String payload;
  serializeJson(doc, payload);
  mqttClient.publish(topicTeleState.c_str(), payload.c_str(), false);
  LOGD("MQTT TX telemetry: %s", payload.c_str());
}

void updateMqttTopics()
{
  // Normalisiert Basis-Topic und erzeugt daraus alle Einzel-Topics.
  topicBase = config.mqttTopic;
  topicBase.trim();
  if (topicBase.length() == 0 || topicBase == "vacubear" || topicBase.startsWith("vacubear/"))
  {
    topicBase = deviceId;
  }
  while (topicBase.endsWith("/"))
  {
    topicBase.remove(topicBase.length() - 1);
  }

  topicShowSet = topicBase + "/show/set";
  topicShowState = topicBase + "/show/state";
  topicLightSet = topicBase + "/light/set";
  topicLightState = topicBase + "/light/state";
  topicLightRgbSet = topicBase + "/light/rgb/set";
  topicLightRgbState = topicBase + "/light/rgb/state";
  topicLightRgbwSet = topicBase + "/light/rgbw/set";
  topicLightRgbwState = topicBase + "/light/rgbw/state";
  topicCfgShowLengthSet = topicBase + "/config/show_length_s/set";
  topicCfgShowLengthState = topicBase + "/config/show_length_s/state";
  topicCfgNachlaufSet = topicBase + "/config/nachlauf_s/set";
  topicCfgNachlaufState = topicBase + "/config/nachlauf_s/state";
  topicOtaInstall = topicBase + "/ota/update/install";
  topicOtaState = topicBase + "/ota/update/state";
  topicAvailability = topicBase + "/availability";
  topicTeleState = "tele/" + deviceId + "/STATE";
  LOGI("MQTT topics initialized: base=%s", topicBase.c_str());
}

void setupWiFi()
{
  // Strategie:
  // 1) STA-Verbindung asynchron starten
  // 2) waehrenddessen loop() aktiv lassen (z. B. Boot-LED)
  // 3) bei Timeout AP + Captive Portal starten
  LOGI("Initializing WiFi");
  apSsid = defaultApSsid();

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.hostname(deviceId.c_str());

  if (config.wifiSsid.length() == 0)
  {
    LOGW("No WiFi SSID configured yet");
    startAccessPoint();
    setupCaptivePortal();
    return;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
  wifiStartupPending = true;
  wifiStartupAt = millis();
  bootIndicatorActive = HAS_LED_OUTPUT;
  apModeIndicatorActive = false;
  lastWifiRetryAt = wifiStartupAt;
  LOGI("Connecting to WiFi SSID '%s' asynchronously...", config.wifiSsid.c_str());
}

void enterAccessPointMode(const char *reason)
{
  LOGI("Entering AP mode (%s)", reason);
  stopShow();
  setSafeOutputState();
  wifiStartupPending = false;
  bootIndicatorActive = false;
  apModeIndicatorActive = HAS_LED_OUTPUT;
  startAccessPoint();
  if (!captivePortalEnabled)
  {
    setupCaptivePortal();
  }
}

void startAccessPoint()
{
  // AP+STA erlaubt parallel:
  // - lokale Konfiguration ueber AP
  // - gleichzeitiger STA-Reconnect im Hintergrund
  apModeIndicatorActive = HAS_LED_OUTPUT;
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSsid.c_str());

  LOGI("AP started. SSID=%s AP_IP=%s", apSsid.c_str(), WiFi.softAPIP().toString().c_str());

  if (config.wifiSsid.length() > 0)
  {
    WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
    lastWifiRetryAt = millis();
  }
}

void setupCaptivePortal()
{
  // DNS wildcard auf AP-IP, damit Clients automatisch auf das Web-UI landen.
  dnsServer.start(53, "*", WiFi.softAPIP());
  captivePortalEnabled = true;
  LOGI("Captive portal enabled");
}

void setupWebServer()
{
  // Web-API:
  // - "/"          Konfigseite
  // - "/save"      Konfig speichern + reboot
  // - "/status"    Laufzeitstatus JSON (legacy)
  // - "/api/*"     REST-Endpunkte
  // - "/ota/*"     OTA-Status, Check, Update (legacy + UI)
  server.on("/", HTTP_GET, handleRoot);
  server.on("/app.js", HTTP_GET, handleAppJs);
  server.on("/generate_204", HTTP_GET, handleCaptiveProbe);
  server.on("/gen_204", HTTP_GET, handleCaptiveProbe);
  server.on("/hotspot-detect.html", HTTP_GET, handleCaptiveProbe);
  server.on("/library/test/success.html", HTTP_GET, handleCaptiveProbe);
  server.on("/success.txt", HTTP_GET, handleCaptiveProbe);
  server.on("/connecttest.txt", HTTP_GET, handleCaptiveProbe);
  server.on("/redirect", HTTP_GET, handleCaptiveProbe);
  server.on("/fwlink", HTTP_GET, handleCaptiveProbe);
  server.on("/ncsi.txt", HTTP_GET, handleCaptiveProbe);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/wifi/scan", HTTP_GET, handleApiWifiScan);
  server.on("/api/config", HTTP_GET, handleApiConfigGet);
  server.on("/api/config", HTTP_POST, handleApiConfigPost);
  server.on("/api/show", HTTP_GET, handleApiShowGet);
  server.on("/api/show", HTTP_POST, handleApiShowPost);
  server.on("/api/light", HTTP_GET, handleApiLightGet);
  server.on("/api/light", HTTP_POST, handleLightConfig);
  server.on("/api/ota/status", HTTP_GET, handleOtaStatus);
  server.on("/api/ota/check", HTTP_POST, handleOtaCheck);
  server.on("/api/ota/update", HTTP_POST, handleOtaUpdate);
  server.on("/ota/status", HTTP_GET, handleOtaStatus);
  server.on("/ota/check", HTTP_POST, handleOtaCheck);
  server.on("/ota/update", HTTP_POST, handleOtaUpdate);
  server.on("/ota/upload", HTTP_POST, handleOtaUpload, handleOtaUploadData);
  server.onNotFound(handleNotFound);
  server.begin();

  LOGI("Web server started on port 80");
}

void handleRoot()
{
  LOGD("HTTP GET /");
  if (handleCaptivePortalRedirect())
  {
    return;
  }
  sendHtmlPage();
}

void handleAppJs()
{
  LOGD("HTTP GET /app.js");
  static const char APP_JS[] PROGMEM = R"rawliteral(
function parseJsonText(text) {
  try {
    return text ? JSON.parse(text) : {};
  } catch (e) {
    return {};
  }
}

var otaPollTimer = null;
var otaPollFailureCount = 0;
var otaLocalBusyState = '';

function otaSetLocalBusyState(state) {
  otaLocalBusyState = state || '';
}

function otaClearLocalBusyState() {
  otaLocalBusyState = '';
}

function scheduleOtaRefresh(delayMs) {
  if (otaPollTimer) {
    clearTimeout(otaPollTimer);
  }
  otaPollTimer = setTimeout(refreshOta, Math.max(0, Number(delayMs || 0)));
}

function requestJson(path, method, body, contentType, onSuccess, onError, timeoutMs) {
  var xhr = new XMLHttpRequest();
  xhr.open(method || 'GET', path, true);
  xhr.timeout = Number(timeoutMs || 8000);
  if (contentType) {
    xhr.setRequestHeader('Content-Type', contentType);
  }
  xhr.onreadystatechange = function() {
    var json;
    if (xhr.readyState !== 4) return;
    json = parseJsonText(xhr.responseText);
    if (xhr.status >= 200 && xhr.status < 300) {
      if (onSuccess) onSuccess(json, xhr);
      return;
    }
    if (onError) onError(new Error(json.error || ('HTTP ' + xhr.status)), json, xhr);
  };
  xhr.onerror = function() {
    if (onError) onError(new Error('Verbindung fehlgeschlagen'), {}, xhr);
  };
  xhr.ontimeout = function() {
    if (onError) onError(new Error('Zeitueberschreitung beim Warten auf Antwort'), {}, xhr);
  };
  xhr.send(body || null);
}

function apiJson(path, method, onSuccess, onError, timeoutMs) {
  requestJson(path, method || 'GET', null, null, onSuccess, onError, timeoutMs);
}

function describeWifiNetwork(network) {
  var parts = [];
  var rssi = Number(network && network.rssi);
  if (network && network.secure) {
    parts.push('gesichert');
  } else {
    parts.push('offen');
  }
  if (!isNaN(rssi) && rssi !== 0) {
    parts.push(rssi + ' dBm');
  }
  return parts.join(' | ');
}

function renderWifiNetworks(networks) {
  var select = document.getElementById('wifi-network-list');
  var status = document.getElementById('wifi-scan-status');
  var ssidInput = document.getElementById('ssid');
  var currentSsid = ssidInput ? ssidInput.value : '';
  var i;
  var option;
  if (!select) return;
  select.options.length = 0;
  if (!networks || !networks.length) {
    option = document.createElement('option');
    option.value = '';
    option.text = 'Keine Netzwerke gefunden';
    select.appendChild(option);
    if (status) status.innerHTML = 'Es wurden keine sichtbaren WLAN-Netzwerke gefunden.';
    return;
  }
  for (i = 0; i < networks.length; i++) {
    option = document.createElement('option');
    option.value = networks[i].ssid || '';
    option.text = (networks[i].ssid || '(ohne SSID)') + ' - ' + describeWifiNetwork(networks[i]);
    if (currentSsid && option.value === currentSsid) {
      option.selected = true;
    }
    select.appendChild(option);
  }
  if (status) {
    status.innerHTML = String(networks.length) + ' WLAN-Netzwerke gefunden.';
  }
}

function applySelectedWifiSsid() {
  var select = document.getElementById('wifi-network-list');
  var ssidInput = document.getElementById('ssid');
  if (!select || !ssidInput || !select.value) return;
  ssidInput.value = select.value;
}

function refreshWifiScan() {
  var status = document.getElementById('wifi-scan-status');
  if (status) {
    status.innerHTML = 'Suche nach verfuegbaren WLAN-Netzwerken...';
  }
  apiJson(
    '/api/wifi/scan',
    'GET',
    function(response) {
      renderWifiNetworks(response.networks || []);
    },
    function(error) {
      if (status) {
        status.innerHTML = 'WLAN-Scan fehlgeschlagen: ' + error.message;
      }
    },
    15000
  );
}

function initWifiControls() {
  var select = document.getElementById('wifi-network-list');
  var status = document.getElementById('wifi-scan-status');
  if (!select) return;
  select.onchange = applySelectedWifiSsid;
  if (select.options && select.options.length) {
    if (status) {
      status.innerHTML = String(select.options.length) + ' WLAN-Netzwerke vorgeladen.';
    }
  } else if (status) {
    status.innerHTML = 'Noch keine WLAN-Netzwerke geladen.';
  }
}

function formatBytes(value) {
  var bytes = Number(value || 0);
  if (!bytes) return '0 B';
  if (bytes >= 1024 * 1024) return (bytes / (1024 * 1024)).toFixed(2) + ' MB';
  if (bytes >= 1024) return (bytes / 1024).toFixed(1) + ' KB';
  return bytes + ' B';
}

function clampByte(value) {
  var numeric = Number(value || 0);
  return Math.max(0, Math.min(255, Math.round(numeric)));
}

function byteToHex(value) {
  var hex = clampByte(value).toString(16).toUpperCase();
  return hex.length < 2 ? ('0' + hex) : hex;
}

function hexToRgb(hex) {
  var normalized = String(hex || '').replace('#', '');
  if (normalized.length !== 6) {
    return {r: 255, g: 255, b: 255};
  }
  return {
    r: parseInt(normalized.substr(0, 2), 16),
    g: parseInt(normalized.substr(2, 2), 16),
    b: parseInt(normalized.substr(4, 2), 16)
  };
}

function rgbToCss(r, g, b, w) {
  var white = clampByte(w);
  var red = clampByte(r + white);
  var green = clampByte(g + white);
  var blue = clampByte(b + white);
  return 'rgb(' + red + ', ' + green + ', ' + blue + ')';
}

function syncLightPickerFromHidden() {
  var colorInput = document.getElementById('light_color');
  var hiddenR = document.getElementById('light_r');
  var hiddenG = document.getElementById('light_g');
  var hiddenB = document.getElementById('light_b');
  var hex;
  if (!colorInput || !hiddenR || !hiddenG || !hiddenB) return;
  hex = '#' + byteToHex(hiddenR.value) + byteToHex(hiddenG.value) + byteToHex(hiddenB.value);
  colorInput.value = hex;
}

function syncHiddenFromLightPicker() {
  var colorInput = document.getElementById('light_color');
  var hiddenR = document.getElementById('light_r');
  var hiddenG = document.getElementById('light_g');
  var hiddenB = document.getElementById('light_b');
  var rgb;
  if (!colorInput || !hiddenR || !hiddenG || !hiddenB) return;
  rgb = hexToRgb(colorInput.value);
  hiddenR.value = rgb.r;
  hiddenG.value = rgb.g;
  hiddenB.value = rgb.b;
}

function syncWhiteControls(sourceId) {
  var slider = document.getElementById('light_w_slider');
  var input = document.getElementById('light_w');
  var source = sourceId === 'slider' ? slider : input;
  var value;
  if (!slider || !input || !source) return;
  value = clampByte(source.value);
  slider.value = value;
  input.value = value;
}

function updateLightPreviewCard() {
  var preview = document.getElementById('light-preview');
  var text = document.getElementById('light-preview-text');
  var enabled = document.getElementById('light_enabled');
  var hiddenR = document.getElementById('light_r');
  var hiddenG = document.getElementById('light_g');
  var hiddenB = document.getElementById('light_b');
  var white = document.getElementById('light_w');
  var r;
  var g;
  var b;
  var w;
  if (!preview || !text || !hiddenR || !hiddenG || !hiddenB || !white) return;
  r = clampByte(hiddenR.value);
  g = clampByte(hiddenG.value);
  b = clampByte(hiddenB.value);
  w = clampByte(white.value);
  preview.style.background = rgbToCss(r, g, b, w);
  text.innerHTML = 'RGBW: ' + r + ', ' + g + ', ' + b + ', ' + w +
    ' | Show-Beleuchtung: ' + (enabled && enabled.checked ? 'AN' : 'AUS');
}

function buildLightPayloadString(preview) {
  var enabled = document.getElementById('light_enabled');
  var hiddenR = document.getElementById('light_r');
  var hiddenG = document.getElementById('light_g');
  var hiddenB = document.getElementById('light_b');
  var white = document.getElementById('light_w');
  syncHiddenFromLightPicker();
  syncWhiteControls('input');
  updateLightPreviewCard();
  return 'enabled=' + encodeURIComponent(enabled && enabled.checked ? '1' : '0') +
    '&r=' + encodeURIComponent(hiddenR ? hiddenR.value : '0') +
    '&g=' + encodeURIComponent(hiddenG ? hiddenG.value : '0') +
    '&b=' + encodeURIComponent(hiddenB ? hiddenB.value : '0') +
    '&w=' + encodeURIComponent(white ? white.value : '0') +
    '&preview=' + encodeURIComponent(preview ? '1' : '0');
}

function saveLightConfig(preview) {
  if (!document.getElementById('light_color')) return;
  requestJson(
    '/api/light',
    'POST',
    buildLightPayloadString(preview),
    'application/x-www-form-urlencoded;charset=UTF-8',
    function() {
      updateLightPreviewCard();
    },
    function(error) {
      alert('Beleuchtung konnte nicht gespeichert werden: ' + error.message);
    }
  );
}

function previewLightConfig() {
  saveLightConfig(true);
}

function setOtaProgress(percent, text) {
  var bar = document.getElementById('ota-progress-bar');
  var label = document.getElementById('ota-progress-text');
  var safePercent = Math.max(0, Math.min(100, Number(percent || 0)));
  if (bar) {
    bar.style.width = safePercent + '%';
  }
  if (label) {
    label.innerHTML = text || '';
  }
}

function renderOta(status) {
  var el = document.getElementById('ota-box');
  var text = '';
  var progressText = status.status_text || status.phase || 'Bereit';
  var busy = !!(status.check_in_progress || status.update_in_progress || status.reboot_pending ||
    status.phase === 'queued' || status.phase === 'checking' || status.phase === 'downloading' || status.phase === 'flashing');
  if (!el) return;
  otaPollFailureCount = 0;
  if (!busy) {
    otaClearLocalBusyState();
  }
  text += 'Konfiguriert: ' + (status.configured ? 'ja' : 'nein') + '<br>';
  text += 'Manifest URL: ' + (status.manifest_url || '-') + '<br>';
  text += 'WLAN verbunden: ' + (status.wifi_connected ? 'ja' : 'nein') + '<br>';
  text += 'Aktuell: ' + (status.current_version || '-') + '<br>';
  text += 'Neueste Version: ' + (status.latest_version || '-') + '<br>';
  text += 'Update verfuegbar: ' + (status.update_available ? 'ja' : 'nein') + '<br>';
  text += 'Quelle: ' + (status.source || '-') + '<br>';
  text += 'Status: ' + (status.status_text || status.phase || '-') + '<br>';
  if (status.last_error) {
    text += 'Letzter Fehler: ' + status.last_error + '<br>';
  }
  if (status.release_notes) {
    text += 'Hinweis: ' + status.release_notes + '<br>';
  }
  el.innerHTML = text;
  if (status.progress_total > 0) {
    progressText += ' (' + (status.progress_percent || 0) + '% / ' +
      formatBytes(status.progress_bytes) + ' von ' + formatBytes(status.progress_total) + ')';
  } else if (status.progress_bytes > 0) {
    progressText += ' (' + formatBytes(status.progress_bytes) + ')';
  }
  if (status.reboot_pending) {
    progressText += ' - Neustart ausstehend';
  }
  setOtaProgress(status.progress_percent || 0, progressText);
  scheduleOtaRefresh(busy ? 2000 : 5000);
}

function renderOtaOfflineHint(error) {
  var el = document.getElementById('ota-box');
  var message = '';
  if (!el) return;
  if (otaLocalBusyState === 'update') {
    message = 'Online-Update wurde gestartet.<br>' +
      'Waehend Download, Flash und Neustart antwortet das Geraet zeitweise nicht auf Statusabfragen.<br>' +
      'Bitte das Geraet in dieser Phase nicht manuell neu starten oder ausschalten.<br>' +
      'Letzter Verbindungsfehler: ' + (error && error.message ? error.message : 'unbekannt');
    el.innerHTML = message;
    setOtaProgress(100, 'Update laeuft oder Neustart wird vorbereitet. Verbindung wird erneut geprueft...');
    return;
  }
  if (otaLocalBusyState === 'check') {
    message = 'OTA-Pruefung wurde gestartet.<br>' +
      'Waehend der Pruefung kann die Statusabfrage kurzzeitig aussetzen.<br>' +
      'Letzter Verbindungsfehler: ' + (error && error.message ? error.message : 'unbekannt');
    el.innerHTML = message;
    setOtaProgress(0, 'Pruefe erneut auf Rueckmeldung...');
    return;
  }
  el.innerHTML = 'OTA-Status konnte nicht geladen werden: ' + (error && error.message ? error.message : 'unbekannt');
}

function refreshOta() {
  apiJson(
    '/ota/status',
    'GET',
    function(status) {
      renderOta(status);
    },
    function(error) {
      otaPollFailureCount += 1;
      renderOtaOfflineHint(error);
      if (otaLocalBusyState === 'update') {
        scheduleOtaRefresh(Math.min(15000, 4000 + (otaPollFailureCount * 1000)));
        return;
      }
      if (otaLocalBusyState === 'check') {
        scheduleOtaRefresh(Math.min(10000, 2500 + (otaPollFailureCount * 1000)));
        return;
      }
      scheduleOtaRefresh(8000);
    },
    2500
  );
}

function otaCheck() {
  apiJson(
    '/ota/check',
    'POST',
    function() {
      otaSetLocalBusyState('check');
      setOtaProgress(0, 'OTA-Pruefung wurde gestartet...');
      scheduleOtaRefresh(400);
    },
    function(error) {
      alert('OTA-Check fehlgeschlagen: ' + error.message);
      scheduleOtaRefresh(2000);
    },
    5000
  );
}

function otaUpdate() {
  if (!confirm('Firmware-Update jetzt starten?')) return;
  apiJson(
    '/ota/update',
    'POST',
    function() {
      otaSetLocalBusyState('update');
      setOtaProgress(100, 'Online-Update wurde gestartet. Das Geraet kann waehrend Download, Flash und Neustart kurzzeitig nicht antworten.');
      scheduleOtaRefresh(1500);
    },
    function(error) {
      alert('OTA-Update fehlgeschlagen: ' + error.message);
      scheduleOtaRefresh(2000);
    },
    5000
  );
}

function otaUploadFile() {
  var input = document.getElementById('ota-file');
  var file;
  var form;
  var xhr;
  if (!input || !input.files || !input.files.length) {
    alert('Bitte zuerst eine Firmware-Datei auswaehlen.');
    return;
  }
  file = input.files[0];
  if (!confirm('Firmware-Datei jetzt hochladen und installieren?')) return;
  form = new FormData();
  form.append('firmware', file, file.name);
  setOtaProgress(0, 'Upload wird gestartet...');
  xhr = new XMLHttpRequest();
  xhr.open('POST', '/ota/upload', true);
  if (xhr.upload) {
    xhr.upload.onprogress = function(event) {
      var percent;
      if (!event.lengthComputable) return;
      percent = Math.round((event.loaded / event.total) * 100);
      setOtaProgress(percent, 'Upload laeuft (' + percent + '% / ' +
        formatBytes(event.loaded) + ' von ' + formatBytes(event.total) + ')');
    };
  }
  xhr.onreadystatechange = function() {
    var json;
    if (xhr.readyState !== 4) return;
    json = parseJsonText(xhr.responseText);
    if (xhr.status >= 200 && xhr.status < 300) {
      input.value = '';
      setTimeout(refreshOta, 200);
      return;
    }
    alert('Firmware-Upload fehlgeschlagen: ' + (json.error || ('HTTP ' + xhr.status)));
    setTimeout(refreshOta, 200);
  };
  xhr.onerror = function() {
    alert('Firmware-Upload fehlgeschlagen: Upload-Verbindung fehlgeschlagen');
    setTimeout(refreshOta, 200);
  };
  xhr.send(form);
}

function initLightControls() {
  var colorInput = document.getElementById('light_color');
  var enabled = document.getElementById('light_enabled');
  var whiteSlider = document.getElementById('light_w_slider');
  var whiteInput = document.getElementById('light_w');
  if (!colorInput || !enabled || !whiteSlider || !whiteInput) return;
  syncLightPickerFromHidden();
  syncWhiteControls('input');
  updateLightPreviewCard();
  colorInput.onchange = function() { saveLightConfig(true); };
  enabled.onchange = function() { saveLightConfig(false); };
  whiteSlider.oninput = function() {
    syncWhiteControls('slider');
    updateLightPreviewCard();
  };
  whiteSlider.onchange = function() { saveLightConfig(true); };
  whiteInput.oninput = function() {
    syncWhiteControls('input');
    updateLightPreviewCard();
  };
  whiteInput.onchange = function() { saveLightConfig(true); };
}

refreshOta();
initWifiControls();
initLightControls();
)rawliteral";
  server.send_P(200, "application/javascript", APP_JS);
}

void handleCaptiveProbe()
{
  LOGD("HTTP captive probe: %s", server.uri().c_str());
  if (handleCaptivePortalRedirect())
  {
    return;
  }
  sendHtmlPage();
}

void handleSave()
{
  // Uebernimmt Formwerte, validiert/clamped sie, speichert und rebootet.
  LOGI("HTTP /save requested");
  if (server.hasArg("ssid"))
  {
    config.wifiSsid = server.arg("ssid");
  }
  if (server.hasArg("wifi_password"))
  {
    config.wifiPassword = server.arg("wifi_password");
  }
  if (server.hasArg("mqtt_host"))
  {
    config.mqttHost = server.arg("mqtt_host");
  }
  if (server.hasArg("mqtt_port"))
  {
    config.mqttPort = (uint16_t)clampU32((uint32_t)server.arg("mqtt_port").toInt(), 1, 65535);
  }
  if (server.hasArg("mqtt_user"))
  {
    config.mqttUser = server.arg("mqtt_user");
  }
  if (server.hasArg("mqtt_password"))
  {
    config.mqttPassword = server.arg("mqtt_password");
  }
  if (server.hasArg("mqtt_topic"))
  {
    config.mqttTopic = server.arg("mqtt_topic");
  }
  if (server.hasArg("ota_manifest_url"))
  {
    config.otaManifestUrl = server.arg("ota_manifest_url");
  }
  if (server.hasArg("show_length"))
  {
    config.showLengthMs = clampU32((uint32_t)server.arg("show_length").toInt(), SHOW_LENGTH_MIN_MS, SHOW_LENGTH_MAX_MS);
  }
  if (server.hasArg("show_nachlauf"))
  {
    config.showNachlaufMs = clampMinU32((uint32_t)server.arg("show_nachlauf").toInt(), SHOW_NACHLAUF_MIN_MS);
  }
  if (HAS_LED_OUTPUT)
  {
    config.lightEnabled = server.hasArg("light_enabled") && server.arg("light_enabled") != "0";
    if (server.hasArg("light_r"))
    {
      config.lightR = clampU8(server.arg("light_r").toInt());
    }
    if (server.hasArg("light_g"))
    {
      config.lightG = clampU8(server.arg("light_g").toInt());
    }
    if (server.hasArg("light_b"))
    {
      config.lightB = clampU8(server.arg("light_b").toInt());
    }
    if (server.hasArg("light_w"))
    {
      config.lightW = clampU8(server.arg("light_w").toInt());
    }
  }

  sanitizeConfig();
  applyConfigToRuntime(true, true, true);

  if (!saveConfig())
  {
    LOGW("Config save failed");
    sendHtmlPage("Fehler beim Speichern.", 500);
    return;
  }

  LOGI("Config saved, restarting device");
  sendHtmlPage("Gespeichert. Das Geraet startet neu...");
  delay(500);
  ESP.restart();
}

void handleApiWifiScan()
{
  LOGD("HTTP GET /api/wifi/scan");
  int networkCount = WiFi.scanNetworks();
  if (networkCount < 0)
  {
    sendJsonError(500, "WLAN-Scan fehlgeschlagen");
    return;
  }

  JsonDocument doc;
  JsonArray networks = doc["networks"].to<JsonArray>();
  for (int i = 0; i < networkCount; i++)
  {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0)
    {
      continue;
    }
    JsonObject network = networks.add<JsonObject>();
    network["ssid"] = ssid;
    network["rssi"] = WiFi.RSSI(i);
    network["secure"] = WiFi.encryptionType(i) != ENC_TYPE_NONE;
  }
  WiFi.scanDelete();
  doc["ok"] = true;
  sendJsonDocument(200, doc);
}

void handleApiConfigGet()
{
  LOGD("HTTP GET /api/config");
  JsonDocument doc;
  buildConfigJson(doc.to<JsonObject>());
  sendJsonDocument(200, doc);
}

void handleApiConfigPost()
{
  LOGI("HTTP POST /api/config");
  JsonDocument doc;
  String errorText;
  if (!parseJsonBody(doc, errorText))
  {
    sendJsonError(400, errorText);
    return;
  }

  bool wifiChanged = false;
  bool mqttChanged = false;
  bool lightChanged = false;
  bool lightColorChanged = false;

  if (doc["wifi"].is<JsonObject>())
  {
    JsonObject wifi = doc["wifi"].as<JsonObject>();
    if (!wifi["ssid"].isNull())
    {
      String value = wifi["ssid"].as<String>();
      if (config.wifiSsid != value)
      {
        config.wifiSsid = value;
        wifiChanged = true;
      }
    }
    if (!wifi["password"].isNull())
    {
      String value = wifi["password"].as<String>();
      if (config.wifiPassword != value)
      {
        config.wifiPassword = value;
        wifiChanged = true;
      }
    }
  }

  if (doc["mqtt"].is<JsonObject>())
  {
    JsonObject mqtt = doc["mqtt"].as<JsonObject>();
    if (!mqtt["host"].isNull())
    {
      String value = mqtt["host"].as<String>();
      if (config.mqttHost != value)
      {
        config.mqttHost = value;
        mqttChanged = true;
      }
    }
    if (!mqtt["port"].isNull())
    {
      uint16_t value = (uint16_t)clampU32((uint32_t)mqtt["port"].as<unsigned long>(), 1, 65535);
      if (config.mqttPort != value)
      {
        config.mqttPort = value;
        mqttChanged = true;
      }
    }
    if (!mqtt["user"].isNull())
    {
      String value = mqtt["user"].as<String>();
      if (config.mqttUser != value)
      {
        config.mqttUser = value;
        mqttChanged = true;
      }
    }
    if (!mqtt["password"].isNull())
    {
      String value = mqtt["password"].as<String>();
      if (config.mqttPassword != value)
      {
        config.mqttPassword = value;
        mqttChanged = true;
      }
    }
    if (!mqtt["topic"].isNull())
    {
      String value = mqtt["topic"].as<String>();
      if (config.mqttTopic != value)
      {
        config.mqttTopic = value;
        mqttChanged = true;
      }
    }
  }

  if (doc["ota"].is<JsonObject>())
  {
    JsonObject ota = doc["ota"].as<JsonObject>();
    if (!ota["manifest_url"].isNull())
    {
      config.otaManifestUrl = ota["manifest_url"].as<String>();
    }
  }

  if (doc["show"].is<JsonObject>())
  {
    JsonObject show = doc["show"].as<JsonObject>();
    if (!show["length_ms"].isNull())
    {
      config.showLengthMs = clampU32(show["length_ms"].as<unsigned long>(), SHOW_LENGTH_MIN_MS, SHOW_LENGTH_MAX_MS);
    }
    else if (!show["length_s"].isNull())
    {
      config.showLengthMs = clampU32(secToMs(show["length_s"].as<unsigned long>()), SHOW_LENGTH_MIN_MS, SHOW_LENGTH_MAX_MS);
    }

    if (!show["nachlauf_ms"].isNull())
    {
      config.showNachlaufMs = clampMinU32(show["nachlauf_ms"].as<unsigned long>(), SHOW_NACHLAUF_MIN_MS);
    }
    else if (!show["nachlauf_s"].isNull())
    {
      config.showNachlaufMs = clampMinU32(secToMs(show["nachlauf_s"].as<unsigned long>()), SHOW_NACHLAUF_MIN_MS);
    }
  }

  if (HAS_LED_OUTPUT && doc["light"].is<JsonObject>())
  {
    JsonObject light = doc["light"].as<JsonObject>();
    if (!light["enabled"].isNull())
    {
      bool value = light["enabled"].as<bool>();
      if (config.lightEnabled != value)
      {
        config.lightEnabled = value;
        lightChanged = true;
      }
    }
    if (!light["r"].isNull())
    {
      uint8_t value = clampU8(light["r"].as<int>());
      if (config.lightR != value)
      {
        config.lightR = value;
        lightChanged = true;
        lightColorChanged = true;
      }
    }
    if (!light["g"].isNull())
    {
      uint8_t value = clampU8(light["g"].as<int>());
      if (config.lightG != value)
      {
        config.lightG = value;
        lightChanged = true;
        lightColorChanged = true;
      }
    }
    if (!light["b"].isNull())
    {
      uint8_t value = clampU8(light["b"].as<int>());
      if (config.lightB != value)
      {
        config.lightB = value;
        lightChanged = true;
        lightColorChanged = true;
      }
    }
    if (!light["w"].isNull())
    {
      uint8_t value = clampU8(light["w"].as<int>());
      if (config.lightW != value)
      {
        config.lightW = value;
        lightChanged = true;
        lightColorChanged = true;
      }
    }
  }

  sanitizeConfig();
  if (!saveConfig())
  {
    sendJsonError(500, "Konfiguration konnte nicht gespeichert werden");
    return;
  }

  if (lightColorChanged && !showStatus.isRunning)
  {
    requestLightPreview();
  }
  else if (!config.lightEnabled && !showStatus.isRunning)
  {
    lightControl.previewUntilAt = 0;
  }

  applyConfigToRuntime(wifiChanged, mqttChanged, true);

  JsonDocument response;
  response["ok"] = true;
  response["wifi_changed"] = wifiChanged;
  response["mqtt_changed"] = mqttChanged;
  response["light_changed"] = lightChanged;
  buildConfigJson(response["config"].to<JsonObject>());
  sendJsonDocument(200, response);
}

void handleApiShowGet()
{
  LOGD("HTTP GET /api/show");
  JsonDocument doc;
  doc["running"] = showStatus.isRunning;
  doc["phase"] = getShowPhase();
  doc["length_ms"] = config.showLengthMs;
  doc["nachlauf_ms"] = config.showNachlaufMs;
  doc["fade_in_done_at_ms"] = showStatus.fadeInDoneAt;
  doc["end_at_ms"] = showStatus.endAt;
  doc["open_valve_at_ms"] = showStatus.openValveAt;
  doc["finish_at_ms"] = showStatus.finishAt;
  sendJsonDocument(200, doc);
}

void handleApiShowPost()
{
  LOGI("HTTP POST /api/show");
  JsonDocument doc;
  String errorText;
  if (!parseJsonBody(doc, errorText))
  {
    sendJsonError(400, errorText);
    return;
  }

  String state = doc["state"] | "";
  state.trim();
  state.toUpperCase();
  if (state == "ON")
  {
    startShow();
  }
  else if (state == "OFF")
  {
    stopShow();
  }
  else
  {
    sendJsonError(400, "state muss ON oder OFF sein");
    return;
  }

  publishShowState(true);
  publishTelemetry(true);
  JsonDocument response;
  response["ok"] = true;
  response["queued_state"] = state;
  response["running"] = showStatus.isRunning;
  response["phase"] = getShowPhase();
  response["fade_in_done_at_ms"] = showStatus.fadeInDoneAt;
  response["end_at_ms"] = showStatus.endAt;
  response["open_valve_at_ms"] = showStatus.openValveAt;
  response["finish_at_ms"] = showStatus.finishAt;
  sendJsonDocument(202, response);
}

void handleApiLightGet()
{
  LOGD("HTTP GET /api/light");
  if (!HAS_LED_OUTPUT)
  {
    sendJsonError(404, "Keine LED-Hardware konfiguriert");
    return;
  }
  JsonDocument doc;
  buildLightStatusJson(doc);
  sendJsonDocument(200, doc);
}

void handleLightConfig()
{
  LOGI("HTTP /api/light requested");
  if (!HAS_LED_OUTPUT)
  {
    sendJsonError(404, "Keine LED-Hardware konfiguriert");
    return;
  }

  bool changed = false;
  bool colorChanged = false;
  bool previewRequested = false;
  JsonDocument requestDoc;
  String errorText;
  bool hasJsonBody = requestBodyLooksLikeJson();

  if (hasJsonBody)
  {
    if (!parseJsonBody(requestDoc, errorText))
    {
      sendJsonError(400, errorText);
      return;
    }
    if (!requestDoc["enabled"].isNull())
    {
      bool newEnabled = requestDoc["enabled"].as<bool>();
      if (config.lightEnabled != newEnabled)
      {
        config.lightEnabled = newEnabled;
        changed = true;
      }
    }
    if (!requestDoc["preview"].isNull())
    {
      previewRequested = requestDoc["preview"].as<bool>();
    }
    if (!requestDoc["r"].isNull())
    {
      uint8_t value = clampU8(requestDoc["r"].as<int>());
      if (config.lightR != value)
      {
        config.lightR = value;
        changed = true;
        colorChanged = true;
      }
    }
    if (!requestDoc["g"].isNull())
    {
      uint8_t value = clampU8(requestDoc["g"].as<int>());
      if (config.lightG != value)
      {
        config.lightG = value;
        changed = true;
        colorChanged = true;
      }
    }
    if (!requestDoc["b"].isNull())
    {
      uint8_t value = clampU8(requestDoc["b"].as<int>());
      if (config.lightB != value)
      {
        config.lightB = value;
        changed = true;
        colorChanged = true;
      }
    }
    if (!requestDoc["w"].isNull())
    {
      uint8_t value = clampU8(requestDoc["w"].as<int>());
      if (config.lightW != value)
      {
        config.lightW = value;
        changed = true;
        colorChanged = true;
      }
    }
  }
  else
  {
    previewRequested = server.hasArg("preview") && server.arg("preview") != "0";
    if (server.hasArg("enabled"))
    {
      bool newEnabled = server.arg("enabled") != "0";
      if (config.lightEnabled != newEnabled)
      {
        config.lightEnabled = newEnabled;
        changed = true;
      }
    }
    if (server.hasArg("r"))
    {
      uint8_t value = clampU8(server.arg("r").toInt());
      if (config.lightR != value)
      {
        config.lightR = value;
        changed = true;
        colorChanged = true;
      }
    }
    if (server.hasArg("g"))
    {
      uint8_t value = clampU8(server.arg("g").toInt());
      if (config.lightG != value)
      {
        config.lightG = value;
        changed = true;
        colorChanged = true;
      }
    }
    if (server.hasArg("b"))
    {
      uint8_t value = clampU8(server.arg("b").toInt());
      if (config.lightB != value)
      {
        config.lightB = value;
        changed = true;
        colorChanged = true;
      }
    }
    if (server.hasArg("w"))
    {
      uint8_t value = clampU8(server.arg("w").toInt());
      if (config.lightW != value)
      {
        config.lightW = value;
        changed = true;
        colorChanged = true;
      }
    }
  }

  if (previewRequested || colorChanged)
  {
    requestLightPreview();
  }
  else if (!config.lightEnabled && !showStatus.isRunning)
  {
    lightControl.previewUntilAt = 0;
  }

  if (changed)
  {
    scheduleConfigSave();
    publishLightState(true);
    publishTelemetry(true);
  }

  JsonDocument status;
  buildLightStatusJson(status);
  status["preview_requested"] = previewRequested || colorChanged;
  sendJsonDocument(200, status);
}

void handleStatus()
{
  LOGD("HTTP GET /status");
  JsonDocument status;
  buildStatusJson(status);
  sendJsonDocument(200, status);
}

void handleOtaStatus()
{
  LOGD("HTTP GET /ota/status");
  JsonDocument status;
  buildOtaStatusJson(status);
  sendJsonDocument(200, status);
}

void handleOtaCheck()
{
  LOGI("HTTP POST /ota/check");
  if (config.otaManifestUrl.length() == 0)
  {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"OTA-Manifest-URL fehlt\"}");
    return;
  }
  if (WiFi.status() != WL_CONNECTED)
  {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"WLAN nicht verbunden\"}");
    return;
  }
  if (otaStatus.checkInProgress || otaStatus.updateInProgress)
  {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"OTA bereits aktiv\"}");
    return;
  }

  otaStatus.lastError = "";
  otaResetProgress("queued", "manifest", "OTA-Pruefung wurde gestartet");
  otaCheckRequested = true;
  server.send(202, "application/json", "{\"ok\":true,\"queued\":true}");
}

void handleOtaUpdate()
{
  LOGI("HTTP POST /ota/update");
  if (WiFi.status() != WL_CONNECTED)
  {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"WLAN nicht verbunden\"}");
    return;
  }
  if (showStatus.isRunning)
  {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"Show laeuft noch\"}");
    return;
  }
  if (otaStatus.updateInProgress || otaStatus.checkInProgress)
  {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"OTA bereits aktiv\"}");
    return;
  }
  if (!otaStatus.updateAvailable || otaStatus.firmwareUrl.length() == 0)
  {
    server.send(409, "application/json", "{\"ok\":false,\"error\":\"Kein Update verfuegbar\"}");
    return;
  }

  otaStatus.lastError = "";
  otaResetProgress("queued", "manifest", "Online-Update wurde gestartet");
  otaUpdateRequested = true;
  server.send(202, "application/json", "{\"ok\":true,\"queued\":true}");
}

void handleOtaUpload()
{
  LOGI("HTTP POST /ota/upload");
  JsonDocument doc;

  if (otaStatus.rebootPending && otaStatus.source == "browser")
  {
    doc["ok"] = true;
    doc["rebooting"] = true;
    doc["status"] = otaStatus.statusText;
    String out;
    serializeJson(doc, out);
    server.send(200, "application/json", out);
    return;
  }

  if (otaStatus.phase == "failed" && otaStatus.source == "browser")
  {
    doc["ok"] = false;
    doc["error"] = otaStatus.lastError.length() > 0 ? otaStatus.lastError : "Firmware-Upload fehlgeschlagen";
    String out;
    serializeJson(doc, out);
    server.send(500, "application/json", out);
    return;
  }

  doc["ok"] = otaStatus.updateInProgress && otaStatus.source == "browser";
  doc["queued"] = otaStatus.updateInProgress;
  doc["status"] = otaStatus.statusText;
  String out;
  serializeJson(doc, out);
  server.send(otaStatus.updateInProgress ? 202 : 409, "application/json", out);
}

void handleOtaUploadData()
{
  HTTPUpload &upload = server.upload();

  if (upload.status == UPLOAD_FILE_START)
  {
    LOGI("OTA upload start: %s", upload.filename.c_str());

    if (showStatus.isRunning)
    {
      otaResetProgress("failed", "browser", "Show laeuft noch");
      otaStatus.lastError = "Show laeuft noch";
      return;
    }
    if (otaStatus.checkInProgress || otaStatus.updateInProgress)
    {
      otaResetProgress("failed", "browser", "OTA bereits aktiv");
      otaStatus.lastError = "OTA bereits aktiv";
      return;
    }

    otaStatus.updateInProgress = true;
    otaStatus.lastError = "";
    otaResetProgress("uploading", "browser", "Firmware wird hochgeladen");
    otaStatus.progressTotal = (uint32_t)upload.contentLength;
    setSafeOutputState();

    if (mqttClient.connected())
    {
      publishTelemetry(true);
      publishAvailability("offline", true);
    }
    mqttClient.disconnect();

    size_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
    if (!Update.begin(maxSketchSpace))
    {
      otaFinalizeFailure(Update.getErrorString());
      LOGW("OTA upload begin failed: %s", otaStatus.lastError.c_str());
      return;
    }
  }

  if (upload.status == UPLOAD_FILE_WRITE)
  {
    if (!otaStatus.updateInProgress || otaStatus.source != "browser")
    {
      return;
    }

    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
    {
      otaFinalizeFailure(Update.getErrorString());
      LOGW("OTA upload write failed: %s", otaStatus.lastError.c_str());
      return;
    }

    size_t written = upload.totalSize + upload.currentSize;
    otaSetProgress(written, otaStatus.progressTotal);
    otaSetPhase("uploading", "Firmware wird hochgeladen");
    return;
  }

  if (upload.status == UPLOAD_FILE_END)
  {
    if (!otaStatus.updateInProgress || otaStatus.source != "browser")
    {
      return;
    }

    if (!Update.end(true))
    {
      otaFinalizeFailure(Update.getErrorString());
      LOGW("OTA upload finalize failed: %s", otaStatus.lastError.c_str());
      return;
    }

    otaStatus.lastError = "";
    otaSetProgress(upload.totalSize, upload.totalSize);
    otaScheduleRestart("Upload abgeschlossen, Neustart laeuft");
    publishTelemetry(true);
    LOGI("OTA upload done: %u bytes", (unsigned int)upload.totalSize);
    return;
  }

  if (upload.status == UPLOAD_FILE_ABORTED)
  {
    otaFinalizeFailure("Firmware-Upload abgebrochen");
    LOGW("OTA upload aborted");
  }
}

void handleNotFound()
{
  // Captive-Portal-Umleitung fuer "fremde" Hostnamen.
  LOGD("HTTP NotFound: %s", server.uri().c_str());
  if (handleCaptivePortalRedirect())
  {
    return;
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

bool handleCaptivePortalRedirect()
{
  // Leitet nur um, wenn Hostname keine direkte IP/AP-Adresse ist.
  if (!captivePortalEnabled)
  {
    return false;
  }

  String host = server.hostHeader();
  if (host.length() == 0)
  {
    return false;
  }

  if (isIpAddress(host) || host == WiFi.softAPIP().toString() || host == WiFi.localIP().toString())
  {
    return false;
  }

  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString(), true);
  server.send(302, "text/plain", "");
  LOGD("Captive redirect for host=%s", host.c_str());
  return true;
}

void loadConfig()
{
  // Liest JSON-Konfiguration aus LittleFS.
  // Fallbacks:
  // - Datei fehlt/leer -> Defaults + sofort speichern
  // - Legacy-Textformat -> migrieren und als JSON neu speichern
  config = AppConfig();
  LOGI("Loading config from %s", CONFIG_FILE);

  if (!LittleFS.exists(CONFIG_FILE))
  {
    config.mqttTopic = deviceId;
    saveConfig();
    showStatus.showDuration = config.showLengthMs;
    showStatus.showNachlauf = config.showNachlaufMs;
    return;
  }

  File f = LittleFS.open(CONFIG_FILE, "r");
  if (!f)
  {
    LOGW("Could not open config file");
    return;
  }

  String raw = f.readString();
  f.close();

  String trimmed = raw;
  trimmed.trim();

  if (trimmed.length() == 0)
  {
    config.mqttTopic = deviceId;
    saveConfig();
    showStatus.showDuration = config.showLengthMs;
    showStatus.showNachlauf = config.showNachlaufMs;
    return;
  }

  if (trimmed[0] == '{')
  {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, raw);
    if (!err)
    {
      config.wifiSsid = doc["wifi"]["ssid"] | config.wifiSsid;
      config.wifiPassword = doc["wifi"]["password"] | config.wifiPassword;
      config.mqttHost = doc["mqtt"]["host"] | config.mqttHost;
      config.mqttPort = doc["mqtt"]["port"] | config.mqttPort;
      config.mqttUser = doc["mqtt"]["user"] | config.mqttUser;
      config.mqttPassword = doc["mqtt"]["password"] | config.mqttPassword;
      config.mqttTopic = doc["mqtt"]["topic"] | config.mqttTopic;
      config.otaManifestUrl = doc["ota"]["manifest_url"] | config.otaManifestUrl;
      config.showLengthMs = doc["show"]["length_ms"] | config.showLengthMs;
      config.showNachlaufMs = doc["show"]["nachlauf_ms"] | config.showNachlaufMs;
      config.lightEnabled = doc["light"]["enabled"] | config.lightEnabled;
      config.lightR = doc["light"]["r"] | config.lightR;
      config.lightG = doc["light"]["g"] | config.lightG;
      config.lightB = doc["light"]["b"] | config.lightB;
      config.lightW = doc["light"]["w"] | config.lightW;
    }
    else
    {
      LOGW("Config JSON parse failed, loading defaults");
    }
  }
  else
  {
    loadLegacyConfig(raw);
    saveConfig();
  }

  sanitizeConfig();
  showStatus.showDuration = config.showLengthMs;
  showStatus.showNachlauf = config.showNachlaufMs;
}

void loadLegacyConfig(const String &raw)
{
  // Alte zeilenbasierte Konfiguration wird positionsbasiert eingelesen.
  const int MAX_LINES = 16;
  String lines[MAX_LINES];
  int count = 0;
  int start = 0;
  int rawLen = (int)raw.length();

  while (start <= rawLen && count < MAX_LINES)
  {
    int end = raw.indexOf('\n', start);
    if (end < 0)
    {
      end = raw.length();
    }
    lines[count] = raw.substring(start, end);
    lines[count].trim();
    count++;
    start = end + 1;
    if (end >= (int)raw.length())
    {
      break;
    }
  }

  if (count > 0)
    config.wifiSsid = lines[0];
  if (count > 1)
    config.wifiPassword = lines[1];
  if (count > 2)
    config.mqttHost = lines[2];
  if (count > 3)
    config.mqttPort = (uint16_t)lines[3].toInt();
  if (count > 4)
    config.mqttUser = lines[4];
  if (count > 5)
    config.mqttPassword = lines[5];
  if (count > 6)
    config.mqttTopic = lines[6];
  if (count > 7)
    config.showLengthMs = (uint32_t)lines[7].toInt();
  if (count > 8)
    config.showNachlaufMs = (uint32_t)lines[8].toInt();
  if (count > 9)
    config.lightEnabled = lines[9] != "0";
  if (count > 10)
    config.lightR = clampU8(lines[10].toInt());
  if (count > 11)
    config.lightG = clampU8(lines[11].toInt());
  if (count > 12)
    config.lightB = clampU8(lines[12].toInt());
  if (count > 13)
    config.lightW = clampU8(lines[13].toInt());
}

bool saveConfig()
{
  // Persistiert die aktuelle AppConfig in /config.json.
  JsonDocument doc;

  JsonObject wifi = doc["wifi"].to<JsonObject>();
  wifi["ssid"] = config.wifiSsid;
  wifi["password"] = config.wifiPassword;

  JsonObject mqtt = doc["mqtt"].to<JsonObject>();
  mqtt["host"] = config.mqttHost;
  mqtt["port"] = config.mqttPort;
  mqtt["user"] = config.mqttUser;
  mqtt["password"] = config.mqttPassword;
  mqtt["topic"] = config.mqttTopic;

  JsonObject ota = doc["ota"].to<JsonObject>();
  ota["manifest_url"] = config.otaManifestUrl;

  JsonObject show = doc["show"].to<JsonObject>();
  show["length_ms"] = config.showLengthMs;
  show["nachlauf_ms"] = config.showNachlaufMs;

  JsonObject light = doc["light"].to<JsonObject>();
  light["enabled"] = config.lightEnabled;
  light["r"] = config.lightR;
  light["g"] = config.lightG;
  light["b"] = config.lightB;
  light["w"] = config.lightW;

  File f = LittleFS.open(CONFIG_FILE, "w");
  if (!f)
  {
    LOGW("Could not write config file");
    return false;
  }

  serializeJson(doc, f);
  f.close();
  LOGI("Config saved to LittleFS");
  return true;
}

bool isIpAddress(const String &host)
{
  for (size_t i = 0; i < host.length(); i++)
  {
    char c = host[i];
    if ((c != '.') && (c < '0' || c > '9'))
    {
      return false;
    }
  }
  return true;
}

bool isLegacyOtaManifestUrl(const String &url)
{
  // Erkennt fruehere OTA-URL-Varianten, um auf neues Schema zu migrieren.
  String normalized = url;
  normalized.trim();
  normalized.toLowerCase();
  return normalized.endsWith("/releases/latest/download/manifest.json") ||
         normalized.indexOf("raw.githubusercontent.com/kingbear79/vacubear/") >= 0;
}

String defaultApSsid()
{
  char buf[24];
  snprintf(buf, sizeof(buf), "VacUBear-%06X", ESP.getChipId());
  return String(buf);
}

String defaultDeviceId()
{
  // Device-ID-Quelle:
  // - bevorzugt komplette MAC (stabil + weltweit eindeutig)
  // - fallback auf Chip-ID
  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  WiFi.macAddress(mac);

  bool validMac = false;
  for (int i = 0; i < 6; i++)
  {
    if (mac[i] != 0x00 && mac[i] != 0xFF)
    {
      validMac = true;
      break;
    }
  }

  char buf[32];
  if (validMac)
  {
    snprintf(buf, sizeof(buf), "%s-%02X%02X%02X%02X%02X%02X", DEVICE_PREFIX, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  }
  else
  {
    snprintf(buf, sizeof(buf), "%s-%06X", DEVICE_PREFIX, ESP.getChipId());
  }
  return String(buf);
}

String rgbToHex(uint8_t r, uint8_t g, uint8_t b)
{
  char buf[8];
  snprintf(buf, sizeof(buf), "#%02X%02X%02X", r, g, b);
  return String(buf);
}

uint32_t clampU32(uint32_t value, uint32_t minValue, uint32_t maxValue)
{
  if (value < minValue)
    return minValue;
  if (value > maxValue)
    return maxValue;
  return value;
}

uint32_t clampMinU32(uint32_t value, uint32_t minValue)
{
  if (value < minValue)
    return minValue;
  return value;
}

uint32_t secToMs(uint32_t seconds)
{
  if (seconds > (UINT32_MAX / 1000UL))
  {
    return UINT32_MAX;
  }
  return seconds * 1000UL;
}

uint32_t msToSec(uint32_t milliseconds)
{
  return milliseconds / 1000UL;
}

uint8_t clampU8(int value)
{
  if (value < 0)
    return 0;
  if (value > 255)
    return 255;
  return static_cast<uint8_t>(value);
}

void scheduleConfigSave()
{
  // Entkoppelt haeufige Eingaben von Flash-Schreibzyklen.
  // Jede neue Aenderung schiebt den Save-Zeitpunkt nach hinten.
  configSavePending = true;
  configSaveAt = millis() + CONFIG_SAVE_DELAY_MS;
  LOGD("Config save scheduled");
}

String htmlEscape(const String &input)
{
  String out;
  out.reserve(input.length() + 16);
  for (size_t i = 0; i < input.length(); i++)
  {
    char c = input[i];
    if (c == '&')
      out += "&amp;";
    else if (c == '<')
      out += "&lt;";
    else if (c == '>')
      out += "&gt;";
    else if (c == '"')
      out += "&quot;";
    else if (c == '\'')
      out += "&#39;";
    else
      out += c;
  }
  return out;
}

void sendHtmlPage(const String &message, int statusCode)
{
  server.send(statusCode, "text/html", buildHtmlPage(message));
}

String buildHtmlPage(const String &message)
{
  // Einfache, eingebettete Setup-Seite ohne externe Assets.
  // Vorteil: laeuft auch im AP/Captive-Portal-Modus robust.
  int networkCount = WiFi.scanNetworks();
  String networkOptions;
  if (networkCount > 0)
  {
    for (int i = 0; i < networkCount; i++)
    {
      String ssid = WiFi.SSID(i);
      if (ssid.length() == 0)
      {
        continue;
      }
      String optionText = htmlEscape(ssid) + " - ";
      optionText += (WiFi.encryptionType(i) == ENC_TYPE_NONE) ? "offen" : "gesichert";
      optionText += " | ";
      optionText += String(WiFi.RSSI(i));
      optionText += " dBm";
      networkOptions += "<option value='" + htmlEscape(ssid) + "'";
      if (ssid == config.wifiSsid)
      {
        networkOptions += " selected";
      }
      networkOptions += ">" + optionText + "</option>";
    }
  }
  WiFi.scanDelete();

  String stateText = (WiFi.status() == WL_CONNECTED)
                         ? (String("Verbunden mit ") + htmlEscape(config.wifiSsid) + " (" + WiFi.localIP().toString() + ")")
                         : "Kein STA-Link. Captive Portal im AP-Modus aktiv.";
  String lightHex = rgbToHex(config.lightR, config.lightG, config.lightB);

  String html;
  html.reserve(24000);
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>VacUBear Setup</title>";
  html += "<style>body{font-family:Arial,sans-serif;background:#f3f5f8;color:#17212b;margin:0;}";
  html += ".wrap{max-width:760px;margin:24px auto;padding:0 14px;}";
  html += ".card{background:#fff;border-radius:12px;padding:16px;box-shadow:0 6px 22px rgba(0,0,0,.08);}";
  html += "label{display:block;font-weight:600;margin-top:12px;}";
  html += "input,select{width:100%;padding:10px;border-radius:8px;border:1px solid #c8d1db;margin-top:6px;box-sizing:border-box;}";
  html += "input[type='checkbox']{width:auto;margin:0;}";
  html += "input[type='range']{padding:0;border:0;}";
  html += "input[type='color']{padding:4px;height:52px;}";
  html += "select[size]{min-height:150px;}";
  html += "button{margin-top:16px;padding:11px 16px;background:#005bbb;color:#fff;border:0;border-radius:8px;font-weight:700;cursor:pointer;}";
  html += ".btn-secondary{background:#4b5563;margin-right:10px;}";
  html += ".btn-danger{background:#b91c1c;}";
  html += ".btn-upload{background:#0f766e;}";
  html += ".status{background:#eef6ff;border-left:4px solid #005bbb;padding:10px;border-radius:8px;margin-bottom:14px;}";
  html += ".msg{background:#e8f8ed;border-left:4px solid #16853f;padding:10px;border-radius:8px;margin-bottom:14px;}";
  html += ".row{display:flex;gap:10px;flex-wrap:wrap;align-items:center;}";
  html += ".row button{margin-top:0;}";
  html += ".row .grow{flex:1 1 220px;}";
  html += ".toggle{display:flex;gap:10px;align-items:center;margin-top:12px;font-weight:600;}";
  html += ".swatch{height:56px;border-radius:10px;border:1px solid #c8d1db;margin-top:10px;background:#000;box-shadow:inset 0 0 0 1px rgba(255,255,255,.2);}";
  html += ".upload-box{border:1px dashed #9fb4c7;border-radius:10px;padding:12px;margin-top:14px;background:#f8fbff;}";
  html += ".progress{height:12px;background:#d7dee6;border-radius:999px;overflow:hidden;margin:10px 0 6px;}";
  html += ".progress-bar{height:100%;width:0;background:linear-gradient(90deg,#005bbb,#0f766e);transition:width .25s ease;}";
  html += "hr{border:0;border-top:1px solid #d7dee6;margin:18px 0;}";
  html += ".small{color:#5a6978;font-size:.9rem;margin-top:10px;}</style></head><body><div class='wrap'><div class='card'>";
  html += "<h2>VacUBear WiFi / MQTT Setup</h2>";
  html += "<div class='status'>" + stateText + "<br>AP SSID: <strong>" + htmlEscape(apSsid) + "</strong>";
  if (captivePortalEnabled)
  {
    html += " (" + WiFi.softAPIP().toString() + ")";
  }
  html += "</div>";

  if (message.length() > 0)
  {
    html += "<div class='msg'>" + htmlEscape(message) + "</div>";
  }

  html += "<form method='post' action='/save'>";
  html += "<label for='ssid'>WiFi SSID</label><input id='ssid' name='ssid' value='" + htmlEscape(config.wifiSsid) + "' required>";
  html += "<label for='wifi-network-list'>Verfuegbare WLAN-Netzwerke</label>";
  html += "<select id='wifi-network-list' size='6'>";
  if (networkOptions.length() > 0)
  {
    html += networkOptions;
  }
  else
  {
    html += "<option value=''>Keine Netzwerke gefunden</option>";
  }
  html += "</select>";
  html += "<div class='row'><button type='button' class='btn-secondary' onclick='applySelectedWifiSsid()'>Auswahl uebernehmen</button><button type='button' class='btn-upload' onclick='refreshWifiScan()'>Netzwerke aktualisieren</button></div>";
  html += "<div id='wifi-scan-status' class='small'></div>";
  html += "<label>WiFi Passwort</label><input name='wifi_password' type='password' value='" + htmlEscape(config.wifiPassword) + "'>";
  html += "<label>MQTT Host</label><input name='mqtt_host' value='" + htmlEscape(config.mqttHost) + "'>";
  html += "<label>MQTT Port</label><input name='mqtt_port' type='number' min='1' max='65535' value='" + String(config.mqttPort) + "'>";
  html += "<label>MQTT User</label><input name='mqtt_user' value='" + htmlEscape(config.mqttUser) + "'>";
  html += "<label>MQTT Passwort</label><input name='mqtt_password' type='password' value='" + htmlEscape(config.mqttPassword) + "'>";
  html += "<label>MQTT Basis-Topic</label><input name='mqtt_topic' value='" + htmlEscape(config.mqttTopic) + "'>";
  html += "<label>OTA Manifest URL</label><input name='ota_manifest_url' value='" + htmlEscape(config.otaManifestUrl) + "' placeholder='https://.../manifest.json'>";
  html += "<label>Laenge der Show (ms)</label><input name='show_length' type='number' min='" + String(SHOW_LENGTH_MIN_MS) + "' max='" + String(SHOW_LENGTH_MAX_MS) + "' value='" + String(config.showLengthMs) + "'>";
  html += "<label>Nachlaufzeit (ms)</label><input name='show_nachlauf' type='number' min='" + String(SHOW_NACHLAUF_MIN_MS) + "' value='" + String(config.showNachlaufMs) + "'>";
  if (HAS_LED_OUTPUT)
  {
    html += "<hr><h3>Beleuchtung</h3>";
    html += "<div class='status'>Die Beleuchtung bleibt ausserhalb der Show aus. Farbwerte lassen sich hier ohne Neustart speichern und fuer 5 Sekunden pruefweise anzeigen.</div>";
    html += "<label class='toggle'><input id='light_enabled' name='light_enabled' type='checkbox' value='1'";
    if (config.lightEnabled)
    {
      html += " checked";
    }
    html += "> Beleuchtung waehrend der Show aktivieren</label>";
    html += "<label for='light_color'>Lichtfarbe</label><input id='light_color' type='color' value='" + lightHex + "'>";
    html += "<div class='row'>";
    html += "<div class='grow'><label for='light_w_slider'>Weiss-Anteil</label><input id='light_w_slider' type='range' min='0' max='255' value='" + String(config.lightW) + "'></div>";
    html += "<div style='width:110px'><label for='light_w'>Weiss (0-255)</label><input id='light_w' name='light_w' type='number' min='0' max='255' value='" + String(config.lightW) + "'></div>";
    html += "</div>";
    html += "<input id='light_r' name='light_r' type='hidden' value='" + String(config.lightR) + "'>";
    html += "<input id='light_g' name='light_g' type='hidden' value='" + String(config.lightG) + "'>";
    html += "<input id='light_b' name='light_b' type='hidden' value='" + String(config.lightB) + "'>";
    html += "<div id='light-preview' class='swatch'></div>";
    html += "<div id='light-preview-text' class='small'>RGBW: " + String(config.lightR) + ", " + String(config.lightG) + ", " + String(config.lightB) + ", " + String(config.lightW) + "</div>";
    html += "<div class='row'><button type='button' class='btn-secondary' onclick='saveLightConfig(false)'>Beleuchtung speichern</button><button type='button' class='btn-upload' onclick='previewLightConfig()'>Vorschau 5s</button></div>";
    html += "<div class='small'>Der native Browser-Color-Picker deckt RGB ab. Der Weiss-Kanal bleibt deshalb separat einstellbar.</div>";
  }
  html += "<button type='submit'>Speichern & Neustart</button></form>";
  html += "<hr><h3>OTA Firmware</h3>";
  html += "<div id='ota-box' class='status'>Lade OTA-Status...</div>";
  html += "<div class='progress'><div id='ota-progress-bar' class='progress-bar'></div></div>";
  html += "<div id='ota-progress-text' class='small'>Kein Update aktiv.</div>";
  html += "<div class='row'>";
  html += "<button type='button' class='btn-secondary' onclick='otaCheck()'>Nach Update suchen</button>";
  html += "<button type='button' class='btn-danger' onclick='otaUpdate()'>Firmware installieren</button>";
  html += "</div>";
  html += "<div class='upload-box'>";
  html += "<label for='ota-file'>Firmware lokal hochladen (.bin)</label>";
  html += "<input id='ota-file' type='file' accept='.bin,application/octet-stream'>";
  html += "<button type='button' class='btn-upload' onclick='otaUploadFile()'>Datei hochladen & installieren</button>";
  html += "<div class='small'>Der Browser-Upload ergaenzt die Manifest-OTA und eignet sich fuer manuelle Tests oder lokale Release-Dateien.</div>";
  html += "</div>";
  html += "<div class='small'>Aktuelle Firmware: " + String(FW_VERSION) + "</div>";
  html += "<div class='small'>Bei gueltigem WLAN im lokalen Netz erreichbar. Bei Fehler startet AP + Captive Portal.</div>";
  html += "<script src='/app.js'></script>";
  html += "</div></div></body></html>";

  return html;
}
