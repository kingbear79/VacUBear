#include <Arduino.h>
#include <JC_Button.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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

#define PUMPE1 D5
#define PUMPE2 D6
#define VENTIL D7
#define TASTER D2

#define SHOW_LENGTH 50000UL
#define SHOW_NACHLAUF 50000UL

static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000UL;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 30000UL;
static const uint32_t MQTT_RECONNECT_INTERVAL_MS = 5000UL;
static const uint32_t CONFIG_SAVE_DELAY_MS = 2000UL;
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
static const uint32_t TELEMETRY_INTERVAL_MS = 10000UL;

static const char *CONFIG_FILE = "/config.json";
static const char *DEVICE_PREFIX = "vacubear";

struct AppConfig
{
  String wifiSsid;
  String wifiPassword;
  String mqttHost;
  uint16_t mqttPort;
  String mqttUser;
  String mqttPassword;
  String mqttTopic;
  uint32_t showLengthMs;
  uint32_t showNachlaufMs;
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
        showLengthMs(SHOW_LENGTH),
        showNachlaufMs(SHOW_NACHLAUF),
        lightR(255),
        lightG(255),
        lightB(255),
        lightW(0)
  {
  }
};

class ShowStatus
{
public:
  bool isRunning;
  unsigned long endAt;
  unsigned long openValveAt;
  bool shouldStart;
  unsigned long showDuration;
  unsigned long showNachlauf;

  ShowStatus()
      : isRunning(false),
        endAt(0),
        openValveAt(0),
        shouldStart(false),
        showDuration(SHOW_LENGTH),
        showNachlauf(SHOW_NACHLAUF)
  {
  }
};

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

Button button(TASTER, 25, true);
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
String topicLightRgbwSet;
String topicLightRgbwState;
String topicCfgShowLengthSet;
String topicCfgShowLengthState;
String topicCfgNachlaufSet;
String topicCfgNachlaufState;
String topicAvailability;
String topicTeleState;
PumpSoftControl pumpControl;

void startShow(void);
void stopShow(void);
void handleShow(void);
void setupPumpControl(void);
void setPumpTarget(bool enabled);
void updatePumpControl(void);
void applyPumpPwm(uint16_t pwm);
void setupWiFi(void);
void setupWebServer(void);
void setupMqtt(void);
void mqttLoop(void);
void mqttEnsureConnected(void);
void mqttCallback(char *topic, byte *payload, unsigned int length);
void updateMqttTopics(void);
void publishAvailability(const char *state, bool retained = true);
void publishShowState(bool retained = true);
void publishLightState(bool retained = true);
void publishConfigState(bool retained = true);
void publishDiscovery(void);
void publishSensorDiscovery(void);
void publishTelemetry(bool force = false);
const char *getShowPhase(void);

void loadConfig(void);
bool saveConfig(void);
void loadLegacyConfig(const String &raw);
bool connectToConfiguredWifi(uint32_t timeoutMs);
void startAccessPoint(void);
void setupCaptivePortal(void);
void handleRoot(void);
void handleSave(void);
void handleNotFound(void);
void handleStatus(void);
bool handleCaptivePortalRedirect(void);
String htmlEscape(const String &input);
String buildHtmlPage(const String &message = "");
bool isIpAddress(const String &host);
String defaultApSsid(void);
String defaultDeviceId(void);
uint32_t clampU32(uint32_t value, uint32_t minValue, uint32_t maxValue);
uint32_t clampMinU32(uint32_t value, uint32_t minValue);
uint32_t secToMs(uint32_t seconds);
uint32_t msToSec(uint32_t milliseconds);
uint8_t clampU8(int value);
void scheduleConfigSave(void);

void setup()
{
  Serial.begin(115200);
  LOGI("Booting VacUBear firmware");

  pinMode(PUMPE1, OUTPUT);
  pinMode(PUMPE2, OUTPUT);
  pinMode(VENTIL, OUTPUT);
  setupPumpControl();
  digitalWrite(VENTIL, LOW);

  button.begin();

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
  button.read();
  if (button.wasPressed())
  {
    LOGI("Button pressed");
    if (showStatus.isRunning)
    {
      stopShow();
    }
    else
    {
      startShow();
    }
  }

  handleShow();
  updatePumpControl();

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
      publishLightState(true);
      publishTelemetry(true);
    }
  }

  server.handleClient();

  if (captivePortalEnabled)
  {
    dnsServer.processNextRequest();
  }

  if (WiFi.status() != WL_CONNECTED && config.wifiSsid.length() > 0)
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

  if (configSavePending && millis() >= configSaveAt)
  {
    saveConfig();
    configSavePending = false;
  }
}

void startShow()
{
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
  if (!showStatus.isRunning)
  {
    LOGD("stopShow ignored: not running");
    return;
  }
  LOGI("stopShow requested");
  showStatus.endAt = millis() + 10;
  showStatus.openValveAt = millis() + 10;
}

void handleShow()
{
  if (showStatus.shouldStart)
  {
    showStatus.endAt = millis() + showStatus.showDuration;
    showStatus.openValveAt = showStatus.endAt + showStatus.showNachlauf;
    showStatus.shouldStart = false;
    showStatus.isRunning = true;
    LOGI("Show started: Vakuumieren=%lu ms, Haltezeit=%lu ms", showStatus.showDuration, showStatus.showNachlauf);
  }

  if (showStatus.isRunning)
  {
    if (millis() < showStatus.endAt)
    {
      setPumpTarget(true);
      digitalWrite(VENTIL, HIGH);
    }
    else if (millis() < showStatus.openValveAt)
    {
      setPumpTarget(false);
      digitalWrite(VENTIL, HIGH);
    }
    else
    {
      setPumpTarget(false);
      digitalWrite(VENTIL, LOW);
      showStatus.isRunning = false;
      LOGI("Show finished");
    }
  }
  else
  {
    setPumpTarget(false);
    digitalWrite(VENTIL, LOW);
    showStatus.isRunning = false;
  }
}

void setupPumpControl()
{
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
  analogWrite(PUMPE1, pwm);
  analogWrite(PUMPE2, pwm);
}

void setupMqtt()
{
  mqttClient.setServer(config.mqttHost.c_str(), config.mqttPort);
  mqttClient.setBufferSize(1024);
  mqttClient.setCallback(mqttCallback);
  updateMqttTopics();
  LOGI("MQTT setup: broker=%s:%u", config.mqttHost.c_str(), config.mqttPort);
}

void mqttLoop()
{
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

void mqttEnsureConnected()
{
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

  mqttClient.subscribe(topicShowSet.c_str());
  mqttClient.subscribe(topicLightSet.c_str());
  mqttClient.subscribe(topicLightRgbwSet.c_str());
  mqttClient.subscribe(topicCfgShowLengthSet.c_str());
  mqttClient.subscribe(topicCfgNachlaufSet.c_str());
  publishAvailability("online", true);
  publishDiscovery();
  publishShowState(true);
  publishLightState(true);
  publishConfigState(true);
  publishTelemetry(true);
}

void mqttCallback(char *topic, byte *payload, unsigned int length)
{
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

  if (topicStr == topicLightSet)
  {
    bool changed = false;

    if (payloadStr == "ON" || payloadStr == "OFF")
    {
      publishLightState(true);
      publishTelemetry(true);
      return;
    }

    StaticJsonDocument<384> doc;
    DeserializationError err = deserializeJson(doc, payloadStr);
    if (!err)
    {
      if (doc["color"].is<JsonObject>())
      {
        JsonObject color = doc["color"].as<JsonObject>();
        if (color.containsKey("r"))
        {
          config.lightR = clampU8(color["r"].as<int>());
          changed = true;
        }
        if (color.containsKey("g"))
        {
          config.lightG = clampU8(color["g"].as<int>());
          changed = true;
        }
        if (color.containsKey("b"))
        {
          config.lightB = clampU8(color["b"].as<int>());
          changed = true;
        }
        if (color.containsKey("w"))
        {
          config.lightW = clampU8(color["w"].as<int>());
          changed = true;
        }
      }

      if (doc.containsKey("white_value"))
      {
        config.lightW = clampU8(doc["white_value"].as<int>());
        changed = true;
      }
      if (doc.containsKey("white"))
      {
        config.lightW = clampU8(doc["white"].as<int>());
        changed = true;
      }
    }

    if (changed)
    {
      LOGI("Lichtfarbe updated via JSON command");
      scheduleConfigSave();
    }
    publishLightState(true);
    publishTelemetry(true);
    return;
  }

  if (topicStr == topicLightRgbwSet)
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
      LOGI("Lichtfarbe updated via RGBW topic: %u,%u,%u,%u", config.lightR, config.lightG, config.lightB, config.lightW);
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
  }
}

void publishAvailability(const char *state, bool retained)
{
  mqttClient.publish(topicAvailability.c_str(), state, retained);
  LOGD("MQTT TX availability=%s", state);
}

void publishShowState(bool retained)
{
  mqttClient.publish(topicShowState.c_str(), showStatus.isRunning ? "ON" : "OFF", retained);
  LOGD("MQTT TX show_state=%s", showStatus.isRunning ? "ON" : "OFF");
}

void publishLightState(bool retained)
{
  mqttClient.publish(topicLightState.c_str(), "ON", retained);
  String rgbw = String(config.lightR) + "," + String(config.lightG) + "," + String(config.lightB) + "," + String(config.lightW);
  mqttClient.publish(topicLightRgbwState.c_str(), rgbw.c_str(), retained);
  LOGD("MQTT TX light_state=ON rgbw=%s", rgbw.c_str());
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
  LOGI("Publishing Home Assistant discovery");
  StaticJsonDocument<512> switchCfg;
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

  StaticJsonDocument<512> lightCfg;
  lightCfg["name"] = "Lichtfarbe";
  lightCfg["object_id"] = "light";
  lightCfg["unique_id"] = deviceId + "-light";
  lightCfg["command_topic"] = topicLightSet;
  lightCfg["state_topic"] = topicLightState;
  lightCfg["payload_on"] = "ON";
  lightCfg["payload_off"] = "OFF";
  lightCfg["rgbw_command_topic"] = topicLightRgbwSet;
  lightCfg["rgbw_state_topic"] = topicLightRgbwState;
  JsonArray colorModes = lightCfg["supported_color_modes"].to<JsonArray>();
  colorModes.add("rgbw");
  lightCfg["availability_topic"] = topicAvailability;
  lightCfg["payload_available"] = "online";
  lightCfg["payload_not_available"] = "offline";
  JsonObject lightDevice = lightCfg["device"].to<JsonObject>();
  JsonArray lightIds = lightDevice["identifiers"].to<JsonArray>();
  lightIds.add(deviceId);
  lightDevice["name"] = "VacUBear";
  lightDevice["manufacturer"] = "KingBEAR";
  lightDevice["model"] = "Frame-25";

  String lightPayload;
  serializeJson(lightCfg, lightPayload);
  String lightDiscoveryTopic = "homeassistant/light/" + deviceId + "/light/config";
  mqttClient.publish(lightDiscoveryTopic.c_str(), lightPayload.c_str(), true);

  publishSensorDiscovery();
}

void publishSensorDiscovery()
{
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
    StaticJsonDocument<384> cfg;
    cfg["name"] = "Show aktiv";
    cfg["object_id"] = "show_aktiv";
    cfg["unique_id"] = deviceId + "-show-aktiv";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ 'ON' if value_json.Show.Aktiv else 'OFF' }}";
    cfg["payload_on"] = "ON";
    cfg["payload_off"] = "OFF";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/binary_sensor/") + deviceId + "/show_aktiv/config").c_str(), payload.c_str(), true);
  }

  {
    StaticJsonDocument<384> cfg;
    cfg["name"] = "Show Phase";
    cfg["object_id"] = "show_phase";
    cfg["unique_id"] = deviceId + "-show-phase";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.Show.Phase }}";
    cfg["icon"] = "mdi:state-machine";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/show_phase/config").c_str(), payload.c_str(), true);
  }

  {
    StaticJsonDocument<384> cfg;
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
    StaticJsonDocument<384> cfg;
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
    StaticJsonDocument<384> cfg;
    cfg["name"] = "Pumpen PWM";
    cfg["object_id"] = "pumpen_pwm";
    cfg["unique_id"] = deviceId + "-pumpen-pwm";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.Pumpen.PWM }}";
    cfg["unit_of_measurement"] = "raw";
    cfg["icon"] = "mdi:fan";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/pumpen_pwm/config").c_str(), payload.c_str(), true);
  }

  {
    StaticJsonDocument<384> cfg;
    cfg["name"] = "Show Länge";
    cfg["object_id"] = "show_laenge";
    cfg["unique_id"] = deviceId + "-show-laenge";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.Show.LaengeS }}";
    cfg["unit_of_measurement"] = "s";
    cfg["icon"] = "mdi:timer-sand";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/show_laenge/config").c_str(), payload.c_str(), true);
  }

  {
    StaticJsonDocument<384> cfg;
    cfg["name"] = "Nachlaufzeit";
    cfg["object_id"] = "nachlaufzeit";
    cfg["unique_id"] = deviceId + "-nachlaufzeit";
    cfg["state_topic"] = topicTeleState;
    cfg["value_template"] = "{{ value_json.Show.NachlaufS }}";
    cfg["unit_of_measurement"] = "s";
    cfg["icon"] = "mdi:timer-outline";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/sensor/") + deviceId + "/nachlaufzeit/config").c_str(), payload.c_str(), true);
  }

  {
    StaticJsonDocument<384> cfg;
    cfg["name"] = "Vakuumier-Zeit";
    cfg["object_id"] = "show_laenge_setzen";
    cfg["unique_id"] = deviceId + "-show-laenge-setzen";
    cfg["command_topic"] = topicCfgShowLengthSet;
    cfg["state_topic"] = topicCfgShowLengthState;
    cfg["unit_of_measurement"] = "s";
    cfg["min"] = SHOW_LENGTH_MIN_S;
    cfg["max"] = SHOW_LENGTH_MAX_S;
    cfg["step"] = 1;
    cfg["mode"] = "slider";
    cfg["availability_topic"] = topicAvailability;
    addDevice(cfg.as<JsonObject>());
    String payload;
    serializeJson(cfg, payload);
    mqttClient.publish((String("homeassistant/number/") + deviceId + "/show_laenge_setzen/config").c_str(), payload.c_str(), true);
  }

  {
    StaticJsonDocument<384> cfg;
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

  LOGI("Published Home Assistant sensor discovery");
}

const char *getShowPhase()
{
  unsigned long now = millis();
  if (showStatus.isRunning)
  {
    if (now < showStatus.endAt)
    {
      return "Vakuumieren";
    }
    if (now < showStatus.openValveAt)
    {
      return "Haltezeit";
    }
    return "Belueften";
  }

  if (showStatus.openValveAt > 0 && (now - showStatus.openValveAt) < 2000UL)
  {
    return "Belueften";
  }
  return "Pause";
}

void publishTelemetry(bool force)
{
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

  StaticJsonDocument<768> doc;
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
  show["EndAt"] = showStatus.endAt;
  show["OpenValveAt"] = showStatus.openValveAt;

  JsonObject pumpen = doc["Pumpen"].to<JsonObject>();
  pumpen["PWM"] = pumpControl.currentPwm;
  pumpen["TargetPWM"] = pumpControl.targetPwm;

  JsonObject licht = doc["Lichtfarbe"].to<JsonObject>();
  licht["R"] = config.lightR;
  licht["G"] = config.lightG;
  licht["B"] = config.lightB;
  licht["W"] = config.lightW;

  String payload;
  serializeJson(doc, payload);
  mqttClient.publish(topicTeleState.c_str(), payload.c_str(), false);
  LOGD("MQTT TX telemetry: %s", payload.c_str());
}

void updateMqttTopics()
{
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
  topicLightRgbwSet = topicBase + "/light/rgbw/set";
  topicLightRgbwState = topicBase + "/light/rgbw/state";
  topicCfgShowLengthSet = topicBase + "/config/show_length_s/set";
  topicCfgShowLengthState = topicBase + "/config/show_length_s/state";
  topicCfgNachlaufSet = topicBase + "/config/nachlauf_s/set";
  topicCfgNachlaufState = topicBase + "/config/nachlauf_s/state";
  topicAvailability = topicBase + "/availability";
  topicTeleState = "tele/" + deviceId + "/STATE";
  LOGI("MQTT topics initialized: base=%s", topicBase.c_str());
}

void setupWiFi()
{
  LOGI("Initializing WiFi");
  apSsid = defaultApSsid();

  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.hostname(deviceId.c_str());

  bool connected = connectToConfiguredWifi(WIFI_CONNECT_TIMEOUT_MS);
  if (!connected)
  {
    startAccessPoint();
    setupCaptivePortal();
  }
  else
  {
    LOGI("STA connected. IP=%s", WiFi.localIP().toString().c_str());
  }
}

bool connectToConfiguredWifi(uint32_t timeoutMs)
{
  if (config.wifiSsid.length() == 0)
  {
    LOGW("No WiFi SSID configured yet");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());

  LOGI("Connecting to WiFi SSID '%s'...", config.wifiSsid.c_str());

  unsigned long startAt = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startAt) < timeoutMs)
  {
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

void startAccessPoint()
{
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
  dnsServer.start(53, "*", WiFi.softAPIP());
  captivePortalEnabled = true;
  LOGI("Captive portal enabled");
}

void setupWebServer()
{
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/status", HTTP_GET, handleStatus);
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
  server.send(200, "text/html", buildHtmlPage());
}

void handleSave()
{
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
  if (server.hasArg("show_length"))
  {
    config.showLengthMs = clampU32((uint32_t)server.arg("show_length").toInt(), SHOW_LENGTH_MIN_MS, SHOW_LENGTH_MAX_MS);
  }
  if (server.hasArg("show_nachlauf"))
  {
    config.showNachlaufMs = clampMinU32((uint32_t)server.arg("show_nachlauf").toInt(), SHOW_NACHLAUF_MIN_MS);
  }
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

  config.wifiSsid.trim();
  config.wifiPassword.trim();
  config.mqttHost.trim();
  config.mqttUser.trim();
  config.mqttPassword.trim();
  config.mqttTopic.trim();

  if (config.mqttPort == 0)
  {
    config.mqttPort = 1883;
  }
  if (config.mqttTopic.length() == 0 || config.mqttTopic == "vacubear" || config.mqttTopic.startsWith("vacubear/"))
  {
    config.mqttTopic = deviceId;
  }

  showStatus.showDuration = config.showLengthMs;
  showStatus.showNachlauf = config.showNachlaufMs;
  publishConfigState(true);

  if (!saveConfig())
  {
    LOGW("Config save failed");
    server.send(500, "text/html", buildHtmlPage("Fehler beim Speichern."));
    return;
  }

  LOGI("Config saved, restarting device");
  server.send(200, "text/html", buildHtmlPage("Gespeichert. Das Geraet startet neu..."));
  delay(500);
  ESP.restart();
}

void handleStatus()
{
  LOGD("HTTP GET /status");
  StaticJsonDocument<512> status;
  status["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  status["sta_ip"] = WiFi.localIP().toString();
  status["ap_enabled"] = captivePortalEnabled;
  status["ap_ip"] = WiFi.softAPIP().toString();
  status["show_running"] = showStatus.isRunning;
  status["show_length"] = config.showLengthMs;
  status["show_nachlauf"] = config.showNachlaufMs;
  status["pump_pwm"] = pumpControl.currentPwm;
  status["pump_target_pwm"] = pumpControl.targetPwm;

  JsonObject color = status["light"].to<JsonObject>();
  color["r"] = config.lightR;
  color["g"] = config.lightG;
  color["b"] = config.lightB;
  color["w"] = config.lightW;

  String out;
  serializeJson(status, out);
  server.send(200, "application/json", out);
}

void handleNotFound()
{
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
    StaticJsonDocument<768> doc;
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
      config.showLengthMs = doc["show"]["length_ms"] | config.showLengthMs;
      config.showNachlaufMs = doc["show"]["nachlauf_ms"] | config.showNachlaufMs;
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

  config.wifiSsid.trim();
  config.wifiPassword.trim();
  config.mqttHost.trim();
  config.mqttUser.trim();
  config.mqttPassword.trim();
  config.mqttTopic.trim();

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

  showStatus.showDuration = config.showLengthMs;
  showStatus.showNachlauf = config.showNachlaufMs;
}

void loadLegacyConfig(const String &raw)
{
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
    config.lightR = clampU8(lines[9].toInt());
  if (count > 10)
    config.lightG = clampU8(lines[10].toInt());
  if (count > 11)
    config.lightB = clampU8(lines[11].toInt());
  if (count > 12)
    config.lightW = clampU8(lines[12].toInt());
}

bool saveConfig()
{
  StaticJsonDocument<768> doc;

  JsonObject wifi = doc["wifi"].to<JsonObject>();
  wifi["ssid"] = config.wifiSsid;
  wifi["password"] = config.wifiPassword;

  JsonObject mqtt = doc["mqtt"].to<JsonObject>();
  mqtt["host"] = config.mqttHost;
  mqtt["port"] = config.mqttPort;
  mqtt["user"] = config.mqttUser;
  mqtt["password"] = config.mqttPassword;
  mqtt["topic"] = config.mqttTopic;

  JsonObject show = doc["show"].to<JsonObject>();
  show["length_ms"] = config.showLengthMs;
  show["nachlauf_ms"] = config.showNachlaufMs;

  JsonObject light = doc["light"].to<JsonObject>();
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

String defaultApSsid()
{
  char buf[24];
  snprintf(buf, sizeof(buf), "VacUBear-%06X", ESP.getChipId());
  return String(buf);
}

String defaultDeviceId()
{
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

String buildHtmlPage(const String &message)
{
  int networkCount = WiFi.scanNetworks();
  String options;
  if (networkCount > 0)
  {
    for (int i = 0; i < networkCount; i++)
    {
      String ssid = WiFi.SSID(i);
      if (ssid.length() == 0)
      {
        continue;
      }
      options += "<option value='" + htmlEscape(ssid) + "'>";
    }
  }
  WiFi.scanDelete();

  String stateText = (WiFi.status() == WL_CONNECTED)
                         ? (String("Verbunden mit ") + htmlEscape(config.wifiSsid) + " (" + WiFi.localIP().toString() + ")")
                         : "Kein STA-Link. Captive Portal im AP-Modus aktiv.";

  String html;
  html.reserve(7000);
  html += "<!doctype html><html><head><meta charset='utf-8'>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<title>VacUBear Setup</title>";
  html += "<style>body{font-family:Arial,sans-serif;background:#f3f5f8;color:#17212b;margin:0;}";
  html += ".wrap{max-width:760px;margin:24px auto;padding:0 14px;}";
  html += ".card{background:#fff;border-radius:12px;padding:16px;box-shadow:0 6px 22px rgba(0,0,0,.08);}";
  html += "label{display:block;font-weight:600;margin-top:12px;}";
  html += "input{width:100%;padding:10px;border-radius:8px;border:1px solid #c8d1db;margin-top:6px;box-sizing:border-box;}";
  html += "button{margin-top:16px;padding:11px 16px;background:#005bbb;color:#fff;border:0;border-radius:8px;font-weight:700;cursor:pointer;}";
  html += ".status{background:#eef6ff;border-left:4px solid #005bbb;padding:10px;border-radius:8px;margin-bottom:14px;}";
  html += ".msg{background:#e8f8ed;border-left:4px solid #16853f;padding:10px;border-radius:8px;margin-bottom:14px;}";
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
  html += "<label>WiFi SSID</label><input name='ssid' list='ssid-list' value='" + htmlEscape(config.wifiSsid) + "' required>";
  html += "<datalist id='ssid-list'>" + options + "</datalist>";
  html += "<label>WiFi Passwort</label><input name='wifi_password' type='password' value='" + htmlEscape(config.wifiPassword) + "'>";
  html += "<label>MQTT Host</label><input name='mqtt_host' value='" + htmlEscape(config.mqttHost) + "'>";
  html += "<label>MQTT Port</label><input name='mqtt_port' type='number' min='1' max='65535' value='" + String(config.mqttPort) + "'>";
  html += "<label>MQTT User</label><input name='mqtt_user' value='" + htmlEscape(config.mqttUser) + "'>";
  html += "<label>MQTT Passwort</label><input name='mqtt_password' type='password' value='" + htmlEscape(config.mqttPassword) + "'>";
  html += "<label>MQTT Basis-Topic</label><input name='mqtt_topic' value='" + htmlEscape(config.mqttTopic) + "'>";
  html += "<label>Laenge der Show (ms)</label><input name='show_length' type='number' min='" + String(SHOW_LENGTH_MIN_MS) + "' max='" + String(SHOW_LENGTH_MAX_MS) + "' value='" + String(config.showLengthMs) + "'>";
  html += "<label>Nachlaufzeit (ms)</label><input name='show_nachlauf' type='number' min='" + String(SHOW_NACHLAUF_MIN_MS) + "' value='" + String(config.showNachlaufMs) + "'>";
  html += "<label>Lichtfarbe R (0-255)</label><input name='light_r' type='number' min='0' max='255' value='" + String(config.lightR) + "'>";
  html += "<label>Lichtfarbe G (0-255)</label><input name='light_g' type='number' min='0' max='255' value='" + String(config.lightG) + "'>";
  html += "<label>Lichtfarbe B (0-255)</label><input name='light_b' type='number' min='0' max='255' value='" + String(config.lightB) + "'>";
  html += "<label>Lichtfarbe W (0-255)</label><input name='light_w' type='number' min='0' max='255' value='" + String(config.lightW) + "'>";
  html += "<button type='submit'>Speichern & Neustart</button></form>";
  html += "<div class='small'>Bei gueltigem WLAN im lokalen Netz erreichbar. Bei Fehler startet AP + Captive Portal.</div>";
  html += "</div></div></body></html>";

  return html;
}
