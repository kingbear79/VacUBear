#include <Arduino.h>
#include <JC_Button.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

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
static const uint32_t SHOW_LENGTH_MIN_MS = 1000UL;
static const uint32_t SHOW_LENGTH_MAX_MS = 3600000UL;
static const uint32_t SHOW_NACHLAUF_MIN_MS = 0UL;
static const uint32_t SHOW_NACHLAUF_MAX_MS = 3600000UL;

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
bool lastShowRunningState = false;
bool configSavePending = false;
unsigned long configSaveAt = 0;

String topicBase;
String topicShowSet;
String topicShowState;
String topicLightSet;
String topicLightState;
String topicAvailability;

void startShow(void);
void stopShow(void);
void handleShow(void);
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
void publishDiscovery(void);

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
uint8_t clampU8(int value);
void scheduleConfigSave(void);

void setup()
{
  Serial.begin(115200);

  pinMode(PUMPE1, OUTPUT);
  pinMode(PUMPE2, OUTPUT);
  pinMode(VENTIL, OUTPUT);
  digitalWrite(PUMPE1, LOW);
  digitalWrite(PUMPE2, LOW);
  digitalWrite(VENTIL, LOW);

  button.begin();

  if (!LittleFS.begin())
  {
    Serial.println("LittleFS mount failed");
  }

  deviceId = defaultDeviceId();
  loadConfig();
  setupWiFi();
  setupWebServer();
  setupMqtt();

  lastShowRunningState = showStatus.isRunning;
  Serial.println("Setup complete");
}

void loop()
{
  button.read();
  if (button.wasPressed())
  {
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

  if (showStatus.isRunning != lastShowRunningState)
  {
    lastShowRunningState = showStatus.isRunning;
    if (mqttClient.connected())
    {
      publishShowState(true);
      publishLightState(true);
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
      Serial.println("Retrying WiFi STA connection...");
    }
  }

  mqttLoop();

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
    return;
  }
  showStatus.shouldStart = true;
}

void stopShow()
{
  if (!showStatus.isRunning)
  {
    return;
  }
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
  }

  if (showStatus.isRunning)
  {
    if (millis() < showStatus.endAt)
    {
      digitalWrite(PUMPE1, HIGH);
      digitalWrite(PUMPE2, HIGH);
      digitalWrite(VENTIL, HIGH);
    }
    else if (millis() < showStatus.openValveAt)
    {
      digitalWrite(PUMPE1, LOW);
      digitalWrite(PUMPE2, LOW);
      digitalWrite(VENTIL, HIGH);
    }
    else
    {
      digitalWrite(PUMPE1, LOW);
      digitalWrite(PUMPE2, LOW);
      digitalWrite(VENTIL, LOW);
      showStatus.isRunning = false;
    }
  }
  else
  {
    digitalWrite(PUMPE1, LOW);
    digitalWrite(PUMPE2, LOW);
    digitalWrite(VENTIL, LOW);
    showStatus.isRunning = false;
  }
}

void setupMqtt()
{
  mqttClient.setServer(config.mqttHost.c_str(), config.mqttPort);
  mqttClient.setBufferSize(1024);
  mqttClient.setCallback(mqttCallback);
  updateMqttTopics();
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
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqttClient.state());
    return;
  }

  Serial.println("MQTT connected");

  mqttClient.subscribe(topicShowSet.c_str());
  mqttClient.subscribe(topicLightSet.c_str());
  publishAvailability("online", true);
  publishDiscovery();
  publishShowState(true);
  publishLightState(true);
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
    return;
  }

  if (topicStr == topicLightSet)
  {
    bool changed = false;

    if (payloadStr == "ON" || payloadStr == "OFF")
    {
      publishLightState(true);
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
      scheduleConfigSave();
    }
    publishLightState(true);
  }
}

void publishAvailability(const char *state, bool retained)
{
  mqttClient.publish(topicAvailability.c_str(), state, retained);
}

void publishShowState(bool retained)
{
  mqttClient.publish(topicShowState.c_str(), showStatus.isRunning ? "ON" : "OFF", retained);
}

void publishLightState(bool retained)
{
  StaticJsonDocument<192> doc;
  doc["state"] = "ON";
  JsonObject color = doc["color"].to<JsonObject>();
  color["r"] = config.lightR;
  color["g"] = config.lightG;
  color["b"] = config.lightB;
  color["w"] = config.lightW;
  doc["white_value"] = config.lightW;

  String payload;
  serializeJson(doc, payload);
  mqttClient.publish(topicLightState.c_str(), payload.c_str(), retained);
}

void publishDiscovery()
{
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
  lightCfg["schema"] = "json";
  lightCfg["command_topic"] = topicLightSet;
  lightCfg["state_topic"] = topicLightState;
  lightCfg["rgb"] = true;
  lightCfg["white_value"] = true;
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
  topicAvailability = topicBase + "/availability";
}

void setupWiFi()
{
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
    Serial.print("STA connected. IP: ");
    Serial.println(WiFi.localIP());
  }
}

bool connectToConfiguredWifi(uint32_t timeoutMs)
{
  if (config.wifiSsid.length() == 0)
  {
    Serial.println("No WiFi SSID configured yet.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());

  Serial.print("Connecting to WiFi '");
  Serial.print(config.wifiSsid);
  Serial.println("'...");

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

  Serial.print("AP started. SSID: ");
  Serial.println(apSsid);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

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
  Serial.println("Captive portal enabled");
}

void setupWebServer()
{
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/status", HTTP_GET, handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();

  Serial.println("Web server started on port 80");
}

void handleRoot()
{
  if (handleCaptivePortalRedirect())
  {
    return;
  }
  server.send(200, "text/html", buildHtmlPage());
}

void handleSave()
{
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
    config.showNachlaufMs = clampU32((uint32_t)server.arg("show_nachlauf").toInt(), SHOW_NACHLAUF_MIN_MS, SHOW_NACHLAUF_MAX_MS);
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

  if (!saveConfig())
  {
    server.send(500, "text/html", buildHtmlPage("Fehler beim Speichern."));
    return;
  }

  server.send(200, "text/html", buildHtmlPage("Gespeichert. Das Geraet startet neu..."));
  delay(500);
  ESP.restart();
}

void handleStatus()
{
  StaticJsonDocument<256> status;
  status["wifi_connected"] = WiFi.status() == WL_CONNECTED;
  status["sta_ip"] = WiFi.localIP().toString();
  status["ap_enabled"] = captivePortalEnabled;
  status["ap_ip"] = WiFi.softAPIP().toString();
  status["show_running"] = showStatus.isRunning;
  status["show_length"] = config.showLengthMs;
  status["show_nachlauf"] = config.showNachlaufMs;

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
  return true;
}

void loadConfig()
{
  config = AppConfig();

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
    Serial.println("Could not open config file");
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
      Serial.println("Config JSON parse failed, loading defaults.");
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
  config.showNachlaufMs = clampU32(config.showNachlaufMs, SHOW_NACHLAUF_MIN_MS, SHOW_NACHLAUF_MAX_MS);

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
    Serial.println("Could not write config file");
    return false;
  }

  serializeJson(doc, f);
  f.close();
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
  html += "<label>Laenge der Show (ms)</label><input name='show_length' type='number' min='1000' max='3600000' value='" + String(config.showLengthMs) + "'>";
  html += "<label>Nachlaufzeit (ms)</label><input name='show_nachlauf' type='number' min='0' max='3600000' value='" + String(config.showNachlaufMs) + "'>";
  html += "<label>Lichtfarbe R (0-255)</label><input name='light_r' type='number' min='0' max='255' value='" + String(config.lightR) + "'>";
  html += "<label>Lichtfarbe G (0-255)</label><input name='light_g' type='number' min='0' max='255' value='" + String(config.lightG) + "'>";
  html += "<label>Lichtfarbe B (0-255)</label><input name='light_b' type='number' min='0' max='255' value='" + String(config.lightB) + "'>";
  html += "<label>Lichtfarbe W (0-255)</label><input name='light_w' type='number' min='0' max='255' value='" + String(config.lightW) + "'>";
  html += "<button type='submit'>Speichern & Neustart</button></form>";
  html += "<div class='small'>Bei gueltigem WLAN im lokalen Netz erreichbar. Bei Fehler startet AP + Captive Portal.</div>";
  html += "</div></div></body></html>";

  return html;
}
