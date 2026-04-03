#include <Arduino.h>
#include <LittleFS.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecureBearSSL.h>
#include <ArduinoJson.h>

#include "pins.h"

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

#ifndef FW_VERSION
#define FW_VERSION "0.0.0-legacy-migrator"
#endif

#ifndef LEGACY_MIGRATOR_TARGET_MANIFEST_URL
#define LEGACY_MIGRATOR_TARGET_MANIFEST_URL "http://ota.kinkbear.de/vacubear/manifest.json"
#endif

static const char *CONFIG_FILE = "/config.json";
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000UL;
static const uint32_t WIFI_RETRY_INTERVAL_MS = 30000UL;
static const uint32_t OTA_HTTP_TIMEOUT_MS = 30000UL;
static const uint32_t UPDATE_RETRY_INTERVAL_MS = 60000UL;

struct AppConfig
{
  String wifiSsid;
  String wifiPassword;
};

struct ParsedHttpUrl
{
  bool valid;
  bool https;
  String host;
  uint16_t port;
  String path;

  ParsedHttpUrl()
      : valid(false), https(false), host(""), port(0), path("/")
  {
  }
};

AppConfig config;
bool wifiConnectStarted = false;
unsigned long wifiConnectStartedAt = 0;
unsigned long lastWifiRetryAt = 0;
unsigned long lastUpdateAttemptAt = 0;
bool migrationComplete = false;

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

bool configureSecureClientForUrl(BearSSL::WiFiClientSecure &secureClient, const String &url)
{
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
      return true;
    }
  }

  return true;
}

void setSafeOutputs()
{
  pinMode(PIN_PUMPE1, OUTPUT);
  pinMode(PIN_PUMPE2, OUTPUT);
  pinMode(PIN_VENTIL, OUTPUT);
  analogWrite(PIN_PUMPE1, 0);
  analogWrite(PIN_PUMPE2, 0);
  digitalWrite(PIN_VENTIL, LOW);
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
}

void loadConfig()
{
  config = AppConfig();

  if (!LittleFS.exists(CONFIG_FILE))
  {
    LOGW("Config file missing: %s", CONFIG_FILE);
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
  raw.trim();
  if (raw.length() == 0)
  {
    LOGW("Config file is empty");
    return;
  }

  if (raw[0] == '{')
  {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, raw);
    if (err)
    {
      LOGW("Config JSON parse failed: %s", err.c_str());
      return;
    }
    config.wifiSsid = doc["wifi"]["ssid"] | "";
    config.wifiPassword = doc["wifi"]["password"] | "";
  }
  else
  {
    loadLegacyConfig(raw);
  }

  config.wifiSsid.trim();
  config.wifiPassword.trim();
  LOGI("Loaded WiFi config: ssid=%s", config.wifiSsid.c_str());
}

bool connectToConfiguredWifi()
{
  if (config.wifiSsid.length() == 0)
  {
    return false;
  }

  if (!wifiConnectStarted)
  {
    WiFi.mode(WIFI_STA);
    WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
    wifiConnectStarted = true;
    wifiConnectStartedAt = millis();
    lastWifiRetryAt = wifiConnectStartedAt;
    LOGI("Connecting WiFi: %s", config.wifiSsid.c_str());
    return false;
  }

  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED)
  {
    LOGI("WiFi connected: %s", WiFi.localIP().toString().c_str());
    return true;
  }

  if ((millis() - wifiConnectStartedAt) > WIFI_CONNECT_TIMEOUT_MS &&
      (millis() - lastWifiRetryAt) > WIFI_RETRY_INTERVAL_MS)
  {
    lastWifiRetryAt = millis();
    WiFi.disconnect();
    delay(50);
    WiFi.begin(config.wifiSsid.c_str(), config.wifiPassword.c_str());
    LOGW("Retrying WiFi connection");
  }

  return false;
}

bool fetchManifest(String &firmwareUrl, String &version, String &notes)
{
  firmwareUrl = "";
  version = "";
  notes = "";

  String requestUrl = String(LEGACY_MIGRATOR_TARGET_MANIFEST_URL);
  requestUrl += requestUrl.indexOf('?') >= 0 ? "&nocache=" : "?nocache=";
  requestUrl += String(millis());

  ParsedHttpUrl requestTarget;
  if (!parseHttpUrl(requestUrl, requestTarget))
  {
    LOGW("Invalid manifest URL: %s", requestUrl.c_str());
    return false;
  }

  BearSSL::WiFiClientSecure secureClient;
  WiFiClient plainClient;
  if (requestTarget.https)
  {
    configureSecureClientForUrl(secureClient, requestUrl);
  }

  HTTPClient http;
  http.setTimeout(OTA_HTTP_TIMEOUT_MS);
  http.setReuse(false);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);

  bool beginOk = requestTarget.https ? http.begin(secureClient, requestUrl) : http.begin(plainClient, requestUrl);
  if (!beginOk)
  {
    LOGW("Manifest begin() failed");
    return false;
  }

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK)
  {
    LOGW("Manifest HTTP error: %d", httpCode);
    http.end();
    return false;
  }

  String payload = http.getString();
  http.end();

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, payload);
  if (err)
  {
    LOGW("Manifest JSON invalid: %s", err.c_str());
    return false;
  }

  version = doc["version"] | "";
  firmwareUrl = doc["firmware_url"] | "";
  if (firmwareUrl.length() == 0)
  {
    firmwareUrl = doc["url"] | "";
  }
  notes = doc["notes"] | "";
  firmwareUrl.trim();
  version.trim();
  notes.trim();
  return firmwareUrl.length() > 0;
}

bool applyTargetFirmware(const String &firmwareUrl)
{
  ParsedHttpUrl target;
  if (!parseHttpUrl(firmwareUrl, target))
  {
    LOGW("Invalid firmware URL: %s", firmwareUrl.c_str());
    return false;
  }

  setSafeOutputs();

  ESPhttpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  ESPhttpUpdate.setClientTimeout(OTA_HTTP_TIMEOUT_MS);
  ESPhttpUpdate.rebootOnUpdate(true);

  t_httpUpdate_return result;
  if (target.https)
  {
    BearSSL::WiFiClientSecure secureClient;
    configureSecureClientForUrl(secureClient, firmwareUrl);
    result = ESPhttpUpdate.update(secureClient, firmwareUrl);
  }
  else
  {
    WiFiClient plainClient;
    result = ESPhttpUpdate.update(plainClient, firmwareUrl);
  }

  if (result == HTTP_UPDATE_OK)
  {
    return true;
  }

  LOGW("Target firmware update failed: %s", ESPhttpUpdate.getLastErrorString().c_str());
  return false;
}

void setup()
{
  Serial.begin(115200);
  delay(20);
  LOGI("Booting legacy migrator: %s", FW_VERSION);
  setSafeOutputs();

  if (!LittleFS.begin())
  {
    LOGW("LittleFS mount failed");
  }

  loadConfig();
  WiFi.persistent(false);
}

void loop()
{
  if (migrationComplete)
  {
    delay(1000);
    return;
  }

  if (!connectToConfiguredWifi())
  {
    delay(100);
    return;
  }

  if ((millis() - lastUpdateAttemptAt) < UPDATE_RETRY_INTERVAL_MS)
  {
    delay(100);
    return;
  }
  lastUpdateAttemptAt = millis();

  String firmwareUrl;
  String version;
  String notes;
  if (!fetchManifest(firmwareUrl, version, notes))
  {
    LOGW("Manifest fetch failed, retry in %u ms", (unsigned int)UPDATE_RETRY_INTERVAL_MS);
    return;
  }

  LOGI("Target manifest version=%s", version.c_str());
  LOGI("Target firmware=%s", firmwareUrl.c_str());
  if (notes.length() > 0)
  {
    LOGD("Release notes: %s", notes.c_str());
  }

  if (applyTargetFirmware(firmwareUrl))
  {
    migrationComplete = true;
    return;
  }

  LOGW("Migration update failed, retry in %u ms", (unsigned int)UPDATE_RETRY_INTERVAL_MS);
}
