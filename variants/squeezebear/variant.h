#pragma once

#include <Arduino.h>

namespace ProductVariant
{
static constexpr char kId[] = "squeezebear";
static constexpr char kDisplayName[] = "SqueezeBear";
static constexpr char kFirmwareTitle[] = "SqueezeBear Firmware";
static constexpr char kSetupPageTitle[] = "SqueezeBear Setup";
static constexpr char kSetupPageHeader[] = "SqueezeBear WiFi / MQTT Setup";
static constexpr char kApSsidPrefix[] = "SqueezeBear";
static constexpr char kDeviceModel[] = "Cube-10";
static constexpr char kDefaultOtaManifestUrl[] = "http://ota.kinkbear.de/squeezebear/manifest.json";
static constexpr char kLegacyGitHubPathToken[] = "raw.githubusercontent.com/kingbear79/vacubear/";
static constexpr char kLegacyOtaHostToken[] = "ota.kinkbear.de/";
static constexpr char kLegacyMasterPathToken[] = "ota.kinkbear.de/master/";
static constexpr char kLegacyVariantPathToken[] = "ota.kinkbear.de/ableger/";
static constexpr char kLegacyVariantId[] = "ableger";
static constexpr bool kSupportsLighting = false;
static constexpr bool kSupportsInflation = true;
static constexpr bool kValveOpenHigh = false;
static constexpr uint32_t kBootInflateDurationMs = 10000UL;
static constexpr uint32_t kDefaultShowLengthMs = 10000UL;
static constexpr uint32_t kDefaultShowNachlaufMs = 20000UL;
static constexpr uint16_t kDefaultMqttPort = 1883;
} // namespace ProductVariant
