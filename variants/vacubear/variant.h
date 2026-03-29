#pragma once

#include <Arduino.h>

namespace ProductVariant
{
static constexpr char kId[] = "vacubear";
static constexpr char kDisplayName[] = "VacUBear";
static constexpr char kFirmwareTitle[] = "VacUBear Firmware";
static constexpr char kSetupPageTitle[] = "VacUBear Setup";
static constexpr char kSetupPageHeader[] = "VacUBear WiFi / MQTT Setup";
static constexpr char kApSsidPrefix[] = "VacUBear";
static constexpr char kDeviceModel[] = "Frame-25";
static constexpr char kDefaultOtaManifestUrl[] = "http://ota.kinkbear.de/vacubear/manifest.json";
static constexpr char kLegacyGitHubPathToken[] = "raw.githubusercontent.com/kingbear79/vacubear/";
static constexpr char kLegacyOtaHostToken[] = "ota.kinkbear.de/";
static constexpr char kLegacyMasterPathToken[] = "ota.kinkbear.de/master/";
static constexpr char kLegacyVariantPathToken[] = "";
static constexpr char kLegacyVariantId[] = "";
static constexpr bool kSupportsLighting = true;
static constexpr bool kSupportsInflation = false;
static constexpr bool kValveOpenHigh = true;
static constexpr uint32_t kBootInflateDurationMs = 0UL;
static constexpr uint32_t kDefaultShowLengthMs = 10000UL;
static constexpr uint32_t kDefaultShowNachlaufMs = 20000UL;
static constexpr uint16_t kDefaultMqttPort = 1883;
} // namespace ProductVariant
