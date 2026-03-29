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
static constexpr char kDefaultOtaManifestUrl[] = "http://ota.kingbear.de/vacubear/manifest.json";
static constexpr char kLegacyGitHubPathToken[] = "raw.githubusercontent.com/kingbear79/vacubear/";
static constexpr char kLegacyOtaHostToken[] = "ota.kinkbear.de/";
static constexpr char kLegacyMasterPathToken[] = "ota.kingbear.de/master/";
static constexpr uint32_t kDefaultShowLengthMs = 10000UL;
static constexpr uint32_t kDefaultShowNachlaufMs = 20000UL;
static constexpr uint16_t kDefaultMqttPort = 1883;
} // namespace ProductVariant
