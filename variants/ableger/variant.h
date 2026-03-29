#pragma once

#include <Arduino.h>

namespace ProductVariant
{
static constexpr char kId[] = "ableger";
static constexpr char kDisplayName[] = "Ableger";
static constexpr char kFirmwareTitle[] = "Ableger Firmware";
static constexpr char kSetupPageTitle[] = "Ableger Setup";
static constexpr char kSetupPageHeader[] = "Ableger WiFi / MQTT Setup";
static constexpr char kApSsidPrefix[] = "Ableger";
static constexpr char kDefaultOtaManifestUrl[] = "http://ota.kingbear.de/ableger/manifest.json";
static constexpr char kLegacyGitHubPathToken[] = "raw.githubusercontent.com/kingbear79/vacubear/";
static constexpr char kLegacyOtaHostToken[] = "ota.kinkbear.de/";
static constexpr char kLegacyMasterPathToken[] = "ota.kingbear.de/master/";
static constexpr uint32_t kDefaultShowLengthMs = 10000UL;
static constexpr uint32_t kDefaultShowNachlaufMs = 20000UL;
static constexpr uint16_t kDefaultMqttPort = 1883;
} // namespace ProductVariant
