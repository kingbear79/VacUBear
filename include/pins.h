#pragma once

#include <Arduino.h>

// Zentrale Pinbelegung fuer diese Hardwarevariante.
// Bei neuen PCB-/Geraeteversionen nur hier anpassen.
//
// ESP-12F GPIO-Zuordnung (vormals D1-mini Labels):
// D7->GPIO13, D6->GPIO12, D5->GPIO14, D2->GPIO4, D1->GPIO5
static const uint8_t PIN_PUMPE1 = 13; // GPIO13
static const uint8_t PIN_PUMPE2 = 12; // GPIO12
static const uint8_t PIN_VENTIL = 14; // GPIO14
static const uint8_t PIN_TASTER = 4;  // GPIO4
static const uint8_t PIN_LED = 5;     // GPIO5
