#pragma once

#include <Arduino.h>

#include <stdint.h>

enum PumpMode : uint8_t
{
  PUMP_MODE_OFF = 0,
  PUMP_MODE_PRIMARY = 1,
  PUMP_MODE_SECONDARY = 2,
};

struct ShowStatus
{
  bool isRunning;
  unsigned long fadeInDoneAt;
  unsigned long vacuumEndAt;
  unsigned long holdEndAt;
  unsigned long openValveAt;
  unsigned long finishAt;
  bool shouldStart;
  bool inflateSkipped;
  unsigned long showDuration;
  unsigned long showNachlauf;
  unsigned long showInflate;

  ShowStatus(unsigned long defaultDuration = 0, unsigned long defaultNachlauf = 0, unsigned long defaultInflate = 0)
      : isRunning(false),
        fadeInDoneAt(0),
        vacuumEndAt(0),
        holdEndAt(0),
        openValveAt(0),
        finishAt(0),
        shouldStart(false),
        inflateSkipped(false),
        showDuration(defaultDuration),
        showNachlauf(defaultNachlauf),
        showInflate(defaultInflate)
  {
  }
};

struct ShowOutputs
{
  PumpMode pumpMode;
  bool valveOpen;
  bool applyLightTarget;
  uint16_t lightTargetLevel;

  ShowOutputs()
      : pumpMode(PUMP_MODE_OFF),
        valveOpen(false),
        applyLightTarget(false),
        lightTargetLevel(0)
  {
  }
};

void requestShowStart(ShowStatus &status);
bool requestShowStop(ShowStatus &status, unsigned long now, bool lightingActive, uint32_t fadeOutMs, unsigned long &lightOnUntilAt);
void tickShowVariant(ShowStatus &status,
                     unsigned long now,
                     bool lightingActive,
                     uint32_t fadeInMs,
                     uint32_t fadeOutMs,
                     uint16_t lightLevelMax,
                     unsigned long &lightOnUntilAt,
                     ShowOutputs &outputs);
const char *getShowVariantPhase(const ShowStatus &status, unsigned long now);
void resetShowState(ShowStatus &status);
