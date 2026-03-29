#pragma once

#include <Arduino.h>

struct ShowStatus
{
  bool isRunning;
  unsigned long fadeInDoneAt;
  unsigned long endAt;
  unsigned long openValveAt;
  unsigned long finishAt;
  bool shouldStart;
  unsigned long showDuration;
  unsigned long showNachlauf;

  ShowStatus(unsigned long defaultDuration = 0, unsigned long defaultNachlauf = 0)
      : isRunning(false),
        fadeInDoneAt(0),
        endAt(0),
        openValveAt(0),
        finishAt(0),
        shouldStart(false),
        showDuration(defaultDuration),
        showNachlauf(defaultNachlauf)
  {
  }
};

struct ShowOutputs
{
  bool pumpEnabled;
  bool valveOpen;
  bool applyLightTarget;
  uint16_t lightTargetLevel;

  ShowOutputs()
      : pumpEnabled(false),
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
