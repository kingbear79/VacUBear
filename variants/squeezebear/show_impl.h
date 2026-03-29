#pragma once

#include "show_variant.h"

namespace ShowVariantImpl
{
inline void requestShowStart(ShowStatus &status)
{
  if (status.isRunning)
  {
    return;
  }
  status.shouldStart = true;
}

inline bool requestShowStop(ShowStatus &status, unsigned long now, bool lightingActive, uint32_t fadeOutMs, unsigned long &lightOnUntilAt)
{
  (void)lightingActive;
  (void)fadeOutMs;
  (void)lightOnUntilAt;
  if (!status.isRunning)
  {
    return false;
  }

  unsigned long stopAt = now + 10UL;
  status.fadeInDoneAt = stopAt;
  status.vacuumEndAt = stopAt;
  status.holdEndAt = stopAt;
  status.openValveAt = stopAt;
  status.finishAt = stopAt + 1000UL;
  status.inflateSkipped = true;
  return true;
}

inline void tickShow(ShowStatus &status,
                     unsigned long now,
                     bool lightingActive,
                     uint32_t fadeInMs,
                     uint32_t fadeOutMs,
                     uint16_t lightLevelMax,
                     unsigned long &lightOnUntilAt,
                     ShowOutputs &outputs)
{
  (void)lightingActive;
  (void)fadeInMs;
  (void)fadeOutMs;
  (void)lightLevelMax;
  (void)lightOnUntilAt;
  outputs = ShowOutputs();

  if (status.shouldStart)
  {
    status.fadeInDoneAt = now;
    status.vacuumEndAt = status.fadeInDoneAt + status.showDuration;
    status.holdEndAt = status.vacuumEndAt + status.showNachlauf;
    status.openValveAt = status.holdEndAt;
    status.finishAt = status.openValveAt + status.showInflate;
    status.shouldStart = false;
    status.inflateSkipped = false;
    status.isRunning = true;
  }

  if (status.isRunning)
  {
    if (now < status.vacuumEndAt)
    {
      outputs.pumpMode = PUMP_MODE_PRIMARY;
      outputs.valveOpen = false;
      return;
    }
    if (now < status.holdEndAt)
    {
      outputs.pumpMode = PUMP_MODE_OFF;
      outputs.valveOpen = false;
      return;
    }
    if (now < status.finishAt)
    {
      outputs.pumpMode = status.inflateSkipped ? PUMP_MODE_OFF : PUMP_MODE_SECONDARY;
      outputs.valveOpen = true;
      return;
    }

    status.isRunning = false;
  }

  outputs.pumpMode = PUMP_MODE_OFF;
  outputs.valveOpen = false;
}

inline const char *getShowPhase(const ShowStatus &status, unsigned long now)
{
  if (status.isRunning)
  {
    if (now < status.vacuumEndAt)
    {
      return "Vakuumieren";
    }
    if (now < status.holdEndAt)
    {
      return "Haltezeit";
    }
    if (now < status.openValveAt)
    {
      return "Aufblasen";
    }
    if (now < status.finishAt)
    {
      return status.inflateSkipped ? "Belueften" : "Aufblasen";
    }
    return "Belueften";
  }

  if (status.finishAt > 0 && (now - status.finishAt) < 2000UL)
  {
    return "Belueften";
  }

  return "Pause";
}

inline void resetShowState(ShowStatus &status)
{
  status.shouldStart = false;
  status.isRunning = false;
  status.fadeInDoneAt = 0;
  status.vacuumEndAt = 0;
  status.holdEndAt = 0;
  status.openValveAt = 0;
  status.finishAt = 0;
  status.inflateSkipped = false;
}
} // namespace ShowVariantImpl
