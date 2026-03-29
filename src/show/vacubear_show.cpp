#include "show_variant.h"

void requestShowStart(ShowStatus &status)
{
  if (status.isRunning)
  {
    return;
  }
  status.shouldStart = true;
}

bool requestShowStop(ShowStatus &status, unsigned long now, bool lightingActive, uint32_t fadeOutMs, unsigned long &lightOnUntilAt)
{
  if (!status.isRunning)
  {
    return false;
  }

  unsigned long stopAt = now + 10UL;
  unsigned long effectiveFadeOutMs = lightingActive ? fadeOutMs : 0UL;
  status.fadeInDoneAt = stopAt;
  status.endAt = stopAt;
  status.openValveAt = stopAt;
  status.finishAt = stopAt + effectiveFadeOutMs;
  lightOnUntilAt = status.openValveAt;
  return true;
}

void tickVacubearShow(ShowStatus &status,
                      unsigned long now,
                      bool lightingActive,
                      uint32_t fadeInMs,
                      uint32_t fadeOutMs,
                      uint16_t lightLevelMax,
                      unsigned long &lightOnUntilAt,
                      ShowOutputs &outputs)
{
  outputs = ShowOutputs();

  if (status.shouldStart)
  {
    unsigned long effectiveFadeInMs = lightingActive ? fadeInMs : 0UL;
    unsigned long effectiveFadeOutMs = lightingActive ? fadeOutMs : 0UL;
    status.fadeInDoneAt = now + effectiveFadeInMs;
    status.endAt = status.fadeInDoneAt + status.showDuration;
    status.openValveAt = status.endAt + status.showNachlauf;
    status.finishAt = status.openValveAt + effectiveFadeOutMs;
    status.shouldStart = false;
    status.isRunning = true;
    lightOnUntilAt = status.openValveAt;
    outputs.applyLightTarget = true;
    outputs.lightTargetLevel = lightLevelMax;
  }

  if (status.isRunning)
  {
    if (now < status.fadeInDoneAt)
    {
      outputs.pumpEnabled = false;
      outputs.valveOpen = false;
      return;
    }
    if (now < status.endAt)
    {
      outputs.pumpEnabled = true;
      outputs.valveOpen = true;
      return;
    }
    if (now < status.openValveAt)
    {
      outputs.pumpEnabled = false;
      outputs.valveOpen = true;
      return;
    }
    if (now < status.finishAt)
    {
      outputs.pumpEnabled = false;
      outputs.valveOpen = false;
      return;
    }

    status.isRunning = false;
  }

  outputs.pumpEnabled = false;
  outputs.valveOpen = false;
}

const char *getVacubearShowPhase(const ShowStatus &status, unsigned long now)
{
  if (status.isRunning)
  {
    if (now < status.fadeInDoneAt)
    {
      return "FadeIn";
    }
    if (now < status.endAt)
    {
      return "Vakuumieren";
    }
    if (now < status.openValveAt)
    {
      return "Haltezeit";
    }
    return "FadeOut";
  }

  if (status.finishAt > 0 && (now - status.finishAt) < 2000UL)
  {
    return "FadeOut";
  }

  return "Pause";
}

void resetShowState(ShowStatus &status)
{
  status.shouldStart = false;
  status.isRunning = false;
  status.fadeInDoneAt = 0;
  status.endAt = 0;
  status.openValveAt = 0;
  status.finishAt = 0;
}
