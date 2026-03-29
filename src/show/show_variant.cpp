#include "show_variant.h"
#include "show_impl.h"

void requestShowStart(ShowStatus &status)
{
  ShowVariantImpl::requestShowStart(status);
}

bool requestShowStop(ShowStatus &status, unsigned long now, bool lightingActive, uint32_t fadeOutMs, unsigned long &lightOnUntilAt)
{
  return ShowVariantImpl::requestShowStop(status, now, lightingActive, fadeOutMs, lightOnUntilAt);
}

void tickShowVariant(ShowStatus &status,
                     unsigned long now,
                     bool lightingActive,
                     uint32_t fadeInMs,
                     uint32_t fadeOutMs,
                     uint16_t lightLevelMax,
                     unsigned long &lightOnUntilAt,
                     ShowOutputs &outputs)
{
  ShowVariantImpl::tickShow(status, now, lightingActive, fadeInMs, fadeOutMs, lightLevelMax, lightOnUntilAt, outputs);
}

const char *getShowVariantPhase(const ShowStatus &status, unsigned long now)
{
  return ShowVariantImpl::getShowPhase(status, now);
}

void resetShowState(ShowStatus &status)
{
  ShowVariantImpl::resetShowState(status);
}
