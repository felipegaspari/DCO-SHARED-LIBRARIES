#ifndef __AUTOTUNE_MEASUREMENT_H__
#define __AUTOTUNE_MEASUREMENT_H__

#include "autotune_constants.h"
#include <stdint.h>

// Core timing measurement function implemented in autotune_impl.h
float find_gap(uint8_t specialMode);

// Structured return wrapper for duty error timing
struct GapMeasurement {
  bool  timedOut;
  float value;
};

inline GapMeasurement measure_gap(uint8_t specialMode) {
  float v = find_gap(specialMode);
  return { (v == kGapTimeoutSentinel), v };
}

#endif  // __AUTOTUNE_MEASUREMENT_H__