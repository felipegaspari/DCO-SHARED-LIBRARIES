/**
 * @file autotune_measurement.h
 * @brief Frequency and pulse-width gap measurement routines using hardware capture timers.
 */

 #ifndef DCO_AUTOTUNE_MEASUREMENT_H
 #define DCO_AUTOTUNE_MEASUREMENT_H
 
 #include "autotune_constants.h"
 #include <stdint.h>
 
 float find_gap(uint8_t specialMode);
 
 /**
  * @struct GapMeasurement
  * @brief Structured return wrapper for duty cycle and frequency timing capture.
  */
 struct GapMeasurement {
   bool  timedOut; ///< True if measurement exceeded timeout sentinel without edge trigger.
   float value;    ///< Measured gap duration in microseconds or raw counts.
 };
 
 /**
  * @brief Measures oscillator pulse gap with timeout detection.
  */
 inline GapMeasurement measure_gap(uint8_t specialMode) {
   float v = find_gap(specialMode);
   return { (v == kGapTimeoutSentinel), v };
 }
 
 #endif  // DCO_AUTOTUNE_MEASUREMENT_H