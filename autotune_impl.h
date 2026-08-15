#ifndef __AUTOTUNE_IMPL_H__
#define __AUTOTUNE_IMPL_H__

#include "hardware/watchdog.h"
#include "autotune.h"
#include "autotune_search_impl.h"

// =============================================================================
// Global Variable Definitions
// =============================================================================

bool calibrationFlag = false;
bool manualCalibrationFlag = false;
bool firstTuneFlag = false;
volatile bool calibrationCancelRequested = false;

uint8_t calibrationScope = CAL_SCOPE_FULL;
uint8_t calibrationPrecision = CAL_PRECISION_NORMAL;
uint8_t autotuneAmp0Mode = (uint8_t)AUTOTUNE_AMP0_MODE_DEFAULT;
uint8_t autotuneAmpMethod = (uint8_t)AUTOTUNE_AMP_METHOD_DEFAULT;
uint8_t autotuneSearchMode = (uint8_t)AUTOTUNE_SEARCH_MODE_DEFAULT;

uint8_t manualCalibrationStage = 0;
int8_t  manualCalibrationOffset[NUM_OSCILLATORS] = { 0 };
uint8_t manualCalibrationStep = 0;
uint16_t ampComp440[NUM_OSCILLATORS] = { 0 };
int16_t  ampCompDutyOffset[NUM_OSCILLATORS] = { 0 };

uint32_t calibrationData[chanLevelVoiceDataSize] = { 0 };
float    calPointDutyErrPct[kCalReportPairs];
uint8_t  calPointSource[kCalReportPairs];
int      calReportLadderInterval = 0;
int      calReportAnchorPair     = -1;
uint32_t calRunProbes  = 0;
unsigned long calRunStartMs = 0;

volatile bool calibrationVerifyRequested = false;
volatile bool pwCvProbeRequested = false;
uint8_t manualCalSavedSyncMode = 0;
uint8_t manualCalSavedSoftSyncChunks = 0;
volatile bool calSyncNeutralRequested = false;

uint8_t currentDCO = 0;
unsigned long DCOCalibrationStart = 0;
volatile uint16_t ampCompCalibrationVal = 0;
float calibrationFreqHz = 0.0f;
float gapGateFreqHz = 0.0f;
float g_lastDrivenFreqHz = 0.0f;

uint16_t initManualAmpCompCalibrationVal[NUM_OSCILLATORS];
volatile uint16_t ampCompLowestFreqVal = (uint16_t)(10u * DIV_COUNTER / 14000u);
uint8_t DCO_calibration_current_note = DCO_calibration_start_note;
byte autotuneDebug = 2;

static double g_gapLogCurrentPeriodUs = 0.0;
static double g_gapLogTargetDutyFraction = 0.5;

///////////////////////////////////
// CALIBRATION SUMMARY STRUCTURES

struct OscCalSummary {
  bool     attempted;
  bool     ok;
  bool     cancelled;
  uint8_t  pwCh;
  bool     hasPW;
  uint16_t pwCenter;
  uint16_t pwLow;
  uint16_t pwHigh;
  uint16_t anchorAmp;
  float    lowestHz;
  float    highestHz;
  float    avgDutyErrPct;
  float    worstDutyErrPct;
  int      worstPair;
  float    worstHz;
  uint32_t probes;
  uint32_t elapsedMs;
};

static OscCalSummary g_oscSummary[NUM_OSCILLATORS];

struct PWCalSummary {
  bool     attempted;
  bool     ok;
  bool     cancelled;
  uint8_t  ch;
  uint8_t  pin;
  uint16_t pwCenter;
  double   centerDuty;
  uint16_t pwLow;
  double   lowDuty;
  uint16_t pwHigh;
  double   highDuty;
  int      probes;
  uint32_t elapsedMs;
};

static PWCalSummary g_pwSummary[NUM_PW_CHANNELS];


// =============================================================================
// Voice Engine & PWM Hardware Control Helpers
// =============================================================================

static inline void autotune_fill_init_manual_amp() {
  static bool filled = false;
  if (filled) return;
  for (int i = 0; i < NUM_OSCILLATORS; ++i) {
    initManualAmpCompCalibrationVal[i] = initManualAmpCompCalibrationValPreset;
  }
  filled = true;
}

static inline void apply_pw_center(uint8_t ch) {
  if (ch >= NUM_PW_CHANNELS || PW_PINS[ch] == PW_PIN_UNASSIGNED) return;

  uint16_t center = PW_CENTER[ch];
  if (center < (DIV_COUNTER_PW / 10) || center > (DIV_COUNTER_PW * 9 / 10)) {
    center = DIV_COUNTER_PW / 2;
  }

  pwm_set_chan_level(PW_PWM_SLICES[ch], pwm_gpio_to_channel(PW_PINS[ch]), center);
  PW[ch] = center;

  Serial.println((String)"  [PW_HARDWARE] ch=" + ch + " GP" + PW_PINS[ch] +
  " -> PW_CENTER=" + center + " (Readback CC=" + pw_level_readback(ch) + ")");
}

static void apply_pw_center_solo(uint8_t soloCh) {
  for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    if (PW_PINS[ch] == PW_PIN_UNASSIGNED) continue;
    if (ch == soloCh) {
      apply_pw_center(ch);
    } else {
      pwm_set_chan_level(PW_PWM_SLICES[ch], pwm_gpio_to_channel(PW_PINS[ch]), 0);
    }
  }
}

static void reset_pw_to_DIV_COUNTER_PW() {
  for (int i = 0; i < NUM_PW_CHANNELS; i++) {
    if (PW_PINS[i] == PW_PIN_UNASSIGNED) continue;
    pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), DIV_COUNTER_PW);
  }
}

static void restore_voice_engine_after_calibration() {
  for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    apply_pw_center(ch);
  }
  start_voice_sms();
  for (int i = 0; i < NUM_VOICES_TOTAL; i++) {
    note_on_flag[i] = 1;
  }
}

// Helper: Cleanly mute oscillators and keep state machines running so the
// internal DCO core capacitors and RANGE RC filters can naturally drain to 0V.
static void disable_all_oscillators_and_range_pwm() {
  // 1. Keep state machines running in sync
  start_voice_sms();

  // 2. Mute all RANGE amplitudes cleanly.
  // The PIO keeps cycling, which naturally drains the analog RC filters to 0V
  // WITHOUT altering the synth's internal polyphonic voice routing.
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    #ifndef RANGE0_PIO_DITHER_TEST
    gpio_set_function(RANGE_PINS[i], GPIO_FUNC_PWM);
    #endif
    write_range_pwm(i, 0); // Mute voltage
  }

  // 3. Mute all PW channels (0% duty / 0V)
  for (int ch = 0; ch < NUM_PW_CHANNELS; ch++) {
    if (PW_PINS[ch] == PW_PIN_UNASSIGNED) continue;
    pwm_set_chan_level(PW_PWM_SLICES[ch], pwm_gpio_to_channel(PW_PINS[ch]), 0);
    PW[ch] = 0;
  }

  g_lastDrivenFreqHz = 0.0f;
}

void restart_DCO_calibration() {
  autotune_fill_init_manual_amp();

  VOICE_NOTES[0] = DCO_calibration_start_note;
  DCO_calibration_current_note = DCO_calibration_start_note;

  calibrationData[0] = 0;
  calibrationData[1] = ampCompLowestFreqVal;
  calibrationData[2] = (uint32_t)(note_to_freq(DCO_calibration_current_note - calibration_note_interval) * 100);
  calibrationData[3] = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];

  DCOCalibrationStart = millis();

  // 1. Ensure inactive oscillators stay muted
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    if (i != currentDCO) {
      write_range_pwm(i, 0);
    }
  }

  // 2. CONFIGURE PW FOR ACTIVE OSCILLATOR ONLY
  const uint8_t pwCh = cal_pw_channel(currentDCO);
  const bool hasPW   = osc_has_pw(currentDCO);

  if (hasPW) {
    uint16_t center = PW_CENTER[pwCh];
    // Sanity check: ensure center is a valid square wave
    if (center < 100 || center > DIV_COUNTER_PW - 100) center = DIV_COUNTER_PW / 2;

    for (int ch = 0; ch < NUM_PW_CHANNELS; ch++) {
      if (PW_PINS[ch] == PW_PIN_UNASSIGNED) continue;
      if (ch == pwCh) {
        pwm_set_chan_level(PW_PWM_SLICES[ch], pwm_gpio_to_channel(PW_PINS[ch]), center);
        PW[ch] = center;
      } else {
        pwm_set_chan_level(PW_PWM_SLICES[ch], pwm_gpio_to_channel(PW_PINS[ch]), 0);
        PW[ch] = 0;
      }
    }
  } else {
    // Mute all PW channels for odd/fixed-wave oscillators
    for (int ch = 0; ch < NUM_PW_CHANNELS; ch++) {
      if (PW_PINS[ch] != PW_PIN_UNASSIGNED) {
        pwm_set_chan_level(PW_PWM_SLICES[ch], pwm_gpio_to_channel(PW_PINS[ch]), 0);
        PW[ch] = 0;
      }
    }
  }

  // 3. APPLY STARTING AMPLITUDE & PITCH TO ACTIVE OSCILLATOR
  ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];
  write_range_pwm(currentDCO, ampCompCalibrationVal);

  // Kickstart frequency exactly how Manual Calibration does it
  voice_task_autotune(0, ampCompCalibrationVal);

  // 4. Print Hardware Diagnostics
  Serial.println((String)"\n--- [HARDWARE STATE DCO " + currentDCO + "] ---");
  Serial.println((String)"  PIO: " + VOICE_TO_PIO[currentDCO] + " | SM: " + VOICE_TO_SM[currentDCO]);
  Serial.println((String)"  RANGE Pin: GP" + RANGE_PINS[currentDCO] + " | AMP=" + ampCompCalibrationVal +
  " (HW RangeCC=" + range_level_readback(currentDCO) + ")");
  if (hasPW) {
    Serial.println((String)"  PW Channel: " + pwCh + " (GP" + PW_PINS[pwCh] + ") | PW_CENTER=" + PW_CENTER[pwCh] +
    " (HW PW_CC=" + pw_level_readback(pwCh) + ")");
  } else {
    Serial.println((String)"  PW Channel: None / Muted (HW PW_CC=" + pw_level_readback(pwCh) + ")");
  }
  Serial.println("--------------------------------------------------");

  g_lastDrivenFreqHz = 0.0f;

  // Give the active DCO's analog circuit a moment to rise to its starting baseline
  delay(150);
}

// =============================================================================
// Edge-Timing Core Measurement Routine (find_gap)
// =============================================================================

float find_gap(uint8_t specialMode) {
  double freqHz = (gapGateFreqHz > 0.0f) ? (double)gapGateFreqHz : (double)note_to_freq(DCO_calibration_current_note);
  double idealPeriodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;

  uint16_t samplesTarget = kGapSamplesDefault;
  if (specialMode == 2 || specialMode == 3) {
    const CalPrecisionProfile &prec = cal_precision();
    samplesTarget = prec.gapSamplesMin;
    const double halfPeriodMs = idealPeriodUs / 2000.0;
    if (halfPeriodMs > 0.0) {
      long n = lround((double)prec.gapWindowMs / halfPeriodMs);
      if (n > (long)prec.gapSamplesMax) n = (long)prec.gapSamplesMax;
      if (n > (long)samplesTarget)      samplesTarget = (uint16_t)n;
      if (freqHz < (double)kSearchStepVeryLowHz && samplesTarget < kGapSamplesVeryLowMin) {
        samplesTarget = kGapSamplesVeryLowMin;
      }
      long cap = lround((double)prec.gapMaxWindowMs / halfPeriodMs);
      if (cap < 4) cap = 4;
      if ((long)samplesTarget > cap) samplesTarget = (uint16_t)cap;
    }
  }

  unsigned long timeoutUs = kGapTimeoutUs;
  if (idealPeriodUs > 0.0) {
    double scaled = idealPeriodUs * kGapTimeoutPeriods;
    if (scaled > (double)timeoutUs) {
      timeoutUs = (scaled > (double)kGapTimeoutMaxUs) ? kGapTimeoutMaxUs : (unsigned long)scaled;
    }
  }

  double dtMinUs = 0.0, dtMaxUs = 0.0;
  if (idealPeriodUs > 0.0) {
    dtMinUs = idealPeriodUs * 0.01;
    dtMaxUs = idealPeriodUs * 0.99;
    if (dtMinUs < (double)kEdgeDebounceMinUs) dtMinUs = (double)kEdgeDebounceMinUs;
    if (dtMaxUs > (double)timeoutUs)         dtMaxUs = (double)timeoutUs;
  }

  const uint32_t cyclesPerUs    = (uint32_t)(rp2040.f_cpu() / 1000000);
  const double   usPerCycle     = 1.0 / (double)cyclesPerUs;
  const uint32_t debounceCycles = (uint32_t)kEdgeDebounceMinUs * cyclesPerUs;
  const uint32_t dtMinCycles    = (uint32_t)(dtMinUs * (double)cyclesPerUs);
  const uint32_t dtMaxCycles    = (uint32_t)(dtMaxUs * (double)cyclesPerUs);

  int      pulseCount       = 0;
  uint16_t acceptedSamples  = 0;
  uint64_t risingSumCycles  = 0;
  uint64_t fallingSumCycles = 0;
  bool     lastVal          = 0;
  uint16_t risingCount      = 0;
  uint16_t fallingCount     = 0;
  uint16_t edgesSeen        = 0;
  uint16_t edgesRejected    = 0;

  unsigned long lastEdgeTime   = micros();
  uint32_t      lastEdgeCycles = rp2040.getCycleCount();
  uint8_t       pollTick       = 0;

  while (acceptedSamples < samplesTarget) {
    const bool rawVal = (bool)gpio_get(DCO_calibration_pin);
    const bool val    = kGapPolarityInverted ? !rawVal : rawVal;

    if (val == lastVal && ((++pollTick & 0x3F) != 0)) continue;

    const uint32_t      nowCycles = rp2040.getCycleCount();
    const unsigned long nowUs     = micros();

    if ((nowUs - lastEdgeTime) > timeoutUs) {
      if (autotuneDebug >= 1) {
        Serial.println((String)"  [GAP_TIMEOUT] DCO=" + currentDCO + " Mode=" + specialMode +
        " Note=" + DCO_calibration_current_note + " Freq=" + fmt_freq((float)freqHz) + "Hz" +
        " AMP=" + ampCompCalibrationVal + " (RangeCC=" + range_level_readback(currentDCO) +
        " PW_CC=" + pw_level_readback(cal_pw_channel(currentDCO)) + ")" +
        " EdgesSeen=" + edgesSeen + " Rej=" + edgesRejected + " Accepted=" + acceptedSamples + "/" + samplesTarget +
        " TimeoutUs=" + timeoutUs + " RawPin=" + (int)rawVal);
      }
      return kGapTimeoutSentinel;
    }

    if (val != lastVal) {
      const uint32_t fine24     = (nowCycles - lastEdgeCycles) & 0x00FFFFFFu;
      const uint64_t wallCycles = (uint64_t)(nowUs - lastEdgeTime) * cyclesPerUs;
      const int64_t  lostWraps  = ((int64_t)wallCycles - (int64_t)fine24 + (int64_t)(1u << 23)) >> 24;
      const uint32_t dtCycles   = (lostWraps > 0) ? (fine24 + (uint32_t)((uint64_t)lostWraps << 24)) : fine24;

      if (dtCycles >= debounceCycles) {
        lastVal = val;
        edgesSeen++;
        if (pulseCount == 1 && val == 0) pulseCount = 0;

        if (pulseCount > 2) {
          bool intervalOk = (idealPeriodUs <= 0.0 || (dtCycles >= dtMinCycles && dtCycles <= dtMaxCycles));
          if (intervalOk) {
            if (val == 0) {
              fallingSumCycles += dtCycles;
              fallingCount++;
            } else {
              risingSumCycles += dtCycles;
              risingCount++;
            }
            acceptedSamples++;
          } else {
            edgesRejected++;
          }
        }
        lastEdgeTime   = nowUs;
        lastEdgeCycles = nowCycles;
        pulseCount++;
      }
    }
  }

  if (specialMode == 3 && (risingCount == 0 || fallingCount == 0)) {
    if (autotuneDebug >= 2) {
      Serial.println((String)"  [GAP_ONESIDED] DCO=" + currentDCO + " Highs=" + risingCount + " Lows=" + fallingCount + " (Rail Pegged)");
    }
    return kGapTimeoutSentinel;
  }

  float avgLowUs  = (fallingCount > 0) ? (float)((double)fallingSumCycles * usPerCycle / (double)fallingCount) : 0.0f;
  float avgHighUs = (risingCount > 0)  ? (float)((double)risingSumCycles * usPerCycle / (double)risingCount)  : 0.0f;
  float measuredPeriodUs = avgLowUs + avgHighUs;
  float diffUs = avgHighUs - avgLowUs;

  if (specialMode == 3 && idealPeriodUs > 0.0 && fabsf(measuredPeriodUs - (float)idealPeriodUs) > kGapPeriodTolRatio * (float)idealPeriodUs) {
    if (autotuneDebug >= 2) {
      Serial.println((String)"  [GAP_OFFPERIOD] DCO=" + currentDCO + " T_meas=" + measuredPeriodUs + "us T_ideal=" + (float)idealPeriodUs + "us");
    }
    return kGapTimeoutSentinel;
  }

  if (autotuneDebug >= 2) {
    double dutyIdealPct = (idealPeriodUs > 0.0) ? (0.5 + (double)diffUs / (2.0 * idealPeriodUs)) * 100.0 : 0.0;
    Serial.println((String)"    [GAP_MEASURE] Mode=" + specialMode + " DCO=" + currentDCO +
    " Freq=" + fmt_freq((float)freqHz) + "Hz AMP=" + ampCompCalibrationVal +
    " PW=" + pw_level_readback(cal_pw_channel(currentDCO)) +
    " DiffUs=" + diffUs + " Duty≈" + String(dutyIdealPct, 2) + "%");
  }

  return diffUs;
}

// =============================================================================
// Modern Pulse-Width (PW) Calibration Engine
// =============================================================================

// Helper: Program PW hardware and measure the raw edge gap (used by diagnostics & probes)
static GapMeasurement set_pw_and_measure(uint8_t pwCh, uint16_t pw) {
  if (pwCh >= NUM_PW_CHANNELS || PW_PINS[pwCh] == PW_PIN_UNASSIGNED) {
    return { true, kGapTimeoutSentinel };
  }
  pwm_set_chan_level(PW_PWM_SLICES[pwCh], pwm_gpio_to_channel(PW_PINS[pwCh]), pw);
  PW[pwCh] = pw;
  delay(30);
  return measure_gap(2);
}

// Helper: Program PW hardware, wait for analog settle, and return measured duty (0.0..1.0).
// Returns -1.0 on timeout / collapsed pulse.
static double measure_pw_duty(uint8_t pwCh, uint16_t pw, double freqHz) {
  if (pwCh >= NUM_PW_CHANNELS || PW_PINS[pwCh] == PW_PIN_UNASSIGNED) return -1.0;

  pwm_set_chan_level(PW_PWM_SLICES[pwCh], pwm_gpio_to_channel(PW_PINS[pwCh]), pw);
  PW[pwCh] = pw;

  const CalPrecisionProfile &prec = cal_precision();
  wait_periods((float)freqHz, prec.settlePeriods, prec.settleMinMs * 1000u);

  ++calRunProbes;
  GapMeasurement gm = measure_gap(2);
  if (gm.timedOut || freqHz <= 0.0) {
    if (autotuneDebug >= 2) {
      Serial.println((String)"  [PW_PROBE] Ch=" + pwCh + " PW=" + pw + " -> TIMEOUT (Pulse Collapsed)");
    }
    return -1.0;
  }

  double periodUs = 1000000.0 / freqHz;
  double duty = 0.5 + ((double)gm.value / (2.0 * periodUs));
  if (duty < 0.0) duty = 0.0;
  if (duty > 1.0) duty = 1.0;

  if (autotuneDebug >= 2) {
    Serial.println((String)"  [PW_PROBE] Ch=" + pwCh + " PW=" + pw +
    " -> GapUs=" + gm.value + " Duty=" + String(duty * 100.0, 2) + "%");
  }
  return duty;
}

// Low-level unified PW root-finder with Dead-Zone Edge Detection & 2-Point Regression
PWSearchResult find_pw_for_target_duty(
  uint8_t  pwCh,
  double   targetDutyFraction,
  double   dutyToleranceFraction,
  uint16_t pwMin,
  uint16_t pwMax,
  uint16_t pwSeed,
  double   freqHz
) {
  PWSearchResult res = { false, pwSeed, -1.0, 1.0, 0 };
  if (pwCh >= NUM_PW_CHANNELS || PW_PINS[pwCh] == PW_PIN_UNASSIGNED || freqHz <= 0.0) {
    return res;
  }

  const CalPrecisionProfile &prec = cal_precision();
  const int maxProbes = prec.bisectIters;

  // Dynamic dead-zone boundaries (discovered when pulses collapse into timeouts)
  int32_t deadLowPW  = (int32_t)pwMin - 1;
  int32_t deadHighPW = (int32_t)pwMax + 1;

  bool     haveValid  = false;
  uint16_t bestPW     = pwSeed;
  double   bestDuty   = -1.0;
  double   bestAbsErr = 1e9;

  // 2-Point history of valid measurements for slope calculation
  int32_t p0_pw = -1, p1_pw = -1;
  double  p0_duty = 0.0, p1_duty = 0.0;

  double curPW = (double)constrain(pwSeed, pwMin, pwMax);

  for (int probe = 0; probe < maxProbes; ++probe) {
    if (calibrationCancelRequested || (millis() - DCOCalibrationStart > 30000UL)) break;

    int32_t testPW32 = (int32_t)lround(curPW);
    testPW32 = constrain(testPW32, (int32_t)pwMin, (int32_t)pwMax);

    // Keep testPW strictly inside the known-alive window
    if (testPW32 <= deadLowPW)  testPW32 = deadLowPW + 1;
    if (testPW32 >= deadHighPW) testPW32 = deadHighPW - 1;

    // If the alive window is exhausted, no further improvement is possible
    if (testPW32 <= deadLowPW || testPW32 >= deadHighPW || testPW32 < (int32_t)pwMin || testPW32 > (int32_t)pwMax) {
      if (autotuneDebug >= 2) {
        Serial.println((String)"  [PW_EXHAUSTED] Working window fully explored. Dead bounds: [" +
        deadLowPW + " .. " + deadHighPW + "]");
      }
      break;
    }

    uint16_t testPW = (uint16_t)testPW32;
    double duty = measure_pw_duty(pwCh, testPW, freqHz);
    res.probes++;

    // -------------------------------------------------------------------------
    // CASE 1: TIMEOUT / PULSE COLLAPSED
    // -------------------------------------------------------------------------
    if (duty < 0.0) {
      if (haveValid) {
        if (testPW < bestPW) {
          deadLowPW = max(deadLowPW, (int32_t)testPW);
          // If the dead count is immediately adjacent to our best working pulse,
          // we have reached the absolute physical edge of the comparator!
          if (bestPW - deadLowPW <= 1 && targetDutyFraction < bestDuty) {
            if (autotuneDebug >= 2) {
              Serial.println((String)"  [PW_EDGE_LOCK] Reached lowest physical pulse limit at PW=" + bestPW +
              " (duty=" + String(bestDuty * 100.0, 2) + "%). Dead zone at PW=" + deadLowPW);
            }
            break; // Stop immediately, best possible pulse is locked
          }
          curPW = (double)(bestPW + deadLowPW) / 2.0; // Bisect toward safety
        } else {
          deadHighPW = min(deadHighPW, (int32_t)testPW);
          if (deadHighPW - bestPW <= 1 && targetDutyFraction > bestDuty) {
            if (autotuneDebug >= 2) {
              Serial.println((String)"  [PW_EDGE_LOCK] Reached highest physical pulse limit at PW=" + bestPW +
              " (duty=" + String(bestDuty * 100.0, 2) + "%). Dead zone at PW=" + deadHighPW);
            }
            break; // Stop immediately
          }
          curPW = (double)(bestPW + deadHighPW) / 2.0; // Bisect toward safety
        }
      } else {
        // No valid point seen yet: step toward center of allowed range
        double centerApprox = (double)(pwMin + pwMax) / 2.0;
        if (testPW < centerApprox) {
          deadLowPW = max(deadLowPW, (int32_t)testPW);
          curPW = (double)(testPW + centerApprox) / 2.0;
        } else {
          deadHighPW = min(deadHighPW, (int32_t)testPW);
          curPW = (double)(testPW + centerApprox) / 2.0;
        }
      }
      continue;
    }

    // -------------------------------------------------------------------------
    // CASE 2: VALID MEASUREMENT
    // -------------------------------------------------------------------------
    haveValid = true;
    double err = duty - targetDutyFraction;
    double absErr = fabs(err);

    if (absErr < bestAbsErr) {
      bestAbsErr = absErr;
      bestPW     = testPW;
      bestDuty   = duty;
    }

    if (autotuneDebug >= 2) {
      Serial.println((String)"  [PW_PROBE] PW=" + testPW + " duty=" + String(duty * 100.0, 2) +
      "% (target=" + String(targetDutyFraction * 100.0, 1) +
      "% err=" + String(err * 100.0, 3) + "%)");
    }

    // Target achieved within tolerance
    if (absErr <= dutyToleranceFraction) {
      break;
    }

    // Physical boundary limit check: if target wants lower duty, but next lower count is dead
    if (err > 0.0 && (int32_t)testPW <= deadLowPW + 1) {
      if (autotuneDebug >= 2) {
        Serial.println((String)"  [PW_EDGE_LOCK] Target unreachable (requires dead PW < " + (deadLowPW + 1) +
        "). Keeping best valid PW=" + testPW);
      }
      break;
    }
    // Physical boundary limit check: if target wants higher duty, but next higher count is dead
    if (err < 0.0 && (int32_t)testPW >= deadHighPW - 1) {
      if (autotuneDebug >= 2) {
        Serial.println((String)"  [PW_EDGE_LOCK] Target unreachable (requires dead PW > " + (deadHighPW - 1) +
        "). Keeping best valid PW=" + testPW);
      }
      break;
    }

    // Update 2-point valid history for regression
    if (p0_pw < 0) {
      p0_pw = testPW; p0_duty = duty;
    } else if (p1_pw < 0 && testPW != p0_pw) {
      p1_pw = testPW; p1_duty = duty;
    } else if (testPW != p1_pw) {
      p0_pw = p1_pw;  p0_duty = p1_duty;
      p1_pw = testPW; p1_duty = duty;
    }

    // Calculate next candidate via measured slope regression
    if (p0_pw >= 0 && p1_pw >= 0 && fabs(p1_duty - p0_duty) > 1e-4) {
      double slope = (p1_duty - p0_duty) / (double)(p1_pw - p0_pw);
      double estPW = (double)testPW + (targetDutyFraction - duty) / slope;

      // Damping: prevent huge jumps from noise
      double maxJump = (double)(pwMax - pwMin) * 0.35;
      if (estPW < curPW - maxJump) estPW = curPW - maxJump;
      if (estPW > curPW + maxJump) estPW = curPW + maxJump;

      curPW = estPW;
    } else {
      // Baseline proportional step
      double delta = (targetDutyFraction - duty) * (double)(pwMax - pwMin);
      curPW = (double)testPW + delta;
    }
  }

  // ---------------------------------------------------------------------------
  // CONFIRMATION AVERAGING PHASE
  // ---------------------------------------------------------------------------
  if (haveValid && bestDuty >= 0.0) {
    int confirmReads = max(1, (int)prec.confirmReads);
    double sumDuty = 0.0;
    int validReads = 0;

    for (int r = 0; r < confirmReads; ++r) {
      double d = measure_pw_duty(pwCh, bestPW, freqHz);
      if (d >= 0.0) {
        sumDuty += d;
        validReads++;
      }
    }

    if (validReads > 0) {
      bestDuty = sumDuty / (double)validReads;
    }

    res.ok        = true;
    res.pw        = bestPW;
    res.duty      = bestDuty;
    res.errorFrac = bestDuty - targetDutyFraction;
  }

  return res;
}

// High-level PW Center Search
void find_PW_center(uint8_t mode) {
  // Use 440 Hz anchor instead of 16 Hz for massive speedup!
  uint8_t note = manual_cal_reference_note;
  DCO_calibration_current_note = note;
  VOICE_NOTES[0] = note;

  const uint8_t pwCh = cal_pw_channel(currentDCO);
  if (PW_PINS[pwCh] == PW_PIN_UNASSIGNED) return;

  DCOCalibrationStart = millis();

  // Retrieve the strong 440 Hz amplitude (with fallback for fresh EEPROM/Flash)
  uint16_t amp = ampComp440[currentDCO];
  if (amp < 500) {
    amp = (uint16_t)(DIV_COUNTER / 4); // Safe fallback (~3500)
  }
  ampCompCalibrationVal = amp;

  // Force the hardware to the new 440Hz frequency and amplitude
  write_range_pwm(currentDCO, amp);
  voice_task_autotune((mode == 0) ? 2 : 3, amp);

  double freqHz   = (double)note_to_freq(note);
  uint16_t startPW = (firstTuneFlag) ? (DIV_COUNTER_PW / 2) : PW_CENTER[pwCh];
  double dutyTol  = (mode == 0) ? 0.003 : 0.005; // ±0.3% low note, ±0.5% high note

  Serial.println((String)"[PW_CENTER_START] DCO=" + currentDCO + " ch=" + pwCh +
  " note=" + note + " freq=" + fmt_freq((float)freqHz) +
  " anchorAmp=" + amp + " seed=" + startPW);

  PWSearchResult res = find_pw_for_target_duty(
    pwCh,
    kPWCenterDutyFraction,
    dutyTol,
    0,
    DIV_COUNTER_PW,
    startPW,
    freqHz
  );

  if (calibrationCancelRequested) return;

  if (res.ok) {
    PW_CENTER[pwCh] = res.pw;
    update_FS_PWCenter(pwCh, res.pw);
    apply_pw_center(pwCh);

    // Save achieved duty for the PW summary table
    g_pwSummary[pwCh].pwCenter   = res.pw;
    g_pwSummary[pwCh].centerDuty = res.duty;

    Serial.println((String)"[PW_CENTER_RESULT] ch=" + pwCh + " PW_CENTER=" + res.pw +
    " duty=" + String(res.duty * 100.0, 2) + "% err=" +
    String(res.errorFrac * 100.0, 3) + "% (" + res.probes + " probes)");
  } else {
    Serial.println((String)"[PW_CENTER_FAIL] ch=" + pwCh + " keeping previous=" + PW_CENTER[pwCh]);
  }
}

// High-level PW Low/High Limit Search
void find_PW_limit_v2(PWLimitDir dir) {
  // Use 440 Hz anchor for high-speed limit detection
  uint8_t note = manual_cal_reference_note;
  DCO_calibration_current_note = note;
  VOICE_NOTES[0] = note;

  const uint8_t pwCh = cal_pw_channel(currentDCO);
  if (PW_PINS[pwCh] == PW_PIN_UNASSIGNED) return;

  DCOCalibrationStart = millis();

  // Retrieve the strong 440 Hz amplitude
  uint16_t amp = ampComp440[currentDCO];
  if (amp < 500) {
    amp = (uint16_t)(DIV_COUNTER / 4);
  }
  ampCompCalibrationVal = amp;

  // Force the hardware to the new 440Hz frequency and amplitude
  write_range_pwm(currentDCO, amp);
  voice_task_autotune(2, amp);

  double freqHz   = (double)note_to_freq(note);
  uint16_t center = PW_CENTER[pwCh];

  double   targetDuty = (dir == PW_LIMIT_LOW) ? kPWLowDutyFraction  : kPWHighDutyFraction;  // 2% or 98%
  uint16_t minPW      = (dir == PW_LIMIT_LOW) ? 0                   : center;
  uint16_t maxPW      = (dir == PW_LIMIT_LOW) ? center              : DIV_COUNTER_PW;
  uint16_t seedPW     = (dir == PW_LIMIT_LOW) ? (center * 0.15)     : (center + (DIV_COUNTER_PW - center) * 0.85);

  const char *tag = (dir == PW_LIMIT_LOW) ? "PW_LOW" : "PW_HIGH";
  Serial.println((String)"[" + tag + "_START] DCO=" + currentDCO + " ch=" + pwCh +
  " target=" + String(targetDuty * 100.0, 1) + "% anchorAmp=" + amp);

  PWSearchResult res = find_pw_for_target_duty(
    pwCh,
    targetDuty,
    kPWLimitDutyTolerance, // ±1%
    minPW,
    maxPW,
    seedPW,
    freqHz
  );

  if (calibrationCancelRequested) return;

  uint16_t finalLimitPW = res.ok ? res.pw : (dir == PW_LIMIT_LOW ? PW_LOW_LIMIT[pwCh] : PW_HIGH_LIMIT[pwCh]);

  if (res.ok) {
    if (dir == PW_LIMIT_LOW) {
      PW_LOW_LIMIT[pwCh] = res.pw;
      update_FS_PW_Low_Limit(pwCh, res.pw);
      g_pwSummary[pwCh].pwLow   = res.pw;
      g_pwSummary[pwCh].lowDuty = res.duty;
    } else {
      PW_HIGH_LIMIT[pwCh] = res.pw;
      update_FS_PW_High_Limit(pwCh, res.pw);
      g_pwSummary[pwCh].pwHigh   = res.pw;
      g_pwSummary[pwCh].highDuty = res.duty;
    }

    Serial.println((String)"[" + tag + "_RESULT] ch=" + pwCh + " LIMIT=" + res.pw +
    " duty=" + String(res.duty * 100.0, 2) + "% (" + res.probes + " probes)");
  }

  // Restore center after sweep
  apply_pw_center(pwCh);
}

// =============================================================================
// Calibration Reports & Telemetry Formatting
// =============================================================================

void cal_report_reset() {
  for (int p = 0; p < kCalReportPairs; ++p) {
    calPointDutyErrPct[p] = kCalDutyErrUnknown;
    calPointSource[p]     = CAL_SRC_NONE;
  }
  calReportLadderInterval = 0;
  calReportAnchorPair     = -1;
  calRunProbes            = 0;
  calRunStartMs           = millis();
}

void cal_report_set_pair(int pair, float dutyErrPct, uint8_t src) {
  if (pair < 0 || pair >= kCalReportPairs) return;
  calPointDutyErrPct[pair] = dutyErrPct;
  calPointSource[pair]     = src;
}

void cal_report_set_pair_from_gap(int pair, float gapUs, float freqHz, uint8_t src) {
  cal_report_set_pair(pair, duty_err_pct_from_gap(gapUs, freqHz), src);
}

// =============================================================================
// Detailed Per-Oscillator Calibration Report
// =============================================================================

static String cal_pad_left(const String& s, int width) {
  String out = s;
  while ((int)out.length() < width) {
    out = " " + out;
  }
  return out;
}

static const char *cal_point_source_name(uint8_t src) {
  switch (src) {
    case CAL_SRC_RUNG:          return "rung";
    case CAL_SRC_ANCHOR:        return "anchor";
    case CAL_SRC_ENDPOINT_FULL: return "endpoint-full";
    case CAL_SRC_ENDPOINT_AMP0: return "endpoint-amp0";
    case CAL_SRC_MANUAL:        return "manual";
    case CAL_SRC_FILLED:        return "filled";
    case CAL_SRC_SENTINEL:      return "sentinel";
    case CAL_SRC_REFINED:       return "refined";
    default:                    return "-";
  }
}

// Print the complete calibration table and metrics for one oscillator
void print_calibration_report(uint8_t dcoIndex, const uint32_t *data) {
  if (autotuneDebug < 1) return;

  String header = (String)"\n[CAL_REPORT] DCO=" + dcoIndex +
  " method=" +
  ((calibrationPrecision == CAL_PRECISION_FINE)
  ? "REFINE"
  : autotune_amp_method_name(autotuneAmpMethod)) +
  " precision=" + calibration_precision_name(calibrationPrecision) +
  " search=" + autotune_search_mode_name(autotuneSearchMode);
  if (calReportLadderInterval > 0) {
    header += (String)" ladder=" + calReportLadderInterval + " semitones";
  }
  if (calReportAnchorPair >= 0) {
    header += (String)" anchorPair=" + calReportAnchorPair;
  }
  Serial.println(header);
  Serial.println("[CAL_REPORT] pair    freqHz  ampComp  dutyErr%     gapUs    1cnt%  src");

  float    errSum      = 0.0f;
  int      errCount    = 0;
  float    worstErr    = -1.0f;
  int      worstPair   = -1;
  int      measured    = 0;
  int      highestPair = -1;

  for (int p = 0; p < kCalReportPairs; ++p) {
    const float    freqHz = (float)data[2 * p] / 100.0f;
    const uint32_t amp    = data[2 * p + 1];
    const uint8_t  src    = calPointSource[p];
    const bool     isSent = (src == CAL_SRC_SENTINEL);
    const float    err    = calPointDutyErrPct[p];
    const bool     hasErr = (fabsf(err) < 1e8f);

    if (!isSent) {
      highestPair = p;
    }
    if (src == CAL_SRC_RUNG || src == CAL_SRC_ANCHOR ||
      src == CAL_SRC_ENDPOINT_FULL || src == CAL_SRC_ENDPOINT_AMP0 ||
      src == CAL_SRC_REFINED) {
      ++measured;
      }

      String line = "[CAL_REPORT] " + cal_pad_left(String(p), 4);
    line += cal_pad_left(isSent ? String("-") : fmt_freq(freqHz), 10);
    line += cal_pad_left(String(amp), 9);

    if (hasErr) {
      const float absErr = fabsf(err);
      errSum += absErr;
      ++errCount;
      if (absErr > worstErr) {
        worstErr  = absErr;
        worstPair = p;
      }
      const float gapUs = (freqHz > 0.0f) ? (err * 20000.0f / freqHz) : 0.0f;
      line += cal_pad_left(String(err, 3), 10);
      line += cal_pad_left(String(gapUs, 2), 10);
    } else {
      line += cal_pad_left("-", 10);
      line += cal_pad_left("-", 10);
    }

    // Quantisation noise floor (1 count error at this amplitude)
    if (!isSent && amp > 0) {
      line += cal_pad_left(String(50.0f / (float)amp, 3), 9);
    } else {
      line += cal_pad_left("-", 9);
    }
    line += (String)"  " + cal_point_source_name(src);
    Serial.println(line);
  }

  const float lowestHz  = (float)data[0] / 100.0f;
  const float highestHz = (highestPair >= 0) ? ((float)data[2 * highestPair] / 100.0f) : 0.0f;
  String span = "-";
  if (lowestHz > 0.0f && highestHz > lowestHz) {
    span = String(log2f(highestHz / lowestHz), 2);
  }
  Serial.println((String)"[CAL_REPORT] DCO=" + dcoIndex +
  " lowest=" + fmt_freq(lowestHz) + " Hz highest=" + fmt_freq(highestHz) +
  " Hz span=" + span + " octaves measured=" + measured +
  "/" + kCalReportPairs);

  if (errCount > 0 && worstPair >= 0) {
    Serial.println((String)"[CAL_REPORT] DCO=" + dcoIndex +
    " dutyErr avg=" + String(errSum / (float)errCount, 3) +
    "% worst=" + String(worstErr, 3) +
    "% at pair " + worstPair +
    " (" + fmt_freq((float)data[2 * worstPair] / 100.0f) + " Hz)");
  } else {
    Serial.println((String)"[CAL_REPORT] DCO=" + dcoIndex +
    " dutyErr: no measured points");
  }

  const unsigned long elapsedMs = millis() - calRunStartMs;
  Serial.println((String)"[CAL_REPORT] DCO=" + dcoIndex +
  " search=" + autotune_search_mode_name(autotuneSearchMode) +
  " probes=" + calRunProbes +
  " elapsed=" + String(elapsedMs / 1000.0f, 1) + " s" +
  " (" + String((float)elapsedMs / (float)max(calRunProbes, 1u), 1) +
  " ms/probe)\n");
}

// =============================================================================
// Multi-Oscillator Side-by-Side Summary Report Table
// =============================================================================

static String cal_col(const String& s, int width, bool rightAlign = true) {
  String out = s;
  if ((int)out.length() > width) {
    out = out.substring(0, width);
  }
  while ((int)out.length() < width) {
    if (rightAlign) out = " " + out;
    else out = out + " ";
  }
  return out;
}

static void print_multi_osc_summary_table(uint8_t scope, uint8_t precision, uint8_t method) {
  Serial.println("\n======================================================================================================================");
  Serial.println((String)"[CAL_SUMMARY] FINAL MULTI-OSCILLATOR CALIBRATION REPORT  (Scope: " +
  calibration_scope_name(scope) + " | Precision: " +
  calibration_precision_name(precision) + " | Method: " +
  autotune_amp_method_name(method) + ")");
  Serial.println("======================================================================================================================");
  Serial.println(" DCO |  Type   | Status | PW_CTR (LOW - HIGH) | 440_AMP |  Lowest Hz  |  Highest Hz  | AvgErr% | WorstErr% (Pair @ Hz)  | Probes |  Time ");
  Serial.println("-----+---------+--------+---------------------+---------+-------------+--------------+---------+------------------------+--------+-------");

  int successCount = 0;
  int totalProbes  = 0;
  uint32_t totalTimeMs = 0;

  for (int i = 0; i < NUM_OSCILLATORS; ++i) {
    const OscCalSummary& s = g_oscSummary[i];
    totalProbes  += s.probes;
    totalTimeMs  += s.elapsedMs;

    String line = " " + cal_col(String(i), 3, true) + " | ";
    line += cal_col(s.hasPW ? "PW/VAR" : "FIXED", 7, false) + " | ";

    if (s.cancelled) {
      line += cal_col("CANCEL", 6, false) + " | ";
    } else if (!s.attempted) {
      line += cal_col("SKIP", 6, false) + " | ";
    } else if (s.ok) {
      line += cal_col("OK", 6, false) + " | ";
      successCount++;
    } else {
      line += cal_col("FAILED", 6, false) + " | ";
    }

    // PW Column
    if (s.hasPW) {
      String pwStr = String(s.pwCenter) + " (" + String(s.pwLow) + "-" + String(s.pwHigh) + ")";
      line += cal_col(pwStr, 19, false) + " | ";
    } else {
      line += cal_col("Muted (No PW)", 19, false) + " | ";
    }

    // Anchor AMP
    line += cal_col(String(s.anchorAmp), 7, true) + " | ";

    // Lowest & Highest Hz
    if (s.lowestHz > 0.0f) {
      line += cal_col(fmt_freq(s.lowestHz) + " Hz", 11, true) + " | ";
    } else {
      line += cal_col("-", 11, true) + " | ";
    }

    if (s.highestHz > 0.0f) {
      line += cal_col(fmt_freq(s.highestHz) + " Hz", 12, true) + " | ";
    } else {
      line += cal_col("-", 12, true) + " | ";
    }

    // Duty Errors with Worst Pair Breakdown
    if (s.avgDutyErrPct >= 0.0f) {
      line += cal_col(String(s.avgDutyErrPct, 3) + "%", 7, true) + " | ";
      String worstStr = String(s.worstDutyErrPct, 3) + "% (P" + String(s.worstPair) + "@" + fmt_freq(s.worstHz) + ")";
      line += cal_col(worstStr, 22, false) + " | ";
    } else {
      line += cal_col("-", 7, true) + " | ";
      line += cal_col("-", 22, false) + " | ";
    }

    // Probes & Time
    line += cal_col(String(s.probes), 6, true) + " | ";
    line += cal_col(String(s.elapsedMs / 1000.0f, 1) + "s", 5, true);

    Serial.println(line);
  }

  Serial.println("======================================================================================================================");
  Serial.println((String)"Summary: " + successCount + "/" + NUM_OSCILLATORS +
  " Oscillators Calibrated Successfully | Total Probes: " + totalProbes +
  " | Total Duration: " + String(totalTimeMs / 1000.0f, 1) + "s");
  Serial.println("======================================================================================================================\n");
}


// =============================================================================
// Dedicated PW Calibration Summary Structures & Report Table
// =============================================================================

// Print dedicated PW-only summary table
static void print_pw_summary_table() {
  Serial.println("\n======================================================================================================================");
  Serial.println("[PW_SUMMARY] FINAL PULSE-WIDTH (PW) CALIBRATION REPORT  (Reference: 440.0 Hz / Note 81)");
  Serial.println("======================================================================================================================");
  Serial.println(" Ch | Pin  | Oscs  | Status |   PW_CENTER (Duty)   |    PW_LOW (Duty)     |    PW_HIGH (Duty)    | Span | Probes | Time ");
  Serial.println("----+------+-------+--------+----------------------+----------------------+----------------------+------+--------+------");

  int successCount = 0;
  int totalProbes  = 0;
  uint32_t totalTimeMs = 0;

  int oscsPerCh = NUM_OSCILLATORS / NUM_PW_CHANNELS;

  for (int ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    const PWCalSummary& s = g_pwSummary[ch];
    totalProbes += s.probes;
    totalTimeMs += s.elapsedMs;

    if (s.pin == PW_PIN_UNASSIGNED) continue;

    String line = " " + cal_col(String(ch), 2, true) + " | ";
    line += cal_col("GP" + String(s.pin), 4, false) + " | ";

    int firstOsc = ch * oscsPerCh;
    int lastOsc  = firstOsc + oscsPerCh - 1;
    String oscStr = (firstOsc == lastOsc) ? String(firstOsc) : (String(firstOsc) + ", " + String(lastOsc));
    line += cal_col(oscStr, 5, false) + " | ";

    if (s.cancelled) {
      line += cal_col("CANCEL", 6, false) + " | ";
    } else if (!s.attempted) {
      line += cal_col("SKIP", 6, false) + " | ";
    } else if (s.ok) {
      line += cal_col("OK", 6, false) + " | ";
      successCount++;
    } else {
      line += cal_col("FAILED", 6, false) + " | ";
    }

    // Center Duty string (Target 50%)
    if (s.centerDuty >= 0.0) {
      double dev = (s.centerDuty - kPWCenterDutyFraction) * 100.0;
      String cStr = String(s.pwCenter) + " (" + String(s.centerDuty * 100.0, 2) + "%" +
      (dev >= 0.0 ? " +" : " ") + String(dev, 2) + "%)";
      line += cal_col(cStr, 20, false) + " | ";
    } else {
      line += cal_col(String(s.pwCenter), 20, false) + " | ";
    }

    // Low Duty string (Target 2%)
    if (s.lowDuty >= 0.0) {
      double dev = (s.lowDuty - kPWLowDutyFraction) * 100.0;
      String lStr = String(s.pwLow) + " (" + String(s.lowDuty * 100.0, 2) + "%" +
      (dev >= 0.0 ? " +" : " ") + String(dev, 2) + "%)";
      line += cal_col(lStr, 20, false) + " | ";
    } else {
      line += cal_col(String(s.pwLow), 20, false) + " | ";
    }

    // High Duty string (Target 98%)
    if (s.highDuty >= 0.0) {
      double dev = (s.highDuty - kPWHighDutyFraction) * 100.0;
      String hStr = String(s.pwHigh) + " (" + String(s.highDuty * 100.0, 2) + "%" +
      (dev >= 0.0 ? " +" : " ") + String(dev, 2) + "%)";
      line += cal_col(hStr, 20, false) + " | ";
    } else {
      line += cal_col(String(s.pwHigh), 20, false) + " | ";
    }

    // Usable PWM Span
    uint16_t span = (s.pwHigh > s.pwLow) ? (s.pwHigh - s.pwLow) : 0;
    line += cal_col(String(span), 4, true) + " | ";

    // Probes & Time
    line += cal_col(String(s.probes), 6, true) + " | ";
    line += cal_col(String(s.elapsedMs / 1000.0f, 1) + "s", 4, true);

    Serial.println(line);
  }

  Serial.println("======================================================================================================================");
  Serial.println((String)"Summary: " + successCount + "/" + NUM_PW_CHANNELS +
  " PW Channels Calibrated Successfully | Total Probes: " + totalProbes +
  " | Total Duration: " + String(totalTimeMs / 1000.0f, 1) + "s");
  Serial.println("======================================================================================================================\n");
}

// =============================================================================
// Diagnostic Sweeps, Probes & Manual Gap Tracking
// =============================================================================

static void cal_sense_probe_log() {
  static uint32_t lastPrintMs = 0;
  if ((millis() - lastPrintMs) < 500u) return;
  lastPrintMs = millis();

  constexpr uint32_t kWindowUs = 40000u;
  const uint32_t t0 = micros();
  bool lastRaw = digitalRead(DCO_calibration_pin);
  uint32_t edges = 0, minDt = 0xFFFFFFFFu, maxDt = 0, lastEdgeUs = t0;
  bool haveEdge = false;

  while ((micros() - t0) < kWindowUs) {
    bool raw = digitalRead(DCO_calibration_pin);
    if (raw != lastRaw) {
      uint32_t nowUs = micros();
      uint32_t dt = nowUs - lastEdgeUs;
      if (haveEdge) {
        if (dt < minDt) minDt = dt;
        if (dt > maxDt) maxDt = dt;
      }
      lastEdgeUs = nowUs; haveEdge = true; edges++; lastRaw = raw;
    }
  }

  Serial.println((String)"[CAL_SENSE] pin=" + DCO_calibration_pin + " raw=" + (int)digitalRead(DCO_calibration_pin) +
                 " edges=" + edges + " minDt=" + minDt + " maxDt=" + maxDt);
}

void run_calibration_verify_sweep() {
  const uint8_t precisionBefore = calibrationPrecision;
  calibrationPrecision = CAL_PRECISION_FINE;
  calibrationCancelRequested = false;
  calibrationFlag = true;

  disable_all_oscillators_and_range_pwm();

  for (uint8_t osc = 0; osc < NUM_OSCILLATORS && !calibrationCancelRequested; ++osc) {
    currentDCO = osc;
    restart_DCO_calibration();

    float topHz = (plateauStartFreqQ[osc] > 0)
                    ? ((float)plateauStartFreqQ[osc] / (float)(1 << FREQ_FRAC_BITS))
                    : (float)AMP_COMP_MAX_HZ;

    for (uint8_t note = manual_DCO_calibration_start_note; note < 120 && !calibrationCancelRequested; note += 3) {
      float freqHz = note_to_freq(note);
      if (freqHz > topHz) break;
      uint16_t amp = get_chan_level_for_engine(freqHz, osc);

      DCO_calibration_current_note = note;
      VOICE_NOTES[0] = note;
      float gapUs = measure_duty_at_freq(freqHz, amp, true);
      if (gapUs == kGapTimeoutSentinel) continue;

      float errPct = duty_err_pct_from_gap(gapUs, freqHz);
      Serial.println((String)"[CAL_VERIFY] DCO=" + osc + " note=" + note + " freq=" + fmt_freq(freqHz) +
                     " amp=" + amp + " dutyErr=" + String(errPct, 3) + "%");
    }
  }

  disable_all_oscillators_and_range_pwm();
  calibrationFlag = false;
  calibrationPrecision = precisionBefore;
  restore_voice_engine_after_calibration();
}

static const uint16_t kPWProbeLevels[] = {
  0, DIV_COUNTER_PW / 4, DIV_COUNTER_PW / 2, (DIV_COUNTER_PW * 3) / 4, DIV_COUNTER_PW - 1
};

void run_pw_cv_probe() {
  const uint8_t osc      = cal_manual_osc();
  const uint8_t expectCh = cal_pw_channel(osc);
  const double  freqHz   = (double)note_to_freq(DCO_calibration_current_note);
  const double  periodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;

  if (periodUs <= 0.0) return;

  uint8_t bestCh = 0;
  float bestSpan = -1.0f, expectSpan = 0.0f;
  bool anyRead = false;

  for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    if (PW_PINS[ch] == PW_PIN_UNASSIGNED) continue;

    for (uint8_t z = 0; z < NUM_PW_CHANNELS; ++z) {
      if (z != ch && PW_PINS[z] != PW_PIN_UNASSIGNED) {
        pwm_set_chan_level(PW_PWM_SLICES[z], pwm_gpio_to_channel(PW_PINS[z]), 0);
        PW[z] = 0;
      }
    }

    float dutyMin = 0.0f, dutyMax = 0.0f;
    uint8_t reads = 0;
    const uint8_t levels = (uint8_t)(sizeof(kPWProbeLevels) / sizeof(kPWProbeLevels[0]));

    for (uint8_t li = 0; li < levels && !calibrationCancelRequested; ++li) {
      GapMeasurement gm = set_pw_and_measure(ch, kPWProbeLevels[li]);
      if (gm.timedOut) continue;
      float dutyPct = (float)((0.5 + (double)gm.value / (2.0 * periodUs)) * 100.0);
      if (reads == 0 || dutyPct < dutyMin) dutyMin = dutyPct;
      if (reads == 0 || dutyPct > dutyMax) dutyMax = dutyPct;
      ++reads; anyRead = true;
    }

    float span = (reads > 0) ? (dutyMax - dutyMin) : 0.0f;
    if (ch == expectCh) expectSpan = span;
    if (span > bestSpan) { bestSpan = span; bestCh = ch; }
    if (calibrationCancelRequested) break;
  }

  apply_pw_center(expectCh);
}

void DCO_calibration_debug() {
  GapMeasurement gm = measure_gap(0);
  uint8_t reportDCO = cal_manual_osc();
  int32_t dutyErrorPercentTimes100 = 0;

  if (!gm.timedOut) {
    double freqHz = (double)note_to_freq(DCO_calibration_current_note);
    if (freqHz > 0.0) {
      double periodUs = 1000000.0 / freqHz;
      double gapUs = (double)gm.value - (double)duty_trim_gap_us(reportDCO, (float)freqHz);
      double dutyErrorPercent = (gapUs / (2.0 * periodUs)) * 100.0;
      dutyErrorPercentTimes100 = (int32_t)(dutyErrorPercent * 100.0);
    }
  } else {
    dutyErrorPercentTimes100 = kManualGapTimeoutDutyErrTimes100;
  }

  if (autotuneDebug >= 1 && gm.timedOut) cal_sense_probe_log();
  serialSendParam32(PARAM_GAP_FROM_DCO, (uint32_t)dutyErrorPercentTimes100, true);
}


// =============================================================================
// Master Auto-Calibration Entry Point
// =============================================================================

void DCO_calibration() {
  autotune_fill_init_manual_amp();
  calibrationCancelRequested = false;

  const uint8_t scope     = calibrationScope;
  const bool    runPW     = calibration_scope_runs_pw(scope);
  const bool    runAmp    = calibration_scope_runs_amp(scope);
  const bool    fine      = (calibrationPrecision == CAL_PRECISION_FINE);

  // Initialize PW Summary tracking
  for (int ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    g_pwSummary[ch].attempted  = false;
    g_pwSummary[ch].ok         = false;
    g_pwSummary[ch].cancelled  = false;
    g_pwSummary[ch].ch         = ch;
    g_pwSummary[ch].pin        = PW_PINS[ch];
    g_pwSummary[ch].pwCenter   = PW_CENTER[ch];
    g_pwSummary[ch].centerDuty = -1.0;
    g_pwSummary[ch].pwLow      = PW_LOW_LIMIT[ch];
    g_pwSummary[ch].lowDuty    = -1.0;
    g_pwSummary[ch].pwHigh     = PW_HIGH_LIMIT[ch];
    g_pwSummary[ch].highDuty   = -1.0;
    g_pwSummary[ch].probes     = 0;
    g_pwSummary[ch].elapsedMs  = 0;
  }

  // Initialize Amp/Full Summary tracking
  for (int i = 0; i < NUM_OSCILLATORS; ++i) {
    uint8_t ch = cal_pw_channel(i);
    g_oscSummary[i].attempted       = false;
    g_oscSummary[i].ok              = false;
    g_oscSummary[i].cancelled       = false;
    g_oscSummary[i].pwCh            = ch;
    g_oscSummary[i].hasPW           = osc_has_pw(i);
    g_oscSummary[i].pwCenter        = PW_CENTER[ch];
    g_oscSummary[i].pwLow           = PW_LOW_LIMIT[ch];
    g_oscSummary[i].pwHigh          = PW_HIGH_LIMIT[ch];
    g_oscSummary[i].anchorAmp       = ampComp440[i];
    g_oscSummary[i].lowestHz        = 0.0f;
    g_oscSummary[i].highestHz       = 0.0f;
    g_oscSummary[i].avgDutyErrPct   = -1.0f;
    g_oscSummary[i].worstDutyErrPct = -1.0f;
    g_oscSummary[i].worstPair       = -1;
    g_oscSummary[i].worstHz         = 0.0f;
    g_oscSummary[i].probes          = 0;
    g_oscSummary[i].elapsedMs       = 0;
  }

  Serial.println("\n=======================================================");
  Serial.println((String)"[DCO_CAL] *** CALIBRATION STARTED *** Scope: " + calibration_scope_name(scope));
  Serial.println((String)"[DCO_CAL] Precision: " + calibration_precision_name(calibrationPrecision) +
  " | Method: " + autotune_amp_method_name(autotuneAmpMethod));

  Serial.println("\n--- [RECALLED STORED PARAMETERS] ---");
  for (int i = 0; i < NUM_OSCILLATORS; ++i) {
    uint8_t ch = cal_pw_channel(i);
    Serial.println((String)"  DCO " + i + ": ampComp440=" + ampComp440[i] +
    " | manualOffset=" + manualCalibrationOffset[i] +
    " | dutyOffset=" + ampCompDutyOffset[i] +
    " | PW_CENTER[ch " + ch + "]=" + PW_CENTER[ch] +
    " | PW_LOW=" + PW_LOW_LIMIT[ch] + " | PW_HIGH=" + PW_HIGH_LIMIT[ch]);
  }
  Serial.println("=======================================================\n");

  // Global analog drain phase
  Serial.println("[DCO_CAL] ---> DRAINING ANALOG BUS (Waiting for RC filters to reach 0V)...");
  disable_all_oscillators_and_range_pwm();
  delay(2500);
  Serial.println("[DCO_CAL] ---> Analog bus stabilized. Beginning individual DCO tuning.\n");

  bool anyTableUpdated = false;
  bool allSucceeded    = true;

  // ---------------------------------------------------------------------------
  // 1. PW Calibration Stage
  // ---------------------------------------------------------------------------
  if (runPW && !calibrationCancelRequested) {
    Serial.println("\n[DCO_CAL] ---> Starting PW Calibration Stage");
    uint8_t lastCh = 0xFF;
    for (uint8_t osc = 0; osc < NUM_OSCILLATORS; ++osc) {
      if (calibrationCancelRequested) break;
      if (!osc_has_pw(osc)) continue;

      const uint8_t ch = cal_pw_channel(osc);
      if (ch == lastCh || PW_PINS[ch] == PW_PIN_UNASSIGNED) continue;
      lastCh = ch;
      currentDCO = osc;

      g_pwSummary[ch].attempted = true;
      uint32_t pwChStartMs = millis();
      int probesBefore = calRunProbes;

      Serial.println((String)"\n[DCO_CAL] Calibrating PW for Channel " + ch + " (Osc " + osc + ")");
      restart_DCO_calibration();
      DCO_calibration_current_note = manual_cal_reference_note;
      VOICE_NOTES[0] = DCO_calibration_current_note;

      find_PW_center(0);
      if (!calibrationCancelRequested) find_PW_limit_v2(PW_LIMIT_LOW);
      if (!calibrationCancelRequested) find_PW_limit_v2(PW_LIMIT_HIGH);

      g_pwSummary[ch].pwCenter  = PW_CENTER[ch];
      g_pwSummary[ch].pwLow     = PW_LOW_LIMIT[ch];
      g_pwSummary[ch].pwHigh    = PW_HIGH_LIMIT[ch];
      g_pwSummary[ch].probes    = calRunProbes - probesBefore;
      g_pwSummary[ch].elapsedMs = millis() - pwChStartMs;

      if (calibrationCancelRequested) {
        g_pwSummary[ch].cancelled = true;
        allSucceeded = false;
        break;
      }

      g_pwSummary[ch].ok = true;
    }
  }

  // ---------------------------------------------------------------------------
  // 2. Amp Compensation Calibration Stage
  // ---------------------------------------------------------------------------
  if (runAmp && !calibrationCancelRequested) {
    Serial.println("\n[DCO_CAL] ---> Starting Amp Compensation Stage");
    for (int i = 0; i < NUM_OSCILLATORS; i++) {
      if (calibrationCancelRequested) {
        g_oscSummary[i].cancelled = true;
        allSucceeded = false;
        break;
      }

      currentDCO = i;
      g_oscSummary[i].attempted = true;

      Serial.println((String)"\n[DCO_CAL] === Calibrating Amplitude for DCO " + i + " ===");
      restart_DCO_calibration();

      DCOCalibrationContext ctx(
        currentDCO, DCO_calibration_current_note, calibrationData,
        manualCalibrationOffset, initManualAmpCompCalibrationVal
      );

      bool tableOk = true;
      cal_report_reset();

      if (fine) {
        tableOk = refine_DCO_amp_table(ctx);
      } else if (autotuneAmpMethod == AMP_METHOD_FREQ_TRACE) {
        tableOk = calibrate_DCO_freq_trace(ctx);
      } else {
        cal_report_set_pair(0, kCalDutyErrUnknown, CAL_SRC_FILLED);
        cal_report_set_pair(1, kCalDutyErrUnknown, CAL_SRC_MANUAL);
        calibrate_DCO(ctx, 0.001);
      }

      if (!fine && autotuneAmpMethod != AMP_METHOD_FREQ_TRACE && !calibrationCancelRequested && tableOk) {
        if (autotuneAmp0Mode == AMP0_MODE_CALC || calibrationPrecision == CAL_PRECISION_FAST) {
          // Skip live hunt
        } else {
          apply_measured_lowest_freq(ctx);
        }
      }

      // Compute statistics for summary table
      float errSum = 0.0f, worstErr = 0.0f, worstHz = 0.0f;
      int   errCount = 0, highestPair = 0, worstPair = -1;

      for (int p = 0; p < kCalReportPairs; ++p) {
        if (calPointSource[p] != CAL_SRC_SENTINEL && calibrationData[2 * p] > 0) {
          highestPair = p;
        }
        float err = calPointDutyErrPct[p];
        if (fabsf(err) < 1e8f) {
          float absErr = fabsf(err);
          errSum += absErr;
          errCount++;
          if (absErr > worstErr) {
            worstErr  = absErr;
            worstPair = p;
            worstHz   = (float)calibrationData[2 * p] / 100.0f;
          }
        }
      }

      g_oscSummary[i].lowestHz        = (float)calibrationData[0] / 100.0f;
      g_oscSummary[i].highestHz       = (float)calibrationData[2 * highestPair] / 100.0f;
      g_oscSummary[i].avgDutyErrPct   = (errCount > 0) ? (errSum / (float)errCount) : 0.0f;
      g_oscSummary[i].worstDutyErrPct = worstErr;
      g_oscSummary[i].worstPair       = worstPair;
      g_oscSummary[i].worstHz         = worstHz;
      g_oscSummary[i].probes          = calRunProbes;
      g_oscSummary[i].elapsedMs       = millis() - calRunStartMs;
      g_oscSummary[i].anchorAmp       = ampComp440[i];

      if (calibrationCancelRequested) {
        g_oscSummary[i].cancelled = true;
        g_oscSummary[i].ok        = false;
        allSucceeded = false;
        Serial.println((String)"[DCO_CAL] DCO=" + currentDCO + " cancelled mid-run. Discarding table.");
        break;
      }

      if (tableOk) {
        g_oscSummary[i].ok = true;
        update_FS_voice(currentDCO);
        anyTableUpdated = true;
        Serial.println((String)"[DCO_CAL] DCO " + currentDCO + " table successfully saved to filesystem.");
      } else {
        g_oscSummary[i].ok = false;
        allSucceeded = false;
        Serial.println((String)"[DCO_CAL_ERROR] DCO " + currentDCO + " calibration failed! Keeping previous calibration.");
      }

      print_calibration_report(currentDCO, calibrationData);
    }
  }

  // ---------------------------------------------------------------------------
  // 3. Final Summary Report Selection
  // ---------------------------------------------------------------------------
  if (scope == CAL_SCOPE_PW) {
    print_pw_summary_table(); // Specific report for PW-only calibration
  } else if (scope == CAL_SCOPE_AMP) {
    print_multi_osc_summary_table(scope, calibrationPrecision, autotuneAmpMethod); // Amp report
  } else {
    // FULL run: print both PW and Multi-Oscillator reports
    print_pw_summary_table();
    print_multi_osc_summary_table(scope, calibrationPrecision, autotuneAmpMethod);
  }

  if (calibrationCancelRequested) {
    Serial.println("\n=======================================================");
    Serial.println("[DCO_CAL] *** CALIBRATION CANCELLED / STOPPED BY USER ***");
    Serial.println("=======================================================\n");
  } else if (!allSucceeded) {
    Serial.println("\n=======================================================");
    Serial.println("[DCO_CAL] *** CALIBRATION FINISHED WITH ERRORS ***");
    Serial.println("=======================================================\n");
  } else {
    Serial.println("\n=======================================================");
    Serial.println("[DCO_CAL] *** CALIBRATION FINISHED SUCCESSFULLY ***");
    Serial.println("=======================================================\n");
  }

  if (anyTableUpdated) {
    init_FS();
    precompute_amp_comp_for_engine();
  }

  calibrationFlag            = false;
  calibrationCancelRequested = false;
  restore_voice_engine_after_calibration();

  // Release Motherboard from Calibration Mode
  serialSendParam32(PARAM_CALIBRATION_FLAG, 0, true);
  serialSendParam32(PARAM_GAP_FROM_DCO, 0, true);

  Serial.println("[DCO_CAL] Rebooting Pi Pico for clean polyphonic engine startup...");
  Serial.flush();

  // 2. Short delay to allow Serial/UART DMA to finish transmitting packets
  delay(1000);

  if (allSucceeded) {
    // 3. Trigger clean hardware warm-reboot of both RP2040 cores
    watchdog_reboot(0, 0, 0);
    while (true) {
      tight_loop_contents();
    }
  }
}

#endif  // __AUTOTUNE_IMPL_H__
