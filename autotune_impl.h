#ifndef __AUTOTUNE_IMPL_H__
#define __AUTOTUNE_IMPL_H__

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

static constexpr uint16_t initManualAmpCompCalibrationValPreset = (uint16_t)(35u * DIV_COUNTER / 14000u);
uint16_t initManualAmpCompCalibrationVal[NUM_OSCILLATORS];
volatile uint16_t ampCompLowestFreqVal = (uint16_t)(10u * DIV_COUNTER / 14000u);
uint8_t DCO_calibration_current_note = DCO_calibration_start_note;
byte autotuneDebug = 1;

static double g_gapLogCurrentPeriodUs = 0.0;
static double g_gapLogTargetDutyFraction = 0.5;

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
  pwm_set_chan_level(PW_PWM_SLICES[ch], pwm_gpio_to_channel(PW_PINS[ch]), PW_CENTER[ch]);
  PW[ch] = PW_CENTER[ch];
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

static uint16_t pw_level_readback(uint8_t ch) {
  if (ch >= NUM_PW_CHANNELS || PW_PINS[ch] == PW_PIN_UNASSIGNED) return 0;
  const uint32_t cc = pwm_hw->slice[PW_PWM_SLICES[ch]].cc;
  return (pwm_gpio_to_channel(PW_PINS[ch]) == PWM_CHAN_A) ? (uint16_t)(cc & 0xFFFFu) : (uint16_t)(cc >> 16);
}

static void reset_pw_to_DIV_COUNTER_PW() {
  for (int i = 0; i < NUM_PW_CHANNELS; i++) {
    if (PW_PINS[i] == PW_PIN_UNASSIGNED) continue;
    pwm_set_chan_level(PW_PWM_SLICES[i], pwm_gpio_to_channel(PW_PINS[i]), DIV_COUNTER_PW);
  }
}

static void disable_all_oscillators_and_range_pwm() {
  for (int i = 0; i < NUM_OSCILLATORS; i++) {
    PIO     pioN = pio[VOICE_TO_PIO[i]];
    uint8_t smN  = VOICE_TO_SM[i];

    pio_sm_set_enabled(pioN, smN, true);
    pio_sm_put(pioN, smN, 200);
    pio_sm_exec(pioN, smN, pio_encode_pull(false, false));
    delay(200);

    pio_sm_set_enabled(pioN, smN, false);
#ifdef RANGE0_PIO_DITHER_TEST
    range_pio_set_level((uint8_t)i, DIV_COUNTER);
    continue;
#endif
    gpio_init(RANGE_PINS[i]);
    gpio_set_dir(RANGE_PINS[i], GPIO_OUT);
    gpio_put(RANGE_PINS[i], 1);
  }
  reset_pw_to_DIV_COUNTER_PW();
  g_lastDrivenFreqHz = 0.0f;
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

void restart_DCO_calibration() {
  autotune_fill_init_manual_amp();

  VOICE_NOTES[0] = DCO_calibration_start_note;
  DCO_calibration_current_note = DCO_calibration_start_note;

  calibrationData[0] = 0;
  calibrationData[1] = ampCompLowestFreqVal;
  calibrationData[2] = (uint32_t)(note_to_freq(DCO_calibration_current_note - calibration_note_interval) * 100);
  calibrationData[3] = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];

  DCOCalibrationStart = millis();
  disable_all_oscillators_and_range_pwm();

#ifndef RANGE0_PIO_DITHER_TEST
  gpio_set_function(RANGE_PINS[currentDCO], GPIO_FUNC_PWM);
#endif

  PIO pioN = pio[VOICE_TO_PIO[currentDCO]];
  uint8_t sm1N = VOICE_TO_SM[currentDCO];
  pio_sm_set_enabled(pioN, sm1N, true);

  apply_pw_center_solo(cal_pw_channel(currentDCO));
  g_lastDrivenFreqHz = 0.0f;
  delay(100);
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

  unsigned long lastEdgeTime   = micros();
  uint32_t      lastEdgeCycles = rp2040.getCycleCount();
  uint8_t       pollTick       = 0;

  while (acceptedSamples < samplesTarget) {
    const bool rawVal = (bool)gpio_get(DCO_calibration_pin);
    const bool val    = kGapPolarityInverted ? !rawVal : rawVal;

    if (val == lastVal && ((++pollTick & 0x3F) != 0)) continue;

    const uint32_t      nowCycles = rp2040.getCycleCount();
    const unsigned long nowUs     = micros();

    if ((nowUs - lastEdgeTime) > timeoutUs) return kGapTimeoutSentinel;

    if (val != lastVal) {
      const uint32_t fine24     = (nowCycles - lastEdgeCycles) & 0x00FFFFFFu;
      const uint64_t wallCycles = (uint64_t)(nowUs - lastEdgeTime) * cyclesPerUs;
      const int64_t  lostWraps  = ((int64_t)wallCycles - (int64_t)fine24 + (int64_t)(1u << 23)) >> 24;
      const uint32_t dtCycles   = (lostWraps > 0) ? (fine24 + (uint32_t)((uint64_t)lostWraps << 24)) : fine24;

      if (dtCycles >= debounceCycles) {
        lastVal = val;
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
          }
        }
        lastEdgeTime   = nowUs;
        lastEdgeCycles = nowCycles;
        pulseCount++;
      }
    }
  }

  if (specialMode == 3 && (risingCount == 0 || fallingCount == 0)) return kGapTimeoutSentinel;

  float avgLowUs  = (fallingCount > 0) ? (float)((double)fallingSumCycles * usPerCycle / (double)fallingCount) : 0.0f;
  float avgHighUs = (risingCount > 0)  ? (float)((double)risingSumCycles * usPerCycle / (double)risingCount)  : 0.0f;
  float measuredPeriodUs = avgLowUs + avgHighUs;

  if (specialMode == 3 && idealPeriodUs > 0.0 && fabsf(measuredPeriodUs - (float)idealPeriodUs) > kGapPeriodTolRatio * (float)idealPeriodUs) {
    return kGapTimeoutSentinel;
  }

  return avgHighUs - avgLowUs;
}

// =============================================================================
// Pulse-Width (PW) Calibration Subsystem
// =============================================================================

static GapMeasurement set_pw_and_measure(uint8_t pwCh, uint16_t pw) {
  pwm_set_chan_level(PW_PWM_SLICES[pwCh], pwm_gpio_to_channel(PW_PINS[pwCh]), pw);
  PW[pwCh] = pw;
  delay(30);
  return measure_gap(2);
}

static void pw_search_state_init(PWSearchState& st) {
  st.validCount = 0; st.inToleranceCount = 0; st.haveBest = false;
  st.bestGapAbs = 1e12; st.bestPW = 0; st.haveBracket = false;
  st.pwLow = 0; st.pwHigh = 0; st.gapLow = 0.0;
}

static void pw_record_sample(PWSearchState& st, uint16_t pw, double gapDiff, double targetGap, PWRecordMode mode) {
  double absGapDiff = fabs(gapDiff);
  if (absGapDiff <= targetGap) st.inToleranceCount++;
  if (!st.haveBest || absGapDiff < st.bestGapAbs) {
    st.haveBest   = true;
    st.bestGapAbs = absGapDiff;
    st.bestPW     = pw;
  }
  if (mode == PW_RECORD_NO_TABLE) return;

  if (st.validCount < kPWMaxSamples) {
    st.validPW[st.validCount]      = pw;
    st.validGapDiff[st.validCount] = gapDiff;
    st.validCount++;
  } else if (mode == PW_RECORD_REPLACE_WORST) {
    int worstIdx = 0;
    double worstAbs = fabs(st.validGapDiff[0]);
    for (int vi = 1; vi < st.validCount; ++vi) {
      double curAbs = fabs(st.validGapDiff[vi]);
      if (curAbs > worstAbs) { worstAbs = curAbs; worstIdx = vi; }
    }
    if (absGapDiff < worstAbs) {
      st.validPW[worstIdx]      = pw;
      st.validGapDiff[worstIdx] = gapDiff;
    }
  }
}

static void pw_coarse_scan(PWSearchState& st, double gapTarget, double targetGap, uint16_t pwMin, uint16_t pwMax,
                           uint16_t coarseStep, double periodUs, double toleranceDutyPercent) {
  bool     havePrev = false;
  double   prevGapDiff = 0.0;
  uint16_t prevPW = 0;

  for (uint16_t pw = pwMin; pw <= pwMax; pw = (uint16_t)(pw + coarseStep)) {
    if (calibrationCancelRequested || (millis() - DCOCalibrationStart > 60000)) break;

    GapMeasurement gm = set_pw_and_measure(cal_pw_channel(currentDCO), pw);
    if (gm.timedOut) continue;

    double gapDiff = (double)gm.value - gapTarget;
    pw_record_sample(st, pw, gapDiff, targetGap, PW_RECORD_REPLACE_WORST);

    if (havePrev && ((gapDiff > 0.0 && prevGapDiff < 0.0) || (gapDiff < 0.0 && prevGapDiff > 0.0))) {
      st.haveBracket = true;
      st.pwLow  = prevPW;
      st.gapLow = prevGapDiff + gapTarget;
      st.pwHigh = pw;

      double denom = fabs(prevGapDiff) + fabs(gapDiff);
      if (denom > 0.0) {
        double t = fabs(prevGapDiff) / denom;
        uint16_t pwEst = (uint16_t)((double)prevPW + ((double)(pw - prevPW) * t));
        if (pwEst >= pwMin && pwEst <= pwMax) {
          GapMeasurement gmEst = set_pw_and_measure(cal_pw_channel(currentDCO), pwEst);
          if (!gmEst.timedOut) {
            pw_record_sample(st, pwEst, (double)gmEst.value - gapTarget, targetGap, PW_RECORD_APPEND);
          }
        }
      }
      break;
    }
    havePrev = true; prevGapDiff = gapDiff; prevPW = pw;
  }
}

static void pw_bisect_bracket(PWSearchState& st, double gapTarget, double targetGap, double periodUs, double toleranceDutyPercent) {
  uint16_t pwLow  = st.pwLow, pwHigh = st.pwHigh;
  double   gapLow = st.gapLow;

  for (int iter = 0; iter < 14; ++iter) {
    if (calibrationCancelRequested || (millis() - DCOCalibrationStart > 60000)) break;

    uint16_t pwMid = (uint16_t)((pwLow + pwHigh) / 2);
    GapMeasurement gm = set_pw_and_measure(cal_pw_channel(currentDCO), pwMid);
    if (gm.timedOut) continue;

    double gapMid = (double)gm.value;
    double gapDiffMid = gapMid - gapTarget;
    pw_record_sample(st, pwMid, gapDiffMid, targetGap, PW_RECORD_NO_TABLE);

    if ((gapDiffMid > 0.0 && (gapLow - gapTarget) > 0.0) || (gapDiffMid < 0.0 && (gapLow - gapTarget) < 0.0)) {
      pwLow  = pwMid; gapLow = gapMid;
    } else {
      pwHigh = pwMid;
    }
    if (pwHigh - pwLow <= 1) break;
  }
}

static void pw_fine_scan_around_best(PWSearchState& st, double gapTarget, double targetGap, uint16_t pwMin, uint16_t pwMax, uint16_t coarseStep) {
  uint16_t startPW = (st.haveBest && st.bestPW >= pwMin && st.bestPW <= pwMax) ? st.bestPW : (uint16_t)((pwMin + pwMax) / 2);
  uint16_t span = (coarseStep > 0) ? coarseStep * 2 : 4;
  uint16_t fineMin = (startPW > span) ? (startPW - span) : pwMin;
  uint16_t fineMax = (startPW + span < pwMax) ? (startPW + span) : pwMax;
  uint16_t fineStep = (fineMax > fineMin) ? ((fineMax - fineMin) / 16) : 1;
  if (fineStep == 0) fineStep = 1;

  for (uint16_t pw = fineMin; pw <= fineMax; pw = (uint16_t)(pw + fineStep)) {
    if (calibrationCancelRequested || (millis() - DCOCalibrationStart > 60000)) break;
    GapMeasurement gm = set_pw_and_measure(cal_pw_channel(currentDCO), pw);
    if (gm.timedOut) continue;
    pw_record_sample(st, pw, (double)gm.value - gapTarget, targetGap, PW_RECORD_APPEND);
  }
}

static bool pw_lock_in(uint8_t voiceIdx, uint16_t pw, double gapTarget, double targetGap, double periodUs, double& lockedGapOut) {
  int consecutiveOk = 0;
  for (int li = 0; li < 8; ++li) {
    if (calibrationCancelRequested) return false;
    GapMeasurement gm = set_pw_and_measure(voiceIdx, pw);
    if (gm.timedOut || periodUs <= 0.0) { consecutiveOk = 0; continue; }

    double gap = (double)gm.value;
    if (fabs(gap - gapTarget) <= targetGap) {
      if (++consecutiveOk >= 3) { lockedGapOut = gap; return true; }
    } else {
      consecutiveOk = 0;
    }
  }
  return false;
}

static bool pw_select_and_lock(PWSearchState& st, double gapTarget, double targetGap, uint16_t pwMin, uint16_t pwMax,
                               double periodUs, uint16_t& chosenPWOut) {
  for (int attempt = 0; attempt < st.validCount; ++attempt) {
    if (calibrationCancelRequested) return false;
    int bestIdx = -1;
    double bestAbs = 1e12;

    for (int vi = 0; vi < st.validCount; ++vi) {
      double curAbs = fabs(st.validGapDiff[vi]);
      if (curAbs < bestAbs) { bestAbs = curAbs; bestIdx = vi; }
    }
    if (bestIdx < 0 || bestAbs > targetGap * 10.0) return false;

    uint16_t chosenPW = st.validPW[bestIdx];
    double   lockedGap = 0.0;

    if (pw_lock_in(cal_pw_channel(currentDCO), chosenPW, gapTarget, targetGap, periodUs, lockedGap)) {
      uint16_t bestLocalPW     = chosenPW;
      double   bestLocalGapAbs = bestAbs;

      for (int16_t off = -2; off <= 2; ++off) {
        int32_t testPW32 = (int32_t)chosenPW + off;
        if (testPW32 < (int32_t)pwMin || testPW32 > (int32_t)pwMax) continue;
        uint16_t testPW = (uint16_t)testPW32;
        double gapLocal = 0.0;
        if (pw_lock_in(cal_pw_channel(currentDCO), testPW, gapTarget, targetGap, periodUs, gapLocal)) {
          double absDiff = fabs(gapLocal - gapTarget);
          if (absDiff < bestLocalGapAbs) {
            bestLocalGapAbs = absDiff;
            bestLocalPW     = testPW;
          }
        }
      }
      chosenPWOut = bestLocalPW;
      return true;
    }
    st.validGapDiff[bestIdx] = targetGap * 20.0;
  }
  return false;
}

static uint16_t find_PW_for_target_duty(double targetDutyFraction, uint16_t targetGap, uint16_t pwMin, uint16_t pwMax, uint16_t fallbackPW) {
  double freqHz   = (double)note_to_freq(DCO_calibration_current_note);
  double periodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;

  g_gapLogCurrentPeriodUs    = periodUs;
  g_gapLogTargetDutyFraction = targetDutyFraction;

  double toleranceDutyPercent = 0.0, gapTarget = 0.0;
  if (periodUs > 0.0) {
    gapTarget = periodUs * (2.0 * targetDutyFraction - 1.0);
    toleranceDutyPercent = ((double)targetGap / (2.0 * periodUs)) * 100.0;
  }

  uint16_t coarseDiv  = (fabs(targetDutyFraction - 0.5) < 0.05) ? 16 : 32;
  uint16_t coarseStep = (pwMax > pwMin) ? ((pwMax - pwMin) / coarseDiv) : 1;
  if (coarseStep == 0) coarseStep = 1;

  PWSearchState st;
  pw_search_state_init(st);

  pw_coarse_scan(st, gapTarget, (double)targetGap, pwMin, pwMax, coarseStep, periodUs, toleranceDutyPercent);
  if (st.haveBracket) {
    pw_bisect_bracket(st, gapTarget, (double)targetGap, periodUs, toleranceDutyPercent);
  } else {
    pw_fine_scan_around_best(st, gapTarget, (double)targetGap, pwMin, pwMax, coarseStep);
  }

  if (calibrationCancelRequested || st.validCount == 0) return fallbackPW;

  uint16_t chosenPW = fallbackPW;
  if (pw_select_and_lock(st, gapTarget, (double)targetGap, pwMin, pwMax, periodUs, chosenPW)) {
    return chosenPW;
  }
  return fallbackPW;
}

void find_PW_center(uint8_t mode) {
  DCO_calibration_current_note = manual_DCO_calibration_start_note;
  VOICE_NOTES[0] = DCO_calibration_current_note;
  ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];

  uint16_t targetGap;
  uint8_t voiceTaskMode;

  if (mode == 0) {
    targetGap = compute_gap_tolerance_for_freq(note_to_freq(DCO_calibration_current_note), 0.005);
    voiceTaskMode = 2;
  } else {
    DCO_calibration_current_note = 76;
    VOICE_NOTES[0] = DCO_calibration_current_note;
    targetGap = 5;
    voiceTaskMode = 3;
  }

  DCOCalibrationStart = millis();
  const uint8_t pwCh = cal_pw_channel(currentDCO);
  if (PW_PINS[pwCh] == PW_PIN_UNASSIGNED) return;

  uint16_t startPW = (firstTuneFlag == true) ? (DIV_COUNTER_PW / 2) : PW_CENTER[pwCh];
  PW[pwCh] = startPW;

  pwm_set_chan_level(PW_PWM_SLICES[pwCh], pwm_gpio_to_channel(PW_PINS[pwCh]), startPW);
  voice_task_autotune(voiceTaskMode, ampCompCalibrationVal);

  uint16_t centerPW = find_PW_for_target_duty(kPWCenterDutyFraction, targetGap, 0, DIV_COUNTER_PW, startPW);
  if (calibrationCancelRequested) return;

  update_FS_PWCenter(pwCh, centerPW);
  PW_CENTER[pwCh] = centerPW;
  apply_pw_center(pwCh);
}

PWLimitSearchResult search_PW_limit_from_center(uint8_t voiceIdx, uint16_t centerPW, PWLimitDir dir, double periodUs, double targetDuty) {
  PWLimitSearchResult result = { false, centerPW, -1.0 };
  if (periodUs <= 0.0) return result;

  uint16_t minPW = (dir == PW_LIMIT_LOW) ? 0 : centerPW;
  uint16_t maxPW = (dir == PW_LIMIT_LOW) ? centerPW : DIV_COUNTER_PW;
  uint16_t step  = DIV_COUNTER_PW / 64;
  if (step == 0) step = 1;

  bool     haveBest = false;
  uint16_t bestPW   = centerPW;
  double   bestDelta = 1e12, bestDuty = -1.0;
  unsigned long searchStartMs = millis();

  for (uint16_t pw = centerPW; ; ) {
    if (calibrationCancelRequested || (millis() - searchStartMs > 60000UL)) break;
    if (pw < minPW) pw = minPW;
    if (pw > maxPW) pw = maxPW;

    GapMeasurement gm = set_pw_and_measure(voiceIdx, pw);
    if (!gm.timedOut) {
      double duty  = 0.5 + ((double)gm.value / (2.0 * periodUs));
      double delta = fabs(duty - targetDuty);
      if (!haveBest || delta < bestDelta) {
        haveBest = true; bestDelta = delta; bestPW = pw; bestDuty = duty;
      }
      if (delta <= kPWLimitDutyTolerance) break;
    }

    if (dir == PW_LIMIT_LOW) {
      if (pw <= minPW + step) break;
      pw = (uint16_t)(pw - step);
    } else {
      if (pw >= maxPW - step) break;
      pw = (uint16_t)(pw + step);
    }
  }

  if (!haveBest) return result;

  uint16_t refineRadius = step / 2;
  if (refineRadius < 4)  refineRadius = 4;
  if (refineRadius > 32) refineRadius = 32;

  uint16_t startPW = (bestPW > refineRadius) ? (bestPW - refineRadius) : minPW;
  if (startPW < minPW) startPW = minPW;
  uint16_t endPW = bestPW + refineRadius;
  if (endPW > maxPW) endPW = maxPW;

  int consecutiveTimeouts = 0;
  for (uint16_t pw = startPW; pw <= endPW; ++pw) {
    if (calibrationCancelRequested) break;
    GapMeasurement gm = set_pw_and_measure(voiceIdx, pw);
    if (gm.timedOut) {
      if (++consecutiveTimeouts >= 4) break;
      continue;
    }
    consecutiveTimeouts = 0;
    double duty  = 0.5 + ((double)gm.value / (2.0 * periodUs));
    double delta = fabs(duty - targetDuty);
    if (delta < bestDelta) {
      bestDelta = delta; bestPW = pw; bestDuty = duty;
    }
  }

  result.ok = true;
  result.limitPW = bestPW;
  if (bestDuty >= 0.0) result.finalDutyPercent = bestDuty * 100.0;

  double currentDutyFrac = result.finalDutyPercent / 100.0;
  if (result.finalDutyPercent <= 0.0 || fabs(currentDutyFrac - targetDuty) > kPWLimitDutyTolerance) {
    uint16_t boundaryPW = (dir == PW_LIMIT_LOW) ? minPW : maxPW;
    GapMeasurement gmEdge = set_pw_and_measure(voiceIdx, boundaryPW);
    if (!gmEdge.timedOut) {
      result.limitPW        = boundaryPW;
      result.finalDutyPercent = (0.5 + ((double)gmEdge.value / (2.0 * periodUs))) * 100.0;
    } else {
      result.limitPW = boundaryPW;
    }
  }
  return result;
}

void find_PW_limit_v2(PWLimitDir dir) {
  DCO_calibration_current_note = manual_DCO_calibration_start_note;
  VOICE_NOTES[0] = DCO_calibration_current_note;
  ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];
  DCOCalibrationStart = millis();

  double freqHz   = (double)note_to_freq(DCO_calibration_current_note);
  double periodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;
  uint8_t voiceIdx = cal_pw_channel(currentDCO);
  uint16_t centerPW = PW_CENTER[voiceIdx];
  double targetDuty = (dir == PW_LIMIT_LOW) ? kPWLowDutyFraction : kPWHighDutyFraction;

  g_gapLogCurrentPeriodUs    = periodUs;
  g_gapLogTargetDutyFraction = targetDuty;

  voice_task_autotune(2, ampCompCalibrationVal);
  delay(100);

  PWLimitSearchResult res = search_PW_limit_from_center(voiceIdx, centerPW, dir, periodUs, targetDuty);
  if (calibrationCancelRequested || !res.ok) return;

  if (dir == PW_LIMIT_LOW) {
    update_FS_PW_Low_Limit(voiceIdx, res.limitPW);
    PW_LOW_LIMIT[voiceIdx] = res.limitPW;
  } else {
    update_FS_PW_High_Limit(voiceIdx, res.limitPW);
    PW_HIGH_LIMIT[voiceIdx] = res.limitPW;
  }
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

static String cal_pad_left(const String& s, int width) {
  String out = s;
  while ((int)out.length() < width) out = " " + out;
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

void print_calibration_report(uint8_t dcoIndex, const uint32_t *data) {
  if (autotuneDebug < 1) return;

  String header = (String)"[CAL_REPORT] DCO=" + dcoIndex + " method=" +
                  ((calibrationPrecision == CAL_PRECISION_FINE) ? "REFINE" : autotune_amp_method_name(autotuneAmpMethod)) +
                  " precision=" + calibration_precision_name(calibrationPrecision) +
                  " search=" + autotune_search_mode_name(autotuneSearchMode);
  if (calReportLadderInterval > 0) header += (String)" ladder=" + calReportLadderInterval + " semitones";
  if (calReportAnchorPair >= 0)     header += (String)" anchorPair=" + calReportAnchorPair;

  Serial.println(header);
  Serial.println("[CAL_REPORT] pair    freqHz  ampComp  dutyErr%     gapUs    1cnt%  src");

  float errSum = 0.0f, worstErr = -1.0f;
  int errCount = 0, worstPair = -1, measured = 0, highestPair = -1;

  for (int p = 0; p < kCalReportPairs; ++p) {
    const float    freqHz = (float)data[2 * p] / 100.0f;
    const uint32_t amp    = data[2 * p + 1];
    const uint8_t  src    = calPointSource[p];
    const bool     isSent = (src == CAL_SRC_SENTINEL);
    const float    err    = calPointDutyErrPct[p];
    const bool     hasErr = (fabsf(err) < 1e8f);

    if (!isSent) highestPair = p;
    if (src == CAL_SRC_RUNG || src == CAL_SRC_ANCHOR || src == CAL_SRC_ENDPOINT_FULL ||
        src == CAL_SRC_ENDPOINT_AMP0 || src == CAL_SRC_REFINED) {
      ++measured;
    }

    String line = "[CAL_REPORT] " + cal_pad_left(String(p), 4);
    line += cal_pad_left(isSent ? String("-") : fmt_freq(freqHz), 10);
    line += cal_pad_left(String(amp), 9);

    if (hasErr) {
      float absErr = fabsf(err);
      errSum += absErr;
      ++errCount;
      if (absErr > worstErr) { worstErr = absErr; worstPair = p; }
      float gapUs = (freqHz > 0.0f) ? (err * 20000.0f / freqHz) : 0.0f;
      line += cal_pad_left(String(err, 3), 10);
      line += cal_pad_left(String(gapUs, 2), 10);
    } else {
      line += cal_pad_left("-", 10) + cal_pad_left("-", 10);
    }

    if (!isSent && amp > 0) line += cal_pad_left(String(50.0f / (float)amp, 3), 9);
    else                   line += cal_pad_left("-", 9);

    line += (String)"  " + cal_point_source_name(src);
    Serial.println(line);
  }

  const unsigned long elapsedMs = millis() - calRunStartMs;
  Serial.println((String)"[CAL_REPORT] DCO=" + dcoIndex + " search=" + autotune_search_mode_name(autotuneSearchMode) +
                 " probes=" + calRunProbes + " elapsed=" + String(elapsedMs / 1000.0f, 1) + " s (" +
                 String((float)elapsedMs / (float)max(calRunProbes, 1u), 1) + " ms/probe)");
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

  const uint8_t scope   = calibrationScope;
  const bool    runPW   = calibration_scope_runs_pw(scope);
  const bool    runAmp  = calibration_scope_runs_amp(scope);
  const bool    fine    = (calibrationPrecision == CAL_PRECISION_FINE);

  disable_all_oscillators_and_range_pwm();

  if (runPW) {
    uint8_t lastCh = 0xFF;
    for (uint8_t osc = 0; osc < NUM_OSCILLATORS && !calibrationCancelRequested; ++osc) {
      const uint8_t ch = cal_pw_channel(osc);
      if (ch == lastCh || PW_PINS[ch] == PW_PIN_UNASSIGNED) continue;
      lastCh = ch;
      currentDCO = osc;
      restart_DCO_calibration();
      DCO_calibration_current_note = manual_DCO_calibration_start_note;
      VOICE_NOTES[0] = DCO_calibration_current_note;
      find_PW_center(0);
      if (!calibrationCancelRequested) find_PW_limit_v2(PW_LIMIT_LOW);
      if (!calibrationCancelRequested) find_PW_limit_v2(PW_LIMIT_HIGH);
    }
  }

  for (int i = 0; runAmp && i < NUM_OSCILLATORS && !calibrationCancelRequested; i++) {
    currentDCO = i;
    restart_DCO_calibration();

    ampCompCalibrationVal = initManualAmpCompCalibrationVal[currentDCO] + manualCalibrationOffset[currentDCO];
    write_range_pwm(currentDCO, ampCompCalibrationVal);

    DCO_calibration_current_note = DCO_calibration_start_note;
    VOICE_NOTES[0] = DCO_calibration_current_note;

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
        // FAST / CALC skips live hunt
      } else {
        apply_measured_lowest_freq(ctx);
      }
    }

    if (!calibrationCancelRequested) print_calibration_report(currentDCO, calibrationData);

    if (!calibrationCancelRequested && tableOk) {
      update_FS_voice(currentDCO);
    }
  }

  calibrationFlag = false;
  init_FS();
  precompute_amp_comp_for_engine();
  restore_voice_engine_after_calibration();
}

#endif  // __AUTOTUNE_IMPL_H__