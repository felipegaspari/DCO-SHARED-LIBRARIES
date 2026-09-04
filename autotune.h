#ifndef __AUTOTUNE_H__
#define __AUTOTUNE_H__

#include "autotune_constants.h"
#include "autotune_context.h"
#include "autotune_measurement.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include <Arduino.h>
#include <math.h>
#include <stdint.h>

#include "hardware/irq.h"
#include "hardware/structs/sio.h"
#include "pico/multicore.h"

// ==============================================================================
// Hardware SIO Inter-Core Command Tokens
// ==============================================================================

extern volatile bool calibrationFlag;
extern volatile bool manualCalibrationFlag;
extern volatile bool firstTuneFlag;
extern volatile bool calibrationCancelRequested;
extern volatile bool calibrationVerifyRequested;

enum SioCalCommand : uint32_t {
  SIO_CMD_CALIBRATE = 0xCA11B001,
  SIO_CMD_VERIFY    = 0xCA11B002,
};

void core0_request_calibration();
void core0_request_verify_sweep();
void core0_request_calibration_stop();

// Core 1 interrupt init
void init_core1_cal_interrupt();

// =============================================================================
// Calibration Enums & Scope Selectors
// =============================================================================

enum CalibrationScope : uint8_t {
  CAL_SCOPE_AMP = 1,
  CAL_SCOPE_PW = 2,
  CAL_SCOPE_FULL = 3
};

enum CalPrecision : uint8_t {
  CAL_PRECISION_NORMAL = 0,
  CAL_PRECISION_FINE = 1,
  CAL_PRECISION_FAST = 2
};

enum AutotuneAmp0Mode : uint8_t { AMP0_MODE_MEASURE = 0, AMP0_MODE_CALC = 1 };

enum AutotuneAmpMethod : uint8_t {
  AMP_METHOD_CLASSIC = 0,
  AMP_METHOD_FREQ_TRACE = 1
};

enum AutotuneSearchMode : uint8_t {
  SEARCH_BISECT = 0,
  SEARCH_INTERP = 1,
  SEARCH_GATED = 2
};

enum CalPointSource : uint8_t {
  CAL_SRC_NONE = 0,
  CAL_SRC_RUNG,
  CAL_SRC_ANCHOR,
  CAL_SRC_ENDPOINT_FULL,
  CAL_SRC_ENDPOINT_AMP0,
  CAL_SRC_MANUAL,
  CAL_SRC_FILLED,
  CAL_SRC_SENTINEL,
  CAL_SRC_REFINED
};

enum PWSweepMode : uint8_t {
  PW_SWEEP_FULL = 0,      // Full 2% .. 98% sweep (DCO4)
  PW_SWEEP_HALF_HIGH = 1, // 50% .. 98% sweep, LOW = CENTER (DCO3 High)
  PW_SWEEP_HALF_LOW = 2   // 50% .. 2% sweep, HIGH = CENTER (DCO3 Low)
};

#ifndef PW_SWEEP_MODE_DEFAULT
#if defined(PROJECT_INSTRUMENT) && (PROJECT_INSTRUMENT == 3)
#define PW_SWEEP_MODE_DEFAULT PW_SWEEP_HALF_HIGH
#else
#define PW_SWEEP_MODE_DEFAULT PW_SWEEP_FULL
#endif
#endif

extern uint8_t pwSweepMode;

static inline const char *pw_sweep_mode_name(uint8_t mode) {
  switch (mode) {
  case PW_SWEEP_HALF_HIGH:
    return "HALF_HIGH (50-98%)";
  case PW_SWEEP_HALF_LOW:
    return "HALF_LOW (2-50%)";
  case PW_SWEEP_FULL:
  default:
    return "FULL (2-98%)";
  }
}

enum PWLimitDir : uint8_t { PW_LIMIT_LOW = 0, PW_LIMIT_HIGH = 1 };

// =============================================================================
// Helper Data Structures
// =============================================================================

constexpr int kCalReportPairs = (int)(chanLevelVoiceDataSize / 2);
constexpr float kCalDutyErrUnknown = 1e9f;

struct FreqSearchBounds {
  float loHz;
  float hiHz;
};

struct PWSearchResult {
  bool ok;          // True if search converged on a valid signal
  uint16_t pw;      // Chosen PW PWM level
  double duty;      // Measured duty fraction (0.0 .. 1.0)
  double errorFrac; // Deviation from target duty fraction (duty - targetDuty)
  int probes;       // Number of probes spent
};

// =============================================================================
// Global State Declarations (extern)
// =============================================================================
// Cross-core / ISR handshake flags (ONLY THESE ARE VOLATILE)
extern volatile bool calibrationFlag;
extern volatile bool manualCalibrationFlag;
extern volatile bool firstTuneFlag;
extern volatile bool calibrationCancelRequested;
extern volatile bool calibrationVerifyRequested;
extern volatile bool pwCvProbeRequested;
extern volatile bool calSyncNeutralRequested;
extern volatile uint16_t ampCompCalibrationVal;
// =============================================================================

extern uint8_t calibrationScope;
extern uint8_t calibrationPrecision;
extern uint8_t autotuneAmp0Mode;
extern uint8_t autotuneAmpMethod;
extern uint8_t autotuneSearchMode;

extern uint8_t manualCalibrationStage;
extern int8_t manualCalibrationOffset[NUM_OSCILLATORS];
extern uint8_t manualCalibrationStep;
extern uint16_t ampComp440[NUM_OSCILLATORS];
extern int16_t ampCompDutyOffset[NUM_OSCILLATORS];

extern uint32_t calibrationData[chanLevelVoiceDataSize];
extern float calPointDutyErrPct[kCalReportPairs];
extern uint8_t calPointSource[kCalReportPairs];
extern int calReportLadderInterval;
extern int calReportAnchorPair;
extern uint32_t calRunProbes;
extern unsigned long calRunStartMs;

extern uint8_t manualCalSavedSyncMode;
extern uint8_t manualCalSavedSoftSyncChunks;

extern uint8_t currentDCO;
extern unsigned long DCOCalibrationStart;
extern float calibrationFreqHz;
extern float gapGateFreqHz;
extern float g_lastDrivenFreqHz;

constexpr uint16_t initManualAmpCompCalibrationValPreset =
    (uint16_t)(35u * DIV_COUNTER / 14000u);

extern uint16_t initManualAmpCompCalibrationVal[NUM_OSCILLATORS];
extern uint16_t ampCompLowestFreqVal;
extern uint8_t DCO_calibration_current_note;
extern byte autotuneDebug;

// External references to 3-point limits and cache defined in FS_impl.h
extern PWCalLimits PW_CAL_LIMITS[NUM_PW_CHANNELS][kPWCalPoints];
extern uint8_t ampCompTopPair[NUM_OSCILLATORS];
extern PWTrackCache pwTrackCache[NUM_PW_CHANNELS];

// =============================================================================
// Note and Pitch Constants
// =============================================================================

#ifndef NUM_PW_CHANNELS
#define NUM_PW_CHANNELS NUM_OSCILLATORS
#endif

constexpr uint8_t DCO_calibration_start_note = 29;
constexpr uint8_t calibration_note_interval = 5;
constexpr uint8_t manual_DCO_calibration_start_note =
    DCO_calibration_start_note - 5;
constexpr uint8_t manual_cal_reference_note =
    81; // A4 (440 Hz) in current note map

// =============================================================================
// Inline Conversion & Property Helpers
// =============================================================================

static inline const char *calibration_scope_name(uint8_t s) {
  if (s == CAL_SCOPE_AMP)
    return "AMP";
  if (s == CAL_SCOPE_PW)
    return "PW";
  return "FULL";
}

static inline bool calibration_scope_runs_pw(uint8_t s) {
  return s == CAL_SCOPE_PW || s == CAL_SCOPE_FULL;
}
static inline bool calibration_scope_runs_amp(uint8_t s) {
  return s == CAL_SCOPE_AMP || s == CAL_SCOPE_FULL;
}

static inline const CalPrecisionProfile &cal_precision() {
  if (calibrationPrecision == CAL_PRECISION_FINE)
    return kCalPrecisionFine;
  if (calibrationPrecision == CAL_PRECISION_FAST)
    return kCalPrecisionFast;
  return kCalPrecisionNormal;
}

static inline const char *calibration_precision_name(uint8_t p) {
  if (p == CAL_PRECISION_FINE)
    return "FINE";
  if (p == CAL_PRECISION_FAST)
    return "FAST";
  return "NORMAL";
}

static inline const char *autotune_amp0_mode_name(uint8_t m) {
  return (m == AMP0_MODE_CALC) ? "CALC" : "MEASURE";
}

static inline const char *autotune_amp_method_name(uint8_t m) {
  return (m == AMP_METHOD_FREQ_TRACE) ? "FREQ_TRACE" : "CLASSIC";
}

static inline const char *autotune_search_mode_name(uint8_t m) {
  if (m == SEARCH_BISECT)
    return "BISECT";
  if (m == SEARCH_GATED)
    return "GATED";
  return "INTERP";
}

static inline uint8_t cal_pw_channel(uint8_t osc) {
  if (NUM_PW_CHANNELS == NUM_OSCILLATORS)
    return osc;
  return (uint8_t)(osc / (NUM_OSCILLATORS / NUM_PW_CHANNELS));
}

static inline uint16_t pw_level_readback(uint8_t ch) {
  if (ch >= NUM_PW_CHANNELS || PW_PINS[ch] == PW_PIN_UNASSIGNED)
    return 0;
  const uint32_t cc = pwm_hw->slice[PW_PWM_SLICES[ch]].cc;
  return (pwm_gpio_to_channel(PW_PINS[ch]) == PWM_CHAN_A)
             ? (uint16_t)(cc & 0xFFFFu)
             : (uint16_t)(cc >> 16);
}

static inline uint16_t range_level_readback(uint8_t dco) {
  if (dco >= NUM_OSCILLATORS)
    return 0;
  uint slice = pwm_gpio_to_slice_num(RANGE_PINS[dco]);
  uint channel = pwm_gpio_to_channel(RANGE_PINS[dco]);
  const uint32_t cc = pwm_hw->slice[slice].cc;
  return (channel == PWM_CHAN_A) ? (uint16_t)(cc & 0xFFFFu)
                                 : (uint16_t)(cc >> 16);
}

static inline bool osc_has_pw(uint8_t osc) {
  if (NUM_PW_CHANNELS == NUM_OSCILLATORS) {
    uint8_t ch = cal_pw_channel(osc);
    return (ch < NUM_PW_CHANNELS && PW_PINS[ch] != PW_PIN_UNASSIGNED);
  }
  return ((osc % (NUM_OSCILLATORS / NUM_PW_CHANNELS)) == 0);
}

static inline uint8_t cal_stage_to_osc(uint8_t stage) {
  return cal_stage_to_osc_n(stage, NUM_OSCILLATORS);
}
static inline CalStageKind cal_stage_kind(uint8_t stage) {
  return cal_stage_kind_n(stage, NUM_OSCILLATORS);
}
static inline bool cal_stage_is_440(uint8_t stage) {
  return cal_stage_is_440_n(stage, NUM_OSCILLATORS);
}
static inline bool cal_stage_is_saw(uint8_t stage) {
  return cal_stage_is_saw_n(stage, NUM_OSCILLATORS);
}
static inline bool cal_stage_is_tri(uint8_t stage) {
  return cal_stage_is_tri_n(stage, NUM_OSCILLATORS);
}
static inline bool cal_stage_is_pw_edit(uint8_t stage) {
  return cal_stage_is_pw_edit_n(stage, NUM_OSCILLATORS);
}
static inline bool cal_stage_is_square(uint8_t stage) {
  return cal_stage_is_square_n(stage, NUM_OSCILLATORS);
}

static inline uint8_t cal_manual_osc() {
  uint8_t osc = cal_stage_to_osc(manualCalibrationStage);
  return (osc >= NUM_OSCILLATORS) ? (NUM_OSCILLATORS - 1) : osc;
}

static inline uint8_t cal_stage_max() {
  return cal_stage_max_n(NUM_OSCILLATORS);
}

static inline float duty_trim_gap_us(uint8_t osc, float freqHz) {
  if (osc >= NUM_OSCILLATORS || freqHz <= 0.0f || ampCompDutyOffset[osc] == 0) {
    return 0.0f;
  }
  const float offsetFraction = (float)ampCompDutyOffset[osc] / 10000.0f;
  return 2.0f * (1.0e6f / freqHz) * offsetFraction;
}

static inline float duty_err_pct_from_gap(float gapUs, float freqHz) {
  if (gapUs == kGapTimeoutSentinel || freqHz <= 0.0f) {
    return kCalDutyErrUnknown;
  }
  return 100.0f * gapUs * freqHz / 2.0e6f;
}

static inline String fmt_freq(float hz) { return String(hz, 3); }

static inline float note_to_freq(uint8_t midiNote) {
  return sNotePitches[midiNote - 12];
}

static inline void settle_for_freq(double freqHz) {
  uint32_t settleMs = 4;
  if (freqHz > 0.0) {
    double twoPeriodsMs = 2000.0 / freqHz;
    if (twoPeriodsMs > (double)settleMs) {
      settleMs = (uint32_t)(twoPeriodsMs + 0.999);
    }
  }
  delay(settleMs);
}

struct CalPrecisionOverride {
  uint8_t saved;
  explicit CalPrecisionOverride(uint8_t p = CAL_PRECISION_FINE)
      : saved(calibrationPrecision) {
    calibrationPrecision = p;
  }
  ~CalPrecisionOverride() { calibrationPrecision = saved; }
};

// =============================================================================
// Function Prototypes
// =============================================================================
void autotune_drive_core(uint8_t osc, float freqHz, uint16_t ampValue);
void autotune_manual_task();
void autotune_loop_task();

void apply_pw_baseline(uint8_t ch);
void apply_pw_baseline_solo(uint8_t soloCh);
void apply_pw_center(uint8_t ch);
void apply_pw_center_solo(uint8_t soloCh);

// PW Calibration Entry Points
void DCO_calibration();
void restart_DCO_calibration();
void calibrate_pw_channel_3point(uint8_t ch, uint8_t osc);
void find_PW_center(uint8_t mode = 0);
void find_PW_limit_v2(PWLimitDir dir);

PWSearchResult find_pw_for_target_duty(uint8_t pwCh, double targetDutyFraction,
                                       double dutyToleranceFraction,
                                       uint16_t pwMin, uint16_t pwMax,
                                       uint16_t pwSeed, double freqHz);

void run_calibration_verify_sweep();
void run_pw_cv_probe();
void DCO_calibration_debug();

void cal_report_reset();
void cal_report_set_pair(int pair, float dutyErrPct, uint8_t src);
void cal_report_set_pair_from_gap(int pair, float gapUs, float freqHz,
                                  uint8_t src);
void print_calibration_report(uint8_t dcoIndex, const uint32_t *data);

double compute_gap_tolerance_for_freq(double freqHz, double dutyErrorFraction);
void calibrate_DCO(DCOCalibrationContext &ctx, double dutyErrorFraction);
bool calibrate_DCO_freq_trace(DCOCalibrationContext &ctx);
bool refine_DCO_amp_table(DCOCalibrationContext &ctx);

float measure_duty_at_freq(float freqHz, uint16_t amp, bool hiRes = false);
float find_freq_for_duty50(uint16_t amp, float freqGuess, float windowRatio,
                           bool refine = false,
                           const FreqSearchBounds *bounds = nullptr);
float find_highest_freq(DCOCalibrationContext &ctx, int pairsFilled);
float find_lowest_freq();
float measure_lowest_freq_at_amp0(float freqSeedHz,
                                  const FreqSearchBounds *bounds);
void apply_measured_lowest_freq(DCOCalibrationContext &ctx);

float quadraticInterpolation(float x0, float y0, float x1, float y1, float x2,
                             float y2, float x);
uint16_t logarithmicInterpolation(float x0, float y0, float x1, float y1,
                                  float x);
float linearInterpolation(float x0, float y0, float x1, float y1, float x);
double expInterpolationSolveY(double x, double x0, double x1, double y0,
                              double y1);

// =============================================================================
// PRECOMPUTE PW TRACK CACHE
// =============================================================================

#define PW_LUT_SIZE 512
#define PW_LUT_MAX_FREQ AMP_COMP_MAX_HZ 
constexpr float PW_LUT_BINS_PER_HZ = (float)(PW_LUT_SIZE - 1) / PW_LUT_MAX_FREQ;

struct PWLutItem {
    uint16_t base[2]; // [0]: low limit,       [1]: center
    uint16_t span[2]; // [0]: (center - low),   [1]: (high - center)
} __attribute__((aligned(8)));

//  + 1 safety guard element
PWLutItem pw_lut[NUM_PW_CHANNELS][PW_LUT_SIZE + 1];

// Helper to sample the exact output of your original function at any frequency
void get_exact_limits(uint8_t ch, float freq, float &c, float &l, float &h) {
  const PWTrackCache &cache = pwTrackCache[ch];
  const auto* limits = PW_CAL_LIMITS[ch]; 
  float pwMax_f = (float)(DIV_COUNTER_PW - 1);
  
  if (freq >= cache.f1) {
      if (freq < cache.f2) {
          float t = __builtin_fminf(1.0f, (freq - cache.f1) * cache.invSpan12);
          float c1 = (float)limits[1].center, l1 = (float)limits[1].lowLimit, h1 = (float)limits[1].highLimit;
          c = __builtin_fmaf(t, (float)limits[2].center - c1, c1);
          l = __builtin_fmaf(t, (float)limits[2].lowLimit - l1, l1);
          h = __builtin_fmaf(t, (float)limits[2].highLimit - h1, h1);
      } else {
          c = (float)limits[2].center; l = (float)limits[2].lowLimit; h = (float)limits[2].highLimit;
      }
  } else {
      if (freq >= cache.f0) {
          float t = __builtin_fminf(1.0f, (freq - cache.f0) * cache.invSpan01);
          float c0 = (float)limits[0].center, l0 = (float)limits[0].lowLimit, h0 = (float)limits[0].highLimit;
          c = __builtin_fmaf(t, (float)limits[1].center - c0, c0);
          l = __builtin_fmaf(t, (float)limits[1].lowLimit - l0, l0);
          h = __builtin_fmaf(t, (float)limits[1].highLimit - h0, h0);
      } else if (freq > 0.0f) {
          float t = __builtin_fminf(1.0f, freq * cache.invF0); 
          c = (float)limits[0].center; l = (float)limits[0].lowLimit;
          h = __builtin_fmaf(t, (float)limits[0].highLimit - pwMax_f, pwMax_f);
      } else { 
          c = (float)limits[1].center; l = (float)limits[1].lowLimit; h = (float)limits[1].highLimit;
      }
  }
}

void precompute_pw_regions() {
  for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ch++) {

    constexpr float hz_step = PW_LUT_MAX_FREQ / (float)(PW_LUT_SIZE - 1);

    for (int i = 0; i < PW_LUT_SIZE; i++) {
        float freq = (float)i * hz_step;
        float c, l, h;
        get_exact_limits(ch, freq, c, l, h); // Your existing helper

        // Round directly to integer counts
        uint16_t c_u = (uint16_t)(c + 0.5f);
        uint16_t l_u = (uint16_t)(l + 0.5f);
        uint16_t h_u = (uint16_t)(h + 0.5f);

        // Lower half (PWval 0 to 511): lowLim -> center
        pw_lut[ch][i].base[0] = l_u;
        pw_lut[ch][i].span[0] = (c_u > l_u) ? (c_u - l_u) : 0;

        // Upper half (PWval 512 to 1023): center -> highLim
        pw_lut[ch][i].base[1] = c_u;
        pw_lut[ch][i].span[1] = (h_u > c_u) ? (h_u - c_u) : 0;
    }
  }
}
// =============================================================================
// Dual-Engine 3-Point Key-Tracked Pulse-Width Interpolator
// =============================================================================
#ifdef USE_FLOAT_VOICE_TASK

// faster LUT version
// --- 1. FLOAT ENGINE (RP2350 with Hardware FPU) ---
// TEMPLATE: Call this as get_PW_level_interpolated<PW_SWEEP_FULL>(...) etc. || PW_SWEEP_HALF_LOW || PW_SWEEP_HALF_HIGH || PW_SWEEP_FULL
template <uint8_t SweepMode = PW_SWEEP_FULL>
inline uint16_t SRAM_HOT(get_PW_level_interpolated)(uint16_t PWval, uint8_t oscN, float noteFreqHz = 0.0f, bool invertPolarity = PW_POLARITY_INVERTED) {
  const uint8_t ch = cal_pw_channel(oscN);
  if (__builtin_expect(ch >= NUM_PW_CHANNELS || PW_PINS[ch] == PW_PIN_UNASSIGNED, 0)) return 0;

  constexpr int32_t pwMax = (int32_t)(DIV_COUNTER_PW - 1);
  uint32_t val = (PWval > pwMax) ? pwMax : PWval; 
  val = invertPolarity ? (pwMax - val) : val;

  // 1. Direct integer frequency bin lookup (single conversion)
  uint32_t idx = (noteFreqHz > 0.0f) ? (uint32_t)(noteFreqHz * PW_LUT_BINS_PER_HZ) : 0;
  if (__builtin_expect(idx >= PW_LUT_SIZE, 0)) idx = PW_LUT_SIZE - 1;

  const PWLutItem& item = pw_lut[ch][idx];

  // 2. Pure Integer Math
  if constexpr (SweepMode == PW_SWEEP_HALF_LOW) {
      return item.base[0] + (uint16_t)((val * item.span[0]) / pwMax);
  }
  else if constexpr (SweepMode == PW_SWEEP_HALF_HIGH) {
      return item.base[1] + (uint16_t)((val * item.span[1]) / pwMax);
  }
  else { // FULL SWEEP (Default)
      // Pick lower half (0..511) or upper half (512..1023)
      uint32_t h = (val >= 512);
      uint32_t v = h ? (val - 512) : val;

      return item.base[h] + (uint16_t)(((v * item.span[h]) + 256) >> 9);
  }
}

//* Original function, MOST PRECISION. 2us EXECUTION TIME */
// --- 1. FLOAT ENGINE (RP2350 with Hardware FPU) ---
// TEMPLATE: Call this as get_PW_level_interpolated<PW_SWEEP_FULL>(...)  || PW_SWEEP_HALF_LOW || PW_SWEEP_HALF_HIGH || PW_SWEEP_FULL
/*
template <uint8_t SweepMode = PW_SWEEP_FULL>
inline uint16_t SRAM_HOT(get_PW_level_interpolated)(
    uint16_t PWval, uint8_t oscN, float noteFreqHz = 0.0f,
    bool invertPolarity = PW_POLARITY_INVERTED) {
  const uint8_t ch = cal_pw_channel(oscN);
  if (__builtin_expect(
          ch >= NUM_PW_CHANNELS || PW_PINS[ch] == PW_PIN_UNASSIGNED, 0))
    return 0;

  constexpr int32_t pwMax = (int32_t)(DIV_COUNTER_PW - 1);
  constexpr float pwMax_f = (float)pwMax;
  constexpr float divCounterPw_f = (float)DIV_COUNTER_PW;

  // Branchless (IT block) clamping and polarity mapping
  uint32_t val = (PWval > pwMax) ? pwMax : PWval;
  val = invertPolarity ? (pwMax - val) : val;
  float val_f = (float)val;

  const PWTrackCache &cache = pwTrackCache[ch];
  const auto *__restrict limits = PW_CAL_LIMITS[ch];

  float center_f, lowLim_f, highLim_f;

  // ------------------------------------------------------------------------
  // BINARY SEARCH LOOKUP: Reduces max branch depth from 4 to 2
  // ------------------------------------------------------------------------
  if (noteFreqHz >= cache.f1) {
    if (noteFreqHz < cache.f2) {
      float t =
          __builtin_fminf(1.0f, (noteFreqHz - cache.f1) * cache.invSpan12);
      float c1 = (float)limits[1].center, l1 = (float)limits[1].lowLimit,
            h1 = (float)limits[1].highLimit;

      center_f = __builtin_fmaf(t, (float)limits[2].center - c1, c1);
      lowLim_f = __builtin_fmaf(t, (float)limits[2].lowLimit - l1, l1);
      highLim_f = __builtin_fmaf(t, (float)limits[2].highLimit - h1, h1);
    } else {
      center_f = (float)limits[2].center;
      lowLim_f = (float)limits[2].lowLimit;
      highLim_f = (float)limits[2].highLimit;
    }

  } else {
    if (noteFreqHz >= cache.f0) {
      float t =
          __builtin_fminf(1.0f, (noteFreqHz - cache.f0) * cache.invSpan01);
      float c0 = (float)limits[0].center, l0 = (float)limits[0].lowLimit,
            h0 = (float)limits[0].highLimit;

      center_f = __builtin_fmaf(t, (float)limits[1].center - c0, c0);
      lowLim_f = __builtin_fmaf(t, (float)limits[1].lowLimit - l0, l0);
      highLim_f = __builtin_fmaf(t, (float)limits[1].highLimit - h0, h0);
    } else if (noteFreqHz > 0.0f) {
      // 1-Cycle multiply replaces the 14-cycle VDIV instruction
      float t = __builtin_fminf(
          1.0f, __builtin_fmaxf(0.0f, noteFreqHz * cache.invF0));

      center_f = (float)limits[0].center;
      lowLim_f = (float)limits[0].lowLimit;
      highLim_f =
          __builtin_fmaf(t, (float)limits[0].highLimit - pwMax_f, pwMax_f);
    } else { // <= 0 edge case
      center_f = (float)limits[1].center;
      lowLim_f = (float)limits[1].lowLimit;
      highLim_f = (float)limits[1].highLimit;
    }
  }

  // ------------------------------------------------------------------------
  // BRANCHLESS SWEEP MODE EVALUATION (Resolved at compile-time)
  // ------------------------------------------------------------------------
  float out_f;

  if constexpr (SweepMode == PW_SWEEP_HALF_LOW) {
    out_f =
        __builtin_fmaf(lowLim_f - center_f, val_f * (1.0f / pwMax_f), center_f);
  } else if constexpr (SweepMode == PW_SWEEP_HALF_HIGH) {
    out_f = __builtin_fmaf(highLim_f - center_f, val_f * (1.0f / pwMax_f),
                           center_f);
  } else { // FULL
    constexpr float inv512 = 1.0f / 512.0f;
    float x = val_f * inv512; // Maps [0, 1024] to [0.0, 2.0]

    // 1-cycle Hardware Clamping splits the signal mathematically instead of
    // branching
    float x0 = __builtin_fminf(x, 1.0f); // Clamps to [0, 1] for the bottom half
    float x1 =
        __builtin_fmaxf(x - 1.0f, 0.0f); // Clamps to [0, 1] for the top half

    // FMA Pipelined math: low + x0*(center-low) + x1*(high-center)
    float lower_lerp = __builtin_fmaf(x0, center_f - lowLim_f, lowLim_f);
    out_f = __builtin_fmaf(x1, highLim_f - center_f, lower_lerp);
  }
  // Final clamp and hardware float-to-int conversion
  out_f = __builtin_fmaxf(0.0f, __builtin_fminf(divCounterPw_f, out_f + 0.5f));
  return (uint16_t)(int32_t)out_f;
}
*/

#else

// --- 2. FIXED-POINT ENGINE (RP2040: Zero-Float, Hardware SIO Math) ---
template <uint8_t SweepMode = PW_SWEEP_FULL>
inline uint16_t SRAM_HOT(get_PW_level_interpolated)(
    uint16_t PWval, uint8_t oscN,
    int64_t noteFreqQ24 = 0, // Guarded against 32-bit overflow
    bool invertPolarity = PW_POLARITY_INVERTED) {
    
  const uint8_t ch = cal_pw_channel(oscN);
  if (__builtin_expect(ch >= NUM_PW_CHANNELS || PW_PINS[ch] == PW_PIN_UNASSIGNED, 0)) return 0;

  constexpr int32_t pwMax = (int32_t)(DIV_COUNTER_PW - 1);
  uint32_t val = (PWval > pwMax) ? pwMax : PWval; 
  val = invertPolarity ? (pwMax - val) : val;

  // 1. Fixed-Point Frequency Bin Lookup
  // Shift Q24 down to Q16. This safely fits a 12,500 Hz frequency into a 32-bit integer.
  uint32_t freq_Q16 = (noteFreqQ24 > 0) ? (uint32_t)(noteFreqQ24 >> 8) : 0;
  
  // Precalculate the LUT multiplier in Q16 format
  constexpr uint32_t BINS_PER_HZ_Q16 = (uint32_t)(PW_LUT_BINS_PER_HZ * 65536.0f);
  
  // 32x32 -> 64-bit multiply. 
  // Shifting right by 32 is free (the CPU just reads the high register).
  uint32_t idx = ((uint64_t)freq_Q16 * BINS_PER_HZ_Q16) >> 32;

  // Bounds clamp
  if (__builtin_expect(idx >= PW_LUT_SIZE, 0)) idx = PW_LUT_SIZE - 1;

  const PWLutItem& item = pw_lut[ch][idx];

  // 2. Pure Integer Math (Identical to your chosen version)
  if constexpr (SweepMode == PW_SWEEP_HALF_LOW) {
      return item.base[0] + (uint16_t)((val * item.span[0]) / pwMax);
  }
  else if constexpr (SweepMode == PW_SWEEP_HALF_HIGH) {
      return item.base[1] + (uint16_t)((val * item.span[1]) / pwMax);
  }
  else { // FULL SWEEP (Default)
      // Pick lower half (0..511) or upper half (512..1023)
      uint32_t h = (val >= 512);
      uint32_t v = h ? (val - 512) : val;

      return item.base[h] + (uint16_t)(((v * item.span[h]) + 256) >> 9);
  }
}
#endif // USE_FLOAT_VOICE_TASK

#endif // __AUTOTUNE_H__