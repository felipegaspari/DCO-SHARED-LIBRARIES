#ifndef __AUTOTUNE_H__
#define __AUTOTUNE_H__

#include "../include_all.h"
#include "autotune_constants.h"
#include "autotune_context.h"
#include "autotune_measurement.h"

// =============================================================================
// Calibration Enums & Scope Selectors
// =============================================================================

enum CalibrationScope : uint8_t {
  CAL_SCOPE_AMP  = 1,
  CAL_SCOPE_PW   = 2,
  CAL_SCOPE_FULL = 3
};

enum CalPrecision : uint8_t {
  CAL_PRECISION_NORMAL = 0,
  CAL_PRECISION_FINE   = 1,
  CAL_PRECISION_FAST   = 2
};

enum AutotuneAmp0Mode : uint8_t {
  AMP0_MODE_MEASURE = 0,
  AMP0_MODE_CALC    = 1
};

enum AutotuneAmpMethod : uint8_t {
  AMP_METHOD_CLASSIC    = 0,
  AMP_METHOD_FREQ_TRACE = 1
};

enum AutotuneSearchMode : uint8_t {
  SEARCH_BISECT = 0,
  SEARCH_INTERP = 1,
  SEARCH_GATED  = 2
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

// =============================================================================
// Helper Data Structures
// =============================================================================

constexpr int   kCalReportPairs   = (int)(chanLevelVoiceDataSize / 2);
constexpr float kCalDutyErrUnknown = 1e9f;

struct FreqSearchBounds {
  float loHz;
  float hiHz;
};

// =============================================================================
// PW Calibration Types & Prototypes
// =============================================================================

enum PWLimitDir : uint8_t {
  PW_LIMIT_LOW  = 0,
  PW_LIMIT_HIGH = 1
};

struct PWSearchResult {
  bool     ok;           // True if search converged on a valid signal
  uint16_t pw;           // Chosen PW PWM level
  double   duty;         // Measured duty fraction (0.0 .. 1.0)
  double   errorFrac;    // Deviation from target duty fraction (duty - targetDuty)
  int      probes;       // Number of probes spent
};

// High-level PW calibration entry points
void find_PW_center(uint8_t mode = 0);
void find_PW_limit_v2(PWLimitDir dir);

// Low-level modern search engine
PWSearchResult find_pw_for_target_duty(
  uint8_t  pwCh,
  double   targetDutyFraction,
  double   dutyToleranceFraction,
  uint16_t pwMin,
  uint16_t pwMax,
  uint16_t pwSeed,
  double   freqHz
);

// =============================================================================
// Global State Declarations (extern)
// =============================================================================

extern bool calibrationFlag;
extern bool manualCalibrationFlag;
extern bool firstTuneFlag;
extern volatile bool calibrationCancelRequested;

extern uint8_t calibrationScope;
extern uint8_t calibrationPrecision;
extern uint8_t autotuneAmp0Mode;
extern uint8_t autotuneAmpMethod;
extern uint8_t autotuneSearchMode;

extern uint8_t manualCalibrationStage;
extern int8_t  manualCalibrationOffset[NUM_OSCILLATORS];
extern uint8_t manualCalibrationStep;
extern uint16_t ampComp440[NUM_OSCILLATORS];
extern int16_t  ampCompDutyOffset[NUM_OSCILLATORS];

extern uint32_t calibrationData[chanLevelVoiceDataSize];
extern float    calPointDutyErrPct[kCalReportPairs];
extern uint8_t  calPointSource[kCalReportPairs];
extern int      calReportLadderInterval;
extern int      calReportAnchorPair;
extern uint32_t calRunProbes;
extern unsigned long calRunStartMs;

extern volatile bool calibrationVerifyRequested;
extern volatile bool pwCvProbeRequested;
extern uint8_t manualCalSavedSyncMode;
extern uint8_t manualCalSavedSoftSyncChunks;
extern volatile bool calSyncNeutralRequested;

extern uint8_t currentDCO;
extern unsigned long DCOCalibrationStart;
extern volatile uint16_t ampCompCalibrationVal;
extern float calibrationFreqHz;
extern float gapGateFreqHz;
extern float g_lastDrivenFreqHz;

constexpr uint16_t initManualAmpCompCalibrationValPreset =
    (uint16_t)(35u * DIV_COUNTER / 14000u);

extern uint16_t initManualAmpCompCalibrationVal[NUM_OSCILLATORS];
extern volatile uint16_t ampCompLowestFreqVal;
extern uint16_t initManualAmpCompCalibrationVal[NUM_OSCILLATORS];
extern volatile uint16_t ampCompLowestFreqVal;
extern uint8_t DCO_calibration_current_note;
extern byte autotuneDebug;

// =============================================================================
// Note and Pitch Constants
// =============================================================================

#ifndef NUM_PW_CHANNELS
#define NUM_PW_CHANNELS NUM_OSCILLATORS
#endif

constexpr uint8_t DCO_calibration_start_note        = 29;
constexpr uint8_t calibration_note_interval         = 5;
constexpr uint8_t manual_DCO_calibration_start_note = DCO_calibration_start_note - 5;
constexpr uint8_t manual_cal_reference_note         = 81; // A4 (440 Hz) in current note map

// =============================================================================
// Inline Conversion & Property Helpers
// =============================================================================

static inline const char* calibration_scope_name(uint8_t s) {
  if (s == CAL_SCOPE_AMP) return "AMP";
  if (s == CAL_SCOPE_PW)  return "PW";
  return "FULL";
}

static inline bool calibration_scope_runs_pw(uint8_t s)  { return s == CAL_SCOPE_PW || s == CAL_SCOPE_FULL; }
static inline bool calibration_scope_runs_amp(uint8_t s) { return s == CAL_SCOPE_AMP || s == CAL_SCOPE_FULL; }

static inline const CalPrecisionProfile& cal_precision() {
  if (calibrationPrecision == CAL_PRECISION_FINE) return kCalPrecisionFine;
  if (calibrationPrecision == CAL_PRECISION_FAST) return kCalPrecisionFast;
  return kCalPrecisionNormal;
}

static inline const char* calibration_precision_name(uint8_t p) {
  if (p == CAL_PRECISION_FINE) return "FINE";
  if (p == CAL_PRECISION_FAST) return "FAST";
  return "NORMAL";
}

static inline const char* autotune_amp0_mode_name(uint8_t m) {
  return (m == AMP0_MODE_CALC) ? "CALC" : "MEASURE";
}

static inline const char* autotune_amp_method_name(uint8_t m) {
  return (m == AMP_METHOD_FREQ_TRACE) ? "FREQ_TRACE" : "CLASSIC";
}

static inline const char* autotune_search_mode_name(uint8_t m) {
  if (m == SEARCH_BISECT) return "BISECT";
  if (m == SEARCH_GATED)  return "GATED";
  return "INTERP";
}

static inline uint8_t cal_pw_channel(uint8_t osc) {
  if (NUM_PW_CHANNELS == NUM_OSCILLATORS) return osc;
  return (uint8_t)(osc / (NUM_OSCILLATORS / NUM_PW_CHANNELS));
}

// Hardware readback of the live PWM compare register for a PW channel
static inline uint16_t pw_level_readback(uint8_t ch) {
  if (ch >= NUM_PW_CHANNELS || PW_PINS[ch] == PW_PIN_UNASSIGNED) return 0;
  const uint32_t cc = pwm_hw->slice[PW_PWM_SLICES[ch]].cc;
  return (pwm_gpio_to_channel(PW_PINS[ch]) == PWM_CHAN_A) ? (uint16_t)(cc & 0xFFFFu)
  : (uint16_t)(cc >> 16);
}

// Hardware readback of the live PWM compare register for a RANGE amplitude pin
static inline uint16_t range_level_readback(uint8_t dco) {
  if (dco >= NUM_OSCILLATORS) return 0;
  uint slice = pwm_gpio_to_slice_num(RANGE_PINS[dco]);
  uint channel = pwm_gpio_to_channel(RANGE_PINS[dco]);
  const uint32_t cc = pwm_hw->slice[slice].cc;
  return (channel == PWM_CHAN_A) ? (uint16_t)(cc & 0xFFFFu) : (uint16_t)(cc >> 16);
}

// Universal check: returns true if oscillator 'osc' has a pulse-width comparator
static inline bool osc_has_pw(uint8_t osc) {
  if (NUM_PW_CHANNELS == NUM_OSCILLATORS) {
    uint8_t ch = cal_pw_channel(osc);
    return (ch < NUM_PW_CHANNELS && PW_PINS[ch] != PW_PIN_UNASSIGNED);
  }
  return ((osc % (NUM_OSCILLATORS / NUM_PW_CHANNELS)) == 0);
}

static inline uint8_t cal_stage_to_osc(uint8_t stage)    { return cal_stage_to_osc_n(stage, NUM_OSCILLATORS); }
static inline CalStageKind cal_stage_kind(uint8_t stage) { return cal_stage_kind_n(stage, NUM_OSCILLATORS); }
static inline bool cal_stage_is_440(uint8_t stage)       { return cal_stage_is_440_n(stage, NUM_OSCILLATORS); }
static inline bool cal_stage_is_saw(uint8_t stage)       { return cal_stage_is_saw_n(stage, NUM_OSCILLATORS); }
static inline bool cal_stage_is_tri(uint8_t stage)       { return cal_stage_is_tri_n(stage, NUM_OSCILLATORS); }
static inline bool cal_stage_is_pw_edit(uint8_t stage)   { return cal_stage_is_pw_edit_n(stage, NUM_OSCILLATORS); }
static inline bool cal_stage_is_square(uint8_t stage)    { return cal_stage_is_square_n(stage, NUM_OSCILLATORS); }

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

static inline String fmt_freq(float hz) {
  return String(hz, 3);
}

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

// Scoped Precision Override RAII Helper
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

void DCO_calibration();
void restart_DCO_calibration();
void find_PW_center(uint8_t mode);
void find_PW_limit_v2(PWLimitDir dir);
void run_calibration_verify_sweep();
void run_pw_cv_probe();
void DCO_calibration_debug();

void cal_report_reset();
void cal_report_set_pair(int pair, float dutyErrPct, uint8_t src);
void cal_report_set_pair_from_gap(int pair, float gapUs, float freqHz, uint8_t src);
void print_calibration_report(uint8_t dcoIndex, const uint32_t *data);

double compute_gap_tolerance_for_freq(double freqHz, double dutyErrorFraction);
void   calibrate_DCO(DCOCalibrationContext& ctx, double dutyErrorFraction);
bool   calibrate_DCO_freq_trace(DCOCalibrationContext& ctx);
bool   refine_DCO_amp_table(DCOCalibrationContext& ctx);

float  measure_duty_at_freq(float freqHz, uint16_t amp, bool hiRes = false);
float  find_freq_for_duty50(uint16_t amp, float freqGuess, float windowRatio,
                           bool refine = false, const FreqSearchBounds *bounds = nullptr);
float  find_highest_freq(DCOCalibrationContext& ctx, int pairsFilled);
float  find_lowest_freq();
float  measure_lowest_freq_at_amp0(float freqSeedHz, const FreqSearchBounds *bounds);
void   apply_measured_lowest_freq(DCOCalibrationContext& ctx);

float    quadraticInterpolation(float x0, float y0, float x1, float y1, float x2, float y2, float x);
uint16_t logarithmicInterpolation(float x0, float y0, float x1, float y1, float x);
float    linearInterpolation(float x0, float y0, float x1, float y1, float x);
double   expInterpolationSolveY(double x, double x0, double x1, double y0, double y1);

#endif  // __AUTOTUNE_H__
