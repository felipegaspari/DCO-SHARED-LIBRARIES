#ifndef __AMP_COMP_H__
#define __AMP_COMP_H__

#include <math.h>
#include <limits.h>
#include <string.h>
#include "hardware/sync.h"

// Common table dimensions and shared data
static constexpr int     ampCompTableSize = 22;
static constexpr int32_t AMP_COMP_MAX_HZ  = 7000;

int32_t freq_to_amp_comp_array[chanLevelVoiceDataSize * NUM_OSCILLATORS];

// Explicit Max Frequencies loaded from flash (User must populate highestFreqFoundHz before precompute)
float highestFreqFoundHz[NUM_OSCILLATORS];

// Derived high-speed clamp ceilings
int32_t ampCompMaxFreqQ[NUM_OSCILLATORS];

// Calibration levels (PWM counts)
int32_t ampCompArray[NUM_OSCILLATORS][ampCompTableSize + 1];

// ----- Fixed-point (Q8) amp-comp data -----
static constexpr int     FREQ_FRAC_BITS    = 8;
static constexpr int32_t AMP_COMP_SENTINEL_FREQ_Q = 50000000;
static constexpr int32_t AMP_COMP_MAX_HZ_Q = (int32_t)(AMP_COMP_MAX_HZ << FREQ_FRAC_BITS);

int32_t ampCompFrequencyArray[NUM_OSCILLATORS][ampCompTableSize + 1]; // Q8 Hz
static constexpr int T_FRAC = 12;

// Grouped fixed-point window data for better cache locality (AoS)
struct FixedQuadWindow {
    int32_t  xBase;
    int32_t  dx;
    uint32_t invDx_q28;
    int64_t  aQ;          // Q(T_FRAC)
    int64_t  bQ;          // Q(T_FRAC)
    uint16_t cQ;
    int32_t  aQ_fast;
    int32_t  bQ_fast;
};
FixedQuadWindow fixedWin[NUM_OSCILLATORS][ampCompTableSize - 1];

bool amp_quad_muls_i32 = false;

// Last quadratic window per osc for FIXED / FLOAT_QUAD find (-1 = cold).
int16_t ampWinCache[NUM_OSCILLATORS];

// Grouped high-precision float coefficients for better cache locality
struct FloatQuadCoeffs {
    float a;
    float b;
    float c;
};
FloatQuadCoeffs floatCoeffs[NUM_OSCILLATORS][ampCompTableSize - 1];

// Float-domain frequency breakpoints (Hz) used by the float amp-comp path.
#ifdef USE_FLOAT_AMP_COMP
float ampCompMaxFreqHz[NUM_OSCILLATORS];
float ampCompFrequencyHz[NUM_OSCILLATORS][ampCompTableSize + 1];
// Dense LUT: index = integer Hz, value = RANGE PWM. Filled from float quadratic.
uint16_t ampCompLut[NUM_OSCILLATORS][AMP_COMP_MAX_HZ + 1];
#endif

// ---------------------------------------------------------------------------
// Method selection
// ---------------------------------------------------------------------------
enum AmpCompMethod : uint8_t {
  AMP_COMP_FLOAT_QUAD = 0,
  AMP_COMP_LUT        = 1,
  AMP_COMP_FIXED      = 2,
};

#ifndef AMP_COMP_METHOD_DEFAULT
#define AMP_COMP_METHOD_DEFAULT AMP_COMP_FIXED
#endif

volatile uint8_t amp_comp_method = (uint8_t)AMP_COMP_METHOD_DEFAULT;
volatile bool amp_comp_method_ack_pending = false;

static inline const char *amp_comp_method_name(uint8_t m) {
  switch (m) {
    case AMP_COMP_FLOAT_QUAD: return "FLOAT_QUAD";
    case AMP_COMP_LUT:        return "LUT";
    case AMP_COMP_FIXED:      return "FIXED";
    default:                  return "UNKNOWN";
  }
}

static inline void amp_comp_set_method(uint8_t m) {
#ifdef USE_FLOAT_AMP_COMP
  if (m > AMP_COMP_FIXED) m = AMP_COMP_FLOAT_QUAD;
#else
  m = AMP_COMP_FIXED;
#endif
  amp_comp_method = m;
  __dmb();
}

// ---------------------------------------------------------------------------
// Precompute Functions
// ---------------------------------------------------------------------------

static void precomputeCoefficients() {
  static_assert(T_FRAC > 0 && T_FRAC < 28, "T_FRAC must be in a valid range.");

  for (int j = 0; j < NUM_OSCILLATORS; ++j) {
      float maxHz = highestFreqFoundHz[j];
      // Safeguard: fallback to stock ceiling if uncalibrated or invalid
      if (maxHz <= 0.0f || maxHz > (float)AMP_COMP_MAX_HZ) {
          maxHz = (float)AMP_COMP_MAX_HZ;
      }
      ampCompMaxFreqQ[j] = (int32_t)lrintf(maxHz * (float)(1 << FREQ_FRAC_BITS));
      
      ampCompFrequencyArray[j][ampCompTableSize] = AMP_COMP_MAX_HZ_Q;
      ampCompArray[j][ampCompTableSize] = DIV_COUNTER;
  }

  const double freqScale    = (double)(1u << FREQ_FRAC_BITS);
  const double invFreqScale = 1.0 / freqScale;
  const double maxFreqHz    = (double)AMP_COMP_MAX_HZ;

  for (int j = 0; j < NUM_OSCILLATORS; j++) {
    for (int i = 0; i < ampCompTableSize - 1; ++i) {
      double x0_f = (double)ampCompFrequencyArray[j][i]     * invFreqScale;
      double x1_f = (double)ampCompFrequencyArray[j][i + 1] * invFreqScale;
      double x2_f = (double)ampCompFrequencyArray[j][i + 2] * invFreqScale;
      double y0_f = (double)ampCompArray[j][i];
      double y1_f = (double)ampCompArray[j][i + 1];
      double y2_f = (double)ampCompArray[j][i + 2];

      if (ampCompFrequencyArray[j][i + 1] >= AMP_COMP_MAX_HZ_Q) x1_f = maxFreqHz;
      if (ampCompFrequencyArray[j][i + 2] >= AMP_COMP_MAX_HZ_Q) x2_f = maxFreqHz;

      long double denom_ld = (long double)(x0_f - x1_f) * (long double)(x0_f - x2_f) * (long double)(x1_f - x2_f);
      if (denom_ld == 0.0L) denom_ld = 1.0L;
      long double inv_denom_ld = 1.0L / denom_ld;

      long double aVal_ld = ((long double)x2_f * (long double)(y1_f - y0_f) +
                             (long double)x1_f * (long double)(y0_f - y2_f) +
                             (long double)x0_f * (long double)(y2_f - y1_f)) * inv_denom_ld;
      long double bVal_ld = ((long double)x2_f * (long double)x2_f * (long double)(y0_f - y1_f) +
                             (long double)x1_f * (long double)x1_f * (long double)(y2_f - y0_f) +
                             (long double)x0_f * (long double)x0_f * (long double)(y1_f - y2_f)) * inv_denom_ld;
      long double cVal_ld = ((long double)x1_f * (long double)x2_f * (long double)(x1_f - x2_f) * (long double)y0_f +
                             (long double)x2_f * (long double)x0_f * (long double)(x2_f - x0_f) * (long double)y1_f +
                             (long double)x0_f * (long double)x1_f * (long double)(x0_f - x1_f) * (long double)y2_f) * inv_denom_ld;

      // Assign to struct
      floatCoeffs[j][i].a = (float)aVal_ld;
      floatCoeffs[j][i].b = (float)bVal_ld;
      floatCoeffs[j][i].c = (float)cVal_ld;

      long double dx02_ld = (long double)x2_f - (long double)x0_f;
      if (dx02_ld <= 0.0L) dx02_ld = 1.0L;
      long double inv_dx02_ld = 1.0L / dx02_ld;
      long double t1_ld = ((long double)x1_f - (long double)x0_f) * inv_dx02_ld;
      long double d20_ld = (long double)y2_f - (long double)y0_f;
      long double d10_ld = (long double)y1_f - (long double)y0_f;

      long double denom_norm_ld = (t1_ld * t1_ld - t1_ld);
      if (denom_norm_ld == 0.0L) denom_norm_ld = 1.0L;
      long double inv_denom_norm_ld = 1.0L / denom_norm_ld;

      long double aN_ld = (d10_ld - d20_ld * t1_ld) * inv_denom_norm_ld;
      long double bN_ld = d20_ld - aN_ld;

      // Assign to struct
      fixedWin[j][i].xBase = ampCompFrequencyArray[j][i];
      fixedWin[j][i].dx    = ampCompFrequencyArray[j][i + 2] - ampCompFrequencyArray[j][i];
      if (fixedWin[j][i].dx <= 0) fixedWin[j][i].dx = 1;

      uint32_t dxu = (uint32_t)fixedWin[j][i].dx;
      uint64_t num = (uint64_t)1ULL << 28;
      fixedWin[j][i].invDx_q28 = (uint32_t)((num + (dxu >> 1)) / dxu);

      fixedWin[j][i].aQ = (int64_t)llroundl(aN_ld * (long double)(1LL << T_FRAC));
      fixedWin[j][i].bQ = (int64_t)llroundl(bN_ld * (long double)(1LL << T_FRAC));

      int32_t c_temp = (int32_t)lrint(y0_f);
      if (c_temp < 0) c_temp = 0;
      if (c_temp > (int32_t)DIV_COUNTER) c_temp = (int32_t)DIV_COUNTER;
      fixedWin[j][i].cQ = (uint16_t)c_temp;

      int64_t aFastLL = llroundl(aN_ld * (long double)(1 << T_FRAC));
      if (aFastLL > (int64_t)INT32_MAX) aFastLL = (int64_t)INT32_MAX;
      if (aFastLL < (int64_t)INT32_MIN) aFastLL = (int64_t)INT32_MIN;
      fixedWin[j][i].aQ_fast = (int32_t)aFastLL;

      int64_t d20_int = ((int64_t)ampCompArray[j][i + 2] - (int64_t)ampCompArray[j][i]) << T_FRAC;
      int64_t bFastLL = d20_int - aFastLL;
      if (bFastLL > (int64_t)INT32_MAX) bFastLL = (int64_t)INT32_MAX;
      if (bFastLL < (int64_t)INT32_MIN) bFastLL = (int64_t)INT32_MIN;
      fixedWin[j][i].bQ_fast = (int32_t)bFastLL;
    }
  }

  bool ok = true;
  const int64_t t_max = (int64_t)(1 << T_FRAC);
  for (int j = 0; j < NUM_OSCILLATORS && ok; ++j) {
    for (int i = 0; i < ampCompTableSize - 1; ++i) {
      const int64_t pa = (int64_t)fixedWin[j][i].aQ_fast * t_max;
      const int64_t pb = (int64_t)fixedWin[j][i].bQ_fast * t_max;
      if (pa > (int64_t)INT32_MAX || pa < (int64_t)INT32_MIN ||
          pb > (int64_t)INT32_MAX || pb < (int64_t)INT32_MIN) {
        ok = false;
        break;
      }
    }
  }
  amp_quad_muls_i32 = ok;
}

#ifdef USE_FLOAT_AMP_COMP
static void precomputeCoefficients_float() {
  const double maxFreqHz = (double)AMP_COMP_MAX_HZ;

  for (int j = 0; j < NUM_OSCILLATORS; ++j) {
    float maxHz = highestFreqFoundHz[j];
    if (maxHz <= 0.0f || maxHz > (float)AMP_COMP_MAX_HZ) {
        maxHz = (float)AMP_COMP_MAX_HZ;
    }
    ampCompMaxFreqHz[j] = maxHz;
      
    ampCompFrequencyHz[j][ampCompTableSize] = (float)AMP_COMP_MAX_HZ;
    ampCompArray[j][ampCompTableSize]       = DIV_COUNTER;
  }

  for (int j = 0; j < NUM_OSCILLATORS; ++j) {
    for (int i = 0; i < ampCompTableSize - 1; ++i) {
      double x0_f = (double)ampCompFrequencyHz[j][i];
      double x1_f = (double)ampCompFrequencyHz[j][i + 1];
      double x2_f = (double)ampCompFrequencyHz[j][i + 2];
      double y0_f = (double)ampCompArray[j][i];
      double y1_f = (double)ampCompArray[j][i + 1];
      double y2_f = (double)ampCompArray[j][i + 2];

      if (x1_f >= maxFreqHz) x1_f = maxFreqHz;
      if (x2_f >= maxFreqHz) x2_f = maxFreqHz;

      long double denom_ld = (long double)(x0_f - x1_f) * (long double)(x0_f - x2_f) * (long double)(x1_f - x2_f);
      if (denom_ld == 0.0L) denom_ld = 1.0L;
      long double inv_denom_ld = 1.0L / denom_ld;

      long double aVal_ld = ((long double)x2_f * (long double)(y1_f - y0_f) +
                             (long double)x1_f * (long double)(y0_f - y2_f) +
                             (long double)x0_f * (long double)(y2_f - y1_f)) * inv_denom_ld;
      long double bVal_ld = ((long double)x2_f * (long double)x2_f * (long double)(y0_f - y1_f) +
                             (long double)x1_f * (long double)x1_f * (long double)(y2_f - y0_f) +
                             (long double)x0_f * (long double)x0_f * (long double)(y1_f - y2_f)) * inv_denom_ld;
      long double cVal_ld = ((long double)x1_f * (long double)x2_f * (long double)(x1_f - x2_f) * (long double)y0_f +
                             (long double)x2_f * (long double)x0_f * (long double)(x2_f - x0_f) * (long double)y1_f +
                             (long double)x0_f * (long double)x1_f * (long double)(x0_f - x1_f) * (long double)y2_f) * inv_denom_ld;

      // Assign to struct
      floatCoeffs[j][i].a = (float)aVal_ld;
      floatCoeffs[j][i].b = (float)bVal_ld;
      floatCoeffs[j][i].c = (float)cVal_ld;
    }
  }
}

uint16_t get_chan_level_float_quad(float freqHz, uint8_t voiceN);

static inline void fill_amp_comp_lut_from_quad() {
  for (uint8_t o = 0; o < NUM_OSCILLATORS; ++o) {
    int32_t maxHz = (int32_t)ampCompMaxFreqHz[o];
    if (maxHz > AMP_COMP_MAX_HZ) maxHz = AMP_COMP_MAX_HZ;

    // Evaluate curve up to the true maximum
    for (int32_t hz = 0; hz < maxHz; ++hz) {
      ampCompLut[o][hz] = get_chan_level_float_quad((float)hz, o);
    }
    // Block-fill saturated plateau instantly
    for (int32_t hz = maxHz; hz <= AMP_COMP_MAX_HZ; ++hz) {
      ampCompLut[o][hz] = (uint16_t)DIV_COUNTER;
    }
  }
}

static inline void amp_comp_seed_fixed_from_float_tables() {
  const double freqScale = (double)(1u << FREQ_FRAC_BITS);
  for (int j = 0; j < NUM_OSCILLATORS; ++j) {
    for (int i = 0; i <= ampCompTableSize; ++i) {
      double hz = (double)ampCompFrequencyHz[j][i];
      ampCompFrequencyArray[j][i] = (int32_t)llround(hz * freqScale);
    }
  }
}
#endif

// Dispatch to the correct precompute routine
static inline void precompute_amp_comp_for_engine() {
#ifdef USE_FLOAT_AMP_COMP
  precomputeCoefficients_float();
  amp_comp_seed_fixed_from_float_tables();
  precomputeCoefficients();
  fill_amp_comp_lut_from_quad();
#else
  precomputeCoefficients();
#endif
  for (int o = 0; o < NUM_OSCILLATORS; ++o) ampWinCache[o] = -1;

  precompute_pw_tracking_cache();
}

// ---------------------------------------------------------------------------
// Prototypes
// ---------------------------------------------------------------------------
uint16_t get_chan_level_lookup_fast(int32_t x, uint8_t voiceN);
#ifdef USE_FLOAT_AMP_COMP
uint16_t get_chan_level_float_quad(float freqHz, uint8_t voiceN);
uint16_t get_chan_level_lut(float freqHz, uint8_t voiceN);

static inline uint16_t get_chan_level_by_method(float freqHz, uint8_t voiceN) {
  switch (amp_comp_method) {
    case AMP_COMP_LUT:
      return get_chan_level_lut(freqHz, voiceN);
    case AMP_COMP_FIXED: {
      if (freqHz <= 0.0f) return 0;
      if (freqHz >= (float)AMP_COMP_MAX_HZ) {
        return get_chan_level_lookup_fast(AMP_COMP_MAX_HZ_Q, voiceN);
      }
      int32_t x_q = (int32_t)lrintf(freqHz * (float)(1 << FREQ_FRAC_BITS));
      return get_chan_level_lookup_fast(x_q, voiceN);
    }
    case AMP_COMP_FLOAT_QUAD:
    default:
      return get_chan_level_float_quad(freqHz, voiceN);
  }
}

static inline uint16_t get_chan_level_float(float freqHz, uint8_t voiceN) {
  return get_chan_level_by_method(freqHz, voiceN);
}
#endif

static inline uint16_t get_chan_level_for_engine(float freqHz, uint8_t voiceN) {
#ifdef USE_FLOAT_AMP_COMP
  return get_chan_level_by_method(freqHz, voiceN);
#else
  if (freqHz <= 0.0f) return 0;
  if (freqHz >= (float)AMP_COMP_MAX_HZ) return get_chan_level_lookup_fast(AMP_COMP_MAX_HZ_Q, voiceN);

  int32_t x_q = (int32_t)lrintf(freqHz * (float)(1 << FREQ_FRAC_BITS));
  return get_chan_level_lookup_fast(x_q, voiceN);
#endif
}

extern volatile bool amp_comp_bench_speed_pending;
extern volatile bool amp_comp_bench_accuracy_pending;
void print_amp_comp_bench();
void amp_comp_bench_run_speed();
void amp_comp_bench_run_accuracy();

#endif