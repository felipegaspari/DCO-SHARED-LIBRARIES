#ifndef __AMP_COMP_H__
#define __AMP_COMP_H__

#include <math.h>
#include <limits.h>
#include <string.h>
#include "hardware/sync.h"

// ---------------------------------------------------------------------------
// Common table dimensions and shared data
// ---------------------------------------------------------------------------
static constexpr int     ampCompTableSize = 22;
static constexpr int32_t AMP_COMP_MAX_HZ  = 7000;

SRAM_DATA int32_t freq_to_amp_comp_array[chanLevelVoiceDataSize * NUM_OSCILLATORS];

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
static constexpr int     T_FRAC            = 12;

SRAM_DATA int32_t ampCompFrequencyArray[NUM_OSCILLATORS][ampCompTableSize + 1]; // Q8 Hz

// Legacy fixed window data (only needed if NOT in float mode)
#ifndef USE_FLOAT_AMP_COMP
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
SRAM_DATA FixedQuadWindow fixedWin[NUM_OSCILLATORS][ampCompTableSize - 1];
bool amp_quad_muls_i32 = false;
#endif

// Last quadratic window per osc for FIXED / FLOAT_QUAD find (-1 = cold).
SRAM_DATA int16_t ampWinCache[NUM_OSCILLATORS];

// Grouped high-precision float coefficients
struct FloatQuadCoeffs {
    float a;
    float b;
    float c;
};
SRAM_DATA FloatQuadCoeffs floatCoeffs[NUM_OSCILLATORS][ampCompTableSize - 1];

// Float-domain frequency breakpoints (Hz) used by the float amp-comp path.
#ifdef USE_FLOAT_AMP_COMP
SRAM_DATA float ampCompMaxFreqHz[NUM_OSCILLATORS];
SRAM_DATA float ampCompFrequencyHz[NUM_OSCILLATORS][ampCompTableSize + 1];
#endif

// UNIFIED DENSE 16-BIT LUT (168 KB total)
// Used by both Float and Q16/Q24 paths (Eliminated 336 KB Q8 table to prevent Out-Of-Memory)
SRAM_DATA uint16_t ampCompLut[NUM_OSCILLATORS][AMP_COMP_MAX_HZ + 1];

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
// Prototypes
// ---------------------------------------------------------------------------
uint16_t SRAM_HOT(get_chan_level_float_quad)(float freqHz, uint8_t voiceN);
uint16_t SRAM_HOT(get_chan_level_lut)(float freqHz, uint8_t voiceN);

// ---------------------------------------------------------------------------
// Precompute Functions
// ---------------------------------------------------------------------------

  
static void precomputeCoefficients() {
  static_assert(T_FRAC > 0 && T_FRAC < 28, "T_FRAC must be in a valid range.");

  for (int j = 0; j < NUM_OSCILLATORS; ++j) {
      float maxHz = highestFreqFoundHz[j];
      if (maxHz <= 0.0f || maxHz > (float)AMP_COMP_MAX_HZ) {
          maxHz = (float)AMP_COMP_MAX_HZ;
      }
      ampCompMaxFreqQ[j] = (int32_t)lrintf(maxHz * (float)(1 << FREQ_FRAC_BITS));
      
      ampCompFrequencyArray[j][ampCompTableSize] = AMP_COMP_MAX_HZ_Q;
      ampCompArray[j][ampCompTableSize]          = DIV_COUNTER;
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

      // Assign to float struct
      floatCoeffs[j][i].a = (float)aVal_ld;
      floatCoeffs[j][i].b = (float)bVal_ld;
      floatCoeffs[j][i].c = (float)cVal_ld;

#ifndef USE_FLOAT_AMP_COMP
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
#endif
    }
  }

#ifndef USE_FLOAT_AMP_COMP
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
#endif
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

      floatCoeffs[j][i].a = (float)aVal_ld;
      floatCoeffs[j][i].b = (float)bVal_ld;
      floatCoeffs[j][i].c = (float)cVal_ld;
    }
  }
}

static inline void fill_amp_comp_lut_from_quad() {
  for (uint8_t o = 0; o < NUM_OSCILLATORS; ++o) {
    int32_t maxHz = (int32_t)ampCompMaxFreqHz[o];
    if (maxHz > AMP_COMP_MAX_HZ) maxHz = AMP_COMP_MAX_HZ;

    for (int32_t hz = 0; hz < maxHz; ++hz) {
      ampCompLut[o][hz] = get_chan_level_float_quad((float)hz, o);
    }
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

/**
 * @brief High-Precision Fixed-Engine LUT Baker (Boot-time only)
 * Evaluates the analytical quadratic formula directly into ampCompLut with zero circular calls.
 */
 static inline void fill_amp_comp_lut_from_fixed() {
  const int maxWindow = ampCompTableSize - 2;

  for (uint8_t o = 0; o < NUM_OSCILLATORS; ++o) {
      int32_t maxHz = ampCompMaxFreqQ[o] >> FREQ_FRAC_BITS;
      if (maxHz > AMP_COMP_MAX_HZ) maxHz = AMP_COMP_MAX_HZ;

      int window = 0;

      for (int32_t hz = 0; hz <= maxHz; ++hz) {
          int32_t hz_q8 = hz << FREQ_FRAC_BITS;
          
          // Advance window when hz crosses into the next segment
          while (window < maxWindow && hz_q8 >= ampCompFrequencyArray[o][window + 2]) {
              ++window;
          }

          // High-precision Horner polynomial evaluation at exact integer Hz
          const double hzd = (double)hz;
          double val = ((double)floatCoeffs[o][window].a * hzd + 
                        (double)floatCoeffs[o][window].b) * hzd + 
                        (double)floatCoeffs[o][window].c;

          // Clamp bounds
          if (val < 0.0) val = 0.0;
          if (val > (double)DIV_COUNTER) val = (double)DIV_COUNTER;

          // Round to nearest integer PWM count
          ampCompLut[o][hz] = (uint16_t)lrint(val);
      }

      // Fill plateau region above maxHz
      for (int32_t hz = maxHz + 1; hz <= AMP_COMP_MAX_HZ; ++hz) {
          ampCompLut[o][hz] = (uint16_t)DIV_COUNTER;
      }
  }
}
// ---------------------------------------------------------------------------
// Runtime Fast Lookup Functions
// ---------------------------------------------------------------------------

/**
 * @brief Fast 32-bit Amplitude Compensation Engine
 * Operates on the 16-bit LUT directly, preserving sub-PWM LERP accuracy 
 * while saving 336 KB of RAM. ~8 clock cycles on Cortex-M33.
 */
static inline __attribute__((always_inline)) uint16_t SRAM_HOT(get_chan_level_q16_fast)(uint32_t freq_q16, uint8_t voiceN) {
    const uint32_t MAX_FREQ_Q16 = (AMP_COMP_MAX_HZ << 16) - 1;
    freq_q16 = (freq_q16 > MAX_FREQ_Q16) ? MAX_FREQ_Q16 : freq_q16;

    uint32_t idx  = freq_q16 >> 16;
    int32_t  frac = freq_q16 & 0xFFFF;

    const uint16_t* __restrict row = ampCompLut[voiceN];
    int32_t y0 = row[idx];
    int32_t y1 = row[idx + 1];

    int32_t diff = y1 - y0;
    return (uint16_t)(y0 + (((diff * frac) + 32768) >> 16));
}

static inline __attribute__((always_inline)) uint16_t SRAM_HOT(get_chan_level_q24_ultra_accurate)(int64_t freq_q24, uint8_t voiceN) {
    if (freq_q24 < 0) freq_q24 = 0;
    uint32_t freq_q16 = (uint32_t)(freq_q24 >> 8);
    return get_chan_level_q16_fast(freq_q16, voiceN);
}

#ifdef USE_FLOAT_AMP_COMP
static inline __attribute__((always_inline)) uint16_t SRAM_HOT(get_chan_level_by_method)(float freqHz, uint8_t voiceN) {
  switch (amp_comp_method) {
    case AMP_COMP_FIXED: {
      // 1-cycle conversion: Out-of-bounds or negative values are safely clamped inside get_chan_level_q16_fast
      const uint32_t freq_q16 = (freqHz <= 0.0f) ? 0 : (uint32_t)(freqHz * 65536.0f);
      return get_chan_level_q16_fast(freq_q16, voiceN);
    }

    case AMP_COMP_LUT:
      // Direct Dense 1Hz LUT
      return get_chan_level_lut(freqHz, voiceN);

    case AMP_COMP_FLOAT_QUAD:
    default:
      // Analytical Float Quadratic with hardware FMA (VFMA.F32)
      return get_chan_level_float_quad(freqHz, voiceN);
  }
}


/**
 * @brief Pure-float quadratic amp-comp (Hz domain). Live FLOAT_QUAD.
 * RP2350 TUNED: Single-check boundaries, do-while walking, and hardware FMA.
 */
 uint16_t SRAM_HOT(get_chan_level_float_quad)(float freqHz, uint8_t voiceN) {
  // Hoist 2D row pointers to avoid repeated memory offset math
  const float *__restrict hzRow = ampCompFrequencyHz[voiceN];
  
  // Domain early-outs
  if (freqHz <= hzRow[0]) {
    return (uint16_t)ampCompArray[voiceN][0];
  }

  if (freqHz >= ampCompMaxFreqHz[voiceN]) {
    return (uint16_t)DIV_COUNTER;
  }

  const uint32_t maxWindow = ampCompTableSize - 2;
  
  // Cast to unsigned. If ampWinCache is -1, it becomes 4294967295.
  uint32_t window = (uint32_t)ampWinCache[voiceN];

  // OPTIMIZATION 1: Single bounds check. 
  if (window <= maxWindow && freqHz >= hzRow[window] && freqHz < hzRow[window + 2]) {
  } else {
    int cand = (int)window;
    if ((uint32_t)cand > maxWindow) cand = 0; // Combined < 0 and > maxWindow check

    uint32_t steps = 0;

    // OPTIMIZATION 2: Do-While loops eliminate redundant loop-entry branch checks
    if (freqHz >= hzRow[cand + 2]) {
      do {
        ++cand;
        ++steps;
      } while (cand < (int)maxWindow && freqHz >= hzRow[cand + 2]);
    } 
    else if (freqHz < hzRow[cand]) {
      do {
        --cand;
        ++steps;
      } while (cand > 0 && freqHz < hzRow[cand]);
    }

    if ((uint32_t)cand <= maxWindow && freqHz >= hzRow[cand] && freqHz < hzRow[cand + 2]) {
      window = (uint32_t)cand;
    } else {
      window = 0;
      for (uint32_t i = 0; i <= maxWindow; ++i) {
        if (freqHz >= hzRow[i] && freqHz < hzRow[i + 2]) {
          window = i;
          break;
        }
      }
    }
    ampWinCache[voiceN] = (int16_t)window;
  }

  // OPTIMIZATION 3: Localize coefficient row (saves one multiply/offset instruction)
  const FloatQuadCoeffs *coeffRow = floatCoeffs[voiceN];
  const FloatQuadCoeffs &coeff = coeffRow[window];

  // OPTIMIZATION 4: Hardware Fused Multiply-Add (VFMA.F32)
  // Replaces standard math with guaranteed 1-cycle FPU intrinsic instructions
  float interpolatedValue = __builtin_fmaf(__builtin_fmaf(coeff.a, freqHz, coeff.b), freqHz, coeff.c);

  // Still natively rounding in 1 cycle
  return (uint16_t)(interpolatedValue + 0.5f);
}

uint16_t SRAM_HOT(get_chan_level_lut)(float freqHz, uint8_t voiceN) {
  // 1. Instant Float-to-Fixed Q16 conversion.
  // The FPU does this in 1 cycle. 
  // Out-of-bounds floats (or +Inf) automatically saturate in hardware.
  // ZERO FPU flag transfers = ZERO pipeline stalls.
  int32_t phase = (int32_t)(freqHz * 65536.0f);

  // 2. Pure Hardware Integer Clamping.
  // Max index is exactly 1 step below AMP_COMP_MAX_HZ so idx+1 is always valid.
  // The compiler turns this into 1-cycle IT (If-Then) blocks (CMP + MOVLT/MOVGT).
  const int32_t MAX_PHASE = (AMP_COMP_MAX_HZ << 16) - 1;
  phase = (phase < 0) ? 0 : phase;
  phase = (phase > MAX_PHASE) ? MAX_PHASE : phase;

  // 3. Extract Index and Fractional Weight (1-cycle bitwise ops)
  uint32_t idx = (uint32_t)phase >> 16;
  int32_t frac = phase & 0xFFFF;

  // 4. Explicit Aligned 16-bit Loads.
  // This perfectly bypasses the unaligned bus-matrix stalls of the previous version.
  const uint16_t* __restrict row = ampCompLut[voiceN];
  int32_t y0 = row[idx];
  int32_t y1 = row[idx + 1];

  // 5. Hardware single-cycle Multiply and Shift-Add
  // Resolves to ARM DSP instructions: y0 + ( (diff * frac) >> 16 )
  int32_t diff = y1 - y0;

  // Add 32768 (0.5 in Q16) to perfectly round to nearest integer
  return (uint16_t)(y0 + ((diff * frac + 32768) >> 16));
}

#endif // USE_FLOAT_AMP_COMP

/**
 * @brief Universal Float-Frequency Entry Point
 * Used by float-domain callers (voice_task_float, calibration routines, etc.)
 */
 static inline __attribute__((always_inline)) uint16_t SRAM_HOT(get_chan_level_for_engine)(float freqHz, uint8_t voiceN) {
  #ifdef USE_FLOAT_AMP_COMP
      // RP2350: Delegate to dynamic method switch (FLOAT_QUAD, LUT, or FIXED)
      return get_chan_level_by_method(freqHz, voiceN);
  #else
      // RP2040 / Fixed Build: Clean 1-cycle conversion straight to fast Q16 engine
      const uint32_t freq_q16 = (freqHz <= 0.0f) ? 0 : (uint32_t)(freqHz * 65536.0f);
      return get_chan_level_q16_fast(freq_q16, voiceN);
  #endif
  }

// Dispatch to the correct precompute routine
static inline void precompute_amp_comp_for_engine() {
  #ifdef USE_FLOAT_AMP_COMP
    precomputeCoefficients_float();
    amp_comp_seed_fixed_from_float_tables();
    precomputeCoefficients();
    fill_amp_comp_lut_from_quad();
  #else
    precomputeCoefficients();
    fill_amp_comp_lut_from_fixed(); // Fills ampCompLut cleanly from analytical curves
  #endif
  
    for (int o = 0; o < NUM_OSCILLATORS; ++o) ampWinCache[o] = -1;
    precompute_pw_tracking_cache();
  }
// ---------------------------------------------------------------------------
// Benchmarking Hooks
// ---------------------------------------------------------------------------
extern volatile bool amp_comp_bench_speed_pending;
extern volatile bool amp_comp_bench_accuracy_pending;
void print_amp_comp_bench();
void amp_comp_bench_run_speed();
void amp_comp_bench_run_accuracy();

#endif // __AMP_COMP_H__