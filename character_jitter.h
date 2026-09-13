/**
 * @file character_jitter.h
 * @brief Simplified, zero-overhead Character noise jitter scaling.
 */

#ifndef DCO_CHARACTER_JITTER_H
#define DCO_CHARACTER_JITTER_H

#include <stdint.h>

// =============================================================================
// FIXED JITTER PRESETS (0 .. 127)
// Tune the default character profile here.
// =============================================================================
static constexpr uint8_t FIXED_PITCH_JITTER = 30;   ///< 0..127 (80/127 of max semitone)
static constexpr uint8_t FIXED_AMP_JITTER   = 95;  ///< 0..127 (127 gives ~50% max amplitude swing)
static constexpr uint8_t FIXED_PW_JITTER    = 10;   ///< 0..127 (70 gives ~7% PW swing on 2048 wrap)

// Maximum pitch excursion in SEMITONES at full Character & Pitch Jitter
static constexpr float   CHAR_PITCH_MAX_SEMITONES = 2.0f; 

static constexpr float   CHAR_AMP_SCALE_FACTOR = 2.0f;

// Float step precomputed at compile time (Zero runtime division)
// Maps (0..127) * (0..127) * Q15 noise (/32768) into Float Octaves (/12)
static constexpr float PITCH_STEP_F = 
    (CHAR_PITCH_MAX_SEMITONES / 12.0f) / (127.0f * 127.0f * 32768.0f);

// Precomputed scale caches
inline float   char_pitch_scale_f        = 0.0f;
inline int32_t char_pitch_scale_q24_fast = 0;

/**
 * @brief Recalculates all jitter scales once at control rate (when knobs move).
 * Completely branchless math without nested function calls.
 */
static inline void character_recompute_scales(void) {
  if (character == 0) {
    char_pitch_scale_f        = 0.0f;
    char_pitch_scale_q24_fast = 0;
    char_amp_scale_q15        = 0;
    char_pw_scale_q15         = 0;
    return;
  }

  // Use individual parameter if set (>0), otherwise fall back to fixed preset
  const uint32_t p_val = pitchJitter        ? pitchJitter        : FIXED_PITCH_JITTER;
  const uint32_t a_val = ampCompJitter      ? ampCompJitter      : FIXED_AMP_JITTER;
  const uint32_t w_val = pulsewidthJitter   ? pulsewidthJitter   : FIXED_PW_JITTER;

  const uint32_t master = character;

  // 1. Amplitude Jitter: (0..127) * (0..127) directly yields 0 .. 16129 (~50% in Q15!)
  char_amp_scale_q15 = (int32_t)(master * a_val * CHAR_AMP_SCALE_FACTOR);

  // 2. Pitch Jitter: 1 float multiply converts product directly to float octave scale
  char_pitch_scale_f = (float)(master * p_val) * PITCH_STEP_F;

  // 3. Fixed-point Q24 pitch scale for RP2040 (32-bit hardware multiply friendly)
  // Max ~1 semitone in Q24 is ~1398101 counts
  char_pitch_scale_q24_fast = (int32_t)((master * p_val * 43) >> 3);

  // 4. Pulse Width Jitter: (master * w_val) >> 6 yields up to ~250 counts on a 2048 wrap
  char_pw_scale_q15 = (int32_t)((master * w_val) >> 6);
}

// =============================================================================
// AUDIO HOT-PATH DELTAS (1 - 2 instructions each, 0 branches, 0 divisions)
// =============================================================================

/**
 * @brief Hot-path Float Pitch Delta (RP2350).
 * Compiles to: 1 VCVT + 1 VMUL.F32 instruction.
 */
static inline __attribute__((always_inline))
float SRAM_HOT(character_pitch_delta_float)(void) {
  return (float)(int16_t)noiseLevel[0] * char_pitch_scale_f;
}

/**
 * @brief Hot-path Q24 Pitch Delta (RP2040).
 * Compiles to: 1 MULS + 1 ASRS instruction.
 */
static inline __attribute__((always_inline))
int32_t SRAM_HOT(character_pitch_delta_q24)(void) {
  return ((int32_t)(int16_t)noiseLevel[0] * char_pitch_scale_q24_fast) >> 7;
}

/**
 * @brief Normalized bipolar Q15 amplitude modulation factor (-16129 .. +16129).
 * Compiles to: 1 MULS + 1 ASRS instruction.
 */
static inline __attribute__((always_inline))
int32_t SRAM_HOT(character_amp_delta)(void) {
  const int32_t local_noise = (int32_t)(int16_t)noiseLevel[0];
  return (local_noise * char_amp_scale_q15) >> 15;
}

/**
 * @brief Hot-path Pulse-Width Delta.
 * Compiles to: 1 MULS + 1 ASRS instruction.
 */
static inline __attribute__((always_inline))
int32_t SRAM_HOT(character_pw_delta)(void) {
  return ((int32_t)(int16_t)noiseLevel[0] * char_pw_scale_q15) >> 15;
}

/**
 * @brief Branchless Range Amplitude Clamp.
 */
static inline __attribute__((always_inline)) 
uint16_t SRAM_HOT(character_clamp_amp)(int32_t level) {
  int32_t clamped = (level < 0) ? 0 : level;
  return (uint16_t)((clamped > (int32_t)DIV_COUNTER) ? (int32_t)DIV_COUNTER : clamped);
}

#endif  // DCO_CHARACTER_JITTER_H