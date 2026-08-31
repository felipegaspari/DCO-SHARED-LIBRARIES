/**
 * @file character_jitter.h
 * @brief Character noise jitter scaling and hot-path delta calculations.
 */

 #ifndef DCO_CHARACTER_JITTER_H
 #define DCO_CHARACTER_JITTER_H
 
 #include <stdint.h>
 
 static constexpr int32_t CHAR_JITTER_PITCH_MAX_Q24 = (1 << 24) / 20;  ///< ±0.05 oct maximum pitch jitter.
 static constexpr int32_t CHAR_JITTER_AMP_MAX       = (int32_t)DIV_COUNTER / 20;  ///< ±5% maximum amplitude jitter.
 static constexpr int32_t CHAR_JITTER_PW_MAX        = (int32_t)DIV_COUNTER_PW / 10;  ///< ±10% maximum PW jitter.
 
 /**
  * @brief Computes Q15 scale gain for an individual jitter axis.
  */
 static inline int32_t character_axis_scale(uint8_t axis_jitter, int32_t max_delta) {
   if (character == 0 || axis_jitter == 0 || max_delta == 0) {
     return 0;
   }
   const uint8_t eff = (uint8_t)(((uint16_t)axis_jitter * (uint16_t)character) >> 7);
   return (max_delta * (int32_t)eff) >> 7;
 }
 
 /**
  * @brief Recalculates pitch, amplitude, and pulse-width jitter scales when Character parameters change.
  */
 static inline void character_recompute_scales(void) {
   char_pitch_scale_q15 = character_axis_scale(pitchJitter, CHAR_JITTER_PITCH_MAX_Q24);
   char_amp_scale_q15   = character_axis_scale(ampCompJitter, CHAR_JITTER_AMP_MAX);
   char_pw_scale_q15    = character_axis_scale(pulsewidthJitter, CHAR_JITTER_PW_MAX);
 }
 
 /**
  * @brief Computes per-frame Q24 pitch modulation delta from pink noise (noise1).
  */
 static int32_t SRAM_HOT(character_pitch_delta_q24)(void) {
   const int32_t s = char_pitch_scale_q15;
   const int32_t local_noise = (int32_t)noiseLevel[1];
   if (!s) return 0;
   return (int32_t)(((int64_t)(int32_t)local_noise * s) >> 15);
 }

 static float SRAM_HOT(character_pitch_delta_float)(void) {
   const float s = char_pitch_scale_q15;
   const float local_noise = (float)noiseLevel[1];
   if (!s) return 0;
   return local_noise * s;
 }
 
 /**
  * @brief Computes amplitude compensation PWM count delta from white noise (noise0).
  */
 static int32_t SRAM_HOT(character_amp_delta)(void) {
  const int32_t local_noise = (int32_t)noiseLevel[0];
   return ((int32_t)local_noise * char_amp_scale_q15) >> 15;
 }
 
/**
 * RP2350 OPTIMIZED: Branchless Clamp
 * Executes unconditionally in 2 cycles using IT (If-Then) blocks.
 */
 static inline __attribute__((always_inline)) 
 uint16_t SRAM_HOT(character_clamp_amp)(int32_t level) {
   int32_t clamped = (level < 0) ? 0 : level;
   return (uint16_t)((clamped > (int32_t)DIV_COUNTER) ? (int32_t)DIV_COUNTER : clamped);
 }
 
 
 /**
  * @brief Computes pulse-width PWM count delta from white noise (noise0).
  */
 static int32_t SRAM_HOT(character_pw_delta)(void) {
   const int32_t s = char_pw_scale_q15;
   const int32_t local_noise = (int32_t)noiseLevel[0];
   if (!s) return 0;
   return ((int32_t)local_noise * s) >> 15;
 }
 
 #endif  // DCO_CHARACTER_JITTER_H