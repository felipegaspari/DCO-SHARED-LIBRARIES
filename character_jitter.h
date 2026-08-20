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
 static inline int32_t character_pitch_delta_q24(void) {
   const int32_t s = char_pitch_scale_q15;
   if (!s) return 0;
   return (int32_t)(((int64_t)(int32_t)noiseLevel[1] * s) >> 15);
 }
 
 /**
  * @brief Computes amplitude compensation PWM count delta from white noise (noise0).
  */
 static inline int32_t character_amp_delta(void) {
   return ((int32_t)noiseLevel[0] * char_amp_scale_q15) >> 15;
 }
 
 /**
  * @brief Clamps amplitude PWM count to valid hardware slice counter range.
  */
 static inline uint16_t character_clamp_amp(int32_t level) {
   if (level < 0) return 0;
   if (level > (int32_t)DIV_COUNTER) return (uint16_t)DIV_COUNTER;
   return (uint16_t)level;
 }
 
 /**
  * @brief Computes pulse-width PWM count delta from white noise (noise0).
  */
 static inline int32_t character_pw_delta(void) {
   const int32_t s = char_pw_scale_q15;
   if (!s) return 0;
   return ((int32_t)noiseLevel[0] * s) >> 15;
 }
 
 #endif  // DCO_CHARACTER_JITTER_H