/**
 * @file clkdiv.h
 * @brief PIO clock divider cycle calculations across multiple numerical representations.
 * @details Computes total cycle divider values for the DCO PIO pitch generators from
 *          target frequencies in Q24 fixed-point or native floating-point Hz.
 *
 * Mode summary:
 *  - GOLD / GOLD_LIVE: Q24 -> double Hz -> llround(sys_hz / hz) [soft-double]
 *  - Q16 / CLKDIV_Q16: Q16 Hz -> 64/32-bit division (shipping precision: ±1/65536 Hz)
 *  - Q8 / CLKDIV_Q8:   Q8 Hz -> two 32/32-bit divisions with precise fallback (<16 Hz)
 *  - FAST_Q4:          Q4 Hz -> single 32/32-bit division
 *  - FLOAT:            Native float Hz -> fminf(sys_hz / hz + 0.5f, 4e9)
 */

 #ifndef DCO_CLKDIV_H
 #define DCO_CLKDIV_H
 
 #include <math.h>
 #include <stdint.h>
 
 /**
  * @brief Computes total PIO cycles from a floating-point frequency in double precision.
  * @param sys_hz System clock frequency in Hz (e.g., 125000000 or 240000000).
  * @param hz Target frequency in Hz.
  * @return Total PIO cycles clamped to [0, 4000000000].
  */
 static inline __attribute__((always_inline)) uint32_t clkdiv_gold_hz_total_cycles(uint32_t sys_hz, double hz) {
   if (!(hz > 0.0)) return 0;
   double cyc = (double)sys_hz / hz;
   if (cyc >= 4.0e9) return 4000000000u;
   if (cyc <= 0.0) return 0;
   return (uint32_t)llround(cyc);
 }
 
 /**
  * @brief GOLD reference calculation converting Q24 frequency to cycles via double-precision float.
  * @param sys_hz System clock frequency in Hz.
  * @param freq_q24 Target frequency in Q24 fixed-point format (Hz * 2^24).
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline)) uint32_t clkdiv_gold_total_cycles(uint32_t sys_hz, int64_t freq_q24) {
   if (freq_q24 <= 0) return 0;
   double hz = (double)freq_q24 * (1.0 / 16777216.0);
   return clkdiv_gold_hz_total_cycles(sys_hz, hz);
 }
 
 /**
  * @brief Q16 mode calculation using 64-bit / 32-bit integer division.
  * @details Rounds Q24 input to Q16 Hz before performing `((sys_hz << 16) + (freq_q16 / 2)) / freq_q16`.
  * @param sys_hz System clock frequency in Hz.
  * @param freq_q24 Target frequency in Q24 fixed-point format.
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline)) uint32_t clkdiv_q16_total_cycles(uint32_t sys_hz, int64_t freq_q24) {
   if (freq_q24 <= 0) return 0;
   uint32_t freq_q16 = (uint32_t)((freq_q24 + (int64_t)(1 << 7)) >> 8);
   if (freq_q16 == 0) freq_q16 = 1;
   uint64_t num = ((uint64_t)sys_hz << 16) + (uint64_t)(freq_q16 / 2u);
   return (uint32_t)(num / freq_q16);
 }
 
 /**
  * @brief Precise Q8 64/32-bit calculation used as a fallback for sub-16 Hz frequencies in Q8 mode.
  * @param sys_hz System clock frequency in Hz.
  * @param freq_q24 Target frequency in Q24 fixed-point format.
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline)) uint32_t clkdiv_precise_q8_total_cycles(uint32_t sys_hz, int64_t freq_q24) {
   if (freq_q24 <= 0) return 0;
   uint32_t freq_q8 = (uint32_t)((freq_q24 + (int64_t)(1 << 15)) >> 16);
   if (freq_q8 == 0) freq_q8 = 1;
   uint64_t num = ((uint64_t)sys_hz << 8) + (uint64_t)(freq_q8 / 2u);
   return (uint32_t)(num / freq_q8);
 }
 
 /**
  * @brief Fast Q8 mode calculation using two 32-bit divisions instead of 64-bit division.
  * @param sys_hz System clock frequency in Hz.
  * @param freq_q24 Target frequency in Q24 fixed-point format.
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline)) uint32_t clkdiv_q8_total_cycles(uint32_t sys_hz, int64_t freq_q24) {
   if (freq_q24 <= 0) return 0;
   uint32_t freq_q8 = (uint32_t)((freq_q24 + (int64_t)(1 << 15)) >> 16);
   if (freq_q8 == 0) freq_q8 = 1;
   uint32_t f_int = freq_q8 >> 8;
   if (f_int < 16u) {
     return clkdiv_precise_q8_total_cycles(sys_hz, freq_q24);
   }
   uint32_t f_frac = freq_q8 & 0xFFu;
   uint32_t q = sys_hz / f_int;
   uint32_t r = sys_hz - q * f_int;
   uint32_t qf = q * f_frac;
   uint32_t r8 = r << 8;
   uint32_t round = freq_q8 / 2u;
   if (r8 >= qf) {
     return q + (r8 - qf + round) / freq_q8;
   }
   uint32_t diff = qf - r8;
   if (round >= diff) {
     return q + (round - diff) / freq_q8;
   }
   uint32_t D = diff - round;
   return q - (D + freq_q8 - 1u) / freq_q8;
 }
 
 /**
  * @brief Ultra-fast Q4 mode calculation rounding to Q4 Hz and using 32-bit math.
  * @param sys_hz System clock frequency in Hz.
  * @param freq_q24 Target frequency in Q24 fixed-point format.
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline)) uint32_t clkdiv_fast_q4_total_cycles(uint32_t sys_hz, int64_t freq_q24) {
   if (freq_q24 <= 0) return 0;
   uint32_t freq_q4 = (uint32_t)((freq_q24 + (1LL << 19)) >> 20);
   if (freq_q4 == 0) freq_q4 = 1;
   return (sys_hz * 16u + (freq_q4 / 2u)) / freq_q4;
 }
 
 /**
  * @brief Single-precision floating-point cycle calculation.
  * @param sys_hz System clock frequency in Hz.
  * @param hz Target frequency in Hz (float).
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline)) uint32_t clkdiv_float_hz_total_cycles(uint32_t sys_hz, float hz) {
   if (!(hz > 0.0f)) return 0;
   return (uint32_t)fminf((float)sys_hz / hz + 0.5f, 4.0e9f);
 }
 
 /**
  * @brief Float mode calculation accepting Q24 input and converting to single-precision float.
  * @param sys_hz System clock frequency in Hz.
  * @param freq_q24 Target frequency in Q24 fixed-point format.
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline)) uint32_t clkdiv_float_total_cycles(uint32_t sys_hz, int64_t freq_q24) {
   if (freq_q24 <= 0) return 0;
   float hz = (float)freq_q24 * (1.0f / 16777216.0f);
   return clkdiv_float_hz_total_cycles(sys_hz, hz);
 }
 
 #if CLKDIV_MODE == CLKDIV_Q16
 #define clkdiv_live_total_cycles clkdiv_q16_total_cycles
 #elif CLKDIV_MODE == CLKDIV_Q8
 #define clkdiv_live_total_cycles clkdiv_q8_total_cycles
 #elif CLKDIV_MODE == CLKDIV_FAST_Q4
 #define clkdiv_live_total_cycles clkdiv_fast_q4_total_cycles
 #elif CLKDIV_MODE == CLKDIV_FLOAT
 #define clkdiv_live_total_cycles clkdiv_float_total_cycles
 #else
 #define clkdiv_live_total_cycles clkdiv_gold_total_cycles
 #endif
 
 /**
  * @brief Active live float-voice entry point.
  * @param sys_hz System clock frequency in Hz.
  * @param hz Target frequency in Hz.
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline))
 uint32_t clkdiv_live_hz_total_cycles(uint32_t sys_hz, float hz) {
 #if CLKDIV_MODE == CLKDIV_FLOAT
   return clkdiv_float_hz_total_cycles(sys_hz, hz);
 #elif CLKDIV_MODE == CLKDIV_GOLD
   return clkdiv_gold_hz_total_cycles(sys_hz, (double)hz);
 #else
   if (!(hz > 0.0f)) return 0;
   int64_t q24 = (int64_t)llround((double)hz * 16777216.0);
   return clkdiv_live_total_cycles(sys_hz, q24);
 #endif
 }
 
 #endif  // DCO_CLKDIV_H