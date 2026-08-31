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
 static inline __attribute__((always_inline)) SRAM_HOT(uint32_t clkdiv_gold_total_cycles_Hz)(uint32_t sys_hz, double hz) {
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
 static inline __attribute__((always_inline)) SRAM_HOT(uint32_t clkdiv_gold_total_cycles)(uint32_t sys_hz, int64_t freq_q24) {
   if (freq_q24 <= 0) return 0;
   double hz = (double)freq_q24 * (1.0 / 16777216.0);
   return clkdiv_gold_total_cycles_Hz(sys_hz, hz);
 }
 
 /**
 * @brief Ultra-Fast PIO Clock Cycle Calculator for RP2040 and RP2350
 * - RP2350: Hardware FPU VDIV.F32 (~16 cycles total vs ~80 cycles software int64 div)
 * - RP2040: SIO 64/32 Hardware Divider (~8 cycles) with fast bitshift rounding
 */
static inline __attribute__((always_inline)) SRAM_HOT(uint32_t clkdiv_q16_total_cycles)(uint32_t sys_hz, int64_t freq_q16) {
  if (__builtin_expect(freq_q16 == 0, 0)) return 0;
    // =========================================================================
    // UNIVERSAL PURE 32-BIT FIXED-POINT PATH (RP2040 & Generic MCUs)
    // =========================================================================
    
    // 1. Initial 32-bit division estimate (scaled by 2^12)
    // (sys_hz << 4) fits inside uint32_t for clocks up to 268 MHz
    const uint32_t den = (freq_q16 + 2048) >> 12;
    const uint32_t num32 = sys_hz << 4;
    uint32_t q = num32 / (den ? den : 1);

    // 2. Exact remainder via 64-bit multiplication (Single-cycle on M0+/M33/M4)
    const uint64_t target = ((uint64_t)sys_hz << 16) + (uint64_t)(freq_q16 >> 1);
    const uint64_t prod   = (uint64_t)q * (uint64_t)freq_q16;

    // 3. Exact 32-bit refinement step (Resolves the remainder in 32-bit math)
    if (prod < target) {
        const uint32_t rem = (uint32_t)(target - prod);
        q += rem / freq_q16;
    } else if (prod > target) {
        const uint32_t rem = (uint32_t)(prod - target);
        q -= (rem + freq_q16 - 1) / freq_q16;
    }

    return q;
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
 * RP2350 OPTIMIZED: Branchless Float Cycle Calculation
 * Relies on the Cortex-M33 hardware FPU and IT (If-Then) blocks.
 */
 static inline __attribute__((always_inline)) 
 uint32_t SRAM_HOT(clkdiv_float_total_cycles_Hz)(float sys_hz_f, float hz) {
     // ARM Cortex-M33 FPU processes sys_hz_f / 0.0f to +Infinity without crashing.
     // We let the FPU do the math unconditionally to avoid pipeline flushes.
     float cycles = (sys_hz_f / hz) + 0.5f;
     cycles = fminf(cycles, 4.0e9f);
     
     // Ternary operator forces GCC to emit a conditional MOV (IT block) instead of a branch
     return (hz > 0.0f) ? (uint32_t)cycles : 0; 
 }
 
 /**
  * @brief Float mode calculation accepting Q24 input and converting to single-precision float.
  * @param sys_hz System clock frequency in Hz.
  * @param freq_q24 Target frequency in Q24 fixed-point format.
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline)) SRAM_HOT(uint32_t clkdiv_float_total_cycles_Q24)(uint32_t sys_hz, int64_t freq_q24) {
   if (freq_q24 <= 0) return 0;
   float hz = (float)freq_q24 * (1.0f / 16777216.0f);
   return clkdiv_float_total_cycles_Hz(sys_hz, hz);
 }

 #if defined(USE_FLOAT_VOICE_TASK)

 #if CLKDIV_MODE == CLKDIV_Q16
 #define clkdiv_live_total_cycles clkdiv_q16_total_cycles
 #elif CLKDIV_MODE == CLKDIV_Q8
 #define clkdiv_live_total_cycles clkdiv_q8_total_cycles
 #elif CLKDIV_MODE == CLKDIV_FAST_Q4
 #define clkdiv_live_total_cycles clkdiv_fast_q4_total_cycles
 #elif CLKDIV_MODE == CLKDIV_FLOAT
 #define clkdiv_live_total_cycles clkdiv_float_total_cycles_Hz
 #else // CLKDIV_GOLD
 #define clkdiv_live_total_cycles clkdiv_gold_total_cycles_Hz
 #endif

 #else

 #if CLKDIV_MODE == CLKDIV_Q16
 #define clkdiv_live_total_cycles clkdiv_q16_total_cycles
 #elif CLKDIV_MODE == CLKDIV_Q8
 #define clkdiv_live_total_cycles clkdiv_q8_total_cycles
 #elif CLKDIV_MODE == CLKDIV_FAST_Q4
 #define clkdiv_live_total_cycles clkdiv_fast_q4_total_cycles
 #elif CLKDIV_MODE == CLKDIV_FLOAT
 #define clkdiv_live_total_cycles clkdiv_float_total_cycles_Q24
 #else
 #define clkdiv_live_total_cycles clkdiv_gold_total_cycles

 #endif
 #endif
 
 /**
  * @brief Active live float-voice entry point.
  * @param sys_hz System clock frequency in Hz.
  * @param hz Target frequency in Hz.
  * @return Total PIO clock cycles.
  */
 static inline __attribute__((always_inline))
 SRAM_HOT(uint32_t clkdiv_live_hz_total_cycles)(uint32_t sys_hz, float hz) {
 #if CLKDIV_MODE == CLKDIV_FLOAT
   return clkdiv_float_total_cycles_Hz(sys_hz, hz);
 #elif CLKDIV_MODE == CLKDIV_GOLD
   return clkdiv_gold_total_cycles_Hz(sys_hz, (double)hz);
 #else
   if (!(hz > 0.0f)) return 0;
   int64_t q24 = (int64_t)llround((double)hz * 16777216.0);
   return clkdiv_live_total_cycles(sys_hz, q24);
 #endif
 }
 
 #endif  // DCO_CLKDIV_H