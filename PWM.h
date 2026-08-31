/**
 * @file PWM.h
 * @brief Hardware PWM slice writers and optional PIO dithered range level control.
 */

 #ifndef DCO_PWM_H
 #define DCO_PWM_H
  
 
 #ifdef RANGE0_PIO_DITHER_TEST
 #include "range_pwm_dither.pio.h"
 #endif
 
 static constexpr uint32_t RANGE_PIO_FRAMES = 3;
 static constexpr uint32_t RANGE_PIO_PERIOD = DIV_COUNTER / RANGE_PIO_FRAMES;
 static constexpr uint32_t RANGE_PIO_LEVELS = RANGE_PIO_PERIOD * RANGE_PIO_FRAMES;

 void init_pwm();
 
 #ifdef RANGE0_PIO_DITHER_TEST
 void SRAM_HOT(init_range_pio_dither)();
 void SRAM_HOT(range_pio_set_level)(uint8_t osc, uint16_t level);
 #endif
 
 #ifdef ENABLE_CV_OUTS
 void init_cv_pwm();
 void SRAM_HOT(write_cv_pwm)();
 void SRAM_HOT(write_cv_pwm_raw)(uint16_t cutoff, const uint16_t resonance[NUM_FILTERS], uint16_t vca,
                       uint16_t dist_drive, uint16_t dist_mix);
 void init_level_pwm();
 void SRAM_HOT(write_level_pwm)();
 void SRAM_HOT(write_level_pwm_raw)(uint16_t osc1, uint16_t osc2, uint16_t osc3, uint16_t sub);
 #endif
 

 static inline __attribute__((always_inline)) 
 void SRAM_HOT(voice_write_pw)(uint8_t voice, uint16_t level) {
   uint8_t pin = PW_PINS[voice];
   if (__builtin_expect(pin == PW_PIN_UNASSIGNED, 0)) return;
 
   // FIXED: The channel (A=0, B=1) is strictly the 0th bit of the GPIO number.
   uint32_t chan = pin & 1u;
   
   pwm_set_chan_level(PW_PWM_SLICES[voice], chan, level);
 }


 /**
  * @brief Writes oscillator RANGE PWM level using either hardware PWM slice or PIO dither.
  * @param osc Oscillator index (0..NUM_OSCILLATORS-1).
  * @param level Counter compare level.
/**
/**
 * RP2350 OPTIMIZED: Forced Inline PWM Writer
 * Guarantees no function-call stack overhead in the main voice loop.
 */
 static inline __attribute__((always_inline)) 
 void SRAM_HOT(write_range_pwm)(uint8_t osc, uint16_t level) {
 #ifdef RANGE0_PIO_DITHER_TEST
   range_pio_set_level(osc, level);
 #else
   // Safely respects your exact Slice and Channel arrays without clobbering neighbors
   pwm_set_chan_level(RANGE_PWM_SLICES[osc], RANGE_PWM_CHANNELS[osc], level);
 #endif
 }
 


 void SRAM_HOT(voice_write_range_pair)(uint8_t dcoA, uint8_t dcoB, uint16_t chanA, uint16_t chanB) {
  if (char_amp_scale_q15) {
    // Evaluate delta exactly once
    const int32_t amp_j = character_amp_delta();
    
    write_range_pwm(dcoA, character_clamp_amp((int32_t)chanA + amp_j));
    write_range_pwm(dcoB, character_clamp_amp((int32_t)chanB + amp_j));
  } else {
    write_range_pwm(dcoA, chanA);
    write_range_pwm(dcoB, chanB);
  }
}

// =============================================================================
// DIRECT POINTER HARDWARE ROUTING
// =============================================================================
struct PwmDirectRoute {
  volatile uint32_t* hw_cc; // Pointer to pwm_hw->slice[s].cc
  const uint16_t* src_a;    // Direct pointer to RANGE_PWM[x], PW_PWM[x], or ZERO
  const uint16_t* src_b;    // Direct pointer to RANGE_PWM[x], PW_PWM[x], or ZERO
};

inline PwmDirectRoute active_voice_routes[12] = {};
inline uint8_t num_voice_routes = 0;
inline const uint16_t PWM_STATIC_ZERO = 0;

static inline __attribute__((always_inline))
uint32_t pack_pwm_cc(uint16_t chan_a, uint16_t chan_b) {
  return ((uint32_t)chan_b << 16) | (uint32_t)chan_a;
}

/**
 * FAST HARDWARE FLUSH: Reads pointers directly from memory, packs, and writes to HW.
 * Zero copies, zero branches, takes ~12-15 CPU cycles.
 */
static inline __attribute__((always_inline)) 
void SRAM_HOT(flush_voice_pwm)() {
#ifdef RANGE0_PIO_DITHER_TEST
  _Pragma("GCC unroll 8")
  for(uint8_t osc = 0; osc < NUM_OSCILLATORS; ++osc) {
    range_pio_set_level(osc, RANGE_PWM[osc]);
  }
#endif

  _Pragma("GCC unroll 12")
  for (uint8_t i = 0; i < num_voice_routes; ++i) {
    *active_voice_routes[i].hw_cc = pack_pwm_cc(*active_voice_routes[i].src_a, 
                                                *active_voice_routes[i].src_b);
  }
}
 #endif // DCO_PWM_H