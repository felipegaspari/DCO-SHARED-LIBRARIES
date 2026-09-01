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
// DIRECT POINTER HARDWARE ROUTING + ULTRA-FAST DMA DITHERING
// =============================================================================
#define PWM_DITHER_BITS 3                        // Divides wrap by 8
#define PWM_DITHER_STEPS (1 << PWM_DITHER_BITS)  // 8 steps

// Enforce strict 32-bit alignment to trigger ARM LDMIA/STMIA (Load/Store Multiple)
struct __attribute__((aligned(4))) DitherBlock { 
    uint32_t w[8]; 
};

// -----------------------------------------------------------------------------
// COMPILE-TIME BAYER MATRIX GENERATION (Zero Runtime Math Overhead)
// -----------------------------------------------------------------------------
constexpr DitherBlock generate_dither_lut(uint32_t a_rem, uint32_t b_rem) {
    DitherBlock blk = {0};
    blk.w[0] = (0 < a_rem) | ((0 < b_rem) << 16);
    blk.w[1] = (4 < a_rem) | ((4 < b_rem) << 16);
    blk.w[2] = (2 < a_rem) | ((2 < b_rem) << 16);
    blk.w[3] = (6 < a_rem) | ((6 < b_rem) << 16);
    blk.w[4] = (1 < a_rem) | ((1 < b_rem) << 16);
    blk.w[5] = (5 < a_rem) | ((5 < b_rem) << 16);
    blk.w[6] = (3 < a_rem) | ((3 < b_rem) << 16);
    blk.w[7] = (7 < a_rem) | ((7 < b_rem) << 16);
    return blk;
}

struct DitherLUT { DitherBlock table[8][8]; };

constexpr DitherLUT generate_full_lut() {
    DitherLUT lut = {};
    for(uint32_t a = 0; a < 8; a++) {
        for(uint32_t b = 0; b < 8; b++) {
            lut.table[a][b] = generate_dither_lut(a, b);
        }
    }
    return lut;
}

// Stored securely in Flash/L1 Cache (2 KB footprint)
constexpr DitherLUT dither_lut = generate_full_lut();


// -----------------------------------------------------------------------------
// HARDWARE ROUTER
// -----------------------------------------------------------------------------
struct PwmDirectRoute {
  volatile uint32_t* hw_cc; // Pointer to pwm_hw->slice[s].cc
  const uint16_t* src_a;    // Pointer to RANGE_PWM[x] / PW_PWM[x]
  const uint16_t* src_b;    // Pointer to RANGE_PWM[x] / PW_PWM[x]
  uint32_t* dma_buffer;     // Pointer to this slice's 8-word ring buffer
  uint8_t slice_num;        // Hardware slice number (0-11)
  int dma_chan;             // Assigned DMA channel (-1 if none available)
};

inline PwmDirectRoute active_voice_routes[12] = {};
inline uint8_t num_voice_routes = 0;
inline const uint16_t PWM_STATIC_ZERO = 0;

/**
 * ULTRA-FAST HARDWARE FLUSH 
 * Generates all dithering arrays for 12 routes in ~1 microsecond total at 250MHz.
 */
static inline __attribute__((always_inline)) 
void SRAM_HOT(flush_voice_pwm)() {
  
  // 1. Process all routing updates directly from the Cache
  _Pragma("GCC unroll 12")
  for (uint8_t i = 0; i < num_voice_routes; ++i) {
    uint16_t a_val = *active_voice_routes[i].src_a;
    uint16_t b_val = *active_voice_routes[i].src_b;
    
    uint32_t a_base = a_val >> PWM_DITHER_BITS;
    uint32_t b_base = b_val >> PWM_DITHER_BITS;
    uint32_t base_packed = a_base | (b_base << 16);
    
    if (__builtin_expect(active_voice_routes[i].dma_chan >= 0, 1)) {
        uint32_t a_rem = a_val & (PWM_DITHER_STEPS - 1);
        uint32_t b_rem = b_val & (PWM_DITHER_STEPS - 1);
        
        // Instant hardware pointer load to the pre-baked Bayer offsets
        const uint32_t* __restrict lut_ptr = dither_lut.table[a_rem][b_rem].w;
        
        // Assemble the 8-word block via registers
        DitherBlock blk;
        blk.w[0] = base_packed + lut_ptr[0];
        blk.w[1] = base_packed + lut_ptr[1];
        blk.w[2] = base_packed + lut_ptr[2];
        blk.w[3] = base_packed + lut_ptr[3];
        blk.w[4] = base_packed + lut_ptr[4];
        blk.w[5] = base_packed + lut_ptr[5];
        blk.w[6] = base_packed + lut_ptr[6];
        blk.w[7] = base_packed + lut_ptr[7];
        
        // Store Multiple (STMIA) directly into the aligned DMA SRAM Ring Buffer
        *((DitherBlock*)active_voice_routes[i].dma_buffer) = blk;
        
    } else {
        // Safe Direct-hardware fallback if we ran out of DMA channels
        *active_voice_routes[i].hw_cc = base_packed;
    }
  }

  // 2. Zero-Overhead DMA Reset
  // 0xFFFFFFFF lasts 8.4 hours, but we reset it safely every few seconds.
  // ++uint16_t takes exactly 1 CPU cycle and branch prediction assumes it fails.
  static uint16_t dma_reset_prescaler = 0;
  if (__builtin_expect(++dma_reset_prescaler == 0, 0)) {
      for (uint8_t i = 0; i < num_voice_routes; ++i) {
          int chan = active_voice_routes[i].dma_chan;
          if (chan >= 0) {
              dma_channel_abort(chan);
              dma_channel_set_read_addr(chan, active_voice_routes[i].dma_buffer, false);
              dma_channel_set_trans_count(chan, 0xFFFFFFFF, true);
          }
      }
  }
}

void print_dma_pwm_report();
void print_mcu_dma_map();
 #endif // DCO_PWM_H