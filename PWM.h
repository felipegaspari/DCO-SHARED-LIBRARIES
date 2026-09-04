/**
 * @file PWM.h
 * @brief Hardware PWM slice writers with fully automated compile-time DMA dithering.
 */

 #ifndef DCO_PWM_H
 #define DCO_PWM_H
 
 #include <stdint.h>
 #include "hardware/pwm.h"
 #include "hardware/dma.h"
 
 #ifdef RANGE0_PIO_DITHER_TEST
 #include "range_pwm_dither.pio.h"
 #endif

 
 // =============================================================================
 // MASTER CONFIGURATION
 // =============================================================================
 #define PWM_DITHER_BITS          3  // 4 = 16 steps (285.71 kHz carrier)
 #define PWM_DITHER_STEPS         (1 << PWM_DITHER_BITS)
 #define PWM_DITHER_BYTES         (PWM_DITHER_STEPS * 4)
 #define PWM_DMA_RING_SIZE_BITS   (PWM_DITHER_BITS + 2) 
 
 static constexpr uint32_t RANGE_PIO_FRAMES = 3;
 static constexpr uint32_t RANGE_PIO_PERIOD = DIV_COUNTER / RANGE_PIO_FRAMES;
 static constexpr uint32_t RANGE_PIO_LEVELS = RANGE_PIO_PERIOD * RANGE_PIO_FRAMES;
 extern const uint16_t PWM_STATIC_ZERO;
 extern uint8_t num_voice_routes;
 extern int8_t slice_to_route_map[NUM_PWM_SLICES];
 
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
   if (__builtin_expect(voice >= NUM_PW_CHANNELS, 0)) return;
   PW_PWM[voice] = level;
 }
 
 static inline __attribute__((always_inline)) 
 void SRAM_HOT(write_range_pwm)(uint8_t osc, uint16_t level) {
 #ifdef RANGE0_PIO_DITHER_TEST
   range_pio_set_level(osc, level);
 #else
   if (__builtin_expect(osc >= NUM_OSCILLATORS, 0)) return;
   RANGE_PWM[osc] = level;
 #endif
 }
 
 void SRAM_HOT(voice_write_range_pair)(uint8_t dcoA, uint8_t dcoB, uint16_t chanA, uint16_t chanB) {
   if (char_amp_scale_q15) {
     const int32_t amp_j = character_amp_delta();
     write_range_pwm(dcoA, character_clamp_amp((int32_t)chanA + amp_j));
     write_range_pwm(dcoB, character_clamp_amp((int32_t)chanB + amp_j));
   } else {
     write_range_pwm(dcoA, chanA);
     write_range_pwm(dcoB, chanB);
   }
 }
 
// -----------------------------------------------------------------------------
// DYNAMIC COMPILE-TIME BAYER BITMASK GENERATOR (Only 32 Bytes!)
// -----------------------------------------------------------------------------
constexpr uint32_t bayer_reverse(uint32_t x, uint32_t bits) {
  uint32_t res = 0;
  for (uint32_t b = 0; b < bits; b++) {
      if (x & (1u << b)) {
          res |= (1u << (bits - 1 - b));
      }
  }
  return res;
}

// Generates a 16-bit mask where bit 's' is 1 if step 's' gets an extra +1 dither
constexpr uint16_t build_bayer_mask(uint32_t rem) {
  uint16_t mask = 0;
  for (uint32_t s = 0; s < PWM_DITHER_STEPS; s++) {
      if (bayer_reverse(s, PWM_DITHER_BITS) < rem) {
          mask |= (1u << s);
      }
  }
  return mask;
}

struct BayerMaskTable { uint16_t m[PWM_DITHER_STEPS]; };

constexpr BayerMaskTable generate_mask_table() {
  BayerMaskTable t = {};
  for (uint32_t i = 0; i < PWM_DITHER_STEPS; i++) {
      t.m[i] = build_bayer_mask(i);
  }
  return t;
}

// This tiny 32-byte array fits instantly in the RP2350 L1 cache
constexpr BayerMaskTable bayer_masks = generate_mask_table();
 
// -----------------------------------------------------------------------------
// UNIFIED ROUTE STRUCTURE (Optimized 32-bit alignment)
// -----------------------------------------------------------------------------
struct PwmDirectRoute {
  volatile uint32_t* hw_cc;
  const uint16_t* src_a;
  const uint16_t* src_b;
  uint32_t* dma_buffer;
  int dma_chan;
  uint32_t last_ab;
  uint8_t slice_num;
};
 
extern PwmDirectRoute active_voice_routes[NUM_OSCILLATORS + NUM_PW_CHANNELS];
 
/**
* CACHE-AWARE HARDWARE FLUSH (Pure Ordered Dither)
*/
/**
* CACHE-AWARE HARDWARE FLUSH (Hybrid: DMA Dither for Range, Direct 10-bit for PW)
*/
static inline __attribute__((always_inline)) 
void SRAM_HOT(flush_voice_pwm)() {

  const uint8_t n_routes = num_voice_routes;

  for (uint8_t i = 0; i < n_routes; ++i) {
    const uint16_t a_val = *active_voice_routes[i].src_a;
    const uint16_t b_val = *active_voice_routes[i].src_b;
    const uint32_t current_ab = a_val | ((uint32_t)b_val << 16);
    
    // Early exit: If levels haven't changed, skip instantly
    if (__builtin_expect(current_ab == active_voice_routes[i].last_ab, 1)) {
        continue; 
    }
    
    active_voice_routes[i].last_ab = current_ab;
    const int chan = active_voice_routes[i].dma_chan;
    
    // =========================================================================
    // PATH 1: RANGE OSCILLATORS (DMA Dithered @ ~285 kHz)
    // =========================================================================
    if (__builtin_expect(chan >= 0, 1)) {
        const uint32_t base_packed = (a_val >> PWM_DITHER_BITS) | 
                                     (((uint32_t)b_val & ~((1u << PWM_DITHER_BITS) - 1)) << (16 - PWM_DITHER_BITS));
        
        const uint32_t a_rem = a_val & (PWM_DITHER_STEPS - 1);
        const uint32_t b_rem = b_val & (PWM_DITHER_STEPS - 1);
        
        uint32_t amask = bayer_masks.m[a_rem];
        uint32_t bmask = bayer_masks.m[b_rem];
        
        uint32_t* __restrict dest = active_voice_routes[i].dma_buffer;
        
        _Pragma("GCC unroll 16")
        for (uint32_t s = 0; s < PWM_DITHER_STEPS; s++) {
            dest[s] = base_packed + (amask & 1) + ((bmask & 1) << 16);
            amask >>= 1;
            bmask >>= 1;
        }
    } 
    // =========================================================================
    // PATH 2: PW CHANNELS (Direct 10-bit Hardware Write @ 146.5 kHz, Zero Noise)
    // =========================================================================
    else {
        // Writes both Channel A and Channel B 10-bit levels in 1 single CPU cycle!
        *active_voice_routes[i].hw_cc = current_ab;
    }
  }

  // Safe Auto-Reload loop (Only active on Range DMA channels)
  for (uint8_t i = 0; i < n_routes; ++i) {
      const int chan = active_voice_routes[i].dma_chan;
      if (chan >= 0 && __builtin_expect(dma_hw->ch[chan].transfer_count < 0x10000000, 0)) {
          dma_hw->ch[chan].read_addr = (uint32_t)active_voice_routes[i].dma_buffer;
          dma_hw->ch[chan].al1_transfer_count_trig = 0xFFFFFFFF;
      }
  }
}

/**
* @brief Instantly computes and flushes a single route
*/
static inline __attribute__((always_inline)) 
void SRAM_HOT(render_single_dma_route)(uint8_t route_idx) {
  PwmDirectRoute& route = active_voice_routes[route_idx];
  
  const uint16_t a_val = *route.src_a;
  const uint16_t b_val = *route.src_b;
  const uint32_t current_ab = a_val | ((uint32_t)b_val << 16);
  
  // Early Exit! If levels haven't changed, do nothing.
  if (__builtin_expect(current_ab == route.last_ab, 1)) {
      return; 
  }
  
  route.last_ab = current_ab;
  const int chan = route.dma_chan;

  if (__builtin_expect(chan >= 0, 1)) {
      // Range Dither Path
      const uint32_t base_packed = (a_val >> PWM_DITHER_BITS) | 
                                   (((uint32_t)b_val & ~((1u << PWM_DITHER_BITS) - 1)) << (16 - PWM_DITHER_BITS));
      
      const uint32_t a_rem = a_val & (PWM_DITHER_STEPS - 1);
      const uint32_t b_rem = b_val & (PWM_DITHER_STEPS - 1);
      
      uint32_t amask = bayer_masks.m[a_rem];
      uint32_t bmask = bayer_masks.m[b_rem];
      
      uint32_t* __restrict dest = route.dma_buffer;
      
      _Pragma("GCC unroll 16")
      for (uint32_t s = 0; s < PWM_DITHER_STEPS; s++) {
          dest[s] = base_packed + (amask & 1) + ((bmask & 1) << 16);
          amask >>= 1;
          bmask >>= 1;
      }
      
      if (__builtin_expect(dma_hw->ch[chan].transfer_count < 0x10000000, 0)) {
          dma_hw->ch[chan].read_addr = (uint32_t)dest;
          dma_hw->ch[chan].al1_transfer_count_trig = 0xFFFFFFFF;
      }
  } else {
      // PW Direct 10-bit write
      *route.hw_cc = current_ab;
  }
}


/**
 * @brief Ultra-fast O(1) reverse lookup
 */
 static inline int SRAM_HOT(get_route_for_slice)(uint8_t slice) {
  // __builtin_expect tells the compiler to optimize for the valid range.
  if (__builtin_expect(slice >= NUM_PWM_SLICES, 0)) return -1;
  return slice_to_route_map[slice];
}


/**
* @brief Flushes both Range PWMs for a given voice INSTANTLY to DMA.
*/
static inline void SRAM_HOT(flush_voice_range_pair)(uint8_t dcoA, uint8_t dcoB, uint16_t range_a, uint16_t range_b) {
  RANGE_PWM[dcoA] = range_a;
  RANGE_PWM[dcoB] = range_b;
  
  // Memory Barrier: Forces the compiler to finish the SRAM writes above 
  // before allowing the route loader to read them back via pointer indirection.
  // Prevents pipeline stalling.
  __asm__ volatile ("" : : : "memory");
  
  const uint8_t slice_a = RANGE_PWM_SLICES[dcoA];
  const int route_a = get_route_for_slice(slice_a);
  if (__builtin_expect(route_a >= 0, 1)) {
      render_single_dma_route((uint8_t)route_a);
  }
  
  const uint8_t slice_b = RANGE_PWM_SLICES[dcoB];
  if (slice_a != slice_b) {
      const int route_b = get_route_for_slice(slice_b);
      if (__builtin_expect(route_b >= 0, 1)) {
          render_single_dma_route((uint8_t)route_b);
      }
  }
}

/**
* @brief Flushes a single Pulse Width channel INSTANTLY to DMA.
*/
static inline void SRAM_HOT(flush_pw_channel)(uint8_t pw_idx, uint16_t level) {
  PW_PWM[pw_idx] = level;
  
  // Memory Barrier to prevent Read-After-Write CPU stall
  __asm__ volatile ("" : : : "memory");
  
  const int route = get_route_for_slice(PW_PWM_SLICES[pw_idx]);
  if (__builtin_expect(route >= 0, 1)) {
      render_single_dma_route((uint8_t)route);
  }
}

 void print_dma_pwm_report();
 void print_mcu_dma_map();
 
 #endif // DCO_PWM_H