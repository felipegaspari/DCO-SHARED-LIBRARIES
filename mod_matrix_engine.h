/**
 * @file mod_matrix_engine.h
 * @brief High-Precision Polyphonic Modulation Matrix Engine (Header-Only).
 * Optimized for RP2350 ARM Cortex-M33 (Fixed & Float Engine Compatible).
 */

 #ifndef __SHARED_MOD_MATRIX_ENGINE_H__
 #define __SHARED_MOD_MATRIX_ENGINE_H__
 
 #include <string.h>
 
 #ifndef SRAM_HOT
 #define SRAM_HOT(fn) fn
 #endif
 
 // =============================================================================
 // 1. POLYPHONY SIZING
 // =============================================================================
 #if defined(NUM_VOICES_TOTAL)
 #define MAX_SUPPORTED_VOICES NUM_VOICES_TOTAL
 #else
 #define MAX_SUPPORTED_VOICES 4
 #endif
 
 // =============================================================================
 // 2. DATA STRUCTURES (Hybrid SoA Layout)
 // =============================================================================
 struct ModSources {
     // Globals (Assigned ONCE per cycle)
     int16_t lfo1;
     int16_t lfo2;
     int16_t lfo3;
     int16_t noise;
     int16_t pitch_bend;
     int16_t drift_global;
     int16_t expression;
     int16_t breath;
 
     // Voice-Specific (Assigned in a tight loop)
     int16_t env_vcf[MAX_SUPPORTED_VOICES];
     int16_t drift_voice[MAX_SUPPORTED_VOICES];
     int16_t env_vca[MAX_SUPPORTED_VOICES];
     int16_t env_dco[MAX_SUPPORTED_VOICES];
     uint8_t velocity[MAX_SUPPORTED_VOICES];
     uint8_t keytrack_note[MAX_SUPPORTED_VOICES];
 };
 
 struct ModSlot {
     uint8_t src;   
     uint8_t dest;  
     int16_t depth; 
 };
 
 // =============================================================================
 // 3. SHARED MATRIX STATE & BUFFERS
 // =============================================================================
 extern SRAM_DATA ModSlot mod_slots[8];
 extern SRAM_DATA alignas(8) int32_t voice_mod_sums[MAX_SUPPORTED_VOICES][MOD_DEST_COUNT];
 extern SRAM_DATA alignas(8) int32_t prev_depth_mods[MAX_SUPPORTED_VOICES][8];
 
 extern SRAM_DATA int16_t aftertouch_q15;
 extern SRAM_DATA int16_t mod_wheel_q15;
 extern SRAM_DATA int16_t expression_q15;
 extern SRAM_DATA int16_t breath_q15;
 extern SRAM_DATA int16_t random_sh_q15[MAX_SUPPORTED_VOICES];
 
 // Hardware delta buffers
 #if defined(USE_FLOAT_VOICE_TASK) || defined(STM32H7) || defined(STM32H7xx) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
 extern volatile float matrix_pitch_mod_f[MAX_SUPPORTED_VOICES];
 extern volatile float matrix_osc1_pitch_mod_f[MAX_SUPPORTED_VOICES];
 extern volatile float matrix_osc2_pitch_mod_f[MAX_SUPPORTED_VOICES];
 #endif
 
 extern volatile int32_t matrix_pitch_mod_q24[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_osc1_pitch_mod_q24[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_osc2_pitch_mod_q24[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_pw_mod[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_xmod_mod[MAX_SUPPORTED_VOICES];
 
 static inline int32_t mod_clamp_4095(int32_t v) {
 #if defined(__ARM_FEATURE_SAT)
     return __builtin_arm_ssat(v, 13);
 #else
     if (v < -4095) return -4095;
     if (v > 4095) return 4095;
     return v;
 #endif
 }
 
 // =============================================================================
 // 4. CORE ENGINE LIFECYCLE & SETTERS
 // =============================================================================
 inline void mod_matrix_init(uint8_t total_voices) {
     memset(mod_slots, 0, sizeof(mod_slots));
     memset(voice_mod_sums, 0, sizeof(voice_mod_sums));
     for (int i = 0; i < total_voices && i < MAX_SUPPORTED_VOICES; i++) {
         random_sh_q15[i] = 0;
         for (int s = 0; s < 8; s++) prev_depth_mods[i][s] = 0;
     }
 }
 
 inline void mod_matrix_init() {
     mod_matrix_init(MAX_SUPPORTED_VOICES);
 }
 
 void SRAM_HOT(mod_matrix_set_source)(uint8_t slot, int16_t src) { 
     if (slot < 8) mod_slots[slot].src = (src < 0 || src >= MOD_SRC_COUNT) ? SRC_OFF : (uint8_t)src; 
 }
 
 void SRAM_HOT(mod_matrix_set_dest)(uint8_t slot, int16_t dest) { 
     if (slot < 8) mod_slots[slot].dest = (dest < 0 || dest >= MOD_DEST_COUNT) ? 0xFF : (uint8_t)dest; 
 }
 
 void SRAM_HOT(mod_matrix_set_depth)(uint8_t slot, int16_t depth) { 
     if (slot < 8) mod_slots[slot].depth = depth; 
 }
 
 void SRAM_HOT(mod_matrix_set_aftertouch)(uint8_t at) {
     aftertouch_q15 = (int16_t)(((int32_t)at * 32767) / 127);
 }
 
 void SRAM_HOT(mod_matrix_set_mod_wheel)(uint8_t mw) {
     mod_wheel_q15 = (int16_t)(((int32_t)mw * 32767) / 127);
 }
 
 void SRAM_HOT(mod_matrix_set_expression)(uint8_t expr) {
     expression_q15 = (int16_t)(((int32_t)expr * 32767) / 127);
 }
 
 void SRAM_HOT(mod_matrix_set_breath)(uint8_t brt) {
     breath_q15 = (int16_t)(((int32_t)brt * 32767) / 127);
 }
 
 void SRAM_HOT(mod_matrix_on_note_on)(uint8_t voice) {
     if (voice < MAX_SUPPORTED_VOICES) {
         random_sh_q15[voice] = (int16_t)random(-32768, 32767);
     }
 }
 
 void SRAM_HOT(mod_matrix_clear_voice)(uint8_t voice) {
     if (voice < MAX_SUPPORTED_VOICES) {
         memset(voice_mod_sums[voice], 0, sizeof(int32_t) * MOD_DEST_COUNT);
     }
 }
 
 // =============================================================================
 // 5. PARAMETER ROUTER APPLIERS
 // =============================================================================
 #define DECL_MOD_SLOT_APPLIER_SET(N) \
     static inline void apply_param_mod_slot##N##_source(int16_t v) { mod_matrix_set_source(N, v); } \
     static inline void apply_param_mod_slot##N##_dest(int16_t v)   { mod_matrix_set_dest(N, v); }   \
     static inline void apply_param_mod_slot##N##_depth(int16_t v)  { mod_matrix_set_depth(N, v); }
 
 DECL_MOD_SLOT_APPLIER_SET(0)
 DECL_MOD_SLOT_APPLIER_SET(1)
 DECL_MOD_SLOT_APPLIER_SET(2)
 DECL_MOD_SLOT_APPLIER_SET(3)
 DECL_MOD_SLOT_APPLIER_SET(4)
 DECL_MOD_SLOT_APPLIER_SET(5)
 DECL_MOD_SLOT_APPLIER_SET(6)
 DECL_MOD_SLOT_APPLIER_SET(7)
 #undef DECL_MOD_SLOT_APPLIER_SET
 
 // =============================================================================
 // 6. REALTIME ACCUMULATOR CORE
 // =============================================================================
 static const int16_t VOICE_ID_SPREAD_MAP[4][4] = {
     {-32768, 0, 0, 0},
     {-32768, 32767, 0, 0},
     {-32768, 0, 32767, 0},
     {-32768, -10923, 10922, 32767}
 };
 
 __attribute__((always_inline)) static inline int32_t fast_mod_clamp(int32_t v) {
 #if defined(__ARM_FEATURE_SAT)
     return __builtin_arm_ssat(v, 13); 
 #else
     v = (v > 4095) ? 4095 : v;
     return (v < -4096) ? -4096 : v;
 #endif
 }
 
 template <uint8_t NV>
 __attribute__((always_inline)) static inline void mm_slot_accum_core(
     int32_t (* __restrict sums)[MOD_DEST_COUNT],
     const int32_t (* __restrict prevs_in)[8],
     const ModSources* __restrict sources,
     uint8_t spread_idx) 
 {
     for (uint8_t i = 0; i < 8; i++) {
         const uint8_t src_id = mod_slots[i].src;
         if (src_id == SRC_OFF) continue; 
         
         const uint8_t dest = mod_slots[i].dest;
         if (dest >= MOD_DEST_COUNT) continue;
 
         const int32_t base_depth = mod_slots[i].depth;
 
         #define MACRO_ACCUM_VOICES(SRC_EXPR) \
             _Pragma("GCC unroll 8") \
             for (uint8_t v = 0; v < NV; v++) { \
                 int32_t depth = fast_mod_clamp(base_depth + prevs_in[v][i]); \
                 sums[v][dest] += ((SRC_EXPR) * depth); \
             }
 
         switch (src_id) {
             case SRC_LFO1:        MACRO_ACCUM_VOICES(sources->lfo1); break;
             case SRC_LFO2:        MACRO_ACCUM_VOICES(sources->lfo2); break;
             case SRC_LFO3:        MACRO_ACCUM_VOICES(sources->lfo3); break;
             case SRC_NOISE:       MACRO_ACCUM_VOICES(sources->noise); break;
             case SRC_ENV_VCA:     MACRO_ACCUM_VOICES(sources->env_vca[v]); break;
             case SRC_ENV_VCF:     MACRO_ACCUM_VOICES(sources->env_vcf[v]); break;
             case SRC_ENV_DCO:     MACRO_ACCUM_VOICES(sources->env_dco[v]); break;
             case SRC_MODWHEEL:    MACRO_ACCUM_VOICES(mod_wheel_q15); break;
             case SRC_AFTERTC:     MACRO_ACCUM_VOICES(aftertouch_q15); break;
             case SRC_EXPRESSION:  MACRO_ACCUM_VOICES(sources->expression); break;
             case SRC_BREATH:      MACRO_ACCUM_VOICES(sources->breath); break;
             case SRC_VELOCITY:    MACRO_ACCUM_VOICES((int32_t)sources->velocity[v] * 258); break; 
             case SRC_BEND:        MACRO_ACCUM_VOICES((int32_t)sources->pitch_bend << 2); break;
             case SRC_DRIFT:       MACRO_ACCUM_VOICES(sources->drift_global); break;
             case SRC_KEYTRACK:    MACRO_ACCUM_VOICES((int32_t)(sources->keytrack_note[v] - 60) * 546); break; 
             case SRC_DRIFT_VOICE: MACRO_ACCUM_VOICES(sources->drift_voice[v]); break;
             case SRC_RANDOM_SH:   MACRO_ACCUM_VOICES(random_sh_q15[v]); break;
             case SRC_VOICE_ID:    MACRO_ACCUM_VOICES(VOICE_ID_SPREAD_MAP[spread_idx][(v < 4) ? v : 3]); break;
         }
         #undef MACRO_ACCUM_VOICES
     }
 }
 
 void SRAM_HOT(mod_matrix_accumulate_all)(const ModSources* __restrict sources, uint8_t num_voices) {
     if (num_voices == 0) return;
     if (num_voices > MAX_SUPPORTED_VOICES) num_voices = MAX_SUPPORTED_VOICES;
 
     uint8_t spread_idx;
     {
         BENCH_BEGIN(mm_setup);
         
         for (uint8_t v = 0; v < num_voices; v++) {
             int64_t* __restrict row64 = (int64_t*)voice_mod_sums[v];
             constexpr uint8_t double_words = MOD_DEST_COUNT / 2;
             
             _Pragma("GCC unroll 32") 
             for (uint8_t d = 0; d < double_words; d++) {
                 row64[d] = 0;
             }
             
             if constexpr (MOD_DEST_COUNT % 2 != 0) {
                 voice_mod_sums[v][MOD_DEST_COUNT - 1] = 0;
             }
         }
         
         spread_idx = __builtin_arm_usat(num_voices - 1, 2);
         BENCH_END(mm_setup);
     }
 
     int32_t (* __restrict sums)[MOD_DEST_COUNT] = voice_mod_sums;
     const int32_t (* __restrict prevs_in)[8] = (const int32_t(*)[8])prev_depth_mods;
 
     // =========================================================================
     // 2. Polyphonic Multiply-Accumulate Loop Dispatcher
     // =========================================================================
     {
         BENCH_BEGIN(mm_slot_accum);
         
         switch (num_voices) {
             case 1: mm_slot_accum_core<1>(sums, prevs_in, sources, spread_idx); break;
             case 2: mm_slot_accum_core<2>(sums, prevs_in, sources, spread_idx); break;
             case 3: mm_slot_accum_core<3>(sums, prevs_in, sources, spread_idx); break;
             case 4: mm_slot_accum_core<4>(sums, prevs_in, sources, spread_idx); break;
 #if MAX_SUPPORTED_VOICES >= 5
             case 5: mm_slot_accum_core<5>(sums, prevs_in, sources, spread_idx); break;
 #endif
 #if MAX_SUPPORTED_VOICES >= 6
             case 6: mm_slot_accum_core<6>(sums, prevs_in, sources, spread_idx); break;
 #endif
 #if MAX_SUPPORTED_VOICES >= 7
             case 7: mm_slot_accum_core<7>(sums, prevs_in, sources, spread_idx); break;
 #endif
 #if MAX_SUPPORTED_VOICES >= 8
             case 8: mm_slot_accum_core<8>(sums, prevs_in, sources, spread_idx); break;
 #endif
             default: break; 
         }
         
         BENCH_END(mm_slot_accum);
     }
 
     // =========================================================================
     // 3. Depth Slot Feedback Extraction
     // =========================================================================
     {
         BENCH_BEGIN(mm_depth_feedback);
         int32_t (* __restrict prevs_out)[8] = (int32_t(*)[8])prev_depth_mods;
 
         for (uint8_t v = 0; v < num_voices; v++) {
             const int32_t* __restrict v_sums = &sums[v][DEST_MOD_SLOT0_DEPTH];
             int32_t* __restrict v_prev = prevs_out[v];
 
             _Pragma("GCC unroll 8")
             for (uint8_t i = 0; i < 8; i++) {
                 v_prev[i] = v_sums[i] >> 15;
             }
         }
         BENCH_END(mm_depth_feedback);
     }
 }
 
 // =============================================================================
 // 7. HARDWARE DELTA ACCESSORS
 // =============================================================================
 
 template <uint8_t dest>
 static float SRAM_HOT(mod_matrix_get_dest_float)(uint8_t voice) {
     if constexpr (dest >= MOD_DEST_COUNT) return 0.0f;
     const int32_t raw_sum = voice_mod_sums[voice][dest];
 
     if constexpr (dest == DEST_PITCH || dest == DEST_OSC1_PITCH || dest == DEST_OSC2_PITCH) {
         static constexpr float RAW_TO_OCTAVE = 1.0f / 67108864.0f; 
         return (float)raw_sum * RAW_TO_OCTAVE;
 
     } else if constexpr (dest == DEST_PW) {
         static constexpr float RAW_TO_PW = 1.0f / 262144.0f; 
         return (float)raw_sum * RAW_TO_PW;
 
     } else if constexpr (dest == DEST_VCA_LEVEL) {
         static constexpr float RAW_TO_VCA = 1.0f / 16384.0f; 
         return (float)raw_sum * RAW_TO_VCA;
 
     } else if constexpr (dest == DEST_ENV_TO_VCF     || dest == DEST_ENV_TO_VCA   || 
                          dest == DEST_LFO1_DEPTH     || dest == DEST_LFO2_DEPTH   || 
                          dest == DEST_ENV_VCF_ATTACK || dest == DEST_ENV_VCF_DECAY || 
                          dest == DEST_ENV_VCA_ATTACK || dest == DEST_ENV_VCA_DECAY || 
                          dest == DEST_ENV_ALL_TIME   || dest == DEST_CROSSMOD_DEPTH) {
         static constexpr float RAW_TO_ENV_TIME = 1.0f / 4096.0f; 
         return (float)raw_sum * RAW_TO_ENV_TIME;
 
     } else {
         static constexpr float RAW_TO_12BIT = 1.0f / 32768.0f; 
         return (float)raw_sum * RAW_TO_12BIT;
     }
 }
 
 template <uint8_t dest>
 int32_t SRAM_HOT(mod_matrix_get_dest_fast)(uint8_t voice) {
     if constexpr (dest >= MOD_DEST_COUNT) return 0;
     const int32_t raw_sum = voice_mod_sums[voice][dest];
 
     if constexpr (dest == DEST_PITCH || dest == DEST_OSC1_PITCH || dest == DEST_OSC2_PITCH) {
         return raw_sum >> 2; 
     } else if constexpr (dest == DEST_PW) {
         return raw_sum >> 18; 
     } else if constexpr (dest == DEST_VCF_RESO) {
         return raw_sum >> 16;
     } else if constexpr (dest == DEST_VCA_LEVEL) {
         return raw_sum >> 14;
     } else if constexpr (dest == DEST_LFO1_SPEED || dest == DEST_LFO2_SPEED || dest == DEST_LFO3_SPEED) {
         return raw_sum >> 15;
     } else if constexpr (dest == DEST_ENV_TO_VCF     || dest == DEST_ENV_TO_VCA   || 
                          dest == DEST_LFO1_DEPTH     || dest == DEST_LFO2_DEPTH   || 
                          dest == DEST_ENV_VCF_ATTACK || dest == DEST_ENV_VCF_DECAY || 
                          dest == DEST_ENV_VCA_ATTACK || dest == DEST_ENV_VCA_DECAY || 
                          dest == DEST_ENV_ALL_TIME   || dest == DEST_CROSSMOD_DEPTH) {
         return raw_sum >> 12;
     } else {
         return raw_sum >> 15;
     }
 }
 
 int32_t SRAM_HOT(mod_matrix_get_dest)(uint8_t voice, uint8_t dest) {
     if (voice >= MAX_SUPPORTED_VOICES || dest >= MOD_DEST_COUNT) return 0;
     const int32_t raw_sum = voice_mod_sums[voice][dest];
 
     switch ((ModDest)dest) {
         case DEST_PITCH:
         case DEST_OSC1_PITCH:
         case DEST_OSC2_PITCH:     return raw_sum >> 2;
         case DEST_PW:             return raw_sum >> 18;
         case DEST_VCF_RESO:       return raw_sum >> 16;
         case DEST_VCA_LEVEL:      return raw_sum >> 14;
         case DEST_ENV_TO_VCF:
         case DEST_ENV_TO_VCA:
         case DEST_LFO1_DEPTH:
         case DEST_LFO2_DEPTH:
         case DEST_ENV_VCF_ATTACK:
         case DEST_ENV_VCF_DECAY:
         case DEST_ENV_VCA_ATTACK:
         case DEST_ENV_VCA_DECAY:
         case DEST_ENV_ALL_TIME:   
         case DEST_CROSSMOD_DEPTH: return raw_sum >> 12;
         default:                  return raw_sum >> 15;
     }
 }
 
 // =============================================================================
 // 8. VOICE ENGINE BRIDGE
 // =============================================================================
 void SRAM_HOT(mod_matrix_publish_to_voices)(uint8_t num_voices) {
     if (num_voices > MAX_SUPPORTED_VOICES) num_voices = MAX_SUPPORTED_VOICES;
 
     _Pragma("GCC unroll 4")
     for (uint8_t v = 0; v < num_voices; v++) {
 #if defined(USE_FLOAT_VOICE_TASK) || defined(STM32H7) || defined(STM32H7xx) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
         matrix_pitch_mod_f[v]      = mod_matrix_get_dest_float<DEST_PITCH>(v);
         matrix_osc1_pitch_mod_f[v] = mod_matrix_get_dest_float<DEST_OSC1_PITCH>(v);
         matrix_osc2_pitch_mod_f[v] = mod_matrix_get_dest_float<DEST_OSC2_PITCH>(v);
 #else
         matrix_pitch_mod_q24[v]      = mod_matrix_get_dest_fast<DEST_PITCH>(v);
         matrix_osc1_pitch_mod_q24[v] = mod_matrix_get_dest_fast<DEST_OSC1_PITCH>(v);
         matrix_osc2_pitch_mod_q24[v] = mod_matrix_get_dest_fast<DEST_OSC2_PITCH>(v);
 #endif
         matrix_pw_mod[v]   = mod_matrix_get_dest_fast<DEST_PW>(v);
         matrix_xmod_mod[v] = mod_matrix_get_dest_fast<DEST_CROSSMOD_DEPTH>(v);
     }
 }
 
 // 1-Cycle Native Hardware Saturation
 uint16_t SRAM_HOT(apply_mod_dir_12b)(uint16_t base, int32_t mod_delta) {
 #if defined(__ARM_FEATURE_SAT)
     return (uint16_t)__builtin_arm_usat((int32_t)base + mod_delta, 12);
 #else
     int32_t calc = (int32_t)base + mod_delta; 
     if (calc < 0) return 0;
     if (calc > 4095) return 4095;
     return (uint16_t)calc;
 #endif
 }
 
 uint16_t SRAM_HOT(apply_mod_inv_12b)(uint16_t base, int32_t mod_delta) {
 #if defined(__ARM_FEATURE_SAT)
     return (uint16_t)__builtin_arm_usat((int32_t)base - mod_delta, 12);
 #else
     int32_t calc = (int32_t)base - mod_delta;
     if (calc < 0) return 0;
     if (calc > 4095) return 4095;
     return (uint16_t)calc;
 #endif
 }
 
 #endif // __SHARED_MOD_MATRIX_ENGINE_H__