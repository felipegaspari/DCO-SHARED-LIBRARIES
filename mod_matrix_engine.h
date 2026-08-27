/**
 * @file mod_matrix_engine.h
 * @brief High-Precision Polyphonic Modulation Matrix Engine (Header-Only).
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
// 2. DATA STRUCTURES (Optimized to Hybrid SoA)
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
 // 3. SHARED MATRIX STATE
 // =============================================================================
 inline ModSlot mod_slots[8];
 inline int32_t voice_mod_sums[MAX_SUPPORTED_VOICES][MOD_DEST_COUNT];
 inline int32_t prev_depth_mods[MAX_SUPPORTED_VOICES][8];
 
 inline int16_t aftertouch_q15  = 0;
 inline int16_t mod_wheel_q15   = 0;
 inline int16_t expression_q15  = 0;
 inline int16_t breath_q15      = 0;
 inline int16_t random_sh_q15[MAX_SUPPORTED_VOICES] = {0};
 
 // Hardware delta buffers consumed by Core 1 audio synthesis tasks
 extern volatile int32_t matrix_pitch_mod_q24[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_osc1_pitch_mod_q24[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_osc2_pitch_mod_q24[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_pw_mod[MAX_SUPPORTED_VOICES];
 
 static inline int32_t mod_clamp_4095(int32_t v) {
     if (v < -4095) return -4095;
     if (v > 4095) return 4095;
     return v;
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
 
 inline void mod_matrix_set_source(uint8_t slot, int16_t src) { 
     if (slot < 8) mod_slots[slot].src = (src < 0 || src >= MOD_SRC_COUNT) ? SRC_OFF : (uint8_t)src; 
 }
 
 inline void mod_matrix_set_dest(uint8_t slot, int16_t dest) { 
     if (slot < 8) mod_slots[slot].dest = (dest < 0 || dest >= MOD_DEST_COUNT) ? 0xFF : (uint8_t)dest; 
 }
 
 inline void mod_matrix_set_depth(uint8_t slot, int16_t depth) { 
     if (slot < 8) mod_slots[slot].depth = depth; 
 }
 
 inline void mod_matrix_set_aftertouch(uint8_t at) {
     aftertouch_q15 = (int16_t)(((int32_t)at * 32767) / 127);
 }
 
 inline void mod_matrix_set_mod_wheel(uint8_t mw) {
     mod_wheel_q15 = (int16_t)(((int32_t)mw * 32767) / 127);
 }
 
 inline void mod_matrix_set_expression(uint8_t expr) {
     expression_q15 = (int16_t)(((int32_t)expr * 32767) / 127);
 }
 
 inline void mod_matrix_set_breath(uint8_t brt) {
     breath_q15 = (int16_t)(((int32_t)brt * 32767) / 127);
 }
 
 inline void mod_matrix_on_note_on(uint8_t voice) {
     if (voice < MAX_SUPPORTED_VOICES) {
         random_sh_q15[voice] = (int16_t)random(-32768, 32767);
     }
 }
 
 inline void mod_matrix_clear_voice(uint8_t voice) {
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
// 6. REALTIME ACCUMULATOR CORE (1-Cycle MAC Optimized)
// =============================================================================
static const int16_t VOICE_ID_SPREAD_MAP[4][4] = {
    {-32768, 0, 0, 0},
    {-32768, 32767, 0, 0},
    {-32768, 0, 32767, 0},
    {-32768, -10923, 10922, 32767}
};

// Branchless fast clamp leveraging DSP instructions if available (M33/M7 FPU boards)
static inline int32_t fast_mod_clamp(int32_t v) {
#if defined(__ARM_FEATURE_SAT)
    return __builtin_arm_ssat(v, 13); // Native 1-cycle saturation
#else
    if (v < -4095) return -4095;
    if (v > 4095) return 4095;
    return v;
#endif
}

inline void SRAM_HOT(mod_matrix_accumulate_all)(const ModSources* __restrict sources, uint8_t num_voices) {
    if (num_voices > MAX_SUPPORTED_VOICES) num_voices = MAX_SUPPORTED_VOICES;
    if (num_voices == 0) return;
    
    // 1. THE JITTER FIX: Reverted to a compile-time CONSTANT for memset.
    // This forces GCC to emit zero-branch, highly predictable inline DSP store instructions.
    // No more calls to libc, meaning zero execution time spikes.
    memset(voice_mod_sums, 0, sizeof(voice_mod_sums));
    
    uint8_t spread_idx = num_voices - 1;

    // 2. THE ALIASING FIX: By mapping your global arrays to '__restrict' pointers, 
    // we legally guarantee to the compiler that writing to 'sums' will NEVER overwrite 'sources'. 
    // This allows the compiler to hoist global variables (like sources->lfo1) into 
    // a CPU register BEFORE the loop begins, saving a RAM load on every single voice iteration!
    int32_t (* __restrict sums)[MOD_DEST_COUNT] = voice_mod_sums;
    const int32_t (* __restrict prevs)[8] = (const int32_t(*)[8])prev_depth_mods;

    for (uint8_t i = 0; i < 8; i++) {
        const uint8_t src_id = mod_slots[i].src;
        if (src_id == SRC_OFF) continue; 
        
        const uint8_t dest = mod_slots[i].dest;
        if (dest >= MOD_DEST_COUNT) continue;

        const int32_t base_depth = mod_slots[i].depth;

        // 3. LOOP UNROLLING: We tell GCC to unroll the inner loop.
        // Because 'num_voices' is a short loop (e.g. 6-16), checking the loop exit condition 
        // on every voice causes branch-predictor hesitation. Unrolling slashes this overhead.
        #define MACRO_ACCUM_VOICES(SRC_EXPR) \
            _Pragma("GCC unroll 4") \
            for (uint8_t v = 0; v < num_voices; v++) { \
                int32_t depth = fast_mod_clamp(base_depth + prevs[v][i]); \
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
            case SRC_VOICE_ID:    MACRO_ACCUM_VOICES(VOICE_ID_SPREAD_MAP[spread_idx][v]); break;
        }
        #undef MACRO_ACCUM_VOICES
    }

    // 4. Update for next cycle. We unroll the inner constant-length loop 
    // to turn it into a zero-branch sequence of bitshifts.
    for (uint8_t v = 0; v < num_voices; v++) {
        _Pragma("GCC unroll 8")
        for (uint8_t i = 0; i < 8; i++) {
            prev_depth_mods[v][i] = sums[v][DEST_MOD_SLOT0_DEPTH + i] >> 15;
        }
    }
}

// =============================================================================
// 7. HARDWARE DELTA ACCESSORS (Optimized with Bit Shifts)
// =============================================================================
template <uint8_t dest>
inline int32_t SRAM_HOT(mod_matrix_get_dest_fast)(uint8_t voice) {
    // 1. Compile-time bounds check (Zero runtime cost)
    if constexpr (dest >= MOD_DEST_COUNT) return 0;
    
    // 2. ZERO-COST ADDRESSING: Because 'dest' is a template parameter, 
    // the compiler resolves 'voice_mod_sums[dest]' to a hardcoded memory pointer. 
    // The CPU simply fetches (Pointer + voice) in 1 hardware cycle.
    const int32_t raw_sum = voice_mod_sums[dest][voice];

    // 3. Compile-time shift selection (Zero runtime branches)
    if constexpr (dest == DEST_PITCH || dest == DEST_OSC1_PITCH || dest == DEST_OSC2_PITCH) {
        return raw_sum >> 2;
    } else if constexpr (dest == DEST_PW) {
        return raw_sum >> 18;
    } else if constexpr (dest == DEST_VCF_RESO) {
        return raw_sum >> 16;
    } else if constexpr (dest == DEST_VCA_LEVEL) {
        return raw_sum >> 14;
    } else if constexpr (dest == DEST_ENV_TO_VCF || dest == DEST_ENV_TO_VCA || 
                        dest == DEST_LFO1_DEPTH || dest == DEST_LFO2_DEPTH || 
                        dest == DEST_ENV_VCF_ATTACK || dest == DEST_ENV_VCF_DECAY || 
                        dest == DEST_ENV_VCA_ATTACK || dest == DEST_ENV_VCA_DECAY || 
                        dest == DEST_ENV_ALL_TIME) {
        return raw_sum >> 12;
    } else {
        return raw_sum >> 15;
    }
}

inline int32_t SRAM_HOT(mod_matrix_get_dest)(uint8_t voice, uint8_t dest) {
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
        case DEST_ENV_ALL_TIME:   return raw_sum >> 12;
        default:                  return raw_sum >> 15;
    }
}
 
 inline uint16_t SRAM_HOT(apply_mod_dir_12b)(uint16_t base, int32_t mod_delta) {
     int32_t calc = (int32_t)base + mod_delta; 
     if (calc < 0) return 0;
     if (calc > 4095) return 4095;
     return (uint16_t)calc;
 }
 
 inline uint16_t SRAM_HOT(apply_mod_inv_12b)(uint16_t base, int32_t mod_delta) {
     int32_t calc = (int32_t)base - mod_delta;
     if (calc < 0) return 0;
     if (calc > 4095) return 4095;
     return (uint16_t)calc;
 }
 
 #endif // __SHARED_MOD_MATRIX_ENGINE_H__