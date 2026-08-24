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
 // 2. DATA STRUCTURES
 // =============================================================================
 struct ModSources {
     int16_t lfo1;          ///< LFO1 wave (-32768..32767, Bipolar Q15)
     int16_t lfo2;          ///< LFO2 wave (-32768..32767, Bipolar Q15)
     int16_t pitch_bend;    ///< MIDI Pitch Bend (-8192..8191, Bipolar)
     int16_t drift_global;  ///< Master drift (-32768..32767, Bipolar Q15)
     int16_t drift_voice;   ///< Voice drift (-32768..32767, Bipolar Q15)
     int16_t env_vca;       ///< VCA Envelope (0..32767, Unipolar Q15)
     int16_t env_vcf;       ///< VCF Envelope (0..32767, Unipolar Q15)
     int16_t env_dco;       ///< DCO Envelope (0..32767, Unipolar Q15)
     uint8_t velocity;      ///< MIDI Velocity (0..127)
     uint8_t keytrack_note; ///< MIDI Note (0..127)
 };
 
 struct ModSlot {
     uint8_t src;   ///< Uses ModSource enum
     uint8_t dest;  ///< Uses ModDest enum
     int16_t depth; ///< Bipolar depth (-4095..+4095)
 };
 
 // =============================================================================
 // 3. SHARED MATRIX STATE (C++17 inline storage: Single definition everywhere)
 // =============================================================================
 inline ModSlot mod_slots[8];
 inline int32_t voice_mod_sums[MAX_SUPPORTED_VOICES][MOD_DEST_COUNT];
 inline int32_t prev_depth_mods[MAX_SUPPORTED_VOICES][8];
 
 inline int16_t aftertouch_q15 = 0;
 inline int16_t mod_wheel_q15 = 0;
 inline int16_t random_sh_q15[MAX_SUPPORTED_VOICES] = {0};
 
 // Hardware buffers instantiated in cv_out.ino on both boards
 extern volatile int32_t matrix_pitch_mod_q24[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_pw_mod[MAX_SUPPORTED_VOICES];
 extern volatile int32_t matrix_detune_mod[MAX_SUPPORTED_VOICES];
 
 // Safe bounds clamping (RP2040 Cortex-M0+ compatible)
 static inline int32_t mod_clamp_4095(int32_t v) {
     if (v < -4095) return -4095;
     if (v > 4095) return 4095;
     return v;
 }
 
 // =============================================================================
 // 4. CORE ENGINE LIFECYCLE
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
 // 5. CENTRALIZED PARAMETER ROUTER APPLIERS
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
 // 6. REALTIME ACCUMULATOR CORE (DOUBLE-BUFFERED TO PREVENT INTERRUPT GLITCHES)
 // =============================================================================
 
 static const int16_t VOICE_ID_SPREAD_MAP[4][4] = {
     {-32768, 0, 0, 0},                              // 1 Voice Mode
     {-32768, 32767, 0, 0},                          // 2 Voice Mode
     {-32768, 0, 32767, 0},                          // 3 Voice Mode
     {-32768, -10923, 10922, 32767}                  // 4 Voice Mode
 };
 
 inline void SRAM_HOT(mod_matrix_accumulate_all)(const ModSources* sources, uint8_t num_voices) {
     if (num_voices > MAX_SUPPORTED_VOICES) num_voices = MAX_SUPPORTED_VOICES;
     
     uint8_t spread_idx = (num_voices > 0) ? num_voices - 1 : 0;
 
     // 1. LOCAL BUFFER (Prevents high-speed audio timers from reading a cleared array mid-calculation)
     int32_t next_sums[MAX_SUPPORTED_VOICES][MOD_DEST_COUNT];
     memset(next_sums, 0, sizeof(next_sums));
 
     // 2. Iterate Slots (Outer Loop)
     for (uint8_t i = 0; i < 8; i++) {
         if (mod_slots[i].src == SRC_OFF) continue; 
         
         uint8_t dest = mod_slots[i].dest;
         if (dest >= MOD_DEST_COUNT) continue;
 
         ModSource src = (ModSource)mod_slots[i].src;
         int32_t src_vals[MAX_SUPPORTED_VOICES] = {0};
 
         switch (src) {
             case SRC_LFO1:        for(uint8_t v=0; v<num_voices; v++) src_vals[v] = sources[v].lfo1; break;
             case SRC_LFO2:        for(uint8_t v=0; v<num_voices; v++) src_vals[v] = sources[v].lfo2; break;
             case SRC_ENV_VCA:     for(uint8_t v=0; v<num_voices; v++) src_vals[v] = sources[v].env_vca; break;
             case SRC_ENV_VCF:     for(uint8_t v=0; v<num_voices; v++) src_vals[v] = sources[v].env_vcf; break;
             case SRC_ENV_DCO:     for(uint8_t v=0; v<num_voices; v++) src_vals[v] = sources[v].env_dco; break;
             case SRC_MODWHEEL:    for(uint8_t v=0; v<num_voices; v++) src_vals[v] = mod_wheel_q15; break;
             case SRC_AFTERTC:     for(uint8_t v=0; v<num_voices; v++) src_vals[v] = aftertouch_q15; break;
             case SRC_VELOCITY:    for(uint8_t v=0; v<num_voices; v++) src_vals[v] = ((int32_t)sources[v].velocity * 32767) / 127; break;
             case SRC_BEND:        for(uint8_t v=0; v<num_voices; v++) src_vals[v] = (int32_t)sources[v].pitch_bend * 4; break;
             case SRC_DRIFT:       for(uint8_t v=0; v<num_voices; v++) src_vals[v] = sources[v].drift_global; break;
             case SRC_KEYTRACK:    for(uint8_t v=0; v<num_voices; v++) src_vals[v] = ((int32_t)(sources[v].keytrack_note - 60) * 32768) / 60; break;
             case SRC_DRIFT_VOICE: for(uint8_t v=0; v<num_voices; v++) src_vals[v] = sources[v].drift_voice; break;
             case SRC_RANDOM_SH:   for(uint8_t v=0; v<num_voices; v++) src_vals[v] = random_sh_q15[v]; break;
             case SRC_VOICE_ID:    for(uint8_t v=0; v<num_voices; v++) src_vals[v] = VOICE_ID_SPREAD_MAP[spread_idx][v]; break;
             default: break;
         }
 
         // Inner Loop
         for (uint8_t v = 0; v < num_voices; v++) {
             int32_t current_depth = mod_clamp_4095((int32_t)mod_slots[i].depth + prev_depth_mods[v][i]);
             if (current_depth != 0) {
                 next_sums[v][dest] += (src_vals[v] * current_depth);
             }
         }
     }
 
     // 3. Update Depth Modulators for the NEXT frame
     for (uint8_t v = 0; v < num_voices; v++) {
         for (uint8_t i = 0; i < 8; i++) {
             // Using safe division instead of bitshift to avoid compiler ambiguity on negatives
             prev_depth_mods[v][i] = next_sums[v][DEST_MOD_SLOT0_DEPTH + i] / 32768;
         }
     }
 
     // 4. ATOMIC COMMIT (Interrupt routines reading voice_mod_sums will safely read valid values)
     for (uint8_t v = 0; v < num_voices; v++) {
         for (uint8_t d = 0; d < MOD_DEST_COUNT; d++) {
             voice_mod_sums[v][d] = next_sums[v][d];
         }
     }
 }
 
 // =============================================================================
 // 7. HARDWARE DELTA ACCESSORS
 // =============================================================================
 
 template <uint8_t dest>
 inline int32_t SRAM_HOT(mod_matrix_get_dest_fast)(uint8_t voice) {
     if constexpr (dest >= MOD_DEST_COUNT) return 0;
     
     const int32_t raw_sum = voice_mod_sums[voice][dest];
 
     // Divisions act perfectly for bipolar signals on all compilers. 
     // They compile identically to shift instructions.
     if constexpr (dest == DEST_PITCH) return raw_sum / 4;
     else if constexpr (dest == DEST_PW) return raw_sum / 262144;
     else if constexpr (dest == DEST_OSC2_DETUNE) return raw_sum / 524288;
     else if constexpr (dest == DEST_VCF_RESO) return raw_sum / 65536;
     else if constexpr (dest == DEST_ENV_TO_VCF || dest == DEST_ENV_TO_VCA || 
                        dest == DEST_LFO1_DEPTH || dest == DEST_LFO2_DEPTH || 
                        dest == DEST_ENV_VCF_ATTACK || dest == DEST_ENV_VCF_DECAY || 
                        dest == DEST_ENV_VCA_ATTACK || dest == DEST_ENV_VCA_DECAY || 
                        dest == DEST_ENV_ALL_TIME) {
         return raw_sum / 4096;
     } else {
         return raw_sum / 32768;
     }
 }
 
 inline int32_t SRAM_HOT(mod_matrix_get_dest)(uint8_t voice, uint8_t dest) {
     if (voice >= MAX_SUPPORTED_VOICES || dest >= MOD_DEST_COUNT) return 0;
 
     const int32_t raw_sum = voice_mod_sums[voice][dest];
 
     switch ((ModDest)dest) {
         case DEST_PITCH: return raw_sum / 4;
         case DEST_PW: return raw_sum / 262144;
         case DEST_OSC2_DETUNE: return raw_sum / 524288;
         case DEST_VCF_RESO: return raw_sum / 65536;
         case DEST_VCA_LEVEL: return raw_sum >> 14; // 2x Full Modulation Range (+/-8190)
         case DEST_ENV_TO_VCF:
         case DEST_ENV_TO_VCA:
         case DEST_LFO1_DEPTH:
         case DEST_LFO2_DEPTH:
         case DEST_ENV_VCF_ATTACK:
         case DEST_ENV_VCF_DECAY:
         case DEST_ENV_VCA_ATTACK:
         case DEST_ENV_VCA_DECAY:
         case DEST_ENV_ALL_TIME: return raw_sum / 4096;
         default: return raw_sum / 32768;
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