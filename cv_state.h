/**
 * @file cv_state.h
 * @brief Global hardware state buffers and extern declarations.
 */

 #ifndef __CV_STATE_H__
 #define __CV_STATE_H__
 
 #include <stdint.h>
 #include <stdbool.h>
 

 // 1. Envelope Timings & Restarts
 extern uint16_t ADSR1_attack, ADSR1_decay, ADSR1_sustain, ADSR1_release;
 extern uint16_t ADSR2_attack, ADSR2_decay, ADSR2_sustain, ADSR2_release;
 extern bool ADSR1_restart, ADSR2_restart, ADSR3_restart;
 
 // 2. Envelope Curves
 extern uint8_t ADSR1AttackCurveVal, ADSR1DecayCurveVal, ADSR1ReleaseCurveVal;
 extern uint8_t ADSR2AttackCurveVal, ADSR2DecayCurveVal, ADSR2ReleaseCurveVal;
 extern uint8_t ADSR3AttackCurveVal, ADSR3DecayCurveVal, ADSR3ReleaseCurveVal;
 
 // 3. Filter, VCA & Dynamics Baseline Values
 extern uint16_t CUTOFF, RESONANCE, LFO2toVCF, VCALevel, LFO1toVCA;
 extern int16_t ADSR2toVCF, ADSR1toVCA;
 extern uint16_t DIST_DRIVE, DIST_MIX;
 extern uint8_t FILTER_MODE;
 
 // 4. Precomputed Scales & Mod Buffers
 extern int32_t ADSR2toVCF_scale_q15, LFO2toVCF_scale_q15, LFO1toVCA_scale_q15;
 extern int32_t VCFKeytrackModifier_q15, VCFKeytrackPerVoice_q15[NUM_VOICES_TOTAL];
 extern int32_t velocityToVCF_q15, velocityToVCA_q15, vcf_drift_scale_q15;
 extern volatile int16_t VCF_DRIFT[NUM_VOICES_TOTAL];
 
 // Matrix Output Buffers
 extern volatile int32_t matrix_pitch_mod_q24[NUM_VOICES_TOTAL];
 extern volatile int32_t matrix_pw_mod[NUM_VOICES_TOTAL];
 extern volatile int32_t matrix_detune_mod[NUM_VOICES_TOTAL];
 
 extern bool RESONANCEAmpCompensation;
 extern int16_t VCAResonanceCompensation, VCFKeytrack;
 extern int8_t velocityToVCFVal, velocityToVCAVal;
 
 // 5. Hardware Output PWM Buffers
 extern uint16_t VCA_PWM[NUM_VOICES_TOTAL], VCF_PWM[NUM_VOICES_TOTAL], RESONANCE_PWM[NUM_FILTERS];
 extern uint16_t AS2164_VCA_linearize_table[4096];
 
 // 6. Mixer Levels
 extern uint16_t lin_to_log_128[129];
 extern int16_t OSC1LevelVal, OSC2LevelVal, OSC3LevelVal, SubLevelVal;
 extern uint16_t OSC1Level, OSC2Level, OSC3Level, SubLevel;
 extern bool ADSR3Enabled;
 
 #endif // __CV_STATE_H__