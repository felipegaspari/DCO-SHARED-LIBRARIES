/**
 * @file voices.h
 * @brief Voice allocation, portamento glide state, and pitch interpolation tables.
 */

 #ifndef DCO_VOICES_H
 #define DCO_VOICES_H
 
 #include <stdint.h>
 
 void init_voices();
 
 /// Voice note-on trigger edge flags.
 volatile bool note_on_flag_flag[NUM_VOICES_TOTAL];
 
 /// Portamento timer timestamps per voice.
 uint32_t portamentoTimer[NUM_VOICES_TOTAL];
 uint32_t portamentoStartMillis[NUM_VOICES_TOTAL];
 uint32_t portamentoStartMicros[NUM_VOICES_TOTAL];
 
 bool portamento = true;
 uint8_t portamento_parameter_value = 0;  ///< Raw UI / param value (0..255).
 uint32_t portamento_time_fixed = 0;      ///< Duration in µs for PORTA_MODE_TIME.
 uint32_t portamento_time_slew = 0;       ///< Duration in µs for PORTA_MODE_SLEW.
 uint32_t portamento_time = 0;            ///< Active duration for current glide mode.
 
 /**
  * @enum PortamentoMode
  * @brief Portamento glide modes across semitone space.
  */
 enum PortamentoMode : uint8_t {
   PORTA_MODE_TIME = 0, ///< Fixed duration for any interval.
   PORTA_MODE_SLEW = 1  ///< Constant slew rate (duration scales with interval).
 };
 uint8_t portamento_mode = PORTA_MODE_SLEW;
 
 // Portamento state in Q24 (Hz * 2^24) — one slot per oscillator
 int64_t portamento_start_q24[NUM_OSCILLATORS];
 int64_t portamento_stop_q24[NUM_OSCILLATORS];
 int64_t portamento_cur_freq_q24[NUM_OSCILLATORS];
 int64_t freqPortaStep_q24[NUM_OSCILLATORS];
 
 // Portamento state in note-space (Q16 semitones)
 int32_t porta_note_start_q16[NUM_OSCILLATORS];
 int32_t porta_note_stop_q16[NUM_OSCILLATORS];
 int32_t porta_note_cur_q16[NUM_OSCILLATORS];
 int32_t porta_note_step_q16[NUM_OSCILLATORS];  ///< Q16 semitones per µs.
 bool porta_note_valid[NUM_OSCILLATORS];
 
 #ifdef USE_FLOAT_VOICE_TASK
 float porta_freq_start_f[NUM_OSCILLATORS];
 float porta_freq_stop_f [NUM_OSCILLATORS];
 float porta_freq_step_f [NUM_OSCILLATORS];
 float porta_freq_cur_f  [NUM_OSCILLATORS];
 
 float porta_note_start_f[NUM_OSCILLATORS];
 float porta_note_stop_f [NUM_OSCILLATORS];
 float porta_note_cur_f  [NUM_OSCILLATORS];
 float porta_note_step_f [NUM_OSCILLATORS];  ///< Semitones per microsecond.
 #endif
 
 uint8_t highestNote = 124;
 
 static const int multiplierTableSize = 200;
 const int32_t multiplierTableScale = 10000;
 
 #if PITCH_INTERP_MODE == PITCH_INTERP_FLOAT || \
     PITCH_INTERP_MODE == PITCH_INTERP_FLOAT_FAST
 float   xMultiplierTableF[multiplierTableSize];
 float   yMultiplierTableF[multiplierTableSize];
 float   slopeF[multiplierTableSize - 1];
 #else
 int32_t xMultiplierTable[multiplierTableSize];
 int32_t yMultiplierTable[multiplierTableSize];
 int32_t x0Q16_tbl[multiplierTableSize];
 #if PITCH_INTERP_MODE == PITCH_INTERP_RATIO_Q16
 int32_t slopeQ16[multiplierTableSize - 1];
 #elif PITCH_INTERP_MODE == PITCH_INTERP_Q12
 int32_t slopeQ12[multiplierTableSize - 1];
 #endif
 #endif
 
 /// Per-DCO segment cache for linear pitch interpolation.
 int16_t interpSegCache[NUM_OSCILLATORS];
 
 static const uint16_t maxFrequency = 4000;
 
 static constexpr int32_t Q24_ONE = (1 << 24);
 static constexpr int32_t Q24_EPS_DELTA_1P00001 = 168; // round(0.00001 * 2^24)
 static constexpr int32_t Q24_ONE_EPS = Q24_ONE + Q24_EPS_DELTA_1P00001;
 
 #endif // DCO_VOICES_H