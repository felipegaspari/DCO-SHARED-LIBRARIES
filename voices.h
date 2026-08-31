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
 uint8_t portamento_mode = PORTA_MODE_TIME;
 
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
     PITCH_INTERP_MODE == PITCH_INTERP_FLOAT_CACHED
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
 static constexpr int NOTE_PITCH_TABLE_MAX_SIZE = (sizeof(sNotePitches_q24) / sizeof(sNotePitches_q24[0])) - 1; // 135
 
void voice_task_fixed_point();
void voice_task_float();
void voice_task_Q24();
void voice_task_main();
void voice_mark_on(uint8_t voice, uint8_t raw_note, uint8_t note1_idx, uint8_t note2_idx, uint8_t velocity_in);
void voice_mark_off(uint8_t voice);
void voice_mark_regate(uint8_t voice, uint8_t raw_note, uint8_t note1_idx, uint8_t note2_idx);

// Last clk_div handed to each SM. Writing Y consumes the OSR, so clk_div always has to
// be re-pushed afterwards; without a remembered value the SM would read a shifted-out
// OSR as its ramp count and shriek for one control frame. 200 is the slow "park" rate
// used elsewhere during calibration.
uint32_t osc_last_clk_div[NUM_OSCILLATORS] = { 200, 200, 200, 200, 200, 200, 200, 200 };

// Soft sync: number of trailing ramp chunks that poll the master's reset pin.
// 0 = hard sync through the sideset pin (no resolution cost).
// 1..3 = soft sync with that many trailing polled chunks (receptive ~40% / ~67% / ~86%).
// Only one poll program image is resident at a time; changing 1..3 reloads it.
uint8_t softSyncChunks = 0;

// Note-on OSC1/OSC2 restart when oscPhaseSync >= 1 (A/B listen + profiler).
// 0 EXACT_Y: disable + period_split + load Y + jmp + enable_in_sync (default).
// 1 SYNC_JMP: jmp only on running SMs (keep last Y; no disable / enable_in_sync).
#ifndef NOTE_RETRIG_MODE_DEFAULT
#define NOTE_RETRIG_MODE_DEFAULT 0
#endif
enum NoteRetrigMode : uint8_t {
  NOTE_RETRIG_EXACT_Y  = 0,
  NOTE_RETRIG_SYNC_JMP = 1,
};
volatile uint8_t note_retrig_mode = (uint8_t)NOTE_RETRIG_MODE_DEFAULT;
volatile bool note_retrig_mode_ack_pending = false;

static inline const char *note_retrig_mode_name(uint8_t m) {
  return (m == NOTE_RETRIG_SYNC_JMP) ? "SYNC_JMP" : "EXACT_Y";
}

static inline void note_retrig_set_mode(uint8_t m) {
  if (m > NOTE_RETRIG_SYNC_JMP) m = NOTE_RETRIG_EXACT_Y;
  note_retrig_mode = m;
  __dmb();
}

// Sub-oscillator divide ratio: 0 = off, 2 = one octave down, 4 = two octaves.
uint8_t subOscDivide = 0;

// Program-relative addresses.
//   RESTART: `mov x, y`, the top of the reset pulse. Jumping here retriggers a cycle.
//   PHASE_HOLD: last `jmp x--` before `mov x, y` / `set pins, 1` (loop_final). Preload X and
//     jump here for a one-shot delay until OSC2's first flyback. On the old 8-chunk
//     `frequency` program this was hardcoded jmp 10; on frequency_sync_4_jumps it is 9.
//   RAMP_ENTRY[q]: leftover 90° chunk entries (unused by live phase-align).
// Free and poll-1 share entry addresses {0,4,6,8}; poll-2/3 shift later entries because
// polled chunks insert an extra instruction before each countdown.
static constexpr uint8_t PIO_RESTART_ADDR_FREE = 10;
static constexpr uint8_t PIO_RESTART_ADDR_SYNC[4] = { 10, 11, 12, 13 };  // index = softSyncChunks
static constexpr uint8_t PIO_PHASE_HOLD_ADDR_FREE = 9;
static constexpr uint8_t PIO_PHASE_HOLD_ADDR_SYNC[4] = { 9, 10, 11, 12 };  // index = softSyncChunks
static constexpr uint8_t PIO_RAMP_ENTRY_FREE[4] = { 0, 4, 6, 8 };
static constexpr uint8_t PIO_RAMP_ENTRY_SYNC_1[4] = { 0, 4, 6, 8 };
static constexpr uint8_t PIO_RAMP_ENTRY_SYNC_2[4] = { 0, 4, 6, 9 };
static constexpr uint8_t PIO_RAMP_ENTRY_SYNC_3[4] = { 0, 4, 7, 10 };

static inline uint8_t soft_sync_chunks_clamped() {
  uint8_t n = softSyncChunks;
  if (n > 3) n = 3;
  return n;
}

static inline uint32_t osc_program_base(uint8_t osc) {
  const uint8_t blk = VOICE_TO_PIO[osc];
  return osc_uses_sync_program[osc] ? pio_offset_sync[blk] : pio_offset_free[blk];
}

static inline uint32_t osc_restart_target(uint8_t osc) {
  const uint8_t blk = VOICE_TO_PIO[osc];
  if (!osc_uses_sync_program[osc]) {
    return pio_offset_free[blk] + PIO_RESTART_ADDR_FREE;
  }
  return pio_offset_sync[blk] + PIO_RESTART_ADDR_SYNC[soft_sync_chunks_clamped()];
}

static inline uint32_t osc_phase_hold_target(uint8_t osc) {
  const uint8_t blk = VOICE_TO_PIO[osc];
  if (!osc_uses_sync_program[osc]) {
    return pio_offset_free[blk] + PIO_PHASE_HOLD_ADDR_FREE;
  }
  return pio_offset_sync[blk] + PIO_PHASE_HOLD_ADDR_SYNC[soft_sync_chunks_clamped()];
}

// X preload for osc_phase_align_hold_stopped so the first flyback lands at
// remaining ≈ total * (360 - deg) / 360. 0 → caller should jmp restart (0°).
// RECIP_360_Q24 mul/shift; −3 is loop_final fallthrough + mov x,y + set pins,1.
static inline uint32_t osc_phase_hold_x(uint32_t total_cycles, uint16_t deg) {
  if (deg >= 360u) deg = (uint16_t)(deg % 360u);
  if (deg == 0) return 0;
  uint32_t per_deg = (uint32_t)(((uint64_t)total_cycles * RECIP_360_Q24 + (1u << 23)) >> 24);
  uint32_t remaining = per_deg * (uint32_t)(360u - deg);
  return (remaining > 3u) ? remaining - 3u : 0u;
}

static inline uint32_t osc_ramp_entry_target(uint8_t osc, uint8_t quarters) {
  quarters &= 3;
  const uint8_t blk = VOICE_TO_PIO[osc];
  if (!osc_uses_sync_program[osc]) {
    return pio_offset_free[blk] + PIO_RAMP_ENTRY_FREE[quarters];
  }
  switch (soft_sync_chunks_clamped()) {
    case 2:  return pio_offset_sync[blk] + PIO_RAMP_ENTRY_SYNC_2[quarters];
    case 3:  return pio_offset_sync[blk] + PIO_RAMP_ENTRY_SYNC_3[quarters];
    default: return pio_offset_sync[blk] + PIO_RAMP_ENTRY_SYNC_1[quarters];
  }
}

// Period model of whichever program this oscillator is running.
static inline uint32_t osc_ramp_weight(uint8_t osc) {
  if (!osc_uses_sync_program[osc]) return PIO_RAMP_WEIGHT_FREE;
  return PIO_RAMP_WEIGHT_BY_CHUNKS[soft_sync_chunks_clamped()];
}

static inline uint32_t osc_period_overhead(uint8_t osc) {
  if (!osc_uses_sync_program[osc]) return PIO_PERIOD_OVERHEAD_FREE;
  return PIO_PERIOD_OVERHEAD_BY_CHUNKS[soft_sync_chunks_clamped()];
}

// Result of splitting a target period into the PIO's reset pulse and ramp chunks.
struct PioPeriod {
  uint32_t clk_div;  // per-chunk count pushed to the SM's OSR
  uint32_t y;        // reset pulse width; pioPulseLength plus the division remainder
};

// Split `total_cycles` exactly into y + weight*clk_div + overhead.
//
// The remainder of the chunk division lands in the reset pulse rather than being rounded
// away, so the generated period matches the target to the cycle instead of quantising to
// `weight` cycles (about 0.2 cents at 7 kHz with weight 4). The pulse wobbles by
// 0..weight-1 cycles, at most 13 ns at 225 MHz, which is nothing against a reset pulse
// measured in microseconds.
//
// Caveat that shapes how this is used: `y` can only be pushed to the SM by way of the OSR
// (put -> pull -> out y), and the OSR simultaneously holds clk_div for the four
// `mov x, OSR` chunk reads. A Y update on a running SM therefore leaves a window where a
// chunk can latch the pulse width as its ramp count. Callers must only push Y while the SM
// is stopped, which in practice means at note-on.
static inline SRAM_HOT(PioPeriod pio_period_split)(uint32_t total_cycles,
                                         uint32_t weight,
                                         uint32_t overhead) {
  PioPeriod p;

  // Guard the subtraction: very high frequencies can leave no room for a ramp.
  uint32_t fixed = overhead + pioPulseLength;
  if (total_cycles <= fixed) {
    p.clk_div = 0;
    p.y = pioPulseLength;
    return p;
  }

  uint32_t ramp = total_cycles - fixed;
  p.clk_div = ramp / weight;
  p.y = pioPulseLength + (ramp % weight);
  return p;
}

 
 /**
  * RP2350 OPTIMIZED: Branchless Clock Divider
  */
 static inline __attribute__((always_inline)) 
 SRAM_HOT(uint32_t pio_clk_div_for_y)(uint32_t total_cycles, uint32_t y, uint32_t weight, uint32_t overhead) {
     const uint32_t fixed = overhead + y;
     const uint32_t ramp = total_cycles - fixed;
     
     // weight / 2u is replaced by a 1-cycle bitshift (weight >> 1)
     const uint32_t div = (ramp + (weight >> 1)) / weight;
     
     // Ternary avoids the 'if (total_cycles <= fixed) return 0;' branch penalty
     return (total_cycles > fixed) ? div : 0;
 }
 
/**
 * RP2350 OPTIMIZED: Combined parameter fetch (Inlined runtime accessor)
 */
 static inline __attribute__((always_inline)) 
 void SRAM_HOT(get_osc_params)(uint8_t osc, uint32_t& weight, uint32_t& overhead) {
     if (osc_uses_sync_program[osc]) {
         const uint32_t chunks = soft_sync_chunks_clamped();
         weight = PIO_RAMP_WEIGHT_BY_CHUNKS[chunks];
         overhead = PIO_PERIOD_OVERHEAD_BY_CHUNKS[chunks];
     } else {
         weight = PIO_RAMP_WEIGHT_FREE;
         overhead = PIO_PERIOD_OVERHEAD_FREE;
     }
 }
 #endif // DCO_VOICES_H