# DCO-SHARED-LIBRARIES

Plain shared C++ headers used by more than one DCO project. Not an Arduino
library: there is no `library.properties` and the folder is deliberately kept
off the `arduino-cli --libraries` path, so nothing here is auto-scanned or
auto-included. Sketches pull in exactly the headers they name.

Consumers:

- [DCO4-REBORN](https://github.com/felipegaspari/DCO4-REBORN)
- [DCO3-MONOSYNTH](https://github.com/felipegaspari/DCO3-MONOSYNTH)

Compiled Arduino libraries that happen to be shared (ADSR_Bezier, mo-lfo,
DCO_Noise, DCO-PROTOCOL) each stay in their own repo and are symlinked into
`DCO/_build_libs/`. This repo is for the rest: code that is part of the sketch
rather than a library it links against.

## Contents

| Header | What it holds |
|---|---|
| `amp_comp.h` | Amp-comp calibration tables and quadratic window eval |
| `autotune.h` | Calibration state, enums, prototypes; pulls in the three below |
| `autotune_constants.h` | Calibration constants and the NORMAL / FINE / FAST precision profiles |
| `autotune_context.h` | `DCOCalibrationContext` |
| `autotune_measurement.h` | `find_gap()` declaration and the `measure_gap()` wrapper |
| `autotune_impl.h` | Definitions: run orchestration, PW searches, `find_gap()`. Include once |
| `autotune_search_impl.h` | Definitions: amp-comp search, FREQ_TRACE, endpoints. Include once |
| `bench.h` | Hot-path profiler; DCO3-only probes/report via `PROJECT_INSTRUMENT` |
| `character_jitter.h` | Character-tab noise jitter scales (pitch / amp / PW) |
| `clkdiv.h` | PIO clock-divider total-cycle helpers (`CLKDIV_MODE`) |
| `cv_bezier.h` | Bézier helpers for the AS2164 VCA linearize table |
| `cv_out.h` | Software CV math prototypes |
| `mem_diag.h` | SRAM / heap / stack snapshot prototypes |
| `midi.h` | USB and serial MIDI instances |
| `midi_cc.h` | 7-bit MIDI CC control surface |
| `midi_cc_map.h` | Generated `MidiCcEntry` table (from `gen_midi_map.py`) |
| `noise.h` | Sketch-side `DcoNoiseGen` objects |
| `noteList.h` | MIDI note Hz / Q24 tables (`__not_in_flash("pitch_tables")`) |
| `PWM.h` | RANGE / CV / level PWM writers |
| `range_pwm_dither.pio.h` | Generated PIO program for RANGE dither |
| `tusb_config.h` | TinyUSB config (sketch keeps a same-name shim) |
| `utils.h` | Log / exp mapping prototypes |
| `voice_alloc.h` | `VoiceAllocator` (poly voice stealing) and `MonoNoteStack` (mono note priority) |
| `voices.h` | Voice init and portamento state |
| `wave_mux.h` | 74HC595 wave-select prototypes |
| `docs/AUTOTUNE.md` | Shared autotune algorithms, file layout, DCO3/DCO4 hardware table |
| `docs/CALIBRATION_PROCEDURE.md` | Operator calibration bring-up |

## How a sketch consumes it

The repo is a submodule at the root of each superproject, and each sketch that
needs it carries a symlink beside its `_build_libs`:

```
DCO/_shared -> ../../DCO-SHARED-LIBRARIES
```

```cpp
#include "_shared/voice_alloc.h"
```

Header-only, so there is nothing to add to the build command.

---

# autotune — DCO amp-comp and PW calibration

The whole calibration subsystem: the amp-comp table build (classic per-note
search and FREQ_TRACE curve tracing), the refine pass over a stored table, the
PW center and limit searches, and the edge-timing duty measurement they all run
on. Six headers, split into declarations and definitions. Algorithms:
[`docs/AUTOTUNE.md`](docs/AUTOTUNE.md). Operator workflow:
[`docs/CALIBRATION_PROCEDURE.md`](docs/CALIBRATION_PROCEDURE.md).

## Layout

| Header | Include it |
|---|---|
| `autotune.h` | From the sketch, wherever calibration state or prototypes are needed. Pulls in `autotune_constants.h`, `autotune_measurement.h` and `autotune_context.h`, so those need no shim of their own. |
| `autotune_impl.h` | **Exactly once**, from `DCO/autotune.ino`. |
| `autotune_search_impl.h` | **Exactly once**, from `DCO/autotune_search.ino`, which sorts after `autotune.ino`. |

The two `*_impl.h` carry definitions, not declarations. They live behind `.ino`
shims rather than being included from a header because arduino-cli merges the
sketch's `.ino` files alphabetically into one translation unit: keeping the shim
names is what preserves that order, and with it the visibility of each file's
`static` helpers to the one after it.

That also means the Arduino prototype generator no longer sees these functions -
it only scans `.ino` files - so everything called across the boundary is
declared in `autotune.h`, and the file-scope statics used before their
definition are forward-declared at the top of each `*_impl.h`. A function added
to an `*_impl.h` and called from anywhere earlier needs a declaration written by
hand.

## What the sketch has to provide

`autotune.h` and both `*_impl.h` start with `#include "../include_all.h"`, which
resolves to the sketch's own `include_all.h` when reached through `DCO/_shared/`.
Through it the calibration code expects, from the board:

- topology and pins in `globals.h`: `NUM_OSCILLATORS`, `NUM_VOICES_TOTAL`,
  `NUM_PW_CHANNELS`, `DIV_COUNTER`, `DCO_calibration_pin`, `RANGE_*`, `PW_*`,
  `VOICE_TO_PIO/SM`
- `voice_task_autotune()` (drives one oscillator at `calibrationFreqHz`)
- `start_voice_sms()`, to restart the PIO state machines afterwards
- the LittleFS writers `update_FS_voice()`, `update_FS_PW*()`,
  `update_FS_AmpComp440()`, and `chanLevelVoiceDataSize`
- `serialSendParam32()` and `PARAM_GAP_FROM_DCO`
- `amp_comp.h`, `PWM.h` and `noteList.h` from this repo

## Consumers

Both sketches consume this library via `DCO/_shared`.

- [DCO3-MONOSYNTH](https://github.com/felipegaspari/DCO3-MONOSYNTH) — 1 voice × 3 osc; unassigned PW pins skip, so PW cal runs once on channel 0.
- [DCO4-REBORN](https://github.com/felipegaspari/DCO4-REBORN) — 4 voices × 2 osc; PW indexed with `cal_pw_channel(osc)` (`NUM_PW_CHANNELS = 4`, channel = osc / 2).

Algorithms and the operator workflow live in this repo: [`docs/AUTOTUNE.md`](docs/AUTOTUNE.md), [`docs/CALIBRATION_PROCEDURE.md`](docs/CALIBRATION_PROCEDURE.md).

---

# voice_alloc.h

Voice allocation for a polyphonic or paraphonic board, and the mono held-key
stack. One policy value drives both halves of the same decision: in poly it
picks which voice gets stolen when all of them are busy, in mono it picks which
held key sounds. This is `PARAM_VOICE_ALLOC_MODE` (102) in
[DCO-PROTOCOL](https://github.com/felipegaspari/DCO-PROTOCOL).

## Modes

| `VoiceAllocMode` | Poly | Mono |
|---|---|---|
| `0 VOICE_ALLOC_ROUND_ROBIN` | least recently used | last note |
| `1 VOICE_ALLOC_OLDEST` | oldest trigger | first note |
| `2 VOICE_ALLOC_QUIETEST` | lowest EnvVCA level | last note |
| `3 VOICE_ALLOC_QUIETEST_KEEP_LOW` | as 2, spares the lowest held note | low note |
| `4 VOICE_ALLOC_QUIETEST_KEEP_HIGH` | as 2, spares the highest held note | high note |
| `5 VOICE_ALLOC_NO_STEAL` | drops the note-on | first note, later keys never sound |

Every poly mode takes an idle slot first, then the best release tail, and only
steals a held note as a last resort. Mode `5` refuses that last step and returns
`VOICE_ALLOC_NONE`.

Mode `5` in mono is not the same as mode `1`. First-note priority keeps the
later keys on the stack, so they take over as earlier keys are released; mode
`5` refuses them at push time, so they never sound at all.

## State ownership

The library owns only the allocation bookkeeping: per-voice state, trigger
stamp, release stamp, LRU order and its own copy of the sounding note. The
sketch keeps its own gate flags, note table and ADSR edge flags, and mirrors
every change into the allocator through `markOn()` / `markOff()`. Nothing in a
sketch hot path has to change to adopt it.

Voices move `IDLE -> HELD -> RELEASING`. The step back to `IDLE` is not written
by anyone: `alloc()` derives it from the envelope level each time it runs, so
there is no window in which one core frees a slot the other just took.

## Level source

The quietest modes, and the "this release tail has finished" test, need a
per-voice EnvVCA level in Q15 (`0..32767`):

```cpp
voiceAlloc.begin(ADSR_VCA_Level_q15);   // or setLevelSource() later
```

If no level source is registered, the allocator estimates a tail from the
release time instead, which the sketch must keep current:

```cpp
voiceAlloc.setReleaseMs(ADSR_VCA_release);
```

That fallback exists for builds where nothing refreshes the level array, such as
DCO4 with `ENABLE_MB_MOD_STREAM`, where the envelopes are computed on the
Mainboard.

## API

```cpp
template <uint8_t MaxVoices> class VoiceAllocator;  // 1..8 slots
```

| Call | Notes |
|---|---|
| `begin(levels_q15 = nullptr)` | Reset all slots to idle, rebuild the LRU order, register the level source. Also restores the default mode and voice count, so call it first. |
| `setMode(m)` / `mode()` | Out-of-range values are ignored. |
| `setVoiceCount(n)` / `voiceCount()` | The runtime `NUM_VOICES`, clamped to `MaxVoices`. |
| `setLevelSource(p)` / `hasLevelSource()` | See above. |
| `setReleaseMs(ms)` | Only read when there is no level source. |
| `setSilenceThresholdQ15(q15)` | Default 64, about -54 dB. |
| `findNote(note)` | Voice already carrying this note, including a release tail, else `VOICE_ALLOC_NONE`. |
| `alloc(now_ms)` | The allocation. `VOICE_ALLOC_NONE` when mode 5 refuses. |
| `markOn(v, note, now_ms)` | Held, freshly triggered, most recently used. |
| `markOff(v, now_ms)` | Gate off, start tracking the tail. |
| `regate(v, note)` | Re-gate an already allocated slot without retriggering: new pitch, same trigger stamp. The mono priority fallback. |
| `resyncFromGates(gates)` | Realign with the sketch's gate flags after a voice count change. |
| `stateOf(v)` / `noteOf(v)` | Read-only. |

`alloc()`, `markOn()` and `markOff()` also have no-argument overloads that call
`millis()`. The explicit `now_ms` ones exist so a caller that already read the
clock does not read it twice, and so the allocator can be tested off-target.

```cpp
template <uint8_t Depth = 8> class MonoNoteStack;
```

| Call | Notes |
|---|---|
| `clear()` | Drop every held key, on a voice mode change. |
| `push(note, mode)` | Re-strike moves to the top; full drops the oldest. Returns `false` when mode 5 denies the key. |
| `remove(note)` | Returns `false` if the note was not held. |
| `pick(mode)` | The sounding key, or `VOICE_ALLOC_NONE` when nothing is held. |
| `count()` / `empty()` | |

The mode is passed in rather than shared with the allocator, so the two classes
stay independent.

## Build flags

| Flag | Default | Effect |
|---|---|---|
| `VOICE_ALLOC_SRAM_HOT` | `0` | `1` puts `alloc` / `markOn` / `markOff` / the two `pick`s and the stack edits in `.time_critical` (RP2040 `__not_in_flash_func`). No-op where the attribute does not exist. |

Define it before the include, the same way the DCO sketches do for
`ADSR_BEZIER_SRAM_HOT` and `MO_LFO_SRAM_HOT`:

```cpp
#ifndef VOICE_ALLOC_SRAM_HOT
#define VOICE_ALLOC_SRAM_HOT 1
#endif
#include "_shared/voice_alloc.h"
```

Allocation runs on the MIDI path, not in an audio loop, so this buys jitter
consistency rather than throughput: it keeps a note-on off the XIP cache while
the audio core is mid-frame. The active setting is reported once per translation
unit with `#pragma message`, visible in the `arduino-cli` log.

## Usage sketch

```cpp
// once
voiceAlloc.begin(ADSR_VCA_Level_q15);
voiceAlloc.setVoiceCount(NUM_VOICES);

// poly note-on
uint8_t v = voiceAlloc.findNote(note);
if (v == VOICE_ALLOC_NONE) v = voiceAlloc.alloc();
if (v == VOICE_ALLOC_NONE) return;          // mode 5 refused the note
VOICES[v] = 1; VOICE_NOTES[v] = note;       // the sketch's own bookkeeping
voiceAlloc.markOn(v, note);

// poly note-off
VOICES[v] = 0;
voiceAlloc.markOff(v);

// mono note-on
if (!monoStack.push(note, voiceAlloc.mode())) return;
const uint8_t winner = monoStack.pick(voiceAlloc.mode());
```
