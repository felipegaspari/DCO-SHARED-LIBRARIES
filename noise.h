// Sketch-side noise objects (Arduino style). Engines / DcoNoiseGen live in DCO_Noise.
// Ctor: (color, seed). Output is always full int16 Q15 (−32768…32767).
// Set NOISE_ENGINE / ENABLE_NOISE_OUT in DCO.ino before includes.
// Fleet is two gens: noise0 white (Character amp/PW), noise1 pink (Character pitch).

/**
 * @file noise.h
 * @brief Sketch-side PRNG noise generator instances and lookup tables.
 */

 #ifndef NOISE_H
 #define NOISE_H
 
 #ifndef NOISE_ENGINE
 #define NOISE_ENGINE 0
 #endif
 
 #include "../_build_libs/DCO_Noise/DCO_Noise.h"
 
 static constexpr uint8_t NUM_NOISE_GENS = 2;
 
 /// White noise generator instance for amplitude and pulse-width jitter.
 DcoNoiseGen noise0(NOISE_WHITE, 0xC0FFEE01u);
 /// Pink noise generator instance for pitch jitter.
 DcoNoiseGen noise1(NOISE_PINK,  0xC0FFEE02u);
 
 /// Indexed pointer array for modulation matrix routing.
 DcoNoiseGen* const noiseGens[NUM_NOISE_GENS] = {
   &noise0, &noise1
 };
 
 /// Live output sample buffers (Q15: -32768..32767) updated each Core 1 frame.
 volatile int16_t noiseLevel[NUM_NOISE_GENS];
 
 #endif  // DCO_NOISE_H