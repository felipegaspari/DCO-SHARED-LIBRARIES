#ifndef SYNTH_KEYTRACK_H
#define SYNTH_KEYTRACK_H

#include <Arduino.h>

class SynthKeytracker {
public:
    // =========================================================================
    // 1. MOD MATRIX SOURCE GENERATOR
    // =========================================================================
    /**
     * @brief Generates calibrated 12-bit bipolar source for your +-4096 Mod Matrix
     * @return Offset in 12-bit CV units (+409 counts per octave)
     */
    static inline int16_t getModMatrixSource(uint8_t note, uint8_t centerNote = 60) {
        const int32_t deltaNote = (int32_t)note - (int32_t)centerNote;
        // 34.125 counts per semitone -> 409.5 counts per octave
        return (int16_t)((deltaNote * 273) >> 3);
    }

    // =========================================================================
    // 2. DEDICATED HARDWARE DESTINATIONS (For Front-Panel Knobs +-256)
    // =========================================================================

    /**
     * @brief VCF Cutoff (AS3320 1V/Oct Linear Control Voltage)
     * @param modifier_256 Range: -256 to +256 (256 = 100% 1V/Oct)
     * @return Additive 12-bit counts for VCF_PWM (+409 counts/octave at 256)
     */
    static inline int32_t getVcfOffset(uint8_t note, int16_t modifier_256, uint8_t centerNote = 60) {
        if (modifier_256 == 0) return 0;
        const int32_t deltaNote = (int32_t)note - (int32_t)centerNote;
        return (deltaNote * 273 * (int32_t)modifier_256) >> 11;
    }

    /**
     * @brief VCA Headroom Scaling (Key -> Volume Balance)
     * @param vcaTrack_256 Range: -256 to +256
     * @return Q15 Multiplier (32768 = Unity gain at center note)
     */
    static inline int32_t getVcaScaleQ15(uint8_t note, int16_t vcaTrack_256, uint8_t centerNote = 60) {
        if (vcaTrack_256 == 0) return 32768;
        const int32_t deltaNote = (int32_t)note - (int32_t)centerNote;
        // Scales ~2dB per octave at full amount
        int32_t scale = 32768 + (deltaNote * (int32_t)vcaTrack_256 * 4);
        return __USAT(scale, 15); // Fast ARM saturation to 0..32767
    }

    /**
     * @brief Envelope Time Scaling (Key -> Attack/Decay/Release Speed)
     * Higher keys produce shorter envelope times (acoustic instrument behavior).
     * @param envTrack_256 Range: 0 to 256 (0 = off, 256 = 50% time per octave)
     * @return Q15 Multiplier (32768 = 1.0x normal time, <32768 = faster)
     */
    static inline int32_t getEnvTimeScaleQ15(uint8_t note, int16_t envTrack_256, uint8_t centerNote = 60) {
        if (envTrack_256 == 0) return 32768;
        const int32_t deltaNote = (int32_t)note - (int32_t)centerNote;
        // Inverted: higher note = smaller time multiplier
        int32_t factor = 32768 - (deltaNote * (int32_t)envTrack_256 * 4);
        return __USAT(factor, 15);
    }

    /**
     * @brief VCF Resonance Tracking (High notes reduce resonance to prevent squeal)
     * @param resoTrack_256 Range: -256 to +256
     * @return Additive 12-bit counts for RESONANCE_PWM
     */
    static inline int32_t getResonanceOffset(uint8_t note, int16_t resoTrack_256, uint8_t centerNote = 60) {
        if (resoTrack_256 == 0) return 0;
        const int32_t deltaNote = (int32_t)note - (int32_t)centerNote;
        return (deltaNote * 136 * (int32_t)resoTrack_256) >> 11;
    }
};

#endif