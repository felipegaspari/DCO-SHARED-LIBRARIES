#ifndef __AUTOTUNE_MEASUREMENT_H__
#define __AUTOTUNE_MEASUREMENT_H__

#include "autotune_state.h"
#include "autotune_math.h"

// Wraps raw measurement returns
struct GapMeasurement {
    bool timedOut;
    float value;
};

// Core duty cycle gap measurement (See AUTOTUNE_DOCS.md: Gap Measurement)
inline float find_gap(AutotuneContext& ctx, uint8_t specialMode) {
    double freqHz = (ctx.state.gapGateFreqHz > 0.0f) ? ctx.state.gapGateFreqHz : note_to_freq(ctx.state.currentNote);
    double idealPeriodUs = (freqHz > 0.0) ? (1000000.0 / freqHz) : 0.0;
    
    uint16_t samplesTarget = kGapSamplesDefault;
    uint32_t timeoutUs = kGapTimeoutUs;
    double dtMinUs = 0.0, dtMaxUs = 0.0;

    if (specialMode == 2 || specialMode == 3) {
        const auto& prec = ctx.state.getPrecisionProfile();
        samplesTarget = prec.gapSamplesMin;
        if (idealPeriodUs > 0) {
            long n = lround(prec.gapWindowMs / (idealPeriodUs / 2000.0));
            samplesTarget = constrain(n, samplesTarget, min((long)prec.gapSamplesMax, lround(prec.gapMaxWindowMs / (idealPeriodUs / 2000.0))));
            if (freqHz < kSearchStepVeryLowHz) samplesTarget = max(samplesTarget, kGapSamplesVeryLowMin);
        }
    }

    if (idealPeriodUs > 0) {
        timeoutUs = min(kGapTimeoutMaxUs, (uint32_t)(idealPeriodUs * kGapTimeoutPeriods));
        dtMinUs = max((double)kEdgeDebounceMinUs, idealPeriodUs * 0.01);
        dtMaxUs = min((double)timeoutUs, idealPeriodUs * 0.99);
    }

    const uint32_t cyclesPerUs = rp2040.f_cpu() / 1000000;
    const uint32_t debounceCycles = kEdgeDebounceMinUs * cyclesPerUs;
    
    int pulseCount = 0;
    uint16_t acceptedSamples = 0, risingCount = 0, fallingCount = 0;
    uint64_t risingSumCycles = 0, fallingSumCycles = 0;
    bool lastVal = 0;
    uint32_t lastEdgeCycles = rp2040.getCycleCount();
    unsigned long lastEdgeTime = micros();

    while (acceptedSamples < samplesTarget) {
        bool val = digitalRead(DCO_calibration_pin) ^ kGapPolarityInverted;
        uint32_t nowCycles = rp2040.getCycleCount();
        unsigned long nowUs = micros();

        if (nowUs - lastEdgeTime > timeoutUs) return kGapTimeoutSentinel;

        if (val != lastVal) {
            uint32_t dtCycles = nowCycles - lastEdgeCycles; // Cycle-accurate resolution
            if (dtCycles >= debounceCycles) {
                lastVal = val;
                if (pulseCount == 1 && val == 0) pulseCount = 0;
                
                if (pulseCount > 2) {
                    bool intervalOk = (idealPeriodUs == 0) || (dtCycles >= dtMinUs * cyclesPerUs && dtCycles <= dtMaxUs * cyclesPerUs);
                    if (intervalOk) {
                        if (val == 0) { fallingSumCycles += dtCycles; fallingCount++; } 
                        else          { risingSumCycles += dtCycles; risingCount++; }
                        acceptedSamples++;
                    }
                }
                lastEdgeTime = nowUs;
                lastEdgeCycles = nowCycles;
                pulseCount++;
            }
        }
    }

    if (specialMode == 3 && (risingCount == 0 || fallingCount == 0)) return kGapTimeoutSentinel;

    float avgLowUs  = fallingCount ? (fallingSumCycles / (float)cyclesPerUs) / fallingCount : 0.0f;
    float avgHighUs = risingCount  ? (risingSumCycles / (float)cyclesPerUs) / risingCount : 0.0f;
    float measuredPeriodUs = avgLowUs + avgHighUs;

    if (specialMode == 3 && idealPeriodUs > 0 && fabsf(measuredPeriodUs - idealPeriodUs) > kGapPeriodTolRatio * idealPeriodUs) {
        return kGapTimeoutSentinel;
    }

    return avgHighUs - avgLowUs;
}

inline GapMeasurement measure_gap(AutotuneContext& ctx, uint8_t specialMode) {
    float v = find_gap(ctx, specialMode);
    return {v == kGapTimeoutSentinel, v};
}

#endif // __AUTOTUNE_MEASUREMENT_H__