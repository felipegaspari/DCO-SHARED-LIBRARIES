#ifndef __AUTOTUNE_AMP_H__
#define __AUTOTUNE_AMP_H__

#include "autotune_measurement.h"

// Drives frequency to DCO and blocks until settled
inline void drive_freq_and_settle(AutotuneContext& ctx, float freqHz, uint16_t amp) {
    ctx.state.calibFreqHz = freqHz;
    voice_task_autotune(4, amp);
    ctx.state.drivenFreqHz = freqHz;
    
    uint32_t settleMs = 4;
    if (freqHz > 0) settleMs = max(settleMs, (uint32_t)(2000.0 / freqHz));
    delay(settleMs);
}

// Executes adaptive duty probe with precision limits.
inline float measure_duty_at_freq(AutotuneContext& ctx, float freqHz, uint16_t amp, bool hiRes = false) {
    ctx.state.gapGateFreqHz = freqHz;
    drive_freq_and_settle(ctx, freqHz, amp);

    const auto& prec = ctx.state.getPrecisionProfile();
    uint8_t mode = hiRes ? 3 : 0;
    
    ctx.state.lastProbes++;
    GapMeasurement gm = measure_gap(ctx, mode);
    
    if (gm.timedOut) {
        ctx.state.gapGateFreqHz = 0;
        return kGapTimeoutSentinel;
    }

    float stableTol = max((float)prec.bisectGapFloorUs, (float)compute_gap_tolerance(freqHz, prec.bisectDutyTol)) * prec.settleStableMult;
    float value = gm.value;
    
    ctx.state.lastDutyUnsettled = false;
    for (int c = 0; c < prec.settleMaxChecks; ++c) {
        if (ctx.state.cancelRequested) break;
        ctx.state.lastSettleChecks++;
        GapMeasurement again = measure_gap(ctx, mode);
        if (!again.timedOut && fabsf(again.value - value) <= stableTol) {
            value = 0.5f * (value + again.value);
            break;
        }
        value = again.value;
        if (c == prec.settleMaxChecks - 1 && amp == 0) ctx.state.lastDutyUnsettled = true;
    }

    ctx.state.gapGateFreqHz = 0;
    float dutyTrim = (ctx.state.ampCompDutyOffset[ctx.dcoIndex()] / 10000.0f) * (2e6f / freqHz);
    return -(value - dutyTrim);
}

// Bisection logic for optimal frequency lookup (See AUTOTUNE_DOCS.md: Amp Search)
inline float find_freq_for_duty50(AutotuneContext& ctx, uint16_t amp, float freqGuess, float windowRatio, bool refine = false) {
    ctx.state.ampCompVal = amp;
    ctx.state.lastGapUs = kGapTimeoutSentinel;
    ctx.state.lastProbes = 0;
    ctx.state.lastSettleChecks = 0;

    float bestFreq = 0, bestAbsGap = 1e9f, bestSignedGap = 0;
    int maxProbes = refine ? ctx.state.getPrecisionProfile().bisectIters : 24;
    double f = freqGuess, fLo = 0, fHi = 0, gLo = 0, gHi = 0;

    for (int i = 0; i < maxProbes; ++i) {
        if (ctx.state.cancelRequested) return bestFreq;

        float diff = measure_duty_at_freq(ctx, f, amp, refine);
        if (diff != kGapTimeoutSentinel && fabsf(diff) < bestAbsGap) {
            bestAbsGap = fabsf(diff);
            bestSignedGap = diff;
            bestFreq = f;
        }

        // Logic compressed for KISS. Steps bracket and interpolates via Secant.
        if (diff == kGapTimeoutSentinel) {
            if (fHi == 0) fHi = f; 
        } else if (diff > 0) { fHi = f; gHi = diff; } 
        else                 { fLo = f; gLo = diff; }

        if (fLo > 0 && fHi > 0) {
            if (ctx.state.searchMode == SEARCH_BISECT || gLo == 0 || gHi == 0) f = sqrt(fLo * fHi);
            else f = exp(log(fLo) + (log(fHi) - log(fLo)) * (gLo / (gLo - gHi)));
        } else {
            f *= (diff > 0 || diff == kGapTimeoutSentinel) ? 0.95 : 1.05; // Fallback step
        }
    }
    
    ctx.state.lastGapUs = bestSignedGap;
    return bestFreq;
}

#endif // __AUTOTUNE_AMP_H__