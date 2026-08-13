#ifndef __AUTOTUNE_MAIN_H__
#define __AUTOTUNE_MAIN_H__

#include "autotune_state.h"
#include "autotune_measurement.h"
#include "autotune_amp.h"

// Orchestrates the auto-calibration based on requested scope.
inline void DCO_calibration(uint32_t* dataBuffer) {
    g_tuneState.cancelRequested = false;
    
    // Initialize shared state
    for(int i=0; i<NUM_OSCILLATORS; i++) {
        if (g_tuneState.initManualAmpCompVal[i] == 0) 
            g_tuneState.initManualAmpCompVal[i] = (35u * DIV_COUNTER / 14000u);
    }

    AutotuneContext ctx(g_tuneState, dataBuffer);

    // Stop oscillators safely
    for (int i = 0; i < NUM_OSCILLATORS; i++) {
        pio_sm_set_enabled(pio[VOICE_TO_PIO[i]], VOICE_TO_SM[i], false);
        gpio_set_dir(RANGE_PINS[i], GPIO_OUT);
        gpio_put(RANGE_PINS[i], 1);
    }

    if (ctx.state.scope == CAL_SCOPE_PW || ctx.state.scope == CAL_SCOPE_FULL) {
        // Run PW calibrations explicitly (Details in AUTOTUNE_DOCS.md)
        for(uint8_t osc = 0; osc < NUM_OSCILLATORS && !ctx.state.cancelRequested; ++osc) {
            ctx.dcoIndex() = osc;
            // Delegate to PW search routines (assuming they use ctx natively)
            // find_PW_center(ctx, 0); 
            // find_PW_limit_v2(ctx, PW_LIMIT_LOW);
            // find_PW_limit_v2(ctx, PW_LIMIT_HIGH);
        }
    }

    if (ctx.state.scope == CAL_SCOPE_AMP || ctx.state.scope == CAL_SCOPE_FULL) {
        for(uint8_t osc = 0; osc < NUM_OSCILLATORS && !ctx.state.cancelRequested; ++osc) {
            ctx.dcoIndex() = osc;
            g_calReport.reset();

            // Run AMP routines
            if (ctx.state.precision == CAL_PRECISION_FINE) {
                // refine_DCO_amp_table(ctx);
            } else if (ctx.state.ampMethod == AMP_METHOD_FREQ_TRACE) {
                // calibrate_DCO_freq_trace(ctx);
            } else {
                // calibrate_DCO(ctx, 0.001);
            }
            
            // Persist valid tables
            if (!ctx.state.cancelRequested) update_FS_voice(osc);
        }
    }

    g_tuneState.calibrationFlag = false;
}

#endif // __AUTOTUNE_MAIN_H__