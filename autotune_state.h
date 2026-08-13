#ifndef __AUTOTUNE_STATE_H__
#define __AUTOTUNE_STATE_H__

#include "autotune_config.h"

// Unified runtime state for autotune. Avoids scattered globals.
struct AutotuneState {
    bool calibrationFlag = false;
    bool manualCalibrationFlag = false;
    bool firstTuneFlag = false;

    volatile bool cancelRequested = false;
    volatile bool verifyRequested = false;
    volatile bool pwCvProbeRequested = false;
    volatile bool syncNeutralRequested = false;

    uint8_t scope = CAL_SCOPE_FULL;
    uint8_t precision = CAL_PRECISION_NORMAL;
    uint8_t amp0Mode = AMP0_MODE_MEASURE;
    uint8_t ampMethod = AMP_METHOD_CLASSIC;
    uint8_t searchMode = SEARCH_INTERP;

    uint8_t currentDCO = 0;
    uint8_t currentNote = 0;
    volatile uint16_t ampCompVal = 0;
    
    float drivenFreqHz = 0.0f;
    float gapGateFreqHz = 0.0f;
    float calibFreqHz = 0.0f;

    // Search Metrics
    float lastGapUs = 0.0f;
    int   lastProbes = 0;
    int   lastSettleChecks = 0;
    bool  lastDutyUnsettled = false;
    float lastAmp0ScanSeedHz = 0.0f;

    // Extracted Manual Data arrays
    uint16_t ampComp440[NUM_OSCILLATORS] = {0};
    int16_t  ampCompDutyOffset[NUM_OSCILLATORS] = {0};
    int8_t   manualCalibrationOffset[NUM_OSCILLATORS] = {0};
    uint16_t initManualAmpCompVal[NUM_OSCILLATORS] = {0};

    const CalPrecisionProfile& getPrecisionProfile() const {
        if (precision == CAL_PRECISION_FINE) return kCalPrecisionFine;
        if (precision == CAL_PRECISION_FAST) return kCalPrecisionFast;
        return kCalPrecisionNormal;
    }
};

extern AutotuneState g_tuneState;

// Secure operational context passed down to core routines.
struct AutotuneContext {
    AutotuneState& state;
    uint32_t* calibrationData;

    AutotuneContext(AutotuneState& s, uint32_t* data)
        : state(s), calibrationData(data) {}

    uint8_t& dcoIndex() const { return state.currentDCO; }
    uint8_t& currentNote() const { return state.currentNote; }
};

// --- Report Data Structures ---
static constexpr int kCalReportPairs = (int)(chanLevelVoiceDataSize / 2);
enum CalPointSource : uint8_t { CAL_SRC_NONE, CAL_SRC_RUNG, CAL_SRC_ANCHOR, CAL_SRC_ENDPOINT_FULL, CAL_SRC_ENDPOINT_AMP0, CAL_SRC_MANUAL, CAL_SRC_FILLED, CAL_SRC_SENTINEL, CAL_SRC_REFINED };

struct CalReport {
    float   dutyErrPct[kCalReportPairs];
    uint8_t source[kCalReportPairs];
    int     ladderInterval = 0;
    int     anchorPair = -1;
    uint32_t probes = 0;
    unsigned long startMs = 0;

    void reset() {
        for (int p = 0; p < kCalReportPairs; ++p) { dutyErrPct[p] = 1e9f; source[p] = CAL_SRC_NONE; }
        ladderInterval = 0; anchorPair = -1; probes = 0; startMs = millis();
    }
};
extern CalReport g_calReport;

#endif // __AUTOTUNE_STATE_H__