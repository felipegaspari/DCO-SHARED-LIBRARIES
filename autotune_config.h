#ifndef __AUTOTUNE_CONFIG_H__
#define __AUTOTUNE_CONFIG_H__

#include <stdint.h>

// --- System Constants ---
constexpr float    kGapTimeoutSentinel              = 1.16999f;
constexpr int32_t  kManualGapTimeoutDutyErrTimes100 = 99999;
constexpr uint32_t kGapTimeoutUs                    = 100000UL;
constexpr uint32_t kGapTimeoutMaxUs                 = 400000UL;
constexpr double   kGapTimeoutPeriods               = 2.5;
constexpr uint32_t kEdgeDebounceMinUs               = 20UL;

// --- Sampling Thresholds ---
constexpr uint16_t kGapSamplesDefault    = 6;
constexpr uint16_t kGapSamplesHiRes      = 12;
constexpr uint16_t kGapSamplesVeryLowMin = 12;

// --- Enums ---
enum CalibrationScope : uint8_t { CAL_SCOPE_AMP = 1, CAL_SCOPE_PW = 2, CAL_SCOPE_FULL = 3 };
enum CalPrecision : uint8_t     { CAL_PRECISION_NORMAL = 0, CAL_PRECISION_FINE = 1, CAL_PRECISION_FAST = 2 };
enum AutotuneAmp0Mode : uint8_t { AMP0_MODE_MEASURE = 0, AMP0_MODE_CALC = 1 };
enum AutotuneAmpMethod : uint8_t{ AMP_METHOD_CLASSIC = 0, AMP_METHOD_FREQ_TRACE = 1 };
enum AutotuneSearchMode : uint8_t{ SEARCH_BISECT = 0, SEARCH_INTERP = 1, SEARCH_GATED = 2 };
enum PWLimitDir : uint8_t       { PW_LIMIT_LOW = 0, PW_LIMIT_HIGH = 1 };

// --- Precision Profiles ---
struct CalPrecisionProfile {
    uint16_t gapSamplesMin, gapSamplesMax;
    uint32_t gapWindowMs, gapMaxWindowMs;
    float    settlePeriods;
    uint16_t settleMinMs;
    double   bisectDutyTol, bisectGapFloorUs;
    int      bisectIters, bisectWindows, confirmReads, confirmRounds;
    int      anchorTries, rungRetries;
    uint8_t  settleMaxChecks;
    float    settleStableMult;
};

constexpr CalPrecisionProfile kCalPrecisionNormal = { 6,  64,  25, 300, 1.0f, 3, 0.0005, 0.5,  24, 2, 2, 2, 2, 1, 1, 3.0f };
constexpr CalPrecisionProfile kCalPrecisionFine   = { 12, 256, 60, 300, 1.0f, 4, 0.0002, 0.25, 32, 3, 5, 2, 3, 1, 3, 2.0f };
constexpr CalPrecisionProfile kCalPrecisionFast   = { 4,  32,  12, 200, 1.0f, 2, 0.0010, 1.0,  16, 2, 1, 1, 1, 0, 1, 4.0f };

// --- Autotune Parameters ---
constexpr float kSettleSkipCents              = 5.0f;
constexpr float kSettleBigMoveCents           = 100.0f;
constexpr float kManualNoteWindowRatio        = 1.15f;
constexpr float kTopEndpointWindowRatio       = 1.05f;
constexpr float kBottomEndpointWindowRatio    = 1.25f;
constexpr float kAnchorWindowRatio            = 1.15f;
constexpr float kAnchorAcquireWindowRatio     = 2.0f;
constexpr int   kBootstrapSemitones[4]        = { 3, -3, 6, -6 };
constexpr float kEndpointSeedAgreeCents       = 50.0f;

constexpr float kSearchStepVeryLowHz          = 30.0f;
constexpr float kSearchStepLowHz              = 100.0f;
constexpr float kSearchStepHighHz             = 440.0f;
constexpr float kSearchStepCentsVeryLow       = 100.0f;
constexpr float kSearchStepCentsLow           = 100.0f;
constexpr float kSearchStepCentsMid           = 200.0f;
constexpr float kSearchStepCentsHigh          = 400.0f;
constexpr float kMinFreqStepHz                = 0.1f;
constexpr float kSearchSlopeMinPctPer100Cents = 1.5f;
constexpr float kSearchStepFloorCents         = 3.0f;
constexpr float kHuntStepMaxCents             = 600.0f;
constexpr float kAmp0BandRatio                = 2.5f;
constexpr float kAmp0MinFreqHz                = 5.0f;
constexpr int   kAmp0FitPoints                = 5;
constexpr float kAmp0StoreFloorHz             = 2.0f;
constexpr int   kAmp0ScanPoints               = 10;
constexpr uint32_t kAmp0ScanSettleMs          = 20;
constexpr float kGapPeriodTolRatio            = 0.15f;
constexpr float kEndpointAcceptDutyPct        = 0.5f;
constexpr int   kMaxSearchTimeouts            = 6;

// --- PW Targets ---
constexpr double kPWCenterDutyFraction        = 0.5;
constexpr double kPWLowDutyFraction           = 0.02;
constexpr double kPWHighDutyFraction          = 0.98;
constexpr bool   kGapPolarityInverted         = false;
constexpr double kPWLimitDutyTolerance        = 0.01;

#endif // __AUTOTUNE_CONFIG_H__