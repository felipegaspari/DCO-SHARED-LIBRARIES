#ifndef __AUTOTUNE_CONSTANTS_H__
#define __AUTOTUNE_CONSTANTS_H__

#include <stdint.h>

// =============================================================================
// Autotune Hardware & Measurement Constants
// =============================================================================

// Sentinel value returned by gap-measurement routines on timeout / invalid read.
constexpr float   kGapTimeoutSentinel             = 1.16999f;
constexpr int32_t kManualGapTimeoutDutyErrTimes100 = 99999;

// Edge timing limits & debouncing (microseconds)
constexpr unsigned long kGapTimeoutUs      = 100000UL;  // 100 ms baseline timeout
constexpr unsigned long kGapTimeoutMaxUs   = 400000UL;  // 400 ms ceiling at ultra-low freq
constexpr double        kGapTimeoutPeriods = 2.5;       // Period multiple for timeout deadline
constexpr unsigned long kEdgeDebounceMinUs = 20UL;      // Minimum edge interval debounce

// Sampling segment counts
constexpr uint16_t kGapSamplesDefault    = 6;
constexpr uint16_t kGapSamplesHiRes      = 12;
constexpr uint16_t kGapSamplesVeryLowMin = 12;  // Floor for < 30 Hz

// Calibration Sense Pin Polarity
constexpr bool kGapPolarityInverted = false;    // Set true if cal pin is inverted vs DCO output

// =============================================================================
// Precision Profiles (NORMAL, FINE, FAST)
// =============================================================================

struct CalPrecisionProfile {
  uint16_t gapSamplesMin;     // find_gap modes 2/3: min averaged segments
  uint16_t gapSamplesMax;     // max averaged segments
  uint32_t gapWindowMs;       // target measurement duration window
  uint32_t gapMaxWindowMs;    // ceiling on duration for ultra-low frequencies
  float    settlePeriods;     // wait time in waveform periods after freq write
  uint16_t settleMinMs;       // floored settle time in milliseconds
  double   bisectDutyTol;     // search duty acceptance fraction (e.g. 0.0005 = 0.05%)
  double   bisectGapFloorUs;  // floored microsecond gap tolerance
  int      bisectIters;       // probe budget per search
  int      bisectWindows;     // search reach multiplier
  int      confirmReads;      // readings averaged at converged point
  int      confirmRounds;     // allowed correction rounds if confirm misses
  int      anchorTries;       // FREQ_TRACE 440 Hz anchor corrections
  int      rungRetries;       // FREQ_TRACE per-rung corrections
  uint8_t  settleMaxChecks;   // stability checks on large moves
  float    settleStableMult;  // multiplier * tolerance for settled condition
};

// Fast build: for rapid testing tables
constexpr CalPrecisionProfile kCalPrecisionFast = {
  /* gapSamplesMin    */ 4,
  /* gapSamplesMax    */ 32,
  /* gapWindowMs      */ 12,
  /* gapMaxWindowMs   */ 200,
  /* settlePeriods    */ 1.0f,
  /* settleMinMs      */ 2,
  /* bisectDutyTol    */ 0.0010,
  /* bisectGapFloorUs */ 1.0,
  /* bisectIters      */ 16,
  /* bisectWindows    */ 2,
  /* confirmReads     */ 1,
  /* confirmRounds    */ 1,
  /* anchorTries      */ 1,
  /* rungRetries      */ 0,
  /* settleMaxChecks  */ 1,
  /* settleStableMult */ 4.0f,
};

// Normal build: scratch build balanced for speed & accuracy
constexpr CalPrecisionProfile kCalPrecisionNormal = {
  /* gapSamplesMin    */ 6,
  /* gapSamplesMax    */ 64,
  /* gapWindowMs      */ 25,
  /* gapMaxWindowMs   */ 300,
  /* settlePeriods    */ 1.0f,
  /* settleMinMs      */ 3,
  /* bisectDutyTol    */ 0.0005,
  /* bisectGapFloorUs */ 0.5,
  /* bisectIters      */ 24,
  /* bisectWindows    */ 2,
  /* confirmReads     */ 2,
  /* confirmRounds    */ 2,
  /* anchorTries      */ 2,
  /* rungRetries      */ 1,
  /* settleMaxChecks  */ 1,
  /* settleStableMult */ 3.0f,
};

// Fine build: highest quality refinement / verification pass
constexpr CalPrecisionProfile kCalPrecisionFine = {
  /* gapSamplesMin    */ 12,
  /* gapSamplesMax    */ 256,
  /* gapWindowMs      */ 60,
  /* gapMaxWindowMs   */ 300,
  /* settlePeriods    */ 1.0f,
  /* settleMinMs      */ 4,
  /* bisectDutyTol    */ 0.0002,
  /* bisectGapFloorUs */ 0.25,
  /* bisectIters      */ 32,
  /* bisectWindows    */ 3,
  /* confirmReads     */ 5,
  /* confirmRounds    */ 2,
  /* anchorTries      */ 3,
  /* rungRetries      */ 1,
  /* settleMaxChecks  */ 3,
  /* settleStableMult */ 2.0f,
};

// =============================================================================
// Search & Bisection Geometry Constants
// =============================================================================

constexpr float kSettleSkipCents    = 5.0f;
constexpr float kSettleBigMoveCents = 100.0f;

constexpr float kManualNoteWindowRatio     = 1.15f;
constexpr float kTopEndpointWindowRatio    = 1.05f;
constexpr float kBottomEndpointWindowRatio = 1.25f;
constexpr float kAnchorWindowRatio         = 1.15f;
constexpr float kAnchorAcquireWindowRatio  = 2.0f;

constexpr int kBootstrapSemitones[4] = { 3, -3, 6, -6 };
constexpr float kEndpointSeedAgreeCents = 50.0f;

// Frequency step boundaries (cents)
constexpr float kSearchStepVeryLowHz    = 30.0f;
constexpr float kSearchStepLowHz        = 100.0f;
constexpr float kSearchStepHighHz       = 440.0f;
constexpr float kSearchStepCentsVeryLow = 100.0f;
constexpr float kSearchStepCentsLow     = 100.0f;
constexpr float kSearchStepCentsMid     = 200.0f;
constexpr float kSearchStepCentsHigh    = 400.0f;

constexpr float kMinFreqStepHz                = 0.1f;
constexpr float kSearchSlopeMinPctPer100Cents = 1.5f;
constexpr float kSearchStepFloorCents         = 3.0f;
constexpr float kHuntStepMaxCents             = 600.0f;

// Amp-0 extrapolation & limits
constexpr float    kAmp0BandRatio          = 2.5f;
constexpr float    kAmp0MinFreqHz          = 5.0f;
constexpr int      kAmp0FitPoints          = 5;
constexpr float    kAmp0StoreFloorHz       = 2.0f;
constexpr int      kAmp0ScanPoints         = 10;
constexpr uint32_t kAmp0ScanSettleMs       = 20;
constexpr float    kGapPeriodTolRatio      = 0.15f;
constexpr float    kEndpointAcceptDutyPct  = 0.5f;
constexpr int      kMaxSearchTimeouts      = 6;

// Pulse Width Calibration Targets
constexpr double kPWCenterDutyFraction = 0.50;
constexpr double kPWLowDutyFraction    = 0.02;
constexpr double kPWHighDutyFraction   = 0.98;
constexpr double kPWLimitDutyTolerance = 0.01; // ±1%

#endif  // __AUTOTUNE_CONSTANTS_H__
