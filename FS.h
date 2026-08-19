// Calibration storage: bank sizes, RAM buffers, file handles, FS API.
// Shared by DCO3-MONOSYNTH and DCO4-REBORN through the DCO/FS.h shim.
// Definitions live in FS_impl.h. On-flash format, per-board sizing and the
// invariants that keep stored calibration readable: docs/CALIBRATION_STORAGE.md.
//
// Included FROM the sketch's include_all.h (and before amp_comp.h, which sizes
// arrays with chanLevelVoiceDataSize) — so it must not include include_all.h.
// NUM_PW_CHANNELS must already be defined by the sketch's globals.h.
#ifndef __FS_H__
#define __FS_H__

static constexpr uint16_t FSVoiceDataSize = 22 * 2 * 4;
static constexpr uint16_t FSBankSize      = FSVoiceDataSize * NUM_OSCILLATORS;

// =============================================================================
// 1. 3-Point PW Calibration Bank (18 Bytes per Channel)
// 3 points x [center: 2B, lowLimit: 2B, highLimit: 2B] = 18 Bytes
// =============================================================================
static constexpr uint8_t  kPWCalPoints            = 3;
static constexpr uint16_t FSPWLimitsPointDataSize = 6;  // 3 x uint16_t
static constexpr uint16_t FSPWDataSize            = FSPWLimitsPointDataSize * kPWCalPoints; // 18 Bytes / ch
static constexpr uint16_t FSPWBankSize            = FSPWDataSize * NUM_PW_CHANNELS;         // 72B (DCO4) / 54B (DCO3)

// Pure Integer Storage Struct for 1 Calibration Point (6 Bytes)
struct PWCalLimits {
  uint16_t center;
  uint16_t lowLimit;
  uint16_t highLimit;
};

// =============================================================================
// 2. Dual-Engine Pitch Tracking RAM Cache Struct
// =============================================================================
#ifdef USE_FLOAT_VOICE_TASK
struct PWTrackCache {
  float f0, f1, f2;         // Frequency points in Hz
  float invSpan01;          // 1.0f / (f1 - f0)
  float invSpan12;          // 1.0f / (f2 - f1)
};
#else
struct PWTrackCache {
  uint32_t f0, f1, f2;       // Frequency points in Fixed-Point (Q24)
  uint32_t invSpan01_q24;   // (1 << 24) / (f1 - f0) multiplier
  uint32_t invSpan12_q24;   // (1 << 24) / (f2 - f1) multiplier
};
#endif

// =============================================================================
// 3. Amp-Comp Top Valid Pair Storage (1 Byte per Oscillator)
// =============================================================================
static constexpr uint16_t FSAmpCompTopPairDataSize = 1;
static constexpr uint16_t FSAmpCompTopPairBankSize = FSAmpCompTopPairDataSize * NUM_OSCILLATORS;

// Manual Offset & 440 Anchor Bank Sizes
static constexpr uint16_t FSManualOffsetDataSize     = 1;
static constexpr uint16_t FSManualOffsetBankSize     = FSManualOffsetDataSize * NUM_OSCILLATORS;
static constexpr uint16_t FSAmpComp440DataSize       = 2;
static constexpr uint16_t FSAmpComp440BankSize       = FSAmpComp440DataSize * NUM_OSCILLATORS;
static constexpr uint16_t FSAmpCompDutyOffsetDataSize= 2;
static constexpr uint16_t FSAmpCompDutyOffsetBankSize= FSAmpCompDutyOffsetDataSize * NUM_OSCILLATORS;

static constexpr uint16_t chanLevelVoiceDataSize     = FSVoiceDataSize / 4;

// Calibration Buffers (FS-local)
extern uint8_t voiceTablesBankBuffer[FSBankSize];
extern uint8_t PWCalBankBuffer[FSPWBankSize];
extern uint8_t AmpCompTopPairBankBuffer[FSAmpCompTopPairBankSize];
extern uint8_t ManualOffsetBankBuffer[FSManualOffsetBankSize];
extern uint8_t AmpComp440BankBuffer[FSAmpComp440BankSize];
extern uint8_t AmpCompDutyOffsetBankBuffer[FSAmpCompDutyOffsetBankSize];

// Global arrays defined in FS_impl.h
extern PWCalLimits  PW_CAL_LIMITS[NUM_PW_CHANNELS][kPWCalPoints];
extern uint8_t      ampCompTopPair[NUM_OSCILLATORS];
extern PWTrackCache pwTrackCache[NUM_PW_CHANNELS];

// API Prototypes (Declared here so amp_comp.h sees them)
void init_FS();
void precompute_pw_tracking_cache();
void update_FS_voice(byte voiceN);
void update_FS_PW_Channel(byte ch);
void update_FS_AmpCompTopPair(byte oscIndex, uint8_t value);
void update_FS_PWCenter(byte voiceN, uint16_t value);
void update_FS_PW_High_Limit(byte voiceN, uint16_t value);
void update_FS_PW_Low_Limit(byte voiceN, uint16_t value);
void update_FS_ManualCalibrationOffset(byte oscIndex, int8_t value);
void update_FS_AmpComp440(byte oscIndex, uint16_t value);
void update_FS_AmpCompDutyOffset(byte oscIndex, int16_t value);
void seed_fake_calibration_tables(bool force = false);
void write_fs_bank(const char* name, const uint8_t* data, size_t size);

#endif // __FS_H__