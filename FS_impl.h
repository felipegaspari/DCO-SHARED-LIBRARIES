// Calibration storage definitions: the init_FS() loader, the update_FS_*
// writers and the fake-calibration seed. Include exactly once, from the
// sketch's DCO/FS.ino shim. Declarations: FS.h. Format and the rules for
// changing any of it: docs/CALIBRATION_STORAGE.md.

#ifndef __FS_IMPL_H__
#define __FS_IMPL_H__

#include "../include_all.h"

// =============================================================================
// Buffer & RAM Array Instantiations
// =============================================================================
uint8_t voiceTablesBankBuffer[FSBankSize];
uint8_t PWCalBankBuffer[FSPWBankSize];
uint8_t AmpCompTopPairBankBuffer[FSAmpCompTopPairBankSize];
uint8_t ManualOffsetBankBuffer[FSManualOffsetBankSize];
uint8_t AmpComp440BankBuffer[FSAmpComp440BankSize];
uint8_t AmpCompDutyOffsetBankBuffer[FSAmpCompDutyOffsetBankSize];

File fileVoiceTablesFS;
File filePWCalFS;
File fileAmpCompTopPairFS;
File fileManualOffsetFS;
File fileAmpComp440FS;
File fileAmpCompDutyOffsetFS;

// Global RAM Definitions for 3-Point PW & Amp-Comp Tracking
PWCalLimits  PW_CAL_LIMITS[NUM_PW_CHANNELS][3]; // [Channel][0=Low, 1=Mid, 2=High]
uint8_t      ampCompTopPair[NUM_OSCILLATORS];
PWTrackCache pwTrackCache[NUM_PW_CHANNELS];

// =============================================================================
// Low-Level Byte Serialization Helpers (Pure Integer 6-Byte Point)
// =============================================================================

// Pack a 6-byte PW point: [center: 2B, lowLimit: 2B, highLimit: 2B]
static void pack_pw_limits_point(uint8_t* bank, uint8_t ch, uint8_t pt, const PWCalLimits& p) {
  uint16_t offset = (ch * FSPWDataSize) + (pt * FSPWLimitsPointDataSize);

  bank[offset + 0] = (uint8_t)(p.center & 0xFF);
  bank[offset + 1] = (uint8_t)((p.center >> 8) & 0xFF);
  bank[offset + 2] = (uint8_t)(p.lowLimit & 0xFF);
  bank[offset + 3] = (uint8_t)((p.lowLimit >> 8) & 0xFF);
  bank[offset + 4] = (uint8_t)(p.highLimit & 0xFF);
  bank[offset + 5] = (uint8_t)((p.highLimit >> 8) & 0xFF);
}

// Unpack a 6-byte PW point
static PWCalLimits unpack_pw_limits_point(const uint8_t* bank, uint8_t ch, uint8_t pt) {
  uint16_t offset = (ch * FSPWDataSize) + (pt * FSPWLimitsPointDataSize);
  PWCalLimits p;

  p.center    = (uint16_t)bank[offset + 0] | ((uint16_t)bank[offset + 1] << 8);
  p.lowLimit  = (uint16_t)bank[offset + 2] | ((uint16_t)bank[offset + 3] << 8);
  p.highLimit = (uint16_t)bank[offset + 4] | ((uint16_t)bank[offset + 5] << 8);

  return p;
}

// Precompute frequency boundaries and Q31/Float reciprocal multipliers
void precompute_pw_tracking_cache() {
  for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    uint8_t osc = ch * (NUM_OSCILLATORS / NUM_PW_CHANNELS);

    // 1. Determine the 3 operating rung indices from the Amp-Comp curve
    uint8_t topIdx = ampCompTopPair[osc];
    if (topIdx < 6 || topIdx >= (chanLevelVoiceDataSize / 2)) {
      topIdx = 18;
    }

    int anchorIdx = (calReportAnchorPair >= 0) ? calReportAnchorPair : 10;
    int lowIdx    = max(2, anchorIdx - 5);            // Point 0 (LOW: ~100 Hz)
    int midIdx    = anchorIdx;                        // Point 1 (MID: ~440 Hz Anchor)
    int highIdx   = max(midIdx + 2, (int)topIdx - 2); // Point 2 (HIGH: TopPair - 2)

    // =======================================================================
    // A. FLOAT ENGINE PRECOMPUTE (RP2350 / FPU)
    // =======================================================================
#ifdef USE_FLOAT_VOICE_TASK
    float f0 = ampCompFrequencyHz[osc][lowIdx];
    float f1 = ampCompFrequencyHz[osc][midIdx];
    float f2 = ampCompFrequencyHz[osc][highIdx];

    if (f0 <= 0.0f) f0 = 110.0f;
    if (f1 <= f0)   f1 = 440.0f;
    if (f2 <= f1)   f2 = 2800.0f;

    pwTrackCache[ch].f0 = f0;
    pwTrackCache[ch].f1 = f1;
    pwTrackCache[ch].f2 = f2;
    pwTrackCache[ch].invSpan01 = 1.0f / (f1 - f0);
    pwTrackCache[ch].invSpan12 = 1.0f / (f2 - f1);

    // =======================================================================
    // B. FIXED-POINT Q31 PRECOMPUTE (RP2040 Default - Zero Underflow)
    // =======================================================================
#else
    uint32_t f0 = (uint32_t)ampCompFrequencyArray[osc][lowIdx];
    uint32_t f1 = (uint32_t)ampCompFrequencyArray[osc][midIdx];
    uint32_t f2 = (uint32_t)ampCompFrequencyArray[osc][highIdx];

    if (f0 == 0) f0 = (uint32_t)(110ULL << FREQ_FRAC_BITS);
    if (f1 <= f0) f1 = (uint32_t)(440ULL << FREQ_FRAC_BITS);
    if (f2 <= f1) f2 = (uint32_t)(2800ULL << FREQ_FRAC_BITS);

    pwTrackCache[ch].f0 = f0;
    pwTrackCache[ch].f1 = f1;
    pwTrackCache[ch].f2 = f2;

    uint32_t span01 = f1 - f0;
    uint32_t span12 = f2 - f1;

    // Q31 Reciprocal Multipliers (Prevents underflow, enables single-cycle integer MAC)
    pwTrackCache[ch].invSpan01_q24 = (span01 > 0) ? (uint32_t)((1ULL << 31) / span01) : 0;
    pwTrackCache[ch].invSpan12_q24 = (span12 > 0) ? (uint32_t)((1ULL << 31) / span12) : 0;
#endif
  }
}

#if PROJECT_INSTRUMENT == 4
static void ensure_pw_fs_banks();
#endif

// =============================================================================
// Primary LittleFS Initializer & Runtime Loader
// =============================================================================

void init_FS() {
  LittleFS.begin();

  // =========================================================================
  // 1. VOICE TABLES (Amp Compensation)
  // =========================================================================
  if (!LittleFS.exists("voiceTables")) {
    fileVoiceTablesFS = LittleFS.open("voiceTables", "w+");
    memset(voiceTablesBankBuffer, 0, FSBankSize);
    fileVoiceTablesFS.write(voiceTablesBankBuffer, FSBankSize);
  } else {
    fileVoiceTablesFS = LittleFS.open("voiceTables", "r");
    fileVoiceTablesFS.read(voiceTablesBankBuffer, FSBankSize);
  }
  fileVoiceTablesFS.close();

#ifdef ENABLE_FS_CALIBRATION
  for (int i = 0; i < (chanLevelVoiceDataSize * NUM_OSCILLATORS); i++) {
    freq_to_amp_comp_array[i] = (int32_t(voiceTablesBankBuffer[i * 4 + 3]) << 24) |
                                (int32_t(voiceTablesBankBuffer[i * 4 + 2]) << 16) |
                                (int32_t(voiceTablesBankBuffer[i * 4 + 1]) << 8)  |
                                 int32_t(voiceTablesBankBuffer[i * 4]);
  }

  for (int datasetIndex = 0; datasetIndex < NUM_OSCILLATORS; ++datasetIndex) {
    for (int pairIndex = 0; pairIndex < chanLevelVoiceDataSize / 2; ++pairIndex) {
      int rawIndex = datasetIndex * chanLevelVoiceDataSize + pairIndex * 2;
      int32_t freq_x100 = freq_to_amp_comp_array[rawIndex];

      ampCompArray[datasetIndex][pairIndex] = freq_to_amp_comp_array[rawIndex + 1];

#ifdef USE_FLOAT_AMP_COMP
      float freqHz = (float)freq_x100 / 100.0f;
      ampCompFrequencyHz[datasetIndex][pairIndex] = freqHz;
#else
      int64_t scaled = (int64_t)freq_x100 * (1LL << FREQ_FRAC_BITS);
      int32_t freq_fx = (scaled >= 0)
                      ? (int32_t)((scaled + 50LL) / 100LL)
                      : (int32_t)(-(((-scaled) + 50LL) / 100LL));
      ampCompFrequencyArray[datasetIndex][pairIndex] = freq_fx;
#endif
    }
  }

  // =========================================================================
  // 2. AMP COMP TOP VALID PAIR INDICES (ampCompTopPair)
  // =========================================================================
  if (!LittleFS.exists("AmpCompTopPair")) {
    fileAmpCompTopPairFS = LittleFS.open("AmpCompTopPair", "w+");
    for (uint8_t i = 0; i < NUM_OSCILLATORS; ++i) {
      AmpCompTopPairBankBuffer[i] = 18; // Default to pair 18
    }
    fileAmpCompTopPairFS.write(AmpCompTopPairBankBuffer, FSAmpCompTopPairBankSize);
  } else {
    fileAmpCompTopPairFS = LittleFS.open("AmpCompTopPair", "r");
    fileAmpCompTopPairFS.read(AmpCompTopPairBankBuffer, FSAmpCompTopPairBankSize);
  }
  fileAmpCompTopPairFS.close();

  for (uint8_t i = 0; i < NUM_OSCILLATORS; ++i) {
    ampCompTopPair[i] = AmpCompTopPairBankBuffer[i];
    if (ampCompTopPair[i] < 4 || ampCompTopPair[i] >= (chanLevelVoiceDataSize / 2)) {
      ampCompTopPair[i] = 18;
    }
  }

  // =========================================================================
  // 3. 3-POINT PULSE WIDTH CALIBRATION (PWCal3Pt - 18 Bytes / Channel)
  // =========================================================================
#if PROJECT_INSTRUMENT == 4
  ensure_pw_fs_banks();
#endif

  if (!LittleFS.exists("PWCal3Pt")) {
    filePWCalFS = LittleFS.open("PWCal3Pt", "w+");
    for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
      uint16_t c = (pwSweepMode == PW_SWEEP_FULL) ? (DIV_COUNTER_PW / 2) : 0;
      PWCalLimits pDefault = { c, 0, (uint16_t)DIV_COUNTER_PW };
      pack_pw_limits_point(PWCalBankBuffer, ch, 0, pDefault); // Low
      pack_pw_limits_point(PWCalBankBuffer, ch, 1, pDefault); // Mid
      pack_pw_limits_point(PWCalBankBuffer, ch, 2, pDefault); // High
    }
    filePWCalFS.write(PWCalBankBuffer, FSPWBankSize);
  } else {
    filePWCalFS = LittleFS.open("PWCal3Pt", "r");
    filePWCalFS.read(PWCalBankBuffer, FSPWBankSize);
  }
  filePWCalFS.close();

  // Unpack into RAM
  for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    for (uint8_t pt = 0; pt < 3; ++pt) {
      PW_CAL_LIMITS[ch][pt] = unpack_pw_limits_point(PWCalBankBuffer, ch, pt);
    }
    // Set legacy 440 Hz anchor shortcuts from Point 1
    PW_CENTER[ch]     = PW_CAL_LIMITS[ch][1].center;
    PW_LOW_LIMIT[ch]  = PW_CAL_LIMITS[ch][1].lowLimit;
    PW_HIGH_LIMIT[ch] = PW_CAL_LIMITS[ch][1].highLimit;
  }

  // =========================================================================
  // 4. MANUAL OFFSETS & 440 ANCHORS
  // =========================================================================
  if (!LittleFS.exists("ManualOffset")) {
    fileManualOffsetFS = LittleFS.open("ManualOffset", "w+");
    for (int i = 0; i < FSManualOffsetBankSize; ++i) ManualOffsetBankBuffer[i] = 0;
    fileManualOffsetFS.write(ManualOffsetBankBuffer, FSManualOffsetBankSize);
  } else {
    fileManualOffsetFS = LittleFS.open("ManualOffset", "r");
    fileManualOffsetFS.read(ManualOffsetBankBuffer, FSManualOffsetBankSize);
  }
  fileManualOffsetFS.close();

  for (int osc = 0; osc < NUM_OSCILLATORS; ++osc) {
    manualCalibrationOffset[osc] = (int8_t)ManualOffsetBankBuffer[osc];
  }

  if (!LittleFS.exists("AmpComp440")) {
    fileAmpComp440FS = LittleFS.open("AmpComp440", "w+");
    for (int i = 0; i < FSAmpComp440BankSize; ++i) AmpComp440BankBuffer[i] = 0;
    fileAmpComp440FS.write(AmpComp440BankBuffer, FSAmpComp440BankSize);
  } else {
    fileAmpComp440FS = LittleFS.open("AmpComp440", "r");
    fileAmpComp440FS.read(AmpComp440BankBuffer, FSAmpComp440BankSize);
  }
  fileAmpComp440FS.close();

  for (int osc = 0; osc < NUM_OSCILLATORS; ++osc) {
    uint16_t v;
    ((uint8_t *)&v)[0] = AmpComp440BankBuffer[osc * 2];
    ((uint8_t *)&v)[1] = AmpComp440BankBuffer[osc * 2 + 1];
    ampComp440[osc] = v;
  }

  if (!LittleFS.exists("AmpCompDutyOffset")) {
    fileAmpCompDutyOffsetFS = LittleFS.open("AmpCompDutyOffset", "w+");
    for (int i = 0; i < FSAmpCompDutyOffsetBankSize; ++i) AmpCompDutyOffsetBankBuffer[i] = 0;
    fileAmpCompDutyOffsetFS.write(AmpCompDutyOffsetBankBuffer, FSAmpCompDutyOffsetBankSize);
  } else {
    fileAmpCompDutyOffsetFS = LittleFS.open("AmpCompDutyOffset", "r");
    fileAmpCompDutyOffsetFS.read(AmpCompDutyOffsetBankBuffer, FSAmpCompDutyOffsetBankSize);
  }
  fileAmpCompDutyOffsetFS.close();

  for (int osc = 0; osc < NUM_OSCILLATORS; ++osc) {
    int16_t v;
    ((uint8_t *)&v)[0] = AmpCompDutyOffsetBankBuffer[osc * 2];
    ((uint8_t *)&v)[1] = AmpCompDutyOffsetBankBuffer[osc * 2 + 1];
    ampCompDutyOffset[osc] = v;
  }

#endif
}

// =============================================================================
// Update Functions (Writing Calibration Data to LittleFS)
// =============================================================================

void update_FS_voice(byte voiceN) {
  byte calibrationDataBytes[FSVoiceDataSize];

  for (int i = 0; i < chanLevelVoiceDataSize; i++) {
    byte *b = (byte *)&calibrationData[i];
    for (int j = 0; j < 4; j++) {
      calibrationDataBytes[i * 4 + j] = b[j];
    }
  }
  uint16_t startByteN = voiceN * FSVoiceDataSize;

  fileVoiceTablesFS = LittleFS.open("voiceTables", "r+");
  fileVoiceTablesFS.seek(startByteN);
  fileVoiceTablesFS.write(calibrationDataBytes, FSVoiceDataSize);
  fileVoiceTablesFS.close();
}

// Persist complete 3-Point calibration for one PW channel (18 Bytes)
void update_FS_PW_Channel(byte ch) {
  if (ch >= NUM_PW_CHANNELS) return;

  for (uint8_t pt = 0; pt < 3; ++pt) {
    pack_pw_limits_point(PWCalBankBuffer, ch, pt, PW_CAL_LIMITS[ch][pt]);
  }

  PW_CENTER[ch]     = PW_CAL_LIMITS[ch][1].center;
  PW_LOW_LIMIT[ch]  = PW_CAL_LIMITS[ch][1].lowLimit;
  PW_HIGH_LIMIT[ch] = PW_CAL_LIMITS[ch][1].highLimit;

  uint16_t startByteN = ch * FSPWDataSize;

  filePWCalFS = LittleFS.open("PWCal3Pt", "r+");
  filePWCalFS.seek(startByteN);
  filePWCalFS.write(&PWCalBankBuffer[startByteN], FSPWDataSize);
  filePWCalFS.close();
}

// Persist top valid pair index from Amp Comp
void update_FS_AmpCompTopPair(byte oscIndex, uint8_t value) {
  if (oscIndex >= NUM_OSCILLATORS) return;
  ampCompTopPair[oscIndex] = value;
  AmpCompTopPairBankBuffer[oscIndex] = value;

  fileAmpCompTopPairFS = LittleFS.open("AmpCompTopPair", "r+");
  fileAmpCompTopPairFS.seek(oscIndex * FSAmpCompTopPairDataSize);
  fileAmpCompTopPairFS.write(&value, FSAmpCompTopPairDataSize);
  fileAmpCompTopPairFS.close();
}

void update_FS_PWCenter(byte voiceN, uint16_t value) {
  if (voiceN >= NUM_PW_CHANNELS) return;
  PW_CAL_LIMITS[voiceN][1].center = value;
  update_FS_PW_Channel(voiceN);
}

void update_FS_PW_High_Limit(byte voiceN, uint16_t value) {
  if (voiceN >= NUM_PW_CHANNELS) return;
  PW_CAL_LIMITS[voiceN][1].highLimit = value;
  update_FS_PW_Channel(voiceN);
}

void update_FS_PW_Low_Limit(byte voiceN, uint16_t value) {
  if (voiceN >= NUM_PW_CHANNELS) return;
  PW_CAL_LIMITS[voiceN][1].lowLimit = value;
  update_FS_PW_Channel(voiceN);
}

void update_FS_ManualCalibrationOffset(byte oscIndex, int8_t value) {
  if (oscIndex >= NUM_OSCILLATORS) return;
  manualCalibrationOffset[oscIndex] = value;

  uint8_t b = (uint8_t)value;
  uint16_t startByteN = oscIndex * FSManualOffsetDataSize;

  fileManualOffsetFS = LittleFS.open("ManualOffset", "r+");
  fileManualOffsetFS.seek(startByteN);
  fileManualOffsetFS.write(&b, FSManualOffsetDataSize);
  fileManualOffsetFS.close();
}

void update_FS_AmpComp440(byte oscIndex, uint16_t value) {
  if (oscIndex >= NUM_OSCILLATORS) return;
  ampComp440[oscIndex] = value;

  byte *b = (byte *)&value;
  uint16_t startByteN = oscIndex * FSAmpComp440DataSize;

  fileAmpComp440FS = LittleFS.open("AmpComp440", "r+");
  fileAmpComp440FS.seek(startByteN);
  fileAmpComp440FS.write(b, FSAmpComp440DataSize);
  fileAmpComp440FS.close();
}

void update_FS_AmpCompDutyOffset(byte oscIndex, int16_t value) {
  if (oscIndex >= NUM_OSCILLATORS) return;
  ampCompDutyOffset[oscIndex] = value;

  byte *b = (byte *)&value;
  uint16_t startByteN = oscIndex * FSAmpCompDutyOffsetDataSize;

  fileAmpCompDutyOffsetFS = LittleFS.open("AmpCompDutyOffset", "r+");
  fileAmpCompDutyOffsetFS.seek(startByteN);
  fileAmpCompDutyOffsetFS.write(b, FSAmpCompDutyOffsetDataSize);
  fileAmpCompDutyOffsetFS.close();
}

// =============================================================================
// Fake Table Seeding & Diagnostics
// =============================================================================

static const uint16_t kFakeAmpPwmRef[] = {
  40, 50, 62, 79, 101, 130, 170, 222, 292, 386,
  511, 675, 924, 1252, 1688, 2231, 3034, 4132, 5632, 7676, 10000
};
static constexpr int kFakeAmpPwmRefCount = (int)(sizeof(kFakeAmpPwmRef) / sizeof(kFakeAmpPwmRef[0]));
static constexpr uint16_t kFakeAmpPwmRefWrap = 10000;
static constexpr uint32_t kFakeUnreachableFreqX100 = 20000000u;

void write_fs_bank(const char* name, const uint8_t* data, size_t size) {
  File f = LittleFS.open(name, "w");
  if (!f) return;
  f.write(data, size);
  f.close();
}

#if PROJECT_INSTRUMENT == 4
static bool fs_file_size_ok(const char* name, size_t expected) {
  File f = LittleFS.open(name, "r");
  if (!f) return false;
  const size_t sz = f.size();
  f.close();
  return sz == expected;
}

static void ensure_pw_fs_banks() {
  if (fs_file_size_ok("PWCal3Pt", FSPWBankSize) &&
      fs_file_size_ok("AmpCompTopPair", FSAmpCompTopPairBankSize)) {
    return;
  }

  for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    uint16_t c = (pwSweepMode == PW_SWEEP_FULL) ? (DIV_COUNTER_PW / 2) : 0;
    PWCalLimits pDefault = { c, 0, (uint16_t)DIV_COUNTER_PW };
    pack_pw_limits_point(PWCalBankBuffer, ch, 0, pDefault);
    pack_pw_limits_point(PWCalBankBuffer, ch, 1, pDefault);
    pack_pw_limits_point(PWCalBankBuffer, ch, 2, pDefault);
  }

  for (uint8_t osc = 0; osc < NUM_OSCILLATORS; ++osc) {
    AmpCompTopPairBankBuffer[osc] = 18;
  }

  write_fs_bank("PWCal3Pt", PWCalBankBuffer, FSPWBankSize);
  write_fs_bank("AmpCompTopPair", AmpCompTopPairBankBuffer, FSAmpCompTopPairBankSize);
}
#endif

void generate_fake_calibration_data(uint8_t osc, uint32_t* out) {
  if (out == nullptr) return;
  if (osc >= NUM_OSCILLATORS) osc = NUM_OSCILLATORS - 1;

  static const float kOscScale[8] = {
    1.00f, 1.02f, 0.98f, 1.01f, 0.99f, 1.03f, 0.97f, 1.00f
  };
  const float oscScale = kOscScale[osc];
  const uint32_t pwmSat = (uint32_t)(0.98f * (float)DIV_COUNTER);

  out[0] = 0;
  out[1] = (uint32_t)ampCompLowestFreqVal;

  const uint8_t headerNote = (uint8_t)(DCO_calibration_start_note - calibration_note_interval);
  out[2] = (uint32_t)(sNotePitches[headerNote - 12] * 100.0f);
  out[3] = (uint32_t)(initManualAmpCompCalibrationVal[osc] + manualCalibrationOffset[osc]);

  bool plateau = false;
  for (int pair = 2; pair < ampCompTableSize; ++pair) {
    const int i = pair * 2;

    if (plateau) {
      out[i]     = kFakeUnreachableFreqX100;
      out[i + 1] = DIV_COUNTER;
      continue;
    }

    const uint8_t note = (uint8_t)(DCO_calibration_start_note + calibration_note_interval * (pair - 2));
    const int pitchIdx = (int)note - 12;
    if (pitchIdx < 0 || pitchIdx >= (int)(sizeof(sNotePitches) / sizeof(sNotePitches[0]))) {
      out[i]     = kFakeUnreachableFreqX100;
      out[i + 1] = DIV_COUNTER;
      plateau = true;
      continue;
    }

    int refIdx = pair - 1;
    if (refIdx < 0) refIdx = 0;
    if (refIdx >= kFakeAmpPwmRefCount) refIdx = kFakeAmpPwmRefCount - 1;

    float pwmF = ((float)kFakeAmpPwmRef[refIdx] / (float)kFakeAmpPwmRefWrap) * (float)DIV_COUNTER * oscScale;
    if (pwmF < 1.0f) pwmF = 1.0f;
    if (pwmF > (float)DIV_COUNTER) pwmF = (float)DIV_COUNTER;
    uint32_t pwm = (uint32_t)(pwmF + 0.5f);

    if (pwm >= pwmSat) {
      out[i]     = (uint32_t)(sNotePitches[pitchIdx] * 100.0f);
      out[i + 1] = DIV_COUNTER;
      plateau = true;
      continue;
    }

    out[i]     = (uint32_t)(sNotePitches[pitchIdx] * 100.0f);
    out[i + 1] = pwm;
  }
}

void seed_fake_calibration_tables(bool force) {
  LittleFS.begin();
  if (!force && LittleFS.exists("voiceTables")) return;

  for (uint8_t osc = 0; osc < NUM_OSCILLATORS; ++osc) {
    generate_fake_calibration_data(osc, calibrationData);

    const uint16_t startByteN = osc * FSVoiceDataSize;
    for (int i = 0; i < chanLevelVoiceDataSize; ++i) {
      const byte* b = (const byte*)&calibrationData[i];
      for (int j = 0; j < 4; ++j) {
        voiceTablesBankBuffer[startByteN + i * 4 + j] = b[j];
      }
    }
  }

  // Seed 3-Point PW default banks (18 Bytes / Channel)
  for (uint8_t ch = 0; ch < NUM_PW_CHANNELS; ++ch) {
    uint16_t c = (pwSweepMode == PW_SWEEP_FULL) ? (DIV_COUNTER_PW / 2) : 0;
    PWCalLimits pDefault = { c, 0, (uint16_t)DIV_COUNTER_PW };

    pack_pw_limits_point(PWCalBankBuffer, ch, 0, pDefault);
    pack_pw_limits_point(PWCalBankBuffer, ch, 1, pDefault);
    pack_pw_limits_point(PWCalBankBuffer, ch, 2, pDefault);
  }

  // Seed TopPair defaults
  for (uint8_t osc = 0; osc < NUM_OSCILLATORS; ++osc) {
    AmpCompTopPairBankBuffer[osc] = 18;
  }

  static constexpr uint16_t kFakeAmpComp440 = DIV_COUNTER / 10;
  for (uint8_t osc = 0; osc < NUM_OSCILLATORS; ++osc) {
    AmpComp440BankBuffer[osc * FSAmpComp440DataSize + 0] = (uint8_t)(kFakeAmpComp440 & 0xFF);
    AmpComp440BankBuffer[osc * FSAmpComp440DataSize + 1] = (uint8_t)((kFakeAmpComp440 >> 8) & 0xFF);
  }

  write_fs_bank("voiceTables", voiceTablesBankBuffer, FSBankSize);
  write_fs_bank("PWCal3Pt", PWCalBankBuffer, FSPWBankSize);
  write_fs_bank("AmpCompTopPair", AmpCompTopPairBankBuffer, FSAmpCompTopPairBankSize);
  write_fs_bank("AmpComp440", AmpComp440BankBuffer, FSAmpComp440BankSize);

  init_FS();
  if (force) {
    precompute_amp_comp_for_engine();
  }
}

#endif  // __FS_IMPL_H__