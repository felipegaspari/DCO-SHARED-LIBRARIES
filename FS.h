/**
 * @file FS.h
 * @brief Calibration flash storage bank sizes, data structures, and file system API prototypes.
 * @details Shared across DCO3-MONOSYNTH and DCO4-REBORN. Defines the sizing invariants
 *          for LittleFS storage buffers (voice tables, PW limits, amp compensation).
 */

 #ifndef DCO_FS_H
 #define DCO_FS_H
 
 #include <stdint.h>
 #include <stddef.h>
 
 /// Total bytes per voice calibration table: 22 notes * 2 words * 4 bytes = 176 bytes.
 static constexpr uint16_t FSVoiceDataSize = 22 * 2 * 4;
 /// Total bytes for all oscillator calibration tables.
 static constexpr uint16_t FSBankSize      = FSVoiceDataSize * NUM_OSCILLATORS;
 
 // =============================================================================
 // 1. 3-Point PW Calibration Bank (18 Bytes per Channel)
 // =============================================================================
 /// Number of pulse-width calibration anchor points across the keybed.
 static constexpr uint8_t  kPWCalPoints            = 3;
 /// Size in bytes for 1 calibration point (3 x uint16_t = 6 bytes).
 static constexpr uint16_t FSPWLimitsPointDataSize = 6;
 /// Total calibration size per PW channel (18 bytes).
 static constexpr uint16_t FSPWDataSize            = FSPWLimitsPointDataSize * kPWCalPoints;
 /// Total size for all PW channels across the synth.
 static constexpr uint16_t FSPWBankSize            = FSPWDataSize * NUM_PW_CHANNELS;
 
 /**
  * @struct PWCalLimits
  * @brief Raw pulse-width calibration limits for a single anchor point (6 bytes).
  */
 struct PWCalLimits {
   uint16_t center;    ///< Center 50% duty pulse-width PWM counter value.
   uint16_t lowLimit;  ///< Minimum narrow-pulse threshold before oscillator stall.
   uint16_t highLimit; ///< Maximum wide-pulse threshold before oscillator stall.
 };
 
 // =============================================================================
 // 2. Dual-Engine Pitch Tracking RAM Cache Struct
 // =============================================================================
 #ifdef USE_FLOAT_VOICE_TASK
 /**
  * @struct PWTrackCache
  * @brief Precomputed reciprocal spans for floating-point pulse-width pitch tracking.
  */
 struct PWTrackCache {
   float f0, f1, f2;  ///< Anchor frequencies in Hz.
   float invSpan01;   ///< Precomputed 1.0f / (f1 - f0).
   float invSpan12;   ///< Precomputed 1.0f / (f2 - f1).
 };
 #else
 /**
  * @struct PWTrackCache
  * @brief Precomputed reciprocal spans for Q24 fixed-point pulse-width pitch tracking.
  */
 struct PWTrackCache {
   uint32_t f0, f1, f2;      ///< Anchor frequencies in Q24 fixed-point format.
   uint32_t invSpan01_q24;  ///< Precomputed (1 << 24) / (f1 - f0).
   uint32_t invSpan12_q24;  ///< Precomputed (1 << 24) / (f2 - f1).
 };
 #endif
 
 // =============================================================================
 // 3. Amp-Comp Top Valid Pair Storage
 // =============================================================================
 static constexpr uint16_t FSAmpCompTopPairDataSize    = 1;
 static constexpr uint16_t FSAmpCompTopPairBankSize    = FSAmpCompTopPairDataSize * NUM_OSCILLATORS;
 
 static constexpr uint16_t FSManualOffsetDataSize      = 1;
 static constexpr uint16_t FSManualOffsetBankSize      = FSManualOffsetDataSize * NUM_OSCILLATORS;
 static constexpr uint16_t FSAmpComp440DataSize        = 2;
 static constexpr uint16_t FSAmpComp440BankSize        = FSAmpComp440DataSize * NUM_OSCILLATORS;
 static constexpr uint16_t FSAmpCompDutyOffsetDataSize = 2;
 static constexpr uint16_t FSAmpCompDutyOffsetBankSize = FSAmpCompDutyOffsetDataSize * NUM_OSCILLATORS;
 
 static constexpr uint16_t chanLevelVoiceDataSize      = FSVoiceDataSize / 4;
 
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
 
 // API Prototypes
 void init_FS();
 void precompute_pw_tracking_cache();
 void update_FS_voice(uint8_t voiceN);
 void update_FS_PW_Channel(uint8_t ch);
 void update_FS_AmpCompTopPair(uint8_t oscIndex, uint8_t value);
 void update_FS_PWCenter(uint8_t voiceN, uint16_t value);
 void update_FS_PW_High_Limit(uint8_t voiceN, uint16_t value);
 void update_FS_PW_Low_Limit(uint8_t voiceN, uint16_t value);
 void update_FS_ManualCalibrationOffset(uint8_t oscIndex, int8_t value);
 void update_FS_AmpComp440(uint8_t oscIndex, uint16_t value);
 void update_FS_AmpCompDutyOffset(uint8_t oscIndex, int16_t value);
 void seed_fake_calibration_tables(bool force = false);
 void write_fs_bank(const char* name, const uint8_t* data, size_t size);
 
 #endif // DCO_FS_H