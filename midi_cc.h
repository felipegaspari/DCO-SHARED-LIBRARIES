/**
 * @file midi_cc.h
 * @brief 7-bit MIDI Continuous Controller mapping definitions and local block target IDs.
 */

 #ifndef DCO_MIDI_CC_H
 #define DCO_MIDI_CC_H
 
 #include <stdint.h>
 #include <stddef.h>
 
 /// Reserved Pitch Bend range controller.
 #define MIDI_CC_PITCH_BEND_RANGE 42
 
 /**
  * @enum MidiCcCurve
  * @brief Response curves applied to raw 7-bit MIDI CC values.
  */
 enum : uint8_t {
   MIDI_CC_LINEAR   = 0, ///< Linear 0..127 mapping.
   MIDI_CC_EXP_TIME = 1  ///< Exponential curve matching envelope fader responses.
 };
 
 #define MIDI_CC_EXP_BASE 50.0f
 #define MIDI_CC_EXP_MAX 25000
 
 /**
  * @enum LocalCcTarget
  * @brief Virtual target IDs for controls carried via block frames ('a'–'d') instead of 'p' frames.
  */
 enum : uint8_t {
   CC_LOCAL_FIRST = 224,
 
   CC_LOCAL_ADSR_VCA_ATTACK = CC_LOCAL_FIRST,
   CC_LOCAL_ADSR_VCA_DECAY,
   CC_LOCAL_ADSR_VCA_SUSTAIN,
   CC_LOCAL_ADSR_VCA_RELEASE,
 
   CC_LOCAL_ADSR_VCF_ATTACK,
   CC_LOCAL_ADSR_VCF_DECAY,
   CC_LOCAL_ADSR_VCF_SUSTAIN,
   CC_LOCAL_ADSR_VCF_RELEASE,
 
   CC_LOCAL_ADSR_DCO_ATTACK,
   CC_LOCAL_ADSR_DCO_DECAY,
   CC_LOCAL_ADSR_DCO_SUSTAIN,
   CC_LOCAL_ADSR_DCO_RELEASE,
 
   CC_LOCAL_FILTER_CUTOFF,
   CC_LOCAL_FILTER_RESONANCE,
   CC_LOCAL_FILTER_ADSR2_TO_VCF,
   CC_LOCAL_FILTER_LFO2_TO_VCF,
 };
 
 /**
  * @struct MidiCcEntry
  * @brief Lookup table entry mapping a MIDI CC number to an internal target parameter.
  */
 struct MidiCcEntry {
   uint8_t cc;      ///< MIDI CC controller index (0..127).
   uint8_t target;  ///< Canonical ParamId or CC_LOCAL_* target ID.
   int16_t lo;      ///< Value mapped to CC value 0.
   int16_t hi;      ///< Value mapped to CC value 127.
   uint8_t curve;   ///< Response curve (MIDI_CC_LINEAR or MIDI_CC_EXP_TIME).
 };
 
 void midi_cc_handle(uint8_t number, uint8_t value);
 void midi_cc_apply(uint8_t target, int16_t value);
 
 #endif // DCO_MIDI_CC_H