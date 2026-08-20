/**
 * @file autotune_context.h
 * @brief Execution context passed across autotuning passes and calibration routines.
 */

 #ifndef DCO_AUTOTUNE_CONTEXT_H
 #define DCO_AUTOTUNE_CONTEXT_H
 
 #include <stdint.h>
 
 /**
  * @struct DCOCalibrationContext
  * @brief Lightweight execution context tracking active note and calibration state pointers.
  */
 struct DCOCalibrationContext {
   uint8_t&  dcoIndex;             ///< Reference to current oscillator under calibration.
   uint8_t&  currentNote;          ///< Reference to current MIDI note index being tuned.
   uint32_t* calibrationData;      ///< Pointer to working calibration buffer.
   int8_t*   manualOffsetByOsc;    ///< Pointer to per-oscillator manual offset array.
   uint16_t* initManualAmpByOsc;   ///< Pointer to initial manual amplitude values.
 
   DCOCalibrationContext(
     uint8_t& dcoIndexRef,
     uint8_t& currentNoteRef,
     uint32_t* calibrationDataPtr,
     int8_t* manualOffsetPtr,
     uint16_t* initManualAmpPtr
   ) : dcoIndex(dcoIndexRef),
       currentNote(currentNoteRef),
       calibrationData(calibrationDataPtr),
       manualOffsetByOsc(manualOffsetPtr),
       initManualAmpByOsc(initManualAmpPtr) {}
 };
 
 #endif  // DCO_AUTOTUNE_CONTEXT_H