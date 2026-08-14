#ifndef __AUTOTUNE_CONTEXT_H__
#define __AUTOTUNE_CONTEXT_H__

#include <stdint.h>

// Lightweight execution context passed across calibration routines
struct DCOCalibrationContext {
  uint8_t&  dcoIndex;
  uint8_t&  currentNote;
  uint32_t* calibrationData;
  int8_t*   manualOffsetByOsc;
  uint16_t* initManualAmpByOsc;

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

#endif  // __AUTOTUNE_CONTEXT_H__