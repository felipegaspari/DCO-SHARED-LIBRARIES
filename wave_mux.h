/**
 * @file wave_mux.h
 * @brief Waveform multiplexer and 74HC595 shift register control functions.
 */

 #ifndef DCO_WAVE_MUX_H
 #define DCO_WAVE_MUX_H
 
 #include <stdint.h>
 
 void init_waveSelector();
 // Rebuild all 9 OSC×wave bits from waveEnable[][] and shift out.
 void update_waveSelector();
 // Manual cal: all off, then enable that osc's saw (sub 0) or pulse (sub 1/2 —
// the 440 Hz substage plays the square).
 void waveSelector_manual_calibration(uint8_t stage);
 
 #endif // DCO_WAVE_MUX_H