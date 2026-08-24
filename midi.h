/**
 * @file midi.h
 * @brief Hardware and USB MIDI interface instances and mono note tracking declarations.
 */

 #ifndef DCO_MIDI_H
 #define DCO_MIDI_H
 
 #include <MIDI.h>
 
 extern uint8_t velocity[NUM_VOICES_TOTAL];
 
 Adafruit_USBD_MIDI usb_midi;
 
 MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
 MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI_SERIAL);
 
 void init_midi();
 void mono_note_stack_clear();
 
 #endif // DCO_MIDI_H