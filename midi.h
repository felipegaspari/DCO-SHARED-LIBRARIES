/**
 * @file midi.h
 * @brief Hardware and USB MIDI interface instances and mono note tracking declarations.
 */

 #ifndef DCO_MIDI_H
 #define DCO_MIDI_H
 
 #include <MIDI.h>
 #include <Adafruit_TinyUSB.h>
 #include "hardware/uart.h"
 #include "hardware/irq.h"
 
 extern uint8_t velocity[NUM_VOICES_TOTAL];
 
 // -----------------------------------------------------------------------------
 // 1. Lock-free SRAM Ring Buffer (Single Producer ISR, Single Consumer Loop)
 // -----------------------------------------------------------------------------
 struct MidiUartRingBuffer {
   static constexpr uint16_t SIZE = 256;
   static constexpr uint16_t MASK = SIZE - 1;
   uint8_t buf[SIZE];
   volatile uint16_t head = 0;
   volatile uint16_t tail = 0;
 
   inline void push(uint8_t b) {
     uint16_t next = (head + 1) & MASK;
     if (next != tail) {
       buf[head] = b;
       head = next;
     }
   }
 
   inline int available() const {
     return (head - tail) & MASK;
   }
 
   inline uint8_t read() {
     if (head == tail) return 0;
     uint8_t b = buf[tail];
     tail = (tail + 1) & MASK;
     return b;
   }
 };
 
 extern MidiUartRingBuffer midi_din_rx_buf;
 void __not_in_flash_func(on_midi_uart_rx)();
 
 // -----------------------------------------------------------------------------
 // 2. High-Speed Stream Interface for FortySevenEffects SerialMIDI Adapter
 // -----------------------------------------------------------------------------
 struct MidiUartTransport {
   inline void begin(long baud = 31250) {}
   inline void end() {}
 
   // Called by FortySevenEffects SerialMIDI::available()
   inline int available() {
     return midi_din_rx_buf.available(); // ~3 cycles in SRAM
   }
 
   // Called by FortySevenEffects SerialMIDI::read()
   inline uint8_t read() {
     return midi_din_rx_buf.read();
   }
 
   inline void write(uint8_t byte) {
     while (!uart_is_writable(uart0)) {
       tight_loop_contents();
     }
     uart_get_hw(uart0)->dr = byte;
   }
 };
 
 // -----------------------------------------------------------------------------
 // 3. MIDI Instances
 // -----------------------------------------------------------------------------
 extern Adafruit_USBD_MIDI usb_midi;
 extern MidiUartTransport midi_din_transport;
 
 MIDI_CREATE_INSTANCE(Adafruit_USBD_MIDI, usb_midi, MIDI_USB);
 MIDI_CREATE_INSTANCE(MidiUartTransport, midi_din_transport, MIDI_SERIAL);
 
 void init_midi();
 void mono_note_stack_clear();
 
 #endif // DCO_MIDI_H