/**
 * @file mcu_board.h
 * @brief Microcontroller board-level GPIO helpers (RT6150 SMPS PS mode, user key, analog rail fixes).
 */

#ifndef DCO_MCU_BOARD_H
#define DCO_MCU_BOARD_H

#include <stdint.h>

/**
 * @brief Initializes board-specific hardware pins (Pico SMPS ripple reduction and carrier board fixes).
 */
static inline void mcu_board_pins_init() {
  if (SMPS_PS_PIN != MCU_PIN_UNASSIGNED) {
    pinMode(SMPS_PS_PIN, OUTPUT);
    digitalWrite(SMPS_PS_PIN, HIGH);
  }
  if (BOARD_FIX_PIN != MCU_PIN_UNASSIGNED) {
    pinMode(BOARD_FIX_PIN, OUTPUT);
    digitalWrite(BOARD_FIX_PIN, HIGH);
  }
  if (USER_KEY_PIN != MCU_PIN_UNASSIGNED) {
    pinMode(USER_KEY_PIN, INPUT_PULLUP);
  }
}

/**
 * @brief Debounces WeAct user button to trigger A440 test notes during bench bringup.
 */
static inline void user_key_task() {
  if (USER_KEY_PIN == MCU_PIN_UNASSIGNED) return;
  static uint8_t stable = HIGH;
  static uint8_t raw_last = HIGH;
  static bool note_held = false;
  static uint32_t edge_ms = 0;

  uint8_t raw = (uint8_t)digitalRead(USER_KEY_PIN);
  uint32_t t = millis();
  if (raw != raw_last) {
    raw_last = raw;
    edge_ms = t;
  } else if ((uint32_t)(t - edge_ms) >= 20u && raw != stable) {
    stable = raw;
    if (!calibrationFlag) {
      if (stable == LOW) {
        note_on(69, 100);
        note_held = true;
      } else if (note_held) {
        note_off(69);
        note_held = false;
      }
    }
  }
  if (calibrationFlag && note_held) {
    note_off(69);
    note_held = false;
  }
}

#endif // DCO_MCU_BOARD_H