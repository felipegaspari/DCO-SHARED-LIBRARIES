#pragma once

#include <Arduino.h>

// =============================================================================
// MASTER SWITCHES (Default to 1 for high-performance audio engine)
// =============================================================================
#ifndef SRAM_HOT_ENABLE
  #define SRAM_HOT_ENABLE  1
#endif

#ifndef SRAM_DATA_ENABLE
  #define SRAM_DATA_ENABLE 1
#endif

// =============================================================================
// CODE PLACEMENT (Functions -> Instruction RAM / ITCM)
// =============================================================================
#ifndef SRAM_HOT
  #if SRAM_HOT_ENABLE
    #if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || defined(PICO_RP2350)
      #ifndef __not_in_flash_func
        #define __not_in_flash_func(fn) fn
      #endif
      #define SRAM_HOT(fn) __not_in_flash_func(fn)

    #elif defined(STM32H7) || defined(STM32H7xx) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
      // Forces function into .itcmram, prevents inlining into Flash callers, and stops GCC from discarding it
      #define SRAM_HOT(fn) __attribute__((section(".itcmram"), noinline, used)) fn

    #else
      #define SRAM_HOT(fn) fn
    #endif
  #else
    #define SRAM_HOT(fn) fn
  #endif
#endif

// =============================================================================
// DATA PLACEMENT (Variables / Lookups -> Data RAM / DTCM)
// =============================================================================
#ifndef SRAM_DATA
  #if SRAM_DATA_ENABLE
    #if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || defined(PICO_RP2350)
      #define SRAM_DATA  __attribute__((section(".data")))

    #elif defined(STM32H7) || defined(STM32H7xx) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
      #define SRAM_DATA  __attribute__((section(".dtcm"), aligned(4)))

    #else
      #define SRAM_DATA
    #endif
  #else
    #define SRAM_DATA
  #endif
#endif
// Backwards-compatible aliases (so older code keeps working)
#ifndef DTCM_DATA
#define DTCM_DATA  SRAM_DATA
#endif

#ifndef DTCM_BSS
#define DTCM_BSS   SRAM_DATA
#endif