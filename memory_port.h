#pragma once

#include <Arduino.h>
// =============================================================================
// memory_port.h – portable fast-memory placement
// Used by DCO4-REBORN, DCO3-MONOSYNTH and all shared libraries
// =============================================================================

// ----- Master switches (override in project_config.h or build flags) -----
#ifndef SRAM_HOT_ENABLE
#define SRAM_HOT_ENABLE  0
#endif

#ifndef SRAM_DATA_ENABLE
#define SRAM_DATA_ENABLE 0
#endif

// -----------------------------------------------------------------------------
// CODE (functions) → fastest instruction RAM
// -----------------------------------------------------------------------------
#ifndef SRAM_HOT
  #if SRAM_HOT_ENABLE
    #if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || defined(PICO_RP2350)
      #ifndef __not_in_flash_func
        #define __not_in_flash_func(fn) fn
      #endif
      #define SRAM_HOT(fn) __not_in_flash_func(fn)

    #elif defined(STM32H7) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
      #define SRAM_HOT(fn) __attribute__((section(".itcmram"), noinline, used)) fn

    #else
      #define SRAM_HOT(fn) fn
    #endif
  #else
    #define SRAM_HOT(fn) fn
  #endif
#endif

// -----------------------------------------------------------------------------
// DATA (variables / tables) → fastest data RAM
// -----------------------------------------------------------------------------
#ifndef SRAM_DATA
  #if SRAM_DATA_ENABLE
    #if defined(ARDUINO_ARCH_RP2040) || defined(PICO_RP2040) || defined(PICO_RP2350)
      // Force into main SRAM (especially useful for large const tables)
      #define SRAM_DATA  __attribute__((section(".data")))
      // alternative classic Pico style:
      // #define SRAM_DATA  __not_in_flash("data")

    #elif defined(STM32H7) || defined(STM32H750xx) || defined(ARDUINO_ARCH_STM32)
      // DTCM
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