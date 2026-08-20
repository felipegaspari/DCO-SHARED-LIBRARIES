/**
 * @file tusb_config.h
 * @brief TinyUSB stack device configuration for RP2040/RP2350 USB-MIDI and USB-CDC serial.
 */

 #ifndef _TUSB_CONFIG_H_
 #define _TUSB_CONFIG_H_
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 #ifndef CFG_TUSB_MCU
 #error CFG_TUSB_MCU must be defined
 #endif
 
 #if CFG_TUSB_MCU == OPT_MCU_LPC18XX || CFG_TUSB_MCU == OPT_MCU_LPC43XX || CFG_TUSB_MCU == OPT_MCU_MIMXRT10XX || \
     CFG_TUSB_MCU == OPT_MCU_NUC505 || CFG_TUSB_MCU == OPT_MCU_CXD56
 #define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_HIGH_SPEED)
 #else
 #define CFG_TUSB_RHPORT0_MODE     OPT_MODE_DEVICE
 #endif
 
 #ifndef CFG_TUSB_OS
 #define CFG_TUSB_OS               OPT_OS_PICO
 #endif
 
 #ifndef CFG_TUSB_MEM_SECTION
 #define CFG_TUSB_MEM_SECTION
 #endif
 
 #ifndef CFG_TUSB_MEM_ALIGN
 #define CFG_TUSB_MEM_ALIGN        __attribute__ ((aligned(4)))
 #endif
 
 #ifndef CFG_TUD_ENDPOINT0_SIZE
 #define CFG_TUD_ENDPOINT0_SIZE    64
 #endif
 
 //------------- CLASS CONFIGURATION -------------//
 #define CFG_TUD_CDC               1
 #define CFG_TUD_MSC               0
 #define CFG_TUD_HID               0 
 #define CFG_TUD_MIDI              1 
 #define CFG_TUD_VENDOR            0
 
 // CDC FIFO buffer sizing
 #define CFG_TUD_CDC_RX_BUFSIZE    512
 #define CFG_TUD_CDC_TX_BUFSIZE    512
 #define CFG_TUD_CDC_EP_BUFSIZE    64
 
 // MIDI FIFO buffer sizing
 #define CFG_TUD_MIDI_RX_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 512 : 256)
 #define CFG_TUD_MIDI_TX_BUFSIZE   (TUD_OPT_HIGH_SPEED ? 512 : 64)
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif /* _TUSB_CONFIG_H_ */