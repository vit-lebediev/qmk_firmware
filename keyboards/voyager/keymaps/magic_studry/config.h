/*
  Set any config.h overrides for your specific keymap here.
  See config.h options at https://docs.qmk.fm/#/config_options?id=the-configh-file
*/

#define USB_SUSPEND_WAKEUP_DELAY 0
#define FIRMWARE_VERSION u8"gnz47/wlOYj"
#define SERIAL_NUMBER "LWgZM/94N6j"
#define RAW_USAGE_PAGE 0xFF60
#define RAW_USAGE_ID 0x61
#define LAYER_STATE_8BIT
#define COMBO_COUNT 10
#ifndef TAPPING_TOGGLE
#    define TAPPING_TOGGLE 2
#endif

#define RGB_MATRIX_STARTUP_SPD 60
