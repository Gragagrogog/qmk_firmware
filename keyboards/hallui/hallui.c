#include "hallui.h"



#include "hal.h"

#include "led.h"

#define CAPS_LOCK_LED_PIN LINE_PIN13

void led_init_ports() {
	palSetLineMode(CAPS_LOCK_LED_PIN, PAL_MODE_OUTPUT_PUSHPULL);
}

void led_set(uint8_t usb_led) {
    if (usb_led & (1<<USB_LED_CAPS_LOCK)) {
        palSetLine(CAPS_LOCK_LED_PIN);
    } else {
        palClearLine(CAPS_LOCK_LED_PIN);
    }
}

__attribute__ ((weak))
void matrix_init_user(void) {
}

__attribute__ ((weak))
void matrix_scan_user(void) {
}

void matrix_init_kb(void) {
  matrix_init_user();
};

void matrix_scan_kb(void) {
  matrix_scan_user();
};