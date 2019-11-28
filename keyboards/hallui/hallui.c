#include "hallui.h"



#include "hal.h"

#include "led.h"

void led_init_ports() {
	palSetPadMode(GPIOC, 13, PAL_MODE_OUTPUT_PUSHPULL);
}

void led_set(uint8_t usb_led) {
    if (usb_led & (1<<USB_LED_CAPS_LOCK)) {
        palClearPad(GPIOC, 13);
    } else {
        palSetPad(GPIOC, 13);
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