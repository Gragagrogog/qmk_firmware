#include "kutubo.h"

const uint16_t keymaps[][MATRIX_ROWS][MATRIX_COLS] = { //	                  +                                          THIS SHIT                        THIS SHIT
    [0] = { //0       1         2         3         4          5               6      7            8                9            A (10)       B (11)       C (12)        D (13)   E (14)  F (15)     G (16)     BIT INDEX                                                                                                          
        {KC_NO,       KC_T,     KC_P,     KC_J,     KC_NO,     KC_INSERT,      KC_U,  KC_MINUS,    KC_RBRACKET,     KC_KP_5,     KC_NO,       KC_KP_3,     KC_RIGHT,      KC_3,    KC_Q,   KC_RCTRL,  KC_1}, 
        {KC_RALT,     KC_V,     KC_QUOTE, KC_SPACE, KC_NO,     KC_APPLICATION, KC_B,  KC_NO,       KC_LEFT,         KC_KP_6,     KC_NO,       KC_DELETE,   KC_DOWN,       KC_NO,   KC_Z,   KC_NO,     KC_NO}, 
        {KC_NO,       KC_G,     KC_NO,    KC_M,     KC_RSHIFT, KC_NO,          KC_H,  KC_NO,       KC_UP,           KC_KP_DOT,   KC_NO,       KC_HOME,     KC_BSLASH,     KC_D,    KC_S,   KC_NO,     KC_CAPSLOCK}, 
        {KC_NO,       KC_5,     KC_O,     KC_I,     KC_NO,     KC_KP_7,        KC_7,  KC_0,        KC_EQUAL,        KC_KP_2,     KC_NO,       KC_KP_4,     KC_BSPACE,     KC_4,    KC_2,   KC_LCTRL,  KC_GRAVE}, 
        {KC_NO,       KC_F6,    KC_F10,   KC_F9,    KC_NO,     KC_KP_8,        KC_F7, KC_F11,      KC_NUMLOCK,      KC_KP_ENTER, KC_NO,       KC_KP_SLASH, KC_END,        KC_F5,   KC_F3,  KC_NO,     KC_ESCAPE}, 
        {KC_LALT,     KC_F,     KC_DOT,   KC_COMMA, KC_NO,     KC_NO,          KC_N,  KC_SLASH,    KC_PGDOWN,       KC_PGUP,     KC_NO,       KC_KP_9,     KC_SCOLON,     KC_C,    KC_X,   KC_NO,     KC_A}, 
        {KC_NO,       KC_R,     KC_L,     KC_K,     KC_NO,     KC_KP_ASTERISK, KC_Y,  KC_LBRACKET, KC_NONUS_BSLASH, KC_NO,       KC_LGUI,     KC_KP_0,     KC_ENTER,      KC_E,    KC_W,   KC_NO,     KC_TAB}, 
        {KC_NO,       KC_6,     KC_9,     KC_F8,    KC_LSHIFT, KC_KP_MINUS,    KC_8,  KC_F12,      KC_PSCREEN,      KC_KP_1,     KC_NO,       KC_KP_PLUS,  KC_PAUSE,      KC_F4,   KC_F2,  KC_NO,     KC_F1}, 
        {KC_NO,       KC_NO,    KC_NO,    KC_NO,    KC_NO,     KC_NO,          KC_NO, KC_NO,       KC_F24,          KC_F23,      KC_NO,       KC_F22,      KC_F21,        KC_NO,   KC_NO,  KC_NO,     KC_F20}, 
    },
};

const uint16_t fn_actions[] = {

};

// Runs just one time when the keyboard initializes.
void matrix_init_user(void) {

};

// Runs constantly in the background, in a loop.
void matrix_scan_user(void) {

};

void encoder_update_user(uint8_t index, bool clockwise) {
	if (clockwise) {
		//print("Scroll UP \n");
		//unregister_code(KC_AUDIO_VOL_DOWN);
		tap_code(KC_WH_U); //KC_AUDIO_VOL_UP);
	} else {
		//print("Scroll DOWN \n");
		//unregister_code(KC_AUDIO_VOL_UP);
		tap_code(KC_WH_D); //KC_AUDIO_VOL_DOWN);
	} 
}

#ifdef OLED_DRIVER_ENABLE
void oled_task_user(void) {
  // Host Keyboard Layer Status
  oled_write_P(PSTR("Layer: "), false);
  switch (biton32(layer_state)) {
    case 0:
      oled_write_P(PSTR("Default\n"), false);
      break;
    /*case _FN:
      oled_write_P(PSTR("FN\n"), false);
      break;
    case _ADJ:
      oled_write_P(PSTR("ADJ\n"), false);
      break;*/
    default:
      // Or use the write_ln shortcut over adding '\n' to the end of your string
      oled_write_ln_P(PSTR("Undefined"), false);
  }

  // Host Keyboard LED Status
  uint8_t led_usb_state = host_keyboard_leds();
  oled_write_P(led_usb_state & (1<<USB_LED_NUM_LOCK) ? PSTR("NUMLCK ") : PSTR("       "), false);
  oled_write_P(led_usb_state & (1<<USB_LED_CAPS_LOCK) ? PSTR("CAPLCK ") : PSTR("       "), false);
  oled_write_P(led_usb_state & (1<<USB_LED_SCROLL_LOCK) ? PSTR("SCRLCK ") : PSTR("       "), false);
}
#endif


/*
bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  // If console is enabled, it will print the matrix position and status of each key pressed
#ifdef CONSOLE_ENABLE
    uprintf("KL: kc: %u, col: %u, row: %u, pressed: %u\n", keycode, record->event.key.col, record->event.key.row, record->event.pressed);
#endif 
  return true;
}*/