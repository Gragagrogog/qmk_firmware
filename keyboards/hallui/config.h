/*
Copyright 2015 Jun Wako <wakojun@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

/* USB Device descriptor parameter */
#define VENDOR_ID       0x1c10
#define PRODUCT_ID      0xb04c
#define DEVICE_VER      0x0001
#define MANUFACTURER    Kutululu
#define PRODUCT         Hallui/QMK
#define DESCRIPTION     test board for qmk

/* key matrix size */
#define MATRIX_ROWS 1
#define MATRIX_COLS 1

/* define if matrix has ghost */
//#define MATRIX_HAS_GHOST

/* Set 0 if debouncing isn't needed */
#define DEBOUNCE    0

//#define TAPPING_TERM 500

/* Mechanical locking support. Use KC_LCAP, KC_LNUM or KC_LSCR instead in keymap */
//#define LOCKING_SUPPORT_ENABLE
/* Locking resynchronize hack */
//#define LOCKING_RESYNC_ENABLE

/* key combination for command */
#define IS_COMMAND() ( \
    keyboard_report->mods == (MOD_BIT(KC_LCTRL) | MOD_BIT(KC_RCTRL)) \
)

#define MAGIC_KEY_DEBUG D
#define MAGIC_KEY_DEBUG_MATRIX X
#define MAGIC_KEY_HELP	H

#define DEBUG_MATRIX_SCAN_RATE
//#define DEBUG_MOUSE_SCAN_RATE

/*#define NUMBER_OF_ENCODERS 1
//#define ENCODERS_PAL_A_PORT { GPIOB }
//#define ENCODERS_PAL_A_BIT  { 19 }
//#define ENCODERS_PAL_B_PORT { GPIOA }
//#define ENCODERS_PAL_B_BIT  { 5 }
#ifndef B19
	#define B19 PAL_LINE(GPIOB, 19)
#endif

#define ENCODERS_PAD_A { B19 }
#define ENCODERS_PAD_B { A5 }
#define ENCODER_RESOLUTION 2 */

/*
 * Feature disable options
 *  These options are also useful to firmware size reduction.
 */

/* disable debug print */
//#define NO_DEBUG

/* disable print */
//#define NO_PRINT

/* disable action features */
//#define NO_ACTION_LAYER
//#define NO_ACTION_TAPPING
//#define NO_ACTION_ONESHOT
//#define NO_ACTION_MACRO
//#define NO_ACTION_FUNCTION
