//#include <stdint.h>
//#include <stdbool.h>
#include <string.h>
#include "hal.h"
#include "timer.h"
#include "wait.h"
#include "print.h"
#include "matrix.h"
#include "debug.h"

/* matrix state(1:on, 0:off) */
static matrix_row_t matrix[MATRIX_ROWS];
static matrix_row_t matrix_debouncing[MATRIX_ROWS];
static bool debouncing = false;
static uint16_t debouncing_time = 0;

#ifdef DEBUG_MATRIX_SCAN_RATE
uint32_t matrix_timer;
uint32_t matrix_scan_count;
#endif

void matrix_init(void)
{   
    #ifdef DEBUG_MATRIX_SCAN_RATE
      matrix_timer = timer_read32();
      matrix_scan_count = 0;
    #endif

    /* Column(sense) */
     //palSetPadMode(GPIOD,  7,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 5 //BLOWN PIB, USING 23 INSTEAD
     palSetPadMode(GPIOD,  4,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 6
     palSetPadMode(GPIOD,  2,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 7
     palSetPadMode(GPIOD,  3,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 8
     palSetPadMode(GPIOC,  3,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 9
     palSetPadMode(GPIOC,  4,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 10
     palSetPadMode(GPIOC,  6,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 11
     palSetPadMode(GPIOC,  7,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 12
     palSetPadMode(GPIOD,  1,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 14
     palSetPadMode(GPIOC,  0,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 15
     palSetPadMode(GPIOB,  0,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 16
     palSetPadMode(GPIOB,  1,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 17
     palSetPadMode(GPIOB,  3,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 18
     palSetPadMode(GPIOB,  2,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 19
     palSetPadMode(GPIOD,  5,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 20
     palSetPadMode(GPIOD,  6,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 21
     palSetPadMode(GPIOC,  1,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 22

     palSetPadMode(GPIOC,  2,  PAL_MODE_INPUT_PULLDOWN); // teensy pin 23

    /* Row(strobe) */
    palSetPadMode(GPIOB, 16,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 0
    palSetPadMode(GPIOB, 17,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 1
    palSetPadMode(GPIOD,  0,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 2
    palSetPadMode(GPIOA, 12,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 3
    palSetPadMode(GPIOA, 13,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 4
    //palSetPadMode(GPIOC,  2,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 23
    palSetPadMode(GPIOE,  1,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 26
    palSetPadMode(GPIOC,  9,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 27
    palSetPadMode(GPIOC,  8,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 28
    palSetPadMode(GPIOA,  4,   PAL_MODE_OUTPUT_PUSHPULL);    // teensy pin 33 - additional keys


    memset(matrix, 0, MATRIX_ROWS * sizeof(matrix_row_t));
    memset(matrix_debouncing, 0, MATRIX_ROWS * sizeof(matrix_row_t));

    matrix_init_quantum();
}

uint8_t matrix_scan(void)
{
    #ifdef DEBUG_MATRIX_SCAN_RATE
      matrix_scan_count++;

      uint32_t timer_now = timer_read32();
      if (TIMER_DIFF_32(timer_now, matrix_timer)>1000) {
        dprintf("matrix scan frequency: %u \n", matrix_scan_count);
        if (debug_matrix) {
            matrix_print();
        }

        matrix_timer = timer_now;
        matrix_scan_count = 0;
      }
    #endif


    for (int row = 0; row < MATRIX_ROWS; row++) {
        matrix_row_t data = 0;
        // strobe row
        switch (row) {
            case 0: palSetPad(GPIOB, 16);   break;
            case 1: palSetPad(GPIOB, 17);   break;
            case 2: palSetPad(GPIOD,  0);   break;
            case 3: palSetPad(GPIOA, 12);   break;
            case 4: palSetPad(GPIOA, 13);   break;
            case 5: palSetPad(GPIOE,  1);   break;
            case 6: palSetPad(GPIOC,  9);   break;
            case 7: palSetPad(GPIOC,  8);   break;
            case 8: palSetPad(GPIOA,  4);   break;
        }

        // need wait to settle pin state
        // if you wait too short, or have a too high update rate
        // the keyboard might freeze, or there might not be enough
        // processing power to update the LCD screen properly.
        // 20us, or two ticks at 100000Hz seems to be OK
        wait_us(1); //20 originally, 1 seems ok

        uint32_t c_port = palReadPort(GPIOC);

        data =   (palReadPort(GPIOB) & 0xFUL)         |          
                ((palReadPort(GPIOD) & 0x7EUL) <<  3) |
                ((c_port             & 0x1FUL) << 10) |
                ((c_port             & 0xC0UL) <<  9);

        // un-strobe row
        switch (row) {
            case 0: palClearPad(GPIOB, 16);   break;
            case 1: palClearPad(GPIOB, 17);   break;
            case 2: palClearPad(GPIOD,  0);   break;
            case 3: palClearPad(GPIOA, 12);   break;
            case 4: palClearPad(GPIOA, 13);   break;
            case 5: palClearPad(GPIOE,  1);   break;
            case 6: palClearPad(GPIOC,  9);   break;
            case 7: palClearPad(GPIOC,  8);   break;
            case 8: palClearPad(GPIOA,  4);   break;
        }

        if (matrix_debouncing[row] != data) {
            matrix_debouncing[row] = data;
            debouncing = true;
            debouncing_time = timer_read();
        }
    }

    if (debouncing && timer_elapsed(debouncing_time) > DEBOUNCE) {
        for (int row = 0; row < MATRIX_ROWS; row++) {
            matrix[row] = matrix_debouncing[row];
        }
        debouncing = false;
    }

    matrix_scan_quantum();
    return 1;
}

bool matrix_is_on(uint8_t row, uint8_t col)
{
    return (matrix[row] & (1 << col));
}

matrix_row_t matrix_get_row(uint8_t row)
{
    return matrix[row];
}

void matrix_print(void)
{
    xprintf("\nr/c 0123456789ABCDEFG\n");
    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        xprintf("%02X: ", row);
        matrix_row_t data = matrix_get_row(row);
        for (int col = 0; col < MATRIX_COLS; col++) {
            if (data & (1<<col))
                xprintf("1");
            else
                xprintf("0");
        }
        xprintf("\n");
    }

    //wait_ms(50); 
}
