//#include <stdint.h>
//#include <stdbool.h>
#include <string.h>
//#include "ch.h"
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


// Create buffer to store ADC results. This is
// one-dimensional interleaved array

/* at 1000Hz, blue pill's ADC1 is capable of taking
    3 samples (ADC_SAMPLE_7P5 sample time, 8x analog 16-channel multiplexers) - 384 kS/s
    or
    4 samples (ADC_SAMPLE_1P5 sample time, 8x analog 16-channel multiplexers) - 512 kS/s

    1.5 ADC CLOCK sample time seems to work fine if there's no filtering on the inputs
*/
#define ADC_BUF_DEPTH 4 // depth of buffer
#define HALL_SAMPLE_TIME ADC_SAMPLE_1P5 //time select for charging internal capacitor, the actual adc convertion time is govern my STM32_ADCPRE
#define ADC_CH_NUM 8    // number of used ADC channels
#define MULTIPLEXER_NUM_CHANNELS 16
static uint8_t curent_mux_channel = 0;
static adcsample_t samples[ADC_BUF_DEPTH * ADC_CH_NUM * MULTIPLEXER_NUM_CHANNELS]; // results array

#define MULTIPLEXER_S0 A8
#define MULTIPLEXER_S1 A9
#define MULTIPLEXER_S2 A10
#define MULTIPLEXER_S3 B15

#define ADC_SCOPE_TRIG_OUT_PIN B7

static void adc_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {
    (void) buffer;
    (void) n;
    if (adcp->state == ADC_COMPLETE) {
        #ifdef ADC_SCOPE_TRIG_OUT_PIN
        if (curent_mux_channel == 0){
            palWriteLine(ADC_SCOPE_TRIG_OUT_PIN, 1);
        } else if (curent_mux_channel == 1){
            palWriteLine(ADC_SCOPE_TRIG_OUT_PIN, 0);
        }
        #endif

        curent_mux_channel++;
        switch (curent_mux_channel) {
            case  1: palWriteLine(MULTIPLEXER_S3,  1); break; // NEXT CHANNEL:  8
            case  2: palWriteLine(MULTIPLEXER_S2,  1); break; // NEXT CHANNEL: 12
            case  3: palWriteLine(MULTIPLEXER_S0,  1); break; // NEXT CHANNEL: 13
            case  4: palWriteLine(MULTIPLEXER_S2,  0); break; // NEXT CHANNEL:  9
            case  5: palWriteLine(MULTIPLEXER_S1,  1); break; // NEXT CHANNEL: 11
            case  6: palWriteLine(MULTIPLEXER_S2,  1); break; // NEXT CHANNEL: 15
            case  7: palWriteLine(MULTIPLEXER_S3,  0); break; // NEXT CHANNEL:  7
            case  8: palWriteLine(MULTIPLEXER_S1,  0); break; // NEXT CHANNEL:  5
            case  9: palWriteLine(MULTIPLEXER_S0,  0); break; // NEXT CHANNEL:  4
            case 10: palWriteLine(MULTIPLEXER_S1,  1); break; // NEXT CHANNEL:  6
            case 11: palWriteLine(MULTIPLEXER_S3,  1); break; // NEXT CHANNEL: 14
            case 12: palWriteLine(MULTIPLEXER_S2,  0); break; // NEXT CHANNEL: 10
            case 13: palWriteLine(MULTIPLEXER_S3,  0); break; // NEXT CHANNEL:  2
            case 14: palWriteLine(MULTIPLEXER_S0,  1); break; // NEXT CHANNEL:  3
            case 15: palWriteLine(MULTIPLEXER_S1,  0); break; // NEXT CHANNEL:  1
            case 16: palWriteLine(MULTIPLEXER_S0,  0); curent_mux_channel = 0; break; // NEXT CHANNEL:  0
        }
        adcStartConversionI(&ADCD1, &stm32f103_conversion_group, &samples[ADC_BUF_DEPTH * ADC_CH_NUM * curent_mux_channel], ADC_BUF_DEPTH);
    }
}

static const ADCConversionGroup stm32f103_conversion_group = {
    FALSE,                            //CIRCULAR
    (uint16_t)ADC_CH_NUM,                        //NUMB OF CH
    adc_callback,                             //ADC CALLBACK
    NULL,                             //ADC ERROR CALLBACK
    0,                                // CR1 
    0, //ADC_CR2_SWSTART,                  // CR2 

    0,                                // SMPR1 (Sample times for channel 10-18)
    ADC_SMPR2_SMP_AN0(HALL_SAMPLE_TIME) |
    ADC_SMPR2_SMP_AN1(HALL_SAMPLE_TIME) |
    ADC_SMPR2_SMP_AN2(HALL_SAMPLE_TIME) |
    ADC_SMPR2_SMP_AN3(HALL_SAMPLE_TIME) |
    ADC_SMPR2_SMP_AN4(HALL_SAMPLE_TIME) |
    ADC_SMPR2_SMP_AN5(HALL_SAMPLE_TIME) |
    ADC_SMPR2_SMP_AN6(HALL_SAMPLE_TIME) |
    ADC_SMPR2_SMP_AN7(HALL_SAMPLE_TIME),             // SMPR2 (Sample times for channel 0-9)
    ADC_SQR1_NUM_CH(ADC_CH_NUM),       // SQR1 (Conversion group sequence 13th-16th + sequence length.)
    ADC_SQR2_SQ7_N(ADC_CHANNEL_IN6) |
    ADC_SQR2_SQ8_N(ADC_CHANNEL_IN7),   // SQR2 (Conversion group sequence 7th-12th)
    ADC_SQR3_SQ1_N(ADC_CHANNEL_IN0) |
    ADC_SQR3_SQ2_N(ADC_CHANNEL_IN1) |
    ADC_SQR3_SQ3_N(ADC_CHANNEL_IN2) |
    ADC_SQR3_SQ4_N(ADC_CHANNEL_IN3) |
    ADC_SQR3_SQ5_N(ADC_CHANNEL_IN4) |
    ADC_SQR3_SQ6_N(ADC_CHANNEL_IN5)  // SQR3 (Conversion group sequence 1st-6th)
}; 


/*
static void gptcallback1(GPTDriver *gptp) {
    (void) gptp;
    if (ADCD1.state == ADC_READY) {
        #ifdef ADC_SCOPE_TRIG_OUT_PIN
        if (curent_mux_channel == 0){
            palWriteLine(B7, 1);
        } else if (curent_mux_channel == 1){
            palWriteLine(B7, 0);
        }
        #endif

        curent_mux_channel++;
        switch (curent_mux_channel) {
            case  1: palWriteLine(MULTIPLEXER_S3,  1); break; // NEXT CHANNEL:  8
            case  2: palWriteLine(MULTIPLEXER_S2,  1); break; // NEXT CHANNEL: 12
            case  3: palWriteLine(MULTIPLEXER_S0,  1); break; // NEXT CHANNEL: 13
            case  4: palWriteLine(MULTIPLEXER_S2,  0); break; // NEXT CHANNEL:  9
            case  5: palWriteLine(MULTIPLEXER_S1,  1); break; // NEXT CHANNEL: 11
            case  6: palWriteLine(MULTIPLEXER_S2,  1); break; // NEXT CHANNEL: 15
            case  7: palWriteLine(MULTIPLEXER_S3,  0); break; // NEXT CHANNEL:  7
            case  8: palWriteLine(MULTIPLEXER_S1,  0); break; // NEXT CHANNEL:  5
            case  9: palWriteLine(MULTIPLEXER_S0,  0); break; // NEXT CHANNEL:  4
            case 10: palWriteLine(MULTIPLEXER_S1,  1); break; // NEXT CHANNEL:  6
            case 11: palWriteLine(MULTIPLEXER_S3,  1); break; // NEXT CHANNEL: 14
            case 12: palWriteLine(MULTIPLEXER_S2,  0); break; // NEXT CHANNEL: 10
            case 13: palWriteLine(MULTIPLEXER_S3,  0); break; // NEXT CHANNEL:  2
            case 14: palWriteLine(MULTIPLEXER_S0,  1); break; // NEXT CHANNEL:  3
            case 15: palWriteLine(MULTIPLEXER_S1,  0); break; // NEXT CHANNEL:  1
            case 16: palWriteLine(MULTIPLEXER_S0,  0); curent_mux_channel = 0; break; // NEXT CHANNEL:  0
        }


        adcStartConversion(&ADCD1, &stm32f103_conversion_group, &samples[ADC_BUF_DEPTH * ADC_CH_NUM * curent_mux_channel], ADC_BUF_DEPTH);
    }
}

static GPTConfig gptcfg1 = {
  256000,                     // frequency
  gptcallback1,               // callback
  0,
  0
};*/


void matrix_init(void)
{   
    #ifdef DEBUG_MATRIX_SCAN_RATE
      matrix_timer = timer_read32();
      matrix_scan_count = 0;
    #endif

    #ifdef ADC_SCOPE_TRIG_OUT_PIN
      palSetLineMode(ADC_SCOPE_TRIG_OUT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
    #endif

    /* Multiplexer channel select pins */
    palSetLineMode(MULTIPLEXER_S0,   PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(MULTIPLEXER_S1,   PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(MULTIPLEXER_S2,   PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(MULTIPLEXER_S3,   PAL_MODE_OUTPUT_PUSHPULL);

    // clear the pins, start at channel 0 (not sure this is needed)
    palWriteLine(MULTIPLEXER_S0, 0);
    palWriteLine(MULTIPLEXER_S1, 0);
    palWriteLine(MULTIPLEXER_S2, 0);
    palWriteLine(MULTIPLEXER_S3, 0);

    /* Analog signal pins (one per each multiplexer) */
    palSetGroupMode(GPIOA, 0xFFU, 0, PAL_MODE_INPUT_ANALOG); //PA0 through PA7
  
    //osalSysLockFromISR();   
    adcStart(&ADCD1, NULL);
    //adcAcquireBus(&ADCD1);
    adcStartConversionI(&ADCD1, &stm32f103_conversion_group, (adcsample_t*) samples, ADC_BUF_DEPTH);
    //osalSysUnlockFromISR(); 

    //gptStart(&GPTD1, &gptcfg1);
    //gptStartContinuous(&GPTD1, 1);
    
    memset(matrix, 0, MATRIX_ROWS * sizeof(matrix_row_t));
    memset(matrix_debouncing, 0, MATRIX_ROWS * sizeof(matrix_row_t));

    matrix_init_quantum();
}

static uint16_t b_min = 4000, b_max = 0;

#define ADC_VAL_LOW 1130
#define ADC_VAL_HIGH 2350
#define ADC_VAL_HYST 0.2 // 20 percent of fullscale
#define ADC_VAL_CENTER_OFFSET +0.05
const static uint16_t thrs_trigger   = (ADC_VAL_HIGH + ADC_VAL_LOW)/2 - ((ADC_VAL_HIGH + ADC_VAL_LOW)/2)*(ADC_VAL_HYST / 2) + ((ADC_VAL_HIGH + ADC_VAL_LOW)/2)*ADC_VAL_CENTER_OFFSET;
const static uint16_t thrs_untrigger = (ADC_VAL_HIGH + ADC_VAL_LOW)/2 + ((ADC_VAL_HIGH + ADC_VAL_LOW)/2)*(ADC_VAL_HYST / 2) + ((ADC_VAL_HIGH + ADC_VAL_LOW)/2)*ADC_VAL_CENTER_OFFSET;

static bool stored_states[1] = {false};

static uint32_t time_pressed = 0;


uint8_t matrix_scan(void)
{
    #ifdef DEBUG_MATRIX_SCAN_RATE
      matrix_scan_count++;

      uint32_t timer_now = timer_read32();
      if (TIMER_DIFF_32(timer_now, matrix_timer)>1000) {
        //dprintf("matrix scan frequency: %u \n", matrix_scan_count);
        /*if (debug_matrix) {
            matrix_print();
        }*/

        matrix_timer = timer_now;
        matrix_scan_count = 0;
      }
    #endif


    for (int row = 0; row < MATRIX_ROWS; row++) {
        matrix_row_t data = 0;

        //generate digital data from analog readings

        if (b_min > samples[0])
            b_min = samples[0];

        if (b_max < samples[0])
            b_max = samples[0];

        uint16_t avg = (samples[0] + samples[0 + ADC_CH_NUM] + samples[0 + ADC_CH_NUM*2]) / 3;


        if (matrix_scan_count == 1) {
            printf("number: %u avg: %u min: %u max: %u pp: %u \n", samples[0], avg, b_min, b_max, b_max - b_min);
            b_min = 65535, b_max = 0;
            /*
            adcstate_t state = ADCD1.state;
            switch(state) {
                case ADC_UNINIT:   printf("ADC_UNINIT %u", ADC_UNINIT);   break;
                case ADC_STOP:     printf("ADC_STOP %u", ADC_STOP);     break;
                case ADC_READY:    printf("ADC_READY %u", ADC_READY);    break;
                case ADC_ACTIVE:   printf("ADC_ACTIVE %u", ADC_ACTIVE);   break;
                case ADC_COMPLETE: printf("ADC_COMPLETE %u", ADC_COMPLETE); break;
                case ADC_ERROR:    printf("ADC_ERROR %u", ADC_ERROR);    break;
            }*/

        }

        if (stored_states[0] == false && samples[0] < thrs_trigger) {
            stored_states[0] = true;
            data = 1;
            time_pressed = timer_read32();
        }
        else if (stored_states[0] == true && samples[0] > thrs_untrigger) {
            stored_states[0] = false;
            data = 0;
            printf("time presses: %u ms", TIMER_DIFF_32(timer_read32(), time_pressed));
        }
        else {
            if (stored_states[0])
                data = 1;
            else
                data = 0;
        }


        //uint32_t c_port = palReadPort(GPIOC);
        //data =   (palReadPort(GPIOB) & 0xFUL)         |          
        //        ((palReadPort(GPIOD) & 0x7EUL) <<  3) |
        //        ((c_port             & 0x1FUL) << 10) |
        //        ((c_port             & 0xC0UL) <<  9);

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
