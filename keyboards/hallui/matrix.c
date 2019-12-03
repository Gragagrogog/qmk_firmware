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
#define ADC_BUF_DEPTH 1 // depth of buffer
#define ADC_GRP1_NUM_CHANNELS 4    // number of used ADC channels
//#define ADC_GRP2_NUM_CHANNELS 4    // number of used ADC channels
#define MULTIPLEXER_NUM_CHANNELS 16
//static uint8_t curent_mux_channel = 0;
static volatile adcsample_t samples1[ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS];
//static adcsample_t samples[ADC_BUF_DEPTH * ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS]; // results array
static uint8_t adc_grp1_channels[ADC_GRP1_NUM_CHANNELS + 1] = {5, 14, 13, 12, 0x1F}; // ADCx_SC1n_AIEN 0x1F

#define MULTIPLEXER_S0 LINE_PIN5
#define MULTIPLEXER_S1 LINE_PIN6
#define MULTIPLEXER_S2 LINE_PIN7
#define MULTIPLEXER_S3 LINE_PIN8

//the pins must be in the same GPIO bank
static const uint8_t mux_bit_flips[MULTIPLEXER_NUM_CHANNELS] = {
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S3),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S2),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S0),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S2),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S1),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S2),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S3),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S1),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S0),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S1),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S3),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S2),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S3),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S0),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S1),
    (uint8_t) 1 << PAL_PAD(MULTIPLEXER_S0)
};

static const ADCConfig adccfg1 = {
  /* Perform initial calibration */
  true
};

/*
static void gptcallback1(GPTDriver *gptp) {
    (void) gptp;
    if (ADCD1.state == ADC_READY) {
        //adcStart(&ADCD1, &adccfg2);
        adcStartConversion(&ADCD1, &teensy32_cg1, (adcsample_t*) samples, ADC_BUF_DEPTH); //&samples[ADC_BUF_DEPTH * ADC_GRP1_NUM_CHANNELS * curent_mux_channel]
        //adcConvert(&ADCD1, &teensy32_cg1, &samples[ADC_BUF_DEPTH * ADC_GRP1_NUM_CHANNELS * curent_mux_channel], ADC_BUF_DEPTH);
    }
}

static GPTConfig gptcfg1 = {
  10,                     // frequency
  gptcallback1,               // callback
};*/

void analog_matrix_setup(void)
{
    //CFG1 Regiser
    //0 bits: Normal power configuration, Short sample time, 
    ADCD1.adc->CFG1 = ADCx_CFG1_ADIV(ADCx_CFG1_ADIV_DIV_1) |
                      ADCx_CFG1_ADICLK(ADCx_CFG1_ADIVCLK_BUS_CLOCK) |
                      ADCx_CFG1_MODE(ADCx_CFG1_MODE_12_OR_13_BITS);
    //CFG2 Regiser
    //0 bits: Asynchronous clock output disabled, Normal conversion sequence selected
    ADCD1.adc->CFG2 = ADCx_CFG2_MUXSEL_ADxxB;

    /* Set averaging */
    ADCD1.adc->SC3 = ADCx_SC3_AVGE | ADCx_SC3_AVGS(ADCx_SC3_AVGS_AVERAGE_4_SAMPLES);

    ADCD1.adc->SC2 = ADCx_SC2_DMA_ENABLE; // | ADCx_SC2_TRIGGER_HARDWARE;

    ADCD1.adc->SC1A = 0x1F; // RESET COCO bit from calibration

    //DMA Clock Gate
    SIM->SCGC6 |= SIM_SCGC6_DMAMUX;
    SIM->SCGC7 |= SIM_SCGC7_DMA;

    //DMA->CR = 0;
    //DMA->ES = 0;
    //DMA->ERQ = 0;
    //DMA->EEI = 0;
    //DMA->INT = 0;
    //DMA->HRS = 0;

    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].SADDR = (uint32_t)&ADCD1.adc->RA;
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].SOFF = 0; // source increment each transfer
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1); // byte sizes: 1B=0, 2B=1, 4B=2, 16B=4
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].NBYTES_MLNO = 2;     // bytes per transfer DMA_NBYTES_MLOFFYES_DMLOE_MASK |
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].SLAST = 0;
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].DADDR = (uint32_t)&samples1;// where to write to
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].DOFF = 2; // destination increment
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].DLASTSGA = -2 * ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].BITER_ELINKYES = DMA_BITER_ELINKYES_ELINK_MASK | DMA_BITER_ELINKYES_LINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL) | ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].CITER_ELINKYES = DMA_CITER_ELINKYES_ELINK_MASK | DMA_CITER_ELINKYES_LINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL) | ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;    
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].CSR = DMA_CSR_MAJORELINK_MASK | DMA_CSR_MAJORLINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL);
    DMAMUX->CHCFG[KINETIS_HALLUI_ADC0R_DMA_CHANNEL] = DMAMUX_CHCFGn_ENBL | DMAMUX_CHCFGn_SOURCE(40); //ADC0 source = 40

    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].SADDR = (uint32_t)&adc_grp1_channels[0];
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].SOFF = 1; // source increment each transfer
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0); // byte sizes: 1B=0, 2B=1, 4B=2, 16B=4
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].NBYTES_MLNO = 1;     // bytes per transfer
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].SLAST = -(ADC_GRP1_NUM_CHANNELS + 1);
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].DADDR = (uint32_t)&ADCD1.adc->SC1A;// where to write to
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].DOFF = 0; // destination increment
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].DLASTSGA = 0;
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].BITER_ELINKNO = ADC_GRP1_NUM_CHANNELS + 1;
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].CITER_ELINKNO = ADC_GRP1_NUM_CHANNELS + 1;    
    DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].CSR = DMA_CSR_MAJORELINK_MASK | DMA_CSR_MAJORLINKCH(KINETIS_HALLUI_GPIO_DMA_CHANNEL);

    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].SADDR = (uint32_t)&mux_bit_flips[0];
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].SOFF = 1; // source increment each transfer
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0); // byte sizes: 1B=0, 2B=1, 4B=2, 16B=4
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].NBYTES_MLNO = 1;     // bytes per transfer
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].SLAST = -1 * MULTIPLEXER_NUM_CHANNELS;
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].DADDR = (uint32_t)&GPIOD->PTOR;// where to write to
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].DOFF = 0; // destination increment
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].DLASTSGA = 0;
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].BITER_ELINKYES = DMA_BITER_ELINKYES_ELINK_MASK | DMA_BITER_ELINKYES_LINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL) | MULTIPLEXER_NUM_CHANNELS;
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].CITER_ELINKYES = DMA_CITER_ELINKYES_ELINK_MASK | DMA_CITER_ELINKYES_LINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL) | MULTIPLEXER_NUM_CHANNELS;    
    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL].CSR = DMA_CSR_MAJORELINK_MASK | DMA_CSR_MAJORLINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL);

    //enable hardware request, this is not needed for DMA to DMA linked channels
    DMA->SERQ = KINETIS_HALLUI_ADC0R_DMA_CHANNEL; 
    //todo: DMA channel priority DMA_DCHPRIn ? 

    //DMA->TCD[1].CSR = DMA_CSR_START_MASK; //explicit SW start
    DMA->SSRT = KINETIS_HALLUI_ADC0S_DMA_CHANNEL;
    //ADCD1.adc->SC1A = 14; //adc_grp1_channels[0];

    //ADCD1.adc->SC1A = adc_grp1_channels[0];
    //adcAcquireBus(&ADCD1);
    //adcStartConversion(&ADCD1, &teensy32_cg1, (adcsample_t*) samples, ADC_BUF_DEPTH);
    //osalSysUnlockFromISR(); 

    //gptStart(&GPTD1, &gptcfg1);
    //gptStartContinuous(&GPTD1, 1);
}


void matrix_init(void)
{   
    #ifdef DEBUG_MATRIX_SCAN_RATE
      matrix_timer = timer_read32();
      matrix_scan_count = 0;
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
    palSetLineMode(LINE_PIN15, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN18, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN19, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN22, PAL_MODE_INPUT_ANALOG);
  
    adcInit();
    adcStart(&ADCD1, &adccfg1); //adc calibration (blocking)
    adcStart(&ADCD2, &adccfg1); //adc calibration (blocking)

    analog_matrix_setup(); //CPU-free background analog matrix scanning using DMA links
    
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

        if (b_min > samples1[0])
            b_min = samples1[0];

        if (b_max < samples1[0])
            b_max = samples1[0];

        //uint16_t avg = (samples1[0]); // + samples1[0 + ADC_GRP1_NUM_CHANNELS] + samples1[0 + ADC_CH_NUM*2]) / 3;


        if (matrix_scan_count == 1) {
            //printf("number: %u avg: %u min: %u max: %u pp: %u \n", samples1[0], avg, b_min, b_max, b_max - b_min);
            printf("BUFF: %u %u %u %u %u %u %u %u   %u \n", samples1[0], samples1[1], samples1[2], samples1[3], samples1[4], samples1[5], samples1[6], samples1[7], DMA->ES);
            //printf("%u %u\n", SIM->FCFG1, MCM->CR);
            //printf("%u \n", DMA->TCD[KINETIS_HALLUI_ADC0S_DMA_CHANNEL].BITER_ELINKNO);
            b_min = 65535, b_max = 0;

            //ADCD1.adc->SC1A = ADCx_SC1n_ADCH(ADC_AD15);
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

        if (stored_states[0] == false && samples1[0] < thrs_trigger) {
            stored_states[0] = true;
            data = 1;
            time_pressed = timer_read32();
        }
        else if (stored_states[0] == true && samples1[0] > thrs_untrigger) {
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
