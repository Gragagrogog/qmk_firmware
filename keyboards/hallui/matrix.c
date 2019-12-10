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
//#define ADC_BUF_DEPTH 1 // depth of buffer
#define ADC_GRP1_NUM_CHANNELS 4    // number of used ADC0 channels 
#define ADC_GRP2_NUM_CHANNELS 4    // number of used ADC1 channels, must be less or equal to ADC_GRP1_NUM_CHANNELS
#define MULTIPLEXER_NUM_CHANNELS 16
//static uint8_t curent_mux_channel = 0;
static volatile adcsample_t samples[(ADC_GRP1_NUM_CHANNELS + ADC_GRP2_NUM_CHANNELS) * MULTIPLEXER_NUM_CHANNELS];
static volatile uint8_t matrix_scan_complete_flag = 0;
static const uint8_t scan_compl = 1;
static const uint8_t link_dma_channel = KINETIS_HALLUI_ADC1S_DMA_CHANNEL;
static uint8_t adc_grp1_channels[ADC_GRP1_NUM_CHANNELS + 1] = {5, 14, 13, 12, 0x1F}; // ADCx_SC1n_AIEN 0x1F
static uint8_t adc_grp2_channels[ADC_GRP1_NUM_CHANNELS + 1] = {8,  9,  4,  5, 0x1F}; 

#define MULTIPLEXER_S0 LINE_PIN2
#define MULTIPLEXER_S1 LINE_PIN3
#define MULTIPLEXER_S2 LINE_PIN4
#define MULTIPLEXER_S3 LINE_PIN5

#define ADC_SCOPE_TRIG_OUT_PIN LINE_PIN6 //Oscilloscope trigger in

static uint32_t mux_pad[4] = {
    1 << PAL_PAD(MULTIPLEXER_S0),
    1 << PAL_PAD(MULTIPLEXER_S1),
    1 << PAL_PAD(MULTIPLEXER_S2),
    1 << PAL_PAD(MULTIPLEXER_S3),
};

#ifdef ADC_SCOPE_TRIG_OUT_PIN
static uint32_t trig_pad = 1 << PAL_PAD(ADC_SCOPE_TRIG_OUT_PIN);
#define TRIG_BANK_S (uint32_t)&(PAL_PORT(MULTIPLEXER_S3))->PSOR
#define TRIG_BANK_C (uint32_t)&(PAL_PORT(MULTIPLEXER_S3))->PCOR
#endif

#define SGC_CSR              ((uint32_t)(DMA_CSR_ESG_MASK | DMA_CSR_MAJORELINK_MASK | DMA_CSR_MAJORLINKCH(KINETIS_HALLUI_LINK_DMA_CHANNEL)))
#define SGC_CSR_AUTOS        ((uint32_t)(DMA_CSR_ESG_MASK | DMA_CSR_MAJORELINK_MASK | DMA_CSR_MAJORLINKCH(KINETIS_HALLUI_LINK_DMA_CHANNEL) | DMA_CSR_START_MASK))
#define SGC_CSR_NOLINK       ((uint32_t)(DMA_CSR_ESG_MASK))
#define SGC_CSR_NOLINK_AUTOS ((uint32_t)(DMA_CSR_ESG_MASK | DMA_CSR_START_MASK))
#define SGC_ATTR4 ((uint32_t)(DMA_ATTR_SSIZE(2) | DMA_ATTR_DSIZE(2))) // 4 bytes
#define SGC_ATTR1 ((uint32_t)(DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0))) // 1 byte
#define MUX_BANK_S0 (uint32_t)&(PAL_PORT(MULTIPLEXER_S0))->PSOR
#define MUX_BANK_S1 (uint32_t)&(PAL_PORT(MULTIPLEXER_S1))->PSOR
#define MUX_BANK_S2 (uint32_t)&(PAL_PORT(MULTIPLEXER_S2))->PSOR
#define MUX_BANK_S3 (uint32_t)&(PAL_PORT(MULTIPLEXER_S3))->PSOR
#define MUX_BANK_C0 (uint32_t)&(PAL_PORT(MULTIPLEXER_S0))->PCOR
#define MUX_BANK_C1 (uint32_t)&(PAL_PORT(MULTIPLEXER_S1))->PCOR
#define MUX_BANK_C2 (uint32_t)&(PAL_PORT(MULTIPLEXER_S2))->PCOR
#define MUX_BANK_C3 (uint32_t)&(PAL_PORT(MULTIPLEXER_S3))->PCOR
#define MATRIX_FLAG (uint32_t)&matrix_scan_complete_flag

DMA_TCD_TypeDef mux_scatter_dma_src[] __attribute__ ((aligned (32))) = {
    {.SADDR=(uint32_t)&mux_pad[3], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_S3, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 1], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[2], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_S2, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 2], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[0], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_S0, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 3], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[2], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C2, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 4], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[1], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_S1, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 5], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[2], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_S2, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 6], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[3], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C3, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 7], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[1], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C1, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 8], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[0], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C0, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 9], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[1], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_S1, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[10], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[3], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_S3, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[11], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[2], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C2, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[12], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[3], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C3, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[13], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[0], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_S0, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[14], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
#ifndef ADC_SCOPE_TRIG_OUT_PIN
    {.SADDR=(uint32_t)&mux_pad[1], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C1, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[15], .CSR=SGC_CSR, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[0], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C0, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[16], .CSR=SGC_CSR_NOLINK, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&scan_compl, .SOFF=0, .ATTR=SGC_ATTR1, .NBYTES_MLNO=1, .SLAST=0, .DADDR=MATRIX_FLAG, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 0], .CSR=SGC_CSR_AUTOS,  .BITER_ELINKNO=1} 
#else
    {.SADDR=(uint32_t)&trig_pad,   .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=TRIG_BANK_S, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[15], .CSR=SGC_CSR_NOLINK, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[1], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C1, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[16], .CSR=SGC_CSR_AUTOS,  .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&trig_pad,   .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=TRIG_BANK_C, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[17], .CSR=SGC_CSR_NOLINK,  .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&mux_pad[0], .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=MUX_BANK_C0, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[18], .CSR=SGC_CSR_NOLINK_AUTOS, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&scan_compl, .SOFF=0, .ATTR=SGC_ATTR1, .NBYTES_MLNO=1, .SLAST=0, .DADDR=MATRIX_FLAG, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&mux_scatter_dma_src[ 0], .CSR=SGC_CSR_AUTOS,  .BITER_ELINKNO=1} 
#endif
};

static uint32_t pdb_sc_val = 0;

DMA_TCD_TypeDef adc_delayed_restart[] __attribute__ ((aligned (32))) = {
    {.SADDR=(uint32_t)&pdb_sc_val,       .SOFF=0, .ATTR=SGC_ATTR4, .NBYTES_MLNO=4, .SLAST=0, .DADDR=(uint32_t)&PDB->SC,   .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&adc_delayed_restart[1], .CSR=SGC_CSR_NOLINK, .BITER_ELINKNO=1}, 
    {.SADDR=(uint32_t)&link_dma_channel, .SOFF=0, .ATTR=SGC_ATTR1, .NBYTES_MLNO=1, .SLAST=0, .DADDR=(uint32_t)&DMA->SSRT, .DOFF=0, .CITER_ELINKNO=1, .DLASTSGA=(uint32_t)&adc_delayed_restart[0], .BITER_ELINKNO=1, 
        .CSR = DMA_CSR_ESG_MASK | DMA_CSR_MAJORELINK_MASK | DMA_CSR_MAJORLINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL)}
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

void adc_config(void) {
    adcInit();
    adcStart(&ADCD1, &adccfg1); //adc calibration (blocking)
    adcStart(&ADCD2, &adccfg1); //adc calibration (blocking)

    //CFG1 Regiser
    //0 bits: Normal power configuration, Short sample time, 
    uint32_t CFG1 = ADCx_CFG1_ADIV(ADCx_CFG1_ADIV_DIV_2) |
                    ADCx_CFG1_ADICLK(ADCx_CFG1_ADIVCLK_BUS_CLOCK) |
                    ADCx_CFG1_MODE(ADCx_CFG1_MODE_12_OR_13_BITS);
    //CFG2 Regiser
    //0 bits: Asynchronous clock output disabled, Normal conversion sequence selected
    uint32_t CFG2 = ADCx_CFG2_MUXSEL; // | // xxB channel select
                    //ADCx_CFG2_ADHSC; // AD high speed clock bit, conversion takes extra 2 ADCLK

    uint32_t SC3  = ADCx_SC3_AVGE | ADCx_SC3_AVGS(ADCx_SC3_AVGS_AVERAGE_4_SAMPLES);

    uint32_t SC2  = ADCx_SC2_DMA_ENABLE; // | ADCx_SC2_TRIGGER_HARDWARE;

    uint32_t SC1A = 0x1F; // Clear COCO bit from calibration

    ADCD1.adc->CFG1 = CFG1;
    ADCD1.adc->CFG2 = CFG2;
    ADCD1.adc->SC3  = SC3;
    ADCD1.adc->SC2  = SC2;
    ADCD1.adc->SC1A = SC1A;

    ADCD2.adc->CFG1 = CFG1;
    ADCD2.adc->CFG2 = CFG2;
    ADCD2.adc->SC3  = SC3;
    ADCD2.adc->SC2  = SC2;
    ADCD2.adc->SC1A = SC1A;
}

void setup_adc_dma_scanning(void)
{
    adc_config();

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
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].DADDR = (uint32_t)&samples[0];// where to write to
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].DOFF = 2; // destination increment
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].DLASTSGA = -2 * ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].BITER_ELINKYES = DMA_BITER_ELINKYES_ELINK_MASK | DMA_BITER_ELINKYES_LINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL) | ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].CITER_ELINKYES = DMA_CITER_ELINKYES_ELINK_MASK | DMA_CITER_ELINKYES_LINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL) | ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;    
    DMA->TCD[KINETIS_HALLUI_ADC0R_DMA_CHANNEL].CSR = DMA_CSR_MAJORELINK_MASK | DMA_CSR_MAJORLINKCH(KINETIS_HALLUI_ADC0S_DMA_CHANNEL);
    DMAMUX->CHCFG[KINETIS_HALLUI_ADC0R_DMA_CHANNEL] = DMAMUX_CHCFGn_ENBL | DMAMUX_CHCFGn_SOURCE(40); //ADC0 source = 40
    DMA->SERQ = KINETIS_HALLUI_ADC0R_DMA_CHANNEL; //enable hardware request, this is not needed for DMA to DMA linked channels

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

    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].SADDR = (uint32_t)&ADCD2.adc->RA;
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].SOFF = 0; // source increment each transfer
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(1) | DMA_ATTR_DSIZE(1); // byte sizes: 1B=0, 2B=1, 4B=2, 16B=4
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].NBYTES_MLNO = 2;     // bytes per transfer DMA_NBYTES_MLOFFYES_DMLOE_MASK |
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].SLAST = 0;
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].DADDR = (uint32_t)&samples[ADC_GRP1_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS];// where to write to
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].DOFF = 2; // destination increment
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].DLASTSGA = -2 * ADC_GRP2_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].BITER_ELINKYES = DMA_BITER_ELINKYES_ELINK_MASK | DMA_BITER_ELINKYES_LINKCH(KINETIS_HALLUI_ADC1S_DMA_CHANNEL) | ADC_GRP2_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].CITER_ELINKYES = DMA_CITER_ELINKYES_ELINK_MASK | DMA_CITER_ELINKYES_LINKCH(KINETIS_HALLUI_ADC1S_DMA_CHANNEL) | ADC_GRP2_NUM_CHANNELS * MULTIPLEXER_NUM_CHANNELS;    
    DMA->TCD[KINETIS_HALLUI_ADC1R_DMA_CHANNEL].CSR = DMA_CSR_MAJORELINK_MASK | DMA_CSR_MAJORLINKCH(KINETIS_HALLUI_ADC1S_DMA_CHANNEL);
    DMAMUX->CHCFG[KINETIS_HALLUI_ADC1R_DMA_CHANNEL] = DMAMUX_CHCFGn_ENBL | DMAMUX_CHCFGn_SOURCE(41); //ADC1 source = 41
    DMA->SERQ = KINETIS_HALLUI_ADC1R_DMA_CHANNEL; //enable hardware request, this is not needed for DMA to DMA linked channels

    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].SADDR = (uint32_t)&adc_grp2_channels[0];
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].SOFF = 1; // source increment each transfer
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].ATTR = DMA_ATTR_SSIZE(0) | DMA_ATTR_DSIZE(0); // byte sizes: 1B=0, 2B=1, 4B=2, 16B=4
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].NBYTES_MLNO = 1;     // bytes per transfer
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].SLAST = -(ADC_GRP2_NUM_CHANNELS + 1);
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].DADDR = (uint32_t)&ADCD2.adc->SC1A;// where to write to
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].DOFF = 0; // destination increment
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].DLASTSGA = 0;
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].BITER_ELINKNO = ADC_GRP2_NUM_CHANNELS + 1;
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].CITER_ELINKNO = ADC_GRP2_NUM_CHANNELS + 1;    
    DMA->TCD[KINETIS_HALLUI_ADC1S_DMA_CHANNEL].CSR = 0;

    DMA->TCD[KINETIS_HALLUI_GPIO_DMA_CHANNEL] = mux_scatter_dma_src[0];

    SIM->SCGC6 |= SIM_SCGC6_PDB;
    pdb_sc_val = PDB_SC_SWTRIG | PDB_SC_TRGSEL(PDB_SC_TRGSEL_SW) | PDB_SC_DMAEN | 
                 PDB_SC_MULT(PDB_SC_MULT_FACTOR1) | PDB_SC_PRESCALER(PDB_SC_PRESCALER_DIV1) | //BUS CLOCK = 48MHz ... 20.83nS period
                 PDB_SC_PDBEN;
    PDB->SC = pdb_sc_val;
    pdb_sc_val |= PDB_SC_LDOK;
    uint16_t delay = 50; //1uS delay (plus extra ~1.2uS triggering DMA->PDB->DMA) 
    PDB->IDLY = PDB_IDLY(delay);
    PDB->MOD = PDB_MOD(delay);

    DMA->TCD[KINETIS_HALLUI_LINK_DMA_CHANNEL] = adc_delayed_restart[0];
    DMAMUX->CHCFG[KINETIS_HALLUI_LINK_DMA_CHANNEL] = DMAMUX_CHCFGn_ENBL | DMAMUX_CHCFGn_SOURCE(48); //PDB source = 48
    DMA->SERQ = KINETIS_HALLUI_LINK_DMA_CHANNEL; //enable hardware request, this is not needed for DMA to DMA linked channels

    //explicit SW start of the adc/dma loop
    DMA->SSRT = KINETIS_HALLUI_LINK_DMA_CHANNEL;

}


void matrix_init(void)
{   
    #ifdef DEBUG_MATRIX_SCAN_RATE
      matrix_timer = timer_read32();
      matrix_scan_count = 0;
    #endif

    /* Multiplexer channel select pins */
    palSetLineMode(MULTIPLEXER_S0, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(MULTIPLEXER_S1, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(MULTIPLEXER_S2, PAL_MODE_OUTPUT_PUSHPULL);
    palSetLineMode(MULTIPLEXER_S3, PAL_MODE_OUTPUT_PUSHPULL);

    #ifdef ADC_SCOPE_TRIG_OUT_PIN
    palSetLineMode(ADC_SCOPE_TRIG_OUT_PIN, PAL_MODE_OUTPUT_PUSHPULL);
    #endif

    // clear the pins, start at channel 0 (not sure this is needed)
    palWriteLine(MULTIPLEXER_S0, 0);
    palWriteLine(MULTIPLEXER_S1, 0);
    palWriteLine(MULTIPLEXER_S2, 0);
    palWriteLine(MULTIPLEXER_S3, 0);

    /* Analog signal pins (one per each multiplexer) */
    palSetLineMode(LINE_PIN14, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN15, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN18, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN19, PAL_MODE_INPUT_ANALOG);

    palSetLineMode(LINE_PIN16, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN17, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN27, PAL_MODE_INPUT_ANALOG);
    palSetLineMode(LINE_PIN28, PAL_MODE_INPUT_ANALOG);
    
    setup_adc_dma_scanning(); //CPU-free background analog matrix scanning using DMA+PDB
    
    memset(matrix, 0, MATRIX_ROWS * sizeof(matrix_row_t));

    matrix_init_quantum();
}

#define ADC_VAL_LOW 1900
#define ADC_VAL_HIGH 3980
#define ADC_VAL_HYST 0.2 // 20 percent of fullscale
#define ADC_VAL_CENTER_OFFSET +0.05
const static uint16_t thrs_trigger   = (ADC_VAL_HIGH + ADC_VAL_LOW)/2 - ((ADC_VAL_HIGH + ADC_VAL_LOW)/2)*(ADC_VAL_HYST / 2) + ((ADC_VAL_HIGH + ADC_VAL_LOW)/2)*ADC_VAL_CENTER_OFFSET;
const static uint16_t thrs_untrigger = (ADC_VAL_HIGH + ADC_VAL_LOW)/2 + ((ADC_VAL_HIGH + ADC_VAL_LOW)/2)*(ADC_VAL_HYST / 2) + ((ADC_VAL_HIGH + ADC_VAL_LOW)/2)*ADC_VAL_CENTER_OFFSET;

static adcsample_t min0 = 0, max0 = 0;
static adcsample_t min1 = 0, max1 = 0;

static uint32_t adc_scan_count = 0;

uint8_t matrix_scan(void)
{
    #ifdef DEBUG_MATRIX_SCAN_RATE
      matrix_scan_count++;

      uint32_t timer_now = timer_read32();
      if (TIMER_DIFF_32(timer_now, matrix_timer)>1000) {
        matrix_timer = timer_now;
        matrix_scan_count = 0;
      }
    #endif

    uint8_t changed = 0;
    
    if (matrix_scan_count == 0) {
        //xprintf("dma error reg: %u \n", DMA->ES);
        //xprintf("DLASTSGA: %u \n", ((uint32_t)&mux_scatter_dma_src[ 1])%32);
    
        /*
        xprintf("trig: %u // untrig: %u \n", thrs_trigger, thrs_untrigger);
        for (int x = 0; x < MATRIX_ROWS; x++) {
            xprintf("row: %u - ", x);
            for (int y = 0; y < MATRIX_COLS; y++) {
                adcsample_t sample = samples[y + x*MATRIX_COLS];
                xprintf("%u ", sample);
            }
            xprintf("\n");
        }*/
    
        xprintf("min0: %u max0: %u   diff: %u \n", min0, max0, max0 - min0);
        xprintf("min1: %u max1: %u   diff: %u \n", min1, max1, max1 - min1);
        min0 = 65535, max0 = 0;
        min1 = 65535, max1 = 0;
        //matrix_print();
        xprintf("adc scan count: %u \n", adc_scan_count);
        adc_scan_count = 0;

    }


    if (matrix_scan_complete_flag){
        adc_scan_count++;

        adcsample_t test = samples[14*4];
        if (test > max0)
            max0 = test;
        if (test < min0)
            min0 = test;

        test = samples[14*4 + 1];
        if (test > max1)
            max1 = test;
        if (test < min1)
            min1 = test;
    
        for (int x = 0; x < MATRIX_ROWS; x++) {
            matrix_row_t data = 0;
            matrix_row_t data_prev = matrix[x];
            matrix_row_t data_uncertain = 0;
    
            for (int y = 0; y < MATRIX_COLS; y++) { 
                adcsample_t sample = samples[y + x*MATRIX_COLS];
                if (sample < thrs_trigger){
                    data |= 1 << y;
                } else if (sample < thrs_untrigger) {
                    data_uncertain |= 1 << y;
                }
            }
    
            data |= data_prev & data_uncertain;
            matrix[x] = data;
    
            if (data != data_prev)
                changed = 1;
        }
        matrix_scan_complete_flag = 0;
    }

    matrix_scan_quantum();
    return changed;
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
    xprintf("\nr/c 0123456789ABCDEF\n");
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
}
