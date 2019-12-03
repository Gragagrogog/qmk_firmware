/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef _MCUCONF_H_
#define _MCUCONF_H_

#define K20x_MCUCONF

/*
 * HAL driver system settings.
 */
/* PEE mode - 48MHz system clock driven by (16 MHz) external crystal. */
#define KINETIS_MCG_MODE            KINETIS_MCG_MODE_PEE
#define KINETIS_PLLCLK_FREQUENCY     96000000UL //96
//#define KINETIS_SYSCLK_FREQUENCY     96000000UL //48


/*
#define KINETIS_PLLCLK_FREQUENCY    72000000UL
#define KINETIS_SYSCLK_FREQUENCY    72000000UL
#define KINETIS_BUSCLK_FREQUENCY    36000000UL
#define KINETIS_FLASHCLK_FREQUENCY  24000000UL*/

#define KINETIS_ADC_USE_ADC0                  TRUE
#define KINETIS_ADC_USE_ADC1                  TRUE

#define KINETIS_HALLUI_ADC0R_DMA_CHANNEL           15
#define KINETIS_HALLUI_ADC0S_DMA_CHANNEL           14

#define KINETIS_HALLUI_GPIO_DMA_CHANNEL           5

/*
 * GPT driver system settings.
 */
//#define KINETIS_GPT_USE_PIT0                  TRUE

/*
 * SERIAL driver system settings.
 */
#define KINETIS_SERIAL_USE_UART0              FALSE

/*
 * USB driver settings
 */
#define KINETIS_USB_USE_USB0                  TRUE

/* Need to redefine this, since the default (configured for K20x) might not apply
 *   2 for Teensy LC
 *   5 for Teensy 3.x */
#define KINETIS_USB_USB0_IRQ_PRIORITY         5

#endif /* _MCUCONF_H_ */
