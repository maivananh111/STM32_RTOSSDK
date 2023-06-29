/*
 * periph_config.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPHERALS_PERIPH_CONFIG_H_
#define PERIPHERALS_PERIPH_CONFIG_H_

#include "sdkconfig.h"
#include st_header

#define ENABLE_EXTI         1
#define ENABLE_IWDG			1
#define ENABLE_ADC          1
#define ENABLE_TIM			1
#define ENABLE_DMA			1
#define ENABLE_USART		1
#define ENABLE_SPI			1
#define ENABLE_I2C			1
#define ENABLE_FMC          1
#define ENABLE_RNG          1

#define ENABLE_ETH			1
#define ENABLE_USB          0


/**
 * RCC.
 */
#define RCC_HSI_TIMEOUT          100U
#define RCC_HSE_TIMEOUT          200U
#define RCC_PLL_TIMEOUT          100U
#define RCC_SWS_TIMEOUT 	 	 5000U

/**
 * GPIO.
 */
#define GPIO_OUTPUTSPEED_DEFAULT GPIO_Speed_VeryHigh



/**
 *  EXTI ENABLE
 */
#if ENABLE_EXTI
#define EXTI_LOG            1
#endif /* ENABLE_EXTI */

/**
 *  ADC ENABLE
 */
#if ENABLE_ADC
#define ADC_LOG             1
#endif /* ENABLE_ADC */

/**
 * DMA STREAM ENABLE.
 */
#if ENABLE_DMA
#define DMA_LOG             0
#endif /* ENABLE_DMA */

/**
 * TIM ENABLE
 */
#if ENABLE_TIM
#define TIM_LOG             1
#define USE_TIM1            1
#define USE_TIM8            1
#endif /* ENABLE_TIM */

/**
 * USART ENABLE
 */
#if ENABLE_USART
#define USART_LOG           1
#define USART_TIMEOUT 100U
#endif /* ENABLE_USART */

/**
 * SPI ENABLE
 */
#if ENABLE_SPI
#define SPI_LOG             1
#define SPI_TIMEOUT 100U
#endif /* ENABLE_SPI */

/**
 * SPI ENABLE
 */
#if ENABLE_I2C
#define I2C_LOG             1
#define I2C_BUSY_TIMEOUT    1000U
#define I2C_TIMEOUT         1000U
#endif /* ENABLE_SPI */



/**
 * FMC_SDRAM.
 */
#if ENABLE_FMC
/** FMC GPIO Configuration
PF0   ------> FMC_A0
PF1   ------> FMC_A1
PF2   ------> FMC_A2
PF3   ------> FMC_A3
PF4   ------> FMC_A4
PF5   ------> FMC_A5
PC0   ------> FMC_SDNWE
PC2   ------> FMC_SDNE0
PC3   ------> FMC_SDCKE0
PF11   ------> FMC_SDNRAS
PF12   ------> FMC_A6
PF13   ------> FMC_A7
PF14   ------> FMC_A8
PF15   ------> FMC_A9
PG0   ------> FMC_A10
PG1   ------> FMC_A11
PE7   ------> FMC_D4
PE8   ------> FMC_D5
PE9   ------> FMC_D6
PE10   ------> FMC_D7
PE11   ------> FMC_D8
PE12   ------> FMC_D9
PE13   ------> FMC_D10
PE14   ------> FMC_D11
PE15   ------> FMC_D12
PD8   ------> FMC_D13
PD9   ------> FMC_D14
PD10   ------> FMC_D15
PD14   ------> FMC_D0
PD15   ------> FMC_D1
PG4   ------> FMC_BA0
PG5   ------> FMC_BA1
PG8   ------> FMC_SDCLK
PD0   ------> FMC_D2
PD1   ------> FMC_D3
PG15   ------> FMC_SDNCAS
PE0   ------> FMC_NBL0
PE1   ------> FMC_NBL1
*/
#define SDRAM_DEVICE FMC_Bank5_6

#define FMC_A0    0
#define FMC_A0_P  GPIOF
#define FMC_A1    1
#define FMC_A1_P  GPIOF
#define FMC_A2    2
#define FMC_A2_P  GPIOF
#define FMC_A3    3
#define FMC_A3_P  GPIOF
#define FMC_A4    4
#define FMC_A4_P  GPIOF
#define FMC_A5    5
#define FMC_A5_P  GPIOF
#define FMC_A6    12
#define FMC_A6_P  GPIOF
#define FMC_A7    13
#define FMC_A7_P  GPIOF
#define FMC_A8    14
#define FMC_A8_P  GPIOF
#define FMC_A9    15
#define FMC_A9_P  GPIOF
#define FMC_A10   0
#define FMC_A10_P GPIOG
#define FMC_A11   1
#define FMC_A11_P GPIOG

#define FMC_D0    14
#define FMC_D0_P  GPIOD
#define FMC_D1    15
#define FMC_D1_P  GPIOD
#define FMC_D2    0
#define FMC_D2_P  GPIOD
#define FMC_D3    1
#define FMC_D3_P  GPIOD
#define FMC_D4    7
#define FMC_D4_P  GPIOE
#define FMC_D5    8
#define FMC_D5_P  GPIOE
#define FMC_D6    9
#define FMC_D6_P  GPIOE
#define FMC_D7    10
#define FMC_D7_P  GPIOE
#define FMC_D8    11
#define FMC_D8_P  GPIOE
#define FMC_D9    12
#define FMC_D9_P  GPIOE
#define FMC_D10   13
#define FMC_D10_P GPIOE
#define FMC_D11   14
#define FMC_D11_P GPIOE
#define FMC_D12   15
#define FMC_D12_P GPIOE
#define FMC_D13   8
#define FMC_D13_P GPIOD
#define FMC_D14   9
#define FMC_D14_P GPIOD
#define FMC_D15   10
#define FMC_D15_P GPIOD

#define FMC_NBL0    0
#define FMC_NBL0_P  GPIOE
#define FMC_NBL1    1
#define FMC_NBL1_P  GPIOE
#define FMC_CAS     15
#define FMC_CAS_P   GPIOG
#define FMC_RAS     11
#define FMC_RAS_P   GPIOF
#define FMC_NWE     0
#define FMC_NWE_P   GPIOC
#define FMC_NE      2
#define FMC_NE_P    GPIOC
#define FMC_CKE     3
#define FMC_CKE_P   GPIOC
#define FMC_CLK     8
#define FMC_CLK_P   GPIOG
#define FMC_BA0     4
#define FMC_BA0_P   GPIOG
#define FMC_BA1     5
#define FMC_BA1_P   GPIOG


#define FMC_SDRAM_TIMEOUT 5000U

#endif /* ENABLE_FMC */

/**
 * RNG.
 */
#if ENABLE_RNG
#define RNG_TIMEOUT 2U
#endif /* ENABLE_RNG */


#endif /* PERIPHERALS_PERIPH_CONFIG_H_ */
