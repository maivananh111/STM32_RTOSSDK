/*
 * sdkconfig.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef SDKCONFIG_H_
#define SDKCONFIG_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "FreeRTOSConfig.h"



#define SDK_VERSION                     "v1.1.0"

#if defined(STM32F0)
#define st_header "stm32f0xx.h"
#define st_hal_header "stm32f0xx_hal.h"
#elif defined(STM32F1)
#define st_header "stm32f1xx.h"
#define st_hal_header "stm32f1xx_hal.h"
#elif defined(STM32F2)
#define st_header "stm32f2xx.h"
#define st_hal_header "stm32f2xx_hal.h"
#elif defined(STM32F3)
#define st_header "stm32f3xx.h"
#define st_hal_header "stm32f3xx_hal.h"
#elif defined(STM32F4)
#define st_header "stm32f4xx.h"
#define st_hal_header "stm32f4xx_hal.h"
#endif
/**
 * RTOS Configuration.
 */
#define CONFIG_TOTAL_HEAP_SIZE                 	    (128U * 1024U)
#define CONFIG_RTOS_HEAP_SIZE                  	    configTOTAL_HEAP_SIZE
#define CONFIG_RTOS_APP_MAIN_TASK_SIZE              byte_to_word(4U*1024U)
#define CONFIG_RTOS_APP_MAIN_TASK_PRIO              1

#define CONFIG_RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY   4 // Your system interrupt priority must be greater than this number.
#define CONFIG_RTOS_TASK_MAX_PRIORITY               32
#define CONFIG_RTOS_TICK_RATE                       CONFIG_SYSTICK_RATE
#define CONFIG_RTOS_USE_IWDG                        1 // Can't debug if turn on iwdg.
#if CONFIG_RTOS_USE_IWDG
#define CONFIG_IWDG_PRESCALER                       4U
#define CONFIG_IWDG_AUTORELOAD                      2500U
/* CONFIG_IWDG_PRESCALER(ms) = (CONFIG_IWDG_AUTORELOAD * 4000U * 2^CONFIG_IWDG_PRESCALER) / LSI_VALUE */
#endif /* CONFIG_RTOS_USE_IWDG */
#define byte_to_word(x)                      		(x/4)

/**
 * Embedded flash memory configuration.
 */
#if defined(STM32F4)
#define CONFIG_FLASH_DATA_CACHE					 	1
#define CONFIG_FLASH_INSTRUCTION_CACHE         	 	1
#define CONFIG_FLASH_PREFETCH_MODE             	 	1
#endif /* STM32F4 */
#define CONFIG_USE_SDRAM                            1
/**
 * System tick configuration.
 */
#define CONFIG_SYSTICK_RATE                    	 	1000U // 1000 for 1ms.
#define CONFIG_SYSTICK_INTERRUPT_PRIORITY           15U

/**
 * Clock configuration.
 */
//#define HSE_VALUE 							 		25000000U // Hz
//#define HSI_VALUE 							 		16000000U // Hz
#define CONFIG_HSI_TRIM_VALUE                 	 	16U
#define CONFIG_OVER_CLOCK                       	1 // Note: if over clock, your device maybe not work correctly

#define LSI_VALUE                        	 		32000U
#define LSE_VALUE                        	 		32768U
// Constant value. Don't change this.
#if defined(STM32F1)
#define CONFIG_MAX_SYSTEM_CLOCK_FREQUENCY       	72000000U // Hz
#define CONFIG_MAX_AHB_CLOCK_FREQUENCY          	72000000U // Hz (AHB is HCLK)
#define CONFIG_MAX_APB1_CLOCK_FREQUENCY          	36000000U  // Hz (APB1 is PCLK1)
#define CONFIG_MAX_APB2_CLOCK_FREQUENCY         	72000000U  // Hz (APB1 is PCLK2)

#define CONFIG_SYSTEM_CLOCK_FREQUENCY          	 	72000000U  // Hz
#define CONFIG_AHB_CLOCK_FREQUENCY          		72000000U  // Hz

// Configuration value, you need to set up them.
#define CONFIG_OSC_CLOCK_SOURCE                	 	HSE_CRYSTAL
#define CONFIG_SYSTEM_CLOCK_MUX                	 	PLLCLK
#define CONFIG_PLL_SOURCE_MUX                  	 	PLL_SOURCE_HSE // if SYSTEM_CLOCK_MUX is not PLLCLK, ignore this value.
#define CONFIG_PLLMUL                               9U

#define CONFIG_AHB_PRESCALER          				Clock_Div_1 // Hz (AHB is HCLK)
#define CONFIG_APB1_PRESCALER          			 	Clock_Div_2 // Hz (APB1 is PCLK1)
#define CONFIG_APB2_PRESCALER          			 	Clock_Div_1 // Hz (APB1 is PCLK2)

#elif defined(STM32F4)

#if !CONFIG_OVER_CLOCK
#define CONFIG_MAX_SYSTEM_CLOCK_FREQUENCY       	168000000U // Hz
#define CONFIG_MAX_AHB_CLOCK_FREQUENCY          	168000000U // Hz (AHB is HCLK)
#define CONFIG_MAX_APB1_CLOCK_FREQUENCY          	42000000U  // Hz (APB1 is PCLK1)
#define CONFIG_MAX_APB2_CLOCK_FREQUENCY         	84000000U  // Hz (APB1 is PCLK2)
#endif /* !CONFIG_OVER_CLOCK */

// Configuration value, you need to set up them.
#define CONFIG_OSC_CLOCK_SOURCE                	 	HSE_CRYSTAL
#define CONFIG_SYSTEM_CLOCK_MUX                	 	PLLCLK
#define CONFIG_PLL_SOURCE_MUX                  	 	PLL_SOURCE_HSE // if SYSTEM_CLOCK_MUX is not PLLCLK, ignore this value.

#define CONFIG_RCC_PLLM                            	25U
#define CONFIG_RCC_PLLP                            	2U
#if !CONFIG_OVER_CLOCK
#define CONFIG_RCC_PLLN                            	336U
#define CONFIG_RCC_PLLQ                            	7U
#else
#define CONFIG_RCC_PLLN                            	432U
#define CONFIG_RCC_PLLQ                            	9U
#endif /* !CONFIG_OVER_CLOCK */

#if !CONFIG_OVER_CLOCK
#define CONFIG_SYSTEM_CLOCK_FREQUENCY          	 	168000000U  // Hz
#define CONFIG_AHB_CLOCK_FREQUENCY          		168000000U  // Hz
#else
#define CONFIG_SYSTEM_CLOCK_FREQUENCY          	 	216000000U  // Hz
#define CONFIG_AHB_CLOCK_FREQUENCY          		216000000U  // Hz
#endif /* !CONFIG_OVER_CLOCK */
#define CONFIG_AHB_PRESCALER          				Clock_Div_1 // Hz (AHB is HCLK)
#define CONFIG_APB1_PRESCALER          			 	Clock_Div_4 // Hz (APB1 is PCLK1)
#define CONFIG_APB2_PRESCALER          			 	Clock_Div_2 // Hz (APB1 is PCLK2)
#endif /* STM32F4 */

/**
 * Log monitor.
 */
#define CONFIG_USE_LOG_MONITOR						1
#if CONFIG_USE_LOG_MONITOR
#define CONFIG_LOG_LEVEL_SHORT                      1
#define CONFIG_LOG_OVER_UART						1
#define CONFIG_LOG_OVER_USB							0
#if CONFIG_LOG_OVER_UART
#define CONFIG_LOG_UART_BAUDRATE                    115200U
#define CONFIG_LOG_UART_NUM 						USART6

#define CONFIG_LOG_UART_TX 							14
#define CONFIG_LOG_UART_TXP 						GPIOG
#define CONFIG_LOG_UART_RX 							9
#define CONFIG_LOG_UART_RXP 						GPIOG
#endif /* LOG_UART */
#define CONFIG_LOG_TICK_TIME 						1
#endif /* LOG_MONITOR */



/**
 * PERIPHERAL CONFIG.
 */
#define CONFIG_FAIL_CHIP_RESET 						1
#if CONFIG_FAIL_CHIP_RESET
#define CONFIG_WAIT_FOR_RESET_TIME 					5
#endif /* CONFIG_FAIL_CHIP_RESET */




















#ifdef __cplusplus
}
#endif

#endif /* SDKCONFIG_H_ */
