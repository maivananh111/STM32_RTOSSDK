/*
 * gpio.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPH_GPIO_H_
#define PERIPH_GPIO_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include st_header
#include "stdio.h"


typedef enum{
	GPIO_Input = 0,
#if defined(STM32F1)
	GPIO_Analog,
#endif /* STM32F1 */
	GPIO_Output,
	GPIO_AlternateFunction,
} gpio_derection_t;

typedef enum{
	GPIO_PushPull = 0,
	GPIO_OpenDrain,
} gpio_outputtype_t;

typedef enum{
	GPIO_Speed_Low = 0,
	GPIO_Speed_Medium,
	GPIO_Speed_High,
#if defined(STM32F4)
	GPIO_Speed_VeryHigh,
#endif /* STM32F4 */
} gpio_outputspeed_t;

typedef enum{
	GPIO_NoPull = 0,
	GPIO_PullUp,
	GPIO_PullDown,
} gpio_pullresistor_t;

typedef enum{
	GPIO_INPUT = 0,
	GPIO_INPUT_PULL,
	GPIO_INPUT_PULLUP,
	GPIO_INPUT_PULLDOWN,

	GPIO_OUTPUT_OPENDRAIN,
	GPIO_OUTPUT_OPENDRAIN_PULLUP,
	GPIO_OUTPUT_OPENDRAIN_PULLDOWN,
	GPIO_OUTPUT_PUSHPULL,
	GPIO_OUTPUT_PUSHPULL_PULLUP,
	GPIO_OUTPUT_PUSHPULL_PULLDOWN,

	GPIO_ANALOG,
} gpio_mode_t;

#if defined(STM32F1)
typedef enum{
	GPIO_ALTERNATE_PUSHPULL,
	GPIO_ALTERNATE_OPENDRAIN,
} gpio_alternatefunction_t;
#elif defined(STM32F4)
typedef enum{
	AF0_SYSTEM = 0,
	AF1_TIM1_2,
	AF2_TIM3_5,
	AF3_TIM8_11,
	AF4_I2C1_3,
#if defined(SPI4) & defined(SPI5) & defined(SPI6)
	AF5_SPI1_6,
#else
	AF5_SPI1_2,
#endif /* defined(SPI4) & defined(SPI5) & defined(SPI6) */
#if defined(SAI1) & defined(SPI2) & defined(SPI3)
	AF6_SPI2_3_SAI1,
#else
	AF6_SPI3,
#endif /* defined(SAI1) & defined(SPI2) & defined(SPI3) */
	AF7_USART1_3,
#if defined(UART7) && defined(UART8)
	AF8_USART4_8,
#else
	AF8_USART4_6,
#endif /* defined(UART7) && defined(UART8) */

#if defined(LTDC)
	AF9_CAN1_2_LTDC_TIM12_14, // ONLY AFRL.
#else
	AF9_CAN1_2_TIM12_14,
#endif /* defined(LTDC) */
	AF10_USB,
	AF11_ETH,
	AF12_FSMC_SDIO_USB,
	AF13_DCMI,
#if defined(LTDC)
	AF14_LTDC,
#else
	AF14,
#endif /* defined(LTDC) */
	AF15_EVENTOUT,
} gpio_alternatefunction_t;
#endif /* STM32F4 */

#if defined(STM32F1)
typedef enum{
	SPI1_Remap 			= AFIO_MAPR_SPI1_REMAP,
	I2C1_Remap 			= AFIO_MAPR_I2C1_REMAP,
	USART1_Remap 		= AFIO_MAPR_USART1_REMAP,
	USART2_Remap 		= AFIO_MAPR_USART2_REMAP,
	USART3_Remap_PC 	= AFIO_MAPR_USART3_REMAP_PARTIALREMAP,
	USART3_Remap_PD 	= AFIO_MAPR_USART3_REMAP_FULLREMAP,
	TIM1_Partial_Remap  = AFIO_MAPR_TIM1_REMAP_PARTIALREMAP,
	TIM1_Full_Remap 	= AFIO_MAPR_TIM1_REMAP_FULLREMAP,
	TIM2_Partial_Remap1 = AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1,
	TIM2_Partial_Remap2 = AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2,
	TIM2_Full_Remap     = AFIO_MAPR_TIM2_REMAP_FULLREMAP,
	TIM3_Partial_Remap  = AFIO_MAPR_TIM3_REMAP_PARTIALREMAP,
	TIM3_Full_Remap     = AFIO_MAPR_TIM3_REMAP_FULLREMAP,
	TIM4_Remap     		= AFIO_MAPR_TIM4_REMAP,
	CAN_Remap_PB 		= AFIO_MAPR_CAN_REMAP_REMAP2,
	CAN_Remap_PD 		= AFIO_MAPR_CAN_REMAP_REMAP3,
} gpio_remap_t;
#endif /* STM32F1 */

typedef struct gpio_config{
	GPIO_TypeDef *port = GPIOA;
	uint16_t pin = 0U;
	gpio_derection_t direction = GPIO_Input;
	gpio_outputtype_t outputtype = GPIO_PushPull;
	gpio_outputspeed_t outputspeed = GPIO_Speed_Low;
	gpio_pullresistor_t pullresister = GPIO_NoPull;
#if defined(STM32F4)
	gpio_alternatefunction_t function = (gpio_alternatefunction_t)0;
#endif /* STM32F4 */
} gpio_config_t;


void gpio_allport_clock_enable(void);
void gpio_port_clock_enable(GPIO_TypeDef *port);

void gpio_init(gpio_config_t *conf);
void gpio_deinit(GPIO_TypeDef *port, uint16_t pin);

void gpio_set_mode(GPIO_TypeDef *port, uint16_t pin, gpio_mode_t mode);
void gpio_set_alternatefunction(GPIO_TypeDef *port, uint16_t pin, gpio_alternatefunction_t function);
void gpio_set_alternatefunction_type(GPIO_TypeDef *port, uint16_t pin, gpio_mode_t mode);

void gpio_set_pullup(GPIO_TypeDef *port, uint16_t pin);
void gpio_set_pulldown(GPIO_TypeDef *port, uint16_t pin);

void gpio_set(GPIO_TypeDef *port, uint16_t pin);
void gpio_reset(GPIO_TypeDef *port, uint16_t pin);
void gpio_toggle(GPIO_TypeDef *port, uint16_t pin);

void gpio_set_level(GPIO_TypeDef *port, uint16_t pin, int level);
int gpio_get_level(GPIO_TypeDef *port, uint16_t pin);

#if defined(STM32F1)
void gpio_remap(gpio_remap_t remap);
#endif /* STM32F1 */

#if defined(GPIOA)
#if defined(STM32F1)
#define GPIOA_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPAEN
#elif defined(STM32F4)
#define GPIOA_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOAEN
#endif /* STM32F4 */
#endif /* defined(GPIOA */
#if defined(GPIOB)
#if defined(STM32F1)
#define GPIOB_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPBEN
#elif defined(STM32F4)
#define GPIOB_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOBEN
#endif /* STM32F4 */
#endif /* defined(GPIOB */
#if defined(GPIOC)
#if defined(STM32F1)
#define GPIOC_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPCEN
#elif defined(STM32F4)
#define GPIOC_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOCEN
#endif /* STM32F4 */
#endif /* defined(GPIOC */
#if defined(GPIOD)
#if defined(STM32F1)
#define GPIOD_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPDEN
#elif defined(STM32F4)
#define GPIOD_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIODEN
#endif /* STM32F4 */
#endif /* defined(GPIOD */
#if defined(GPIOE)
#if defined(STM32F1)
#define GPIOE_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPEEN
#elif defined(STM32F4)
#define GPIOE_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOEEN
#endif /* STM32F4 */
#endif /* defined(GPIOE */
#if defined(GPIOF)
#if defined(STM32F1)
#define GPIOF_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPFEN
#elif defined(STM32F4)
#define GPIOF_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOFEN
#endif /* STM32F4 */
#endif /* defined(GPIOF */
#if defined(GPIOG)
#if defined(STM32F1)
#define GPIOG_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPGEN
#elif defined(STM32F4)
#define GPIOG_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOGEN
#endif /* STM32F4 */
#endif /* defined(GPIOG */
#if defined(GPIOH)
#if defined(STM32F1)
#define GPIOH_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPHEN
#elif defined(STM32F4)
#define GPIOH_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOHEN
#endif /* STM32F4 */
#endif /* defined(GPIOH */
#if defined(GPIOI)
#if defined(STM32F1)
#define GPIOI_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPIEN
#elif defined(STM32F4)
#define GPIOI_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOIEN
#endif /* STM32F4 */
#endif /* defined(GPIOI */
#if defined(GPIOJ)
#if defined(STM32F1)
#define GPIOJ_CLOCKENABLE() RCC -> APB2ENR |= RCC_APB2ENR_IOPJEN
#elif defined(STM32F4)
#define GPIOJ_CLOCKENABLE() RCC -> AHB1ENR |= RCC_AHB1ENR_GPIOJEN
#endif /* STM32F4 */
#endif /* defined(GPIOJ */

#ifdef __cplusplus
}
#endif


#endif /* PERIPH_GPIO_H_ */
