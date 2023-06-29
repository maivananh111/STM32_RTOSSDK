/*
 * exti.h
 *
 *  Created on: Jan 10, 2023
 *      Author: anh
 */

#ifndef PERIPH_EXTI_H_
#define PERIPH_EXTI_H_

#include "peripheral_config.h"

#if ENABLE_EXTI

#ifdef __cplusplus
extern "C" {
#endif

#include "sdkconfig.h"
#include st_header
#include "system/ret_err.h"


typedef enum{
	EXTI_RISING_EDGE  = EXTI_RTSR_TR0,
	EXTI_FALLING_EDGE = EXTI_FTSR_TR0,
	EXTI_RISING_FALLING_EDGE  = EXTI_RTSR_TR0 | EXTI_FTSR_TR0,
} exti_edgedetect_t;

stm_ret_t exti_init(GPIO_TypeDef *Port, uint16_t Pin, exti_edgedetect_t Edge, uint32_t Priority);
void exti_deinit(GPIO_TypeDef *Port, uint16_t Pin);

void exti_register_event_handler(uint16_t pin, void (*function_ptr)(void *param), void *param);
void exti_unregister_event_handler(uint16_t pin);





#ifdef __cplusplus
}
#endif

#endif

#endif /* PERIPH_EXTI_H_ */
