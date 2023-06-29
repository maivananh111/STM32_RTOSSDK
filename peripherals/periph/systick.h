/*
 * systick.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPH_SYSTICK_H_
#define PERIPH_SYSTICK_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "stdio.h"


extern volatile uint32_t sdk_tick;

void systick_init(uint32_t systick_priority);

void increment_tick(void);
void app_systick_process(void);

uint32_t systick_get_tick(void);
void systick_delay_ms(uint32_t ms);

uint32_t get_tick(void);
void delay_ms(uint32_t ms);

void set_function_get_tick(uint32_t (*func_ptr)(void));
void set_function_delay_ms(void(*func_ptr)(uint32_t));

#if !defined(USE_HAL_DRIVER)
void SysTick_Handler(void);
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_SYSTICK_H_ */
