/*
 * systick.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "stdio.h"

#include "periph/systick.h"
#include "sdkconfig.h"
#include "system/system.h"

#include "FreeRTOS.h"
#include "task.h"


volatile uint32_t sdk_tick;

static void (*delay_ms_func)(uint32_t) = systick_delay_ms;
static uint32_t (*get_tick_func)(void) = systick_get_tick;

extern volatile uint32_t systick_total_ticks;
extern volatile uint32_t systick_idle_ticks;
extern volatile float cpu_load_percent;


void systick_init(uint32_t systick_priority){
	SysTick_Config(SystemCoreClock / CONFIG_SYSTICK_RATE);

	NVIC_Set_Priority(SysTick_IRQn, systick_priority, 0U);
}

void increment_tick(void){
	sdk_tick++;
}

uint32_t systick_get_tick(void){
	return sdk_tick;
}

void systick_delay_ms(uint32_t ms){
	uint32_t tickstart = sdk_tick;
	uint32_t wait = ms;

	if (wait < 0xFFFFFFU) wait += 1UL;

	while((sdk_tick - tickstart) < wait);
}

uint32_t get_tick(void){
	return get_tick_func();
}

void delay_ms(uint32_t ms){
	delay_ms_func(ms);
}

void set_function_get_tick(uint32_t (*func_ptr)(void)){
	get_tick_func = func_ptr;
}

void set_function_delay_ms(void(*func_ptr)(uint32_t)){
	delay_ms_func = func_ptr;
}

extern "C"{
	void app_systick_process(void){
		increment_tick();

		systick_total_ticks++;
		if(systick_total_ticks == 1000){
			cpu_load_percent = (float)(100.0 - (((float)systick_idle_ticks/(float)systick_total_ticks)*100.0));
			systick_total_ticks = 0;
			systick_idle_ticks = 0;
		}
	}
}

#if !defined(USE_HAL_DRIVER)
void SysTick_Handler(void){
	app_systick_process();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED){
#endif /* INCLUDE_xTaskGetSchedulerState */
		xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
	}
#endif /* INCLUDE_xTaskGetSchedulerState */
}
#endif /* USE_HAL_DRIVER */











