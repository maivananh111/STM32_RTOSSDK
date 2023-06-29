/*
 * freertos_port.cpp
 *
 *  Created on: Jun 21, 2023
 *      Author: anh
 */

#include "sdkconfig.h"
#include "peripheral_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include "periph/iwdg.h"
#include "system/system.h"
#include "system/log.h"

static const char *TAG = (const char *)"RTOS_PORT";


extern"C"{
	void vApplicationIdleHook(void){
		/** Do something while cpu idle. */
#if CONFIG_RTOS_USE_IWDG && ENABLE_IWDG
		/* Reset Independent Watchdog timer */
		iwdg_refresh();
#endif /* CONFIG_RTOS_USE_IWDG */
		sys_calculate_cpu_load_percent();
	}

	void vApplicationStackOverflowHook(TaskHandle_t xTask, signed char *pcTaskName){
		LOG_ERROR(TAG, "Stack overflow on %s.", pcTaskName);
		for(uint32_t i=0; i< 80000000; i++) __NOP();
		__NVIC_SystemReset();
	}

	void vApplicationMallocFailedHook(void){
		LOG_ERROR(TAG, "Memory allocation fail.");
		for(uint32_t i=0; i< 80000000; i++) __NOP();
		__NVIC_SystemReset();
	}
}
