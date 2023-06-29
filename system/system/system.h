/*
 * system.h
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#ifndef PERIPH_SYSTEM_H_
#define PERIPH_SYSTEM_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "stdio.h"


typedef struct memory_info{
	uint32_t heap_ram_used;
	uint32_t prog_ram_used;
	uint32_t stack_ram_used;
	uint32_t free_ram;
	uint32_t total_free_ram;
} memory_info_t;

uint32_t get_revid(void);
uint32_t get_devid(void);
uint32_t get_flashsize(void);

void system_init(void);

void embedded_flash_init(void);
void embedded_flash_set_latency(uint32_t latency);
uint32_t embedded_flash_calculate_latency(uint32_t freq);
uint32_t embedded_flash_get_latency(void);
void embedded_flash_update_latency(void);

void NVIC_Set_Priority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);

memory_info_t sys_get_memory_info(void);
uint32_t sys_get_free_heap_size(void);
uint32_t sys_get_used_heap_size(void);

void sys_calculate_cpu_load_percent(void);
float sys_get_cpu_percent(void);
float sys_get_ram_percent(void);

void exception_interrupt_handler(const char *tag, char *message);

#if !defined(USE_HAL_DRIVER)
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);
#endif /* USE_HAL_DRIVER */

#ifdef __cplusplus
}
#endif

#endif /* PERIPH_SYSTEM_H_ */
