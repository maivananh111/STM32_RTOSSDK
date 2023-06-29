/*
 * system.c
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */

#include "system/system.h"

#include "sdkconfig.h"

#include "stdio.h"
#include "malloc.h"
#include "periph/systick.h"
#include "FreeRTOS.h"

#if CONFIG_USE_LOG_MONITOR
#include "system/log.h"
#endif /* CONFIG_USE_LOG_MONITOR */

extern "C" char *sbrk(int i);
extern char _end;
extern char _sdata;
extern char _estack;
extern char _Min_Stack_Size;

static char *ramstart = &_sdata;
static char *ramend = &_estack;
static char *minSP = (char*)(ramend - &_Min_Stack_Size);

static volatile uint32_t total_ram_size = CONFIG_TOTAL_HEAP_SIZE;
static volatile uint32_t total_ram_use  = 0U;

#define TICKS_PER_SECOND CONFIG_RTOS_TICK_RATE
volatile uint32_t systick_total_ticks = 0;
volatile uint32_t systick_idle_ticks = 0;
volatile uint32_t last_systick_idle_ticks = 0;
volatile float cpu_load_percent = 0.0;

#if !defined(USE_HAL_DRIVER)
static const char *Excep_TAG = "EXCEPTION";
static const char *Inter_TAG = "INTERRUPT";
#endif /* USE_HAL_DRIVER */

uint32_t get_revid(void){
	return((DBGMCU -> IDCODE) >> 16U);
}

uint32_t get_devid(void){
	return((DBGMCU -> IDCODE) & 0x0FFFU);
}

uint32_t get_flashsize(void){
#if defined(STM32F1)
	return (*(volatile uint16_t*)0x1FFFF7E0);
#elif defined(STM32F4)
	return (*(volatile uint16_t*)0x1FFF7A22);
#endif /* STM32F4 */
}

void system_init(void){
	embedded_flash_init();

	__NVIC_SetPriorityGrouping(0x03U);

	systick_init(CONFIG_SYSTICK_INTERRUPT_PRIORITY);

	RCC -> APB1ENR |= RCC_APB1ENR_PWREN;
#if defined(STM32F4)
	RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	PWR -> CR |= PWR_CR_VOS;
#endif /* STM32F4 */
}

void embedded_flash_init(void){
#if defined(STM32F1)
	/* FLASH LATENCY 2WS, PREFETCH BUFER ENABLE, DATA CACHE ENABLE */
	FLASH -> ACR |= FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE;
	while(!(FLASH -> ACR & FLASH_ACR_PRFTBS));
#elif defined(STM32F4)
#if CONFIG_FLASH_INSTRUCTION_CACHE
	FLASH -> ACR |= FLASH_ACR_ICEN;
#else
	FLASH -> ACR &=~ FLASH_ACR_ICEN;
#endif

#if CONFIG_FLASH_DATA_CACHE
	FLASH -> ACR |= FLASH_ACR_DCEN;

#else
	FLASH -> ACR &=~ FLASH_ACR_DCEN;
#endif

#if CONFIG_FLASH_PREFETCH_MODE
	FLASH -> ACR |= FLASH_ACR_PRFTEN;
#else
	FLASH -> ACR &=~ FLASH_ACR_PRFTEN;
#endif
#endif /* STM32F4 */
}

void embedded_flash_set_latency(uint32_t latency){
	FLASH -> ACR = ((FLASH -> ACR & (~FLASH_ACR_LATENCY_Msk)) | (latency << FLASH_ACR_LATENCY_Pos));
}

void embedded_flash_update_latency(void){
	uint32_t tmpreg = (FLASH -> ACR & (~FLASH_ACR_LATENCY_Msk));

	uint32_t latency= (uint32_t)(SystemCoreClock / 30000000U);
	if(SystemCoreClock == 30000000U || SystemCoreClock == 60000000U || SystemCoreClock == 90000000U
    || SystemCoreClock == 120000000U || SystemCoreClock == 150000000U || SystemCoreClock == 180000000U) latency -= 1;

	tmpreg |= (uint32_t)(latency << FLASH_ACR_LATENCY_Pos);
	FLASH -> ACR |= tmpreg;
}

uint32_t embedded_flash_calculate_latency(uint32_t freq){
#if defined(STM32F1)
	uint32_t latency= (uint32_t)(freq / 24000000U);
	if(freq == 24000000U || freq == 48000000U || freq == 72000000U) latency -= 1;
#elif defined(STM32F4)
	uint32_t latency= (uint32_t)(freq / 30000000U);
	if(freq == 30000000U || freq == 60000000U || freq == 90000000U
    || freq == 120000000U || freq == 150000000U || freq == 180000000U) latency -= 1;
#endif /* STM32F4 */

	return latency;
}

uint32_t embedded_flash_get_latency(void){
	return (FLASH -> ACR & FLASH_ACR_LATENCY_Msk >> FLASH_ACR_LATENCY_Pos);
}


void NVIC_Set_Priority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority){
	uint32_t prioritygroup = 0x00U;

	if(PreemptPriority > 15U) PreemptPriority = 15U;
	if(SubPriority > 15U) SubPriority = 15U;

	prioritygroup = __NVIC_GetPriorityGrouping();

	__NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}

memory_info_t sys_get_memory_info(void){
	memory_info_t mem;
	char *heapend = (char*)sbrk(0);
	char * stack_ptr = (char*)__get_MSP();
	struct mallinfo mi = mallinfo();

	mem.free_ram = ((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks;
	mem.heap_ram_used = mi.uordblks;
	mem.prog_ram_used = &_end - ramstart;
	mem.stack_ram_used = ramend - stack_ptr;
	mem.total_free_ram = mi.fordblks;

	return mem;
}

uint32_t sys_get_free_heap_size(void){
	char *heapend = (char*)sbrk(0);
	char * stack_ptr = (char*)__get_MSP();
	struct mallinfo mi = mallinfo();

	return ((stack_ptr < minSP) ? stack_ptr : minSP) - heapend + mi.fordblks;
}

uint32_t sys_get_used_heap_size(void){
	struct mallinfo mi = mallinfo();

	return mi.uordblks;
}

void sys_calculate_cpu_load_percent(void){
    if((sdk_tick - last_systick_idle_ticks) >= 1){
    	systick_idle_ticks++;
    	last_systick_idle_ticks = sdk_tick;
    }
}

float sys_get_cpu_percent(void){
	return cpu_load_percent;
}

float sys_get_ram_percent(void){
	total_ram_use = sys_get_free_heap_size() + xPortGetFreeHeapSize();

	return (float)(((float)total_ram_use / (float)total_ram_size) * 100.0F);
}


void exception_interrupt_handler(const char *tag, char *message){
#if CONFIG_USE_LOG_MONITOR
	LOG_ERROR(tag, message);
#endif /* CONFIG_USE_LOG_MONITOR */
}

#if !defined(USE_HAL_DRIVER)
void NMI_Handler(void){
	exception_interrupt_handler(Inter_TAG, (char *)"NonMaskable interrupt was unhandled(NMI_Handler)...");
    while(1){}
}


void HardFault_Handler(void){
	exception_interrupt_handler(Excep_TAG, (char *)"Hard fault exception was unhandled(call HardFault_Handler)...");
    while (1){}
}

void MemManage_Handler(void){
	exception_interrupt_handler(Inter_TAG, (char *)"Memory management interrupt was unhandled(MemManage_Handler)...");
    while (1){}
}

void BusFault_Handler(void){
	exception_interrupt_handler(Excep_TAG, (char *)"Bus fault exception was unhandled(call BusFault_Handler)...");
    while (1){}
}

void UsageFault_Handler(void){
	exception_interrupt_handler(Excep_TAG, (char *)"Usage fault exception was unhandled(call UsageFault_Handler)...");
    while (1){}
}

void DebugMon_Handler(void){
	exception_interrupt_handler(Inter_TAG, (char *)"Debug monitor interrupt was unhandled(call DebugMon_Handler)...");
}

#endif /* USE_HAL_DRIVER */








