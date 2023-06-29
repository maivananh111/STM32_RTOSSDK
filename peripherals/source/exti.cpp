/*
 * exti.cpp
 *
 *  Created on: Jan 10, 2023
 *      Author: anh
 */

#include "peripheral_config.h"

#if ENABLE_EXTI

#include "stdio.h"
#include "sdkconfig.h"

#include "periph/exti.h"
#include "system/system.h"
#include "periph/systick.h"
#if CONFIG_USE_LOG_MONITOR && EXTI_LOG
#include "system/log.h"
#endif /* CONFIG_USE_LOG_MONITOR && EXTI_LOG */

#define EXTI_LINE_INDEX 6U

#if CONFIG_USE_LOG_MONITOR && EXTI_LOG
static const char *TAG = "EXTI";
#endif /* CONFIG_USE_LOG_MONITOR && EXTI_LOG */


#if defined(STM32F1)
#define EXTI_REGISTER AFIO
#define GPIO_ADRRESS_OFFSET 0x0800UL
#elif defined(STM32F4)
#define EXTI_REGISTER SYSCFG
#define GPIO_ADRRESS_OFFSET 0x0000UL
#endif /* STM32F4 */

void (*handler_callback[16])(void *param) = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
void *parameter[16] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};

extern "C" {
void EXTI0_IRQHandler(void);           /* EXTI Line0 interrupt */
void EXTI1_IRQHandler(void);           /* EXTI Line1 interrupt */
void EXTI2_IRQHandler(void);           /* EXTI Line2 interrupt */
void EXTI3_IRQHandler(void);           /* EXTI Line3 interrupt */
void EXTI4_IRQHandler(void);           /* EXTI Line4 interrupt */
void EXTI9_5_IRQHandler(void);         /* EXTI Line[9:5] interrupts */
void EXTI15_10_IRQHandler(void);       /* EXTI Line[15:10] interrupts */

void EXTI_IRQHandler(uint16_t Pin);
}

stm_ret_t exti_init(GPIO_TypeDef *Port, uint16_t Pin, exti_edgedetect_t Edge, uint32_t Priority){
	stm_ret_t ret;
	uint8_t CRPos = 0;
	IRQn_Type IRQn;

	if(Priority < CONFIG_RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && EXTI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && EXTI_LOG */
#if CONFIG_FAIL_CHIP_RESET
#if CONFIG_USE_LOG_MONITOR && EXTI_LOG
		LOG_INFO(TAG, "Chip will reset after %ds.", CONFIG_WAIT_FOR_RESET_TIME);
#endif /* CONFIG_USE_LOG_MONITOR && EXTI_LOG */
		systick_delay_ms(CONFIG_WAIT_FOR_RESET_TIME*1000U);
		__NVIC_SystemReset();
#endif /* CONFIG_FAIL_CHIP_RESET */
		return ret;
	}


	if(Pin < 4U) 					CRPos = 0;
	else if(Pin >= 4U && Pin < 8U)  CRPos = 1;
	else if(Pin >= 8U && Pin < 12U) CRPos = 2;
	else 							CRPos = 3;

	if(Pin < 5U) IRQn = (IRQn_Type)(Pin + EXTI_LINE_INDEX);
	else if(Pin >= 5U && Pin < 9U) IRQn = EXTI9_5_IRQn;
	else 						   IRQn = EXTI15_10_IRQn;

#if defined(STM32F1)
	if(!(RCC -> APB2ENR & RCC_APB2ENR_AFIOEN)) RCC -> APB2ENR |= RCC_APB2ENR_AFIOEN;
#elif defined(STM32F4)
	if(!(RCC -> APB2ENR & RCC_APB2ENR_SYSCFGEN)) RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
#endif /* STM32F4 */
	__IO uint32_t tmpreg = EXTI_REGISTER -> EXTICR[CRPos]; 

	tmpreg &=~ (0x0F << ((Pin - CRPos*4U) * 4U));
	tmpreg |= (uint32_t)(((((uint32_t)((uint32_t)(Port) - GPIO_ADRRESS_OFFSET) & 0xFF00U) >> 8U) / 4U) << ((Pin - CRPos*4U) * 4U));

	EXTI_REGISTER -> EXTICR[CRPos] = tmpreg;

	if(Edge & EXTI_RTSR_TR0) EXTI -> RTSR |= (1U << Pin);
	if(Edge & EXTI_FTSR_TR0) EXTI -> FTSR |= (1U << Pin);

	EXTI -> IMR |= (1U << Pin);

	__NVIC_SetPriority(IRQn, Priority);
	__NVIC_EnableIRQ(IRQn);

	return ret;
}

void exti_deinit(GPIO_TypeDef *Port, uint16_t Pin){
	uint8_t CRPos = 0;
	if(Pin < 4U) 					CRPos = 0;
	else if(Pin >= 4U && Pin < 8U)  CRPos = 1;
	else if(Pin >= 8U && Pin < 12U) CRPos = 2;
	else 							CRPos = 3;
	EXTI -> PR = (1U << Pin);
	EXTI_REGISTER -> EXTICR[CRPos] &=~ (0x0F << ((Pin - CRPos*4U) * 4U));

	EXTI -> RTSR &=~ (1U << Pin);
	EXTI -> FTSR &=~ (1U << Pin);

	EXTI -> IMR &=~ (1U << Pin);

	IRQn_Type IRQn;
	if(Pin < 5U) IRQn = (IRQn_Type)(Pin + EXTI_LINE_INDEX);
	else if(Pin >= 5U && Pin < 9U) IRQn = EXTI9_5_IRQn;
	else 						   IRQn = EXTI15_10_IRQn;
	__NVIC_DisableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);
}

void exti_register_event_handler(uint16_t pin, void (*function_ptr)(void *param), void *param){
		handler_callback[pin] = function_ptr;
		parameter[pin] = param;
}

void exti_unregister_event_handler(uint16_t pin){
	handler_callback[pin] = NULL;
}

extern "C"{
void EXTI_IRQHandler(uint16_t Pin){
	if(EXTI -> PR & (1U << Pin)){
		EXTI -> PR = (1U << Pin);
		if(handler_callback[Pin] != NULL) handler_callback[Pin](parameter[Pin]);
	}
}


void EXTI0_IRQHandler(void){
	EXTI_IRQHandler(0);
}

void EXTI1_IRQHandler(void){
	EXTI_IRQHandler(1);
}

void EXTI2_IRQHandler(void){
	EXTI_IRQHandler(2);
}

void EXTI3_IRQHandler(void){
	EXTI_IRQHandler(3);
}

void EXTI4_IRQHandler(void){
	EXTI_IRQHandler(4);
}

void EXTI9_5_IRQHandler(void){
	EXTI_IRQHandler(5);
	EXTI_IRQHandler(6);
	EXTI_IRQHandler(7);
	EXTI_IRQHandler(8);
	EXTI_IRQHandler(9);
}

void EXTI15_10_IRQHandler(void){
	EXTI_IRQHandler(10);
	EXTI_IRQHandler(11);
	EXTI_IRQHandler(12);
	EXTI_IRQHandler(13);
	EXTI_IRQHandler(14);
	EXTI_IRQHandler(15);
}

}

#endif











