
#include "startup.h"

#include "stdio.h"
#include "stdlib.h"
#include "stdint.h"
#include "math.h"


#define mkstr(s) #s

#if defined(STM32F1)
rcc_config_t rcc = {
	.hse_frequency   = HSE_VALUE,
	.hsi_frequency   = HSI_VALUE,
	.hsi_trim        = CONFIG_HSI_TRIM_VALUE,
	.osc_source      = CONFIG_OSC_CLOCK_SOURCE,
	.sysclock_source = CONFIG_SYSTEM_CLOCK_MUX,
	.pll_source      = CONFIG_PLL_SOURCE_MUX,
	.sysclock_frequency = CONFIG_SYSTEM_CLOCK_FREQUENCY,
	.ahb_prescaler      = CONFIG_AHB_PRESCALER,
	.apb1_prescaler 	= CONFIG_APB1_PRESCALER,
	.apb2_prescaler 	= CONFIG_APB2_PRESCALER,
	.pll_mul            = CONFIG_PLLMUL,
};
#elif defined(STM32F4)
rcc_config_t rcc = {
	.hse_frequency 		= HSE_VALUE,
	.hsi_frequency 		= HSI_VALUE,
	.hsi_trim 			= CONFIG_HSI_TRIM_VALUE,
	.osc_source 		= CONFIG_OSC_CLOCK_SOURCE,
	.sysclock_source 	= CONFIG_SYSTEM_CLOCK_MUX,
	.pll_source 		= CONFIG_PLL_SOURCE_MUX,
	.sysclock_frequency = CONFIG_SYSTEM_CLOCK_FREQUENCY,
	.ahb_prescaler      = CONFIG_AHB_PRESCALER,
	.apb1_prescaler 	= CONFIG_APB1_PRESCALER,
	.apb2_prescaler 	= CONFIG_APB2_PRESCALER,
	rcc.pll.pllm 		= CONFIG_RCC_PLLM,
	rcc.pll.plln 		= CONFIG_RCC_PLLN,
	rcc.pll.pllp 		= CONFIG_RCC_PLLP,
	rcc.pll.pllq 		= CONFIG_RCC_PLLQ,
};
#endif /* STM32F4 */

#if CONFIG_USE_SDRAM && ENABLE_FMC
sdram_config_t sdram_conf = {};
#endif /* CONFIG_USE_SDRAM && ENABLE_FMC */


#if CONFIG_USE_LOG_MONITOR
static const char* TAG = (const char *)"STARTUP";
#if ENABLE_USART && CONFIG_LOG_OVER_UART
static USART_TypeDef *log_uart = (USART_TypeDef *)CONFIG_LOG_UART_NUM;
SemaphoreHandle_t log_semaph;
#endif /* ENABLE_USART && LOG_UART */
#endif /* LOG_MONITOR */

#if defined(USE_HAL_DRIVER)
int main_application(void){
#else
int main(void){
#endif /* USE_HAL_DRIVER */
	system_init();
	rcc_init(&rcc);

	gpio_port_clock_enable(GPIOH);

#if CONFIG_USE_SDRAM && ENABLE_FMC
	fmc_sdram_init(&sdram_conf);
	sdram_init();
#endif /* CONFIG_USE_SDRAM && ENABLE_FMC */

#if CONFIG_RTOS_USE_IWDG && ENABLE_IWDG
	iwdg_init(CONFIG_IWDG_PRESCALER, CONFIG_IWDG_AUTORELOAD);
	iwdg_disable_in_debugmode();
#endif /* CONFIG_RTOS_USE_IWDG && ENABLE_IWDG */

#if	ENABLE_RNG && defined(RNG)
	rng_init();
#endif /* ENABLE_RNG && defined(RNG) */

#if CONFIG_USE_LOG_MONITOR
	log_semaph = xSemaphoreCreateMutex();
#if ENABLE_USART && CONFIG_LOG_OVER_UART
	uart_log_init();
	log_init(uart_log);
#endif
#if ENABLE_USB && CONFIG_LOG_OVER_USB
	MX_USB_DEVICE_Init();
	stm_log_init(usb_log);
#endif
	LOG_INFO(TAG, "Target        : %s",     mkstr(STM32F429xx));
	LOG_INFO(TAG, "Revision ID   : 0x%04x", get_revid());
	LOG_INFO(TAG, "Device ID     : 0x%04x", get_devid());
	LOG_INFO(TAG, "Flash size    : %dKb",   get_flashsize());
	LOG_INFO(TAG, "Ram size      : %dKb",   CONFIG_TOTAL_HEAP_SIZE/1024);
	LOG_INFO(TAG, "SDK version   : %s",     SDK_VERSION);
	LOG_INFO(TAG, "Core frequency: %luHz",  rcc_get_bus_frequency(SYSCLK));
	LOG_INFO(TAG, "AHB frequency : %luHz",  rcc_get_bus_frequency(AHB));
	LOG_INFO(TAG, "APB1 frequency: %luHz",  rcc_get_bus_frequency(APB1));
	LOG_INFO(TAG, "APB2 frequency: %luHz",  rcc_get_bus_frequency(APB2));

#endif
	BaseType_t app_start_status = xTaskCreate(app_main_task, "app_main_task", CONFIG_RTOS_APP_MAIN_TASK_SIZE, NULL, CONFIG_RTOS_APP_MAIN_TASK_PRIO, NULL);
	if(app_start_status != pdTRUE) {
		LOG_ERROR(TAG, "Error when start main application at %s -> %s Line: %d", __FILE__, __FUNCTION__, __LINE__);
		return 0;
	}
	LOG_INFO(TAG, "Starting scheduler on CPU.");
	vTaskStartScheduler();

	return (int)app_start_status;
}


void app_main_task(void *param){
	LOG_INFO(TAG, "Calling app_main().");
	extern void app_main(void);
	app_main();
	LOG_INFO(TAG, "Returned from app_main().");
	vTaskDelete(NULL);
}
extern"C"{
	void vApplicationIdleHook(void){
		/** Do something while cpu idle. */
#if CONFIG_RTOS_USE_IWDG && ENABLE_IWDG
		/* Reset Independent Watchdog timer */
		iwdg_refresh();
#endif /* CONFIG_RTOS_USE_IWDG */
		sys_calculate_cpu_load_percent();
	}

	void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName){
		LOG_ERROR(TAG, "Stack overflow on %s.", pcTaskName);
		for(uint32_t i=0; i< 4000000; i++) __NOP();
		__NVIC_SystemReset();
	}

	void vApplicationMallocFailedHook(void){
		LOG_ERROR(TAG, "Memory allocation fail.");
		__NVIC_SystemReset();
	}
}

#if CONFIG_USE_LOG_MONITOR
#if ENABLE_USART && CONFIG_LOG_OVER_UART
static void uart_log_init(void){
	__IO uint32_t USART_BusFreq = 0UL;

	gpio_port_clock_enable(CONFIG_LOG_UART_TXP);
	gpio_port_clock_enable(CONFIG_LOG_UART_RXP);
	if(log_uart == USART1 || log_uart == USART2 || log_uart == USART3){
		gpio_set_alternatefunction(CONFIG_LOG_UART_TXP, CONFIG_LOG_UART_TX, AF7_USART1_3);
		gpio_set_alternatefunction(CONFIG_LOG_UART_RXP, CONFIG_LOG_UART_RX, AF7_USART1_3);
	}
	else{
		gpio_set_alternatefunction(CONFIG_LOG_UART_TXP, CONFIG_LOG_UART_TX, AF8_USART4_8);
		gpio_set_alternatefunction(CONFIG_LOG_UART_RXP, CONFIG_LOG_UART_RX, AF8_USART4_8);
	}
	if(log_uart == USART1 || log_uart == USART6) {
		if(log_uart == USART1) 		RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
		else if(log_uart == USART6) RCC -> APB2ENR |= RCC_APB2ENR_USART6EN;
		USART_BusFreq = rcc_get_bus_frequency(APB2);
	}
	else {
		if(log_uart == USART2) 		RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
		else if(log_uart == USART3) RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;
		else if(log_uart == UART4)  RCC -> APB1ENR |= RCC_APB1ENR_UART4EN;
		else if(log_uart == UART5)  RCC -> APB1ENR |= RCC_APB1ENR_UART5EN;
		USART_BusFreq = rcc_get_bus_frequency(APB1);
	}

	log_uart -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
	float USARTDIV = (float)(USART_BusFreq/(CONFIG_LOG_UART_BAUDRATE * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	DIV_Fraction = ceil(Fraction);
	log_uart -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	uart_log((char *)"\r\n\r\n*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*Target starting*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*\r\n");
}

static void uart_log(char *log){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	BaseType_t ret, in_it = xPortIsInsideInterrupt();
	(in_it == pdTRUE)? (ret = xSemaphoreTakeFromISR(log_semaph, &xHigherPriorityTaskWoken)) : (ret = xSemaphoreTake(log_semaph, 10));

	if(ret == pdTRUE){
		while(*log) {
			log_uart -> DR = *log++;
			while(!(log_uart -> SR & USART_SR_TC));
		}

		(in_it == pdTRUE)? xSemaphoreGiveFromISR(log_semaph, &xHigherPriorityTaskWoken) : xSemaphoreGive(log_semaph);
	}
}
#endif /* ENABLE_USART */

#if ENABLE_USB && CONFIG_LOG_OVER_USB
static void usb_log(char *log){
	CDC_Transmit_FS((uint8_t *)log, strlen(log));
}
#endif /* ENABLE_USB && LOG_USB */

#endif /* LOG_MONITOR */












