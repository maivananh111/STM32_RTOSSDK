/*
 * usart.cpp
 *
 *  Created on: 20 thg 11, 2022
 *      Author: anh
 */
#include "peripheral_config.h"
#if ENABLE_USART
#include "periph/usart.h"

#include "math.h"
#include "string.h"

#include "periph/gpio.h"
#include "periph/systick.h"
#include "periph/rcc.h"
#if CONFIG_USE_LOG_MONITOR && USART_LOG
#include "system/log.h"
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */


#if CONFIG_USE_LOG_MONITOR && USART_LOG
static const char *TAG = "USART";
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */

void USART_IRQ_Handler(usart *usart);

usart::usart(USART_TypeDef *usart){
	_usart = usart;
}

stm_ret_t usart::init(usart_config_t *conf){
	stm_ret_t ret;
	_conf = conf;
#if ENABLE_DMA
	_rxdma = _conf -> rxdma;
	_txdma = _conf -> txdma;
#endif /* ENABLE_DMA */
	__IO uint32_t usart_bus_frequency = 0UL;

	gpio_port_clock_enable(_conf -> txport);
	gpio_port_clock_enable(_conf -> rxport);

#if defined(STM32F1)
	gpio_set_alternatefunction(conf->txport, conf->txpin, GPIO_ALTERNATE_PUSHPULL);
	gpio_set_mode(conf->rxport, conf->rxpin, GPIO_INPUT);
#elif defined(STM32F4)
	if(
#if defined(USART1)
			_usart == USART1
#endif /* defined(USART1) */
			||
#if defined(USART2)
			_usart == USART2
#endif /* defined(USART2) */
			||
#if defined(USART3)
			_usart == USART3
#endif /* defined(USART3) */
			){
		gpio_set_alternatefunction(_conf -> txport, _conf -> txpin, AF7_USART1_3);
		gpio_set_alternatefunction(_conf -> rxport, _conf -> rxpin, AF7_USART1_3);
	}
	else{
#if defined(UART7) & defined(UART8)
		gpio_set_alternatefunction(_conf -> txport, _conf -> txpin, AF8_USART4_8);
		gpio_set_alternatefunction(_conf -> rxport, _conf -> rxpin, AF8_USART4_8);
#else
		gpio_set_alternatefunction(_conf -> txport, _conf -> txpin, AF8_USART4_6);
		gpio_set_alternatefunction(_conf -> rxport, _conf -> rxpin, AF8_USART4_6);
#endif /* defined(UART7) & defined(UART8) */
	}
	gpio_set_alternatefunction_type(_conf -> txport, _conf -> txpin, GPIO_OUTPUT_PUSHPULL);
	gpio_set_alternatefunction_type(_conf -> rxport, _conf -> rxpin, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */

#if defined(USART1)
	if(_usart == USART1){
		RCC -> APB2ENR |= RCC_APB2ENR_USART1EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB2);
	}
#endif /* defined(USART1) */
#if defined(USART2)
	if(_usart == USART2){
		RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(USART2) */
#if defined(USART3)
	if(_usart == USART3){
		RCC -> APB1ENR |= RCC_APB1ENR_USART3EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(USART3) */
#if defined(UART4)
	if(_usart == UART4){
		RCC -> APB1ENR |= RCC_APB1ENR_UART4EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(UART4) */
#if defined(UART5)
	if(_usart == UART5){
		RCC -> APB1ENR |= RCC_APB1ENR_UART5EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(UART5) */
#if defined(USART6)
	if(_usart == USART6){
		RCC -> APB2ENR |= RCC_APB2ENR_USART6EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB2);
	}
#endif /* defined(USART6) */
#if defined(UART7)
	if(_usart == UART7){
		RCC -> APB1ENR |= RCC_APB1ENR_UART7EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(UART7) */
#if defined(UART8)
	if(_usart == UART8){
		RCC -> APB1ENR |= RCC_APB1ENR_UART8EN;
		usart_bus_frequency = rcc_get_bus_frequency(APB1);
	}
#endif /* defined(UART8) */


	_usart -> CR1 |= USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;

	float USARTDIV = (float)(usart_bus_frequency/(_conf -> baudrate * 16.0));
	uint16_t DIV_Fraction = 0x00UL;
	uint16_t DIV_Mantissa = (uint16_t)USARTDIV;

	float Fraction = (float)(((float)(((uint16_t)(USARTDIV * 100.0) - (uint16_t)(DIV_Mantissa * 100.0)) / 100.0)) * 16.0);
	DIV_Fraction = ceil(Fraction);
	_usart -> BRR = (DIV_Mantissa << 4) | (DIV_Fraction << 0);

	if(_conf -> control && (USART_INTERRUPT_CONTROL | USART_INTERRUPT_DMA_CONTROL)){

		if(_conf -> interruptpriority < CONFIG_RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
			set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
			LOG_ERROR(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
#if CONFIG_FAIL_CHIP_RESET
#if CONFIG_USE_LOG_MONITOR && USART_LOG
			LOG_INFO(TAG, "Chip will reset after %ds.", CONFIG_WAIT_FOR_RESET_TIME);
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
			systick_delay_ms(CONFIG_WAIT_FOR_RESET_TIME*1000U);
			__NVIC_SystemReset();
#endif /* CONFIG_FAIL_CHIP_RESET */
			return ret;
		}

#if defined(USART1)
		if	   (_usart == USART1) IRQn = USART1_IRQn;
#endif /* defined(USART1) */
#if defined(USART2)
		else if(_usart == USART2) IRQn = USART2_IRQn;
#endif /* defined(USART2) */
#if defined(USART3)
		else if(_usart == USART3) IRQn = USART3_IRQn;
#endif /* defined(USART3) */
#if defined(UART4)
		else if(_usart == UART4)  IRQn = UART4_IRQn;
#endif /* defined(USART4) */
#if defined(UART5)
		else if(_usart == UART5)  IRQn = UART5_IRQn;
#endif /* defined(USART5) */
#if defined(USART6)
		else if(_usart == USART6) IRQn = USART6_IRQn;
#endif /* defined(USART6) */
#if defined(UART7)
		else if(_usart == UART7) IRQn = UART7_IRQn;
#endif /* defined(USART7) */
#if defined(UART8)
		else if(_usart == UART8) IRQn = UART8_IRQn;
#endif /* defined(UART8) */
		__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	}
	transmit('\n');

	return {STM_OKE, 0};
}

stm_ret_t usart::deinit(void){
	gpio_deinit(_conf -> txport, _conf -> txpin);
	gpio_deinit(_conf -> rxport, _conf -> rxpin);

#if defined(USART1)
	if(_usart == USART1) RCC -> APB2ENR &=~ RCC_APB2ENR_USART1EN;
#endif /* defined(USART1) */
#if defined(USART2)
	if(_usart == USART2) RCC -> APB1ENR &=~ RCC_APB1ENR_USART2EN;
#endif /* defined(USART2) */
#if defined(USART3)
	if(_usart == USART3) RCC -> APB1ENR &=~ RCC_APB1ENR_USART3EN;
#endif /* defined(USART3) */
#if defined(UART4)
	if(_usart == UART4) RCC -> APB1ENR &=~ RCC_APB1ENR_UART4EN;
#endif /* defined(UART4) */
#if defined(UART5)
	if(_usart == UART5) RCC -> APB1ENR &=~ RCC_APB1ENR_UART5EN;
#endif /* defined(UART5) */
#if defined(USART6)
	if(_usart == USART6) RCC -> APB2ENR &=~ RCC_APB2ENR_USART6EN;
#endif /* defined(USART6) */
#if defined(UART7)
	if(_usart == UART7) RCC -> APB1ENR &=~ RCC_APB1ENR_UART7EN;
#endif /* defined(UART7) */
#if defined(UART8)
	if(_usart == UART8) RCC -> APB1ENR &=~ RCC_APB1ENR_UART8EN;
#endif /* defined(UART8) */

	_usart -> CR1 = 0x00U;
	_usart -> CR2 = 0x00U;
	_usart -> BRR = 0x00U;
	__IO uint32_t tmp = _usart -> DR;
	tmp = _usart -> SR;
	(void)tmp;
	if(_conf -> control && (USART_INTERRUPT_CONTROL | USART_INTERRUPT_DMA_CONTROL)){
		__NVIC_ClearPendingIRQ(IRQn);
		__NVIC_DisableIRQ(IRQn);
	}

	return {STM_OKE, 0};
}

stm_ret_t usart::register_event_handler(void (*function_ptr)(usart_event_t event, void *param), void *param){
	stm_ret_t ret;
	if(_conf -> control && (USART_INTERRUPT_CONTROL | USART_INTERRUPT_DMA_CONTROL)) {
		handler_callback = function_ptr;
		parameter = param;

		return ret;
	}
	else{
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_WARN(TAG, "%s -> %s, USART peripheral control unsuported register event handler.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
	}

	return ret;
}


stm_ret_t usart::transmit(uint8_t data){
	stm_ret_t ret;

	_usart -> DR = data;

	ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return ret;
}


stm_ret_t usart::transmit(uint8_t *data, uint16_t len){
	stm_ret_t ret;
	uint16_t TxCount = len;

	while(TxCount--) {
		_usart -> DR = *data++;

		ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	return ret;
}



stm_ret_t usart::send_string(char *string){
	stm_ret_t ret;

	while(*string) {
		_usart -> DR = *string++;

		ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	return ret;
}



stm_ret_t usart::receive(uint8_t *data){
	stm_ret_t ret;

	ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_RXNE, FLAG_SET, USART_TIMEOUT);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	(*data) = _usart -> DR;

	return ret;
}



stm_ret_t usart::receive(uint16_t len){
	stm_ret_t ret;
	__IO uint16_t RxCount = len;

	rxlen = len;
	rxcount = 0;
	reception = USART_RECEPTION_NORMAL;
	if(rxbuffer != NULL) {
		free(rxbuffer);
		rxbuffer = NULL;
	}
	rxbuffer = (uint8_t *)malloc((rxlen +1) * sizeof(uint8_t));
	if(rxbuffer == NULL){
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART memory allocation fail.", __FILE__, __FUNCTION__ );
#endif
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}
	memset(rxbuffer, '\0', rxlen+1);

	while(RxCount--){
		ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_RXNE, FLAG_SET, USART_TIMEOUT);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}

		(*rxbuffer++) = _usart -> DR;
	}
	return ret;
}

uint8_t usart::get_data(void){
	return _usart -> DR;
}


stm_ret_t usart::transmit_start_it(uint8_t *data, uint16_t len){
	stm_ret_t ret;

	if(_conf -> control && (USART_INTERRUPT_CONTROL | USART_INTERRUPT_DMA_CONTROL)) {
		if(_conf -> interruptoption && (USART_TRANSMIT_INTERRUPT | USART_TRANSMIT_RECEIVE_INTERRUPT))
			_usart -> CR1 |= USART_CR1_TCIE;
		else {
#if CONFIG_USE_LOG_MONITOR && USART_LOG
			LOG_ERROR(TAG, "%s -> %s, USART not selected transmit interrupt.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		}
	}
	else{
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART not selected interrupt control.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		return ret;
	}

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;

	uint16_t TxLength = len;
	uint16_t TxCount  = 0;
	while(TxCount++ < TxLength) {
		_usart -> DR = *data++;

		ret = wait_flag_in_register_timeout(&(_usart -> SR), USART_SR_TC, FLAG_SET, USART_TIMEOUT);
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}

	_usart -> CR1 &=~ (USART_CR1_PEIE | USART_CR1_TCIE);
	_usart -> CR3 &=~ USART_CR3_EIE;

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_EnableIRQ(IRQn);

	tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return ret;
}



stm_ret_t usart::receive_start_it(uint16_t buffer_size){
	stm_ret_t ret;

	if(_conf -> control && (USART_INTERRUPT_CONTROL | USART_INTERRUPT_DMA_CONTROL)) {
		if(_conf -> interruptoption && (USART_RECEIVE_INTERRUPT | USART_TRANSMIT_RECEIVE_INTERRUPT))
			_usart -> CR1 |= USART_CR1_RXNEIE;
		else {
#if CONFIG_USE_LOG_MONITOR && USART_LOG
			LOG_ERROR(TAG, "%s -> %s, USART not selected receive interrupt.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		}
	}
	else{
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART not selected interrupt control.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		return ret;
	}

	rxlen = buffer_size;
	rxcount = 0;
	reception = USART_RECEPTION_NORMAL;
	if(rxbuffer != NULL) {
		free(rxbuffer);
		rxbuffer = NULL;
	}
	rxbuffer = (uint8_t *)malloc((rxlen +1) * sizeof(uint8_t));
	if(rxbuffer == NULL){
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART memory allocation fail.", __FILE__, __FUNCTION__ );
#endif
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}
	memset(rxbuffer, '\0', rxlen+1);
	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_EnableIRQ(IRQn);

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	return ret;
}

stm_ret_t usart::transmit_stop_it(void){
	stm_ret_t ret;

	if(_conf -> control && (USART_INTERRUPT_CONTROL | USART_INTERRUPT_DMA_CONTROL)) {
		if((_conf -> interruptoption && (USART_TRANSMIT_INTERRUPT | USART_TRANSMIT_RECEIVE_INTERRUPT))\
				&& (_usart -> CR1 & USART_CR1_TCIE)){

			_usart -> CR1 &=~ USART_CR1_TCIE;

			__NVIC_ClearPendingIRQ(IRQn);
			__NVIC_DisableIRQ(IRQn);

			return ret;
		}
		else{
#if CONFIG_USE_LOG_MONITOR && USART_LOG
			LOG_ERROR(TAG, "%s -> %s, USART not selected transmit interrupt.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		}
	}

	set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
	LOG_ERROR(TAG, "%s -> %s, USART not started transmit interrupt.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */

	return ret;
}

stm_ret_t usart::receive_stop_it(void){
	stm_ret_t ret;

	if(_conf -> control && (USART_INTERRUPT_CONTROL | USART_INTERRUPT_DMA_CONTROL)) {
		if((_conf -> interruptoption && (USART_RECEIVE_INTERRUPT | USART_TRANSMIT_RECEIVE_INTERRUPT))\
					&& (_usart -> CR1 & USART_CR1_RXNEIE)){

			_usart -> CR1 &=~ USART_CR1_RXNEIE;

			__NVIC_ClearPendingIRQ(IRQn);
			__NVIC_DisableIRQ(IRQn);

			if(rxbuffer != NULL) {
				free(rxbuffer);
				rxbuffer = NULL;
			}
			rxcount = 0;
			rxlen = 0;
			reception = USART_RECEPTION_NORMAL;

			return ret;
		}
		else {
#if CONFIG_USE_LOG_MONITOR && USART_LOG
			LOG_ERROR(TAG, "%s -> %s, USART not selected receive interrupt.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		}
	}

	set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
	LOG_ERROR(TAG, "%s -> %s, USART not started receive interrupt.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */

	return ret;
}

#if ENABLE_DMA
stm_ret_t usart::transmit_start_dma(uint8_t *data, uint16_t len){
	stm_ret_t ret;

	if(!(_conf -> control && (USART_DMA_CONTROl | USART_INTERRUPT_DMA_CONTROL)) || _txdma == NULL){
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART not selected dma control.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		return ret;
	}

	_usart -> CR3 &=~ USART_CR3_DMAT;
	ret = _txdma -> start((uint32_t)data, (uint32_t)&_usart -> DR, len);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	_usart -> SR &=~ USART_SR_TC;
	_usart -> CR3 |= USART_CR3_DMAT;

	return ret;
}

stm_ret_t usart::receive_start_dma(uint16_t len){
	stm_ret_t ret;

	if(!(_conf -> control && (USART_DMA_CONTROl | USART_INTERRUPT_DMA_CONTROL)) || _rxdma == NULL){
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART not selected dma control.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		return ret;
	}

	rxlen = len;
	rxcount = 0;
	reception = USART_RECEPTION_NORMAL;
	if(rxbuffer != NULL) {
		free(rxbuffer);
		rxbuffer = NULL;
	}
	rxbuffer = (uint8_t *)malloc((rxlen +1) * sizeof(uint8_t));
	if(rxbuffer == NULL){
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART memory allocation fail.", __FILE__, __FUNCTION__ );
#endif
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}
	memset(rxbuffer, '\0', rxlen+1);
	_usart -> CR3 &=~ USART_CR3_DMAR;
	ret = _rxdma -> start((uint32_t)&_usart -> DR, (uint32_t)rxbuffer, len);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	volatile uint32_t tmp = _usart -> SR;
	tmp = _usart -> DR;
	(void)tmp;

	_usart -> CR1 |= USART_CR1_PEIE;
	_usart -> CR3 |= USART_CR3_EIE;
	_usart -> CR3 |= USART_CR3_DMAR;

	return ret;
}

stm_ret_t usart::transmit_stop_dma(void){
	stm_ret_t ret;

	if(!(_conf -> control && (USART_DMA_CONTROl | USART_INTERRUPT_DMA_CONTROL)) || _txdma == NULL){
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART not selected dma control.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		return ret;
	}

	if(_usart -> CR3 & USART_CR3_DMAT){
		_usart -> CR3 &=~ USART_CR3_DMAT;
		ret = _txdma -> stop();
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_usart -> CR1 &=~ USART_CR1_TXEIE;
	}

	else{
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART not started transmit dma.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
	}

	return ret;
}

stm_ret_t usart::receive_stop_dma(void){
	stm_ret_t ret;

	if(!(_conf -> control && (USART_DMA_CONTROl | USART_INTERRUPT_DMA_CONTROL)) || _rxdma == NULL){
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART not selected dma control.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
		return ret;
	}

	if(_usart -> CR3 & USART_CR3_DMAR){
		_usart -> CR3 &=~ USART_CR3_DMAR;
		ret = _rxdma -> stop();
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_usart -> CR1 &=~ USART_CR1_PEIE;
		_usart -> CR3 &=~ USART_CR3_EIE;

		if(rxbuffer != NULL) {
			free(rxbuffer);
			rxbuffer = NULL;
		}
		rxlen = 0;
		rxcount = 0;
		reception = USART_RECEPTION_NORMAL;
	}

	else{
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
		LOG_ERROR(TAG, "%s -> %s, USART not started receive dma.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */
	}

	return ret;
}
#endif /* ENABLE_DMA */



stm_ret_t usart::receive_to_idle_start_it(uint16_t buffer_size){
	stm_ret_t ret = receive_start_it(buffer_size);

	reception = USART_RECEPTION_TOIDLE;

	_usart -> CR1 |= USART_CR1_IDLEIE;

	return ret;
}
stm_ret_t usart::receive_to_idle_stop_it(void){
	stm_ret_t ret = receive_stop_it();

	_usart -> CR1 &=~ USART_CR1_IDLEIE;

	return ret;
}

#if ENABLE_DMA
stm_ret_t usart::receive_to_idle_start_it_dma(uint16_t buffer_size){
	stm_ret_t ret = receive_start_dma(buffer_size);

	reception = USART_RECEPTION_TOIDLE;

	_usart -> CR1 |= USART_CR1_IDLEIE;

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_EnableIRQ(IRQn);

	return ret;
}

stm_ret_t usart::receive_to_idle_stop_it_dma(void){
	stm_ret_t ret = receive_stop_dma();

	_usart -> CR1 &=~ USART_CR1_IDLEIE;

	return ret;
}
#endif /* ENABLE_DMA */

stm_ret_t usart::receive_to_endchar_start_it(uint16_t buffer_size, char endchar){
	stm_ret_t ret =receive_start_it(buffer_size);

	this->endchar = endchar;
	reception = USART_RECEPTION_TOENDCHAR;

	return ret;
}

stm_ret_t usart::receive_to_endchar_stop_it(void){
	this->endchar = '\0';
	return receive_stop_it();
}

#if ENABLE_DMA
stm_ret_t usart::receive_to_endchar_start_dma(uint16_t buffer_size, char endchar){
	stm_ret_t ret =receive_start_dma(buffer_size);

	this->endchar = endchar;
	reception = USART_RECEPTION_TOENDCHAR;

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_EnableIRQ(IRQn);

	return ret;
}

stm_ret_t usart::receive_to_endchar_stop_dma(void){
	this->endchar = '\0';
	return receive_stop_dma();
}
#endif /* ENABLE_DMA */

stm_ret_t usart::get_buffer(uint8_t **data){
	stm_ret_t ret;

	if(rxbuffer != NULL){
		rxbuffer[rxcount+1] = '\0';
		*data = (uint8_t *)malloc(rxcount+1);
		if(data == NULL){
	#if CONFIG_USE_LOG_MONITOR && USART_LOG
			LOG_ERROR(TAG, "%s -> %s, USART memory allocation fail.", __FILE__, __FUNCTION__ );
	#endif
			set_return(&ret, STM_ERR, __LINE__);
			return ret;
		}
		memcpy(*data, rxbuffer, rxcount+1);

		free(rxbuffer);
		rxbuffer = NULL;

		return ret;
	}

	set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && USART_LOG
	LOG_ERROR(TAG, "%s -> %s, USART receive buffer empty.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && USART_LOG */

	return ret;
}


uint16_t usart::get_bufferlen(void){
	return rxcount;
}

usart_config_t *usart::get_config(void){
	return _conf;
}


void USART_IRQ_Handler(usart *usart){
	uint32_t StatusReg = usart -> _usart -> SR, CR1Reg = usart -> _usart -> CR1;
	usart_event_t event = USART_EVENT_NOEVENT;
	if(StatusReg & USART_SR_RXNE && CR1Reg & USART_CR1_RXNEIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
//		tmp = usart -> _usart -> DR;
		(void)tmp;

		event = USART_EVENT_RECEIVE_COMPLETE;
		usart -> _usart -> SR &=~ USART_SR_RXNE;

		if(usart -> rxcount < usart -> rxlen)
			usart -> rxbuffer[usart -> rxcount] = usart -> _usart -> DR;
		else{
			event = USART_EVENT_BUFFER_OVERFLOW;
			goto EventCB;
		}
		if(usart -> reception == USART_RECEPTION_TOENDCHAR){
			if(usart -> rxbuffer[usart -> rxcount] == usart -> endchar) {
				event = USART_EVENT_RECEIVE_ENDCHAR;
				usart -> _usart -> CR1 &=~ USART_CR1_PEIE;
				usart -> _usart -> CR3 &=~ USART_CR3_EIE;
			}
		}
		usart -> rxcount++;
		goto EventCB;
	}

	if(StatusReg & USART_SR_TC&& CR1Reg & USART_CR1_TCIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
		tmp = usart -> _usart -> DR;
		(void)tmp;

		event = USART_EVENT_TRANSMIT_COMPLETE;
		usart -> _usart -> SR &=~ USART_SR_TC;

		goto EventCB;
	}

	if(StatusReg & USART_SR_IDLE && CR1Reg & USART_CR1_IDLEIE) {
		volatile uint32_t tmp = usart -> _usart -> SR;
		tmp = usart -> _usart -> DR;
		(void)tmp;

		if(usart -> reception == USART_RECEPTION_TOIDLE){
			usart -> _usart -> SR &=~ (USART_SR_IDLE | USART_SR_RXNE);
#if ENABLE_DMA
			if(usart -> _usart -> CR3 & USART_CR3_DMAR){
				usart -> rxcount = usart -> rxlen - usart -> _rxdma -> get_counter();
				if(usart -> _rxdma -> get_config() -> mode != DMA_MODE_CIRCULAR){
					usart -> _rxdma -> stop();
					usart -> _usart -> CR3 &=~ (USART_CR3_EIE | USART_CR3_DMAR);
				}
				event = USART_EVENT_IDLE_STATE;
				goto EventCB;
			}
			else{
#endif /* ENABLE_DMA */
				event = USART_EVENT_IDLE_STATE;
				goto EventCB;
#if ENABLE_DMA
			}
#endif /* ENABLE_DMA */
		}
	}

	EventCB:
	if(usart -> handler_callback != NULL) usart -> handler_callback(event, usart -> parameter);

}

#if defined(USART1)
usart usart_1(USART1);
usart_t usart1 = &usart_1;
void USART1_IRQHandler(void){
	USART_IRQ_Handler(&usart_1);
}
#endif /* defined(USART1) */
#if defined(USART2)
usart usart_2(USART2);
usart_t usart2 = &usart_2;
void USART2_IRQHandler(void){
	USART_IRQ_Handler(&usart_2);
}
#endif /* defined(USART2) */
#if defined(USART3)
usart usart_3(USART3);
usart_t usart3 = &usart_3;
void USART3_IRQHandler(void){
	USART_IRQ_Handler(&usart_3);
}
#endif /* defined(USART3) */
#if defined(UART4)
usart uart_4 (UART4);
usart_t uart4 = &uart_4;
void UART4_IRQHandler(void){
	USART_IRQ_Handler(&uart_4);
}
#endif /* defined(UART4) */
#if defined(UART5)
usart uart_5 (UART5);
usart_t uart5 = &uart_5;
void UART5_IRQHandler(void){
	USART_IRQ_Handler(&uart_5);
}
#endif /* defined(UART5) */
#if defined(USART6)
usart usart_6(USART6);
usart_t usart6 = &usart_6;
void USART6_IRQHandler(void){
	USART_IRQ_Handler(&usart_6);
}
#endif /* defined(USART6) */
#if defined(UART7)
usart uart_7 (UART7);
usart_t uart7 = &uart_7;
void UART7_IRQHandler(void){
	USART_IRQ_Handler(&uart_7);
}
#endif /* defined(UART7) */
#if defined(UART8)
usart uart_8 (UART8);
usart_t uart8 = &uart_8;
void UART8_IRQHandler(void){
	USART_IRQ_Handler(&uart_8);
}
#endif /* defined(UART8) */

#endif /* ENABLE_USART */





