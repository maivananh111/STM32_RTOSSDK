/*
 * spi.cpp
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */

#include "peripheral_config.h"
#if ENABLE_SPI

#include "periph/spi.h"
#include "periph/gpio.h"
#include "periph/systick.h"
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
#include "system/log.h"
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */


#if CONFIG_USE_LOG_MONITOR && SPI_LOG
static const char *TAG = "SPI";
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */


stm_ret_t cc;
stm_ret_t SPI_Tx_ISR(spi *spi);
stm_ret_t SPI_Rx_ISR(spi *spi);

void SPI_IRQHandler(spi *spi);

spi::spi(SPI_TypeDef *Spi){
	_spi = Spi;
}

stm_ret_t spi::init(spi_config_t *conf){
	stm_ret_t ret;
	_conf = conf;
#if ENABLE_DMA
	_txdma = _conf->txdma;
	_rxdma = _conf->rxdma;
#endif /* ENABLE_DMA */

#if defined(SPI1)
	if     (_spi == SPI1) RCC -> APB2ENR |= RCC_APB2ENR_SPI1EN;
#endif /* defined(SPI1) */
#if defined(SPI2)
	else if(_spi == SPI2) RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;
#endif /* defined(SPI2) */
#if defined(SPI3)
	else if(_spi == SPI3) RCC -> APB1ENR |= RCC_APB1ENR_SPI3EN;
#endif /* defined(SPI3) */
#if defined(SPI4)
	else if(_spi == SPI4) RCC -> APB2ENR |= RCC_APB2ENR_SPI4EN;
#endif /* defined(SPI4) */
#if defined(SPI5)
	else if(_spi == SPI5) RCC -> APB2ENR |= RCC_APB2ENR_SPI5EN;
#endif /* defined(SPI6) */
#if defined(SPI6)
	else if(_spi == SPI6) RCC -> APB2ENR |= RCC_APB2ENR_SPI6EN;
#endif /* defined(SPI4) */


#if defined(STM32F1)
	if(_conf->mode == SPI_FULLDUPLEX_MASTER || _conf->mode == SPI_HALFDUPLEX_MASTER){
		gpio_set_alternatefunction(_conf->clkport, _conf->clkpin, GPIO_ALTERNATE_PUSHPULL);
		gpio_set_alternatefunction(_conf->mosiport, _conf->mosipin, GPIO_ALTERNATE_PUSHPULL);
		if(_conf->mode == SPI_FULLDUPLEX_MASTER) gpio_set_mode(_conf->misoport, _conf->misopin, GPIO_INPUT);
		if(_conf->nss == SPI_HARDWARE_NSS){
			if(_conf->nss_dir == SPI_NSS_INPUT) gpio_set_mode(_conf->nssport, _conf->nsspin, GPIO_INPUT);
			else gpio_set_alternatefunction(_conf->nssport, _conf->nsspin, GPIO_ALTERNATE_PUSHPULL);
		}
	}
	else{
		gpio_set_alternatefunction(_conf->misoport, _conf->misopin, GPIO_ALTERNATE_PUSHPULL);
		gpio_set_mode(_conf->clkport, _conf->clkpin, GPIO_INPUT);
		if(_conf->mode == SPI_FULLDUPLEX_SLAVE) gpio_set_mode(_conf->mosiport, _conf->mosipin, GPIO_INPUT);
		if(_conf->nss == SPI_HARDWARE_NSS) gpio_set_mode(_conf->nssport, _conf->nsspin, GPIO_INPUT);
	}

#elif defined(STM32F4)
/* Configuration CLK Pin */
	gpio_port_clock_enable(_conf -> clkport);
#if defined(SPI4) && defined(SPI5) && defined(SPI6)
	gpio_set_alternatefunction(_conf -> clkport, _conf -> clkpin, AF5_SPI1_6);
#elif defined(SPI1) || defined(SPI2)
	if(_spi == SPI1 || _spi == SPI2){
		gpio_set_alternatefunction(_conf -> clkport, _conf -> clkpin, AF5_SPI1_2);
	}
	else{
		gpio_set_alternatefunction(_conf -> clkport, _conf -> clkpin, AF6_SPI3);
	}
	gpio_set_alternatefunction_type(_conf -> clkport, _conf -> clkpin, GPIO_OUTPUT_PUSHPULL);
#endif /* defined(SPI4) && defined(SPI5) defined(SPI6) */
	gpio_set_alternatefunction_type(_conf -> clkport, _conf -> clkpin, GPIO_OUTPUT_PUSHPULL);

/* Configuration MISO Pin */
	if(_conf -> mode & (SPI_FULLDUPLEX_MASTER | SPI_FULLDUPLEX_SLAVE | SPI_HALFDUPLEX_SLAVE)){
		gpio_port_clock_enable(_conf -> misoport);
#if defined(SPI4) && defined(SPI5) && defined(SPI6)
		gpio_set_alternatefunction(_conf -> misoport, _conf -> misopin, AF5_SPI1_6);
#elif defined(SPI1) || defined(SPI2)
		if(_spi == SPI1 || _spi == SPI2){
			gpio_set_alternatefunction(_conf -> misoport, _conf -> misopin, AF5_SPI1_2);
		}
		else{
			gpio_set_alternatefunction(_conf -> misoport, _conf -> misopin, AF6_SPI3);
		}
#endif /* defined(SPI4) && defined(SPI5) defined(SPI6) */
		gpio_set_alternatefunction_type(_conf -> misoport, _conf -> misopin, GPIO_OUTPUT_PUSHPULL);
	}

/* Configuration MOSI Pin */
	if(_conf -> mode & (SPI_FULLDUPLEX_MASTER | SPI_HALFDUPLEX_MASTER | SPI_FULLDUPLEX_SLAVE)){
		gpio_port_clock_enable(_conf -> mosiport);
#if defined(SPI4) && defined(SPI5) && defined(SPI6)
		gpio_set_alternatefunction(_conf -> mosiport, _conf -> mosipin, AF5_SPI1_6);
#elif defined(SPI1) || defined(SPI2)
		if(_spi == SPI1 || _spi == SPI2){
			gpio_set_alternatefunction(_conf -> mosiport, _conf -> mosipin, AF5_SPI1_2);
		}
		else{
			gpio_set_alternatefunction(_conf -> mosiport, _conf -> mosipin, AF6_SPI3);
		}
#endif /* defined(SPI4) && defined(SPI5) defined(SPI6) */
		gpio_set_alternatefunction_type(_conf -> mosiport, _conf -> mosipin, GPIO_OUTPUT_PUSHPULL);
	}

/* Configuration NSS Pin */
	if(_conf -> nss == SPI_HARDWARE_NSS){
		gpio_port_clock_enable(_conf -> nssport);
#if defined(SPI4) && defined(SPI5) && defined(SPI6)
		gpio_set_alternatefunction(_conf -> nssport, _conf -> nsspin, AF5_SPI1_6);
#elif defined(SPI1) || defined(SPI2)
		if(_spi == SPI1 || _spi == SPI2){
			gpio_set_alternatefunction(_conf -> nssport, _conf -> nsspin, AF5_SPI1_2);
		}
		else{
			gpio_set_alternatefunction(_conf -> nssport, _conf -> nsspin, AF6_SPI3);
		}
#endif /* defined(SPI4) && defined(SPI5) defined(SPI6) */
		gpio_set_alternatefunction_type(_conf -> nssport, _conf -> nsspin, GPIO_OUTPUT_PUSHPULL);
	}
#endif /* STM32F4 */

	_spi -> CR1 = 0x00U;

	_spi -> CR1 |= (_conf->clocksample << SPI_CR1_CPHA_Pos) | (_conf->clockdivision << SPI_CR1_BR_Pos);
	_spi -> CR1 |= (_conf->datasize << SPI_CR1_DFF_Pos) | (_conf->bitordering << SPI_CR1_LSBFIRST_Pos);
	if(_conf -> mode & (SPI_FULLDUPLEX_MASTER | SPI_HALFDUPLEX_MASTER)) _spi -> CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI;
	if(_conf -> mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE)) _spi -> CR1 |= SPI_CR1_BIDIMODE;
	if(_conf -> nss == SPI_SOFTWARE_NSS) _spi -> CR1 |= SPI_CR1_SSM;

	_spi -> CR2 = 0x00U;
	if((_conf -> mode & (SPI_FULLDUPLEX_MASTER | SPI_HALFDUPLEX_MASTER)) && (_conf -> nss == SPI_HARDWARE_NSS)) _spi -> CR2 |= SPI_CR2_SSOE;

	if(_conf->control & (SPI_INTERRUPT_CONTROL | SPI_INTERRUPT_DMA_CONTROL)){
#if defined(SPI1)
		if     (_spi == SPI1) IRQn = SPI1_IRQn;
#endif /* defined(SPI1) */
#if defined(SPI2)
		else if(_spi == SPI2) IRQn = SPI2_IRQn;
#endif /* defined(SPI2) */
#if defined(SPI3)
		else if(_spi == SPI3) IRQn = SPI3_IRQn;
#endif /* defined(SPI3) */
#if defined(SPI4)
		else if(_spi == SPI4) IRQn = SPI4_IRQn;
#endif /* defined(SPI4) */
#if defined(SPI5)
		else if(_spi == SPI5) IRQn = SPI5_IRQn;
#endif /* defined(SPI6) */
#if defined(SPI6)
		else if(_spi == SPI6) IRQn = SPI6_IRQn;
#endif /* defined(SPI4) */

		if(_conf -> interruptpriority < CONFIG_RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
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
		__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	}

	return ret;
}

stm_ret_t spi::deinit(void){
#if ENABLE_DMA
	_txdma = NULL;
	_rxdma = NULL;
#endif /* ENABLE_DMA */
	gpio_deinit(_conf -> clkport, _conf -> clkpin);
	if(_conf -> mode & (SPI_FULLDUPLEX_MASTER | SPI_FULLDUPLEX_SLAVE | SPI_HALFDUPLEX_SLAVE))
		gpio_deinit(_conf -> misoport, _conf -> misopin);
	if(_conf -> mode & (SPI_FULLDUPLEX_MASTER | SPI_HALFDUPLEX_MASTER | SPI_FULLDUPLEX_SLAVE))
		gpio_deinit(_conf -> mosiport, _conf -> mosipin);
	if(_conf -> nss == SPI_HARDWARE_NSS)
		gpio_deinit(_conf -> nssport, _conf -> nsspin);

#if defined(SPI1)
	if     (_spi == SPI1) RCC -> APB2ENR &=~ RCC_APB2ENR_SPI1EN;
#endif /* defined(SPI1) */
#if defined(SPI2)
	else if(_spi == SPI2) RCC -> APB1ENR &=~ RCC_APB1ENR_SPI2EN;
#endif /* defined(SPI2) */
#if defined(SPI3)
	else if(_spi == SPI3) RCC -> APB1ENR &=~ RCC_APB1ENR_SPI3EN;
#endif /* defined(SPI3) */
#if defined(SPI4)
	else if(_spi == SPI4) RCC -> APB2ENR &=~ RCC_APB2ENR_SPI4EN;
#endif /* defined(SPI4) */
#if defined(SPI5)
	else if(_spi == SPI5) RCC -> APB2ENR &=~ RCC_APB2ENR_SPI5EN;
#endif /* defined(SPI6) */
#if defined(SPI4)
	else if(_spi == SPI6) RCC -> APB2ENR &=~ RCC_APB2ENR_SPI6EN;
#endif /* defined(SPI4) */

	_spi -> CR1 = 0x00U;
	_spi -> CR2 = 0x00U;
	__IO uint32_t tmp = _spi -> DR;
	tmp = _spi -> SR;
	(void)tmp;

	if(_conf -> control && (SPI_INTERRUPT_CONTROL | SPI_INTERRUPT_DMA_CONTROL)){
		__NVIC_ClearPendingIRQ(IRQn);
		__NVIC_DisableIRQ(IRQn);
	}

	return {STM_OKE, 0};
}

stm_ret_t spi::register_event_handler(void (*function_ptr)(spi_event_t event, void *param), void *param){
	stm_ret_t ret;
	if(_conf -> control && (SPI_INTERRUPT_CONTROL | SPI_INTERRUPT_DMA_CONTROL)) {
		handler_callback = function_ptr;
		parameter = param;

		return ret;
	}
	else{
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_WARN(TAG, "%s -> %s, Peripheral control unsupported register event handler.", __FILE__, __FUNCTION__ );
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
	}

	return ret;
}

spi_config_t *spi::get_config(void){
	return _conf;
}

stm_ret_t spi::transmit(uint32_t data, uint32_t size){
	stm_ret_t ret;

	txcount = 0U;
	txlen = size;
	txbuf = data;

	if(_conf -> mode & SPI_HALFDUPLEX_SLAVE){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Mode half duplex slave unsupported transmit function.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	if(_conf -> mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE)){
		_spi -> CR1 &=~ SPI_CR1_SPE;
		_spi -> CR1 |= SPI_CR1_BIDIOE;
	}

	if(!(_spi -> CR1 & SPI_CR1_SPE)) _spi -> CR1 |= SPI_CR1_SPE;

	while(txcount < txlen){
		ret = wait_flag_in_register_timeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}

		_spi -> DR = *(uint32_t *)(txbuf);

		if(_conf->datasize == SPI_DATASIZE_8BIT) 		txbuf += sizeof(uint8_t);
		else if(_conf->datasize == SPI_DATASIZE_16BIT)  txbuf += sizeof(uint16_t);

		txcount++;
	}

	ret = wait_flag_in_register_timeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return_line(&ret, __LINE__);
		return ret;
	}

	if(!(_conf->mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE))){
		__IO uint32_t tmp = _spi -> DR;
		tmp = _spi -> SR;
		(void)tmp;
	}

	return ret;
}

stm_ret_t spi::receive(uint32_t data, uint32_t size){
	stm_ret_t ret;

	rxcount = 0;
	rxlen = size;
	rxbuf = data;

	if(_conf->mode == SPI_HALFDUPLEX_MASTER) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Mode half duplex master unsupported receive function.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	if(_conf -> mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE)){
		_spi -> CR1 &=~ SPI_CR1_SPE;
		_spi -> CR1 |= SPI_CR1_BIDIOE;
	}

	if(!(_spi -> CR1 & SPI_CR1_SPE)) _spi -> CR1 |= SPI_CR1_SPE;

	while(rxcount < rxlen){
		if(_conf -> mode & SPI_FULLDUPLEX_MASTER){
			ret = wait_flag_in_register_timeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
			if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
				LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
				set_return_line(&ret, __LINE__);
				return ret;
			}
			_spi -> DR = 0x00UL;
		}

		ret = wait_flag_in_register_timeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
		*(uint32_t *)rxbuf = _spi -> DR;

		if(_conf->datasize == SPI_DATASIZE_8BIT) rxbuf += sizeof(uint8_t);
		else if(_conf->datasize == SPI_DATASIZE_16BIT) rxbuf += sizeof(uint16_t);

		rxcount++;
	}

	ret = wait_flag_in_register_timeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return_line(&ret, __LINE__);
		return ret;
	}
	if(!(_conf->mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE))){
		__IO uint32_t tmp = _spi -> DR;
		tmp = _spi -> SR;
		(void)tmp;
	}

	return ret;
}

stm_ret_t spi::transmit_receive(uint32_t txdata, uint32_t rxdata, uint32_t size){
	stm_ret_t ret;

	txcount = 0U;
	rxcount = 0U;
	rxlen = size;
	txlen = size;
	txbuf = txdata;
	rxbuf = rxdata;

	if(_conf->mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Mode half duplex master/slave unsupported transmit and receive function.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	if(!(_spi -> CR1 & SPI_CR1_SPE)) _spi -> CR1 |= SPI_CR1_SPE;

	while(txcount < txlen){
		ret = wait_flag_in_register_timeout(&(_spi -> SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_spi -> DR = *(uint32_t *)txbuf;

		ret = wait_flag_in_register_timeout(&(_spi -> SR), SPI_SR_RXNE, FLAG_SET, SPI_TIMEOUT);
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
		*(uint32_t *)rxbuf = _spi -> DR;

		if(_conf->datasize == SPI_DATASIZE_8BIT) {
			txbuf += sizeof(uint8_t);
			rxbuf += sizeof(uint8_t);
		}
		else if(_conf->datasize == SPI_DATASIZE_16BIT) {
			txbuf += sizeof(uint16_t);
			rxbuf += sizeof(uint16_t);
		}

		txcount++;
		rxcount++;
	}

	ret = wait_flag_in_register_timeout(&(_spi -> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
	if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return_line(&ret, __LINE__);
		return ret;
	}
	if(!(_conf->mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE))){
		__IO uint32_t tmp = _spi -> DR;
		tmp = _spi -> SR;
		(void)tmp;
	}

	return ret;
}


stm_ret_t spi::transmit_start_it(uint32_t data, uint32_t size){
	stm_ret_t ret;

	txcount = 0U;
	txlen = size;
	txbuf = data;

	if(_conf -> mode & SPI_HALFDUPLEX_SLAVE){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Mode half duplex slave unsupported transmit function.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	if(_conf -> mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE)){
		_spi -> CR1 &=~ SPI_CR1_SPE;
		_spi -> CR1 |= SPI_CR1_BIDIOE;
	}

	_spi -> CR2 |= SPI_CR2_TXEIE | SPI_CR2_ERRIE;

	if(!(_spi -> CR1 & SPI_CR1_SPE)) _spi -> CR1 |= SPI_CR1_SPE;

	return ret;
}

stm_ret_t spi::receive_start_it(uint32_t data, uint32_t size){
	stm_ret_t ret;

	rxcount = 0U;
	rxlen = size;
	rxbuf = data;

	if(_conf -> mode == SPI_FULLDUPLEX_MASTER){
		return transmit_receive_start_it(rxbuf, rxbuf, rxlen);
	}

	if(_conf -> mode & SPI_HALFDUPLEX_MASTER){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Mode half duplex master unsupported receive function.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	if(_conf -> mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE)){
		_spi -> CR1 &=~ SPI_CR1_SPE;
		_spi -> CR1 |= SPI_CR1_BIDIOE;
	}

	_spi -> CR2 |= SPI_CR2_RXNEIE | SPI_CR2_ERRIE;

	if((_spi -> CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) _spi -> CR1 |= SPI_CR1_SPE;

	return ret;
}

stm_ret_t spi::transmit_receive_start_it(uint32_t txdata, uint32_t rxdata, uint32_t size){
	stm_ret_t ret;

	txcount = 0U;
	rxcount = 0U;
	rxlen = size;
	txlen = size;
	txbuf = txdata;
	rxbuf = rxdata;

	if(_conf -> mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE)){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Mode half duplex master/slave unsupported transmit function.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	_spi -> CR2 |= SPI_CR2_RXNEIE | SPI_CR2_TXEIE | SPI_CR2_ERRIE;

	if((_spi -> CR1 & SPI_CR1_SPE) != SPI_CR1_SPE) _spi -> CR1 |= SPI_CR1_SPE;

	return ret;
}


#if ENABLE_DMA
stm_ret_t spi::transmit_start_dma(uint32_t data, uint32_t size){
	stm_ret_t ret;
	txcount = 0U;
	txlen = size;
	txbuf = data;

	if((_conf->mode & SPI_HALFDUPLEX_SLAVE) || (_txdma == NULL)){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Mode half duplex slave unsupported transmit function or not selected dma.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	dma_config_t *_dma_conf = _txdma -> get_config();
	if((uint8_t)_dma_conf -> datasize != (uint8_t)_conf -> datasize){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Data size not match with dma.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}

	_spi -> CR1 &=~ SPI_CR1_SPE;
	if(_conf -> mode == SPI_HALFDUPLEX_MASTER) _spi -> CR1 |= SPI_CR1_BIDIOE;

	_spi -> CR2 &=~ SPI_CR2_TXDMAEN;

	ret = _txdma -> start(txbuf, (uint32_t) &(_spi -> DR), txlen);
	if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Transmit dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return_line(&ret, __LINE__);
		return ret;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_TXDMAEN;

	return ret;
}

stm_ret_t spi::receive_start_dma(uint32_t data, uint32_t size){
	stm_ret_t ret;

	txcount = 0U;
	txlen = size;
	txbuf = data;
	rxcount = 0U;
	rxlen = size;
	rxbuf = data;


	if((_conf->mode & SPI_HALFDUPLEX_MASTER) || (_rxdma == NULL)){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Mode half duplex master unsupported receive function or not selected dma.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	if(_conf -> mode & (SPI_FULLDUPLEX_MASTER)){
		return transmit_receive_start_dma(txbuf, rxbuf, rxlen);
	}

	dma_config_t *_dma_conf = _txdma -> get_config();
	if((uint8_t)_dma_conf -> datasize != (uint8_t)_conf -> datasize){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Data size not match with dma.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ SPI_CR2_RXDMAEN;

	ret = _rxdma -> start((uint32_t) &(_spi -> DR), rxbuf, rxlen);
	if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Receive dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return_line(&ret, __LINE__);
		return ret;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_RXDMAEN;

	return ret;
}

stm_ret_t spi::transmit_receive_start_dma(uint32_t txdata, uint32_t rxdata, uint32_t size){
	stm_ret_t ret;

	txcount = 0U;
	txlen = size;
	txbuf = txdata;
	rxcount = 0U;
	rxlen = size;
	rxbuf = rxdata;

	if((_conf->mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE)) || (_rxdma == NULL || _rxdma == NULL)){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Mode half duplex master/slave unsupported receive function or not selected dma.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	dma_config_t *_txdma_conf = _txdma -> get_config();
	dma_config_t *_rxdma_conf = _rxdma -> get_config();
	if(((uint8_t)_txdma_conf -> datasize != (uint8_t)_conf -> datasize) || ((uint8_t)_rxdma_conf -> datasize != (uint8_t)_conf -> datasize)){
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Data size not match with dma.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}

	_spi -> CR1 &=~ SPI_CR1_SPE;
	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

	ret = _rxdma -> start((uint32_t) &(_spi -> DR), (uint32_t)rxbuf, rxlen);
	if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Receive dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return_line(&ret, __LINE__);
		return ret;
	}


	ret = _txdma -> start((uint32_t)txbuf, (uint32_t) &(_spi -> DR), txlen);
	if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Transmit dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
		set_return_line(&ret, __LINE__);
		return ret;
	}

	_spi -> CR1 |= SPI_CR1_SPE;
	_spi -> CR2 |= SPI_CR2_ERRIE;
	_spi -> CR2 |= SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;

	return ret;
}

stm_ret_t spi::transmit_stop_dma(void){
	stm_ret_t ret;

	txcount = 0U;
	txlen = 0U;
	txbuf = 0U;

	if(_spi -> CR2 & SPI_CR2_TXDMAEN) {
		ret = _txdma -> stop();
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Transmit dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_spi -> CR2 &=~ SPI_CR2_TXDMAEN;
		_spi -> CR1 &=~ SPI_CR1_SPE;
	}
	else{
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Transmit dma not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
	}

	return ret;
}

stm_ret_t spi::receive_stop_dma(void){
	stm_ret_t ret;

	rxcount = 0U;
	rxlen = 0U;
	rxbuf = 0U;

	if(_spi -> CR2 & SPI_CR2_RXDMAEN) {
		ret = _rxdma -> stop();
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Receive dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_spi -> CR2 &=~ SPI_CR2_RXDMAEN;
		_spi -> CR1 &=~ SPI_CR1_SPE;
	}
	else{
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
		LOG_ERROR(TAG, "%s -> %s -> Receive dma not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
	}

	return ret;
}

stm_ret_t spi::transmit_receive_stop_dma(void){
	stm_ret_t ret;

	txcount = 0U;
	txlen = 0U;
	txbuf = 0U;
	rxcount = 0U;
	rxlen = 0U;
	rxbuf = 0U;

	if(_spi -> CR2 & SPI_CR2_RXDMAEN) {
		ret = _rxdma -> stop();
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Receive dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}

	if(_spi -> CR2 & SPI_CR2_TXDMAEN) {
		ret = _txdma -> stop();
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Transmit dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}

	_spi -> CR2 &=~ (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);
	_spi -> CR1 &=~ SPI_CR1_SPE;

	return ret;
}
#endif /* ENABLE_DMA */



stm_ret_t SPI_Tx_ISR(spi *spi){
	stm_ret_t ret;

	spi -> _spi -> DR = *(uint32_t *)(spi -> txbuf);
	spi -> txcount++;

	if(spi -> _conf -> datasize ==  SPI_DATASIZE_8BIT)
		spi -> txbuf += sizeof(uint8_t);
	else if(spi -> _conf -> datasize ==  SPI_DATASIZE_16BIT)
		spi -> txbuf += sizeof(uint16_t);

	if(spi -> txcount == spi -> txlen){
		stm_ret_t ret = wait_flag_in_register_timeout(&(spi->_spi->SR), SPI_SR_TXE, FLAG_SET, SPI_TIMEOUT);
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			return ret;
		}

		spi -> _spi -> CR2 &=~ (SPI_CR2_TXEIE | SPI_CR2_ERRIE);

		ret = wait_flag_in_register_timeout(&(spi->_spi-> SR), SPI_SR_BSY, FLAG_RESET, SPI_TIMEOUT);
		if(spi -> _conf -> mode & (SPI_FULLDUPLEX_MASTER | SPI_HALFDUPLEX_MASTER)){
			if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
				LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
				set_return_line(&ret, __LINE__);

				spi -> _spi -> CR2 &=~ (SPI_CR2_TXEIE | SPI_SR_RXNE | SPI_CR2_ERRIE);
				if(spi -> _conf -> mode & (SPI_HALFDUPLEX_MASTER)) spi -> _spi -> CR1 &=~ SPI_CR1_SPE;

				return ret;
			}
		}

		if(!(spi->_conf->mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE))){
			__IO uint32_t tmp = spi -> _spi -> DR;
			tmp = spi -> _spi -> SR;
			(void)tmp;
		}
	}

	return ret;
}

stm_ret_t SPI_Rx_ISR(spi *spi){
	stm_ret_t ret;

	*(uint32_t *)(spi -> rxbuf) = spi -> _spi -> DR ;
	spi -> rxcount++;
	if(spi -> _conf -> datasize ==  SPI_DATASIZE_8BIT)
		spi -> rxbuf += sizeof(uint8_t);
	else if(spi -> _conf -> datasize ==  SPI_DATASIZE_16BIT)
		spi -> rxbuf += sizeof(uint16_t);

	if(spi -> rxcount == spi -> rxlen){
		spi -> _spi -> CR2 &=~ (SPI_CR2_RXNEIE | SPI_CR2_ERRIE);

		stm_ret_t ret = wait_flag_in_register_timeout(&(spi->_spi->SR), SPI_SR_RXNE, FLAG_RESET, SPI_TIMEOUT);
		if(!is_oke(&ret)) {
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
			LOG_ERROR(TAG, "%s -> %s -> Wait flag timeout.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}

		if(!(spi->_conf->mode & (SPI_HALFDUPLEX_MASTER | SPI_HALFDUPLEX_SLAVE))){
			__IO uint32_t tmp = spi -> _spi -> DR;
			tmp = spi -> _spi -> SR;
			(void)tmp;
		}
	}

	return ret;
}

void SPI_IRQHandler(spi *spi){
	spi_event_t event = SPI_EVENT_NOEVENT;
	__IO uint32_t cr2_reg = spi -> _spi -> CR2;
	__IO uint32_t sr_reg  = spi -> _spi -> SR;

	if((cr2_reg & SPI_CR2_TXEIE) && (sr_reg & SPI_SR_TXE)){
		stm_ret_t ret = SPI_Tx_ISR(spi);
		if(!is_oke(&ret)){
			event = SPI_EVENT_ERROR;
			goto EventCB;
		}
		event = SPI_EVENT_TRANSMIT_COMPLETE;
		goto EventCB;
	}
	if((cr2_reg & SPI_CR2_RXNEIE) && (sr_reg & SPI_SR_RXNE)){
		stm_ret_t ret = SPI_Rx_ISR(spi);
		if(!is_oke(&ret)){
			event = SPI_EVENT_ERROR;
			goto EventCB;
		}
		event = SPI_EVENT_RECEIVE_COMPLETE;
		goto EventCB;
	}
	if((sr_reg & SPI_SR_OVR)
#if defined(STM32F4)
			|| (sr_reg & SPI_SR_FRE)
#endif /* STM32F4 */
	){
		event = SPI_EVENT_ERROR;
		__IO uint32_t tmp = spi -> _spi -> DR;
		tmp = spi -> _spi -> SR;
		(void)tmp;
		goto EventCB;
	}


	EventCB:
	if(spi -> handler_callback != NULL) spi -> handler_callback(event, spi -> parameter);
}


#if defined(SPI1)
spi spi_1(SPI1);
spi_t spi1 = &spi_1;
void SPI1_IRQHandler(void){
	SPI_IRQHandler(&spi_1);
}
#endif /* defined(SPI1) */
#if defined(SPI2)
spi spi_2(SPI2);
spi_t spi2 = &spi_2;
void SPI2_IRQHandler(void){
	SPI_IRQHandler(&spi_2);
}
#endif /* defined(SPI2) */
#if defined(SPI3)
spi spi_3(SPI3);
spi_t spi3 = &spi_3;
void SPI3_IRQHandler(void){
	SPI_IRQHandler(&spi_3);
}
#endif /* defined(SPI3) */
#if defined(SPI4)
spi spi_4(SPI4);
spi_t spi4 = &spi_4;
void SPI4_IRQHandler(void){
	SPI_IRQHandler(&spi_4);
}
#endif /* defined(SPI4) */
#if defined(SPI5)
spi spi_5(SPI5);
spi_t spi5 = &spi_5;
void SPI5_IRQHandler(void){
	SPI_IRQHandler(&spi_5);
}
#endif /* defined(SPI5) */
#if defined(SPI6)
spi spi_6(SPI6);
spi_t spi6 = &spi_6;
void SPI6_IRQHandler(void){
	SPI_IRQHandler(&spi_6);
}
#endif /* defined(SPI6) */


#endif /* ENABLE_SPI */
