/*
 * i2c.cpp
 *
 *  Created on: Mar 21, 2023
 *      Author: anh
 */

#include "peripheral_config.h"
#if ENABLE_I2C

#include "periph/i2c.h"
#include "periph/systick.h"
#include "periph/rcc.h"
#include "periph/gpio.h"
#if CONFIG_USE_LOG_MONITOR && SPI_LOG
#include "system/log.h"

static const char *TAG = "I2C";
#endif /* CONFIG_USE_LOG_MONITOR && SPI_LOG */


#define I2C_MIN_FREQ_STANDARD       2000000U // Minimum is 2MHz of APB1 bus in standard mode.
#define I2C_MIN_FREQ_FAST           4000000U // Minimum is 4MHz of APB1 bus in standard mode.

void I2C_IRQHandler(i2c *i2c);


i2c::i2c(I2C_TypeDef *i2c){
	_i2c = i2c;
}

stm_ret_t i2c::init(i2c_config_t *conf){
	stm_ret_t ret;

	_conf = conf;
#if ENABLE_DMA
	_txdma = _conf -> txdma;
	_rxdma = _conf -> rxdma;
#endif /* ENABLE_DMA */

	if(_conf -> mode == I2C_STANDARD_MODE && _conf -> frequency > 100000U) {
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}

	/* ***********************I2C CLK ENABLE*********************** */
#if defined(I2C1)
	if(_i2c == I2C1)      RCC -> APB1ENR |= RCC_APB1ENR_I2C1EN;
#endif /* defined(I2C1) */
#if defined(I2C2)
	else if(_i2c == I2C2) RCC -> APB1ENR |= RCC_APB1ENR_I2C2EN;
#endif /* defined(I2C2) */
#if defined(I2C3)
	else if(_i2c == I2C3) RCC -> APB1ENR |= RCC_APB1ENR_I2C3EN;
#endif /* defined(I2C3) */
#if defined(I2C4)
	else if(_i2c == I2C4) RCC -> APB1ENR |= RCC_APB1ENR_I2C4EN;
#endif /* defined(I2C4) */

	/* ***********************GPIO INIT********************** */
#if defined(STM32F1)
	gpio_set_alternatefunction(_conf -> sclport, _conf -> sclpin, GPIO_ALTERNATE_OPENDRAIN);
	gpio_set_alternatefunction(_conf -> sdaport, _conf -> sdapin, GPIO_ALTERNATE_OPENDRAIN);
	gpio_set_pullup(_conf -> sclport, _conf -> sclpin);
	gpio_set_pullup(_conf -> sdaport, _conf -> sdapin);
	if(_conf ->gpio_remap && _i2c == I2C1) gpio_remap(I2C1_Remap);
#elif defined(STM32F4)
	gpio_set_alternatefunction(_conf -> sclport, _conf -> sclpin, AF4_I2C1_3);
	gpio_set_alternatefunction_type(_conf -> sclport, _conf -> sclpin, GPIO_OUTPUT_OPENDRAIN);
	gpio_set_alternatefunction(_conf -> sdaport, _conf -> sdapin, AF4_I2C1_3);
	gpio_set_alternatefunction_type(_conf -> sdaport, _conf -> sdapin, GPIO_OUTPUT_OPENDRAIN);
#endif /* STM32F4 */
	/* ***********************DISABLE I2C********************** */
	_i2c -> CR1 &=~ I2C_CR1_PE;
	/* ***********************RESET I2C********************** */
	_i2c -> CR1 |= I2C_CR1_SWRST;
	delay_ms(5);
	_i2c -> CR1 &= ~I2C_CR1_SWRST;
	/* ***********************CHECK MINIMUM ALLOWED FREQUENCY********************** */
	uint32_t apb1_freq = rcc_get_bus_frequency(APB1);
	if(((_conf -> frequency <= 100000U)? (apb1_freq < I2C_MIN_FREQ_STANDARD) : (apb1_freq < I2C_MIN_FREQ_FAST))) {
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}
	/* ***********************SET ABP1 FREQUENCY TO CR2 REGISTER********************** */
	_i2c -> CR2 |= (uint32_t)((apb1_freq / 1000000U) >> I2C_CR2_FREQ_Pos);
	/* ***********************SET SCL RISE TIME********************** */
	_i2c -> TRISE = (uint32_t)((_conf -> mode == I2C_STANDARD_MODE)? ((apb1_freq/1000000U) + 1U) : ((((apb1_freq/1000000U) * 300U) / 1000U) + 1U));
	/* ***********************SET I2C SPEED********************** */
	if(_conf -> mode == I2C_STANDARD_MODE){
		_i2c -> CCR &=~ I2C_CCR_FS;
		// T_high = T_low = CCR * T_PCLK1, T = 2 * T_high.
		uint32_t ccr_tmp = (uint32_t)((apb1_freq - 1U)/(2U * _conf -> frequency) + 1U);
		if(ccr_tmp < 4U) ccr_tmp = 4U;
		_i2c -> CCR |= ccr_tmp;
	}
	else if(_conf -> mode == I2C_FAST_MODE){
		_i2c -> CCR |= I2C_CCR_FS;
		if(_conf -> clockduty == I2C_CLOCKDUTY_2){
			_i2c -> CCR &=~ I2C_CCR_DUTY;
			// T_high = (1/2)T_low = CCR * TPCLK1, T = 3 * T_high.
			uint32_t ccr_tmp = (uint32_t)((apb1_freq)/(3U * _conf -> frequency));
			if(ccr_tmp < 1U) ccr_tmp = 1U;
			_i2c -> CCR |= ccr_tmp;

		}
		else if(_conf -> clockduty == I2C_CLOCKDUTY_16_9){
			_i2c -> CCR |= I2C_CCR_DUTY;
			// T_high = (9/16)T_low = CCR * TPCLK1, T = 25 * T_high.
			uint32_t ccr_tmp = (uint32_t)((apb1_freq)/(25U * _conf -> frequency));
			if(ccr_tmp < 4U) ccr_tmp = 4U;
			_i2c -> CCR |= ccr_tmp;
		}
	}
	/* ***********************ENABLE I2C PERIPHERAL********************** */
	_i2c -> CR1 |= I2C_CR1_PE;
	/* ***********************CLEAR ALL FLAG********************** */
	_i2c -> SR1 = 0UL;
	_i2c -> SR2 = 0UL;

	return ret;
}

void i2c::ACKError_Action(void){
	_i2c -> CR1 |= I2C_CR1_STOP;
	_i2c -> SR1 = ~(I2C_SR1_AF & 0xFFFF);
}

void i2c::ClearADDR(void){
	__IO uint32_t tmp = _i2c -> SR1 | _i2c -> SR2;
	(void)tmp;
}

stm_ret_t i2c::WaitBusy(void){
	stm_ret_t ret;
	set_return(&ret, STM_BUSY, 0U);

	/* ***********************CHECK BUSY FLAG********************** */
	ret = wait_flag_in_register_timeout(&(_i2c -> SR2), I2C_SR2_BUSY, FLAG_RESET, I2C_BUSY_TIMEOUT);
	if(is_oke(&ret)){
		set_return(&ret, STM_READY, __LINE__);
		return ret;
	}

	set_return(&ret, STM_BUSY, __LINE__);

	return ret;
}

stm_ret_t i2c::send_start(void){
	stm_ret_t ret;

	/* ***********************WAIT I2C NOT BUSY********************** */
	ret = WaitBusy();
	if(!is_ready(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	/* ***********************DISABLE POS********************** */
	_i2c -> CR1 &=~ I2C_CR1_POS;
	/* ***********************GENERATE START********************** */
	_i2c -> CR1 |= I2C_CR1_START;
	/* ***********************WAIT SB FLAG IS SET********************** */
	ret = wait_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_TIMEOUT);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	return ret;
}

stm_ret_t i2c::send_repeatstart(void){
	stm_ret_t ret;

	/* ***********************GENERATE START********************** */
	_i2c -> CR1 |= I2C_CR1_START;
	/* ***********************WAIT SB FLAG IS SET********************** */
	ret = wait_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_SB, FLAG_SET, I2C_TIMEOUT);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	return ret;
}

stm_ret_t i2c::send_slaveaddress(uint16_t slaveaddress, i2c_action_t action){
	stm_ret_t ret;

	/* ***********************SEND SLAVE ADDRESS*********************** */
	if(_conf -> addresssize == I2C_ADDRESSSIZE_7BIT){
		if(action == I2C_WRITE)_i2c -> DR = (uint8_t)((slaveaddress & 0x00FF) & ~I2C_OAR1_ADD0); // SEND 8BIT SLAVE ADDRESS WITH LSB BIT IS RESET(TRANSMIT MODE).
		else                   _i2c -> DR = (uint8_t)((slaveaddress & 0x00FF) |  I2C_OAR1_ADD0); // SEND 8BIT SLAVE ADDRESS WITH LSB BIT IS SET(RECEIVE MODE).
	}
	/* ***********************WAIT ADDR FLAG IS SET*********************** */
	ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_ADDR, FLAG_SET, I2C_TIMEOUT);
	if(!is_oke(&ret)){
		if(is_err(&ret))
			ACKError_Action();
		set_return_line(&ret, __LINE__);
		return ret;
	}
	/* ***********************CLEAR ADDR FLAG*********************** */
	ClearADDR();

	return ret;
}

stm_ret_t i2c::send_start_slaveaddress(uint16_t slaveaddress, i2c_action_t action){
	stm_ret_t ret;

	/* ************* SEND START CONDITION **************** */
	ret = send_start();
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	/* ************* SEND SLAVE ADDRESS **************** */
	ret = send_slaveaddress(slaveaddress, action);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	return ret;
}

stm_ret_t i2c::send_repeatstart_slaveaddress(uint16_t slaveaddress, i2c_action_t action){
	stm_ret_t ret;

	/* ************* SEND REPEAT START CONDITION **************** */
	ret = send_repeatstart();
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	/* ************* SEND SLAVE ADDRESS **************** */
	ret = send_slaveaddress(slaveaddress, action);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	return ret;
}

stm_ret_t i2c::check_device(uint16_t slaveaddress, uint8_t trials, uint16_t timeout){
	stm_ret_t ret;

	uint8_t I2C_Trials = 1U;
	__IO uint32_t tick = get_tick();

	ret = WaitBusy();
	if(!is_ready(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}

	_i2c -> CR1 |= I2C_CR1_PE;
	_i2c -> CR1 &=~ I2C_CR1_POS;

	do{
		ret = send_start();
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}

		_i2c -> DR = (uint8_t)((slaveaddress & 0x00FF) & ~I2C_OAR1_ADD0);

		tick = get_tick();
		while(!(_i2c -> SR1 & I2C_SR1_ADDR) && !(_i2c -> SR1 & I2C_SR1_AF)){
			if(get_tick() - tick > timeout){
				break;
			}
		}
		if(_i2c -> SR1 & I2C_SR1_ADDR){
			_i2c -> CR1 |= I2C_CR1_STOP;
			_i2c -> SR1 &=~ I2C_SR1_ADDR;
			while(_i2c -> SR2 & I2C_SR2_BUSY){
				if(get_tick() - tick > timeout){
					set_return(&ret, STM_ERR, __LINE__);
					return ret;
				}
			}
			set_return(&ret, STM_OKE, __LINE__);
			return ret;
		}
		else{
			_i2c -> CR1 |= I2C_CR1_STOP;
			_i2c -> SR1 &=~ I2C_SR1_AF;
			while(_i2c -> SR2 & I2C_SR2_BUSY){
				if(get_tick() - tick > timeout){
					set_return(&ret, STM_ERR, __LINE__);
					return ret;
				}
			}
		}
		I2C_Trials++;
	} while(I2C_Trials < trials);

	set_return(&ret, STM_ERR, __LINE__);

	return ret;
}

stm_ret_t i2c::transmit(uint8_t *data, uint16_t size){
	stm_ret_t ret;
	uint32_t TxCount = size;

	/* ***********************WAIT TXE FLAG IS SET*********************** */
	ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_TIMEOUT);
	if(!is_oke(&ret)){
		if(is_err(&ret))
			ACKError_Action();
		set_return_line(&ret, __LINE__);
		return ret;
	}
	/* ***********************TRANSMIT DATA*********************** */
	while(TxCount > 0U){
		/* ***********************WAIT TXE FLAG IS SET*********************** */
		ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_TIMEOUT);
		if(!is_oke(&ret)){
			if(is_err(&ret))
				ACKError_Action();
			set_return_line(&ret, __LINE__);
			return ret;
		}
		/* ***********************WRITE DATA TO DR*********************** */
		_i2c -> DR = *data++;
		TxCount--;
		if((_i2c -> SR1 & I2C_SR1_BTF) && TxCount != 0U){
			/* ***********************WRITE DATA TO DR*********************** */
			_i2c -> DR = *data++;
			TxCount--;
		}
	}
	/* ***********************WAIT BTF FLAG IS SET*********************** */
	ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_TIMEOUT);
	if(!is_oke(&ret)){
		if(is_err(&ret))
			ACKError_Action();
		set_return_line(&ret, __LINE__);
		return ret;
	}

	return ret;
}

stm_ret_t i2c::transmit(uint8_t data){
	stm_ret_t ret;
	/* ***********************WAIT TXE FLAG IS SET*********************** */
	ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_TXE, FLAG_SET, I2C_TIMEOUT);
	if(!is_oke(&ret)){
		if(is_err(&ret))
			ACKError_Action();
		set_return_line(&ret, __LINE__);
		return ret;
	}
	/* ***********************WRITE DATA TO DR*********************** */
	_i2c -> DR = data;
	/* ***********************WAIT BTF FLAG IS SET*********************** */
	ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_TIMEOUT);
	if(!is_oke(&ret)){
		if(is_err(&ret))
			ACKError_Action();
		set_return_line(&ret, __LINE__);
		return ret;
	}

	return ret;
}

stm_ret_t i2c::receive(uint8_t *data, uint16_t size){
	stm_ret_t ret;
	uint32_t RxCount = size;

	/* ********************START READ DATA****************** */
	// SETUP BEFORE READ DATA.
	if(RxCount == 0U){
		// CLEAR ADDR FLAG.
		ClearADDR();
		// GENERATE STOP.
		_i2c -> CR1 |= I2C_CR1_STOP;
	}
	else if(RxCount == 1U){
		// DISABLE ACK.
		_i2c -> CR1 &=~ I2C_CR1_ACK;
		// CLEAR ADDR FLAG.
		ClearADDR();
		// GENERATE STOP.
		_i2c -> CR1 |= I2C_CR1_STOP;
	}
	else if(RxCount == 2U){
		// ENABLE POS.
		_i2c -> CR1 |= I2C_CR1_POS;
		// CLEAR ADDR FLAG.
		ClearADDR();
		// DISABLE ACK.
		_i2c -> CR1 &=~ I2C_CR1_ACK;
	}
	else{
		// ENABLE ACK.
		_i2c -> CR1 |= I2C_CR1_ACK;
		// CLEAR ADDR FLAG.
		ClearADDR();
	}
	// START READ DATA.
	while(RxCount > 0U){
		if(RxCount <= 3U){
			if(RxCount == 1U){
				// WAIT RXNE IS SET.
				ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_TIMEOUT);
				if(!is_oke(&ret)){
					if(is_err(&ret)) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
					set_return_line(&ret, __LINE__);
					return ret;
				}
				// READ FORM DR.
				*data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
			else if(RxCount == 2U){
				// WAIT BTF FLAG IS SET.
				ret = wait_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_TIMEOUT);
				if(ret.Status != STM_OKE) {
					set_return_line(&ret, __LINE__);
					return ret;
				}
				// GENERATE STOP.
				_i2c -> CR1 |= I2C_CR1_STOP;
				// READ FORM DR.
				*data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// READ FORM DR.
				*data++ = (uint8_t)_i2c -> DR;
				RxCount--;

			}
			else{
				// WAIT BTF FLAG IS SET.
				ret = wait_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_TIMEOUT);
				if(ret.Status != STM_OKE) {
					set_return_line(&ret, __LINE__);
					return ret;
				}
				// DISABLE ACK.
				_i2c -> CR1 &=~ I2C_CR1_ACK;
				// READ FORM DR.
				*data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// WAIT BTF FLAG IS SET.
				ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_AF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_BTF, FLAG_SET, I2C_TIMEOUT);
				if(!is_oke(&ret)){
					set_return_line(&ret, __LINE__);
					return ret;
				}
				// GENERATE STOP.
				_i2c -> CR1 |= I2C_CR1_STOP;
				// READ FORM DR.
				*data++ = (uint8_t)_i2c -> DR;
				RxCount--;
				// READ FORM DR.
				*data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
		}
		else{
			// WAIT RXNE IS SET.
			ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_TIMEOUT);
			if(!is_oke(&ret)){
				if(is_err(&ret)) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
				set_return_line(&ret, __LINE__);
				return ret;
			}
			// READ FORM DR.
			*data++ = (uint8_t)_i2c -> DR;
			RxCount--;
			if(_i2c -> SR1 & I2C_SR1_BTF){
				// READ FORM DR.
				*data++ = (uint8_t)_i2c -> DR;
				RxCount--;
			}
		}
	}
	return ret;
}

stm_ret_t i2c::receive(uint8_t *data){
	stm_ret_t ret;

	// DISABLE ACK.
	_i2c -> CR1 &=~ I2C_CR1_ACK;
	// CLEAR ADDR FLAG.
	ClearADDR();
	// GENERATE STOP.
	_i2c -> CR1 |= I2C_CR1_STOP;
	// WAIT RXNE IS SET.
	ret = wait_check_flag_in_register_timeout(&(_i2c -> SR1), I2C_SR1_STOPF, FLAG_SET, &(_i2c -> SR1), I2C_SR1_RXNE, FLAG_SET, I2C_TIMEOUT);
	if(!is_oke(&ret)){
		if(is_err(&ret)) _i2c -> SR1 = ~(I2C_SR1_STOPF & 0xFFFF);
		set_return_line(&ret, __LINE__);
		return ret;
	}
	// READ FORM DR.
	*data = (uint8_t)_i2c -> DR;

	return ret;
}

stm_ret_t i2c::send_stop(void){
	// SEND STOP CONDITION.
	_i2c -> CR1 |= I2C_CR1_STOP;

	return {STM_OKE, 0 };
}

#if ENABLE_DMA
stm_ret_t i2c::transmit_start_dma(uint8_t *data, uint16_t size){
	stm_ret_t ret;

	// SETUP AND START DMA.
	if(_txdma != NULL){
		ret = _txdma -> start((uint32_t)data, (uint32_t)(&_i2c -> DR), size);
		if(!is_oke(&ret)){
#if I2C_LOG
			LOG_ERROR(TAG, "%s -> %s, I2C dma start fail.", __FILE__, __FUNCTION__);
#endif /* I2C_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	else{
#if I2C_LOG
		LOG_ERROR(TAG, "%s -> %s, I2C not set dma.", __FILE__, __FUNCTION__);
#endif /* I2C_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}
	// ENABLE I2C TX DMA
	_i2c -> CR2 |= I2C_CR2_DMAEN;

	return ret;
}

stm_ret_t i2c::receive_start_dma(uint8_t *data, uint16_t size){
	stm_ret_t ret;

	// SETUP AND START DMA.
	if(_rxdma != NULL){
		ret = _rxdma -> start((uint32_t)(&_i2c -> DR), (uint32_t)data, size);
		if(!is_oke(&ret)){
#if I2C_LOG
			LOG_ERROR(TAG, "%s -> %s, I2C dma start fail.", __FILE__, __FUNCTION__);
#endif /* I2C_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	else{
#if I2C_LOG
		LOG_ERROR(TAG, "%s -> %s, I2C not set dma.", __FILE__, __FUNCTION__);
#endif /* I2C_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}

	// RECEIVE ONLY 1 BYTE, NO NEED ACK.
	if(size == 1U) _i2c -> CR1 &=~ I2C_CR1_ACK;
	// IF RECEIVE MORE THAN 1 BYTE, ENABLE DMA CHECK LAST BYTE.
	else _i2c -> CR2 |= I2C_CR2_LAST;
	// ENABLE I2C RX DMA
	_i2c -> CR2 |= I2C_CR2_DMAEN;

	return ret;
}

stm_ret_t i2c::transmit_stop_dma(void){
	stm_ret_t ret;

	if(_i2c -> CR2 & I2C_CR2_DMAEN){
		ret = _txdma -> stop();
		if(!is_oke(&ret)){
#if I2C_LOG
			LOG_ERROR(TAG, "%s -> %s, I2C dma stop fail.", __FILE__, __FUNCTION__);
#endif /* I2C_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_i2c -> CR2 &=~ I2C_CR2_DMAEN;
	}
	else{
#if I2C_LOG
		LOG_ERROR(TAG, "%s -> %s, I2C not set dma.", __FILE__, __FUNCTION__);
#endif /* I2C_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
	}

	return ret;
}
stm_ret_t i2c::receive_stop_dma(void){
	stm_ret_t ret;

	if(_i2c -> CR2 & I2C_CR2_DMAEN){
		ret = _rxdma -> stop();
		if(!is_oke(&ret)){
#if I2C_LOG
			LOG_ERROR(TAG, "%s -> %s, I2C dma stop fail.", __FILE__, __FUNCTION__);
#endif /* I2C_LOG */
			set_return_line(&ret, __LINE__);
			return ret;
		}
		_i2c -> CR2 &=~ I2C_CR2_DMAEN;
	}
	else{
#if I2C_LOG
		LOG_ERROR(TAG, "%s -> %s, I2C not set dma.", __FILE__, __FUNCTION__);
#endif /* I2C_LOG */
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
	}

	return ret;
}
#endif /* ENABLE_DMA */

stm_ret_t i2c::i2c_write_register(uint16_t slaveaddress, uint16_t registeraddress, uint8_t registeraddress_size, uint8_t *data, uint16_t size){
	stm_ret_t ret;

	ret = send_start();
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	ret = send_slaveaddress(slaveaddress, I2C_WRITE);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	if(registeraddress_size == 1U) {
		ret = transmit((uint8_t)(registeraddress & 0x00FF)); // MEMORY ADDRESS LSB.
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	else{
		ret = transmit((uint8_t)((registeraddress & 0x00FF) >> 8U)); // MEMORY ADDretS MSB.
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
		ret = transmit((uint8_t)(registeraddress & 0x00FF));         // MEMORY ADDRESS LSB.
		if(!is_oke(&ret)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	ret = transmit(data, size);
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	send_stop();

	return ret;
}

stm_ret_t i2c::i2c_read_register(uint16_t slaveaddress, uint16_t registeraddress, uint8_t registeraddress_size, uint8_t *data, uint16_t size){
	stm_ret_t ret;

	ret = send_start();
	if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	ret = send_slaveaddress(slaveaddress, I2C_WRITE);
		if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	if(registeraddress_size == 1U) {
		ret = transmit((uint8_t)(registeraddress & 0x00FF)); // MEMORY ADDRESS LSB.
			if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	}
	else{
		ret = transmit((uint8_t)((registeraddress & 0x00FF) >> 8U)); // MEMORY ADDRESS MSB.
			if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
		ret = transmit((uint8_t)(registeraddress & 0x00FF));         // MEMORY ADDRESS LSB.
			if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	}
	ret = send_repeatstart();
		if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	ret = send_slaveaddress(slaveaddress, I2C_READ);
		if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
	ret = receive(data, size);
		if(!is_oke(&ret)){
		set_return_line(&ret, __LINE__);
		return ret;
	}
		send_stop();

	return ret;
}


void I2C_IRQHandler(i2c *i2c){

}


#if defined(I2C1)
i2c i2c_1(I2C1);
i2c_t i2c1 = &i2c_1;
void I2C1_IRQHandler(void);
void I2C1_IRQHandler(void){

}
#endif /* defined(I2C1) */
#if defined(I2C2)
i2c i2c_2(I2C2);
i2c_t i2c2 = &i2c_2;
void I2C2_IRQHandler(void);
void I2C2_IRQHandler(void){

}
#endif /* defined(I2C2) */
#if defined(I2C3)
i2c i2c_3(I2C3);
i2c_t i2c3 = &i2c_3;
void I2C3_IRQHandler(void);
void I2C3_IRQHandler(void){

}
#endif /* defined(I2C3) */
#if defined(I2C4)
i2c i2c_4(I2C4);
i2c_t i2c4 = &i2c_4;
void I2C4_IRQHandler(void);
void I2C4_IRQHandler(void){

}
#endif /* defined(I2C4) */
#if defined(I2C5)
i2c i2c_5(I2C5);
i2c_t i2c5 = &i2c_5;
void I2C5_IRQHandler(void);
void I2C5_IRQHandler(void){

}
#endif /* defined(I2C4) */
#if defined(I2C6)
i2c i2c_6(I2C6);
i2c_t i2c6 = &i2c_6;
void I2C6_IRQHandler(void);
void I2C6_IRQHandler(void){

}
#endif /* defined(I2C4) */


#endif /* ENABLE_I2C */

