/*
 * tim.cpp
 *
 *  Created on: Jan 13, 2023
 *      Author: anh
 */


#include "peripheral_config.h"
#if ENABLE_TIM

#include "periph/rcc.h"
#include "periph/gpio.h"
#include "periph/tim.h"
#include "periph/systick.h"
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
#include "system/log.h"
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */


#if CONFIG_USE_LOG_MONITOR && TIM_LOG
static const char *TAG = "TIM";
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */

void TIM_IRQHandler(tim *timptr);

tim::tim(TIM_TypeDef *Timer){
	_tim = Timer;
}

/* TIM Basic */
stm_ret_t tim::init(tim_config_t *conf){
	stm_ret_t ret;

	_conf = conf;

#if defined(TIM1)
	if     (_tim == TIM1)  RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
#endif
#if defined(TIM2)
	else if(_tim == TIM2)  RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
#endif
#if defined(TIM3)
	else if(_tim == TIM3)  RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;
#endif
#if defined(TIM4)
	else if(_tim == TIM4)  RCC -> APB1ENR |= RCC_APB1ENR_TIM4EN;
#endif
#if defined(TIM5)
	else if(_tim == TIM5)  RCC -> APB1ENR |= RCC_APB1ENR_TIM5EN;
#endif
#if defined(TIM6)
	else if(_tim == TIM6)  RCC -> APB1ENR |= RCC_APB1ENR_TIM6EN;
#endif
#if defined(TIM7)
	else if(_tim == TIM7)  RCC -> APB1ENR |= RCC_APB1ENR_TIM7EN;
#endif
#if defined(TIM8)
	else if(_tim == TIM8)  RCC -> APB2ENR |= RCC_APB2ENR_TIM8EN;
#endif
#if defined(TIM9)
	else if(_tim == TIM9)  RCC -> APB2ENR |= RCC_APB2ENR_TIM9EN;
#endif
#if defined(TIM10)
	else if(_tim == TIM10) RCC -> APB2ENR |= RCC_APB2ENR_TIM10EN;
#endif
#if defined(TIM11)
	else if(_tim == TIM11) RCC -> APB2ENR |= RCC_APB2ENR_TIM11EN;
#endif
#if defined(TIM12)
	else if(_tim == TIM12) RCC -> APB1ENR |= RCC_APB1ENR_TIM12EN;
#endif
#if defined(TIM13)
	else if(_tim == TIM13) RCC -> APB1ENR |= RCC_APB1ENR_TIM13EN;
#endif
#if defined(TIM14)
	else if(_tim == TIM14) RCC -> APB1ENR |= RCC_APB1ENR_TIM14EN;
#endif

	/* BASIC TIMER */
	_tim -> CR1 = 0U;
	_tim -> CR1 |= (_conf -> direction << TIM_CR1_DIR_Pos) | (_conf -> autoreloadpreload << TIM_CR1_ARPE_Pos) | (_conf -> align << TIM_CR1_CMS_Pos);

	_tim -> ARR = 0U;
	_tim -> ARR = _conf -> reload - 1;
	_tim -> PSC = 0U;
	_tim -> PSC = _conf -> prescaler - 1;

	_tim -> EGR = TIM_EGR_UG;

	if(_conf -> interrupt == TIM_INTERRUPT_ENABLE){
#if defined(TIM1)
		if	   (_tim == TIM1)  IRQn = TIM1_UP_TIM10_IRQn;
#endif
#if defined(TIM2)
		else if(_tim == TIM2)  IRQn = TIM2_IRQn;
#endif
#if defined(TIM3)
		else if(_tim == TIM3)  IRQn = TIM3_IRQn;
#endif
#if defined(TIM4)
		else if(_tim == TIM4)  IRQn = TIM4_IRQn;
#endif
#if defined(TIM5)
		else if(_tim == TIM5)  IRQn = TIM5_IRQn;
#endif
#if defined(TIM6)
		else if(_tim == TIM6)  IRQn = TIM6_DAC_IRQn;
#endif
#if defined(TIM7)
		else if(_tim == TIM7)  IRQn = TIM7_IRQn;
#endif
#if defined(TIM8)
		else if(_tim == TIM8)  IRQn = TIM8_UP_TIM13_IRQn;
#endif
#if defined(TIM9)
		else if(_tim == TIM9)  IRQn = TIM1_BRK_TIM9_IRQn;
#endif
#if defined(TIM10)
		else if(_tim == TIM10) IRQn = TIM1_UP_TIM10_IRQn;
#endif
#if defined(TIM11)
		else if(_tim == TIM11) IRQn = TIM1_TRG_COM_TIM11_IRQn;
#endif
#if defined(TIM12)
		else if(_tim == TIM12) IRQn = TIM8_BRK_TIM12_IRQn;
#endif
#if defined(TIM13)
		else if(_tim == TIM13) IRQn = TIM8_UP_TIM13_IRQn;
#endif
#if defined(TIM14)
		else if(_tim == TIM14) IRQn = TIM8_TRG_COM_TIM14_IRQn;
#endif
	}

	return ret;
}

tim_config_t *tim::get_config(void){
	return _conf;
}

void tim::set_prescaler (uint32_t psc){
	_conf -> prescaler = psc;
	_tim -> PSC = psc;
}
void tim::set_autoreload(uint32_t arl){
	_conf -> reload = arl;
	_tim -> ARR = arl;
}

void tim::reset_counter(void){
	_tim -> CNT = 0;
}

uint32_t tim::get_counter(void){
	return _tim -> CNT;
}

void tim::delay_us(uint32_t us){
	_tim -> CNT = 0;
	while(_tim -> CNT < us);
}

void tim::delay_ms(uint32_t ms){
	for (uint32_t i=0; i<ms; i++) delay_us(1000);
}

void tim::clear_update_isr(void){
	_tim -> SR &=~ TIM_DIER_UIE;
}


stm_ret_t tim::register_event_handler(void(*function_ptr)(tim_channel_t channel, tim_event_t event, void *param), void *param){
	stm_ret_t ret;

	if(_conf -> interrupt != TIM_INTERRUPT_ENABLE) {
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Timer interrupt disabled, can't register timer event handler.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	parameter = param;
	handler_callback = function_ptr;

	return ret;
}

stm_ret_t tim::unregister_event_handler(void){
	stm_ret_t ret;

	if(handler_callback == NULL){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Timer interrupt wasn't register event handler, can't unregister.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	handler_callback = NULL;

	return ret;
}

stm_ret_t tim::start(void){
	stm_ret_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, STM_BUSY, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer started, can't restart.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::stop(void){
	stm_ret_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::start_it(void){
	stm_ret_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}


	if(_conf -> interruptpriority < CONFIG_RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
#if CONFIG_FAIL_CHIP_RESET
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_INFO(TAG, "Chip will reset after %ds.", CONFIG_WAIT_FOR_RESET_TIME);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		systick_delay_ms(CONFIG_WAIT_FOR_RESET_TIME*1000U);
			__NVIC_SystemReset();
#endif /* CONFIG_FAIL_CHIP_RESET */
		return ret;
	}


	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, STM_BUSY, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	_tim -> DIER |= TIM_DIER_UIE;
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::stop_it(void){
	stm_ret_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer not started, can't stop in interrupt mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> CR1 &=~ TIM_CR1_CEN;

	if(_conf -> interrupt == TIM_INTERRUPT_ENABLE){
		 _tim -> DIER &=~ TIM_DIER_UIE;
		__NVIC_ClearPendingIRQ(IRQn);
		__NVIC_DisableIRQ(IRQn);
	}

	return ret;
}

#if ENABLE_DMA
stm_ret_t tim::start_dma(uint32_t *count_buf, uint16_t size){
	stm_ret_t ret;

	if(_conf -> dma_upd == NULL){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer not set dma, can't start in dma mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	_tim -> CR1  &=~ TIM_CR1_CEN;

	ret = _conf -> dma_upd -> start((uint32_t)count_buf, (uint32_t)&_tim -> CNT, size);
	if(!is_oke(&ret)) {
		set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::stop_dma(void){
	stm_ret_t ret;

	if(_conf -> dma_upd == NULL){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer not set dma, can't stop in dma mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	if((_tim -> DIER & TIM_DIER_CC1DE) && (_tim -> CR1 & TIM_CR1_CEN)){
		ret = _conf -> dma_upd -> stop();
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
			LOG_ERROR(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
			return ret;
		}

		_tim -> DIER &=~ TIM_DIER_CC1DE;
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer dma not start.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
	}

	return ret;
}
#endif

/* TIMER PWM OUTPUT MODE */
stm_ret_t tim::set_mode_pwm_output(tim_channel_t channel, tim_pwm_t *conf){
	stm_ret_t ret;

	gpio_port_clock_enable(conf->port);
#if defined(STM32F1)
	gpio_set_alternatefunction(conf->port, conf->pin, GPIO_ALTERNATE_PUSHPULL);
#elif defined(STM32F4)
	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
#if defined(LTDC)
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_LTDC_TIM12_14;
#else
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
#endif /* defined(LTDC) */
	else {
		set_return(&ret, STM_NOTAVAILABLE, __LINE__);
		return ret;
	}
	gpio_set_alternatefunction(conf->port, conf->pin, func);
	gpio_set_alternatefunction_type(conf->port, conf->pin, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */

	if(channel < TIM_CHANNEL3){ // Channel 1-2
		_tim -> CCMR1 &=~ (0xFF << (channel*8));
		_tim -> CCMR1 &=~ (TIM_CCMR1_CC1S << (channel*8));
		_tim -> CCMR1 |= ((conf->preload << TIM_CCMR1_OC1PE_Pos) << (channel*8));
		_tim -> CCMR1 |= ((conf->fastmode << TIM_CCMR1_OC1FE_Pos) << (channel*8));
		_tim -> CCMR1 |= ((conf->invert << TIM_CCMR1_OC1M_Pos) << (channel*8));
	}
	else if(channel >= TIM_CHANNEL3 && channel < TIM_NOCHANNEL){ // Channel 3-4
		_tim -> CCMR2 &=~ (0xFF << ((channel-2)*8));
		_tim -> CCMR2 &=~ (TIM_CCMR2_CC3S << ((channel-2)*8));
		_tim -> CCMR2 |= ((conf->preload << TIM_CCMR2_OC3PE_Pos) << ((channel - 2)*8));
		_tim -> CCMR2 |= ((conf->fastmode << TIM_CCMR2_OC3FE_Pos) << ((channel - 2)*8));
		_tim -> CCMR2 |= ((conf->invert << TIM_CCMR2_OC3M_Pos) << ((channel - 2)*8));
	}

	return ret;
}

stm_ret_t tim::pwm_output_start(tim_channel_t channel, uint32_t pwm){
	stm_ret_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, STM_BUSY, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	pwm_output_set_duty(channel, pwm);

	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::pwm_output_stop(tim_channel_t channel){
	stm_ret_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}


stm_ret_t tim::pwm_output_start_it(tim_channel_t channel, uint32_t pwm){
	stm_ret_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}


	if(_conf -> interruptpriority < CONFIG_RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
#if CONFIG_FAIL_CHIP_RESET
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_INFO(TAG, "Chip will reset after %ds.", CONFIG_WAIT_FOR_RESET_TIME);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		systick_delay_ms(CONFIG_WAIT_FOR_RESET_TIME*1000U);
			__NVIC_SystemReset();
#endif /* CONFIG_FAIL_CHIP_RESET */
		return ret;
	}


	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, STM_BUSY, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	pwm_output_set_duty(channel, pwm);

	_tim -> DIER |= (TIM_DIER_CC1IE << channel);
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::pwm_output_stop_it(tim_channel_t channel){
	stm_ret_t ret;
	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	if((_tim -> DIER & TIM_DIER_CC1IE << channel) && (_tim -> CR1 & TIM_CR1_CEN)){
		_tim -> DIER &=~ (TIM_DIER_CC1IE << channel);
		_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
		_tim -> CR1 &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer not started, can't stop in interrupt mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_DisableIRQ(IRQn);

	return ret;
}

#if ENABLE_DMA
stm_ret_t tim::pwm_output_start_dma(tim_channel_t channel, uint32_t *pwm_buffer, uint16_t size){
	stm_ret_t ret;
	dma_t dma = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dma = _conf -> dma_ch1;
		break;
		case TIM_CHANNEL2:
			dma = _conf -> dma_ch2;
		break;
		case TIM_CHANNEL3:
			dma = _conf -> dma_ch3;
		break;
		case TIM_CHANNEL4:
			dma = _conf -> dma_ch4;
		break;
		default:
		break;
	};

	if(dma == NULL){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer not set dma, can't start in dma mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_tim -> DIER &=~ TIM_DIER_CC1DE << channel;
	_tim -> CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	switch(channel){
		case TIM_CHANNEL1:
			CCRx_addr = (uint32_t)&_tim -> CCR1;
		break;
		case TIM_CHANNEL2:
			CCRx_addr = (uint32_t)&_tim -> CCR2;
		break;
		case TIM_CHANNEL3:
			CCRx_addr = (uint32_t)&_tim -> CCR3;
		break;
		case TIM_CHANNEL4:
			CCRx_addr = (uint32_t)&_tim -> CCR4;
		break;
		default:
		break;
	};
	ret = dma -> start((uint32_t)pwm_buffer, CCRx_addr, size);
	if(!is_oke(&ret)) {
		set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> DIER |= TIM_DIER_CC1DE << channel;
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::pwm_output_stop_dma(tim_channel_t channel){
	stm_ret_t ret;
	dma_t dma = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dma = _conf -> dma_ch1;
		break;
		case TIM_CHANNEL2:
			dma = _conf -> dma_ch2;
		break;
		case TIM_CHANNEL3:
			dma = _conf -> dma_ch3;
		break;
		case TIM_CHANNEL4:
			dma = _conf -> dma_ch4;
		break;
		default:
		break;
	};

	if(dma == NULL){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer not set dma, can't stop in dma mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	if((_tim -> DIER & TIM_DIER_CC1DE << channel) && (_tim -> CR1 & TIM_CR1_CEN)){
		_tim -> DIER &=~ (TIM_DIER_CC1DE << channel);
		ret = dma -> stop();
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
			LOG_ERROR(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
			return ret;
		}
		_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer dma not started.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
	}

	return ret;
}
#endif

stm_ret_t tim::pwm_output_set_duty(tim_channel_t channel, uint32_t pwm){
	stm_ret_t ret;

	switch(channel){
		case TIM_CHANNEL1:
			_tim -> CCR1 = pwm;
		break;
		case TIM_CHANNEL2:
			_tim -> CCR2 = pwm;
		break;
		case TIM_CHANNEL3:
			_tim -> CCR3 = pwm;
		break;
		case TIM_CHANNEL4:
			_tim -> CCR4 = pwm;
		break;
		default:
		break;
	};

	return ret;
}


/* TIMER PWM INPUT MODE */
stm_ret_t tim::set_mode_pwm_input(tim_channel_t channel, tim_pwm_t *conf){
	stm_ret_t ret;

	if(channel == TIM_CHANNEL3 || channel == TIM_CHANNEL4) {
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer PWM input unsupported on channel3 and channel 4.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	if(conf -> polarity == TIM_BOTH_EDGE){
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer PWM input unsupported both edge.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	gpio_port_clock_enable(conf->port);
#if defined(STM32F1)
	gpio_set_mode(conf->port, conf->pin, GPIO_INPUT);
#elif defined(STM32F4)
	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
#if defined(LTDC)
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_LTDC_TIM12_14;
#else
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
#endif /* defined(LTDC) */
	else {
		set_return(&ret, STM_NOTAVAILABLE, __LINE__);
		return ret;
	}
	gpio_set_alternatefunction(conf->port, conf->pin, func);
	gpio_set_alternatefunction_type(conf->port, conf->pin, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */
	_tim -> CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);

	_tim -> SMCR &=~ TIM_SMCR_TS_Msk;
	_tim -> SMCR |= ((channel+5U) << TIM_SMCR_TS_Pos) | TIM_SMCR_SMS_2;

	_tim -> CCMR1 &=~ (TIM_CCMR1_IC1F_Msk << (channel*8));
	_tim -> CCMR1 |= (conf -> ch1_filter << TIM_CCMR1_IC1F_Pos);
	_tim -> CCMR1 |= (conf -> ch2_filter << TIM_CCMR1_IC2F_Pos);

	_tim -> CCER &=~ (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);
	if(channel == TIM_CHANNEL1){
		_tim -> CCER |= (conf -> polarity << TIM_CCER_CC1P_Pos);
		_tim -> CCER |= (!(conf -> polarity) << TIM_CCER_CC2P_Pos);
	}
	else{
		_tim -> CCER |= (!(conf -> polarity) << TIM_CCER_CC1P_Pos);
		_tim -> CCER |= (conf -> polarity << TIM_CCER_CC2P_Pos);
	}

	_tim -> CCMR1 &=~ (TIM_CCMR1_CC1S_Msk << (channel*8U));
	_tim -> CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_1;

	return ret;
}

stm_ret_t tim::pwm_input_start(tim_channel_t channel){
	return inputcapture_start(channel);
}

stm_ret_t tim::pwm_input_stop(tim_channel_t channel){
	return inputcapture_stop(channel);
}

stm_ret_t tim::pwm_input_start_it(tim_channel_t channel){
	return inputcapture_start_it(channel);
}

stm_ret_t tim::pwm_input_stop_it(tim_channel_t channel){
	return inputcapture_stop_it(channel);
}
#if ENABLE_DMA
stm_ret_t tim::pwm_input_start_dma(tim_channel_t channel, uint32_t *pwm_buffer, uint16_t size){
	return inputcapture_start_dma(channel, pwm_buffer, size);
}

stm_ret_t tim::pwm_input_stop_dma(tim_channel_t channel){
	return inputcapture_stop_dma(channel);
}
#endif




/* TIMER ENCODER MODE */
stm_ret_t tim::set_mode_encoder(tim_encoder_t *conf){
	stm_ret_t ret;

	gpio_port_clock_enable(conf -> encA_ch1_port);
	gpio_port_clock_enable(conf -> encB_ch2_port);
#if defined(STM32F1)
	gpio_set_mode(conf->encA_ch1_port, conf->encA_ch1_pin, GPIO_INPUT_PULL);
	gpio_set_mode(conf->encB_ch2_port, conf->encB_ch2_pin, GPIO_INPUT_PULL);
#elif defined(STM32F4)
	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
#if defined(LTDC)
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_LTDC_TIM12_14;
#else
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
#endif /* defined(LTDC) */
	else {
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}
	gpio_set_alternatefunction(conf -> encA_ch1_port, conf -> encA_ch1_pin, func);
	gpio_set_alternatefunction(conf -> encB_ch2_port, conf -> encB_ch2_pin, func);

	gpio_set_alternatefunction_type(conf -> encA_ch1_port, conf -> encA_ch1_pin, GPIO_OUTPUT_PUSHPULL);
	gpio_set_alternatefunction_type(conf -> encB_ch2_port, conf -> encB_ch2_pin, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */
	if	   (_tim == TIM1)  IRQn = TIM1_CC_IRQn;
#if defined(TIM8)
	else if(_tim == TIM8)  IRQn = TIM8_CC_IRQn;
#endif /* TIM8 */
	_tim -> SMCR &=~ TIM_SMCR_SMS;
	_tim -> SMCR |= (conf -> mode << TIM_SMCR_SMS_Pos);

	_tim -> CCMR1 = 0U;
	_tim -> CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	_tim -> CCMR1 |= ((conf -> encA_ch1_prescaler << TIM_CCMR1_IC1PSC_Pos) | (conf -> encB_ch2_prescaler << TIM_CCMR1_IC2PSC_Pos));
	_tim -> CCMR1 |= ((conf -> encA_ch1_filter << TIM_CCMR1_IC1F_Pos) | (conf -> encB_ch2_filter << TIM_CCMR1_IC2F_Pos));

	_tim -> CCER &=~ (TIM_CCER_CC1P | TIM_CCER_CC1NP | TIM_CCER_CC2P | TIM_CCER_CC2NP);
	_tim -> CCER |= ((conf -> encA_ch1_edge << TIM_CCER_CC1P_Pos) | (conf -> encB_ch2_edge << TIM_CCER_CC2P_Pos));

	return ret;
}

stm_ret_t tim::encoder_start(void){
	stm_ret_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, STM_BUSY, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer started, can't restart.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_tim -> CCER |= TIM_CCER_CC1E;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_tim -> CCER |= TIM_CCER_CC2E;
	}
	else{
		_tim -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
	}

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::encoder_stop(void){
	stm_ret_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_tim -> CCER &=~ TIM_CCER_CC1E;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_tim -> CCER &=~ TIM_CCER_CC2E;
	}
	else{
		_tim -> CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);
	}

	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::encoder_start_it(void){
	stm_ret_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, STM_NOTAVAILABLE, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}


	if(_conf -> interruptpriority < CONFIG_RTOS_MAX_SYSTEM_INTERRUPT_PRIORITY){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Invalid priority, please increase the priority value.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
#if CONFIG_FAIL_CHIP_RESET
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_INFO(TAG, "Chip will reset after %ds.", CONFIG_WAIT_FOR_RESET_TIME);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		systick_delay_ms(CONFIG_WAIT_FOR_RESET_TIME*1000U);
		__NVIC_SystemReset();
#endif /* CONFIG_FAIL_CHIP_RESET */
		return ret;
	}


	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, STM_BUSY, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer started, can't restart in interrupt mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	if(mode == TIM_ENCODER_MODE1){
		_tim -> CCER |= TIM_CCER_CC1E;
		_tim -> DIER |= TIM_DIER_CC1IE;
	}
	else if(mode == TIM_ENCODER_MODE2){
		_tim -> CCER |= TIM_CCER_CC2E;
		_tim -> DIER |= TIM_DIER_CC2IE;
	}
	else{
		_tim -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
		_tim -> DIER |= TIM_DIER_CC1IE | TIM_DIER_CC2IE;
	}

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::encoder_stop_it(void){
	stm_ret_t ret;

	if(_conf -> interrupt == TIM_INTERRUPT_DISABLE){
		set_return(&ret, STM_NOTAVAILABLE, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s -> Timer interrupt disabled.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> CR1 &=~ TIM_CR1_CEN;
	_tim -> DIER &=~ TIM_DIER_UIE;
	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_DisableIRQ(IRQn);

	return ret;
}

#if ENABLE_DMA
stm_ret_t tim::encoder_start_dma(uint32_t *encA_buffer, uint32_t *encB_buffer, uint16_t size){
	stm_ret_t ret;

	if((_conf -> dma_ch1 == NULL && _conf -> dma_ch2 == NULL) || (encA_buffer == NULL && encB_buffer == NULL)){
		set_return(&ret, STM_NOTAVAILABLE, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer not set dma or data buffer, can't start in dma mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	_tim -> CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
	switch(mode){
		case TIM_ENCODER_MODE1:
			_tim -> DIER &=~ TIM_DIER_CC1DE;
			_tim -> CCER &=~ TIM_CCER_CC1E;

			CCRx_addr = (uint32_t)&_tim -> CCR1;
			ret = _conf -> dma_ch1 -> start(CCRx_addr, (uint32_t)encA_buffer, size);
			if(!is_oke(&ret)) {
				set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
				LOG_ERROR(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
				return ret;
			}

			_tim -> DIER |= TIM_DIER_CC1DE;
			_tim -> CCER |= TIM_CCER_CC1E;
		break;

		case TIM_ENCODER_MODE2:
			_tim -> DIER &=~ TIM_DIER_CC2DE;
			_tim -> CCER &=~ TIM_CCER_CC2E;

			CCRx_addr = (uint32_t)&_tim -> CCR2;
			ret = _conf -> dma_ch2 -> start(CCRx_addr, (uint32_t)encB_buffer, size);
			if(!is_oke(&ret)) {
				set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
				LOG_ERROR(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
				return ret;
			}

			_tim -> DIER |= TIM_DIER_CC2DE;
			_tim -> CCER |= TIM_CCER_CC2E;
		break;

		case TIM_ENCODER_MODE3:
			_tim -> DIER &=~ (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
			_tim -> CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);

			CCRx_addr = (uint32_t)&_tim -> CCR1;
			ret = _conf -> dma_ch1 -> start(CCRx_addr, (uint32_t)encA_buffer, size);
			if(!is_oke(&ret)) {
				set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
				LOG_ERROR(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
				return ret;
			}

			CCRx_addr = (uint32_t)&_tim -> CCR2;
			ret = _conf -> dma_ch2 -> start(CCRx_addr, (uint32_t)encB_buffer, size);
			if(!is_oke(&ret)) {
				set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
				LOG_ERROR(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
				return ret;
			}

			_tim -> DIER |= (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
			_tim -> CCER |= (TIM_CCER_CC1E | TIM_CCER_CC2E);
		break;

		default:
		break;
	};
	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::encoder_stop_dma(void){
	stm_ret_t ret;

	if((_conf -> dma_ch1 == NULL && _conf -> dma_ch2 == NULL)){
		set_return(&ret, STM_NOTAVAILABLE, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer not set dma or data buffer, can't stop in dma mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	if(((_tim -> DIER & TIM_DIER_CC1DE) || (_tim -> DIER & TIM_DIER_CC2DE)) && (_tim -> CR1 & TIM_CR1_CEN)){
		tim_encoder_mode_t mode = (tim_encoder_mode_t)((_tim -> SMCR & TIM_SMCR_SMS_Msk) >> TIM_SMCR_SMS_Pos);
		switch(mode){
			case TIM_ENCODER_MODE1:
				_tim -> DIER &=~ TIM_DIER_CC1DE;
				ret = _conf -> dma_ch1 -> stop();
				if(!is_oke(&ret)) {
					set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
					LOG_ERROR(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
					return ret;
				}
				_tim -> CCER &=~ TIM_CCER_CC1E;
			break;

			case TIM_ENCODER_MODE2:
				_tim -> DIER &=~ TIM_DIER_CC2DE;
				ret = _conf -> dma_ch2 -> stop();
				if(!is_oke(&ret)) {
					set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
					LOG_ERROR(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
					return ret;
				}
				_tim -> CCER &=~ TIM_CCER_CC2E;
			break;

			case TIM_ENCODER_MODE3:
				_tim -> DIER &=~ (TIM_DIER_CC1DE | TIM_DIER_CC2DE);
				ret = _conf -> dma_ch1 -> stop();
				if(!is_oke(&ret)) {
					set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
					LOG_ERROR(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
					return ret;
				}
				ret = _conf -> dma_ch2 -> stop();
				if(!is_oke(&ret)) {
					set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
					LOG_ERROR(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
					return ret;
				}
				_tim -> CCER &=~ (TIM_CCER_CC1E | TIM_CCER_CC2E);
			break;

			default:
			break;
		};
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer dma not started.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
	}

	return ret;
}
#endif

int16_t tim::encoder_get_counter(void){
	return (int16_t)((int16_t)counter/4);
}

uint32_t tim::encoder_get_base_counter(void){
	return counter;
}


/* TIMER INPUT CAPTURE MODE */
stm_ret_t tim::set_mode_inputcapture(tim_channel_t channel, tim_inputcapture_t *conf){
	stm_ret_t ret;

	gpio_port_clock_enable(conf -> port);
#if defined(STM32F1)
	gpio_set_mode(conf->port, conf->pin, GPIO_INPUT);
#elif defined(STM32F4)
	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
#if defined(LTDC)
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_LTDC_TIM12_14;
#else
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
#endif /* defined(LTDC) */
	else {
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}
	gpio_set_alternatefunction(conf -> port, conf -> pin, func);
	gpio_set_alternatefunction_type(conf -> port, conf -> pin, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */
	if	   (_tim == TIM1)  IRQn = TIM1_CC_IRQn;
#if defined(TIM8)
	else if(_tim == TIM8)  IRQn = TIM8_CC_IRQn;
#endif /* TIM8 */

	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4U));
	if(channel < TIM_CHANNEL3){
		_tim -> CCMR1 &=~ (0xFF << (channel));
		_tim -> CCMR1 |= (TIM_CCMR1_CC1S_0 << (channel*8U));

		_tim -> CCMR1 |= ((conf -> filter << TIM_CCMR1_IC1F_Pos) << (channel*8U));

		_tim -> CCMR1 |= (conf -> prescaler << (channel*8U));
	}
	else if(channel >= TIM_CHANNEL3 && channel < TIM_NOCHANNEL){
		_tim -> CCMR2 &=~ (0xFF << ((channel-2)));
		_tim -> CCMR2 |= (TIM_CCMR2_CC3S_0 << ((channel-2)*8U));

		_tim -> CCMR2 |= ((conf -> filter << TIM_CCMR2_IC3F_Pos) << ((channel-2)*8U));

		_tim -> CCMR2 |=(conf -> prescaler << (channel*8U));
	}
	_tim -> CCER &=~ ((TIM_CCER_CC1P | TIM_CCER_CC1NP) << (channel*4));
	_tim -> CCER |= ((conf -> polarity << TIM_CCER_CC1P_Pos) << (channel*4));

	return ret;
}

stm_ret_t tim::inputcapture_start(tim_channel_t channel){
	stm_ret_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, STM_BUSY, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer started, can't restart.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::inputcapture_stop(tim_channel_t channel){
	stm_ret_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::inputcapture_start_it(tim_channel_t channel){
	stm_ret_t ret;

	if(_tim -> CR1 & TIM_CR1_CEN){
		set_return(&ret, STM_BUSY, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer started, can't restart.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> DIER |= (TIM_DIER_CC1IE << channel);
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_SetPriority(IRQn, _conf -> interruptpriority);
	__NVIC_EnableIRQ(IRQn);
	__NVIC_ClearPendingIRQ(IRQn);

	_tim -> CR1 |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::inputcapture_stop_it(tim_channel_t channel){
	stm_ret_t ret;

	if(!(_tim -> CR1 & TIM_CR1_CEN)){
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_WARN(TAG, "%s -> %s, Timer not started, can't stop.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> DIER &=~ (TIM_DIER_CC1IE << channel);
	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));

	__NVIC_ClearPendingIRQ(IRQn);
	__NVIC_DisableIRQ(IRQn);

	_tim -> CR1 &=~ TIM_CR1_CEN;

	return ret;
}

#if ENABLE_DMA
stm_ret_t tim::inputcapture_start_dma(tim_channel_t channel, uint32_t *capture_buffer, uint16_t size){
	stm_ret_t ret;
	dma_t dma = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dma = _conf -> dma_ch1;
		break;
		case TIM_CHANNEL2:
			dma = _conf -> dma_ch2;
		break;
		case TIM_CHANNEL3:
			dma = _conf -> dma_ch3;
		break;
		case TIM_CHANNEL4:
			dma = _conf -> dma_ch4;
		break;
		default:
		break;
	};

	if(dma == NULL){
		set_return(&ret, STM_NOTAVAILABLE, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer not set dma, can't start in dma mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
	_tim -> DIER &=~ TIM_DIER_CC1DE << channel;
	_tim -> CR1  &=~ TIM_CR1_CEN;

	uint32_t CCRx_addr = 0x00U;
	switch(channel){
		case TIM_CHANNEL1:
			CCRx_addr = (uint32_t)&_tim -> CCR1;
		break;
		case TIM_CHANNEL2:
			CCRx_addr = (uint32_t)&_tim -> CCR2;
		break;
		case TIM_CHANNEL3:
			CCRx_addr = (uint32_t)&_tim -> CCR3;
		break;
		case TIM_CHANNEL4:
			CCRx_addr = (uint32_t)&_tim -> CCR4;
		break;
		default:
		break;
	};
	ret = dma -> start(CCRx_addr, (uint32_t)capture_buffer, size);
	if(!is_oke(&ret)) {
		set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer dma start fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}
	_tim -> DIER |= TIM_DIER_CC1DE << channel;
	_tim -> CCER |= (TIM_CCER_CC1E << (channel*4));
	_tim -> CR1  |= TIM_CR1_CEN;

	return ret;
}

stm_ret_t tim::inputcapture_stop_dma(tim_channel_t channel){
	stm_ret_t ret;
	dma_t dma = NULL;

	switch(channel){
		case TIM_CHANNEL1:
			dma = _conf -> dma_ch1;
		break;
		case TIM_CHANNEL2:
			dma = _conf -> dma_ch2;
		break;
		case TIM_CHANNEL3:
			dma = _conf -> dma_ch3;
		break;
		case TIM_CHANNEL4:
			dma = _conf -> dma_ch4;
		break;
		default:
		break;
	};

	if(dma == NULL){
		set_return(&ret, STM_NOTAVAILABLE, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer not set dma, can't stop in dma mode.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
		return ret;
	}

	if((_tim -> DIER & TIM_DIER_CC1DE << channel) && (_tim -> CR1 & TIM_CR1_CEN)){
		_tim -> DIER &=~ (TIM_DIER_CC1DE << channel);
		ret = dma -> stop();
		if(!is_oke(&ret)) {
			set_return_line(&ret, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
			LOG_ERROR(TAG, "%s -> %s, Timer dma stop fail.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
			return ret;
		}
		_tim -> CCER &=~ (TIM_CCER_CC1E << (channel*4));
		_tim -> CR1  &=~ TIM_CR1_CEN;
	}
	else{
		set_return(&ret, STM_ERR, __LINE__);
#if CONFIG_USE_LOG_MONITOR && TIM_LOG
		LOG_ERROR(TAG, "%s -> %s, Timer dma not started.", __FILE__, __FUNCTION__);
#endif /* CONFIG_USE_LOG_MONITOR && TIM_LOG */
	}

	return ret;
}
#endif

uint32_t tim::get_capture_counter(tim_channel_t channel){
	switch(channel){
		case TIM_CHANNEL1:
			return _tim -> CCR1;

		case TIM_CHANNEL2:
			return _tim -> CCR2;

		case TIM_CHANNEL3:
			return _tim -> CCR3;

		case TIM_CHANNEL4:
			return _tim -> CCR4;
		default:
			return 0U;
	};
	return 0U;
}

/* TIMER OuTPUT COMPARE MODE*/
stm_ret_t tim::set_mode_outputcompare(tim_channel_t channel, tim_outputcompare_t *conf){
	stm_ret_t ret;

	gpio_port_clock_enable(conf -> port);
#if defined(STM32F1)
	gpio_set_alternatefunction(conf->port, conf->pin, GPIO_ALTERNATE_PUSHPULL);
#elif defined(STM32F4)
	gpio_alternatefunction_t func;
	if(_tim == TIM1 || _tim == TIM2) 										func = AF1_TIM1_2;
	else if(_tim == TIM3 || _tim == TIM4 || _tim == TIM5) 					func = AF2_TIM3_5;
	else if(_tim == TIM8 || _tim == TIM9 || _tim == TIM10 || _tim == TIM11) func = AF3_TIM8_11;
#if defined(LTDC)
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_LTDC_TIM12_14;
#else
	else if(_tim == TIM12 || _tim == TIM13 || _tim == TIM14) 				func = AF9_CAN1_2_TIM12_14;
#endif /* defined(LTDC) */
	else {
		set_return(&ret, STM_UNSUPPORTED, __LINE__);
		return ret;
	}
	gpio_set_alternatefunction(conf -> port, conf -> pin, func);
	gpio_set_alternatefunction_type(conf -> port, conf -> pin, GPIO_OUTPUT_PUSHPULL);
#endif /* STM32F4 */

	if	   (_tim == TIM1)  IRQn = TIM1_CC_IRQn;
#if defined(TIM8)
	else if(_tim == TIM8)  IRQn = TIM8_CC_IRQn;
#endif /* TIM8 */

	_tim -> CCER &=~ TIM_CCER_CC1E;

	if(channel < TIM_CHANNEL3){
		_tim -> CCMR1 &=~ (0xFF << (channel*8));
		_tim -> CCMR1 |= ((conf -> mode << TIM_CCMR1_OC1M_Pos) << (channel*8));
	}
	else if (channel > TIM_CHANNEL2 && channel < TIM_NOCHANNEL){
		_tim -> CCMR2 &=~ (0xFF << ((channel-2)*8));
		_tim -> CCMR2 |= ((conf -> mode << TIM_CCMR2_OC3M_Pos) << ((channel-2)*8));
	}
	_tim -> CCER &=~ (0x0F << (channel*4));
	_tim -> CCER |= ((conf -> level_polarity << TIM_CCER_CC1P_Pos) << (channel*4));

	return ret;
}

stm_ret_t tim::outputcompare_start(tim_channel_t channel, uint32_t value){
	return pwm_output_start(channel, value);
}

stm_ret_t tim::outputcompare_stop(tim_channel_t channel){
	return pwm_output_stop(channel);
}

stm_ret_t tim::outputcompare_start_it(tim_channel_t channel, uint32_t value){
	return pwm_output_start_it(channel, value);
}

stm_ret_t tim::outputcompare_stop_it(tim_channel_t channel){
	return pwm_output_stop_it(channel);
}

#if ENABLE_DMA
stm_ret_t tim::outputcompare_start_dma(tim_channel_t channel, uint32_t *oc_buffer, uint16_t size){
	return pwm_output_start_dma(channel, oc_buffer, size);
}

stm_ret_t tim::outputcompare_stop_dma(tim_channel_t channel){
	return pwm_output_stop_dma(channel);
}
#endif


stm_ret_t tim::set_pulse(tim_channel_t channel, uint32_t pulse){
	stm_ret_t ret;


	return ret;
}


void TIM_IRQHandler(tim *timptr){
	tim_event_t event = TIM_EVENT_NOEVENT;
	tim_channel_t channel = TIM_NOCHANNEL;

	timptr -> counter = timptr -> _tim -> CNT;

	/* TIMER CAPTURE-COMPARE 1 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC1IF && timptr -> _tim -> DIER & TIM_DIER_CC1IE){
		timptr -> _tim -> SR =~ TIM_SR_CC1IF;
		event = TIM_EVENT_CAPTURECOMPARE1;
		channel = TIM_CHANNEL1;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 2 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC2IF && timptr -> _tim -> DIER & TIM_DIER_CC2IE){
		timptr -> _tim -> SR =~ TIM_SR_CC2IF;
		event = TIM_EVENT_CAPTURECOMPARE2;
		channel = TIM_CHANNEL2;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 3 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC3IF && timptr -> _tim -> DIER & TIM_DIER_CC3IE){
		timptr -> _tim -> SR =~ TIM_SR_CC3IF;
		event = TIM_EVENT_CAPTURECOMPARE3;
		channel = TIM_CHANNEL3;
		goto EventCB;
	}

	/* TIMER CAPTURE-COMPARE 4 INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_CC4IF && timptr -> _tim -> DIER & TIM_DIER_CC4IE){
		timptr -> _tim -> SR =~ TIM_SR_CC4IF;
		event = TIM_EVENT_CAPTURECOMPARE4;
		channel = TIM_CHANNEL4;
		goto EventCB;
	}

	/* TIMER UPDATE INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_UIF && timptr -> _tim -> DIER & TIM_DIER_UIE){
		timptr -> _tim -> SR =~ TIM_SR_UIF;
		event = TIM_EVENT_UPDATE;
		goto EventCB;
	}

	/* TIMER BREAK INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_BIF && timptr -> _tim -> DIER & TIM_DIER_BIE){
		timptr -> _tim -> SR =~ TIM_SR_BIF;
		event = TIM_EVENT_BREAK;
		goto EventCB;
	}

	/* TIMER TRIGER INTERRUPT */
	if(timptr -> _tim -> SR & TIM_SR_TIF && timptr -> _tim -> DIER & TIM_DIER_TIE){
		timptr -> _tim -> SR =~ TIM_SR_TIF;
		event = TIM_EVENT_TRIGER;
		goto EventCB;
	}

	EventCB:
	if(timptr -> handler_callback != NULL) timptr -> handler_callback(channel, event, timptr -> parameter);
}

#if defined(TIM1)
tim tim_1(TIM1);
tim_t tim1 = &tim_1;
void TIM1_CC_IRQHandler(void){
	TIM_IRQHandler(&tim_1);
}
#if !defined(TIM9)
void TIM1_BRK_TIM9_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void){
#if USE_TIM1
//	TIM_IRQHandler(&tim_1);
#endif
}
#endif /* !defined(TIM9) */
#if !defined(TIM10)
void TIM1_UP_TIM10_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void){
#if USE_TIM1
//	TIM_IRQHandler(&tim_1);
#endif
}
#endif /* !defined(TIM10) */
#if !defined(TIM11)
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void){
#if USE_TIM1
//	TIM_IRQHandler(&tim_1);
#endif
}
#endif /* !defined(TIM11) */
#endif /* defined(TIM1) */

#if defined(TIM2)
tim tim_2(TIM2);
tim_t tim2 = &tim_2;
void TIM2_IRQHandler(void);
void TIM2_IRQHandler(void){
	TIM_IRQHandler(&tim_2);
}
#endif /* defined(TIM2) */

#if defined(TIM3)
tim tim_3(TIM3);
tim_t tim3 = &tim_3;
void TIM3_IRQHandler(void);
void TIM3_IRQHandler(void){
	TIM_IRQHandler(&tim_3);
}
#endif /* defined(TIM3) */

#if defined(TIM4)
tim tim_4(TIM4);
tim_t tim4 = &tim_4;
void TIM4_IRQHandler(void);
void TIM4_IRQHandler(void){
	TIM_IRQHandler(&tim_4);
}
#endif /* defined(TIM4) */

#if defined(TIM5)
tim tim_5(TIM5);
tim_t tim5 = &tim_5;
void TIM5_IRQHandler(void);
void TIM5_IRQHandler(void){
	TIM_IRQHandler(&tim_5);
}
#endif /* defined(TIM5) */

#if defined(TIM6)
tim tim_6(TIM6);
tim_t tim6 = &tim_6;
void TIM6_DAC_IRQHandler(void);
void TIM6_DAC_IRQHandler(void){
	TIM_IRQHandler(&tim_6);
#if ENABLE_DAC
	DAC_IRQHandler();
#endif
}
#endif /* defined(TIM6) */

#if defined(TIM7)
tim tim_7(TIM7);
tim_t tim7 = &tim_7;
void TIM7_IRQHandler(void);
void TIM7_IRQHandler(void){
	TIM_IRQHandler(&tim_7);
}
#endif /* defined(TIM7) */

#if defined(TIM8)
tim tim_8(TIM8);
tim_t tim8 = &tim_8;
void TIM8_CC_IRQHandler(void);
void TIM8_CC_IRQHandler(void){
	TIM_IRQHandler(&tim_8);
}
#if !defined(TIM12)
void TIM8_BRK_TIM12_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void){
#if USE_TIM8
	TIM_IRQHandler(&tim_8);
#endif
}
#endif /* !defined(TIM12) */
#if !defined(TIM13)
void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void){
#if USE_TIM8
	TIM_IRQHandler(&tim_8);
#endif
}
#endif /* !defined(TIM13) */
#if !defined(TIM14)
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void){
#if USE_TIM8
	TIM_IRQHandler(&tim_8);
#endif
	TIM_IRQHandler(&tim_14);
}
#endif /* !defined(TIM14) */
#endif /* defined(TIM8) */

#if defined(TIM9) && defined(TIM1)
tim tim_9(TIM9);
tim_t tim9 = &tim_9;
void TIM1_BRK_TIM9_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void){
#if USE_TIM1
	TIM_IRQHandler(&tim_1);
#endif
	TIM_IRQHandler(&tim_9);
}
#endif /* defined(TIM9) && defined(TIM1) */

#if defined(TIM10) && defined(TIM1)
tim tim_10(TIM10);
tim_t tim10 = &tim_10;
void TIM1_UP_TIM10_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler(void){
#if USE_TIM1
	TIM_IRQHandler(&tim_1);
#endif
	TIM_IRQHandler(&tim_10);
}
#endif /* defined(TIM10) && defined(TIM1) */

#if defined(TIM11) && defined(TIM1)
tim tim_11(TIM11);
tim_t tim11 = &tim_11;
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void){
#if USE_TIM1
	TIM_IRQHandler(&tim_1);
#endif
	TIM_IRQHandler(&tim_11);
}
#endif /* defined(TIM11) && defined(TIM1) */

#if defined(TIM12) && defined(TIM8)
tim tim_12(TIM12);
tim_t tim12 = &tim_12;
void TIM8_BRK_TIM12_IRQHandler(void);
void TIM8_BRK_TIM12_IRQHandler(void){
#if USE_TIM8
	TIM_IRQHandler(&tim_8);
#endif
	TIM_IRQHandler(&tim_12);
}
#endif /* defined(TIM12) && defined(TIM8) */

#if defined(TIM13) && defined(TIM8)
tim tim_13(TIM13);
tim_t tim13 = &tim_13;
void TIM8_UP_TIM13_IRQHandler(void);
void TIM8_UP_TIM13_IRQHandler(void){
#if USE_TIM8
	TIM_IRQHandler(&tim_8);
#endif
	TIM_IRQHandler(&tim_13);
}
#endif /* defined(TIM13) && defined(TIM8) */

#if defined(TIM14) && defined(TIM8)
tim tim_14(TIM14);
tim_t tim14 = &tim_14;
void TIM8_TRG_COM_TIM14_IRQHandler(void);
void TIM8_TRG_COM_TIM14_IRQHandler(void){
#if USE_TIM8
	TIM_IRQHandler(&tim_8);
#endif
	TIM_IRQHandler(&tim_14);
}
#endif /* defined(TIM14) && defined(TIM8) */


#endif


