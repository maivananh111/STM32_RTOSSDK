/*
 * rcc.cpp
 *
 *  Created on: Jan 5, 2023
 *      Author: anh
 */


#include "periph/rcc.h"
#include "peripheral_config.h"

#include "sdkconfig.h"
#include "system/system.h"
#include "periph/systick.h"
#include "stdio.h"
#include "math.h"


static rcc_config_t *_conf;

stm_ret_t rcc_init(rcc_config_t *rcc_conf){
#if (!CONFIG_OVER_CLOCK)
#if(SYSTEM_CLOCK_FREQUENCY > CONFIG_MAX_SYSTEM_CLOCK_FREQUENCY)
#error "SYSTEM_CLOCK_FREQUENCY out of range. Modify System clock frequency less than or equal to CONFIG_MAX_SYSTEM_CLOCK_FREQUENCY in sdkconfig.h file."
#endif
#if(AHB_CLOCK_FREQUENCY > CONFIG_MAX_AHB_CLOCK_FREQUENCY)
#error "AHB_CLOCK_FREQUENCY out of range. Modify AHB clock frequency less than or equal to CONFIG_MAX_AHB_CLOCK_FREQUENCY in sdkconfig.h file."
#endif
#if(APB1_CLOCK_FREQUENCY > CONFIG_MAX_APB1_CLOCK_FREQUENCY)
#error "APB1_CLOCK_FREQUENCY out of range. Modify APB1 clock frequency less than or equal to CONFIG_MAX_APB1_CLOCK_FREQUENCY in sdkconfig.h file."
#endif
#if(APB2_CLOCK_FREQUENCY > CONFIG_MAX_APB2_CLOCK_FREQUENCY)
#error "APB2_CLOCK_FREQUENCY out of range. Modify APB2 clock frequency less than or equal to CONFIG_MAX_APB2_CLOCK_FREQUENCY in sdkconfig.h file."
#endif
#endif


	stm_ret_t ret;
	__IO uint32_t tmpreg = 0;
	_conf = rcc_conf;

	/**
	 * OSC Configuration.
	 */

#if defined(STM32F1)
	if((RCC -> CFGR & RCC_CFGR_SWS_HSE) || ((RCC -> CFGR & RCC_CFGR_SWS_PLL) && (RCC -> CFGR & RCC_CFGR_PLLSRC))){
#elif defined(STM32F4)
	if((RCC -> CFGR & RCC_CFGR_SWS_HSE) || ((RCC -> CFGR & RCC_CFGR_SWS_PLL) && (RCC -> PLLCFGR & RCC_PLLCFGR_PLLSRC_HSE))){
#endif /* STM32F4 */
		if(!(RCC -> CR & RCC_CR_HSERDY)){
			set_return(&ret, STM_ERR, __LINE__);
			return ret;
		}
	}

	if(_conf -> osc_source == HSI_CRYSTAL){
//#if defined(STM32F1)
//		_conf->pll_mul = (uint32_t)(_conf->sysclock_frequency / (HSI_VALUE/2)) - 2;
//		if(_conf->pll_mul > 16UL) _conf->pll_mul = 16UL;
//#endif /* STM32F1 */
		RCC -> CR |= RCC_CR_HSION;
		ret = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_HSIRDY, FLAG_SET, RCC_HSI_TIMEOUT);
		if(is_timeout(&ret)) {
			set_return_line(&ret, __LINE__);
			return ret;
		}
		RCC -> CR &= ~RCC_CR_HSITRIM_Msk;
		RCC -> CR |= (_conf -> hsi_trim << RCC_CR_HSITRIM_Pos);

	}
	else if(_conf -> osc_source == HSE_CRYSTAL){
//#if defined(STM32F1)
//		_conf->pll_mul = (uint32_t)(_conf->sysclock_frequency / HSE_VALUE) - 2;
//		if(_conf->pll_mul > 16UL) _conf->pll_mul = 16UL;
//#endif /* STM32F1 */
		RCC -> CR |= RCC_CR_HSEON;
		ret = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_HSERDY, FLAG_SET, RCC_HSE_TIMEOUT);
		if(is_timeout(&ret)) {
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	else{
		set_return(&ret, STM_ERR, __LINE__);
		return ret;
	}

	/**
	 * PLL Configuration.
	 */
	if(_conf -> sysclock_source == PLLCLK){
		if(!(RCC -> CFGR & RCC_CFGR_SWS_PLL)){
			RCC -> CR &=~ RCC_CR_PLLON;
			ret = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_RESET, RCC_PLL_TIMEOUT);
			if(is_timeout(&ret)) {
				set_return_line(&ret, __LINE__);
				return ret;
			}
#if defined(STM32F1)
			if(_conf->pll_mul > 16UL) _conf->pll_mul = 16UL;
			_conf->pll_mul = _conf->pll_mul - 2U;
			RCC -> CFGR |= (_conf->pll_mul << RCC_CFGR_PLLMULL_Pos) | (_conf->pll_source << RCC_CFGR_PLLSRC_Pos);
#elif defined(STM32F4)
			tmpreg = RCC -> PLLCFGR;
			tmpreg &= ~(RCC_PLLCFGR_PLLM_Msk | RCC_PLLCFGR_PLLN_Msk | RCC_PLLCFGR_PLLP_Msk | RCC_PLLCFGR_PLLQ_Msk | RCC_PLLCFGR_PLLSRC_Msk);
			tmpreg |= ((_conf -> pll.pllm) << RCC_PLLCFGR_PLLM_Pos) | ((_conf -> pll.plln) << RCC_PLLCFGR_PLLN_Pos) | ((((_conf -> pll.pllp) >> 1U) - 1U) << RCC_PLLCFGR_PLLP_Pos) |
					  ((_conf -> pll.pllq) << RCC_PLLCFGR_PLLQ_Pos) | ((_conf -> pll_source) << RCC_PLLCFGR_PLLSRC_Pos);
			RCC -> PLLCFGR = tmpreg;
#endif /* STM32F4 */
			RCC -> CR |= RCC_CR_PLLON;
			ret = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_SET, RCC_PLL_TIMEOUT);
			if(is_timeout(&ret)) {
				set_return_line(&ret, __LINE__);
				return ret;
			}
		}
	}


	/**
	 * Calculation flash latency and update latency if new latency great than current latency.
	 */
	uint32_t latency = embedded_flash_calculate_latency(_conf->sysclock_frequency);
	uint32_t current_latency = embedded_flash_get_latency();
	if(latency > current_latency) embedded_flash_set_latency(latency);


	/**
	 * Check over clock to enable Over-drive mode.
	 */
#if CONFIG_OVER_CLOCK && defined(STM32F4)
	PWR -> CR |= PWR_CR_ODEN;
	ret = wait_flag_in_register_timeout(&(PWR -> CSR), PWR_CSR_ODRDY, FLAG_SET, 1000U);
	if(is_timeout(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}

	PWR -> CR |= PWR_CR_ODSWEN;
	ret = wait_flag_in_register_timeout(&(PWR -> CSR), PWR_CSR_ODSWRDY, FLAG_SET, 1000U);
	if(is_timeout(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}
#endif /* CONFIG_OVER_CLOCK */

	/**
	 * Check and set system clock source.
	 */
	if(_conf -> sysclock_source == HSI){
		if(!(RCC -> CR & RCC_CR_HSIRDY)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	else if(_conf -> sysclock_source == HSE){
		if(!(RCC -> CR & RCC_CR_HSERDY)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}
	else if(_conf -> sysclock_source == PLLCLK){
		if(!(RCC -> CR & RCC_CR_PLLRDY)){
			set_return_line(&ret, __LINE__);
			return ret;
		}
	}

	RCC -> CFGR = ((RCC -> CFGR & !RCC_CFGR_SW_Msk) | (_conf -> sysclock_source << RCC_CFGR_SW_Pos));
	ret = wait_flag_in_register_timeout(&(RCC -> CFGR), (_conf -> sysclock_source << RCC_CFGR_SW_Pos), FLAG_SET, RCC_SWS_TIMEOUT);
	if(is_timeout(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}

	/**
	 * SYSCLK, AHB, APB1, APB2 frequency configuration.
	 */
	tmpreg = RCC -> CFGR;
	tmpreg &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE1_Msk | RCC_CFGR_PPRE2_Msk);
	tmpreg |= ((_conf -> ahb_prescaler + 7U) << RCC_CFGR_HPRE_Pos) | ((_conf -> apb1_prescaler + 3U) << RCC_CFGR_PPRE1_Pos) | ((_conf -> apb2_prescaler + 3U) << RCC_CFGR_PPRE2_Pos);
	RCC -> CFGR = tmpreg;

	/**
	 * Update system core clock..
	 */
	SystemCoreClockUpdate();

	/**
	 * Update latency if new latency less than current latency.
	 */
	if(latency < current_latency) embedded_flash_set_latency(latency);

	/**
	 * ReInit system tick.
	 */
	systick_init(CONFIG_SYSTICK_INTERRUPT_PRIORITY);

#if defined(STM32F1)
	/** ENABLE AFIO CLOCK */
	RCC -> APB2ENR |= RCC_APB2ENR_AFIOEN;
	/** DISABLE JTAG */
	AFIO -> MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;

#endif /* STM32F1 */

	return ret;
}

stm_ret_t rcc_deinit(void){
	stm_ret_t ret;

	/**
	 * Turn on HSI (default).
	 */
	RCC -> CR |= RCC_CR_HSION;
	ret = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_HSIRDY, FLAG_SET, RCC_HSI_TIMEOUT);
	if(is_timeout(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}
	RCC -> CR &= ~RCC_CR_HSITRIM_Msk;
	RCC -> CR |= (0x10U << RCC_CR_HSITRIM_Pos);

	/**
	 * Clear CFGR register.
	 */
	RCC -> CFGR = 0x00U;
	ret = wait_flag_in_register_timeout(&(RCC -> CFGR), 0xFFFFFFFFU, FLAG_RESET, RCC_SWS_TIMEOUT);
	if(is_timeout(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}

	/**
	 * Turn off HSE.
	 */
	RCC -> CR &=~ RCC_CR_HSEON;
	ret = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_HSERDY, FLAG_RESET, RCC_HSE_TIMEOUT);
	if(is_timeout(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}

	/**
	 * Turn off PLL.
	 */
	RCC -> CR &=~ RCC_CR_PLLON;
	ret = wait_flag_in_register_timeout(&(RCC -> CR), RCC_CR_PLLRDY, FLAG_RESET, RCC_PLL_TIMEOUT);
	if(is_timeout(&ret)) {
		set_return_line(&ret, __LINE__);
		return ret;
	}
#if defined(STM32F1)
	RCC -> CFGR &=~ (RCC_CFGR_PLLMULL_Msk);
#elif defined(STM32F4)
	RCC -> PLLCFGR = RCC_PLLCFGR_PLLM_4 | RCC_PLLCFGR_PLLN_6 | RCC_PLLCFGR_PLLN_7 | RCC_PLLCFGR_PLLQ_2;
#endif /* STM32F4 */
	SystemCoreClock = HSI_VALUE;

	/**
	 * ReInit system tick.
	 */
	systick_init(CONFIG_SYSTICK_INTERRUPT_PRIORITY);


	return ret;
}

uint32_t rcc_get_bus_frequency(rcc_busclock_t bus){
	switch(bus){
		case SYSCLK:
			if(_conf -> osc_source == HSE_CRYSTAL){ // HSE.
				if(_conf -> sysclock_source == HSE) return (uint32_t)HSE_VALUE;
#if defined(STM32F1)
				else if(_conf -> sysclock_source == PLLCLK) return (uint32_t)(HSE_VALUE * (_conf->pll_mul + 2U));
#elif defined(STM32F4)
				else if(_conf -> sysclock_source == PLLCLK) return (uint32_t)(((HSE_VALUE / _conf -> pll.pllm) * _conf -> pll.plln) / _conf -> pll.pllp);
#endif /* STM32F4 */
			}
			else{ // HSI.
				if(_conf -> sysclock_source == HSI) return (uint32_t)HSI_VALUE;
#if defined(STM32F1)
				else if(_conf -> sysclock_source == PLLCLK) return (uint32_t)((HSI_VALUE / 2U) * (_conf->pll_mul + 2U));
#elif defined(STM32F4)
				else if(_conf -> sysclock_source == PLLCLK) return (uint32_t)(((HSE_VALUE / _conf -> pll.pllm) * _conf -> pll.plln) / _conf -> pll.pllp);
#endif /* STM32F4 */
			}
		break;

		case AHB:
			if(_conf -> ahb_prescaler <= 7) return (uint32_t)SystemCoreClock;
			return (uint32_t)(SystemCoreClock / (uint32_t)abs((int)(_conf -> ahb_prescaler - 6U)));
		break;

		case APB1:
			return (uint32_t)(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]);
		break;

		case APB2:
			return (uint32_t)(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]);
		break;

		case APB1_TIMER:
			return (uint32_t)(2*(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos]));
		break;

		case APB2_TIMER:
			return (uint32_t)(2*(SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos]));
		break;

	}
	return 0;
}



































