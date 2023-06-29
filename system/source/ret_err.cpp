/*
 * ret_err.cpp
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */


#include "system/ret_err.h"

#include "periph/systick.h"
#include "stdlib.h"
#include "string.h"



uint32_t (*GetCounterFunction)(void) = get_tick;


/**
 * @fn stm_err_t check_flag_in_register(volatile uint32_t*, uint32_t, flaglevel_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param StatusCheck
 * @return
 */
stm_err_t check_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t StatusCheck){
	if((StatusCheck == FLAG_SET)? ((*Register & Flag) != 0U) : ((*Register & Flag) == 0U)) return STM_OKE;
	return STM_ERR;
}

/**
 * @fn void wait_flag_in_register(volatile uint32_t*, uint32_t, flaglevel_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param Level
 */
void wait_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level){
	while((Level == FLAG_RESET)?((*Register & Flag)) : (!(*Register & Flag)));
}

/**
 * @fn stm_ret_t wait_flag_in_register_timeout(volatile uint32_t*, uint32_t, flaglevel_t, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param Level
 * @param TimeOut
 * @return
 */
stm_ret_t wait_flag_in_register_timeout(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level, uint16_t TimeOut){
	stm_ret_t res;

	__IO uint32_t time = GetCounterFunction();
	while((Level == FLAG_RESET)?(*Register & Flag) : (!(*Register & Flag))){
		if(TimeOut != NO_TIMEOUT){
			if(GetCounterFunction() - time >= TimeOut) {
				res.Status  = STM_TIMEOUT;
				return res;
			}
		}
	}
	return res;
}

/**
 * @fn stm_ret_t wait_check_flag_in_register_timeout(volatile uint32_t*, uint32_t, flaglevel_t, volatile uint32_t*, uint32_t, flaglevel_t, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param RegisterCheck
 * @param FlagCheck
 * @param LevelCheck
 * @param RegisterWait
 * @param FlagWait
 * @param LevelWait
 * @param TimeOut
 * @return
 */
stm_ret_t wait_check_flag_in_register_timeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, flaglevel_t LevelCheck,
									  	  	 __IO uint32_t *RegisterWait, uint32_t FlagWait, flaglevel_t LevelWait,uint16_t TimeOut){
	stm_ret_t res;

	__IO uint32_t time = GetCounterFunction();
	while((LevelWait == FLAG_RESET)? (*RegisterWait & FlagWait) : (!(*RegisterWait & FlagWait))){
		if((LevelCheck == FLAG_RESET)? (!(*RegisterCheck & FlagCheck)) : (*RegisterCheck & FlagCheck)) {
			res.Status = STM_ERR;
			return res;
		}
		if(TimeOut != NO_TIMEOUT){
			if(GetCounterFunction() - time >= TimeOut) {
				res.Status  = STM_TIMEOUT;
				return res;
			}
		}
	}

	return res;
}

/**
 * @fn void set_return(stm_ret_t*, stm_err_t, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @param Status
 * @param CodeLine
 */
void set_return(stm_ret_t *res, stm_err_t Status, uint32_t CodeLine){
	res -> Status = Status;
	res -> Line = CodeLine;
}

/**
 * @fn void set_return_line(stm_ret_t*, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @param line
 */
void set_return_line(stm_ret_t *res, uint16_t line){
	res -> Line = line;
}

/**
 * @fn bool is_err(stm_ret_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_err(stm_ret_t *res){
	if(res -> Status == STM_ERR) return true;
	return false;
}

/**
 * @fn bool is_oke(stm_ret_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_oke(stm_ret_t *res){
	if(res -> Status == STM_OKE) return true;
	return false;
}

/**
 * @fn bool is_timeout(stm_ret_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_timeout(stm_ret_t *res){
	if(res -> Status == STM_TIMEOUT) return true;
	return false;
}

/**
 * @fn bool is_unsupported(stm_ret_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_unsupported(stm_ret_t *res){
	if(res -> Status == STM_UNSUPPORTED) return true;
	return false;
}

/**
 * @fn bool is_busy(stm_ret_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_busy(stm_ret_t *res){
	if(res -> Status == STM_BUSY) return true;
	return false;
}

/**
 * @fn bool is_ready(stm_ret_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_ready(stm_ret_t *res){
	if(res -> Status == STM_READY) return true;
	return false;
}

/**
 * @fn bool is_unavailable(stm_ret_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool is_notavailable(stm_ret_t *res){
	if(res -> Status == STM_NOTAVAILABLE) return true;
	return false;
}


















