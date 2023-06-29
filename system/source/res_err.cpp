/*
 * res_err.cpp
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */


#include "system/res_err.h"

#include "periph/systick.h"
#include "stdlib.h"
#include "string.h"



uint32_t (*GetCounterFunction)(void) = get_tick;


/**
 * @fn error_t check_flag_in_register(volatile uint32_t*, uint32_t, flaglevel_t)
 * @brief
 *
 * @pre
 * @post
 * @param Register
 * @param Flag
 * @param StatusCheck
 * @return
 */
error_t check_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t StatusCheck){
	if((StatusCheck == FLAG_SET)? ((*Register & Flag) != 0U) : ((*Register & Flag) == 0U)) return E_OKE;
	return E_FAIL;
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
 * @fn result_t wait_flag_in_register_timeout(volatile uint32_t*, uint32_t, flaglevel_t, uint16_t)
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
result_t wait_flag_in_register_timeout(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level, uint16_t TimeOut){
	result_t res;

	__IO uint32_t time = GetCounterFunction();
	while((Level == FLAG_RESET)?(*Register & Flag) : (!(*Register & Flag))){
		if(TimeOut != NO_TIMEOUT){
			if(GetCounterFunction() - time >= TimeOut) {
				res.Status  = E_TIMEOUT;
				return res;
			}
		}
	}
	return res;
}

/**
 * @fn result_t wait_check_flag_in_register_timeout(volatile uint32_t*, uint32_t, flaglevel_t, volatile uint32_t*, uint32_t, flaglevel_t, uint16_t)
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
result_t wait_check_flag_in_register_timeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, flaglevel_t LevelCheck,
									  	  	 __IO uint32_t *RegisterWait, uint32_t FlagWait, flaglevel_t LevelWait,uint16_t TimeOut){
	result_t res;

	__IO uint32_t time = GetCounterFunction();
	while((LevelWait == FLAG_RESET)? (*RegisterWait & FlagWait) : (!(*RegisterWait & FlagWait))){
		if((LevelCheck == FLAG_RESET)? (!(*RegisterCheck & FlagCheck)) : (*RegisterCheck & FlagCheck)) {
			res.Status = E_FAIL;
			return res;
		}
		if(TimeOut != NO_TIMEOUT){
			if(GetCounterFunction() - time >= TimeOut) {
				res.Status  = E_TIMEOUT;
				return res;
			}
		}
	}

	return res;
}

/**
 * @fn void set_return(result_t*, error_t, uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @param Status
 * @param CodeLine
 */
void set_return(result_t *res, error_t Status, uint32_t CodeLine){
	res -> Status = Status;
	res -> Line = CodeLine;
}

/**
 * @fn void set_return_line(result_t*, uint16_t)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @param line
 */
void set_return_line(result_t *res, uint16_t line){
	res -> Line = line;
}

/**
 * @fn bool err_is_fail(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_fail(result_t *res){
	if(res -> Status == E_FAIL) return true;
	return false;
}

/**
 * @fn bool err_is_oke(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_oke(result_t *res){
	if(res -> Status == E_OKE) return true;
	return false;
}

/**
 * @fn bool err_is_timeout(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_timeout(result_t *res){
	if(res -> Status == E_TIMEOUT) return true;
	return false;
}

/**
 * @fn bool err_is_unsupported(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_unsupported(result_t *res){
	if(res -> Status == E_UNSUPPORTED) return true;
	return false;
}

/**
 * @fn bool err_is_busy(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_busy(result_t *res){
	if(res -> Status == E_BUSY) return true;
	return false;
}

/**
 * @fn bool err_is_ready(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_ready(result_t *res){
	if(res -> Status == E_READY) return true;
	return false;
}

/**
 * @fn bool err_is_unavailable(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_notavailable(result_t *res){
	if(res -> Status == E_NOTAVAILABLE) return true;
	return false;
}

/**
 * @fn bool err_err_is_notfound(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_err_is_notfound(result_t *res){
	if(res -> Status == E_NOTFOUND) return true;
	return false;
}

/**
 * @fn bool err_is_memory(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_memory(result_t *res){
	if(res -> Status == E_MEMORY) return true;
	return false;
}

/**
 * @fn bool err_is_invalid(result_t*)
 * @brief
 *
 * @pre
 * @post
 * @param res
 * @return
 */
bool err_is_invalid(result_t *res){
	if(res -> Status == E_INVALID) return true;
	return false;
}
















