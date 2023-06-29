/*
 * ret_err.cpp
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */

#ifndef STM_RET_ERR_H_
#define STM_RET_ERR_H_


#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"


typedef enum{
	FLAG_RESET = 0,
	FLAG_SET,
} flaglevel_t;

typedef enum{
	STM_ERR = 0,
	STM_OKE,
	STM_TIMEOUT,
	STM_UNSUPPORTED,
	STM_BUSY,
	STM_READY,
	STM_NOTAVAILABLE,
} stm_err_t;


typedef struct stm_ret{
	stm_err_t Status = STM_OKE;
	uint32_t Line = 0U;
} stm_ret_t;

enum{
	NO_TIMEOUT = 0,
	DEFAULT_TIMEOUT = 1000U,
};

stm_err_t check_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t StatusCheck);

void wait_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level);
stm_ret_t wait_flag_in_register_timeout(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level, uint16_t TimeOut);
stm_ret_t wait_check_flag_in_register_timeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, flaglevel_t LevelCheck,
									  __IO uint32_t *RegisterWait, uint32_t FlagWait, flaglevel_t LevelWait,uint16_t TimeOut);


void set_return(stm_ret_t *res, stm_err_t Status, uint32_t CodeLine);
void set_return_line(stm_ret_t *res, uint16_t line);


bool is_err(stm_ret_t *res);
bool is_oke(stm_ret_t *res);
bool is_timeout(stm_ret_t *res);
bool is_unsupported(stm_ret_t *res);
bool is_busy(stm_ret_t *res);
bool is_ready(stm_ret_t *res);
bool is_unavailable(stm_ret_t *res);


#ifdef __cplusplus
}
#endif

#endif /* STM_RET_ERR_H_ */
