/*
 * res_err.cpp
 *
 *  Created on: Oct 12, 2022
 *      Author: anh
 */

#ifndef SYSTEM_RES_ERR_H_
#define SYSTEM_RES_ERR_H_


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
	E_FAIL = -1,
	E_OKE = 0,
	E_READY,
	E_TIMEOUT,
	E_UNSUPPORTED,
	E_BUSY,
	E_NOTAVAILABLE,
	E_NOTFOUND,
	E_MEMORY,
	E_INVALID,
} error_t;


typedef struct stm_ret{
	error_t Status = E_OKE;
	uint32_t Line = 0U;
} result_t;

enum{
	NO_TIMEOUT = 0,
	DEFAULT_TIMEOUT = 1000U,
};

error_t check_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t StatusCheck);

void wait_flag_in_register(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level);
result_t wait_flag_in_register_timeout(__IO uint32_t *Register, uint32_t Flag, flaglevel_t Level, uint16_t TimeOut);
result_t wait_check_flag_in_register_timeout(__IO uint32_t *RegisterCheck, uint32_t FlagCheck, flaglevel_t LevelCheck,
									  __IO uint32_t *RegisterWait, uint32_t FlagWait, flaglevel_t LevelWait,uint16_t TimeOut);


void set_return(result_t *res, error_t Status, uint32_t CodeLine);
void set_return_line(result_t *res, uint16_t line);

bool err_is_oke(result_t *res);
bool err_is_fail(result_t *res);
bool err_is_timeout(result_t *res);
bool err_is_unsupported(result_t *res);
bool err_is_busy(result_t *res);
bool err_is_ready(result_t *res);
bool err_is_unavailable(result_t *res);
bool err_is_notfound(result_t *res);
bool err_is_memory(result_t *res);
bool err_is_invalid(result_t *res);


#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_RES_ERR_H_ */
