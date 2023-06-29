/*
 * iwdg.h
 *
 *  Created on: Mar 13, 2023
 *      Author: anh
 */

#ifndef PERIPH_IWDG_H_
#define PERIPH_IWDG_H_

#include "peripheral_config.h"
#if ENABLE_IWDG

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "system/ret_err.h"



stm_ret_t iwdg_init(uint32_t psc, uint32_t arr);

void iwdg_enable_in_debugmode(void);
void iwdg_disable_in_debugmode(void);

void iwdg_refresh(void);


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_IWDG */

#endif /* PERIPH_IWDG_H_ */
