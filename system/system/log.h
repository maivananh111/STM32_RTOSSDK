/*
 * log.h
 *
 *  Created on: 28 thg 7, 2022
 *      Author: anh
 */

#ifndef LOG_H_
#define LOG_H_

#include "sdkconfig.h"
#if CONFIG_USE_LOG_MONITOR


#ifdef __cplusplus
extern "C"{
#endif

#include "stdio.h"
#include "stdarg.h"
#include "system/ret_err.h"

//#define HAL_TICK

typedef enum{
	SIMP_BLACK = 0,
	SIMP_RED,
	SIMP_GREEN,
	SIMP_YELLOW,
	SIMP_BLUE,
	SIMP_PURPLE,
	SIMP_CYAN,
	SIMP_WHITE,

	BOLD_BLACK = 8,
	BOLD_RED,
	BOLD_GREEN,
	BOLD_YELLOW,
	BOLD_BLUE,
	BOLD_PURPLE,
	BOLD_CYAN,
	BOLD_WHITE,

	ITALIC_BLACK = 16,
	ITALIC_RED,
	ITALIC_GREEN,
	ITALIC_YELLOW,
	ITALIC_BLUE,
	ITALIC_PURPLE,
	ITALIC_CYAN,
	ITALIC_WHITE,

	BCKGRN_BLACK = 24,
	BCKGRN_RED,
	BCKGRN_GREEN,
	BCKGRN_YELLOW,
	BCKGRN_BLUE,
	BCKGRN_PURPLE,
	BCKGRN_CYAN,
	BCKGRN_WHITE,
} log_type_t;

void log_init(void (*PrintString_Function)(char*));

void set_log(char *func, log_type_t log_type);

void LOG(log_type_t log_type, const char *tag, const char *format, ...);

void LOG_INFO (const char *tag,  const char *format, ...);
void LOG_WARN (const char *tag,  const char *format, ...);
void LOG_ERROR(const char *tag,  const char *format, ...);
void LOG_DEBUG(const char *tag,  const char *format, ...);
void LOG_PARAM(const char *tag,  const char *format, ...);
void LOG_MEM  (const char *tag,  const char *format, ...);
void LOG_RET  (const char *tag,  const char *format, ...);

void LOG_RES(stm_ret_t res);
void LOG_MEMORY(void);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_USE_LOG_MONITOR */
#endif /* LOG_H_ */
