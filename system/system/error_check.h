/*
 * error_check.h
 *
 *  Created on: Jun 19, 2023
 *      Author: anh
 */

#ifndef SYSTEM_ERROR_CHECK_H_
#define SYSTEM_ERROR_CHECK_H_


#ifdef __cplusplus
extern "C"{
#endif


#include "system/log.h"

#define CHECK_RETURN(expression, loglevel, tag, format, ...) {\
		if(expression){\
			loglevel(tag, "%s[%d] >>> " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			return;\
		}\
}

#define CHECK_RETURN_VAL(expression, val_return, loglevel, tag, format, ...) {\
		if(expression){\
			loglevel(tag, "%s[%d] >>> " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			return val_return;\
		}\
}


#define CHECK_GOTO(expression, tag_goto, loglevel, tag, format, ...) {\
		if(expression){\
			loglevel(tag, "%s[%d] >>> " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			goto tag_goto;\
		}\
}


#define CHECK_BREAK(expression, loglevel, tag, format, ...) {\
		if(expression){\
			loglevel(tag, "%s[%d] >>> " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			break;\
		}\
}

#define CHECK_CONTINUE(expression, loglevel, tag, format, ...) {\
		if(expression){\
			loglevel(tag, "%s[%d] >>> " format, __FUNCTION__, __LINE__ __VA_OPT__(,) __VA_ARGS__);\
			continue;\
		}\
}


#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_ERROR_CHECK_H_ */
