/*
 * http_utils.h
 *
 *  Created on: Jun 19, 2023
 *      Author: anh
 */

#ifndef HTTP_COM_HTTP_UTILS_H_
#define HTTP_COM_HTTP_UTILS_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "sys/time.h"


char *http_utils_assign_string(char **str, const char *new_str, int len);

char *http_utils_append_string(char **str, const char *new_str, int len);

void http_utils_trim_whitespace(char **str);

char *http_utils_get_string_between(const char *str, const char *begin, const char *end);

char *http_utils_join_string(const char *first_str, size_t len_first, const char *second_str, size_t len_second);

int http_utils_str_starts_with(const char *str, const char *start);


#ifdef __cplusplus
}
#endif

#endif /* HTTP_COM_HTTP_UTILS_H_ */
