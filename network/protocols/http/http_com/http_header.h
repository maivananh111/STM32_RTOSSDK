/*
 * http_header.h
 *
 *  Created on: Jun 19, 2023
 *      Author: anh
 */

#ifndef HTTP_COM_HTTP_HEADER_H_
#define HTTP_COM_HTTP_HEADER_H_

#include "sys/queue.h"
#include "system/res_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct http_header *http_header_handle_t;
typedef struct http_header_item *http_header_item_handle_t;

http_header_handle_t http_header_init(void);

error_t http_header_clean(http_header_handle_t header);

error_t http_header_destroy(http_header_handle_t header);

error_t http_header_set(http_header_handle_t header, const char *key, const char *value);

int http_header_set_format(http_header_handle_t header, const char *key, const char *format, ...);

error_t http_header_get(http_header_handle_t header, const char *key, char **value);

int http_header_generate_string(http_header_handle_t header, int index, char *buffer, int *buffer_len);

error_t http_header_delete(http_header_handle_t header, const char *key);

#ifdef __cplusplus
}
#endif

#endif /* HTTP_COM_HTTP_HEADER_H_ */
