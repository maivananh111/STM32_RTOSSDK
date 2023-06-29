/*
 * http_auth.h
 *
 *  Created on: Jun 19, 2023
 *      Author: anh
 */

#ifndef HTTP_COM_HTTP_AUTH_H_
#define HTTP_COM_HTTP_AUTH_H_

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"

typedef struct {
    char *method;       /*!< Request method, example: GET */
    char *algorithm;    /*!< Authentication algorithm */
    char *uri;          /*!< URI of request example: /path/to/file.html */
    char *realm;        /*!< Authentication realm */
    char *nonce;        /*!< Authentication nonce */
    char *qop;          /*!< Authentication qop */
    char *opaque;       /*!< Authentication opaque */
    uint64_t cnonce;    /*!< Authentication cnonce */
    int nc;             /*!< Authentication nc */
} http_auth_data_t;


char *http_auth_digest(const char *username, const char *password, http_auth_data_t *auth_data);

char *http_auth_basic(const char *username, const char *password);

#ifdef __cplusplus
}
#endif

#endif  /* HTTP_COM_HTTP_AUTH_H_ */
