/*
 * http.h
 *
 *  Created on: Apr 4, 2023
 *      Author: anh
 */

#ifndef PROTOCOLS_HTTP_H_
#define PROTOCOLS_HTTP_H_

#include "net_protocols_config.h"
#if ENABLE_HTTP

#ifdef __cplusplus
extern "C"{
#endif

#include "stdio.h"
#include "stdint.h"


typedef enum{
	HTTP_HTTP_PORT = 80,
	HTTP_HTTPS_PORT = 443,
} http_port_t;

typedef enum{
	HTTP_TCP_TRANSPORT,
	HTTP_SSL_TRANSPORT,
} http_transport_t;

#define HTTP_0_9 (char *)"HTTP/0.9"
#define HTTP_1_0 (char *)"HTTP/1.0"
#define HTTP_1_1 (char *)"HTTP/1.1"

typedef enum {
    /* 2xx - Success */
    HTTP_STATUS_OK               = 200,
    /* 3xx - Redirection */
    HTTP_STATUS_MULTIPLECHOICES   = 300,
    HTTP_STATUS_MOVEDPERMANENTLY  = 301,
    HTTP_STATUS_FOUND             = 302,
    HTTP_STATUS_SEEOTHER          = 303,
    HTTP_STATUS_TEMPORARYREDIRECT = 307,
    HTTP_STATUS_PERMANENTREDIRECT = 308,
    /* 4xx - Client Error */
    HTTP_STATUS_BADREQUEST        = 400,
    HTTP_STATUS_UNAUTHORIZED      = 401,
    HTTP_STATUS_FORBIDDEN         = 403,
    HTTP_STATUS_NOTFOUND          = 404,
    /* 5xx - Server Error */
    HTTP_STATUS_INTERNALERROR     = 500
} http_status_t;

typedef enum{
    HTTP_METHOD_GET = 0,
    HTTP_METHOD_POST,
    HTTP_METHOD_PUT,
    HTTP_METHOD_PATCH,
    HTTP_METHOD_DELETE,
    HTTP_METHOD_HEAD,
    HTTP_METHOD_NOTIFY,
    HTTP_METHOD_SUBSCRIBE,
    HTTP_METHOD_UNSUBSCRIBE,
    HTTP_METHOD_OPTIONS,
    HTTP_METHOD_COPY,
    HTTP_METHOD_MOVE,
    HTTP_METHOD_LOCK,
    HTTP_METHOD_UNLOCK,
    HTTP_METHOD_PROPFIND,
    HTTP_METHOD_PROPPATCH,
    HTTP_METHOD_MKCOL,
    HTTP_METHOD_MAX,
} http_method_t;

extern const char *http_method_str[];


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_HTTP */

#endif /* PROTOCOLS_HTTP_H_ */
