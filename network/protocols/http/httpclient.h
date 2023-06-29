/*
 * httpclient.h
 *
 *  Created on: Apr 4, 2023
 *      Author: anh
 */

#ifndef PROTOCOLS_HTTPCLIENT_H_
#define PROTOCOLS_HTTPCLIENT_H_

#include "net_protocols_config.h"
#if ENABLE_HTTP

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "stddef.h"

#include "protocols/http/http.h"

#include <netdb.h>

#include "lwip/err.h"
#include "lwip/ip_addr.h"
#include "lwip/sockets.h"

#include "mbedtls/ssl.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/net_sockets.h"


#define HTTPCLIENT_DEFAULT_CONNECTION_HEADER "close"
#define HTTPCLIENT_DEFAULT_USER_AGENT_HEADER "STMicroelectronic"
#define HTTPCLIENT_DEFAULT_CONTENT_TYPE_HEADER "text/plain"
#define HTTPCLIENT_DEFAULT_ACCEPT_HEADER "/"

#define HTTP_MAX_RESP_LEN 1024

typedef enum{
    HTTP_EVENT_ERROR = 0,
	HTTP_EVENT_CONNECT_ERROR,
    HTTP_EVENT_CONNECTED,
    HTTP_EVENT_DISCONNECTED,
	HTTP_EVENT_SSL_ERROR,
    HTTP_EVENT_REQUEST,
	HTTP_EVENT_REQUEST_FAIL,
	HTTP_EVENT_RESPONSE,
	HTTP_EVENT_NOT_RESPONSE,
} httpclient_event_t;

typedef struct{
	httpclient_event_t event = HTTP_EVENT_CONNECTED;
	char *data = NULL;
	uint16_t data_len = 0;
} httpclient_event_handler_t;

struct httpclient;
typedef struct httpclient{
	/**
	 * client settings.
	 */
	char *hostname = NULL;
	char *url = NULL;
	http_port_t port = HTTP_HTTP_PORT;
	char *version = HTTP_1_1;
	http_transport_t transport = HTTP_TCP_TRANSPORT;
	const char *cert_pem = NULL;
	void (*event_handler)(httpclient *client, httpclient_event_handler_t *event, void *param);
	void *parameter = NULL;

	uint32_t keepalive_time = 20;
	uint8_t  keepalive_count = 5;
	uint32_t keepalive_interval = 10;

	/**
	 * action settings.
	 */
	http_method_t method = HTTP_METHOD_GET;
	http_status_t status = HTTP_STATUS_OK;

	/** default header key */
	char *connection  = NULL;
	char *content_type = NULL;
	char *content_length = (char *)"0";
	char *accept       = NULL;

	char *requestline = NULL;

	char *header      = NULL;
	char *default_header = NULL;
	char *other_header   = NULL;

	char *packet = NULL;

	char *data        = NULL;
	/**
	 * Privates.
	 */
	ip_addr_t ip;
	mbedtls_net_context sock_fd;
	struct addrinfo hints;
	struct addrinfo *sock_info;
	fd_set r_fd, w_fd;

	mbedtls_ssl_context ssl;
	mbedtls_ssl_config conf;
	mbedtls_x509_crt cacert;
	mbedtls_entropy_context entropy;
	mbedtls_ctr_drbg_context ctr_drbg;

	httpclient_event_handler_t *event;
} httpclient_t;


err_t httpclient_init(httpclient_t *client);

err_t httpclient_connect(httpclient_t *client);
err_t httpclient_disconnect(httpclient_t *client);

void httpclient_set_header(httpclient_t *client, char *newheader);
void httpclient_add_header(httpclient_t *client, char *key, char *value);
void httpclient_remove_header(httpclient_t *client, char *key);

err_t httpclient_request(httpclient_t *client, char *buffer, uint16_t len);



#ifdef __cplusplus
}
#endif

#endif /* ENABLE_HTTP */

#endif /* PROTOCOLS_HTTPCLIENT_H_ */
