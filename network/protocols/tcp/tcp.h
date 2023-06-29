/*
 * tcp.h
 *
 *  Created on: Apr 4, 2023
 *      Author: anh
 */

#ifndef PROTOCOLS__TCP_H_
#define PROTOCOLS_TCP_H_

#include "net_protocols_config.h"
#if ENABLE_TCP

#ifdef __cplusplus
extern "C"{
#endif

#include "stdint.h"
#include "stddef.h"

#include "lwip/tcp.h"
#include "lwip/ip_addr.h"

/**
 * TCP port enum.
 */
typedef enum {
	TCP_FTP_PORT    = 21, /**< TCP_FTP_PORT */
	TCP_SSH_PORT    = 22, /**< TCP_SSH_PORT */
	TCP_TELNET_PORT = 23, /**< TCP_TELNET_PORT */
	TCP_SMTP_PORT   = 25, /**< TCP_SMTP_PORT */
	TCP_DNS_PORT    = 53, /**< TCP_DNS_PORT */
	TCP_HTTP_PORT   = 80, /**< TCP_HTTP_PORT */
	TCP_POP3_PORT   = 110,/**< TCP_POP3_PORT */
	TCP_IMAP_PORT   = 143,/**< TCP_IMAP_PORT */
	TCP_HTTPS_PORT  = 443,/**< TCP_HTTPS_PORT */
} tcp_portnum_t;

/**
 * TCP event enum.
 */
typedef enum {
	/**
	 * For TCP server.
	 */
	TCP_SERVER_EVENT_ACCEPT,

	/**
	 * For TCP client.
	 */
	TCP_CLIENT_EVENT_GETSERVERIP,
	TCP_CLIENT_EVENT_CONNECTED,
	TCP_CLIENT_EVENT_DISCONNECTED,
	TCP_CLIENT_EVENT_TRANSMITED,
	TCP_CLIENT_EVENT_RECEIVED,
} tcp_event_t;

/**
 * TCP ep enum.
 */
typedef enum{
	ES_NONE = 0,
	ES_CONNECTED,
	ES_RECEIVED,
	ES_CLOSING
} tcp_ep_status_t;

struct tcp_client_es_t{
	tcp_ep_status_t state;
	struct tcp_pcb *pcb;
	struct pbuf *buf;
};

typedef struct{
	void *connection = NULL;
	tcp_client_es_t *es;
} tcp_handler_t;

typedef struct {
	/** public settings */
	char *name = NULL;
	char *hostname = NULL;
	char *ip_str = (char *)"0.0.0.0";
	tcp_portnum_t port = TCP_HTTP_PORT;
	void *parameter = NULL;
	void (*event_handler)(tcp_event_t, void *) = NULL;
	uint32_t keepalive_time = 20;
	uint8_t  keepalive_count = 5;
	uint32_t keepalive_interval = 10;

	/** public data */
	void *data = NULL;
	uint16_t len = 0;
	/** private */
	struct tcp_pcb *conn_pcb;
	ip_addr_t ip;
	tcp_handler_t handler;
} tcp_connection_t;



#ifdef __cplusplus
}
#endif

#endif /* ENABLE_TCP */

#endif /* PROTOCOLS_TCP_H_ */
