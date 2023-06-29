/*
 * transport.h
 *
 *  Created on: Jun 26, 2023
 *      Author: anh
 */

#ifndef NETWORK_TRANSPORT_TRANSPORT_H_
#define NETWORK_TRANSPORT_TRANSPORT_H_

#include "net_protocols_config.h"

#if ENABLE_NETWORK_TRANSPORT

#ifdef __cplusplus
extern "C"{
#endif


#include "stdlib.h"

#include "sys/select.h"
#include "sys/_timeval.h"

#include "lwip/ip_addr.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

#if ENABLE_TRANSPORT_LAYER_SERCURITY
#include "tls/tls.h"
#endif /* ENABLE_TRANSPORT_LAYER_SERCURITY */


#define INVALID_SOCKET_FD                    -1
#define DEFAULT_SOCKET_NON_BLOCK 			 false
#define DEFAULT_SOCKET_TIMEOUT_S 			 5U
#define DEFAULT_SOCKET_KEEP_ALIVE_IDLE_S     10U
#define DEFAULT_SOCKET_KEEP_ALIVE_INTERVAL_S 1U
#define DEFAULT_SOCKET_KEEP_ALIVE_COUNT      10U
#define DEFUALT_SOCKET_CONNECT_TIMEOUT       10U

typedef enum{
#if ENABLE_TRANSPORT_LAYER_SERCURITY
	TRANSPORT_ERR_TLS_CREATE 	   = -14,
	TRANSPORT_ERR_TLS_HANDSHAKE    = -13,
#endif /* ENABLE_TRANSPORT_LAYER_SERCURITY */
	TRANSPORT_ERR_RESOLVE_HOSTNAME = -12,
	TRANSPORT_ERR_CREATE      	   = -11,
	TRANSPORT_ERR_INVALID_PROTOCOL = -10,
	TRANSPORT_ERR_SET_OPTION 	   = -9,
	TRANSPORT_ERR_GET_OPTION 	   = -8,
	TRANSPORT_ERR_SET_IFNAME 	   = -7,
	TRANSPORT_ERR_OPEN 			   = -6,
	TRANSPORT_ERR_CLOSE 		   = -5,
	TRANSPORT_ERR_POLL 			   = -4,
	TRANSPORT_ERR_READ 			   = -3,
	TRANSPORT_ERR_WRITE 		   = -2,
	TRANSPORT_ERR_NONE 			   = 1,
} transport_error_t;

typedef enum{
	TRANSPORT_POLL_READ  = 0x01,
	TRANSPORT_POLL_WRITE = 0x02,
} transport_polloption_t;

typedef enum{
	TRANSPORT_STATE_DISCONNECTED = -2,
	TRANSPORT_STATE_ERROR = -1,
	TRANSPORT_STATE_CONNECTED = 0,
	TRANSPORT_STATE_PASS,
	TRANSPORT_STATE_READY,
	TRANSPORT_STATE_TIMEOUT,
} transport_state_t;

typedef enum{
	TRANSPORT_OVER_TCP,
#if ENABLE_TRANSPORT_LAYER_SERCURITY
	TRANSPORT_OVER_SSL,
#endif /* ENABLE_TRANSPORT_LAYER_SERCURITY */
} transport_type_t;

typedef enum{
	TRANSPORT_EVENT_ERROR,
	TRANSPORT_EVENT_DISCONNECTED_BY_PEER,
	TRANSPORT_EVENT_DISCONNECTED,
	TRANSPORT_EVENT_CONNECTED,
	TRANSPORT_EVENT_ON_DATA,
	TRANSPORT_EVENT_WRITTEN,
	TRANSPORT_EVENT_TIMEOUT,
} transport_event_t;

typedef void (*event_handler_function)(transport_event_t event, void *data);

typedef struct{
	/**
	 * Main configuration.
	 */
	char *hostname = NULL;
	char *ipaddress = NULL;
	int  port = 0;

	int  timeout_s = 0;
	int  keepalive_idle_s = 0;
	int  keepalive_interval_s = 0;
	int  keepalive_count = 0;
	bool socket_non_block = false;
	char *socket_if_name = NULL;

	transport_type_t transport_type = TRANSPORT_OVER_TCP;
	bool use_crt_bundle = false;
	char *server_cert_pem = NULL;
	bool renegotiation = true;
	char **alpn_protocols = NULL;
} transport_config_t;



class transport_t{
	public:
		transport_t(void);

		void register_event_handler(event_handler_function f_event_handler);
		transport_state_t connect_to_host(transport_config_t *config);
		transport_state_t disconnect_to_host(void);

		transport_state_t poll_for_ready(transport_polloption_t pollopt, int timeout = 0);
		transport_state_t write_data(unsigned char *buffer, int len, int *byte_write);
		transport_state_t read_data(unsigned char *buffer, int len, int *byte_read);

		transport_state_t state(void);
		transport_error_t error(void);

		bool can_continue(void);
		bool acction_pass(void);


	private:
		transport_config_t *_config;

		transport_state_t _state;
		transport_error_t _error;

		int _sockfd = INVALID_SOCKET_FD;
		ip_addr_t _host_ipaddr;
		struct addrinfo *_addr_info;
		struct sockaddr_storage _sock_addr;
		struct timeval _timeout;

		event_handler_function fp_event_handler;
#if ENABLE_TRANSPORT_LAYER_SERCURITY
		tls_t _tls = tls_t(TLS_MODE_CLIENT);
#endif /* ENABLE_TRANSPORT_LAYER_SERCURITY */

		transport_error_t create_socket(void);
		transport_error_t set_socket_option(void);
		transport_error_t open_socket(void);

		transport_error_t set_socket_timeout(void);
		transport_error_t set_socket_keep_alive(void);
		transport_error_t set_socket_non_block_mode(bool mode);
		transport_error_t set_socket_ifname(void);

		transport_error_t close_socket(void);
};


#ifdef __cplusplus
}
#endif

#endif /* ENABLE_NETWORK_TRANSPORT */

#endif /* NETWORK_TRANSPORT_TRANSPORT_H_ */
