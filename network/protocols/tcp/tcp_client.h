/*
 * tcp_client.h
 *
 *  Created on: Apr 4, 2023
 *      Author: anh
 */

#ifndef PROTOCOLS_TCP_CLIENT_H_
#define PROTOCOLS_TCP_CLIENT_H_

#include "net_protocols_config.h"
#if ENABLE_TCP

#ifdef __cplusplus
extern "C"{
#endif

#include "tcp.h"
#include "system/ret_err.h"



err_t tcp_client_connect(tcp_connection_t *tcp);

void tcp_client_send_data(tcp_connection_t *tcp, char *data);

void tcp_client_disconnect(tcp_connection_t *tcp);

#ifdef __cplusplus
}
#endif /* ENABLE_TCP */

#endif /* ENABLE_TCP */

#endif /* PROTOCOLS_TCP_CLIENT_H_ */
