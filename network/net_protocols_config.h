/*
 * net_protocols_config.h
 *
 *  Created on: Apr 1, 2023
 *      Author: anh
 */

#ifndef NETWORK_NET_PROTOCOLS_CONFIG_H_
#define NETWORK_NET_PROTOCOLS_CONFIG_H_


#define ENABLE_NETWORK_INTERFACE       1


#if ENABLE_NETWORK_INTERFACE
#define NETWORK_WAITIP_TIMEOUT_DEFAULT  5000U // 5s.
#define NETWORK_LOG                    1

/**
 * TCP.
 */
#define ENABLE_TCP                     1
#if ENABLE_TCP
#define TCP_LOG                        1
#define TCP_DEBUG                      1

#endif /* ENABLE_TCP */

/**
 * HTTP.
 */
#define ENABLE_HTTP                    0
#if ENABLE_HTTP
#define HTTP_LOG                       1
#define HTTP_DEBUG                     1

#endif /* ENABLE_HTTP */


#endif /* ENABLE_NETWORK_INTERFACE */

#endif /* NETWORK_NET_PROTOCOLS_CONFIG_H_ */
