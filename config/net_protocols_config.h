/*
 * net_protocols_config.h
 *
 *  Created on: Apr 1, 2023
 *      Author: anh
 */

#ifndef NETWORK_NET_PROTOCOLS_CONFIG_H_
#define NETWORK_NET_PROTOCOLS_CONFIG_H_

#include "sdkconfig.h"
#include "peripheral_config.h"

#define ETH_OVER_SPI                   1
#if (defined(ETH) || ETH_OVER_SPI) && ENABLE_ETH
#define ENABLE_NETWORK_INTERFACE       1


#if ENABLE_NETWORK_INTERFACE
#define NETWORK_WAITIP_TIMEOUT_DEFAULT   5000U // 5s.
#define NETWORK_LOG                      1
#define ENABLE_NETWORK_TRANSPORT         1
#define ENABLE_TRANSPORT_LAYER_SERCURITY 1

#if ENABLE_NETWORK_TRANSPORT
#define NETWORK_TRANSPORT_SSL          1
#define NETWORK_TRANSPORT_LOG          1
#define NETWORK_TRANSPORT_DEBUG        1


#endif /* ENABLE_NETWORK_TRANSPORT */
#endif /* ENABLE_NETWORK_INTERFACE */
#endif /* (defined(ETH) || ETH_OVER_SPI) && ENABLE_ETH */
#endif /* NETWORK_NET_PROTOCOLS_CONFIG_H_ */
