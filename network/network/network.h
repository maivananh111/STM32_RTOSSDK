/*
 * network.h
 *
 *  Created on: Mar 31, 2023
 *      Author: anh
 */

#ifndef NETWORK_NETWORK_NETWORK_H_
#define NETWORK_NETWORK_NETWORK_H_

#include "net_protocols_config.h"
#if ENABLE_NETWORK_INTERFACE

#ifdef __cplusplus
extern "C"{
#endif

#include "lwip.h"
#include "lwip/netif.h"
#include "lwip/dhcp.h"
#include "lwip/ip_addr.h"
#include "lwip/etharp.h"


#define ETHERNET_LINK_TASK_SIZE_KBYTE 1U
#define ETHERNET_LINK_TASK_PRIORITY   16

typedef enum{
	NET_EVENT_LINKDOWN = 0,
	NET_EVENT_LINKUP,
	NET_EVENT_GOTIP,
} net_event_t;

/**
 * Get global net interface and ethernet handler API.
 */
struct netif *net_get_netif(void);
ETH_HandleTypeDef *net_get_eth(void);

/**
 * Initialize and link.
 */
void net_init_link(uint32_t waitIP_timeout);

void net_register_event_handler(void (*fp_link_handler)(net_event_t));

bool net_is_connected(void);

/**
 * Address API.
 */
uint8_t *net_get_mac_address(void);
char *net_get_mac_address_str(void);

void net_set_ip_address(ip_addr_t ip, ip_addr_t netmask, ip_addr_t gateway);
void net_set_ip_address_str(char * ip, char *netmask, char *gateway);
ip_addr_t *net_get_ip_address(void);
char *net_get_ip_address_str(void);
void net_wait_ip_address(void);

ip_addr_t *net_get_netmask(void);
char *net_get_netmask_str(void);

ip_addr_t *net_get_gateway_address(void);
char *net_get_netmask_gateway_str(void);

/**
 * DHCP API.
 */
void net_dhcp_start(void);
void net_dhcp_stop(void);



#ifdef __cplusplus
}
#endif

#endif /* ENABLE_NETWORK_INTERFACE */

#endif /* NETWORK_NETWORK_NETWORK_H_ */
