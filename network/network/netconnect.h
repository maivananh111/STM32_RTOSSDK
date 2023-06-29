/*
 * netconnect.h
 *
 *  Created on: Mar 31, 2023
 *      Author: anh
 */

#ifndef NETWORK_NETCONNECT_H_
#define NETWORK_NETCONNECT_H_

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


#define ETHERNET_LINK_TASK_SIZE     4096 /* Bytes */
#define ETHERNET_LINK_TASK_PRIORITY 16   /* osPriorityBelowNormal */
/**
 * Get global net interface and ethernet handler API.
 */
struct netif *net_get_netif(void);
ETH_HandleTypeDef *net_get_eth(void);

/**
 * Initialize and net configuration API.
 */
void net_init_start(uint32_t waitIP_timeout);

bool net_is_connected(void);

void net_linkup_register_event_handler(void (*link_callback)(void));
void net_linkdown_register_event_handler(void (*link_callback)(void));

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

#endif /* NETWORK_NETCONNECT_H_ */

#endif /* NETWORK_NETCONNECT_H_ */
