/*
 * network_interface.cpp
 *
 *  Created on: Mar 31, 2023
 *      Author: anh
 */

#include "net_protocols_config.h"
#if ENABLE_NETWORK_INTERFACE

#include "network/netconnect.h"

#include "sdkconfig.h"
#include "stdio.h"
#if NETWORK_LOG
#include "system/log.h"
#endif /* NETWORK_LOG */
#include "periph/systick.h"



/**
 * Extern ethernet, net interface struct and variable.
 */
#if NETWORK_LOG
static const char *TAG = "NETIF";
#endif /* NETWORK_LOG */
extern struct netif gnetif;
extern ETH_HandleTypeDef heth;

extern ip4_addr_t ipaddr;
extern ip4_addr_t netmask;
extern ip4_addr_t gw;

void (*linkup_event_handler)(void) = NULL;
void (*linkdown_event_handler)(void) = NULL;


static __IO uint32_t _waitIP_timeout = 0xFFFFFFFF;
static bool connected = false;

static void link_event_handler(struct netif *);
static void tcpip_init_done(void *);
/**
 * Get global net interface and ethernet handler API.
 */
struct netif *net_get_netif(void){
	return (&gnetif);
}
ETH_HandleTypeDef *net_get_eth(void){
	return (&heth);
}

/**
 * Initialize and net configuration API.
 */
void net_init_start(uint32_t waitIP_timeout){
	_waitIP_timeout = waitIP_timeout;
	/* Initialize the LwIP stack */
	tcpip_init(tcpip_init_done, NULL);

	/* IP addresses initialization with DHCP (IPv4) */
	ipaddr.addr = 0;
	netmask.addr = 0;
	gw.addr = 0;

	/* Add the network interface (IPv4/IPv6) */
	netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

	/* Registers the default network interface */
	netif_set_default(&gnetif);

	/* Check first link event while starting */
	link_event_handler(&gnetif);

	/* Set the link event handler function*/
	netif_set_link_callback(&gnetif, link_event_handler);

	/* Start task check link event */
	xTaskCreate(ethernet_link_thread, "ETH check link", byte_to_word(ETHERNET_LINK_TASK_SIZE), &gnetif, ETHERNET_LINK_TASK_PRIORITY, NULL);
}

bool net_is_connected(void){
	return connected;
}

void net_linkup_register_event_handler(void (*link_callback)(void)){
	linkup_event_handler = link_callback;
}
void net_linkdown_register_event_handler(void (*link_callback)(void)){
	linkdown_event_handler = link_callback;
}

static void link_event_handler(struct netif *netif){
	if (netif_is_up(netif) || netif_is_link_up(netif)){
		netif_set_up(&gnetif);
		dhcp_start(&gnetif);

#if NETWORK_LOG
		LOG_INFO(TAG, "Ethernet link up.");
#endif /* NETWORK_LOG */

		/* Registers the default network interface */
		netif_set_default(&gnetif);

		net_wait_ip_address();

		if(linkup_event_handler) linkup_event_handler();

	}
	else {
		netif_set_down(&gnetif);

#if NETWORK_LOG
		LOG_INFO(TAG, "Ethernet link down.");
#endif /* NETWORK_LOG */

		if(linkdown_event_handler) linkdown_event_handler();
	}
}

void net_wait_ip_address(void){
	__IO uint32_t tick = get_tick();
	while(!dhcp_supplied_address(&gnetif)){
		delay_ms(100);
		if(get_tick() - tick > _waitIP_timeout){
			LOG_ERROR(TAG, "Can't get IP address form dhcp, access to network failed.");

			netif_set_down(&gnetif);
#if NETWORK_LOG
			LOG_INFO(TAG, "Ethernet link down.");
#endif /* NETWORK_LOG */
			if(linkdown_event_handler) linkdown_event_handler();

			return;
		}
	}

#if NETWORK_LOG
	LOG_INFO(TAG, "Ethernet MAC address: %02x:%02x:%02x:%02x:%02x:%02x",
					gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2], gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);
	char ip_str[16];
	sprintf(ip_str, "%s", ip4addr_ntoa(netif_ip4_addr(&gnetif)));
	LOG_INFO(TAG, "Ethernet IP address : %s", ip_str);
	sprintf(ip_str, "%s", ip4addr_ntoa(netif_ip4_netmask(&gnetif)));
	LOG_INFO(TAG, "Ethernet subnet mask: %s", ip_str);
	sprintf(ip_str, "%s", ip4addr_ntoa(netif_ip4_gw(&gnetif)));
	LOG_INFO(TAG, "Ethernet gateway    : %s", ip_str);
#endif /* NETWORK_LOG */
}

static void tcpip_init_done(void *){
#if NETWORK_LOG
	LOG_INFO(TAG, "TCP/IP Stack initialize done.");
#endif /* NETWORK_LOG */
}

/**
 * DHCP API.
 */
void net_dhcp_start(void){
	dhcp_start(&gnetif);
}
void net_dhcp_stop(void){
	dhcp_stop(&gnetif);
}

/**
 * Address API.
 */
// MAC address.
uint8_t *net_get_mac_address(void){
	return (uint8_t *)(&gnetif.hwaddr[0]);
}
char *net_get_mac_address_str(void){
	char *mac = (char *)malloc(18);

	sprintf(mac, "%02x:%02x:%02x:%02x:%02x:%02x",
			gnetif.hwaddr[0], gnetif.hwaddr[1], gnetif.hwaddr[2], gnetif.hwaddr[3], gnetif.hwaddr[4], gnetif.hwaddr[5]);

	return mac;
}

// IP address.
void net_set_ip_address(ip_addr_t ip, ip_addr_t netmask, ip_addr_t gateway){
	netif_set_addr(&gnetif, (ip4_addr_t *)&ip, (ip4_addr_t *)&netmask, (ip4_addr_t *)&gateway);
}
void net_set_ip_address_str(char *ip, char *netmask, char *gateway){
	ip4_addr_t _ip, _netmask, _gateway;

	ip4addr_aton(ip,      &_ip);
	ip4addr_aton(netmask, &_netmask);
	ip4addr_aton(gateway, &_gateway);

	netif_set_addr(&gnetif, &_ip, &_netmask, &_gateway);

	LOG_WARN(TAG, "Ethernet changed to new IP address.");
}
ip_addr_t *net_get_ip_address(void){
	return (ip_addr_t *)netif_ip4_addr(&gnetif);
}
char *net_get_ip_address_str(void){
	return ip4addr_ntoa(netif_ip4_addr(&gnetif));
}

// Subnet mask.
ip_addr_t *net_get_netmask(void){
	return (ip_addr_t *)netif_ip4_netmask(&gnetif);
}
char *net_get_netmask_str(void){
	return ip4addr_ntoa(netif_ip4_netmask(&gnetif));
}

// Gateway address.
ip_addr_t *net_get_gateway_address(void){
	return (ip_addr_t *)netif_ip4_gw(&gnetif);
}
char *net_get_netmask_gateway_str(void){
	return ip4addr_ntoa(netif_ip4_gw(&gnetif));
}




#endif /* ENABLE_NETWORK_INTERFACE */
