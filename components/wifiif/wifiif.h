/*
 * wifiif.h
 *
 *  Created on: Apr 29, 2023
 *      Author: anh
 */

#ifndef COMPONENTS_WIFI_H_
#define COMPONENTS_WIFI_H_

#include "component_config.h"
#if ENABLE_COMPONENT_WIFIIF

#ifdef __cplusplus
extern "C"{
#endif

#include "sdkconfig.h"
#include st_header
#include "stdio.h"


#define WIFI_DEFAULT_TIMEOUT 15000U // 3S
#define DATA_EVENTBIT        (1<<15)


typedef enum {
	WIFI_ERR = 0x00,

	WIFI_CHECK_CONNECTION,
	WIFI_RESTART,
	WIFI_REQUIRE_CONNECT,

	WIFI_CONNECT,
	WIFI_DISCONNECT,

	WIFI_FIREBASE_INIT,
	WIFI_FIREBASE_SET_DATA,
	WIFI_FIREBASE_GET_DATA,
	WIFI_FIREBASE_REMOVE_DATA,
	WIFI_FIREBASE_RESPONSE,

	WIFI_CMD_NUM,
} wifi_cmd_t;


void wifiif_init(void (*prequest)(char *, uint16_t));
void wifiif_register_command_handler(void (*pcommand_handler)(wifi_cmd_t cmd, void *param));
void wifiif_get_break_data(char *brk_data);

void wifiif_restart(void);

void wifiif_check_connection(void);
void wifiif_wifi_connect(char *ssid, char *pass, char *auth);
void wifiif_wifi_disconnect(void);

void wifiif_firebase_init(char *project_url, char *auth);
void wifiif_firebase_set_data(char *path, char *data);
void wifiif_firebase_get_data(char *path);
void wifiif_firebase_remove_data(char *path);

bool wifiif_wificonnected(void);
bool wifiif_state_is_running(void);



#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_WIFIIF */

#endif /* COMPONENTS_WIFIIF_H_ */
