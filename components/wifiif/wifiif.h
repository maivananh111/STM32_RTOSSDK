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


#define WIFI_DEFAULT_TIMEOUT 10000U // 3S

typedef enum {
	WIFI_ERR = 0x00,

	WIFI_RESTART,
	/**
	 * Network control command.
	 */
	WIFI_SCAN,
	WIFI_ISCONNECTED,
	WIFI_CONN,
	WIFI_DISCONN,
	WIFI_GETIP,

	/**
	 * HTTP client command.
	 */
	WIFI_HTTP_CLIENT_NEW,
	WIFI_HTTP_CLIENT_CONFIG,
	WIFI_HTTP_CLIENT_INIT,
	WIFI_HTTP_CLIENT_CLEAN,
	WIFI_HTTP_CLIENT_SET_HEADER,
	WIFI_HTTP_CLIENT_SET_URL,
	WIFI_HTTP_CLIENT_SET_METHOD,
	WIFI_HTTP_CLIENT_SET_DATA,
	WIFI_HTTP_CLIENT_REQUEST,
	WIFI_HTTP_CLIENT_RESPONSE,

	WIFI_CMD_NUM,
} wifi_cmd_t;


void wifiif_register_request_function(void (*prequest)(char *));
void wifiif_register_command_handler(void (*pcommand_handler)(wifi_cmd_t cmd, void *param));

void wifiif_set_response_state(char *resp_data);
void wifiif_reset_response_state(void);

void wifiif_restart(void);
void wifiif_scan(void);
void wifiif_isconnect(void);
void wifiif_connect(char *ssid, char *pass, char *auth);
void wifiif_disconnect(void);
void wifiif_getIP(void);

void wifiif_http_client_new(void);
void wifiif_http_client_config(char *config);
void wifiif_http_client_init(void);
void wifiif_http_client_clean(void);
void wifiif_http_client_set_header(char *key, char *value);
void wifiif_http_client_set_url(char *url);
void wifiif_http_client_set_method(char *method);
void wifiif_http_client_set_data(char *data);
void wifiif_http_client_request(void);

void wifiif_state_running(bool state);
bool wifiif_state_is_running(void);



#ifdef __cplusplus
}
#endif

#endif /* ENABLE_COMPONENT_WIFIIF */

#endif /* COMPONENTS_WIFIIF_H_ */
