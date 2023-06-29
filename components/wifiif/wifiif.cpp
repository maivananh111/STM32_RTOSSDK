/*
 * wifi.cpp
 *
 *  Created on: Apr 29, 2023
 *      Author: anh
 */

#include "component_config.h"
#if ENABLE_COMPONENT_WIFIIF

#include "wifiif/wifiif.h"
#include "parse_packet/parse_packet.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#include "system/log.h"
#include "periph/systick.h"

#include "FreeRTOS.h"
#include "task.h"

#if ENABLE_COMPONENT_WIFIIF_DEBUG
static const char *TAG = "WiFiIF";
#define LOG_LEVEL LOG_DEBUG
#endif /* ENABLE_COMPONENT_WIFIIF_DEBUG */
static const char *command_string[] = {
	"WIFI_ERR",

	"WIFI_RESTART",
	/**
	 * Network control command.
	 */
	"WIFI_SCAN",
	"WIFI_ISCONNECTED",
	"WIFI_CONN",
	"WIFI_DISCONN",
	"WIFI_GETIP",

	/**
	 * HTTP client command.
	 */
	"WIFI_HTTP_CLIENT_NEW",
	"WIFI_HTTP_CLIENT_CONFIG",
	"WIFI_HTTP_CLIENT_INIT",
	"WIFI_HTTP_CLIENT_CLEAN",
	"WIFI_HTTP_CLIENT_SET_HEADER",
	"WIFI_HTTP_CLIENT_SET_URL",
	"WIFI_HTTP_CLIENT_SET_METHOD",
	"WIFI_HTTP_CLIENT_SET_DATA",
	"WIFI_HTTP_CLIENT_REQUEST",
	"WIFI_HTTP_CLIENT_RESPONSE",

	"WIFI_CMD_NUM",
};


static void (*fprequest)(char *str) = NULL;
static void (*fpcommand_handler)(wifi_cmd_t cmd, void *param);
static bool had_response = false;
static char *response_data;

static volatile bool wifi_state = false;

static void wifiif_debug(char *str, int line, char *func){
#if ENABLE_COMPONENT_WIFIIF_DEBUG
	LOG_LEVEL(TAG, "%s, Line: %d Function: %s", str, line, func);
#endif /* ENABLE_COMPONENT_WIFIIF_DEBUG */
}

static int wifiif_wait_response(void){
	__IO uint32_t tick = get_tick();

	while(!had_response){
		if(get_tick() - tick > WIFI_DEFAULT_TIMEOUT){
			return 0;
		}
		vTaskDelay(100);
	}

	return 1;
}

static int wifiif_is_err(char *str){
	return strcmp(str, "ERR");
}

static void wifiif_request(wifi_cmd_t cmd, char *data){
	char *cmd_str = cmd_to_str(cmd, command_string);
	char *req_data;
	asprintf(&req_data, "%s: %s", cmd_str, data);
	fprequest(req_data);
#if ENABLE_COMPONENT_WIFIIF_DEBUG
	wifiif_debug(req_data, __LINE__, (char *)__FUNCTION__);
#endif /* ENABLE_COMPONENT_WIFIIF_DEBUG */
	free(req_data);

	int resp_stt = wifiif_wait_response();
	if(resp_stt){
		pkt_t pkt;
		pkt_err_t err = parse_packet(response_data, &pkt);
		if(err != PKT_ERR_OK){
			wifiif_debug((char *)"Can't parse response.", __LINE__, (char *)__FUNCTION__);

			release_packet(&pkt);
			wifiif_reset_response_state();

			return;
		}
		if(wifiif_is_err(pkt.data_str) != 0){
			char *data = (char *)malloc(strlen(pkt.data_str)+1);
			memcpy(data, pkt.data_str, strlen(pkt.data_str));
			data[strlen(pkt.data_str)] = '\0';

			wifi_cmd_t command = cmd;
			if(command == WIFI_HTTP_CLIENT_REQUEST) command = WIFI_HTTP_CLIENT_RESPONSE;
			if(fpcommand_handler) fpcommand_handler(command, data);

			if(data != NULL) free(data);
		}
		else{
			wifiif_debug((char *)"WiFi module error.", __LINE__, (char *)__FUNCTION__);
			if(fpcommand_handler) fpcommand_handler(WIFI_ERR, NULL);
		}
		release_packet(&pkt);
	}
	else{
		wifiif_debug((char *)"WiFi module not response the request.", __LINE__, (char *)__FUNCTION__);
		if(fpcommand_handler) fpcommand_handler(WIFI_ERR, NULL);
	}

	wifiif_reset_response_state();
}
/**
 * WiFi setup function.
 */
void wifiif_register_request_function(void (*prequest)(char *)){
	fprequest = prequest;
}

void wifiif_register_command_handler(void (*pcommand_handler)(wifi_cmd_t cmd, void *param)){
	fpcommand_handler = pcommand_handler;
}

void wifiif_set_response_state(char *resp_data){
	had_response = true;
	response_data = resp_data;
}

void wifiif_reset_response_state(void){
	had_response = false;
	if(response_data != NULL){
		free(response_data);
		response_data = NULL;
	}
}

/**
 * WiFi control.
 */
void wifiif_restart(void){
	wifiif_request(WIFI_RESTART, (char *)"{}");
}
void wifiif_scan(void){
	wifiif_request(WIFI_SCAN, (char *)"{}");
}
void wifiif_isconnect(void){
	wifiif_request(WIFI_ISCONNECTED, (char *)"{}");
}
void wifiif_connect(char *ssid, char *pass, char *auth){
	char *data;
	asprintf(&data, "{\"ssid\":\"%s\",\"pass\":\"%s\",\"auth\":\"%s\"}", ssid, pass, auth);

	wifiif_request(WIFI_CONN, data);

	free(data);
}
void wifiif_disconnect(void){
	wifiif_request(WIFI_DISCONN, (char *)"{}");
}
void wifiif_getIP(void){
	wifiif_request(WIFI_GETIP, (char *)"{}");
}




/**
 * HTTP Client.
 */
void wifiif_http_client_new(void){
	wifiif_request(WIFI_HTTP_CLIENT_NEW, (char *)"{}");
}

void wifiif_http_client_config(char *config){
	wifiif_request(WIFI_HTTP_CLIENT_CONFIG, config);
}

void wifiif_http_client_init(void){
	wifiif_request(WIFI_HTTP_CLIENT_INIT, (char *)"{}");
}

void wifiif_http_client_clean(void){
	wifiif_request(WIFI_HTTP_CLIENT_CLEAN, (char *)"{}");
}

void wifiif_http_client_set_header(char *key, char *value){
	char *data;
	asprintf(&data, "{\"key\":\"%s\",\"value\":\"%s\"}", key, value);

	wifiif_request(WIFI_HTTP_CLIENT_SET_HEADER, data);

	free(data);
}

void wifiif_http_client_set_url(char *url){
	char *tmp;
	asprintf(&tmp, "{\"url\":\"%s\"}", url);

	wifiif_request(WIFI_HTTP_CLIENT_SET_URL, tmp);

	free(tmp);
}

void wifiif_http_client_set_method(char *method){
	char *tmp;
	asprintf(&tmp, "{\"method\":\"%s\"}", method);

	wifiif_request(WIFI_HTTP_CLIENT_SET_METHOD, tmp);

	free(tmp);
}

void wifiif_http_client_set_data(char *data){
	char *tmp;
	asprintf(&tmp, "{\"data\":%s}", data);

	wifiif_request(WIFI_HTTP_CLIENT_SET_DATA, tmp);

	free(tmp);
}

void wifiif_http_client_request(void){
	wifiif_request(WIFI_HTTP_CLIENT_REQUEST, (char *)"{}");
}


void wifiif_state_running(bool state){
	wifi_state = state;
}
bool wifiif_state_is_running(void){
	return wifi_state;
}



#endif /* ENABLE_COMPONENT_WIFIIF */
