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
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

static const char *TAG = "WiFiIF";
#if ENABLE_COMPONENT_WIFIIF_DEBUG
#define LOG_LEVEL LOG_DEBUG
#endif /* ENABLE_COMPONENT_WIFIIF_DEBUG */
static const char *command_string[] = {
	"WIFI_ERR",

	"WIFI_CHECK_CONNECTION",
	"WIFI_RESTART",
	"WIFI_REQUIRE_CONNECT",

	"WIFI_CONNECT",
	"WIFI_DISCONNECT",

	"WIFI_FIREBASE_INIT",
	"WIFI_FIREBASE_SET_DATA",
	"WIFI_FIREBASE_GET_DATA",
	"WIFI_FIREBASE_REMOVE_DATA",
	"WIFI_FIREBASE_RESPONSE",

	"WIFI_CMD_NUM",
};


static void (*fprequest)(char *str, uint16_t len) = NULL;
static void (*fpcommand_handler)(wifi_cmd_t cmd, void *param);
static volatile bool wifi_state = false, wifi_connected = false;
static SemaphoreHandle_t s_transmit, s_response;
static QueueHandle_t q_response;


static void wifiif_debug(char *str, int line, const char *func){
#if ENABLE_COMPONENT_WIFIIF_DEBUG
	LOG_LEVEL(TAG, "%s, Line: %d Function: %s", str, line, func);
#endif /* ENABLE_COMPONENT_WIFIIF_DEBUG */
}

static void wifiif_transmit(char *str){
	if(xSemaphoreTake(s_transmit, 10)){
		uint8_t MAX_UART_TX_BUFFER_SIZE = 100;
		int16_t len = strlen(str);
		int16_t remaining = len;
		while(remaining > 0){
			int16_t sendSize = (remaining > MAX_UART_TX_BUFFER_SIZE)? MAX_UART_TX_BUFFER_SIZE : remaining;
			if(fprequest) fprequest(str, sendSize);
			remaining -= sendSize;
			str += sendSize;
		}
		if(fprequest) fprequest((char *)"\r\nend\r\n", 7);
		xSemaphoreGive(s_transmit);
	}
}

void wifiif_get_break_data(char *brk_data){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if(strcmp(brk_data, "\r\nend\r\n") == 0) {
    	free(brk_data);
    	xSemaphoreGiveFromISR(s_response, &xHigherPriorityTaskWoken);
    }
    else{
    	if(xQueueSendFromISR(q_response, &brk_data, &xHigherPriorityTaskWoken) != pdTRUE) LOG_ERROR(TAG, "Send to queue fail.");
    }
}

static void wifiif_merge_data(char **dest_buffer){
 	   char *break_data;
 	   uint16_t total_len = 0;
 	   uint8_t queue_len = uxQueueMessagesWaiting(q_response);
 	   /** get total string length */
 	   for(uint8_t i=0; i<queue_len; i++){
 		   if(xQueueReceive(q_response, &break_data, 10) == pdTRUE){
 			   total_len += strlen(break_data);
 			   xQueueSend(q_response, &break_data, 10);
 		   }
 	   }

 	   /** string concatenate */
 	  *dest_buffer = (char *)malloc(total_len + 1);
 	   char *tmp_data = *dest_buffer;
 	   for(uint8_t i=0; i<queue_len; i++){
 		   if(xQueueReceive(q_response, &break_data, 10) == pdTRUE){
 			   uint16_t len = strlen(break_data);
 			   memcpy(tmp_data, break_data, len);
 			   tmp_data += len;
 			   free(break_data);
 		   }
 	   }
 	  (*dest_buffer)[total_len] = '\0';
}

static int wifiif_is_err(char *str){
	return strcmp(str, "ERR");
}

static void wifiif_request(wifi_cmd_t cmd, char *data){
	char *cmd_str = cmd_to_str(cmd, command_string);
	char *req_data;
	asprintf(&req_data, "%s: %s", cmd_str, data);
	wifiif_transmit(req_data);
#if ENABLE_COMPONENT_WIFIIF_DEBUG
//	wifiif_debug(req_data, __LINE__, __FUNCTION__);
#endif /* ENABLE_COMPONENT_WIFIIF_DEBUG */
	free(req_data);

	if(xQueueSemaphoreTake(s_response, WIFI_DEFAULT_TIMEOUT)){
		char *response_data;
		pkt_t pkt;
		pkt_err_t err = PKT_ERR_OK;

		wifiif_merge_data(&response_data);
		err = parse_packet(response_data, &pkt);
		if(err != PKT_ERR_OK){
			release_packet(&pkt);
			if(response_data != NULL) free(response_data);

			return;
		}
		if(wifiif_is_err(pkt.data_str) != 0){ // Is not wifi command error.
			char *data = (char *)malloc(strlen(pkt.data_str)+1);
			memcpy(data, pkt.data_str, strlen(pkt.data_str));
			data[strlen(pkt.data_str)] = '\0';

			wifi_cmd_t command = (wifi_cmd_t)str_to_cmd(pkt.cmd_str, command_string, WIFI_CMD_NUM);
			if(command == WIFI_CHECK_CONNECTION){
				pkt_json_t json;
				pkt_err_t err = json_get_object(pkt.data_str, &json, (char *)"state");
				if(err == PKT_ERR_OK){
					if(strcmp(json.value, "connected") == 0) {
						wifi_state = true;
						wifi_connected = true;
					}
					else if(strcmp(json.value, "disconnected") == 0) {
						wifi_state = false;
						wifi_connected = false;
					}
				}
				json_release_object(&json);
			}
			if(command == WIFI_RESTART || command == WIFI_REQUIRE_CONNECT){
				wifi_state = false;
				wifi_connected = false;
			}
			else if(command == WIFI_CONNECT){
				wifi_state = true;
				wifi_connected = true;
			}

			if(fpcommand_handler) fpcommand_handler(command, data);

			if(data != NULL) free(data);
		}
		else{ // Wifi command error.
			wifiif_debug((char *)"WiFi module error", __LINE__, __FUNCTION__);
			if(fpcommand_handler) fpcommand_handler(WIFI_ERR, NULL);
		}
		release_packet(&pkt);
		if(response_data != NULL) free(response_data);
	}
	else{
		wifiif_debug((char *)"WiFi module not response the request", __LINE__, __FUNCTION__);
		if(fpcommand_handler) fpcommand_handler(WIFI_ERR, NULL);
	}

}

/**
 * WiFi setup function.
 */
void wifiif_init(void (*prequest)(char *, uint16_t)){
	fprequest = prequest;

	q_response = xQueueCreate(20, sizeof(char *));
	s_transmit = xSemaphoreCreateBinary();
	s_response = xSemaphoreCreateBinary();
	xSemaphoreGive(s_transmit);

	fprequest((char *)"\r\nend\r\n", 7);
}

void wifiif_register_command_handler(void (*pcommand_handler)(wifi_cmd_t cmd, void *param)){
	fpcommand_handler = pcommand_handler;
}

/**
 * WiFi control.
 */
void wifiif_restart(void){
	wifiif_request(WIFI_RESTART, (char *)"{}");
	wifi_state = false;
	wifi_connected = false;
}

void wifiif_check_connection(void){
	wifiif_request(WIFI_CHECK_CONNECTION, (char *)"{}");
}

void wifiif_wifi_connect(char *ssid, char *pass, char *auth){
	char *data;
	asprintf(&data, "{\"ssid\":\"%s\",\"pass\":\"%s\",\"auth\":\"%s\"}", ssid, pass, auth);

	wifiif_request(WIFI_CONNECT, data);

	free(data);
}

void wifiif_disconnect(void){
	wifiif_request(WIFI_DISCONNECT, (char *)"{}");
}

/**
 * Firebase.
 */
void wifiif_firebase_init(char *project_url, char *auth){
	char *buffer;

	if(auth != NULL)
		asprintf(&buffer, "{\"url\":\"%s\",\"auth\":\"%s\"}", project_url, auth);
	else
		asprintf(&buffer, "{\"url\":\"%s\"}", project_url);

	wifiif_request(WIFI_FIREBASE_INIT, buffer);
	free(buffer);
}


void wifiif_firebase_set_data(char *path, char *data){
	char *buffer;

	asprintf(&buffer, "{\"path\":\"%s\",\"jsondata\":%s}", path, data);
	wifiif_request(WIFI_FIREBASE_SET_DATA, buffer);

	free(data);
}

void wifiif_firebase_get_data(char *path){
	char *buffer;
	asprintf(&buffer, "{\"path\":\"%s\"}", path);

	wifiif_request(WIFI_FIREBASE_GET_DATA, buffer);

	free(buffer);
}

void wifiif_firebase_remove_data(char *path){
	char *buffer;
	asprintf(&buffer, "{\"path\":\"%s\"}", path);

	wifiif_request(WIFI_FIREBASE_REMOVE_DATA, buffer);

	free(buffer);
}

bool wifiif_state_is_running(void){
	return wifi_state;
}
bool wifiif_wificonnected(void){
	return wifi_connected;
}


#endif /* ENABLE_COMPONENT_WIFIIF */
