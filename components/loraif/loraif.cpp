/*
 * loraif.cpp
 *
 *  Created on: Apr 29, 2023
 *      Author: anh
 */

#include "component_config.h"
#if ENABLE_COMPONENT_LORAIF

#include "loraif/loraif.h"
#include "parse_packet/parse_packet.h"
#include "crc/crc.h"

#include "stdlib.h"
#include "string.h"

#include "system/log.h"
#include "periph/systick.h"
#include "periph/rng.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

using namespace std;


#if ENABLE_COMPONENT_LORAIF_DEBUG
static const char *TAG = "LoRaIF";
#define LOG_LEVEL LOG_DEBUG
#endif /* ENABLE_COMPONENT_LORAIF_DEBUG */
static const char *command_string[] = {
	"LORA_ERR",

	"LORA_REQ_ADDRESS",
	"LORA_UPDATE_ADDRESS",
	"LORA_UPDATE_STATE",

	"LORA_UPDATE_SETTINGS",
	"LORA_REQ_DATA",
	"LORA_UPDATE_DATA",

	"LORA_DEL_DEVICE",

	"LORA_CMD_NUM",
};

sx127x *loraif;
static void (*fpeventhandler)(lora_event_t event, char *data);

static uint32_t req_address;
static bool req_had_resp = false;
static lora_event_t req_cmd;

tim_t timer;
uint32_t resp_timeout;
uint8_t max_not_resp;

QueueHandle_t response_queue;
SemaphoreHandle_t tranfer_smp;
list<loraif_dev_t *> loraif_device_list;

static void loraif_transmit(char *str);
static void loraif_debug(char *str, int line, const char *func);
static void show_device_list(void);
static bool device_is_available(uint32_t num);

static void loraif_debug(char *str, int line, const char *func){
#if ENABLE_COMPONENT_LORAIF_DEBUG
	LOG_LEVEL(TAG, "%s, Line: %d Function: %s", str, line, func);
#endif /* ENABLE_COMPONENT_LORAIF_DEBUG */
}

static void show_device_list(void){
    for (auto device = loraif_device_list.begin(); device != loraif_device_list.end(); ++device) {
    	char *tmp;
    	asprintf(&tmp, "Device 0x%08x.", (*device)->address);
    	loraif_debug(tmp, __LINE__, __FUNCTION__);
    	free(tmp);
    }
}

static bool device_is_available(uint32_t num){
    for (auto device = loraif_device_list.begin(); device != loraif_device_list.end(); ++device) {
        if ((*device)->address == num) {
            return true;
        }
    }

    return false;
}


void loraif_init(sx127x *lora, tim_t tim, uint32_t timeout, uint8_t max_not_response){
	loraif = lora;
	timer = tim;
	tim_config_t conf = {
		.prescaler = 43200,
		.reload = 0xFFFF,
	};
	timer->init(&conf);

	resp_timeout = timeout;
	max_not_resp = max_not_response;
	response_queue = xQueueCreate(5, sizeof(uint32_t));
	tranfer_smp = xSemaphoreCreateBinary();
	xSemaphoreGive(tranfer_smp);
}

void loraif_register_event_handler(void (*peventhandler)(lora_event_t event, char *data)){
	fpeventhandler = peventhandler;
}

bool loraif_check_crc(char *data){
	char *src_cpy = data;
	char *crc_start;
	uint16_t crc, icrc, len;
	pkt_t pkt;
	pkt_err_t err;
	pkt_json_t json;

	crc_start = strstr(src_cpy, "\"crc\"");
	if(crc_start == NULL) return false;

	err = parse_packet(src_cpy, &pkt);
	if(err != PKT_ERR_OK) return false;
	err = json_get_object(pkt.data_str, &json, (char *)"crc");
	if(err != PKT_ERR_OK) return false;
	icrc = strtol(json.value, NULL, 16);
	json_release_object(&json);
	release_packet(&pkt);

	len = (uint32_t)crc_start - (uint32_t)src_cpy;
	crc = cal_crc16((uint8_t *)src_cpy, len);
	if(crc != icrc) return false;

	return true;
}

static void loraif_transmit(char *str){
	if(xSemaphoreTake(tranfer_smp, portMAX_DELAY)){
		loraif->beginPacket();
		loraif->transmit((uint8_t *)str, (size_t)strlen(str));
		loraif->endPacket();
		loraif->Receive(0);
		xSemaphoreGive(tranfer_smp);
	}
}

static void set_response_ok(uint32_t addr, lora_event_t cmd){
	uint16_t crc = 0;
	char *temp;
	char *response_to_device;
	char *cmd_str = cmd_to_str(cmd, command_string);

	asprintf(&temp, "%s: {\"addr\":0x%08x,\"state\":OK,", cmd_str, addr);
	crc = cal_crc16((uint8_t *)temp, strlen(temp));
	free(temp);

	asprintf(&response_to_device, "%s: {\"addr\":0x%08x,\"state\":OK,\"crc\":0x%04x}", cmd_str, addr, crc);

	if(xQueueSend(response_queue, &response_to_device, 2) == pdFALSE){
		loraif_debug((char *)"Can't send to response queue", __LINE__, __FUNCTION__);
	}
}

void loraif_process(void *param){
	QueueHandle_t *queue = (QueueHandle_t *)param;
	char *rx_full;

	if(xQueueReceive(*queue, &rx_full, 10)){
		pkt_err_t err;
		pkt_t pkt;
		pkt_json_t json;
		lora_event_t cmd;
		char *evt_data = NULL;
		char *response_to_device;

		err = parse_packet(rx_full, &pkt);
/** Parse packet success */
		if(err == PKT_ERR_OK){
			cmd = (lora_event_t)str_to_cmd(pkt.cmd_str, command_string, (int)LORA_CMD_NUM);

		/** Response without device address */
			if(cmd == LORA_REQ_ADDRESS){
				err = json_get_object(pkt.data_str, &json, (char *)"random_number");

				if(err == PKT_ERR_OK){
					/** Generate new address */
					uint32_t new_addr;
					uint32_t rand_num = strtol(json.value, NULL, 16);

					rng_set_seed(rand_num);
					while(1){
						new_addr = rng_generate_random_number();
					    if(device_is_available(new_addr)) continue;
					    else break;
					}
					new_addr &= 0x7FFFFFFFU;
					/** Response new address to new device */
					uint16_t crc = 0;
					char *temp;

					asprintf(&temp, "LORA_RES_ADDRESS: {\"addr\":0x%08x,", new_addr);
					crc = cal_crc16((uint8_t *)temp, strlen(temp));
					free(temp);

					asprintf(&response_to_device, "LORA_RES_ADDRESS: {\"addr\":0x%08x,\"crc\":0x%04x}", new_addr, crc);
					loraif_debug(response_to_device, __LINE__, __FUNCTION__);
					if(xQueueSend(response_queue, &response_to_device, 2) == pdFALSE){
						loraif_debug((char *)"Can't send address to new device", __LINE__, __FUNCTION__);
					}

					goto event_handle;
				}
				else{
					loraif_debug((char *)"Can't allocate new device address", __LINE__, __FUNCTION__);
				}

				json_release_object(&json);
			}

		/** Response with device address */
			else{
				err = json_get_object(pkt.data_str, &json, (char *)"addr");
				uint32_t addr = strtol(json.value, NULL, 16);

				if(err == PKT_ERR_OK){
					if(cmd == LORA_ERR){
						loraif_debug((char *)"Device error", __LINE__, __FUNCTION__);
					}

					else if(cmd == LORA_UPDATE_ADDRESS){
//							show_device_list();
							/** Response state to new device */
							set_response_ok(addr, cmd);

							loraif_debug((char *)"New device update address", __LINE__, __FUNCTION__);
							goto event_handle;
					}

					else if(cmd == LORA_UPDATE_STATE){
							/** Response state to new device */
							set_response_ok(addr, cmd);

							loraif_debug((char *)"Device update state", __LINE__, __FUNCTION__);
							goto event_handle;
					}

					/** LORA_UPDATE_SETTINGS, LORA_REQ_DATA, LORA_UPDATE_DATA */
					else{
						if(addr == req_address && cmd == req_cmd){
							req_had_resp = true;

							/**
							 * Reset device not response count.
							 */
							loraif_dev_t *err_dev = loraif_select_device(pkt.data_str);
							err_dev->err_count = 0;

							/** Response state to new device */
							set_response_ok(addr, cmd);

							loraif_debug((char *)"Device had response", __LINE__, __FUNCTION__);
							goto event_handle;
						}
					}
				}
				else{
					loraif_debug((char *)"Unknown device", __LINE__, __FUNCTION__);
				}

				json_release_object(&json);
			}

			event_handle:
			asprintf(&evt_data, "%s", pkt.data_str);
			if(fpeventhandler != NULL) fpeventhandler(cmd, evt_data);
			free(evt_data);
		}

/** Parse packet error */
		else{
			loraif_debug((char *)"Can't parse received packet", __LINE__, __FUNCTION__);
		}

		release_packet(&pkt);
	}
}


void loraif_response(void){
	char *response;

	if(xQueueReceive(response_queue, &response, 10)){
		loraif_transmit(response);
		free(response);
	}
}

void loraif_check_timeout(void){
	while(req_had_resp == false){
		if(timer->get_counter() >= resp_timeout){
			timer->stop();
			timer->reset_counter();
			req_had_resp = false;

			char *evt_data;
			asprintf(&evt_data, "{\"addr\":0x%08x}", req_address);
			loraif_dev_t *err_dev = loraif_select_device(evt_data);
			if(err_dev->err_count >= max_not_resp){
				char *tmp;
				asprintf(&tmp, "Device 0x%08x %d times not response", req_address, max_not_resp);
				loraif_debug(tmp, __LINE__, __FUNCTION__);
				free(tmp);

				if(fpeventhandler != NULL) fpeventhandler(LORA_DEL_DEVICE, evt_data);
			}
			else{
				err_dev->err_count++;
			}
			free(evt_data);
		}
		vTaskDelay(1);
	}
}


void loraif_request(uint32_t dev_address, lora_event_t cmd, char *data){
	uint16_t crc = 0;
	char *req_data, *temp;
	char *cmd_str = cmd_to_str(cmd, command_string);
	asprintf(&temp, "%s: {\"addr\":0x%08x,\"data\":%s,", cmd_str, dev_address, data);
	crc = cal_crc16((uint8_t *)temp, strlen(temp));
	free(temp);

	asprintf(&req_data, "%s: {\"addr\":0x%08x,\"data\":%s,\"crc\":0x%04x}", cmd_str, dev_address, data, crc);

	req_address = dev_address;
	req_cmd = cmd;
	req_had_resp = false;
	loraif_transmit(req_data);
	timer->start();

	free(req_data);
}


void loraif_request_data(void){
    if(loraif_device_list.empty()) {
        return;
    }

    for (auto device = loraif_device_list.begin(); device != loraif_device_list.end(); ++device) {
    	vTaskDelay(10);
        loraif_request((*device)->address, LORA_REQ_DATA, (char *)"?");
        loraif_check_timeout();
    }
}


void loraif_new_device(char *jdata, void *dev_data){
	pkt_err_t err;
	pkt_json_t json;
	uint32_t addr;
    loraif_dev_t *newdev = (loraif_dev_t *)malloc(sizeof(loraif_dev_t));

    newdev->err_count = 0;
    newdev->data = dev_data;

	err = json_get_object(jdata, &json, (char *)"addr");
	if(err == PKT_ERR_OK)
		newdev->address = strtol(json.value, NULL, 16);
	json_release_object(&json);

	err = json_get_object(jdata, &json, (char *)"name");
	if(err == PKT_ERR_OK)
		asprintf(&newdev->name, "%s", json.value);
	json_release_object(&json);

	loraif_device_list.push_back(newdev);

	char *tmp;
	asprintf(&tmp, "Add device 0x%08x(%s)", newdev->address, newdev->name);
	loraif_debug(tmp, __LINE__, __FUNCTION__);
	free(tmp);
}

void loraif_remove_device(char *jdata){
	pkt_err_t err;
	pkt_json_t json;
	uint32_t del_addr;

	err = json_get_object(jdata, &json, (char *)"addr");
	if(err == PKT_ERR_OK)
		del_addr = strtol(json.value, NULL, 16);
	json_release_object(&json);

    if (loraif_device_list.empty()) {
    	loraif_debug((char *)"Device list empty", __LINE__, __FUNCTION__);
        return;
    }

    auto device = loraif_device_list.begin();
    while (device != loraif_device_list.end()) {
        if ((*device)->address == del_addr) {
            break;
        }
        ++device;
    }

    if (device == loraif_device_list.end()) {
    	loraif_debug((char *)"This device not available in device list", __LINE__, __FUNCTION__);
        return;
    }

    free((*device)->name);
    free(*device);

    loraif_device_list.erase(device);
}

loraif_dev_t *loraif_select_device(char *jdata){
	pkt_err_t err;
	pkt_json_t json;
	uint32_t sel_addr;

	err = json_get_object(jdata, &json, (char *)"addr");
	if(err == PKT_ERR_OK)
		sel_addr = strtol(json.value, NULL, 16);
	json_release_object(&json);

    if (loraif_device_list.empty()) {
    	loraif_debug((char *)"Device list empty", __LINE__, __FUNCTION__);
        return NULL;
    }

    auto device = loraif_device_list.begin();
    while (device != loraif_device_list.end()) {
        if ((*device)->address == sel_addr) {
            break;
        }
        ++device;
    }

    if (device == loraif_device_list.end()) {
    	loraif_debug((char *)"This device not available in device list", __LINE__, __FUNCTION__);
        return NULL;
    }

    return (*device);
}



#endif /* ENABLE_COMPONENT_WIFIIF */
