/*
 * httpclient.cpp
 *
 *  Created on: Apr 4, 2023
 *      Author: anh
 */

#include "net_protocols_config.h"
#if ENABLE_HTTP

#include "protocols/http/httpclient.h"

#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/_timeval.h>

#include "mbedtls/error.h"
#include "string.h"

#include "FreeRTOS.h"
#include "task.h"

#if HTTP_LOG
#include "system/log.h"
#endif /* HTTP_LOG */


#if HTTP_LOG
static const char *TAG = (char *)"HTTP";
#endif /* HTTP_LOG */

#define DEFAULT_CHECK_SOCKET_TIMEOUT 5 // 5s.
#define MAX_ERRSTR_LEN 133

const char *alpn_protocols[] = {"http/0.9", "http/1.0", "http/1.1", "http/2", NULL};

static uint8_t buf[1024];

static err_t create_socket_connection(httpclient_t *client);
static err_t check_socket_connection_established(httpclient_t *client);
static void ssl_cleanup(httpclient_t *client);
static void error_debug(int ret, int line, char *func, char *message);
static void error_handler(httpclient_t *client, httpclient_event_t event);

static void error_debug(int ret, int line, char *func, char *message){
#if HTTP_DEBUG
	char *strerr = (char *)malloc(MAX_ERRSTR_LEN + 1);
	mbedtls_strerror(ret, strerr, MAX_ERRSTR_LEN);
	LOG_DEBUG(TAG, "%s, returned -0x%04x(%s) line %d function %s", message, -ret, strerr, line-2, func);
	free(strerr);
#endif
}

static void error_handler(httpclient_t *client, httpclient_event_t event){
//	if(client->event->data != NULL) free(client->event->data);
//	client->event->data_len = 0;
//	client->event->event = event;
//	if(client->event_handler != NULL) client->event_handler(client, client->event, client->parameter);
}

static err_t create_socket_connection(httpclient_t *client){
	err_t ret;
	struct sockaddr_storage address;

	/**
	 * OPEN SOCKET.
	 */
	client->hints.ai_family = AF_INET;
	client->hints.ai_socktype = SOCK_STREAM;

    ret = lwip_getaddrinfo(client->hostname, NULL, &client->hints, &client->sock_info);
    if (ret != 0 || client->sock_info == NULL) {
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Couldn't get address infor");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
        return ret;
    }

    client->sock_fd.fd = lwip_socket(client->sock_info->ai_family, client->sock_info->ai_socktype, client->sock_info->ai_protocol);
    if (client->sock_fd.fd < 0) {
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Couldn't create socket");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
        lwip_freeaddrinfo(client->sock_info);
        return ERR_CONN;
    }

    if (client->sock_info->ai_family == AF_INET) {
        struct sockaddr_in *p = (struct sockaddr_in *)client->sock_info->ai_addr;
        p->sin_port = htons(client->port);
        client->ip.addr = (uint32_t)p->sin_addr.s_addr;
        LOG_INFO(TAG, "Socket [sock=%d, family %d, socktype %d, protocol %d, host IP address: %s, port=%d]"
        		, client -> sock_fd.fd, client->sock_info->ai_family, client->sock_info->ai_socktype, client->sock_info->ai_protocol
        		, ip4addr_ntoa((const ip_addr_t*)&client->ip), (uint16_t)ntohs(p->sin_port));
        memcpy(&address, p, sizeof(struct sockaddr ));
    }
    lwip_freeaddrinfo(client->sock_info);

    /**
     * SET SOCKET OPTTIONS.
     */
    int keep_alive_enable = 1;
    int keep_alive_idle = client->keepalive_time;
    int keep_alive_interval = client->keepalive_interval;
    int keep_alive_count = client->keepalive_count;

    ret = lwip_setsockopt(client->sock_fd.fd, SOL_SOCKET, SO_KEEPALIVE, &keep_alive_enable, sizeof(keep_alive_enable));
    if(ret != 0) {
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Fail to set socket option SO_KEEPALIVE");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
        return ret;
    }
    ret = lwip_setsockopt(client->sock_fd.fd, IPPROTO_TCP, TCP_KEEPIDLE, &keep_alive_idle, sizeof(keep_alive_idle));
    if(ret != 0) {
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Fail to set socket option TCP_KEEPIDLE");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
        return ret;
    }
    ret = lwip_setsockopt(client->sock_fd.fd, IPPROTO_TCP, TCP_KEEPINTVL, &keep_alive_interval, sizeof(keep_alive_interval));
    if(ret != 0) {
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Fail to set socket option TCP_KEEPINTVL");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
        return ret;
    }
    ret = lwip_setsockopt(client->sock_fd.fd, IPPROTO_TCP, TCP_KEEPCNT, &keep_alive_count, sizeof(keep_alive_count));
    if(ret != 0) {
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Fail to set socket option TCP_KEEPCNT");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
        return ret;
    }

    /**
     * SOCKET CONNECT.
     */
    ret = lwip_connect(client->sock_fd.fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
    if(ret != 0) {
        error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Error connection");
        error_handler(client, HTTP_EVENT_CONNECT_ERROR);
        if (errno == EINPROGRESS) {
        	LOG_ERROR(TAG, "EINPROGRESS.");
        	return ERR_OK;
        }

    	return ret;
	}

    return ret;
}

static err_t check_socket_connection_established(httpclient_t *client){
	err_t ret;

    FD_ZERO(&client->r_fd);
    FD_SET(client->sock_fd.fd, &client->r_fd);
    FD_ZERO(&client->w_fd);
    FD_SET(client->sock_fd.fd, &client->w_fd);

    struct timeval timeout;
    timeout.tv_sec = DEFAULT_CHECK_SOCKET_TIMEOUT;
    timeout.tv_usec = 0;

    ret = lwip_select(client->sock_fd.fd + 1, &client->r_fd, &client->w_fd, NULL, &timeout);
    if(ret == 0) {
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Socket select timeout");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
    	ret = -1;
    	return ret;
    }

    if (FD_ISSET(client->sock_fd.fd, &client->r_fd) || FD_ISSET(client->sock_fd.fd, &client->w_fd)) {
        int error;
        socklen_t len = sizeof(error);
        ret = lwip_getsockopt(client->sock_fd.fd, SOL_SOCKET, SO_ERROR, &error, &len);
        if(ret < 0) {
        	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Non blocking connect failed");
        	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
            return ret;
        }
    }

    return ret;
}

static void ssl_cleanup(httpclient_t *client){
	if(client->cert_pem != NULL){
		mbedtls_x509_crt_free(&client->cacert);
	}
	mbedtls_x509_crt_free(&client->cacert);
	mbedtls_entropy_free(&client->entropy);
	mbedtls_ssl_config_free(&client->conf);
	mbedtls_ctr_drbg_free(&client->ctr_drbg);
	mbedtls_ssl_free(&client->ssl);
}

err_t httpclient_init(httpclient_t *client){
	client->event = (httpclient_event_handler_t *)malloc(sizeof(httpclient_event_handler_t));

	if(client->connection == NULL) asprintf(&client->connection, HTTPCLIENT_DEFAULT_CONNECTION_HEADER);
	if(client->content_type == NULL) asprintf(&client->content_type, HTTPCLIENT_DEFAULT_CONTENT_TYPE_HEADER);
	if(client->accept == NULL) asprintf(&client->accept, HTTPCLIENT_DEFAULT_ACCEPT_HEADER);
	/** create request line string */
	asprintf(&client -> requestline, "%s %s %s\r\n", http_method_str[client -> method], client -> url, client -> version);
	/** create header string */
	asprintf(&client -> default_header, "Host: %s\r\nConnection: %s\r\nContent-Type: %s\r\nContent-Length: %s\r\nAccept: %s\r\n",
			client -> hostname, client->connection, client->content_type, client->content_length, client->accept);

	return ERR_OK;
}

err_t httpclient_connect(httpclient_t *client){
	err_t ret;
	const unsigned char *pers = (const unsigned char *)"ssl_client1";

//    if ((ret = mbedtls_net_connect(&client->sock_fd, client->hostname, "443", MBEDTLS_NET_PROTO_TCP)) != 0) {
//    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL net connect failed");
//    }

	/** Establish connection (esp_tls_low_level_conn)*/
	ret = create_socket_connection(client);
	if(ret != ERR_OK){
		error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Failed to create socket connection");
		error_handler(client, HTTP_EVENT_CONNECT_ERROR);
		return ret;
	}
	/** Check socket connect established */
	ret = check_socket_connection_established(client);
	if(ret != ERR_OK){
		error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Failed to check socket connection");
		error_handler(client, HTTP_EVENT_CONNECT_ERROR);
		return ret;
	}
	error_handler(client, HTTP_EVENT_CONNECTED);

	/** SSL initialize */
    mbedtls_ssl_init(&client->ssl);
    mbedtls_ssl_config_init(&client->conf);
    mbedtls_x509_crt_init(&client->cacert);
    mbedtls_ctr_drbg_init(&client->ctr_drbg);
    mbedtls_entropy_init(&client->entropy);
    /** SSL set entropy random seed */
	ret = mbedtls_ctr_drbg_seed(&client->ctr_drbg, mbedtls_entropy_func, &client->entropy,
								pers, strlen((char *)pers));
	if(ret != 0){
		error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL configure drbg seed failed");
		ssl_cleanup(client);
		error_handler(client, HTTP_EVENT_SSL_ERROR);
		return ret;
	}
	/** SSL x509 crt pem parse */
	if(client->cert_pem != NULL){
		ret = mbedtls_x509_crt_parse(&client->cacert, (const unsigned char *)client->cert_pem, strlen(client->cert_pem) +1);
		if(ret != 0){
			error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL cert pem parse failed");
			ssl_cleanup(client);
			error_handler(client, HTTP_EVENT_SSL_ERROR);
			return ret;
		}
	} else LOG_ERROR(TAG, "Https require ssl certificate pem.");

    /** SSL client configuration */
    ret = mbedtls_ssl_config_defaults(&client->conf,
    		MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
	if(ret != 0){
		error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL set default configure failed");
		ssl_cleanup(client);
		error_handler(client, HTTP_EVENT_SSL_ERROR);
		return ret;
	}
	mbedtls_ssl_conf_renegotiation(&client->conf, MBEDTLS_SSL_RENEGOTIATION_ENABLED);
	ret = mbedtls_ssl_conf_alpn_protocols(&client->conf, alpn_protocols);
	if(ret != 0){
		error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL set alpn protocols failed");
		ssl_cleanup(client);
		error_handler(client, HTTP_EVENT_SSL_ERROR);
		return ret;
	}
	mbedtls_ssl_conf_authmode(&client->conf, MBEDTLS_SSL_VERIFY_REQUIRED);
	mbedtls_ssl_conf_ca_chain(&client->conf, &client->cacert, NULL);
	mbedtls_ssl_conf_rng(&client->conf, mbedtls_ctr_drbg_random, &client->ctr_drbg);
	mbedtls_ssl_conf_dbg(&client->conf, NULL, NULL);
	ret = mbedtls_ssl_setup(&client->ssl, &client->conf);
	if(ret != 0){
		error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL setup failed");
		ssl_cleanup(client);
		error_handler(client, HTTP_EVENT_SSL_ERROR);
		return ret;
	}
	/** SSL set host name */
    ret = mbedtls_ssl_set_hostname(&client->ssl, client -> hostname);
    if(ret != 0){
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL set host name failed");
    	ssl_cleanup(client);
    	error_handler(client, HTTP_EVENT_SSL_ERROR);
    	return ret;
    }
    /** SSL set network socket interface */
    mbedtls_ssl_set_bio(&client->ssl, &client->sock_fd, mbedtls_net_send, mbedtls_net_recv, NULL);

    /** SSL Start handshake */
    while((ret = mbedtls_ssl_handshake(&client->ssl)) != 0) {
    	LOG_INFO(TAG, "SSL handshaking.....");
        if(ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
			error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL handshake failed(MBEDTLS_ERR_SSL_WANT_READ | MBEDTLS_ERR_SSL_WANT_WRITE)");
			ssl_cleanup(client);
			error_handler(client, HTTP_EVENT_SSL_ERROR);
			return ret;
        }
        vTaskDelay(2000);
    }
    LOG_INFO(TAG, "SSL handshake successful.");
    /** SSL verify result */
    ret = mbedtls_ssl_get_verify_result(&client->ssl);
    if(ret != 0){
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL verify failed");
    	ssl_cleanup(client);
    	error_handler(client, HTTP_EVENT_SSL_ERROR);
    	return ret;
    }
    LOG_INFO(TAG, "SSL verify oke.");

	return ret;
}

err_t httpclient_disconnect(httpclient_t *client){
	err_t ret = ERR_OK;

//	mbedtls_net_free(&client -> sock_fd);
	ret = mbedtls_ssl_close_notify(&client->ssl);
    if(ret != 0){
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"SSL close notify failed");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
    	return ret;
    }

    ret = lwip_shutdown(client->sock_fd.fd, SHUT_RDWR);
    if(ret != 0){
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Socket shutdown failed");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
    	return ret;
    }
    ret = lwip_close(client->sock_fd.fd);
    if(ret != 0){
    	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"Socket close failed");
    	error_handler(client, HTTP_EVENT_CONNECT_ERROR);
    	return ret;
    }
    client->sock_fd.fd = -1;

    mbedtls_x509_crt_free(&client->cacert);
    mbedtls_ssl_free(&client->ssl);
    mbedtls_ssl_config_free(&client->conf);
    mbedtls_ctr_drbg_free(&client->ctr_drbg);
    mbedtls_entropy_free(&client->entropy);

    if(client->header != NULL) free(client->header);
    if(client->requestline != NULL) free(client->requestline);

    error_handler(client, HTTP_EVENT_DISCONNECTED);

	return ret;
}

void httpclient_set_header(httpclient_t *client, char *newheader){
	if(client->header != NULL) free(client->header);
	asprintf(&client->header, newheader);
}

void httpclient_add_header(httpclient_t *client, char *key, char *value){
	if(strcmp(key, (char *)"Connection") == 0){
		if(client->connection == NULL) free(client->connection);
		asprintf(&client->connection, "%s", value);
	}
	else if(strcmp(key, (char *)"Content-Type") == 0){
		if(client->content_type == NULL) free(client->content_type);
		asprintf(&client->content_type, "%s", value);
	}
	else if(strcmp(key, (char *)"Content-Length") == 0){
		if(client->content_length == NULL) free(client->content_length);
		asprintf(&client->content_length, "%s", value);
	}
	else if(strcmp(key, (char *)"Accept") == 0){
		if(client->accept == NULL) free(client->accept);
		asprintf(&client->accept, "%s", value);
	}
	else{
		if(client->other_header == NULL) asprintf(&client->other_header, "%s: %s\r\n", key, value);
		else{
		uint16_t len = strlen(client->other_header);
			client->other_header = (char *)realloc(client->other_header, (len + strlen(key) + strlen(value) + 5)*sizeof(char));
			sprintf(client -> other_header, "%s%s: %s\r\n", client->other_header, key, value);
		}
		return;
	}

	client -> default_header = (char *)realloc(client -> default_header,
			strlen(client -> hostname) + strlen(client -> connection) + strlen(client -> content_type) + strlen(client -> content_length) + strlen(client -> accept)
		  + strlen("Host: ") + strlen("\r\nConnection: ") + strlen("\r\nContent-Type: ") + strlen("\r\nContent-Length: ") + strlen("\r\nAccept: ") + 2);

	sprintf(client -> default_header, "Host: %s\r\nConnection: %s\r\nContent-Type: %s\r\nContent-Length: %s\r\nAccept: %s\r\n",
			client -> hostname, client->connection, client->content_type, client->content_length, client->accept);

}

void httpclient_remove_header(httpclient_t *client, char *key){
	if((strcmp(key, (char *)"Connection") == 0) ||
	   (strcmp(key, (char *)"Content-Type") == 0) ||
	   (strcmp(key, (char *)"Content-Length") == 0) ||
	   (strcmp(key, (char *)"Accept") == 0)){
		LOG_ERROR(TAG, "Header %s is default, can't remove.", key);
		return;
	}
	else{
		char *pstart = strstr(client->other_header, key);
		if(pstart == NULL){
			LOG_ERROR(TAG, "Can't find header %s in current header list.", key);
			return;
		}
		char *pend = strstr(pstart, (char *)"\r\n");
		if(pstart == NULL){
			LOG_ERROR(TAG, "Header format error, can't find '\r\n' at end of value.", key);
			return;
		}
		pend += 2;
		memmove(pstart, pend, strlen(pend)+1);
		client->other_header = (char *)realloc(client->other_header, strlen(client->other_header)+1);
	}
}


err_t httpclient_request(httpclient_t *client, char *buffer, uint16_t len){
	err_t ret;
	char *len_str = NULL;
	unsigned char buf[HTTP_MAX_RESP_LEN];
	uint16_t ret_len = 0;

	asprintf(&len_str, "%d", len);
	httpclient_add_header(client, (char *)"Content-Length", len_str);
	if(client -> other_header == NULL){
		asprintf(&client->packet, "%s" /** Request line */
								  "%s" /** Default header */
								  "\r\n"
								  "%s"
		,client->requestline, client->default_header, buffer);
	}
	else{
		asprintf(&client->packet, "%s" /** Request line */
								  "%s" /** Default header */
								  "%s" /** Other header */
								  "\r\n"
								  "%s"
		,client->requestline, client->default_header, client->other_header, buffer);
	}

	LOG_INFO(TAG, "\r\n%s", client->packet);


    while((ret = mbedtls_ssl_write(&client->ssl, (const unsigned char *)client->packet, strlen(client->packet))) <= 0) {
        if(ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
        	error_handler(client, HTTP_EVENT_REQUEST_FAIL);
        	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"mbedtls ssl write failed");
        	return ret;
        }
    }
	asprintf(&client->event->data, "%s", client->packet);
	client->event->data_len = strlen(client->packet);
	error_handler(client, HTTP_EVENT_REQUEST);

    do {
    	ret_len = sizeof(buf) - 1;
        memset(buf, 0, sizeof(buf));
        ret = mbedtls_ssl_read(&client->ssl, buf, ret_len);

        if(ret == MBEDTLS_ERR_SSL_WANT_READ || ret == MBEDTLS_ERR_SSL_WANT_WRITE) continue;
        if(ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) break;
        if(ret < 0) {
        	error_handler(client, HTTP_EVENT_NOT_RESPONSE);
        	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"mbedtls ssl read failed");
            break;
        }

        if (ret == 0) {
        	error_debug(ret, __LINE__, (char *)__FUNCTION__, (char *)"mbedtls ssl read EOF");
            break;
        }

        ret_len = ret;
        LOG_INFO(TAG, "response len = %d", ret_len);
        LOG_INFO(TAG, "%s", (char *)buf);
//        client->event->data_len = resp_len;
//        client->event->data = (char *)malloc(resp_len+1);
//        memcpy(client->event->data)
    	error_handler(client, HTTP_EVENT_RESPONSE);
    } while (1);

    mbedtls_ssl_close_notify(&client->ssl);

	free(client->packet);
	free(len_str);

	return 0;
}

#endif /* ENABLE_HTTP */

