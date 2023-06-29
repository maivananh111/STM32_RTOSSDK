/*
 * tcp_client.cpp
 *
 *  Created on: Apr 4, 2023
 *      Author: anh
 */

#include "net_protocols_config.h"
#if ENABLE_TCP

#include "protocols/tcp/tcp_client.h"

#include "lwip/netdb.h"
#include "lwip/ip4_addr.h"
#include "lwip/sockets.h"
#include "lwip/err.h"

#include "string.h"

#include "FreeRTOS.h"
#include "task.h"

#if TCP_LOG
#include "system/log.h"
#endif /* TCP_LOG */


#if TCP_LOG
static const char *TAG = (char *)"TCP/IP";
#endif /* TCP_LOG */

static err_t tcp_client_connected_handler(void *arg, struct tcp_pcb *tpcb, err_t err);
static err_t tcp_client_receive_handler(void *arg, struct tcp_pcb *tpcb, struct pbuf *buf, err_t err);
static err_t tcp_client_poll_handler(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_client_sent_handler(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_client_disconnect_handler(struct tcp_pcb *tpcb, tcp_handler_t *handle);
static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_es_t *es);

err_t tcp_client_connect(tcp_connection_t *tcp){
#if TCP_DEBUG
	if(tcp -> name != NULL) LOG_DEBUG(TAG, "%s call tcp_client_connect.", tcp -> name);
	else LOG_DEBUG(TAG, "call tcp_client_connect.");
#endif /* TCP_DEBUG */
	err_t err;

	if(strcmp(tcp -> ip_str, "0.0.0.0") != 0 && tcp -> hostname == NULL){
		if(!ip4addr_aton(tcp -> ip_str, &(tcp -> ip))){
#if TCP_LOG
			LOG_ERROR(TAG, "Couldn't convert ip_str to server ip address.");
#endif /* TCP_LOG */
		}
	}
	else{
	    struct hostent *he;
	    struct in_addr **addr_list;
	    if ((he = lwip_gethostbyname(tcp -> hostname)) == NULL) {
#if TCP_LOG
	    	LOG_ERROR(TAG, "Failed to get server ip by name.");
#endif /* TCP_LOG */
	        return ERR_VAL;
	    }
	    addr_list = (struct in_addr **) he->h_addr_list;

	    ip4addr_aton(inet_ntoa(*addr_list[0]), &(tcp -> ip));
	}
#if TCP_LOG
	LOG_INFO(TAG, "Server IP: %s", ip4addr_ntoa(&(tcp -> ip)));
#endif /* TCP_LOG */

	tcp -> handler.connection = (void *)tcp;
	/** open new tcp connect */
	tcp -> conn_pcb = tcp_new();
	tcp -> conn_pcb -> keep_idle  = tcp -> keepalive_time;
	tcp -> conn_pcb -> keep_intvl = tcp -> keepalive_interval;
	tcp -> conn_pcb -> keep_cnt   = tcp -> keepalive_count;
	/** set tcp argument */
	tcp_arg(tcp -> conn_pcb, (void *)(&tcp -> handler));
	/** define ip to socket */
	err = tcp_connect(tcp -> conn_pcb, &(tcp -> ip), tcp -> port, tcp_client_connected_handler);

	return err;
}

void tcp_client_disconnect(tcp_connection_t *tcp){
#if TCP_DEBUG
	if(tcp -> name != NULL) LOG_DEBUG(TAG, "%s call tcp_client_disconnect.", tcp -> name);
	else LOG_DEBUG(TAG, "call tcp_client_disconnect.");
#endif /* TCP_DEBUG */
	tcp_arg(tcp -> conn_pcb, NULL);
	tcp_sent(tcp -> conn_pcb, NULL);
	tcp_recv(tcp -> conn_pcb, NULL);
	tcp_err(tcp -> conn_pcb, NULL);
	tcp_poll(tcp -> conn_pcb, NULL, 0);
	tcp_close(tcp -> conn_pcb);

	if(tcp -> event_handler != NULL) tcp -> event_handler(TCP_CLIENT_EVENT_DISCONNECTED, tcp -> parameter);
}

void tcp_client_send_data(tcp_connection_t *tcp, char *data){
#if TCP_DEBUG
	if(tcp -> name != NULL) LOG_DEBUG(TAG, "%s call tcp_client_send_data.", tcp -> name);
	else LOG_DEBUG(TAG, "call tcp_client_send_data.");
#endif /* TCP_DEBUG */
	tcp_sent(tcp -> conn_pcb, tcp_client_sent_handler);
	tcp_write(tcp -> conn_pcb, (void*)data, strlen(data), 1);
	tcp_output(tcp -> conn_pcb);
}

static err_t tcp_client_connected_handler(void *arg, struct tcp_pcb *tpcb, err_t err){
	tcp_handler_t *handle = (tcp_handler_t *)arg;
	tcp_connection_t *conn = (tcp_connection_t *)handle -> connection;

#if TCP_DEBUG
	if(conn -> name != NULL) LOG_DEBUG(TAG, "%s call tcp_client_connected_handler.", conn -> name);
	else LOG_DEBUG(TAG, "call tcp_client_connected_handler.");
#endif /* TCP_DEBUG */

	struct tcp_client_es_t *es;
	err_t ret_err;

	es = (struct tcp_client_es_t *)mem_malloc(sizeof(struct tcp_client_es_t));
	handle -> es = es;
	if(err == ERR_OK && es != NULL){
		es -> state = ES_CONNECTED;
		es -> pcb = tpcb;
		es -> buf = NULL;

		/** pass newly allocated es structure as argument to newpcb */
		tcp_arg(tpcb, (void *)handle);

		/** initialize lwip tcp_recv callback function for newpcb  */
		tcp_recv(tpcb, tcp_client_receive_handler);

		/** initialize lwip tcp_poll callback function for newpcb */
		tcp_poll(tpcb, tcp_client_poll_handler, 0);

		/** initialize LwIP tcp_sent callback function */
		tcp_sent(tpcb, tcp_client_sent_handler);

		/** send data */
		tcp_client_send(tpcb, es);

		/** handle the TCP data */
		if(conn -> event_handler != NULL) conn -> event_handler(TCP_CLIENT_EVENT_CONNECTED, conn -> parameter);

		ret_err = ERR_OK;

#if TCP_DEBUG
	LOG_DEBUG(TAG, "tcp connection opened, call tcp_client_disconnect_handler, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
	}
	else{
		/**  close tcp connection */
		tcp_client_disconnect_handler(tpcb, handle);
		/** return memory error */
		ret_err = ERR_MEM;
#if TCP_DEBUG
	LOG_DEBUG(TAG, "can't open tcp connection, call tcp_client_disconnect_handler, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
	}

	return ret_err;
}

static err_t tcp_client_receive_handler(void *arg, struct tcp_pcb *tpcb, struct pbuf *buf, err_t err){
	tcp_handler_t *handle = (tcp_handler_t *)arg;
	tcp_connection_t *conn = (tcp_connection_t *)handle -> connection;
#if TCP_DEBUG
	if(conn -> name != NULL) LOG_DEBUG(TAG, "%s call tcp_client_receive_handler.", conn -> name);
	else LOG_DEBUG(TAG, "call tcp_client_receive_handler.");
#endif /* TCP_DEBUG */
	struct tcp_client_es_t *es = handle -> es;
	err_t ret_err;

	/** if we receive an empty tcp frame from server => close connection */
	if (buf == NULL){
		/** remote host closed connection */
		es -> state = ES_CLOSING;
		if(es -> buf == NULL){
			/** we're done sending, close connection */
			tcp_client_disconnect_handler(tpcb, handle);
#if TCP_DEBUG
	LOG_DEBUG(TAG, "es closing, call tcp_client_disconnect_handler, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
		}
		ret_err = ERR_OK;
	}
	/** a non empty frame was received from server but for some reason err != ERR_OK */
	else if(err != ERR_OK){
	/** free received pbuf*/
		if(buf != NULL){
			es->buf = NULL;
			pbuf_free(buf);
		}
		ret_err = err;
	}
	else if(es->state == ES_CONNECTED){
		/** Acknowledge the received data */
		tcp_recved(tpcb, buf->tot_len);

		/** store reference to incoming pbuf (chain) */
		es->buf = buf;

		/** get data in payload */
		conn -> len = es->buf->len;
		conn -> data = (void *)malloc(es->buf->len);
		strncpy((char *)conn -> data, (char *)es->buf->payload, es->buf->len);

		/** handle the received data */
		if(conn -> event_handler) conn -> event_handler(TCP_CLIENT_EVENT_RECEIVED, conn -> parameter);

		pbuf_free(buf);
		free(conn -> data);
		conn -> len = 0;

		ret_err = ERR_OK;
#if TCP_DEBUG
	LOG_DEBUG(TAG, "received data, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
	}
	else if(es->state == ES_RECEIVED){
		ret_err = ERR_OK;
	}
	else{
		/** unknown es->state, trash data  */
		tcp_recved(tpcb, buf->tot_len);
		es->buf = NULL;
		pbuf_free(buf);
		ret_err = ERR_OK;
#if TCP_DEBUG
	LOG_DEBUG(TAG, "received trash data, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
	}

	return ret_err;
}

static err_t tcp_client_poll_handler(void *arg, struct tcp_pcb *tpcb){
	tcp_handler_t *handle = (tcp_handler_t *)arg;
#if TCP_DEBUG
	tcp_connection_t *conn = (tcp_connection_t *)handle -> connection;
	if(conn -> name != NULL) LOG_DEBUG(TAG, "%s call tcp_client_poll_handler.", conn -> name);
	else LOG_DEBUG(TAG, "call tcp_client_poll_handler.");
#endif /* TCP_DEBUG */
	struct tcp_client_es_t *es = handle -> es;
	err_t ret_err;

	if (es != NULL){
		if(es->buf != NULL){

		}
		else{
			/** no remaining pbuf (chain) */
			if(es->state == ES_CLOSING){
			/** close tcp connection */
				tcp_client_disconnect_handler(tpcb, handle);
#if TCP_DEBUG
	LOG_DEBUG(TAG, "es closing, call tcp_client_disconnect_handler, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
			}
		}
		ret_err = ERR_OK;
	}
	else{
		/** nothing to be done */
		tcp_abort(tpcb);
		ret_err = ERR_ABRT;
#if TCP_DEBUG
	LOG_DEBUG(TAG, "ERR_ABRT, call tcp_abort, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
	}

	return ret_err;
}

static err_t tcp_client_sent_handler(void *arg, struct tcp_pcb *tpcb, u16_t len){
	tcp_handler_t *handle = (tcp_handler_t *)arg;
	tcp_connection_t *conn = (tcp_connection_t *)handle -> connection;
#if TCP_DEBUG
	if(conn -> name != NULL) LOG_DEBUG(TAG, "%s call tcp_client_sent_handler.", conn -> name);
	else LOG_DEBUG(TAG, "call tcp_client_sent_handler.");
#endif /* TCP_DEBUG */
	struct tcp_client_es_t *es = handle -> es;

	LWIP_UNUSED_ARG(len);

	es = (struct tcp_client_es_t *)arg;

	if(es->buf != NULL){
		if(conn -> event_handler) conn -> event_handler(TCP_CLIENT_EVENT_TRANSMITED, conn -> parameter);
		tcp_client_send(tpcb, es);
	}
	else{
		/** if no more data to send and client closed connection*/
		if(es->state == ES_CLOSING){
			tcp_client_disconnect_handler(tpcb, handle);
#if TCP_DEBUG
	LOG_DEBUG(TAG, "es closing, call tcp_client_disconnect_handler, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
		}
	}
	return ERR_OK;
}

static void tcp_client_send(struct tcp_pcb *tpcb, struct tcp_client_es_t *es){
	struct pbuf *ptr;
	err_t wr_err = ERR_OK;

	while ((wr_err == ERR_OK) && (es->buf != NULL) && (es->buf->len <= tcp_sndbuf(tpcb))){
		ptr = es->buf;
		/** enqueue data for transmission */
		wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
		if (wr_err == ERR_OK){
			es->buf = ptr->next;
			if(es->buf != NULL){
				pbuf_ref(es->buf);
			}
			pbuf_free(ptr);
		}
		else if(wr_err == ERR_MEM){
			es->buf = ptr;
#if TCP_DEBUG
	LOG_DEBUG(TAG, "ERR_MEM, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
		}
		else{
#if TCP_DEBUG
	LOG_DEBUG(TAG, "orther error, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
		}
	}
}


static void tcp_client_disconnect_handler(struct tcp_pcb *tpcb, tcp_handler_t *handle){
	tcp_connection_t *conn = (tcp_connection_t *)handle -> connection;
#if TCP_DEBUG
	if(conn -> name != NULL) LOG_DEBUG(TAG, "%s call tcp_client_disconnect_handler.", conn -> name);
	else LOG_DEBUG(TAG, "call tcp_client_disconnect_handler.");
#endif /* TCP_DEBUG */

	tcp_arg(tpcb, NULL);
	tcp_sent(tpcb, NULL);
	tcp_recv(tpcb, NULL);
	tcp_err(tpcb, NULL);
	tcp_poll(tpcb, NULL, 0);

	if (handle -> es != NULL) {
		mem_free(handle -> es);
	}
	else{
#if TCP_DEBUG
	LOG_DEBUG(TAG, "es = NULL, line %d function %s file %s", __LINE__, __FUNCTION__, __FILE__);
#endif /* TCP_DEBUG */
	}

	tcp_close(tpcb);

	if(conn -> event_handler != NULL) conn -> event_handler(TCP_CLIENT_EVENT_DISCONNECTED, conn -> parameter);
}

#endif /* ENABLE_TCP */
