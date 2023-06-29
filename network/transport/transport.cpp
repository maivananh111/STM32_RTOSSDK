/*
 * transport.cpp
 *
 *  Created on: Jun 26, 2023
 *      Author: anh
 */
#include "net_protocols_config.h"

#if ENABLE_NETWORK_TRANSPORT

#include "transport/transport.h"

#include "lwip/err.h"
#include "lwip/timeouts.h"

#include "system/log.h"

static const char *TAG = (const char *)"Transport";


static void transport_error_debug(char *mess, int ret, const char *func, int line){
	LOG_ERROR(TAG, "%s[-0x%04x] in %s[%d]", mess, -ret, func, line);
}




transport_t::transport_t(void){
	_config = NULL;

	_state = TRANSPORT_STATE_DISCONNECTED;
	_error = TRANSPORT_ERR_NONE;
}



/**
 * Set socket option and mode.
 */
transport_error_t transport_t::set_socket_timeout(void){
	int ret;
	if(_error == TRANSPORT_ERR_NONE){
		if(_config->timeout_s != 0){
			_timeout.tv_sec = _config->timeout_s;
			_timeout.tv_usec = 0U;
			ret = lwip_setsockopt(_sockfd, SOL_SOCKET, SO_RCVTIMEO, &_timeout, sizeof(struct timeval));
			if(ret < 0){
				transport_error_debug((char *)"Error set socket option receive timeout", ret, __func__, __LINE__);
				goto _error;
			}
			ret = lwip_setsockopt(_sockfd, SOL_SOCKET, SO_SNDTIMEO, &_timeout, sizeof(struct timeval));
			if(ret < 0){
				transport_error_debug((char *)"Error set socket option send timeout", ret, __func__, __LINE__);
				goto _error;
			}
		}
	}
	return _error;

	_error:
	_error = TRANSPORT_ERR_SET_OPTION;
	return _error;
}

transport_error_t transport_t::set_socket_keep_alive(void){
	int ret;

	if(_error == TRANSPORT_ERR_NONE){
		if(_config->keepalive_idle_s != 0 || _config->keepalive_interval_s != 0 || _config->keepalive_count != 0) {
			int keep_alive_enable = 1;

			ret = lwip_setsockopt(_sockfd, SOL_SOCKET, SO_KEEPALIVE, &keep_alive_enable, sizeof(keep_alive_enable));
			if(ret != 0){
				transport_error_debug((char *)"Error set socket option keep alive", ret, __func__, __LINE__);
				goto _error;
			}
			ret = lwip_setsockopt(_sockfd, IPPROTO_TCP, TCP_KEEPIDLE, &_config->keepalive_idle_s, sizeof(_config->keepalive_idle_s));
			if(ret != 0){
				transport_error_debug((char *)"Error set socket option keep alive idle", ret, __func__, __LINE__);
				goto _error;
			}
			ret = lwip_setsockopt(_sockfd, IPPROTO_TCP, TCP_KEEPINTVL, &_config->keepalive_interval_s, sizeof(_config->keepalive_interval_s));
			if(ret != 0){
				transport_error_debug((char *)"Error set socket option keep alive interval", ret, __func__, __LINE__);
				goto _error;
			}
			ret = lwip_setsockopt(_sockfd, IPPROTO_TCP, TCP_KEEPCNT, &_config->keepalive_count, sizeof(_config->keepalive_count));
			if(ret != 0){
				transport_error_debug((char *)"Error set socket option keep alive count", ret, __func__, __LINE__);
				goto _error;
			}
		}
	}
	return _error;

	_error:
	_error = TRANSPORT_ERR_SET_OPTION;
	return _error;
}

transport_error_t transport_t::set_socket_non_block_mode(bool mode){
	int ret;
	int socket_flag = 0;

	if(_error == TRANSPORT_ERR_NONE){
		socket_flag = lwip_fcntl(_sockfd, F_GETFL, 0);
		if(socket_flag < 0){
			transport_error_debug((char *)"Error get socket flag", socket_flag, __func__, __LINE__);
			goto _error;
		}

		if(mode) socket_flag |= O_NONBLOCK;
		else 	 socket_flag &=~ O_NONBLOCK;

		ret = lwip_fcntl(_sockfd, F_SETFL, socket_flag);
		if(ret < 0){
			transport_error_debug((char *)"Error set socket option block/non-block mode", ret, __func__, __LINE__);
			goto _error;
		}
	}
	return _error;

	_error:
	_error = TRANSPORT_ERR_SET_OPTION;
	return _error;
}

transport_error_t transport_t::set_socket_ifname(void){
	int ret;

	if(_error == TRANSPORT_ERR_NONE){
		if(_config->socket_if_name != NULL){
			struct ifreq ifr_name;
			memcpy(ifr_name.ifr_name, _config->socket_if_name, IFNAMSIZ);

			ret = lwip_setsockopt(_sockfd, SOL_SOCKET, SO_BINDTODEVICE, &ifr_name, sizeof(struct ifreq));
			if(ret != 0){
				transport_error_debug((char *)"Error set socket option interface name", ret, __func__, __LINE__);
				_error = TRANSPORT_ERR_SET_IFNAME;
				return _error;
			}
		}
	}

	return _error;
}



/**
 * Socket progress.
 */
transport_error_t transport_t::create_socket(void){
	int ret;

	if(_error == TRANSPORT_ERR_NONE){
		_sockfd = INVALID_SOCKET_FD;

		struct addrinfo _hints;
		memset(&_hints, 0, sizeof(struct addrinfo));
		_hints.ai_family = AF_INET;
		_hints.ai_socktype = SOCK_STREAM;
		_hints.ai_protocol = IPPROTO_TCP;

		if(_config->hostname != NULL && _config->ipaddress == NULL){
			ret = lwip_getaddrinfo(_config->hostname, NULL, &_hints, &_addr_info);
			if(ret != 0 ||_addr_info == NULL){
				transport_error_debug((char *)"Error resolve host name to address info", ret, __func__, __LINE__);
				_error = TRANSPORT_ERR_RESOLVE_HOSTNAME;
				return _error;
			}
		}
		else if(_config->ipaddress != NULL){
			char port_str[5];
			memset(port_str, '\0', 5);
			sprintf(port_str, "%d", _config->port);
			ret = lwip_getaddrinfo(NULL, (const char *)port_str, &_hints, &_addr_info);
			if(ret != 0 ||_addr_info == NULL){
				transport_error_debug((char *)"Error resolve ip address to address info", ret, __func__, __LINE__);
				_error = TRANSPORT_ERR_RESOLVE_HOSTNAME;
				return _error;
			}
		}
		else{
			transport_error_debug((char *)"Error invalid argument hostname and server ip address", 0, __func__, __LINE__);
			_error = TRANSPORT_ERR_RESOLVE_HOSTNAME;
			return _error;
		}

		_sockfd = lwip_socket(_addr_info->ai_family, _addr_info->ai_socktype, _addr_info->ai_protocol);
		if(_sockfd < 0){
			transport_error_debug((char *)"Error create socket with host name", ret, __func__, __LINE__);
			lwip_freeaddrinfo(_addr_info);
			_error = TRANSPORT_ERR_CREATE;
			return _error;
		}
		if(_addr_info->ai_family == AF_INET){
			struct sockaddr_in *p = (struct sockaddr_in *)_addr_info->ai_addr; // convert to ipv4 struct.
			p->sin_port = lwip_htons(_config->port);
			_host_ipaddr.addr = (uint32_t)p->sin_addr.s_addr;
			memcpy(&_sock_addr, p, sizeof(struct sockaddr ));
		}
		else{
			transport_error_debug((char *)"Socket protocol family unsupported", ret, __func__, __LINE__);
			lwip_freeaddrinfo(_addr_info);
			_error = TRANSPORT_ERR_INVALID_PROTOCOL;
			return _error;
		}
		lwip_freeaddrinfo(_addr_info);
	}

	return _error;
}

transport_error_t transport_t::set_socket_option(void){
	set_socket_timeout();
    set_socket_keep_alive();
    set_socket_ifname();

	return _error;
}

transport_error_t transport_t::open_socket(void){
	int ret;

	if(_error == TRANSPORT_ERR_NONE){
		set_socket_non_block_mode(true);

		if(_error == TRANSPORT_ERR_NONE){
			ret = lwip_connect(_sockfd, (struct sockaddr *)&_sock_addr, sizeof(struct sockaddr));
			if(ret < 0 && errno == EINPROGRESS) {
				fd_set fdset;
				struct timeval tv;

				if(_config->socket_non_block == true) return _error; // return oke if socket non-block mode.

				FD_ZERO(&fdset);
				FD_SET(_sockfd, &fdset);
				tv.tv_sec = DEFUALT_SOCKET_CONNECT_TIMEOUT;
				tv.tv_usec = 0U;

				if(lwip_select(_sockfd + 1, NULL, &fdset, NULL, &tv) < 0){
					LOG_ERROR(TAG, "Error select socket, returned -0x%04x", -ret);
					_error = TRANSPORT_ERR_POLL;
					return _error;
				}
				else if(ret == 0) {
					LOG_ERROR(TAG, "Error connection timeout, returned -0x%04x", -ret);
					_error = TRANSPORT_ERR_OPEN;
					return _error;
				}
				else{
					int err;
					socklen_t err_len = sizeof(err);

					ret = lwip_getsockopt(_sockfd, SOL_SOCKET, SO_ERROR, &err, &err_len);
					if(ret < 0) {
						LOG_ERROR(TAG, "Error get socket option, returned -0x%04x", -ret);
						_error = TRANSPORT_ERR_GET_OPTION;
						return _error;
					}
					else if(err > 0){
						LOG_ERROR(TAG, "Error open because socket problem, returned -0x%04x", -ret);
						_error = TRANSPORT_ERR_OPEN;
						return _error;
					}
				}
			}
			set_socket_non_block_mode(_config->socket_non_block);
		}
	}

	return _error;
}

transport_error_t transport_t::close_socket(void){
	int ret;

	ret = lwip_close(_sockfd);
	if(ret != 0){
		transport_error_debug((char *)"Error close socket", ret, __func__, __LINE__);
		_error = TRANSPORT_ERR_CLOSE;
		return _error;
	}
	_error = TRANSPORT_ERR_NONE;
	_sockfd = INVALID_SOCKET_FD;

	return _error;
}




void transport_t::register_event_handler(event_handler_function f_event_handler){
	fp_event_handler = NULL;
	fp_event_handler = f_event_handler;
}


transport_state_t transport_t::connect_to_host(transport_config_t *config){
	_config = config;
	_state = TRANSPORT_STATE_DISCONNECTED;
	_error = TRANSPORT_ERR_NONE;
	_sockfd = INVALID_SOCKET_FD;
	_addr_info = NULL;

	LOG_WARN(TAG, "create_socket");
	create_socket();
	LOG_WARN(TAG, "set_socket_option");
	set_socket_option();
	LOG_WARN(TAG, "open_socket");
	open_socket();

	if(_error == TRANSPORT_ERR_NONE && _state == TRANSPORT_STATE_DISCONNECTED){
#if ENABLE_TRANSPORT_LAYER_SERCURITY
		if(_config->transport_type == TRANSPORT_OVER_SSL){
			_tls.tls_set_socketfd(_sockfd);
			_tls.tls_set_hostname(_config->hostname);
			_tls.tls_set_server_certificate(_config->server_cert_pem);
			_tls.tls_set_renegotiation(_config->renegotiation);
			_tls.tls_set_alpn(_config->alpn_protocols);

			LOG_WARN(TAG, "tls_create");
			if(_tls.tls_create() != TLS_ERR_NONE) {
				_error = TRANSPORT_ERR_TLS_CREATE;
				goto _error_handle;
			}
			LOG_WARN(TAG, "tls_handshake");
			if(_tls.tls_handshake() != TLS_ERR_NONE) {
				_error = TRANSPORT_ERR_TLS_HANDSHAKE;
				goto _error_handle;
			}
			LOG_WARN(TAG, "tls_handshake oke");
			_state = TRANSPORT_STATE_CONNECTED;
		}
		else if(_config->transport_type == TRANSPORT_OVER_TCP)
#endif /* ENABLE_TRANSPORT_LAYER_SERCURITY */
		_state = TRANSPORT_STATE_CONNECTED;
		fp_event_handler(TRANSPORT_EVENT_CONNECTED, NULL);

		return _state;
	}

	_error_handle:
	_state = TRANSPORT_STATE_ERROR;
	fp_event_handler(TRANSPORT_EVENT_ERROR, NULL);
	disconnect_to_host();

	return _state;
}

transport_state_t transport_t::disconnect_to_host(void){
#if ENABLE_TRANSPORT_LAYER_SERCURITY
	_tls.tls_delete();
#endif /* ENABLE_TRANSPORT_LAYER_SERCURITY */
	close_socket();

	_error = TRANSPORT_ERR_NONE;
	_state = TRANSPORT_STATE_DISCONNECTED;
	fp_event_handler(TRANSPORT_EVENT_DISCONNECTED, NULL);

	return _state;
}


transport_state_t transport_t::poll_for_ready(transport_polloption_t pollopt, int timeout){
    if(_state >= TRANSPORT_STATE_CONNECTED){
        int ret;
        fd_set readset, writeset, errorset;
		struct timeval tv;

		FD_ZERO(&readset);
		FD_ZERO(&writeset);
		FD_ZERO(&errorset);
		FD_SET(_sockfd, &readset);
		FD_SET(_sockfd, &writeset);
		FD_SET(_sockfd, &errorset);

		tv.tv_sec = timeout;
		tv.tv_usec = 0U;

		ret = lwip_select(_sockfd + 1,
						 (pollopt & TRANSPORT_POLL_READ)? &readset : NULL,
						 (pollopt & TRANSPORT_POLL_WRITE)? &writeset : NULL,
						 &errorset,
						 (timeout > 0)? &tv : &_timeout
			  );
		if(ret < 0){
			transport_error_debug((char *)"Error socket select", ret, __func__, __LINE__);
			goto _error;
		}
		else if(ret > 0 && FD_ISSET(_sockfd, &errorset)) {
			int err;
			socklen_t err_len = sizeof(err);

			ret = lwip_getsockopt(_sockfd, SOL_SOCKET, SO_ERROR, &err, &err_len);
			transport_error_debug((char *)"Error open socket problem", ret, __func__, __LINE__);
			LOG_ERROR(TAG, "Socket error=%d(%s)", err, strerror(err));

			goto _error;
		}
		else if(ret == 0) {
			transport_error_debug((char *)"Error socket timeout", ret, __func__, __LINE__);
			_state = TRANSPORT_STATE_TIMEOUT;
			fp_event_handler(TRANSPORT_EVENT_TIMEOUT, NULL);

			return _state;
		}
		_state = TRANSPORT_STATE_READY;
    }

    return _state;

    _error:
	fp_event_handler(TRANSPORT_EVENT_ERROR, NULL);
	disconnect_to_host();

	return _state;
}


transport_state_t transport_t::write_data(unsigned char *buffer, int len, int *byte_write){
	int ret;

	if(_state >= TRANSPORT_STATE_CONNECTED){
		poll_for_ready(TRANSPORT_POLL_WRITE);
		if(_state == TRANSPORT_STATE_READY){
			if(_config->transport_type == TRANSPORT_OVER_TCP){
				ret = lwip_send(_sockfd, buffer, len, 0);
				if(ret < 0)
					goto _write_error;
			}
			else if(_config->transport_type == TRANSPORT_OVER_SSL){
				if(_tls.tls_write((char *)buffer, len, &ret) != TLS_ERR_NONE)
					goto _write_error;
			}
			_state = TRANSPORT_STATE_PASS;
			*byte_write = ret;
			fp_event_handler(TRANSPORT_EVENT_WRITTEN, buffer);

			return _state;

			_write_error:
			transport_error_debug((char *)"Error write to socket", ret, __func__, __LINE__);
			_error = TRANSPORT_ERR_WRITE;
			_state = TRANSPORT_STATE_PASS;
			fp_event_handler(TRANSPORT_EVENT_ERROR, NULL);

			return _state;
		}
		else if(_state == TRANSPORT_STATE_TIMEOUT) {
			transport_error_debug((char *)"Error socket timeout", 0, __func__, __LINE__);
			_state = TRANSPORT_STATE_PASS;
		}
		else if(_state == TRANSPORT_STATE_DISCONNECTED) {
			transport_error_debug((char *)"Error socket error, disconnected", 0, __func__, __LINE__);
		}
	}

	*byte_write = ret;
	return _state;
}

transport_state_t transport_t::read_data(unsigned char *buffer, int len, int *byte_read){
	int ret;

	if(_state != TRANSPORT_STATE_DISCONNECTED && _state != TRANSPORT_STATE_ERROR){
		poll_for_ready(TRANSPORT_POLL_READ);
		if(_state == TRANSPORT_STATE_READY){
			if(_config->transport_type == TRANSPORT_OVER_TCP){
			ret = lwip_recv(_sockfd, buffer, len, 0);
				if(ret < 0)goto _read_error;
				else if(ret == 0){
					transport_error_debug((char *)"Error socket connection has been closed.", ret, __func__, __LINE__);
					goto _read_error;
				}
			}
			else if(_config->transport_type == TRANSPORT_OVER_SSL){
				tls_err_t err = _tls.tls_read((char *)buffer, len, &ret);
				if(err == TLS_ERR_PEER_CLOSE_NOTIFY){
					fp_event_handler(TRANSPORT_EVENT_DISCONNECTED_BY_PEER, NULL);
					goto _read_error;
				}
				else if(err != TLS_ERR_PEER_CLOSE_NOTIFY && err != TLS_ERR_NONE){
					fp_event_handler(TRANSPORT_EVENT_ERROR, NULL);
					goto _read_error;
				}
			}

			_state = TRANSPORT_STATE_PASS;
			*byte_read = ret;
			fp_event_handler(TRANSPORT_EVENT_ON_DATA, buffer);

			return _state;

			_read_error:
			transport_error_debug((char *)"Error read from socket", ret, __func__, __LINE__);
			disconnect_to_host();
			connect_to_host(_config);

			return _state;
		}
		else if(_state == TRANSPORT_STATE_TIMEOUT){
			transport_error_debug((char *)"Error socket timeout", 0, __func__, __LINE__);
			_state = TRANSPORT_STATE_PASS;
		}
		else if(_state == TRANSPORT_STATE_DISCONNECTED) {
			transport_error_debug((char *)"Error socket error, disconnected", 0, __func__, __LINE__);
		}
	}

	*byte_read = ret;
	return _state;
}

transport_state_t transport_t::state(void){
	return _state;
}

transport_error_t transport_t::error(void){
	return _error;
}

bool transport_t::can_continue(void){
	return (_state >= TRANSPORT_STATE_CONNECTED);
}

bool transport_t::acction_pass(void){
	return (_state == TRANSPORT_STATE_PASS);
}



#endif /* ENABLE_NETWORK_TRANSPORT */
