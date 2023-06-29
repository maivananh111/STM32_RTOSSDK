/*
 * tls.cpp
 *
 *  Created on: Jun 26, 2023
 *      Author: anh
 */
#include "net_protocols_config.h"

#if ENABLE_TRANSPORT_LAYER_SERCURITY

#include "tls/tls.h"

#include "string.h"
#include "system/log.h"



static const char *TAG = (const char *)"TLS";

static void tls_transport_error_debug(char *mess, int ret, const char *func, int line){
	LOG_ERROR(TAG, "%s[-0x%04x] in %s[%d]", mess, -ret, func, line);
}



/**
 * Constructor.
 */
tls_t::tls_t(tls_mode_t mode){
	_mode = mode;
	_hostname = NULL;
	_use_crt_bundle = false;
	_server_cert_pem = NULL;
	_renegotiation = true;
	_alpn_protocols = NULL;

	_error = TLS_ERR_NONE;
	_net.fd = -1;
}

/**
 * Configure.
 */
tls_err_t tls_t::tls_set_socketfd(int socketfd){
	_net.fd = socketfd;
	return _error;
}
tls_err_t tls_t::tls_set_hostname(const char *hostname){
	_hostname = (char *)hostname;
	return _error;
}
tls_err_t tls_t::tls_set_server_certificate(const char *server_cert_pem){
	_server_cert_pem = (char *)server_cert_pem;
	return _error;
}
tls_err_t tls_t::tls_set_renegotiation(bool renegotiation){
	_renegotiation  = renegotiation;
	return _error;
}
tls_err_t tls_t::tls_set_alpn(char **alpn_protocol){
	if(alpn_protocol == NULL) return _error;
	_alpn_protocols = (char **)malloc(sizeof(alpn_protocol));

	for(uint8_t i=0; i<sizeof(alpn_protocol); i++){
		_alpn_protocols[i] = (char *)malloc(strlen(alpn_protocol[i]));
		memcpy(_alpn_protocols[i], alpn_protocol[i], strlen(alpn_protocol[i]));
 	}

	return _error;
}




tls_err_t tls_t::client_config(void){
	int ret;
	int hostlen = strlen(_hostname);
    char *use_host = strndup(_hostname, hostlen);

    ret = mbedtls_ssl_set_hostname(&_ssl, use_host);
    if(ret != 0){
		cleanup();
		tls_transport_error_debug((char *)"Error set tls host name", ret, __func__, __LINE__);
		free(use_host);
		_error = TLS_ERR_SET_HOSTNAME;
		return _error;
    }
    free(use_host);

    ret = mbedtls_ssl_config_defaults(&_conf, MBEDTLS_SSL_IS_CLIENT, MBEDTLS_SSL_TRANSPORT_STREAM, MBEDTLS_SSL_PRESET_DEFAULT);
	if(ret != 0){
		cleanup();
		tls_transport_error_debug((char *)"Error set tls default config", ret, __func__, __LINE__);
		_error = TLS_ERR_SET_CONFIG;
		return _error;
	}

	mbedtls_ssl_conf_renegotiation(&_conf, MBEDTLS_SSL_RENEGOTIATION_ENABLED);

	if(_alpn_protocols != NULL){
		ret = mbedtls_ssl_conf_alpn_protocols(&_conf, (const char **)_alpn_protocols);
		if(ret != 0){
			cleanup();
			tls_transport_error_debug((char *)"Error config tls ALPN protocols", ret, __func__, __LINE__);
			_error = TLS_ERR_SET_ALPN;
			return _error;
		}
	}

	if(_server_cert_pem != NULL){
		int cert_len = strlen(_server_cert_pem) + 1U;
		mbedtls_x509_crt_init(&_cert);
		ret = mbedtls_x509_crt_parse(&_cert, (const unsigned char *)_server_cert_pem, cert_len);
		if(ret != 0){
			cleanup();
			tls_transport_error_debug((char *)"Error parse certificate pem format", ret, __func__, __LINE__);
			_error = TLS_ERR_CRT_PARSE;
			return _error;
		}

		mbedtls_ssl_conf_authmode(&_conf, MBEDTLS_SSL_VERIFY_REQUIRED);
		mbedtls_ssl_conf_ca_chain(&_conf, &_cert, NULL);
	}
	else if(_use_crt_bundle == true){
		/**
		 * Crt bundle attach.
		 */
	}
	else {
		mbedtls_ssl_conf_authmode(&_conf, MBEDTLS_SSL_VERIFY_NONE);
		tls_transport_error_debug((char *)"Connect without certificate", ret, __func__, __LINE__);
	}

	return _error;
}
tls_err_t tls_t::server_config(void){
	return _error;
}

tls_err_t tls_t::cleanup(void){
    mbedtls_x509_crt_free(&_cert);
    mbedtls_entropy_free(&_entropy);
    mbedtls_ssl_config_free(&_conf);
    mbedtls_ctr_drbg_free(&_ctr_drbg);
    mbedtls_ssl_free(&_ssl);

	return _error;
}


tls_err_t tls_t::tls_create(void){
	int ret = 0;
    mbedtls_ssl_init(&_ssl);
    mbedtls_ctr_drbg_init(&_ctr_drbg);
    mbedtls_ssl_config_init(&_conf);
    mbedtls_entropy_init(&_entropy);

    if(_mode == TLS_MODE_CLIENT) {
    	client_config();
    	if(_error != TLS_ERR_NONE) {
    		tls_transport_error_debug((char *)"Error config tls client", ret, __func__, __LINE__);
    		return _error;
    	}
    }
    else if(_mode == TLS_MODE_SERVER) {
    	server_config();
    	if(_error != TLS_ERR_NONE) {
    		tls_transport_error_debug((char *)"Error config tls server", ret, __func__, __LINE__);
    		return _error;
    	}
    }

	ret = mbedtls_ctr_drbg_seed(&_ctr_drbg, mbedtls_entropy_func, &_entropy, NULL, 0);
	if(ret != 0){
		cleanup();
		tls_transport_error_debug((char *)"Error set tls ctrdrbg seed", ret, __func__, __LINE__);
		_error = TLS_ERR_CTR_DRBG;
		return _error;
	}
	mbedtls_ssl_conf_rng(&_conf, mbedtls_ctr_drbg_random, &_ctr_drbg);

	ret = mbedtls_ssl_setup(&_ssl, &_conf);
	if(ret != 0){
		cleanup();
		tls_transport_error_debug((char *)"Error tls setup", ret, __func__, __LINE__);
		_error = TLS_ERR_SETUP;
		return _error;
	}

    mbedtls_ssl_set_bio(&_ssl, &_net, mbedtls_net_send, mbedtls_net_recv, NULL);

    return _error;
}

tls_err_t tls_t::tls_handshake(void){
	int ret = 0;

	if(_error == TLS_ERR_NONE){
		ret = mbedtls_ssl_handshake(&_ssl);
		if(ret != 0) {
			if(ret != MBEDTLS_ERR_SSL_WANT_READ && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
				cleanup();
				tls_transport_error_debug((char *)"Connection handshake fail", ret, __func__, __LINE__);
				int flags;
				if((flags = mbedtls_ssl_get_verify_result(&_ssl)) != 0){
					tls_transport_error_debug((char *)"Fail to verify certificate", ret, __func__, __LINE__);

					char buf[100];
					bzero(buf, sizeof(buf));
					mbedtls_x509_crt_verify_info(buf, sizeof(buf), "  ! ", flags);
					LOG_DEBUG(TAG, "verification info: %s", buf);

					_error = TLS_ERR_CERT_INVALID;
					return _error;
				}
				_error = TLS_ERR_HANDSHAKE;
			}
		}
	}
	return _error;
}

tls_err_t tls_t::tls_read(char *data, int len, int *read_len){
	int ret = 0;

	if(_error == TLS_ERR_NONE){
		ret = mbedtls_ssl_read(&_ssl, (unsigned char *)data, len);
		if(ret < 0) {
			if(ret != MBEDTLS_ERR_SSL_WANT_READ  && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
				tls_transport_error_debug((char *)"Fail to read data from tls", ret, __func__, __LINE__);
				_error = TLS_ERR_READ;
			}
			if(ret == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) {
				tls_transport_error_debug((char *)"Connection has been close by peer", ret, __func__, __LINE__);
				_error = TLS_ERR_PEER_CLOSE_NOTIFY;
				ret = 0;
			}
		}
		*read_len = ret;
	}

	return _error;
}

tls_err_t tls_t::tls_write(char *data, int len, int *written_len){
	int datalen = len;
	int written = 0;
	int write_len = datalen;

    while(written < datalen) {
        if(write_len > MBEDTLS_SSL_OUT_CONTENT_LEN) write_len = MBEDTLS_SSL_OUT_CONTENT_LEN;
        if(datalen > MBEDTLS_SSL_OUT_CONTENT_LEN) LOG_DEBUG(TAG, "Fragmenting data of excessive size: %d", datalen);

        int ret = mbedtls_ssl_write(&_ssl, (unsigned char*)(data + written), write_len);
        if(ret <= 0) {
        	if(ret != MBEDTLS_ERR_SSL_WANT_READ  && ret != MBEDTLS_ERR_SSL_WANT_WRITE) {
				tls_transport_error_debug((char *)"Fail to write data to tls", ret, __func__, __LINE__);
				_error = TLS_ERR_WRITE;
                goto _return;
            }
        	else {
                LOG_DEBUG(TAG, "End of tls write, exited.");
                goto _return;
            }
        }
        written += ret;
        write_len = datalen - written;
    }

    _return:
    *written_len = written;

	return _error;
}

tls_err_t tls_t::tls_delete(void){
	cleanup();
	_net.fd = -1;
	_error = TLS_ERR_NONE;

	return _error;
}


#endif /* ENABLE_TRANSPORT_LAYER_SERCURITY */
