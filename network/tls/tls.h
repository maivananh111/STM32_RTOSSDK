/*
 * tls.h
 *
 *  Created on: Jun 26, 2023
 *      Author: anh
 */

#ifndef NETWORK_TLS_TLS_H_
#define NETWORK_TLS_TLS_H_

#include "net_protocols_config.h"

#if ENABLE_TRANSPORT_LAYER_SERCURITY

#ifdef __cplusplus
extern "C"{
#endif

#include "mbedtls/mbedtls_config.h"
#include "mbedtls/ssl.h"
#include "mbedtls/net_sockets.h"
#include "mbedtls/entropy.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/debug.h"
#include "mbedtls/x509.h"




typedef enum{
	TLS_ERR_MEM,
	TLS_ERR_ARG,
	TLS_ERR_NONE,
	TLS_ERR_SET_HOSTNAME,
	TLS_ERR_SET_CONFIG,
	TLS_ERR_SET_ALPN,
	TLS_ERR_CRT_PARSE,
	TLS_ERR_CTR_DRBG,
	TLS_ERR_SETUP,

	TLS_ERR_CERT_INVALID,
	TLS_ERR_HANDSHAKE,
	TLS_ERR_READ,
	TLS_ERR_WRITE,
	TLS_ERR_PEER_CLOSE_NOTIFY,
} tls_err_t;

typedef enum{
	TLS_MODE_CLIENT = 0,
	TLS_MODE_SERVER,
} tls_mode_t;


class tls_t{
	public:
		tls_t(tls_mode_t mode = TLS_MODE_CLIENT);

		/**
		 * Configure.
		 */
		tls_err_t tls_set_socketfd(int socketfd);
		tls_err_t tls_set_hostname(const char *hostname);
		tls_err_t tls_set_server_certificate(const char *server_cert_pem);
		tls_err_t tls_set_renegotiation(bool renegotiation);
		tls_err_t tls_set_alpn(char **alpn_protocol);

		/**
		 * Action.
		 */
		tls_err_t tls_create(void);
		tls_err_t tls_handshake(void);
		tls_err_t tls_read(char *data, int len, int *read_len);
		tls_err_t tls_write(char *data, int len, int *written_len);
		tls_err_t tls_delete(void);

		tls_err_t tls_get_client_session(void);
		tls_err_t tls_free_client_session(void);

		tls_err_t tls_get_server_create_session(void);
		tls_err_t tls_get_server_delete_session(void);


	private:
		tls_mode_t _mode = TLS_MODE_CLIENT;
		char *_hostname = NULL;
		bool _use_crt_bundle = false;
		char *_server_cert_pem = NULL;
		bool _renegotiation = true;
		char **_alpn_protocols = NULL;

		mbedtls_net_context _net;
		mbedtls_ssl_context _ssl;
		mbedtls_ssl_config _conf;
		mbedtls_x509_crt _cert;
		mbedtls_ctr_drbg_context _ctr_drbg;
		mbedtls_entropy_context _entropy;

		tls_err_t _error;


		tls_err_t client_config(void);
		tls_err_t server_config(void);
		tls_err_t cleanup(void);
};

#ifdef __cplusplus
}
#endif

#endif /* ENABLE_TRANSPORT_LAYER_SERCURITY */

#endif /* NETWORK_TLS_TLS_H_ */
