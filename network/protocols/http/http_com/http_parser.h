/*
 * http_parser.h
 *
 *  Created on: Jun 20, 2023
 *      Author: anh
 */

#ifndef HTTP_COM_HTTP_PARSER_H_
#define HTTP_COM_HTTP_PARSER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <sys/types.h>
#include <stdint.h>



#ifndef HTTP_PARSER_STRICT
#define HTTP_PARSER_STRICT 1
#endif

#define HTTP_PARSER_VERSION_MAJOR 2
#define HTTP_PARSER_VERSION_MINOR 7
#define HTTP_PARSER_VERSION_PATCH 0

#ifndef HTTP_MAX_HEADER_SIZE
#define HTTP_MAX_HEADER_SIZE (80*1024)
#endif

typedef struct http_parser http_parser;
typedef struct http_parser_settings http_parser_settings;

typedef int (*http_data_cb) (http_parser*, const char *at, size_t length);
typedef int (*http_cb) (http_parser*);


#define HTTP_METHOD_MAP(XX)         \
  XX(0,  DELETE,      DELETE)       \
  XX(1,  GET,         GET)          \
  XX(2,  HEAD,        HEAD)         \
  XX(3,  POST,        POST)         \
  XX(4,  PUT,         PUT)          \
  /* pathological */                \
  XX(5,  CONNECT,     CONNECT)      \
  XX(6,  OPTIONS,     OPTIONS)      \
  XX(7,  TRACE,       TRACE)        \
  /* WebDAV */                      \
  XX(8,  COPY,        COPY)         \
  XX(9,  LOCK,        LOCK)         \
  XX(10, MKCOL,       MKCOL)        \
  XX(11, MOVE,        MOVE)         \
  XX(12, PROPFIND,    PROPFIND)     \
  XX(13, PROPPATCH,   PROPPATCH)    \
  XX(14, SEARCH,      SEARCH)       \
  XX(15, UNLOCK,      UNLOCK)       \
  XX(16, BIND,        BIND)         \
  XX(17, REBIND,      REBIND)       \
  XX(18, UNBIND,      UNBIND)       \
  XX(19, ACL,         ACL)          \
  /* subversion */                  \
  XX(20, REPORT,      REPORT)       \
  XX(21, MKACTIVITY,  MKACTIVITY)   \
  XX(22, CHECKOUT,    CHECKOUT)     \
  XX(23, MERGE,       MERGE)        \
  /* upnp */                        \
  XX(24, MSEARCH,     M-SEARCH)     \
  XX(25, NOTIFY,      NOTIFY)       \
  XX(26, SUBSCRIBE,   SUBSCRIBE)    \
  XX(27, UNSUBSCRIBE, UNSUBSCRIBE)  \
  /* RFC-5789 */                    \
  XX(28, PATCH,       PATCH)        \
  XX(29, PURGE,       PURGE)        \
  /* CalDAV */                      \
  XX(30, MKCALENDAR,  MKCALENDAR)   \
  /* RFC-2068, section 19.6.1.2 */  \
  XX(31, LINK,        LINK)         \
  XX(32, UNLINK,      UNLINK)       \

enum http_method {
#define XX(num, name, string) HTTP_##name = num,
  HTTP_METHOD_MAP(XX)
#undef XX
};


enum http_parser_type{
	HTTP_REQUEST,
	HTTP_RESPONSE,
	HTTP_BOTH
};

enum http_flags{
	F_CHUNKED               = (1 << 0),
	F_CONNECTION_KEEP_ALIVE = (1 << 1),
	F_CONNECTION_CLOSE      = (1 << 2),
	F_CONNECTION_UPGRADE    = (1 << 3),
	F_TRAILING              = (1 << 4),
	F_UPGRADE               = (1 << 5),
	F_SKIPBODY              = (1 << 6),
	F_CONTENTLENGTH         = (1 << 7),
};


#define HTTP_ERRNO_MAP(XX)                                           \
  /* No error */                                                     \
  XX(OK, "success")                                                  \
                                                                     \
  /* Callback-related errors */                                      \
  XX(CB_message_begin, "the on_message_begin callback failed")       \
  XX(CB_url, "the on_url callback failed")                           \
  XX(CB_header_field, "the on_header_field callback failed")         \
  XX(CB_header_value, "the on_header_value callback failed")         \
  XX(CB_headers_complete, "the on_headers_complete callback failed") \
  XX(CB_body, "the on_body callback failed")                         \
  XX(CB_message_complete, "the on_message_complete callback failed") \
  XX(CB_status, "the on_status callback failed")                     \
  XX(CB_chunk_header, "the on_chunk_header callback failed")         \
  XX(CB_chunk_complete, "the on_chunk_complete callback failed")     \
                                                                     \
  /* Parsing-related errors */                                       \
  XX(INVALID_EOF_STATE, "stream ended at an unexpected time")        \
  XX(HEADER_OVERFLOW,                                                \
     "too many header bytes seen; overflow detected")                \
  XX(CLOSED_CONNECTION,                                              \
     "data received after completed connection: close message")      \
  XX(INVALID_VERSION, "invalid HTTP version")                        \
  XX(INVALID_STATUS, "invalid HTTP status code")                     \
  XX(INVALID_METHOD, "invalid HTTP method")                          \
  XX(INVALID_URL, "invalid URL")                                     \
  XX(INVALID_HOST, "invalid host")                                   \
  XX(INVALID_PORT, "invalid port")                                   \
  XX(INVALID_PATH, "invalid path")                                   \
  XX(INVALID_QUERY_STRING, "invalid query string")                   \
  XX(INVALID_FRAGMENT, "invalid fragment")                           \
  XX(LF_EXPECTED, "LF character expected")                           \
  XX(INVALID_HEADER_TOKEN, "invalid character in header")            \
  XX(INVALID_CONTENT_LENGTH,                                         \
     "invalid character in content-length header")                   \
  XX(UNEXPECTED_CONTENT_LENGTH,                                      \
     "unexpected content-length header")                             \
  XX(INVALID_CHUNK_SIZE,                                             \
     "invalid character in chunk size header")                       \
  XX(INVALID_CONSTANT, "invalid constant string")                    \
  XX(INVALID_INTERNAL_STATE, "encountered unexpected internal state")\
  XX(STRICT, "strict mode assertion failed")                         \
  XX(PAUSED, "parser is paused")                                     \
  XX(UNKNOWN, "an unknown error occurred")


#define HTTP_ERRNO_GEN(n, s) HPE_##n,
enum http_errno {
    HTTP_ERRNO_MAP(HTTP_ERRNO_GEN)
};
#undef HTTP_ERRNO_GEN

#define HTTP_PARSER_ERRNO(p)            ((enum http_errno) (p)->http_errno)

struct http_parser {
	/** PRIVATE **/
	unsigned int type : 2;         /* enum http_parser_type */
	unsigned int flags : 8;        /* F_* values from 'flags' enum; semi-public */
	unsigned int state : 7;        /* enum state from http_parser.c */
	unsigned int header_state : 7; /* enum header_state from http_parser.c */
	unsigned int index : 7;        /* index into current matcher */
	unsigned int lenient_http_headers : 1;

	uint32_t nread;                /* # bytes read in various scenarios */
	uint64_t content_length;       /* # bytes in body (0 if no Content-Length header) */

	/** READ-ONLY **/
	unsigned short http_major;
	unsigned short http_minor;
	unsigned int status_code : 16; /* responses only */
	unsigned int method : 8;       /* requests only */
	unsigned int http_errno : 7;
	unsigned int upgrade : 1;

	/** PUBLIC **/
	void *data; /* A pointer to get hook to the "connection" or "socket" object */
};


struct http_parser_settings {
	http_cb      on_message_begin;
	http_data_cb on_url;
	http_data_cb on_status;
	http_data_cb on_header_field;
	http_data_cb on_header_value;
	http_cb      on_headers_complete;
	http_data_cb on_body;
	http_cb      on_message_complete;
	http_cb      on_chunk_header;
	http_cb      on_chunk_complete;
};


enum http_parser_url_fields{
	UF_SCHEMA           = 0,
	UF_HOST             = 1,
	UF_PORT             = 2,
	UF_PATH             = 3,
	UF_QUERY            = 4,
	UF_FRAGMENT         = 5,
	UF_USERINFO         = 6,
	UF_MAX              = 7,
};


struct http_parser_url {
	uint16_t field_set;           /* Bitmask of (1 << UF_*) values */
	uint16_t port;                /* Converted UF_PORT string */

	struct {
		uint16_t off;               /* Offset into buffer in which field starts */
		uint16_t len;               /* Length of run in buffer */
	} field_data[UF_MAX];
};

unsigned long http_parser_version(void);

void http_parser_init(http_parser *parser, enum http_parser_type type);

void http_parser_settings_init(http_parser_settings *settings);

size_t http_parser_execute(http_parser *parser, const http_parser_settings *settings, const char *data, size_t len);

int http_should_keep_alive(const http_parser *parser);

const char *http_method_str(enum http_method m);

const char *http_errno_name(enum http_errno err);

const char *http_errno_description(enum http_errno err);

void http_parser_url_init(struct http_parser_url *u);

int http_parser_parse_url(const char *buf, size_t buflen, int is_connect, struct http_parser_url *u);

void http_parser_pause(http_parser *parser, int paused);

int http_body_is_final(const http_parser *parser);

#ifdef __cplusplus
}
#endif

#endif /* HTTP_COM_HTTP_PARSER_H_ */
