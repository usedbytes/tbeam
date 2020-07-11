// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __NETWORK_SERVICE_H__
#define __NETWORK_SERVICE_H__

#include "esp_http_client.h"
#include "service_manager.h"

#define NETWORK_CMD_SCOPE SERVICE_SCOPE('n', 'e')
#define NETWORK_CMD(_x)   SERVICE_CMD(NETWORK_CMD_SCOPE, _x)

#define NETWORK_CMD_NETWORK_STATUS NETWORK_CMD(3)
#define NETWORK_CMD_TXN_RESULT     NETWORK_CMD(5)

enum network_status {
	NETWORK_STATUS_CONNECTED,
	NETWORK_STATUS_FAILED,
};

struct service *network_service_register();

int network_subscribe_network_status(struct service *service, struct service *subscriber);
int network_unsubscribe_network_status(struct service *service, struct service *subscriber);

struct network_txn;

typedef esp_err_t (*network_send_chunk_fn)(esp_http_client_handle_t client, char *buf, uint32_t len);
typedef esp_err_t (*network_receive_chunk_fn)(esp_http_client_handle_t client, char *buf, int len);

typedef esp_err_t (*network_txn_send_cb)(struct network_txn *, network_send_chunk_fn, esp_http_client_handle_t);
typedef esp_err_t (*network_txn_receive_cb)(struct network_txn *, network_receive_chunk_fn, esp_http_client_handle_t);

/* Structure used to perform HTTP requests via the network service
 *
 * The requestor should populate cfg, content_type (if necessary) and
 * send_cb/receive_cb if it wishes to send and/or receive data.
 *
 * If sender is non-NULL, then upon completion of the transaction, the structure
 * will be sent back to sender in a NETWORK_CMD_TXN_RESULT message. If sender
 * is NULL, the network service will call free() on the transaction structure
 * upon completion.
 *
 * The send and receive callbacks will be called from the network service
 * task, and they should call the chunk send/receive function that's passed
 * as an argument in order to send/receive chunks of data.
 *
 * In the case of sending, the network_send_chunk_fn should be called with
 * an empty chunk (buf = NULL and len = 0) when all data has been sent.
 *
 * In the case of receiving, the network_receive_chunk_fn will return 0 when
 * there is no more data to read, and < 0 in the case of error.
 *
 * The callbacks-calling-callbacks approach avoids any extra data copies by
 * letting the transaction implementation write/read directly into/from the
 * HTTP client library
 *
 * Note: Because of the limitations of the esp_http_client library, all requests
 * which send data (send_cb is non-NULL) will use the POST method.
 */
struct network_txn {
	struct service *sender;

	esp_http_client_config_t cfg;

	// Content-Type header
	const char *content_type;

	// Callbacks for performing send and receive
	network_txn_send_cb send_cb;
	network_txn_receive_cb receive_cb;

	// Filled in by transaction handler
	esp_err_t err;
	int result;
};

int network_txn_perform(struct service *service, struct network_txn *txn);

/* Helpers for a simple static POST.
 *
 * The caller needs to set up base.cfg.url at a minimum; and base.content_type
 * if necessary.
 *
 * All memory allocated by network_new_static_post() can be safely freed by
 * calling free() on the returned structure. This means that it is also safe
 * to call network_new_static_post() with requestor set to NULL and the network
 * service will clean up the structure, as long as the caller hasn't set any
 * fields to point to dynamically allocated memory.
 */
struct network_static_post_txn {
	struct network_txn base;
	int len;
	char data[];
};
struct network_static_post_txn *network_new_static_post(struct service *requestor, char *data, int data_len);
esp_err_t network_static_post_send(struct network_txn *base, network_send_chunk_fn send_chunk,
				   esp_http_client_handle_t client);

/* Helpers for a simple GET which writes to stdout.
 *
 * The caller needs to set up base.cfg.url at a minimum.
 *
 * All memory allocated by network_new_echo_get() can be safely freed by
 * calling free() on the returned structure. This means that it is also safe
 * to call network_new_echo_get() with requestor set to NULL and the network
 * service will clean up the structure, as long as the caller hasn't set any
 * fields to point to dynamically allocated memory.
 */
struct network_txn *network_new_echo_get(struct service *requestor);
esp_err_t network_echo_get_receive(struct network_txn *base, network_receive_chunk_fn recv_chunk,
				   esp_http_client_handle_t client);

#endif /* __NETWORK_SERVICE_H__ */
