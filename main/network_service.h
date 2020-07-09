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

enum network_txn_type {
	NETWORK_TXN_POST,
	NETWORK_TXN_GET,
};

struct network_txn {
	struct service *sender;

	esp_http_client_config_t cfg;

	enum network_txn_type type;
	union {
		struct {
			const char *data;
			int len;
		} post;
		struct {
			int len;
		} get;
	};

	esp_err_t err;
	int result;
};

int network_txn_perform(struct service *service, struct network_txn *txn);

#endif /* __NETWORK_SERVICE_H__ */
