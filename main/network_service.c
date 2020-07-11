// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"

#include "list.h"
#include "network_service.h"
#include "wifi.h"

#define TAG "network_service"

#define NETWORK_CMD_WIFI_EVENT SERVICE_CMD_LOCAL(1)
#define NETWORK_CMD_IP_EVENT   SERVICE_CMD_LOCAL(2)

#define NETWORK_CMD_SUBSCRIBE_NETWORK_STATUS   NETWORK_CMD(1)
#define NETWORK_CMD_UNSUBSCRIBE_NETWORK_STATUS NETWORK_CMD(2)

#define NETWORK_CMD_TXN_PERFORM    NETWORK_CMD(4)

static void network_service_fn(void *param);
struct service *network_service_register()
{
	return service_register("network", network_service_fn, configMAX_PRIORITIES - 1, 4096);
}

static void network_event_handler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data)
{
	struct service *service = (struct service *)arg;
	struct service_message smsg;

	if (event_base == WIFI_EVENT) {
		smsg.cmd = NETWORK_CMD_WIFI_EVENT;
		smsg.arg = event_id;
		service_send_message(service, &smsg, 0);
	} else if (event_base == IP_EVENT) {
		smsg.cmd = NETWORK_CMD_IP_EVENT;
		smsg.arg = event_id;
		service_send_message(service, &smsg, 0);
	}
}

struct network_subscriber {
	struct list_node node;
	struct service *service;
};

static bool match_subscriber(struct list_node *node, void *data)
{
	struct network_subscriber *entry = (struct network_subscriber *)node;

	return entry->service == (struct service *)data;
}

static void notify_subscribers_network_status(struct list_node *subs, uint32_t status)
{
	struct network_subscriber *sub;
	struct service_message smsg = {
		.cmd = NETWORK_CMD_NETWORK_STATUS,
		.arg = status,
	};

	list_for_each(subs, struct network_subscriber, node, sub) {
		service_send_message(sub->service, &smsg, 0);
	}
}

static esp_err_t send_chunk(esp_http_client_handle_t client, char *data, uint32_t len)
{
	int wrote, idx = 0;
	char chunk[16];

	sprintf(chunk, "%X\r\n", len);
	wrote = esp_http_client_write(client, chunk, strlen(chunk));
	if (wrote < strlen(chunk)) {
		ESP_LOGE(TAG, "Short write: %d\n", wrote);
		return ESP_FAIL;
	}

	while (len) {
		wrote = esp_http_client_write(client, data + idx, len);
		if (wrote < 0) {
			ESP_LOGE(TAG, "write failed: %d\n", wrote);
			return ESP_FAIL;
		}
		len -= wrote;
		idx += wrote;
	}

	wrote = esp_http_client_write(client, "\r\n", 2);
	if (wrote < 2) {
		ESP_LOGE(TAG, "short write: %d\n", wrote);
		return ESP_FAIL;
	}

	return ESP_OK;
}

esp_err_t perform_txn(struct network_txn *txn)
{
	int ret;
	esp_err_t err = ESP_OK;

	esp_http_client_handle_t client = esp_http_client_init(&txn->cfg);
	if (!client) {
		ESP_LOGE(TAG, "failed HTTP client init");
		return ESP_FAIL;
	}

	// Set content type if necessary
	if (txn->content_type && strlen(txn->content_type)) {
		err = esp_http_client_set_header(client, "Content-Type", txn->content_type);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "failed to set Content-Type");
			goto done;
		}
	}

	// If we've got data to send, open in chunked mode (len = -1)
	err = esp_http_client_open(client, txn->send_cb ? -1 : 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "failed to open HTTP client");
		goto done;
	}

	// Ask transaction provider to send data
	if (txn->send_cb) {
		err = txn->send_cb(txn, send_chunk, client);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "transaction send failed");
			goto done;
		}
	}

	ret = esp_http_client_fetch_headers(client);
	if (ret < 0) {
		ESP_LOGE(TAG, "fetch headers failed\n");
		err = ESP_FAIL;
		goto done;
	}

	txn->result = esp_http_client_get_status_code(client);
	ESP_LOGI(TAG, "HTTP result: %d\n", ret);

	// Ask transaction provider to receive data
	if (txn->receive_cb) {
		err = txn->receive_cb(txn, esp_http_client_read, client);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "transaction receive failed");
			goto done;
		}
	}

done:
	esp_http_client_cleanup(client);
	return err;
}

static void network_service_fn(void *param)
{
	struct service *service = (struct service *)param;
	struct wifi_ctx *ctx = NULL;

	struct list_node status_subs = {
		.next = NULL,
	};

	enum state {
		STOPPED,
		STARTING,
		CONNECTING,
		CONNECTED,
	} state = STOPPED;

#define MAX_RETRIES 5
	int retry = 0;

	while (1) {
		struct service_message smsg;
		if (service_receive_message(service, &smsg, portMAX_DELAY)) {
			continue;
		}

		switch (smsg.cmd) {
		case SERVICE_CMD_STOP:
			if (state != STOPPED) {
				wifi_stop(ctx);
				ctx = NULL;
				state = STOPPED;
			}
			break;
		case SERVICE_CMD_PAUSE:
			break;
		case SERVICE_CMD_START:
			if (state != STOPPED) {
				break;
			}

			ctx = wifi_init(network_event_handler, service);
			if (ctx) {
				state = STARTING;
			} else {
				ESP_LOGE(TAG, "Failed to init wifi");
			}
			break;
		case SERVICE_CMD_RESUME:
			break;

		case NETWORK_CMD_SUBSCRIBE_NETWORK_STATUS:
		{
			struct network_subscriber *sub = calloc(1, sizeof(*sub));
			if (!sub) {
				break;
			}
			sub->service = (struct service *)smsg.arg;
			list_add_tail(&status_subs, &sub->node);
			break;
		}
		case NETWORK_CMD_UNSUBSCRIBE_NETWORK_STATUS:
		{
			struct list_node *node = list_find(&status_subs, match_subscriber, (void *)smsg.arg);
			if (!node) {
				break;
			}
			list_del(&status_subs, node);

			struct network_subscriber *sub = container_of(node, struct network_subscriber, node);
			free(sub);
			break;
		}
		case NETWORK_CMD_TXN_PERFORM:
		{
			struct network_txn *txn = (struct network_txn *)smsg.argp;

			if (state != CONNECTED) {
				txn->err = ESP_ERR_INVALID_STATE;
				txn->result = 599;
			} else {
				txn->err = perform_txn(txn);
			}

			smsg.cmd = NETWORK_CMD_TXN_RESULT;
			if (txn->sender) {
				service_send_message(txn->sender, &smsg, 0);
			} else {
				free(txn);
			}
			break;
		}

		// Network-specific events
		case NETWORK_CMD_WIFI_EVENT:
			switch ((wifi_event_t)smsg.arg) {
			case WIFI_EVENT_STA_START:
				if (state != STARTING) {
					break;
				}
				retry = 0;
				// Fallthrough
			case WIFI_EVENT_STA_DISCONNECTED:
				if (state == STOPPED) {
					// Deliberate, ignore
					break;
				}

				if (retry++ > MAX_RETRIES) {
					wifi_stop(ctx);
					ctx = NULL;
					state = STOPPED;

					// Notify listeners that connection failed
					notify_subscribers_network_status(&status_subs, NETWORK_STATUS_FAILED);

					break;
				}
				esp_wifi_connect();
				state = CONNECTING;
				break;
			case WIFI_EVENT_STA_CONNECTED:
				// Connected but no IP yet
				break;
			case WIFI_EVENT_STA_STOP:
				// This would happen after wifi_stop()
				break;
			default:
				// Unhandled WiFi event
				break;
			}
			break;
		case NETWORK_CMD_IP_EVENT:
			switch ((ip_event_t)smsg.arg) {
			case IP_EVENT_STA_GOT_IP:
				state = CONNECTED;
				retry = 0;

				// Notify listeners that network is available
				notify_subscribers_network_status(&status_subs, NETWORK_STATUS_CONNECTED);
				break;
			default:
				// Unhandled IP event
				break;
			}
			break;
		default:
			// Unknown command
			break;
		}

		// Acknowledge the command
		service_ack(service);
	}
}

int network_subscribe_network_status(struct service *service, struct service *subscriber)
{
	struct service_message smsg = {
		.cmd = NETWORK_CMD_SUBSCRIBE_NETWORK_STATUS,
		.arg = (uint32_t)subscriber,
	};

	return service_send_message(service, &smsg, 0);
}

int network_unsubscribe_network_status(struct service *service, struct service *subscriber)
{
	struct service_message smsg = {
		.cmd = NETWORK_CMD_UNSUBSCRIBE_NETWORK_STATUS,
		.arg = (uint32_t)subscriber,
	};

	return service_send_message(service, &smsg, 0);
}

int network_txn_perform(struct service *service, struct network_txn *txn)
{
	struct service_message smsg = {
		.cmd = NETWORK_CMD_TXN_PERFORM,
		.argp = txn,
	};

	return service_send_message(service, &smsg, 0);
}

esp_err_t network_static_post_send(struct network_txn *base, network_send_chunk_fn send_chunk, esp_http_client_handle_t client)
{
	struct network_static_post_txn *txn = (struct network_static_post_txn *)base;

	esp_err_t err = send_chunk(client, txn->data, txn->len);
	if (err != ESP_OK) {
		return err;
	}

	err = send_chunk(client, NULL, 0);
	if (err != ESP_OK) {
		return err;
	}

	return ESP_OK;
}

struct network_static_post_txn *network_new_static_post(struct service *requestor, char *data, int data_len)
{
	struct network_static_post_txn *txn = calloc(1, sizeof(*txn) + data_len);
	if (!txn) {
		return NULL;
	}

	txn->base.sender = requestor;
	txn->base.cfg.method = HTTP_METHOD_POST;
	txn->base.send_cb = network_static_post_send;
	txn->base.receive_cb = NULL;

	txn->len = data_len;
	if (data) {
		memcpy(txn->data, data, data_len);
	}

	return txn;
}

struct network_txn *network_new_echo_get(struct service *requestor)
{
	struct network_txn *txn = calloc(1, sizeof(*txn));
	if (!txn) {
		return NULL;
	}

	txn->sender = requestor;
	txn->cfg.method = HTTP_METHOD_GET;
	txn->receive_cb = network_echo_get_receive;

	return (struct network_txn *)txn;
}

esp_err_t network_echo_get_receive(struct network_txn *base, network_receive_chunk_fn recv_chunk,
				   esp_http_client_handle_t client)
{
	int ret;
	char buf[32];

	while ((ret = recv_chunk(client, buf, sizeof(buf))) > 0) {
		printf("%.*s", ret, buf);
	}
	if (ret < 0) {
		ESP_LOGE(TAG, "read data failed\n");
		return ESP_FAIL;
	}

	printf("\n");

	return ESP_OK;
}
