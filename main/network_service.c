// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"

#include "network_service.h"
#include "wifi.h"

#define TAG "network_service"

#define NETWORK_CMD_WIFI_EVENT SERVICE_CMD_LOCAL(1)
#define NETWORK_CMD_IP_EVENT   SERVICE_CMD_LOCAL(2)

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

static void network_service_fn(void *param)
{
	struct service *service = (struct service *)param;
	struct wifi_ctx *ctx = NULL;

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
		case NETWORK_CMD_WIFI_EVENT:
			switch ((wifi_event_t)smsg.arg) {
			case WIFI_EVENT_STA_START:
				if (state != STARTING) {
					break;
				}
				retry = 0;
				/* Fallthrough */
			case WIFI_EVENT_STA_DISCONNECTED:
				if (retry++ > MAX_RETRIES) {
					wifi_stop(ctx);
					ctx = NULL;
					state = STOPPED;
					break;
				}
				esp_wifi_connect();
				state = CONNECTING;
				break;
			default:
				/* Unhandled WiFi event */
				break;
			}
			break;
		case NETWORK_CMD_IP_EVENT:
			switch ((ip_event_t)smsg.arg) {
			case IP_EVENT_STA_GOT_IP:
				state = CONNECTED;
				{
					esp_netif_t *netif = wifi_get_netif(ctx);
					esp_netif_ip_info_t ip;
					esp_err_t err = esp_netif_get_ip_info(netif, &ip);
					if (err != ESP_OK) {
						ESP_LOGE(TAG, "Error getting IP info");
					} else {
						ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&ip.ip));
					}
				}
				break;
			default:
				/* Unhandled IP event */
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
