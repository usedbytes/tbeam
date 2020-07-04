// Based on WiFi station Example, which is Public Domain
//
// Portions copyright (c) 2020 Brian Starkey <stark3y@gmail.com>
// SPDX-License-Identifier: MIT

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "wifi.h"

#define WIFI_SSID      CONFIG_WIFI_SSID
#define WIFI_PASS      CONFIG_WIFI_PASSWORD

static const char *TAG = "wifi";

struct wifi_ctx {
	esp_netif_t *netif;
	esp_event_handler_instance_t wifi_evt;
	esp_event_handler_instance_t ip_evt;
};

struct wifi_ctx *wifi_init(esp_event_handler_t handler, void *handler_arg)
{
	esp_err_t err;
	struct wifi_ctx *ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		return NULL;
	}

	err = esp_netif_init();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_netif_init() failed");
		goto error;
	}

	err = esp_event_loop_create_default();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_event_loop_create_default() failed");
		goto error;
	}

	ctx->netif = esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	err = esp_wifi_init(&cfg);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_wifi_init() failed: %d %s", err, esp_err_to_name(err));
		goto error;
	}

	err = esp_event_handler_instance_register(WIFI_EVENT,
				ESP_EVENT_ANY_ID,
				handler,
				handler_arg,
				&ctx->wifi_evt);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_event_handler_instance_register(wifi_evt) failed");
		goto error;
	}

	err = esp_event_handler_instance_register(IP_EVENT,
				IP_EVENT_STA_GOT_IP,
				handler,
				handler_arg,
				&ctx->ip_evt);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_event_handler_instance_register(ip_evt) failed");
		goto error;
	}

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = WIFI_SSID,
			.password = WIFI_PASS,
		},
	};

	err = esp_wifi_set_mode(WIFI_MODE_STA);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_wifi_set_mode() failed");
		goto error;
	}

	err = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_wifi_set_config() failed");
		goto error;
	}

	err = esp_wifi_start();
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "esp_wifi_start() failed: %d %s", err, esp_err_to_name(err));
		goto error;
	}

	return ctx;

error:
	wifi_stop(ctx);
	return NULL;
}

// XXX: Convenience for prototyping. Should provide higher level APIs later.
esp_netif_t *wifi_get_netif(struct wifi_ctx *ctx)
{
	return ctx->netif;
}

void wifi_stop(struct wifi_ctx *ctx)
{
	esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ctx->wifi_evt);
	esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, ctx->ip_evt);

	esp_wifi_stop();
	esp_wifi_deinit();

	free(ctx);
}
