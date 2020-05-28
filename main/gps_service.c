// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"

#include "esp_log.h"

#include "axp192.h"
#include "gps.h"
#include "list.h"
#include "ubx.h"
#include "gps_service.h"
#include "pmic_service.h"

#define TAG "gps_service"

#define GPS_UART_TXD (GPIO_NUM_12)
#define GPS_UART_RXD (GPIO_NUM_34)

#define GPS_CMD_SCOPE SERVICE_SCOPE('g', 'p')
#define GPS_CMD(_x)   SERVICE_CMD(GPS_CMD_SCOPE, _x)

#define GPS_CMD_SUBSCRIBE_LOCK_STATUS   GPS_CMD(1)
#define GPS_CMD_UNSUBSCRIBE_LOCK_STATUS GPS_CMD(2)

static void gps_service_fn(void *param);

struct service *gps_service_register()
{
	return service_register("gps", gps_service_fn, 1, 4096);
}

#define GPS_SERVICE_UART_READ_CMD SERVICE_CMD_LOCAL(1)

struct uart_ctx {
	struct service *service;
	QueueHandle_t uart_queue;
	EventGroupHandle_t flags;
};

static void uart_event_handler(void *param)
{
	struct uart_ctx *ctx = (struct uart_ctx *)param;
	static const struct service_message smsg = {
		.cmd = GPS_SERVICE_UART_READ_CMD,
	};

	while (1) {
		uart_event_t event;
		if (!xQueueReceive(ctx->uart_queue, (void * )&event, portMAX_DELAY)) {
			continue;
		}

		if (event.type != UART_DATA) {
			continue;
		}

		if (!xEventGroupGetBits(ctx->flags)) {
			continue;
		}

		service_send_message(ctx->service, &smsg, portMAX_DELAY);
	}
}

struct gps_subscriber {
	struct list_node node;
	struct service *service;
};

static bool match_subscriber(struct list_node *node, void *data)
{
	struct gps_subscriber *entry = (struct gps_subscriber *)node;

	return entry->service == (struct service *)data;
}

static void notify_subscribers_lock_status(struct list_node *subs, uint32_t status)
{
	struct gps_subscriber *sub;
	struct service_message smsg = {
		.cmd = GPS_CMD_LOCK_STATUS,
		.arg = status,
	};

	list_for_each(subs, struct gps_subscriber, node, sub) {
		service_send_message(sub->service, &smsg, 0);
	}
}

static void gps_service_fn(void *param)
{
	struct service *service = (struct service *)param;
	struct service *pmic_service = service_lookup("pmic");
	int ret;

	assert(pmic_service);

	struct uart_ctx *ctx = calloc(1, sizeof(*ctx));
	ctx->service = service;
	ctx->flags = xEventGroupCreate();

	struct gps_ctx *gps = gps_init(UART_NUM_1, GPS_UART_TXD, GPS_UART_RXD, &ctx->uart_queue);

	struct list_node lock_subs = {
		.next = NULL,
	};

	// We monitor for UART activity using a high priority task which should
	// get woken directly from the UART IRQ handler. It's not ideal, but
	// the overhead shouldn't be too awful, hopefully.
	// I did try using a QueueSet to wait on both the service and uart
	// queues directly, but it was a bit messy and fragile.
	xTaskCreatePinnedToCore(uart_event_handler, "gps_uart_event_handler", 4096, ctx, configMAX_PRIORITIES - 1, NULL, 1);

	enum state {
		POWERED_OFF = 0,
		CONFIGURED = 1,
		ACQUISITION = 2,
		TRACKING = 3,
	} state = POWERED_OFF;

	while (1) {
		struct service_message smsg;
		if (service_receive_message(service, &smsg, portMAX_DELAY)) {
			continue;
		}

		switch (smsg.cmd) {
		case SERVICE_CMD_START:
			pmic_request_rail(pmic_service, AXP192_RAIL_LDO3, 3300);
			service_sync(pmic_service);

			ret = gps_set_ubx_protocol(gps);
			if (ret) {
				ESP_LOGE(TAG, "Failed to set UBX protocol: %d\n", ret);
				pmic_release_rail(pmic_service, AXP192_RAIL_LDO3);
				break;
			}

			ret = gps_set_message_rate(gps, UBX_MSG_CLASS_NAV, UBX_MSG_ID_NAV_PVT, 1);
			if (ret) {
				ESP_LOGE(TAG, "Failed to set NAV_PVT rate: %d\n", ret);
				pmic_release_rail(pmic_service, AXP192_RAIL_LDO3);
				break;
			}

			xEventGroupSetBits(ctx->flags, 1);
			state = ACQUISITION;
			break;
		case SERVICE_CMD_STOP:
			xEventGroupClearBits(ctx->flags, 1);
			pmic_release_rail(pmic_service, AXP192_RAIL_LDO3);
			state = POWERED_OFF;
			break;
		case SERVICE_CMD_PAUSE:
			// Send sleep command
			break;
		case SERVICE_CMD_RESUME:
			// Send wake command
			break;
		case GPS_SERVICE_UART_READ_CMD:
		{
			struct ubx_message *msg = gps_receive(gps, 0);
			if (msg != NULL &&
			    msg->hdr.class == UBX_MSG_CLASS_NAV &&
			    msg->hdr.id == UBX_MSG_ID_NAV_PVT) {
				struct ubx_nav_pvt *pvt = (struct ubx_nav_pvt *)msg;

				if ((state != TRACKING) && (pvt->flags & 1)) {
					state = TRACKING;
					notify_subscribers_lock_status(&lock_subs, 1);
				} else if ((state == TRACKING) && !(pvt->flags & 1)) {
					state = ACQUISITION;
					notify_subscribers_lock_status(&lock_subs, 0);
				}

				ubx_print_nav_pvt(pvt);
				ubx_free(msg);
			}
			break;
		}
		case GPS_CMD_SUBSCRIBE_LOCK_STATUS:
		{
			struct gps_subscriber *sub = calloc(1, sizeof(*sub));
			if (!sub) {
				break;
			}
			sub->service = (struct service *)smsg.arg;
			list_add_tail(&lock_subs, &sub->node);
			break;
		}
		case GPS_CMD_UNSUBSCRIBE_LOCK_STATUS:
		{
			struct list_node *node = list_find(&lock_subs, match_subscriber, (void *)smsg.arg);
			if (!node) {
				break;
			}
			list_del(&lock_subs, node);

			struct gps_subscriber *sub = container_of(node, struct gps_subscriber, node);
			free(sub);
			break;
		}
		default:
			// Unknown command
			break;
		}

		// Acknowledge the command
		service_ack(service);
	}
}

int gps_subscribe_lock_status(struct service *service, struct service *subscriber)
{
	struct service_message smsg = {
		.cmd = GPS_CMD_SUBSCRIBE_LOCK_STATUS,
		.arg = (uint32_t)subscriber,
	};

	return service_send_message(service, &smsg, 0);
}

int gps_unsubscribe_lock_status(struct service *service, struct service *subscriber)
{
	struct service_message smsg = {
		.cmd = GPS_CMD_UNSUBSCRIBE_LOCK_STATUS,
		.arg = (uint32_t)subscriber,
	};

	return service_send_message(service, &smsg, 0);
}
