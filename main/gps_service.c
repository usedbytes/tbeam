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
#define GPS_CMD_SUBSCRIBE_PVT           GPS_CMD(3)
#define GPS_CMD_UNSUBSCRIBE_PVT         GPS_CMD(4)

static void gps_service_fn(void *param);

static struct pvt_message *pvt_alloc(struct ubx_nav_pvt *ubx);
static void pvt_get(struct pvt_message *pvt);

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
#define SUB_MASK_LOCK_STATUS (1 << 0)
#define SUB_MASK_PVT         (1 << 1)
	uint32_t mask;
};

static bool match_subscriber(struct list_node *node, void *data)
{
	struct gps_subscriber *entry = (struct gps_subscriber *)node;

	return entry->service == (struct service *)data;
}

static void notify_subscribers(struct list_node *subs, uint32_t mask, const struct service_message *smsg)
{
	struct gps_subscriber *sub;
	list_for_each(subs, struct gps_subscriber, node, sub) {
		if (sub->mask & mask) {
			service_send_message(sub->service, smsg, 0);
		}
	}
}

static void notify_subscribers_lock_status(struct list_node *subs, uint32_t status)
{
	struct service_message smsg = {
		.cmd = GPS_CMD_LOCK_STATUS,
		.arg = status,
	};

	notify_subscribers(subs, SUB_MASK_LOCK_STATUS, &smsg);
}

static void notify_subscribers_pvt(struct list_node *subs, struct pvt_message *pvt)
{
	struct service_message smsg = {
		.cmd = GPS_CMD_PVT,
		.argp = pvt,
	};

	struct gps_subscriber *sub;
	list_for_each(subs, struct gps_subscriber, node, sub) {
		if (sub->mask & SUB_MASK_PVT) {
			pvt_get(pvt);
			int ret = service_send_message(sub->service, &smsg, 0);
			if (ret < 0) {
				// Send failed.
				pvt_put(pvt);
			}
		}
	}
}

static void add_subscription(struct list_node *subs, struct service *service, uint32_t mask)
{
	struct gps_subscriber *sub = NULL;
	struct list_node *node = list_find(subs, match_subscriber, service);
	if (!node) {
		sub = calloc(1, sizeof(*sub));
		if (!sub) {
			ESP_LOGE(TAG, "out of memory allocating subscriber");
			return;
		}
		sub->service = service;
		list_add_tail(subs, &sub->node);
	} else {
		sub = container_of(node, struct gps_subscriber, node);
	}
	sub->mask |= mask;
}

static void remove_subscription(struct list_node *subs, void *service, uint32_t mask)
{
	struct list_node *node = list_find(subs, match_subscriber, service);
	if (!node) {
		return;
	}

	struct gps_subscriber *sub = container_of(node, struct gps_subscriber, node);
	sub->mask &= ~mask;
	if (!sub->mask) {
		list_del(subs, node);
		free(sub);
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

	struct list_node subs_list = {
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
					notify_subscribers_lock_status(&subs_list, 1);
				} else if ((state == TRACKING) && !(pvt->flags & 1)) {
					state = ACQUISITION;
					notify_subscribers_lock_status(&subs_list, 0);
				}

				struct pvt_message *pvt_msg = pvt_alloc(pvt);
				if (pvt) {
					notify_subscribers_pvt(&subs_list, pvt_msg);
				}
				pvt_put(pvt_msg);
			}
			break;
		}
		case GPS_CMD_SUBSCRIBE_LOCK_STATUS:
			add_subscription(&subs_list, (struct service *)smsg.arg, SUB_MASK_LOCK_STATUS);
			break;
		case GPS_CMD_UNSUBSCRIBE_LOCK_STATUS:
			remove_subscription(&subs_list, (struct service *)smsg.arg, SUB_MASK_LOCK_STATUS);
			break;
		case GPS_CMD_SUBSCRIBE_PVT:
			add_subscription(&subs_list, (struct service *)smsg.arg, SUB_MASK_PVT);
			break;
		case GPS_CMD_UNSUBSCRIBE_PVT:
			remove_subscription(&subs_list, (struct service *)smsg.arg, SUB_MASK_PVT);
			break;
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

int gps_subscribe_pvt(struct service *service, struct service *subscriber)
{
	struct service_message smsg = {
		.cmd = GPS_CMD_SUBSCRIBE_PVT,
		.arg = (uint32_t)subscriber,
	};

	return service_send_message(service, &smsg, 0);
}

int gps_unsubscribe_pvt(struct service *service, struct service *subscriber)
{
	struct service_message smsg = {
		.cmd = GPS_CMD_UNSUBSCRIBE_PVT,
		.arg = (uint32_t)subscriber,
	};

	return service_send_message(service, &smsg, 0);
}

static struct pvt_message *pvt_alloc(struct ubx_nav_pvt *ubx)
{
	struct pvt_message *pvt = calloc(1, sizeof(*pvt));
	if (!pvt) {
		return NULL;
	}
	vPortCPUInitializeMutex(&pvt->lock);
	pvt->ref = 1;
	pvt->body = ubx;

	ESP_LOGD(TAG, "pvt_alloc %p -> %d\n", pvt, pvt->ref);

	return pvt;
}

void pvt_put(struct pvt_message *pvt)
{
	uint32_t ref;
	portENTER_CRITICAL(&pvt->lock);
	ref = --pvt->ref;
	portEXIT_CRITICAL(&pvt->lock);

	ESP_LOGD(TAG, "pvt_put %p -> %d\n", pvt, ref);

	// We were holding the last reference
	if (ref == 0) {
		ESP_LOGD(TAG, "pvt_put %p -> free()\n", pvt);
		ubx_free((struct ubx_message *)pvt->body);
		free(pvt);
	}
}

static void pvt_get(struct pvt_message *pvt)
{
	uint32_t ref;
	portENTER_CRITICAL(&pvt->lock);
	ref = ++pvt->ref;
	portEXIT_CRITICAL(&pvt->lock);

	ESP_LOGD(TAG, "pvt_get %p -> %d\n", pvt, ref);

	// Make sure we didn't overflow
	assert(ref > 0);
}
