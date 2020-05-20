// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>
#include <errno.h>

#include <driver/gpio.h>

#include "gps.h"

static struct gps_ctx gps;

// TODO: Probably should parameterise.
struct gps_ctx *gps_init(void)
{
	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};
	uart_driver_install(UART_NUM_1, 256, 0, 0, NULL, 0);
	uart_param_config(UART_NUM_1, &uart_config);
	uart_set_pin(UART_NUM_1, GPIO_NUM_12, GPIO_NUM_34,
		     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	gps.uart = UART_NUM_1;

	return &gps;
}

void gps_send_message(struct gps_ctx *gps, struct ubx_message *msg)
{
	ubx_add_checksum(msg);
	uart_write_bytes(UART_NUM_1, (const char *)msg, msg->hdr.len + 6 + 2);
}

int gps_send_get_ack(struct gps_ctx *gps, struct ubx_message *msg, TickType_t timeout)
{
	struct ubx_message *resp = NULL;

	gps_send_message(gps, msg);

	TickType_t start = xTaskGetTickCount();
	while (((xTaskGetTickCount() - start) < timeout)) {
		int len = uart_read_bytes(UART_NUM_1, gps->buf, sizeof(gps->buf), 100 / portTICK_RATE_MS);
		if (!len) {
			continue;
		}

		resp = ubx_receive(gps->buf, len);
		while (resp) {
			if (resp->hdr.class == 0x5) {
				if (resp->payload_csum[0] == msg->hdr.class &&
				    resp->payload_csum[1] == msg->hdr.id) {
					int ret = resp->hdr.id == 1 ? 0 : -EINVAL;
					free(resp);
					return ret;
			       }
			}
			free(resp);

			resp = ubx_receive(NULL, 0);
		}
	}

	return -ETIMEDOUT;
}

struct ubx_message *gps_send_get_response(struct gps_ctx *gps, struct ubx_message *msg, TickType_t timeout)
{
	struct ubx_message *resp = NULL;

	gps_send_message(gps, msg);

	TickType_t start = xTaskGetTickCount();
	while (((xTaskGetTickCount() - start) < timeout)) {
		int len = uart_read_bytes(UART_NUM_1, gps->buf, sizeof(gps->buf), 100 / portTICK_RATE_MS);
		if (!len) {
			continue;
		}

		resp = ubx_receive(gps->buf, len);
		while (resp) {
			if (resp->hdr.class == msg->hdr.class &&
			    resp->hdr.id == msg->hdr.id) {
				return resp;
			}
			free(resp);

			resp = ubx_receive(NULL, 0);
		}
	}

	return NULL;
}

int gps_set_ubx_protocol(struct gps_ctx *gps)
{
	int ret, retries = 3;
	struct ubx_message *resp;

	// Request config for port 1 (UART)
	struct ubx_cfg_prt *msg = (struct ubx_cfg_prt *)ubx_alloc(UBX_MSG_CLASS_CFG, UBX_MSG_ID_CFG_PRT, 1);
	if (!msg) {
		return -ENOMEM;
	}

	msg->poll.port = 1;
	while (retries--) {
		resp = gps_send_get_response(gps, (struct ubx_message *)msg, 1000 / portTICK_RATE_MS);
		if (resp) {
			break;
		}
	}
	ubx_free((struct ubx_message *)msg);

	if (!resp) {
		return -ETIMEDOUT;
	}

	msg = (struct ubx_cfg_prt *)resp;
	// Defensive...
	resp = NULL;

	// Set UBX only
	msg->uart.outProtoMask = UBX_CFG_PRT_PROTOMASK_UBX;
	retries = 3;
	while (retries--) {
		ret = gps_send_get_ack(gps, (struct ubx_message *)msg, 1000 / portTICK_RATE_MS);
		if (!ret) {
			break;
		} else if (ret != -ETIMEDOUT) {
			// Unexpected error
			break;
		}
	}
	free((struct ubx_message *)msg);

	return ret;
}

int gps_set_message_rate(struct gps_ctx *gps, uint8_t class, uint8_t id, uint8_t rate)
{
	int ret, retries = 3;
	struct ubx_cfg_msg *msg = (struct ubx_cfg_msg *)ubx_alloc(UBX_MSG_CLASS_CFG, UBX_MSG_ID_CFG_MSG, 3);
	if (!msg) {
		return -ENOMEM;
	}

	msg->set_current.msgClass = class;
	msg->set_current.msgID = id;
	msg->set_current.rate = rate;
	while (retries--) {
		ret = gps_send_get_ack(gps, (struct ubx_message *)msg, 1000 / portTICK_RATE_MS);
		if (!ret) {
			break;
		} else if (ret != -ETIMEDOUT) {
			// Unexpected error
			break;
		}
	}
	ubx_free((struct ubx_message *)msg);

	return ret;
}
