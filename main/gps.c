// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>
#include <errno.h>

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "gps.h"

struct gps_ctx {
	uart_port_t uart;
	size_t rxlen;
	uint8_t buf[128];
};

struct gps_ctx *gps_init(uart_port_t uart_num, int tx_io_num, int rx_io_num, QueueHandle_t *eventq)
{
	struct gps_ctx *gps = calloc(1, sizeof(*gps));
	if (!gps) {
		return NULL;
	}

	uart_config_t uart_config = {
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity    = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
		.source_clk = UART_SCLK_APB,
	};

	gps->uart = uart_num;

	uart_driver_install(gps->uart, 256, 256, 20, eventq, 0);
	uart_param_config(gps->uart, &uart_config);
	uart_set_pin(gps->uart, tx_io_num, rx_io_num,
		     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	return gps;
}

void gps_send_message(struct gps_ctx *gps, struct ubx_message *msg)
{
	ubx_add_checksum(msg);
	uart_write_bytes(gps->uart, (const char *)msg, msg->hdr.len + 6 + 2);
}

int gps_send_get_ack(struct gps_ctx *gps, struct ubx_message *msg, TickType_t timeout)
{
	struct ubx_message *resp = NULL;

	TimeOut_t timeout_ctx;
	vTaskSetTimeOutState(&timeout_ctx);

	gps_send_message(gps, msg);

	do {
		resp = gps_receive(gps, timeout);
		if (resp) {
			if (resp->hdr.class == UBX_MSG_CLASS_ACK) {
				struct ubx_ack_ack *ack = (struct ubx_ack_ack *)resp;
				if (ack->clsID == msg->hdr.class &&
				    ack->msgID == msg->hdr.id) {
					int ret = resp->hdr.id == UBX_MSG_ID_ACK_ACK ? 0 : -EINVAL;
					free(resp);
					return ret;
			       }
			}
			free(resp);
		}
	} while (xTaskCheckForTimeOut(&timeout_ctx, &timeout) == pdFALSE);

	return -ETIMEDOUT;
}

struct ubx_message *gps_send_get_response(struct gps_ctx *gps, struct ubx_message *msg, TickType_t timeout)
{
	struct ubx_message *resp = NULL;

	TimeOut_t timeout_ctx;
	vTaskSetTimeOutState(&timeout_ctx);

	gps_send_message(gps, msg);

	do {
		resp = gps_receive(gps, 100 / portTICK_RATE_MS);
		if (resp) {
			if (resp->hdr.class == msg->hdr.class &&
			    resp->hdr.id == msg->hdr.id) {
				return resp;
			}
			free(resp);
		}
	} while (xTaskCheckForTimeOut(&timeout_ctx, &timeout) == pdFALSE);

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

struct ubx_message *gps_receive(struct gps_ctx *gps, TickType_t timeout)
{
	struct ubx_message *msg = NULL;
	TimeOut_t timeout_ctx;
	vTaskSetTimeOutState(&timeout_ctx);

	// There's still some data to process
	if (gps->rxlen) {
		msg = ubx_receive(NULL, gps->rxlen);
	}

	// Read new data until we have a message or timeout
	while (!msg) {
		gps->rxlen = uart_read_bytes(gps->uart, gps->buf, sizeof(gps->buf), timeout);

		msg = ubx_receive(gps->buf, gps->rxlen);

		if (xTaskCheckForTimeOut(&timeout_ctx, &timeout)) {
			break;
		}
	}

	// If there's still no message, then we need to start over
	if (!msg) {
		gps->rxlen = 0;
	}

	return msg;
}
