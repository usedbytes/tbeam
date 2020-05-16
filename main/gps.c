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
