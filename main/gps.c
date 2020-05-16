// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <errno.h>
#include <string.h>

#include <driver/gpio.h>

#include "gps.h"

static void rfc1145_checksum(uint8_t *data, uint16_t len, uint8_t *ck_a_out, uint8_t *ck_b_out)
{
	uint8_t ck_a = 0, ck_b = 0;
	uint16_t i;

	for (i = 0; i < len; i++) {
		ck_a += data[i];
		ck_b += ck_a;
	}

	*ck_a_out = ck_a;
	*ck_b_out = ck_b;

	return;
}

struct ubx_message *alloc_msg(uint8_t class, uint8_t id, uint16_t len)
{
	struct ubx_message *msg = (struct ubx_message *)malloc(sizeof(struct ubx_header) + len + 2);
	if (msg == NULL) {
		return NULL;
	}

	msg->hdr.sync[0] = 0xb5;
	msg->hdr.sync[1] = 0x62;
	msg->hdr.class = class;
	msg->hdr.id = id;
	msg->hdr.len = len;

	return msg;
}

/*
 * Attempts to process UBX messages from the passed buffer.
 *
 * Initially, a buffer containing UBX traffic must be passed as 'data',
 * and the length of the buffer in 'datalen'. Subesquent calls processing the
 * same buffer should set data = NULL. Upon returning NULL, the caller should
 * consider that buffer fully processed, and provide the next data.
 *
 * An internal context is maintained, allowing messages to be split across
 * multiple 'data' buffers.
 *
 * Example usage:
 *
 * while (1) {
 *     uint8_t data[256];
 *     int len = read_from_uart(data, 256);
 *
 *     struct ubx_message *msg = receive_ubx(data, len);
 *     while (msg) {
 *         process_msg(msg);
 *         free(msg);
 *
 *         // Continue processing the same buffer
 *         msg = receive_ubx(NULL, 0);
 *     }
 * }
 *
 * Returns:
 *  - A ubx_message, if a complete valid message has been found
 *  - NULL in the case of reaching the end of the buffer or error
 */
struct ubx_message *receive_ubx(uint8_t *data, size_t datalen)
{
	static enum {
		STATE_SYNC_1 = 0,
		STATE_SYNC_2,
		STATE_CLASS,
		STATE_ID,
		STATE_LEN_1,
		STATE_LEN_2,
		STATE_PAYLOAD,
		STATE_CK_A,
		STATE_CK_B,
	} state = STATE_SYNC_1;

	static struct {
		struct ubx_header hdr;
		uint16_t idx;
		uint8_t ck_a, ck_b;
		struct ubx_message *msg;

		uint8_t *cur;
		size_t datalen;
	} ctx;

	struct ubx_message *ret = NULL;

	if (data != NULL) {
		ctx.cur = data;
		ctx.datalen = datalen;
	}

	while (ctx.datalen) {
		switch (state) {
		case STATE_SYNC_1:
			if (*ctx.cur == 0xb5) {
				state = STATE_SYNC_2;
			}
			ctx.cur++;
			ctx.datalen--;
			break;
		case STATE_SYNC_2:
			if (*ctx.cur == 0x62) {
				state = STATE_CLASS;
			}
			ctx.cur++;
			ctx.datalen--;
			break;
		case STATE_CLASS:
			ctx.hdr.class = *ctx.cur;
			state = STATE_ID;
			ctx.cur++;
			ctx.datalen--;
			break;
		case STATE_ID:
			ctx.hdr.id = *ctx.cur;
			state = STATE_LEN_1;
			ctx.cur++;
			ctx.datalen--;
			break;
		case STATE_LEN_1:
			ctx.hdr.len = *ctx.cur;
			state = STATE_LEN_2;
			ctx.cur++;
			ctx.datalen--;
			break;
		case STATE_LEN_2:
			ctx.hdr.len |= (*ctx.cur << 8);
			state = STATE_PAYLOAD;
			ctx.msg = alloc_msg(ctx.hdr.class, ctx.hdr.id, ctx.hdr.len);
			if (ctx.msg == NULL) {
				state = STATE_SYNC_1;

				// How to propagate error?
				return NULL;
			}
			ctx.idx = 0;
			ctx.cur++;
			ctx.datalen--;
			break;
		case STATE_PAYLOAD:
		{
			int len = ctx.datalen < ctx.hdr.len ? ctx.datalen : ctx.hdr.len;
			memcpy(&ctx.msg->payload_csum[ctx.idx], ctx.cur, len);
			ctx.hdr.len -= len;
			if (ctx.hdr.len == 0) {
				rfc1145_checksum(&ctx.msg->hdr.class, ctx.msg->hdr.len + 4, &ctx.ck_a, &ctx.ck_b);
				state = STATE_CK_A;
			}
			ctx.cur += len;
			ctx.datalen -= len;
			break;
		}
		case STATE_CK_A:
			if (*ctx.cur != ctx.ck_a) {
				free(ctx.msg);
				ctx.msg = NULL;
				state = STATE_SYNC_1;

				// How to propagate error?
				return NULL;
			}
			state = STATE_CK_B;
			ctx.cur++;
			ctx.datalen--;
			break;
		case STATE_CK_B:
			if (*ctx.cur != ctx.ck_b) {
				free(ctx.msg);
				ctx.msg = NULL;

				// How to propagate error?
				return NULL;
			}
			state = STATE_SYNC_1;
			ctx.cur++;

			// Break out of the outer while()
			ctx.datalen = 0;

			ret = ctx.msg;
			ctx.msg = NULL;

			break;
		}
	}

	return ret;
}

void print_ubx(struct ubx_message *msg)
{
	uint16_t i;
	printf("Class: 0x%02x\n", msg->hdr.class);
	printf("ID:    0x%02x\n", msg->hdr.id);
	printf("Len:   %d\n", msg->hdr.len);
	printf("Ck:    0x%02x 0x%02x\n", msg->payload_csum[msg->hdr.len], msg->payload_csum[msg->hdr.len + 1]);
	for (i = 0; i < msg->hdr.len; i++) {
		if (!(i % 16)) {
			if (i > 0) {
				printf("\n");
			}
			printf("%04x  ", i);
		}
		printf("%02x ", msg->payload_csum[i]);
	}
	printf("\n");
}

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

static void calc_message_checksum(struct ubx_message *msg)
{
	rfc1145_checksum(&msg->hdr.class, msg->hdr.len + 4, &msg->payload_csum[msg->hdr.len], &msg->payload_csum[msg->hdr.len + 1]);
}

void send_message(struct gps_ctx *gps, struct ubx_message *msg)
{
	calc_message_checksum(msg);
	uart_write_bytes(UART_NUM_1, (const char *)msg, msg->hdr.len + 6 + 2);
}

int32_t ubx_deg_to_semicircles(int32_t deg)
{
#define SEMI_s31_32_FACTOR ((int64_t)(((1ULL << 63) * 1e-7) / 180))
	// Messy casting to ensure right-shift is defined.
	return ((uint64_t)((int64_t)deg * SEMI_s31_32_FACTOR)) >> 32;
}

void print_ubx_nav_pvt(struct ubx_nav_pvt *pvt)
{
	printf("%04d-%02d-%02d %02d:%02d:%02d\n", pvt->year, pvt->month, pvt->day, pvt->hour, pvt->min, pvt->sec);
	printf("Lon: %f Lat: %f\n", (float)pvt->lon * 1e-7, (float)pvt->lat * 1e-7);
	printf("hAcc: %d vAcc: %d gSpeed: %d\n", pvt->hAcc, pvt->vAcc, pvt->gSpeed);
	printf("fixType: %d fixOK: %d invalid: %d\n", pvt->fixType, pvt->flags & 1, pvt->flags3 & 1);
}

int ubx_send_get_ack(struct gps_ctx *gps, struct ubx_message *msg, TickType_t timeout)
{
	struct ubx_message *resp = NULL;

	send_message(gps, msg);

	TickType_t start = xTaskGetTickCount();
	while (((xTaskGetTickCount() - start) < timeout)) {
		int len = uart_read_bytes(UART_NUM_1, gps->buf, sizeof(gps->buf), 100 / portTICK_RATE_MS);
		if (!len) {
			continue;
		}

		resp = receive_ubx(gps->buf, len);
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

			resp = receive_ubx(NULL, 0);
		}
	}

	return -ETIMEDOUT;
}

struct ubx_message *ubx_send_get_response(struct gps_ctx *gps, struct ubx_message *msg, TickType_t timeout)
{
	struct ubx_message *resp = NULL;

	send_message(gps, msg);

	TickType_t start = xTaskGetTickCount();
	while (((xTaskGetTickCount() - start) < timeout)) {
		int len = uart_read_bytes(UART_NUM_1, gps->buf, sizeof(gps->buf), 100 / portTICK_RATE_MS);
		if (!len) {
			continue;
		}

		resp = receive_ubx(gps->buf, len);
		while (resp) {
			if (resp->hdr.class == msg->hdr.class &&
			    resp->hdr.id == msg->hdr.id) {
				return resp;
			}
			free(resp);

			resp = receive_ubx(NULL, 0);
		}
	}

	return NULL;
}
