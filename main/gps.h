// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __GPS_H__
#define __GPS_H__

#include <stdint.h>
#include <driver/uart.h>

struct gps_ctx {
	uart_port_t uart;
};

struct ubx_message {
	char sync[2];
	uint8_t class;
	uint8_t id;
	uint16_t len;
	// Variable length payload + checksum
	uint8_t payload_csum[/* length + 2 */];
};

struct gps_ctx *gps_init(void);

void print_ubx(struct ubx_message *msg);
struct ubx_message *receive_ubx(uint8_t **data, size_t datalen);
struct ubx_message *alloc_msg(uint8_t class, uint8_t id, uint16_t len);

void send_message(struct gps_ctx *gps, struct ubx_message *msg);

#endif /* __GPS_H__ */
