// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __GPS_H__
#define __GPS_H__

#include <stdint.h>
#include <driver/uart.h>

struct gps_ctx {
	uart_port_t uart;
};

struct ubx_header {
	char sync[2];
	uint8_t class;
	uint8_t id;
	uint16_t len;
};

struct ubx_message {
	struct ubx_header hdr;
	// Variable length payload + checksum
	uint8_t payload_csum[/* length + 2 */];
};

struct ubx_nav_pvt {
	struct ubx_header msg;
	union {
		uint8_t payload_csum[92 + 2];
		struct __attribute__((packed)){
			uint32_t iTOW;
			uint16_t year;
			uint8_t month;
			uint8_t day;
			uint8_t hour;
			uint8_t min;
			uint8_t sec;
			uint8_t valid;
			uint32_t tAcc;
			int32_t nano;
			uint8_t fixType;
			uint8_t flags;
			uint8_t flags2;
			uint8_t numSV;
			int32_t lon;
			int32_t lat;
			int32_t height;
			int32_t hMSL;
			uint32_t hAcc;
			uint32_t vAcc;
			int32_t velN;
			int32_t velE;
			int32_t velD;
			int32_t gSpeed;
			int32_t headMot;
			uint32_t sAcc;
			uint32_t headAcc;
			uint16_t pDOP;
			uint8_t flags3;
			uint8_t reserved1[5];
			int32_t headVeh;
			uint16_t magDec;
			uint16_t magAcc;

			// ck_a, ck_b
		};
	};
};

struct gps_ctx *gps_init(void);

void print_ubx(struct ubx_message *msg);
void print_ubx_nav_pvt(struct ubx_nav_pvt *pvt);
struct ubx_message *receive_ubx(uint8_t **data, size_t datalen);
struct ubx_message *alloc_msg(uint8_t class, uint8_t id, uint16_t len);

void send_message(struct gps_ctx *gps, struct ubx_message *msg);

#endif /* __GPS_H__ */
