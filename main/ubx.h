// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __UBX_H__
#define __UBX_H__

#include <stdint.h>

#define UBX_MSG_CLASS_ACK  0x05
#define UBX_MSG_ID_ACK_NAK 0x00
#define UBX_MSG_ID_ACK_ACK 0x01

#define UBX_MSG_CLASS_CFG  0x06
#define UBX_MSG_ID_CFG_PRT 0x00
#define UBX_MSG_ID_CFG_MSG 0x01

#define UBX_MSG_CLASS_NAV  0x01
#define UBX_MSG_ID_NAV_PVT 0x07


struct ubx_header {
	char sync[2];
	uint8_t class;
	uint8_t id;
	uint16_t len;
};

struct ubx_message {
	struct ubx_header hdr;
	// Variable length payload + checksum
	uint8_t payload_csum[/* hdr.len + 2 */];
};

struct ubx_ack_ack {
	struct ubx_header hdr;
	union {
		uint8_t payload_csum[2 + 2];
		struct {
			uint8_t clsID;
			uint8_t msgID;

			// ck_a, ck_b
		};
	};
};

struct ubx_nav_pvt {
	struct ubx_header hdr;
	union {
		uint8_t payload_csum[92 + 2];
		struct __attribute__((packed)) {
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

struct ubx_cfg_prt {
	struct ubx_header hdr;
	union {
		uint8_t payload_csum[20 + 2];
		struct __attribute__((packed)) {
			uint8_t port;

			// ck_a, ck_b
		} poll;
		struct __attribute__((packed)) {
			uint8_t portID;
			uint8_t reserved1;
#define UBX_CFG_PRT_TXREADY_EN               (1 << 0)
#define UBX_CFG_PRT_TXREADY_POL              (1 << 1)
#define UBX_CFG_PRT_TXREADY_POL_HIGHACTIVE   (0 << 1)
#define UBX_CFG_PRT_TXREADY_POL_LOWACTIVE    (1 << 1)
#define UBX_CFG_PRT_TXREADY_GET_PIN(x)       (((x) >> 2) & 0x1f)
#define UBX_CFG_PRT_TXREADY_SET_PIN(x)       (((x) & 0x1f) << 2)
#define UBX_CFG_PRT_TXREADY_GET_THRESH(x)    (((x) >> 6) & 0x1ff)
#define UBX_CFG_PRT_TXREADY_SET_THRESH(x)    (((x) & 0x1ff) << 6)
			uint16_t txReady;
#define UBX_CFG_PRT_MODE_GET_CHARLEN(x)      (((x) >> 6) & 0x3)
#define UBX_CFG_PRT_MODE_SET_CHARLEN(x)      (((x) & 0x3) & << 6)
#define UBX_CFG_PRT_MODE_CHARLEN_5BIT        0
#define UBX_CFG_PRT_MODE_CHARLEN_6BIT        1
#define UBX_CFG_PRT_MODE_CHARLEN_7BIT        2
#define UBX_CFG_PRT_MODE_CHARLEN_8BIT        3
#define UBX_CFG_PRT_MODE_GET_PARITY(x)       (((x) >> 9) & 0x7)
#define UBX_CFG_PRT_MODE_SET_PARITY(x)       (((x) & 0x7) & << 9)
#define UBX_CFG_PRT_MODE_PARITY_EVEN         0
#define UBX_CFG_PRT_MODE_PARITY_ODD          1
#define UBX_CFG_PRT_MODE_PARITY_NONE         4 // Bit 0 is technically dont-care
#define UBX_CFG_PRT_MODE_GET_STOPBITS(x)     (((x) >> 12) & 0x3)
#define UBX_CFG_PRT_MODE_SET_STOPBITS(x)     (((x) & 0x3) & << 12)
#define UBX_CFG_PRT_MODE_STOPBITS_1BIT       0
#define UBX_CFG_PRT_MODE_STOPBITS_1_5BIT     1
#define UBX_CFG_PRT_MODE_STOPBITS_2BIT       2
#define UBX_CFG_PRT_MODE_STOPBITS_0_5BIT     3
			uint32_t mode;
			uint32_t baudRate;
#define UBX_CFG_PRT_PROTOMASK_UBX           (1 << 0)
#define UBX_CFG_PRT_PROTOMASK_NMEA          (1 << 1)
#define UBX_CFG_PRT_PROTOMASK_RTCM          (1 << 2)
#define UBX_CFG_PRT_PROTOMASK_RTCM3         (1 << 5)
			uint16_t inProtoMask;
			uint16_t outProtoMask;
#define UBX_CFG_PRT_FLAGS_EXTENDEDTXTIMEOUT (1 << 1)
			uint16_t flags;
			uint8_t  reserved2[2];

			// ck_a, ck_b
		} uart;
	};
};

struct ubx_cfg_msg {
	struct ubx_header hdr;
	union {
		uint8_t payload_csum[8 + 2];
		struct __attribute__((packed)) {
			uint8_t msgClass;
			uint8_t msgID;

			// ck_a, ck_b
		} poll;
		struct __attribute__((packed)) {
			uint8_t msgClass;
			uint8_t msgID;
			uint8_t rates[6]; // Rate per-port

			// ck_a, ck_b
		} set_all;
		struct __attribute__((packed)) {
			uint8_t msgClass;
			uint8_t msgID;
			uint8_t rate;

			// ck_a, ck_b
		} set_current;
	};
};

struct ubx_message *ubx_alloc(uint8_t class, uint8_t id, uint16_t len);
void ubx_free(struct ubx_message *msg);

void ubx_add_checksum(struct ubx_message *msg);

struct ubx_message *ubx_receive(uint8_t *data, size_t datalen);

void ubx_print_msg(struct ubx_message *msg);
void ubx_print_nav_pvt(struct ubx_nav_pvt *pvt);

int32_t ubx_deg_to_semicircles(int32_t deg);

#endif /* __UBX_H__ */
