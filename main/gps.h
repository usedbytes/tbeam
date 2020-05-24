// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __GPS_H__
#define __GPS_H__

#include <stdint.h>
#include <driver/uart.h>

#include "ubx.h"

struct gps_ctx;

struct gps_ctx *gps_init(uart_port_t uart_num, int tx_io_num, int rx_io_num, QueueHandle_t *eventq);

struct ubx_message *gps_send_get_response(struct gps_ctx *gps, struct ubx_message *msg, TickType_t timeout);
int gps_send_get_ack(struct gps_ctx *gps, struct ubx_message *msg, TickType_t timeout);
void gps_send_message(struct gps_ctx *gps, struct ubx_message *msg);
struct ubx_message *gps_receive(struct gps_ctx *gps, TickType_t timeout);

int gps_set_ubx_protocol(struct gps_ctx *gps);
int gps_set_message_rate(struct gps_ctx *gps, uint8_t class, uint8_t id, uint8_t rate);

#endif /* __GPS_H__ */
