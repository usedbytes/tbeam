// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __WIFI_H__
#define __WIFI_H__

#include "esp_event.h"
#include "esp_netif.h"

struct wifi_ctx;

struct wifi_ctx *wifi_init(esp_event_handler_t handler, void *handler_arg);
void wifi_stop(struct wifi_ctx *ctx);

#endif /* __WIFI_H__ */
