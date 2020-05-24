// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __SERVICE_MANAGER_H__
#define __SERVICE_MANAGER_H__

#include <stdint.h>

struct __node {
	struct __node *next;
};

struct service {
	const char *name;
	void (*fn)(void *pvParameter);
	UBaseType_t priority;

	// Populated by service_register
	QueueHandle_t cmdq;

	// Internal
	struct __node node;
};

struct service_message {
	uint32_t cmd;
};

int service_register(struct service *service);
struct service *service_lookup(const char *name);

#endif /* __SERVICE_MANAGER_H__ */
