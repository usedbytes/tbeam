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

#define SERVICE_CMD(_scope, _cmd) (((_scope) << 16) | (_cmd))
#define SERVICE_CMD_GLOBAL(_cmd) SERVICE_CMD(0, _cmd)
#define SERVICE_CMD_LOCAL(_cmd) SERVICE_CMD(1, _cmd)

#define SERVICE_CMD_STOP   SERVICE_CMD_GLOBAL(0)
#define SERVICE_CMD_START  SERVICE_CMD_GLOBAL(1)
#define SERVICE_CMD_PAUSE  SERVICE_CMD_GLOBAL(2)
#define SERVICE_CMD_RESUME SERVICE_CMD_GLOBAL(3)

struct service_message {
	uint32_t cmd;
	uint32_t arg;
};

int service_register(struct service *service);
struct service *service_lookup(const char *name);

int service_stop(const struct service *service);
int service_start(const struct service *service);
int service_pause(const struct service *service);
int service_resume(const struct service *service);

#endif /* __SERVICE_MANAGER_H__ */
