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

	// Internal
	struct __node node;
	QueueHandle_t cmdq;
	portMUX_TYPE lock;
	volatile uint32_t sent;
	volatile uint32_t processed;
};

#define SERVICE_CMD(_scope, _cmd)   (((_scope) << 16) | (_cmd))
#define SERVICE_CMD_GLOBAL(_cmd)    SERVICE_CMD(0, _cmd)
#define SERVICE_CMD_LOCAL(_cmd)     SERVICE_CMD(1, _cmd)

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

int service_send_message_from_isr(struct service *service, const struct service_message *smsg,
				  BaseType_t *xHigherPriorityTaskWoken);
int service_send_message(struct service *service, const struct service_message *smsg,
			 TickType_t timeout);
int service_receive_message(struct service *service, struct service_message *smsg,
			    TickType_t timeout);

int service_stop(struct service *service);
int service_start(struct service *service);
int service_pause(struct service *service);
int service_resume(struct service *service);
void service_sync(const struct service *service);

// For use by service "fn" routines only
void service_ack(struct service *service);

#endif /* __SERVICE_MANAGER_H__ */
