// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "service_manager.h"

#define container_of(ptr, type, member) (type *)((char *)(ptr) - offsetof(type, member))

struct __node {
	struct __node *next;
};

struct __node services = {
	.next = NULL,
};

struct service {
	struct __node node; // Keep node as first element
	char name[32];

	QueueHandle_t cmdq;
	portMUX_TYPE lock;
	volatile uint32_t sent;
	volatile uint32_t processed;

	TaskHandle_t task;
	uint32_t stack_depth;
};

#define for_each_service(_s) for (_s = (struct service *)services.next; _s != NULL; _s = (struct service *)_s->node.next)

static void add_service(struct service *service)
{
	struct __node *n = &services;
	while (n->next != NULL) {
		n = n->next;
	}
	n->next = &service->node;
}

static struct service *find_service(const char *name)
{
	struct __node *n = &services;
	struct service *service;

	while (n->next != NULL) {
		n = n->next;
		service = container_of(n, struct service, node);
		if (!strcmp(name, service->name)) {
			return service;
		}
	}

	return NULL;
}

struct service *service_register(const char *name, void (*fn)(void *self), UBaseType_t priority, uint32_t stack_depth)
{
	BaseType_t ret;
	struct service *service = calloc(1, sizeof(*service));
	if (!service) {
		return NULL;
	}

	if (strlen(name) > sizeof(service->name) - 1) {
		return NULL;
	}
	strcpy(service->name, name);

	service->cmdq = xQueueCreate(5, sizeof(struct service_message));
	if (!service->cmdq) {
		goto fail;
	}

	service->sent = 0;
	service->processed = 0;
	vPortCPUInitializeMutex(&service->lock);

	service->stack_depth = stack_depth;

	ret = xTaskCreatePinnedToCore(fn, service->name, stack_depth, service, priority, &service->task, 1);
	if (ret != pdPASS) {
		goto fail;
	}

	add_service(service);

	return service;

fail:
	if (service->cmdq)
		vQueueDelete(service->cmdq);
	free(service);
	return NULL;
}

struct service *service_lookup(const char *name)
{
	return find_service(name);
}

void service_ack(struct service *service)
{
	portENTER_CRITICAL(&service->lock);
	service->processed++;
	portEXIT_CRITICAL(&service->lock);
}

int service_send_message_from_isr(struct service *service, const struct service_message *smsg, BaseType_t *xHigherPriorityTaskWoken)
{
	BaseType_t ret = xQueueSendToBackFromISR(service->cmdq, smsg, xHigherPriorityTaskWoken);
	if (ret != pdTRUE) {
		return -1;
	}

	portENTER_CRITICAL_ISR(&service->lock);
	service->sent++;
	portEXIT_CRITICAL_ISR(&service->lock);

	return 0;
}

int service_send_message(struct service *service, const struct service_message *smsg, TickType_t timeout)
{
	BaseType_t ret = xQueueSendToBack(service->cmdq, smsg, timeout);
	if (ret != pdTRUE) {
		return -1;
	}

	portENTER_CRITICAL(&service->lock);
	service->sent++;
	portEXIT_CRITICAL(&service->lock);

	return 0;
}

int service_receive_message(struct service *service, struct service_message *smsg, TickType_t timeout)
{
	BaseType_t ret = xQueueReceive(service->cmdq, smsg, timeout);
	if (ret != pdTRUE) {
		return -1;
	}

	return 0;
}

int service_stop(struct service *service)
{
	static struct service_message smsg = {
		.cmd = SERVICE_CMD_STOP,
	};
	return service_send_message(service, &smsg, portMAX_DELAY);
}

int service_start(struct service *service)
{
	static struct service_message smsg = {
		.cmd = SERVICE_CMD_START,
	};
	return service_send_message(service, &smsg, portMAX_DELAY);
}

int service_pause(struct service *service)
{
	static struct service_message smsg = {
		.cmd = SERVICE_CMD_PAUSE,
	};
	return service_send_message(service, &smsg, portMAX_DELAY);
}

int service_resume(struct service *service)
{
	static struct service_message smsg = {
		.cmd = SERVICE_CMD_RESUME,
	};
	return service_send_message(service, &smsg, portMAX_DELAY);
}

void service_sync(const struct service *service)
{
	uint32_t sent, processed;

	sent = service->sent;
	processed = service->processed;
	while (processed != sent) {
		// FIXME: Use a semaphore
		vTaskDelay(1);
		sent = service->sent;
		processed = service->processed;
	}
}

static void __service_dump_stats(struct service *service)
{
	UBaseType_t watermark;
	printf("Service '%s':\n", service->name);
	watermark = uxTaskGetStackHighWaterMark(service->task);
	printf("    Stack high mark: %d / %d words used\n", service->stack_depth - watermark, service->stack_depth);
}

void service_dump_stats(struct service *service)
{
	if (service) {
		__service_dump_stats(service);
	} else {
		for_each_service(service) {
			__service_dump_stats(service);
		}
	}
}
