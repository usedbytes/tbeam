// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "list.h"
#include "service_manager.h"

struct list_node services = {
	.next = NULL,
};

struct service {
	struct list_node node; // Keep node as first element
	char name[32];

	QueueHandle_t cmdq;
	portMUX_TYPE lock;
	volatile uint32_t sent;
	volatile uint32_t processed;

	TaskHandle_t task;
	uint32_t stack_depth;
};

#define for_each_service(_s) list_for_each(&services, struct service, node, _s)

static void add_service(struct service *service)
{
	list_add_tail(&services, &service->node);
}

static bool __match_service(struct list_node *node, void *data)
{
	struct service *service = container_of(node, struct service, node);
	const char *name = (const char *)data;

	if (!strcmp(name, service->name)) {
		return true;
	}

	return false;
}

static struct service *find_service(const char *name)
{
	return (struct service *)list_find(&services, __match_service, (void *)name);
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

#if configGENERATE_RUN_TIME_STATS
	// XXX: "Approximately 40 bytes per task should be sufficient." (measured 44)
	static char statsbuf[4096];
	vTaskGetRunTimeStats(statsbuf);
	printf("%s\n", statsbuf);
#endif
}
