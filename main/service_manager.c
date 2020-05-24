// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "service_manager.h"

#define container_of(ptr, type, member) (type *)((char *)(ptr) - offsetof(type, member))

struct __node services = {
	.next = NULL,
};

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

int service_register(struct service *service)
{
	BaseType_t ret;

	assert(service->cmdq == 0);

	service->cmdq = xQueueCreate(5, sizeof(struct service_message));
	if (!service->cmdq) {
		goto fail;
	}

	service->sent = 0;
	service->processed = 0;
	vPortCPUInitializeMutex(&service->lock);

	ret = xTaskCreatePinnedToCore(service->fn, service->name, 4096, service, service->priority, NULL, 1);
	if (ret != pdPASS) {
		goto fail;
	}

	add_service(service);

	return 0;

fail:
	if (service->cmdq)
		vQueueDelete(service->cmdq);
	return -1;
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
