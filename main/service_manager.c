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

	service->cmdq = xQueueCreate(5, sizeof(struct service_message));
	if (!service->cmdq) {
		return -1;
	}

	ret = xTaskCreatePinnedToCore(service->fn, service->name, 4096, service, service->priority, NULL, 1);
	if (ret != pdPASS) {
		vQueueDelete(service->cmdq);
		return -1;
	}

	add_service(service);

	return 0;
}

struct service *service_lookup(const char *name)
{
	return find_service(name);
}
