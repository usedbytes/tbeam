// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "network_service.h"

#define TAG "network_service"

static void network_service_fn(void *param);
struct service *network_service_register()
{
	return service_register("network", network_service_fn, configMAX_PRIORITIES - 1, 4096);
}

static void network_service_fn(void *param)
{
	struct service *service = (struct service *)param;

	enum state {
		STOPPED = 0,
		RUNNING = 1,
	} state = STOPPED;

	while (1) {
		struct service_message smsg;
		if (service_receive_message(service, &smsg, portMAX_DELAY)) {
			continue;
		}

		switch (smsg.cmd) {
		case SERVICE_CMD_STOP:
			break;
		case SERVICE_CMD_PAUSE:
			break;
		case SERVICE_CMD_START:
			break;
		case SERVICE_CMD_RESUME:
			break;
		default:
			// Unknown command
			break;
		}

		// Acknowledge the command
		service_ack(service);
	}
}
