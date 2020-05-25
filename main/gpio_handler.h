// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>
#ifndef __GPIO_HANDLER_H__
#define __GPIO_HANDLER_H__

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "service_manager.h"

#define GPIO_HANDLER_CMD_SCOPE SERVICE_SCOPE('g', 'p')

#define GPIO_IRQ_CMD SERVICE_CMD(GPIO_HANDLER_CMD_SCOPE, 1)

struct gpio_irq_ctx {
	struct service *service;
	uint32_t pin;
};

void gpio_handler_init();
void gpio_handler_add_irq(gpio_num_t gpio_num, struct service *service);

#endif /* __GPIO_HANDLER_H__ */
