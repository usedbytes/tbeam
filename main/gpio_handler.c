// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "gpio_handler.h"
#include "service_manager.h"

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	struct gpio_irq_ctx *ctx = (struct gpio_irq_ctx *)arg;
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	struct service_message smsg = {
		.cmd = GPIO_IRQ_CMD,
		.arg = ctx->pin,
	};

	// Nothing we can do if the queue is full, so just ignore return code
	service_send_message_from_isr(ctx->service, &smsg, &xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR();
	}
}

void gpio_handler_init()
{
	gpio_install_isr_service(0);
}

void gpio_handler_add_irq(gpio_num_t gpio_num, struct service *service)
{
	struct gpio_irq_ctx *ctx = calloc(1, sizeof(*ctx));
	ctx->service = service;
	ctx->pin = gpio_num;

	gpio_isr_handler_add(gpio_num, gpio_isr_handler, (void *)ctx);
}
