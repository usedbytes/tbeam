// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"

#include "accel_service.h"

static void accel_service_fn(void *param);
struct service accel_service = {
	.name = "accelerometer",
	.fn = accel_service_fn,
	.priority = configMAX_PRIORITIES - 1,
};

static void setup_adc()
{
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
}

#define ACCEL_SAMPLE_CMD 1

// Approach derived from https://esp32.com/viewtopic.php?t=1341
static void IRAM_ATTR timer_group0_isr(void *param) {
	QueueHandle_t q = (QueueHandle_t)param;
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static const struct service_message smsg = {
		.cmd = ACCEL_SAMPLE_CMD,
	};

	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
	timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

	// Nothing we can do if the queue is full, so just ignore return code
	xQueueSendToBackFromISR(q, &smsg, &xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(); // this wakes up accel_service immediately
	}
}

static inline int64_t square(int32_t a)
{
	return a * a;
}

static uint32_t calc_variance(uint16_t *vals, int nvals)
{
	int i;
	int32_t mean = 0;
	int64_t var = 0;

	for (i = 0; i < nvals; i++) {
		mean += vals[i];
	}
	mean = mean / 16;

	for (i = 0; i < nvals; i++) {
		var = var + square(mean - vals[i]);
	}

	return var / 16;
}

static void accel_service_fn(void *param)
{
	struct service *service = (struct service *)param;

	setup_adc();

	// Power up accelerometer
	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.pin_bit_mask = (1ULL << GPIO_NUM_25),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
	};
	gpio_config(&io_conf);

	gpio_set_drive_capability(GPIO_NUM_25, GPIO_DRIVE_CAP_1);
	gpio_set_level(GPIO_NUM_25, 1);

	timer_config_t config = {
		.divider = 16,
		.counter_dir = TIMER_COUNT_UP,
		.counter_en = TIMER_PAUSE,
		.alarm_en = TIMER_ALARM_EN,
		.auto_reload = TIMER_AUTORELOAD_EN,
	};
	timer_init(TIMER_GROUP_0, TIMER_0, &config);

#define ACCEL_SAMPLES_PER_SEC 16
	timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);
	timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_BASE_CLK / (16 * ACCEL_SAMPLES_PER_SEC));
	timer_enable_intr(TIMER_GROUP_0, TIMER_0);
	timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, service->cmdq, ESP_INTR_FLAG_IRAM, NULL);

	timer_start(TIMER_GROUP_0, TIMER_0);

	int idx = 0;
	uint16_t samples[3][16];
	while (1) {
		struct service_message smsg;
		if (!xQueueReceive(service->cmdq, &smsg, portMAX_DELAY)) {
			continue;
		}

		switch (smsg.cmd) {
		case ACCEL_SAMPLE_CMD:
			samples[0][idx] = adc1_get_raw(ADC1_CHANNEL_0);
			samples[1][idx] = adc1_get_raw(ADC1_CHANNEL_1);
			samples[2][idx] = adc1_get_raw(ADC1_CHANNEL_3);

			idx++;
			if (idx == 16) {
				printf("%5d, %5d, %5d\n", calc_variance(samples[0], 16),
				       calc_variance(samples[1], 16), calc_variance(samples[2], 16));
				idx = 0;
			}
			break;
		}
	}
}
