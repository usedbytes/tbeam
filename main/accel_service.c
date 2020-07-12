// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/timer.h"

#include "accel_service.h"

#define TAG "accel_service"

#define ACCEL_VCC_PIN        (GPIO_NUM_25)
// XXX: These axes are probably not in the right order.
#define ACCEL_ADC_CHANNEL_X  (ADC1_CHANNEL_0)
#define ACCEL_ADC_CHANNEL_Y  (ADC1_CHANNEL_1)
#define ACCEL_ADC_CHANNEL_Z  (ADC1_CHANNEL_3)

#define ACCEL_SERVICE_SAMPLE_CMD SERVICE_CMD_LOCAL(1)

static void accel_service_fn(void *param);
struct service *accel_service_register()
{
	return service_register("accelerometer", accel_service_fn, configMAX_PRIORITIES - 1, 4096);
}

static void setup_adc()
{
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ACCEL_ADC_CHANNEL_X, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(ACCEL_ADC_CHANNEL_Y, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(ACCEL_ADC_CHANNEL_Z, ADC_ATTEN_DB_11);
}

// Approach derived from https://esp32.com/viewtopic.php?t=1341
static void IRAM_ATTR timer_group0_isr(void *param) {
	struct service *service = (struct service *)param;
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	// DRAM_ATTR is important otherwise this ends up in flash and the ISR
	// crashes when cache is disables.
	static const DRAM_ATTR struct service_message smsg = {
		.cmd = ACCEL_SERVICE_SAMPLE_CMD,
	};

	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
	timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

	// Nothing we can do if the queue is full, so just ignore return code
	service_send_message_from_isr(service, &smsg, &xHigherPriorityTaskWoken);
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

	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_DISABLE,
		.pin_bit_mask = (1ULL << ACCEL_VCC_PIN),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
	};
	gpio_config(&io_conf);
	gpio_set_drive_capability(ACCEL_VCC_PIN, GPIO_DRIVE_CAP_1);

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
	timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, service, ESP_INTR_FLAG_IRAM, NULL);

	int idx = 0;
	uint16_t samples[3][16];

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
		case SERVICE_CMD_PAUSE:
			timer_pause(TIMER_GROUP_0, TIMER_0);
			timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
			gpio_set_level(ACCEL_VCC_PIN, 0);
			state = STOPPED;
			break;
		case SERVICE_CMD_START:
		case SERVICE_CMD_RESUME:
			gpio_set_level(ACCEL_VCC_PIN, 1);
			timer_start(TIMER_GROUP_0, TIMER_0);
			state = RUNNING;
			break;
		case ACCEL_SERVICE_SAMPLE_CMD:
			if (state != RUNNING) {
				break;
			}
			samples[0][idx] = adc1_get_raw(ACCEL_ADC_CHANNEL_X);
			samples[1][idx] = adc1_get_raw(ACCEL_ADC_CHANNEL_Y);
			samples[2][idx] = adc1_get_raw(ACCEL_ADC_CHANNEL_Z);

			idx++;
			if (idx == 16) {
				ESP_LOGI(TAG, "%5d, %5d, %5d\n", calc_variance(samples[0], 16),
				         calc_variance(samples[1], 16), calc_variance(samples[2], 16));
				idx = 0;
			}
			break;
		default:
			// Unknown command
			break;
		}

		// Acknowledge the command
		service_ack(service);
	}
}
