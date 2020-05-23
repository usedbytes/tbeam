// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>
// Portions Copyright (c) 2018 Manuel Bleichenbacher
//    From ttn-esp32 example: https://github.com/manuelbl/ttn-esp32/blob/master/examples/hello_world/main/main.cpp

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "driver/timer.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_spi_flash.h"
#include "esp_spiffs.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "i2c_helper.h"
#include "axp192.h"
#include "gps.h"
#include "ubx.h"

#define TAG "FOO"

#define GPS_UART_TXD (GPIO_NUM_12)
#define GPS_UART_RXD (GPIO_NUM_34)

#define LORA_SCK     (GPIO_NUM_5)
#define LORA_MISO    (GPIO_NUM_19)
#define LORA_MOSI    (GPIO_NUM_27)
#define LORA_SS      (GPIO_NUM_18)
#define LORA_DIO0     (GPIO_NUM_26)
#define LORA_DIO1     (GPIO_NUM_33)
#define LORA_DIO2     (GPIO_NUM_32)
#define LORA_RST     (GPIO_NUM_23)

#define AXP192_I2C_SDA (GPIO_NUM_21)
#define AXP192_I2C_SCL (GPIO_NUM_22)
#define AXP192_IRQ     (GPIO_NUM_35)

#define USER_BTN       (GPIO_NUM_38)
#define PMIC_IRQ       (GPIO_NUM_35)

#define HEADER_VOLTAGE_RAIL AXP192_RAIL_DCDC1
#define ESP32_VOLTAGE_RAIL  AXP192_RAIL_DCDC3
#define GPS_VOLTAGE_RAIL    AXP192_RAIL_LDO3
#define LORA_VOLTAGE_RAIL   AXP192_RAIL_LDO2

#define SPIFFS_MOUNT_POINT "/spiffs"

/*
 * GPS is on:  LDO3 (3v3)
 * LoRa is on: LDO2 (3v3)
 * R55 is pop
 * DCDC1 is "VCC_2.5V" - only goes to headers
 * DCDC2 is NC
 * DCDC3 is main +3v3 (ESP ++)
 * VSYS is 5V
 * VBUS0 goes to LDO, only for CP2104 (ME6211)
 * LDO1 is VCC_RTC - connected to GPS backup battery
 * LDOIO is NC
 *
 * SysEN is connected to LDO1
 * N_OE is floating
 */

#define BUF_SIZE (1024)

const axp192_t axp = {
	.read = &i2c_read,
	.write = &i2c_write,
};
uint8_t irqmask[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
uint8_t irqstatus[5] = { 0 };

static xQueueHandle gpio_evt_queue = NULL;
static EventGroupHandle_t exit_flags;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t)arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// These match the register definition.
enum led_mode {
	// Switch to charger-controlled LED
	LED_MODE_AUTO = 0,
	LED_MODE_MANUAL_OFF = 1,
	LED_MODE_MANUAL_1HZ = 3,
	LED_MODE_MANUAL_4HZ = 5,
	LED_MODE_MANUAL_ON = 7,
};

static void set_chgled(enum led_mode mode)
{
	uint8_t val;
	axp192_read_reg(&axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, &val);

	val &= ~(7 << 3);
	val |= (mode << 3);

	axp192_write_reg(&axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, val);
}

static void power_off()
{
	// Flash LED
	set_chgled(LED_MODE_MANUAL_4HZ);

	// Save whatever state needs saving...

	// Power off all rails.
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC1, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_EXTEN, false);

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Turn off.
	axp192_write_reg(&axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, 0x80);

	for ( ;; ) {
		// This function does not return
	}
}

void monitor_buttons(void *pvParameter)
{
	uint32_t io_num;
	while(1) {
		if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
			switch (io_num) {
				case USER_BTN:
					printf("Button pressed. GPIO[%d] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
					break;
				case PMIC_IRQ:
					axp192_read_irq_status(&axp, irqmask, irqstatus, true);
					if (irqstatus[2] & (1 << 1)) {
						printf("Power button pressed.\n");

						xEventGroupSetBits(exit_flags, 1);
					}
					break;
				default:
					printf("Unexpected GPIO event\n");
			}
		}
	}
}

void setup_battery_charger() {
	// 4.2 V, 780 mA
	axp192_write_reg(&axp, AXP192_CHARGE_CONTROL_1, 0xC8);
	// Default values - 40 min precharge, 8 hour constant current
	axp192_write_reg(&axp, AXP192_CHARGE_CONTROL_2, 0x41);
}

static void setup_adc()
{
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
}

static void setup_buttons()
{
	gpio_config_t io_conf;

	// Clear all PMIC IRQs
	axp192_read_irq_status(&axp, irqmask, irqstatus, true);
	memset(irqstatus, 0, sizeof(irqstatus));

	irqmask[0] = 0;
	irqmask[1] = 0;
	// Short button press IRQ
	irqmask[2] = (1 << 1);
	irqmask[3] = 0;
	irqmask[4] = 0;
	axp192_write_irq_mask(&axp, irqmask);

	io_conf.intr_type = GPIO_INTR_NEGEDGE;
	io_conf.pin_bit_mask = (1ULL << USER_BTN);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_config(&io_conf);

	io_conf.intr_type = GPIO_INTR_NEGEDGE;
	io_conf.pin_bit_mask = (1ULL << PMIC_IRQ);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_config(&io_conf);

	gpio_evt_queue = xQueueCreate(3, sizeof(uint32_t));

	xTaskCreate(monitor_buttons, "monitor_buttons", 1024 * 4, (void* )0, 3, NULL);

	gpio_install_isr_service(0);
	gpio_isr_handler_add(USER_BTN, gpio_isr_handler, (void*)USER_BTN);
	gpio_isr_handler_add(PMIC_IRQ, gpio_isr_handler, (void*)PMIC_IRQ);
}

static void cycle_gps_power(bool on)
{
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
	axp192_set_rail_millivolts(&axp, AXP192_RAIL_LDO3, 3300);

	if (on) {
		axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, true);
	}
}

static QueueHandle_t accel_queue;

#define ADC_TIMER 1

// Approach derived from https://esp32.com/viewtopic.php?t=1341
void IRAM_ATTR timer_group0_isr(void *param) {
	static BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	static const uint32_t event = ADC_TIMER;

	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
	timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);

	xQueueSendToBackFromISR(accel_queue, &event, &xHigherPriorityTaskWoken);
	if( xHigherPriorityTaskWoken) {
		portYIELD_FROM_ISR(); // this wakes up accel_service immediately
	}
}

static inline int64_t square(int32_t a)
{
	return a * a;
}

uint32_t calc_variance(uint16_t *vals, int nvals)
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

void accel_service(void *param)
{
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

	// Set up the timer
	accel_queue = xQueueCreate(5, sizeof(uint32_t));

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
	timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);

	timer_start(TIMER_GROUP_0, TIMER_0);

	int idx = 0;
	uint16_t samples[3][16];
	while (1) {
		uint32_t event;
		if (!xQueueReceive(accel_queue, &event, portMAX_DELAY)) {
			continue;
		}

		switch (event) {
		case ADC_TIMER:
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

struct pvt_record {
	uint32_t timestamp;
	int32_t lon_semis;
	int32_t lat_semis;
	uint32_t acc;
};

void app_main(void)
{
	int i, ret;

	printf("Hello world!\n");

	exit_flags = xEventGroupCreate();

	i2c_init();

	axp192_init(&axp);
	setup_battery_charger();

	// Flash LED
	set_chgled(LED_MODE_MANUAL_1HZ);

	// Power off everything we don't need
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC1, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_EXTEN, false);

	setup_buttons();

	cycle_gps_power(true);

	struct gps_ctx *gps = gps_init(UART_NUM_1, GPS_UART_TXD, GPS_UART_RXD);

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	ret = gps_set_ubx_protocol(gps);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set UBX protocol: %d\n", ret);
		return;
	}

	ret = gps_set_message_rate(gps, UBX_MSG_CLASS_NAV, UBX_MSG_ID_NAV_PVT, 1);
	if (ret) {
		ESP_LOGE(TAG, "Failed to set NAV_PVT rate: %d\n", ret);
		return;
	}

	set_chgled(LED_MODE_AUTO);

	xTaskCreatePinnedToCore(accel_service, "accel_service", 4096, NULL, configMAX_PRIORITIES - 1, NULL, 1);

	for (i = 0; !xEventGroupGetBits(exit_flags); i++) {
		float battvolt;
		struct ubx_message *msg = gps_receive(gps, 500 / portTICK_RATE_MS);
		if (!msg) {
			continue;
		}

		if (msg->hdr.class != UBX_MSG_CLASS_NAV || msg->hdr.id != UBX_MSG_ID_NAV_PVT) {
			continue;
		}

		struct ubx_nav_pvt *pvt = (struct ubx_nav_pvt *)msg;

		axp192_read(&axp, AXP192_BATTERY_VOLTAGE, &battvolt);

		printf("Battery Voltage: %2.3f V\n", battvolt);
		ubx_print_nav_pvt(pvt);
		ubx_free(msg);

		fflush(stdout);
	}

	power_off();
}
