// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>
// Portions Copyright (c) 2018 Manuel Bleichenbacher
//    From ttn-esp32 example: https://github.com/manuelbl/ttn-esp32/blob/master/examples/hello_world/main/main.cpp

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_spi_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "i2c_helper.h"
#include "axp192.h"
#include "gps.h"

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

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
	uint32_t gpio_num = (uint32_t)arg;
	xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void power_off()
{
	// Flash LED
	axp192_write_reg(&axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, 0x6a);

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

						power_off();
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

void app_main(void)
{
	printf("Hello world!\n");

	i2c_init();

	axp192_init(&axp);
	setup_battery_charger();

	// Flash LED
	axp192_write_reg(&axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, 0x6a);
	vTaskDelay(500 / portTICK_PERIOD_MS);
	//axp192_write_reg(&axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, 0x46);

	// Power off everything we don't need
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC1, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_EXTEN, false);

	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_11);
	adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);

	// Clear all IRQs
	axp192_read_irq_status(&axp, irqmask, irqstatus, true);
	memset(irqstatus, 0, sizeof(irqstatus));

	irqmask[0] = 0;
	irqmask[1] = 0;
	// Short button press IRQ
	irqmask[2] = (1 << 1);
	irqmask[3] = 0;
	irqmask[4] = 0;
	axp192_write_irq_mask(&axp, irqmask);

	gpio_config_t io_conf;

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


	// Set the GPS voltage and power it up
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
	axp192_set_rail_millivolts(&axp, AXP192_RAIL_LDO3, 3300);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, true);

	struct gps_ctx *gps = gps_init();

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Configure a temporary buffer for the incoming data
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

	struct ubx_message *msg;
	uint8_t *p;
	int i;

	printf("Configure protocols...\n");
	// TODO: Wrap this up in a send + get-ACK function
	while (1) {
		msg = alloc_msg(0x6, 0x00, 1);
		msg->payload_csum[0] = 1;
		send_message(gps, msg);
		free(msg);
		msg = NULL;

		while (!msg) {
			int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100 / portTICK_RATE_MS);

			if (!len) {
				continue;
			}

			p = data;
			msg = receive_ubx(&p, len);
		}

		if (msg->hdr.class != 0x6 || msg->hdr.id != 0x0) {
			free(msg);
			continue;
		}

		msg->payload_csum[14] = 1;
		send_message(gps, msg);
		free(msg);
		msg = NULL;

		while (!msg) {
			int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100 / portTICK_RATE_MS);

			if (!len) {
				continue;
			}

			p = data;
			msg = receive_ubx(&p, len);
		}

		if (msg->hdr.class != 0x5 || msg->hdr.id != 0x1 || msg->payload_csum[0] != 0x6 || msg->payload_csum[1] != 0) {
			free(msg);
			continue;
		}

		free(msg);
		msg = NULL;
		break;
	}

	printf("Set message config...\n");
	// TODO: Wrap this up in a send + get-ACK function
	while (1) {
		msg = alloc_msg(0x6, 0x01, 3);
		msg->payload_csum[0] = 1;
		msg->payload_csum[1] = 7;
		msg->payload_csum[2] = 1;
		send_message(gps, msg);
		free(msg);
		msg = NULL;

		while (!msg) {
			int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 100 / portTICK_RATE_MS);

			if (!len) {
				continue;
			}

			p = data;
			msg = receive_ubx(&p, len);
		}

		printf("Response:");
		print_ubx(msg);
		if (msg->hdr.class != 0x5 || msg->hdr.id != 0x1 || msg->payload_csum[0] != 0x6 || msg->payload_csum[1] != 1) {
			free(msg);
			continue;
		}

		free(msg);
		msg = NULL;
		break;
	}

	printf("Done...\n");

	for (i = 0; ; i++) {
		int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 500 / portTICK_RATE_MS);

		if (len) {
			p = data;
			msg = receive_ubx(&p, len);
			if (msg) {
				print_ubx_nav_pvt((struct ubx_nav_pvt *)msg);
			} else {
				ESP_LOG_BUFFER_HEXDUMP("FOO", data, len, ESP_LOG_WARN);
			}
		}
		fflush(stdout);
	}
}
