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

#include "gpio_handler.c"
#include "service_manager.h"
#include "accel_service.h"
#include "gps_service.h"
#include "pmic_service.h"

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

static void main_service_fn(void *param);
struct service main_service = {
	.name = "main",
	.fn = main_service_fn,
	.priority = 1,
};

void main_service_fn(void *param)
{
	struct service *service = (struct service *)param;

	while (1) {
		struct service_message smsg;
		if (service_receive_message(service, &smsg, portMAX_DELAY)) {
			continue;
		}

		switch (smsg.cmd) {
		case SERVICE_CMD_START:
			service_start(&pmic_service);
			service_sync(&pmic_service);

			service_start(&accel_service);
			service_start(&gps_service);
			break;
		case SERVICE_CMD_STOP:
			service_stop(&gps_service);
			service_stop(&accel_service);
			service_stop(&pmic_service);
			break;
		case SERVICE_CMD_PAUSE:

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

void app_main(void)
{
	printf("Hello world!\n");

	gpio_handler_init();

	service_register(&main_service);
	service_register(&pmic_service);
	service_register(&accel_service);
	service_register(&gps_service);

	service_start(&main_service);
	service_sync(&main_service);

	// All handled by service tasks now.
}
