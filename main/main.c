// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

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
#include "network_service.h"

#define TAG "main"

#define LORA_SCK     (GPIO_NUM_5)
#define LORA_MISO    (GPIO_NUM_19)
#define LORA_MOSI    (GPIO_NUM_27)
#define LORA_SS      (GPIO_NUM_18)
#define LORA_DIO0     (GPIO_NUM_26)
#define LORA_DIO1     (GPIO_NUM_33)
#define LORA_DIO2     (GPIO_NUM_32)
#define LORA_RST     (GPIO_NUM_23)

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

static int nvs_init()
{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		nvs_flash_erase();
		err = nvs_flash_init();
	}

	if (err != ESP_OK) {
		ESP_LOGE(TAG, "NVS initialisation failed");
		return -1;
	}

	return 0;
}

#define SERVER_URL "http://archer.local:8080"

static struct network_txn *build_battery_report(struct service *service, uint16_t millivolts)
{
	const int max_body_len = strlen("battery=65535") + 1;
	struct network_static_post_txn *post = network_new_static_post(service, NULL, max_body_len);

	post->base.cfg.url = SERVER_URL;
	post->base.content_type = "application/x-www-form-urlencoded";

	sprintf(post->data, "battery=%d", millivolts);
	post->len = strlen(post->data);

	return &post->base;
}

void list_files()
{
	DIR *dir = opendir(SPIFFS_MOUNT_POINT);
	struct dirent *ent;
	struct stat st;
	int ret;

	while ((ent = readdir(dir))) {
		char filename[290];
		snprintf(filename, 290, "/spiffs/%s", ent->d_name);
		stat(filename, &st);
		printf("Entry: %s %ld\n", filename, st.st_size);
	}

	closedir(dir);
}

static void spiffs_init()
{
	esp_vfs_spiffs_conf_t spiffs_conf = {
		.base_path = SPIFFS_MOUNT_POINT,
		.partition_label = NULL,
		.max_files = 5,
		.format_if_mount_failed = true
	};
	esp_err_t err = esp_vfs_spiffs_register(&spiffs_conf);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "couldn't mount spiffs\n");
	} else {
		size_t total = 0, used = 0;
		err = esp_spiffs_info(spiffs_conf.partition_label, &total, &used);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(err));
		} else {
			ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
		}

		if (used == 0) {
			FILE *fp = fopen(SPIFFS_MOUNT_POINT "/hello.txt", "w");
			fwrite("Hello, World!\n", 1, strlen("Hello, World!\n"), fp);
			fflush(fp);
			fclose(fp);
		}

		list_files();
	}
}

void main_service_fn(void *param)
{
	struct service *service = (struct service *)param;

	gpio_handler_init();
	nvs_init();
	spiffs_init();

	struct service *pmic_service = pmic_service_register();
	struct service *accel_service = accel_service_register();
	struct service *gps_service = gps_service_register();
	struct service *network_service = network_service_register();

	gps_subscribe_lock_status(gps_service, service);

	network_subscribe_network_status(network_service, service);

	bool network = false;
	uint16_t battery_mv = 0;

	while (1) {
		struct service_message smsg;
		if (service_receive_message(service, &smsg, portMAX_DELAY)) {
			continue;
		}

		switch (smsg.cmd) {
		case SERVICE_CMD_START:
		{
			service_start(pmic_service);
			service_sync(pmic_service);

			service_start(accel_service);
			service_start(gps_service);
			service_start(network_service);

			ESP_LOGI(TAG, "services started.\n");
			break;
		}
		case SERVICE_CMD_STOP:
			service_stop(network_service);
			service_stop(gps_service);
			service_stop(accel_service);
			service_stop(pmic_service);
			break;
		case SERVICE_CMD_PAUSE:

			break;
		case SERVICE_CMD_RESUME:

			break;
		case GPS_CMD_LOCK_STATUS:
			printf("GPS %s\n", smsg.arg ? "locked" : "not locked");
			break;
		case PMIC_CMD_REPORT_BATTERY:
			battery_mv = smsg.arg;
			if (network) {
				struct network_txn *txn = build_battery_report(service, battery_mv);
				if (txn) {
					network_txn_perform(network_service, txn);
				}
			}
			break;
		case NETWORK_CMD_NETWORK_STATUS:
			if (smsg.arg == NETWORK_STATUS_CONNECTED) {
				ESP_LOGI(TAG, "Network is connected!");
				network = true;
				pmic_request_battery(pmic_service, service);

				FILE *fp = fopen(SPIFFS_MOUNT_POINT "/hello.txt", "r");
				struct network_file_post_txn *txn = network_new_file_post(service, fp);
				txn->base.cfg.url = SERVER_URL "?filename=hello.txt";
				if (txn) {
					network_txn_perform(network_service, &txn->base);
				}
			} else if (smsg.arg == NETWORK_STATUS_FAILED) {
				ESP_LOGE(TAG, "Network connection failed.");
			}
			break;
		case NETWORK_CMD_TXN_RESULT:
		{
			struct network_txn *txn = (struct network_txn *)smsg.argp;
			ESP_LOGI(TAG, "Transaction %s", txn->err == ESP_OK ? "OK" : "FAIL");
			ESP_LOGI(TAG, "Network transaction result: %d", txn->result);

			// TODO: Figure out how to handle destructors for txns
			free(txn);
			break;
		}
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
	ESP_LOGI(TAG, "Hello world!\n");

	struct service *main_service = service_register("main", main_service_fn, 1, 4096);

	service_dump_stats(NULL);

	service_start(main_service);
	service_sync(main_service);

	while (1) {
		vTaskDelay(10000 / portTICK_PERIOD_MS);

		service_dump_stats(NULL);
	}

	// All handled by service tasks now.
}
