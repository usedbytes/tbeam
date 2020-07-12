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

	while ((ent = readdir(dir))) {
		char filename[290];
		snprintf(filename, 290, "/spiffs/%s", ent->d_name);
		stat(filename, &st);
		printf("Entry: %s %ld\n", filename, st.st_size);
	}

	closedir(dir);
}

struct files {
	int idx;
	int num_files;
	char **filenames;
};

static int filter(struct dirent *ent)
{
	return strstr(ent->d_name, ".fit") != NULL;
}

static struct files *find_files()
{
	int i;
	struct files *files = calloc(1, sizeof(*files));
	if (!files) {
		return NULL;
	}

	DIR *dir = opendir(SPIFFS_MOUNT_POINT);
	if (!dir) {
		ESP_LOGE(TAG, "opendir failed\n");
		goto free_files;
	}
	struct dirent *ent;

	// Scan once to find the number of files
	errno = 0;
	while ((ent = readdir(dir))) {
		if (!filter(ent)) {
			continue;
		}

		files->num_files++;
	}
	/* XXX: Seems like spiffs returns spurious I/O errors
	if (errno != 0) {
		ESP_LOGE(TAG, "first dir scan failed: %d %s\n", errno, strerror(errno));
		goto close_dir;
	}
	*/

	// Nothing to upload
	if (!files->num_files) {
		goto close_dir;
	}

	rewinddir(dir);

	files->filenames = calloc(files->num_files, sizeof(*files->filenames));
	if (!files->filenames) {
		ESP_LOGE(TAG, "filenames alloc failed\n");
		goto close_dir;
	}

	i = 0;
	while ((ent = readdir(dir))) {
		if (!filter(ent)) {
			continue;
		}

		files->filenames[i] = malloc(strlen(SPIFFS_MOUNT_POINT) + 1 + strlen(ent->d_name) + 1);
		if (!files->filenames[i]) {
			ESP_LOGE(TAG, "filename[%d] alloc failed\n", i);
			goto free_filenames;
		}
		sprintf(files->filenames[i], "%s/%s", SPIFFS_MOUNT_POINT, ent->d_name);
		i++;
	}
	/* XXX: Seems like spiffs returns spurious I/O errors
	if (errno != 0) {
		ESP_LOGE(TAG, "second dir scan failed\n");
		goto free_filenames;
	}
	*/

	closedir(dir);

	return files;

free_filenames:
	for (i = 0; i < files->num_files; i++) {
		if (files->filenames[i]) {
			free(files->filenames[i]);
		}
	}
	free(files->filenames);
close_dir:
	closedir(dir);
free_files:
	free(files);
	return NULL;
}

static char *next_file(struct files *files)
{
	if (files->idx < files->num_files) {
		return files->filenames[files->idx++];
	}

	return NULL;
}

struct file_upload_txn {
	struct network_file_post_txn base;
	char *filename;
};

static struct file_upload_txn *build_next_upload(struct files *files, struct service *service)
{
	if (!files) {
		return NULL;
	}

	char *filename = next_file(files);
	if (!filename) {
		return NULL;
	}

	struct file_upload_txn *txn = calloc(1, sizeof(*txn) + (strlen(SERVER_URL) + strlen("?filename=") + strlen(filename) + 1));
	if (!txn) {
		return NULL;
	}

	FILE *fp = fopen(filename, "r");
	if (!fp) {
		free(txn);
		return NULL;
	}

	txn->base.base.sender = service;
	txn->base.base.cfg.method = HTTP_METHOD_POST;
	txn->base.base.send_cb = network_file_post_send;
	txn->base.base.receive_cb = NULL;

	sprintf((char *)txn + sizeof(*txn), "%s%s%s", SERVER_URL, "?filename=", filename);
	txn->base.base.cfg.url = (char *)txn + sizeof(*txn);

	txn->base.fp = fp;
	txn->filename = filename;

	ESP_LOGI(TAG, "built upload transaction for: %s\n", filename);

	return txn;
}

static esp_err_t finish_upload_txn(struct file_upload_txn *txn)
{
	esp_err_t err = ESP_OK;

	if (txn->base.base.err == ESP_OK && txn->base.base.result == 200) {
		ESP_LOGI(TAG, "upload of %s sucessful, removing\n", txn->filename);
		unlink(txn->filename);
	} else {
		ESP_LOGI(TAG, "upload of %s failed, keeping\n", txn->filename);
		err = ESP_FAIL;
	}

	free(txn);
	return err;
}

static void free_files(struct files *files)
{
	int i;
	for (i = 0; i < files->num_files; i++) {
		if (files->filenames[i]) {
			free(files->filenames[i]);
		}
	}
	free(files->filenames);
	free(files);
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

		/*
		if (used == 0) {
			FILE *fp = fopen(SPIFFS_MOUNT_POINT "/hello.fit", "w");
			fwrite("Hello, World!\n", 1, strlen("Hello, World!\n"), fp);
			fflush(fp);
			fclose(fp);
		}
		*/

		list_files();
	}
}

void main_service_fn(void *param)
{
	esp_err_t err;
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

	struct files *files_for_upload = NULL;
	struct file_upload_txn *current_upload = NULL;

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

				// If there isn't any pending upload going on, start
				// a new upload context.
				// If there *is* some upload in progress, then we can't
				// do anything and have to let the TXN_RESULT handler
				// clean up.
				if (files_for_upload == NULL) {
					files_for_upload = find_files();
				}
				if (current_upload == NULL) {
					current_upload = build_next_upload(files_for_upload, service);
					if (current_upload) {
						network_txn_perform(network_service, &current_upload->base.base);
					}
				}

			} else if (smsg.arg == NETWORK_STATUS_FAILED) {
				ESP_LOGE(TAG, "Network connection failed.");
				network = false;
			}
			break;
		case NETWORK_CMD_TXN_RESULT:
		{
			struct network_txn *txn = (struct network_txn *)smsg.argp;
			ESP_LOGI(TAG, "Transaction %s", txn->err == ESP_OK ? "OK" : "FAIL");
			ESP_LOGI(TAG, "Network transaction result: %d", txn->result);

			if (txn == &current_upload->base.base) {
				err = finish_upload_txn(current_upload);
				current_upload = NULL;
				if (err == ESP_OK) {
					// There wasn't any error, so let's try the next file
					current_upload = build_next_upload(files_for_upload, service);
				}

				// If we've got a file to upload, then do it.
				// Otherwise, we either finished or there was an error. In both cases
				// just clean up.
				if (current_upload) {
					network_txn_perform(network_service, &current_upload->base.base);
				} else {
					free_files(files_for_upload);
					files_for_upload = NULL;
				}
			} else {
				// TODO: Figure out how to handle destructors for txns
				free(txn);
			}
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
