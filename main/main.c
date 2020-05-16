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
#include "freertos/event_groups.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/adc.h"
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

#define WEB_SERVER "192.168.0.253"
#define WEB_PORT "8000"
#define WEB_PATH "/"

static const char *REQUEST_F = "PUT %s HTTP/1.0\r\n"
    "Host: "WEB_SERVER":"WEB_PORT"\r\n"
    "User-Agent: esp-idf/1.0 esp32\r\n"
    "Content-type: application/octet-stream\r\n"
    "Content-length: %lld\r\n"
    "\r\n";

int upload(char *filename, size_t size)
{
	const struct addrinfo hints = {
		.ai_family = AF_INET,
		.ai_socktype = SOCK_STREAM,
	};
	struct addrinfo *res;
	struct in_addr *addr;
	int s, r;
	char recv_buf[64];

	int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

	if(err != 0 || res == NULL) {
		ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
		return -1;
	}

	addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
	ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

	s = socket(res->ai_family, res->ai_socktype, 0);
	if(s < 0) {
		ESP_LOGE(TAG, "... Failed to allocate socket.");
		freeaddrinfo(res);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		return -1;
	}
	ESP_LOGI(TAG, "... allocated socket");

	if(connect(s, res->ai_addr, res->ai_addrlen) != 0) {
		ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
		close(s);
		freeaddrinfo(res);
		return -1;
	}

	ESP_LOGI(TAG, "... connected");
	freeaddrinfo(res);

	struct timeval receiving_timeout;
	receiving_timeout.tv_sec = 5;
	receiving_timeout.tv_usec = 0;
	if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
				sizeof(receiving_timeout)) < 0) {
		ESP_LOGE(TAG, "... failed to set socket receiving timeout");
		close(s);
		return -1;
	}
	ESP_LOGI(TAG, "... set socket receiving timeout success");

	FILE *fs = fdopen(s, "w+");

	r = fprintf(fs, REQUEST_F, filename, (long long int)size);
	if (r < 0) {
		ESP_LOGE(TAG, "... socket send failed");
		fclose(fs);
		return -1;
	}

	FILE *fi = fopen(filename, "r");

	do {
		r = fread(recv_buf, 1, sizeof(recv_buf), fi);
		if (r > 0) {
			r = fwrite(recv_buf, 1, r, fs);
		}
	} while (r > 0);
	fclose(fi);

	ESP_LOGI(TAG, "... socket send success");

	/* Read HTTP response */
	do {
		bzero(recv_buf, sizeof(recv_buf));
		r = fread(recv_buf, 1, sizeof(recv_buf)-1, fs);
		for(int i = 0; i < r; i++) {
			putchar(recv_buf[i]);
		}
	} while(r > 0);

	ESP_LOGI(TAG, "... done reading from socket. Last read return=%d errno=%d.", r, errno);
	fclose(fs);

	return 0;
}

void send_files()
{
	DIR *dir = opendir(SPIFFS_MOUNT_POINT);
	struct dirent *ent;
	struct stat st;
	int ret;

	while ((ent = readdir(dir))) {
		char filename[290];
		snprintf(filename, 290, "%s/%s", SPIFFS_MOUNT_POINT, ent->d_name);
		stat(filename, &st);
		printf("Entry: %s %ld\n", filename, st.st_size);

		ret = upload(filename, st.st_size);
		if (!ret) {
			unlink(filename);
		}
	}

	closedir(dir);
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

struct pvt_record {
	uint32_t timestamp;
	int32_t lon_semis;
	int32_t lat_semis;
	uint32_t acc;
};

extern int wifi_init_sta(void);
extern void wifi_stop(void);

void app_main(void)
{
	int i, ret;
	esp_err_t err;

	printf("Hello world!\n");

	exit_flags = xEventGroupCreate();

	i2c_init();

	axp192_init(&axp);
	setup_battery_charger();

	// Flash LED
	set_chgled(LED_MODE_MANUAL_4HZ);

	// Power off everything we don't need
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC1, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_EXTEN, false);

	setup_buttons();

	esp_vfs_spiffs_conf_t spiffsconf = {
		.base_path = SPIFFS_MOUNT_POINT,
		.partition_label = NULL,
		.max_files = 5,
		.format_if_mount_failed = true
	};
	err = esp_vfs_spiffs_register(&spiffsconf);
	ESP_ERROR_CHECK(err);

	//Initialize NVS
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
	}
	ESP_ERROR_CHECK(err);

	// Try and upload files
	ret = wifi_init_sta();
	if (ret == 0) {
		send_files();
	}
	wifi_stop();


	cycle_gps_power(true);

	struct gps_ctx *gps = gps_init();

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	struct ubx_message *msg, *resp;

	printf("Configure protocols...\n");
	msg = ubx_alloc(0x6, 0x00, 1);
	msg->payload_csum[0] = 1;
	while (1) {
		resp = gps_send_get_response(gps, msg, 1000 / portTICK_RATE_MS);
		if (resp) {
			break;
		}
		printf("Retry...\n");
	}
	ubx_free(msg);
	msg = resp;
	resp = NULL;

	msg->payload_csum[14] = 1;
	while (1) {
		ret = gps_send_get_ack(gps, msg, 1000 / portTICK_RATE_MS);
		if (!ret) {
			break;
		} else if (ret != -ETIMEDOUT) {
			printf("Unexpected error setting config.\n");
		}
		printf("Retry...\n");
	}

	printf("Set message config...\n");
	msg = ubx_alloc(0x6, 0x01, 3);
	msg->payload_csum[0] = 1;
	msg->payload_csum[1] = 7;
	msg->payload_csum[2] = 1;
	while (1) {
		ret = gps_send_get_ack(gps, msg, 1000 / portTICK_RATE_MS);
		if (!ret) {
			break;
		} else if (ret != -ETIMEDOUT) {
			printf("Unexpected error setting config.\n");
		}
		printf("Retry...\n");
	}
	ubx_free(msg);
	printf("Done...\n");

	bool locked = false;
	set_chgled(LED_MODE_MANUAL_4HZ);

	FILE *f = NULL;

	for (i = 0; !xEventGroupGetBits(exit_flags); i++) {
		int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 500 / portTICK_RATE_MS);

		msg = ubx_receive(data, len);
		while (msg) {
			struct ubx_nav_pvt *pvt = (struct ubx_nav_pvt *)msg;

			struct pvt_record rec = {
				.timestamp = pvt->day * 24 * 3600 + pvt->hour * 3600 + pvt->min * 60 + pvt->sec,
				.lon_semis = ubx_deg_to_semicircles(pvt->lon),
				.lat_semis = ubx_deg_to_semicircles(pvt->lat),
				.acc = pvt->hAcc,
			};

			if (!locked && (pvt->flags & 1)) {
				locked = true;
				// Stop flashing
				set_chgled(LED_MODE_MANUAL_OFF);

				if (f == NULL) {
					char filename[128];
					snprintf(filename, 128, "%s/%d.bin", SPIFFS_MOUNT_POINT, rec.timestamp);
					//f = fopen(filename, "w");
				}
			} else if (locked && !(pvt->flags & 1)) {
				// Start flashing
				locked = false;
				set_chgled(LED_MODE_MANUAL_4HZ);
			}

			if (f != NULL) {
				struct timeval start, end;
				gettimeofday(&start, NULL);
				fwrite(&rec, sizeof(rec), 1, f);
				gettimeofday(&end, NULL);

				int64_t us = ((int64_t)end.tv_sec * 1000000L + (int64_t)end.tv_usec) -
					((int64_t)start.tv_sec * 1000000L + (int64_t)start.tv_usec);
				printf("Flash write took %lld us\n", us);
			}

			ubx_print_nav_pvt((struct ubx_nav_pvt *)msg);
			ubx_free(msg);

			// See if there's any data left
			msg = ubx_receive(NULL, 0);
		}

		fflush(stdout);
	}

	if (f != NULL) {
		fflush(f);
		fclose(f);
	}

	esp_vfs_spiffs_unregister(spiffsconf.partition_label);

	power_off();
}
