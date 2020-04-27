/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdbool.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_spi_flash.h"

#include "i2c_helper.h"
#include "axp192.h"


#define GPS_UART_TXD (GPIO_NUM_12)
#define GPS_UART_RXD (GPIO_NUM_34)

#define LORA_SCK     (GPIO_NUM_5)
#define LORA_MISO    (GPIO_NUM_19)
#define LORA_MOSI    (GPIO_NUM_27)
#define LORA_SS      (GPIO_NUM_18)
#define LORA_DI0     (GPIO_NUM_26)
#define LORA_RST     (GPIO_NUM_23)

#define AXP192_I2C_SDA (GPIO_NUM_21)
#define AXP192_I2C_SCL (GPIO_NUM_22)
#define AXP192_IRQ     (GPIO_NUM_35)

#define USER_BTN       (GPIO_NUM_36)

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

void app_main(void)
{
    printf("Hello world!\n");

    i2c_init();

    axp192_t axp = {
        .read = &i2c_read,
        .write = &i2c_write,
    };

    axp192_init(&axp);

    // Power off everything we don't need
    axp192_set_rail_state(&axp, AXP192_RAIL_DCDC1, false);
    axp192_set_rail_state(&axp, AXP192_RAIL_DCDC2, false);
    axp192_set_rail_state(&axp, AXP192_RAIL_LDO2, false);

    // Set the GPS voltage and power it up
    axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
    axp192_set_rail_millivolts(&axp, AXP192_RAIL_LDO3, 3300);
    axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, true);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, GPS_UART_TXD, GPS_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    int i;
    for (i = 0; i < 3000; i++) {
        // Read data from the UART
        int len = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, 20 / portTICK_RATE_MS);
        // Write data back to the UART
        //uart_write_bytes(UART_NUM_1, (const char *) data, len);
	fwrite(data, 1, len, stdout);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        fflush(stdout);
    }

    printf("Sleeping...\n");
    fflush(stdout);
    axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
    esp_deep_sleep(5000000);
}
