// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"

#include "esp_log.h"

#include "axp192.h"
#include "gpio_handler.h"
#include "i2c_helper.h"
#include "pmic_service.h"

#define TAG "PMIC SERVICE"

#define PMIC_IRQ       (GPIO_NUM_35)

static void pmic_service_fn(void *param);

struct service pmic_service = {
	.name = "pmic",
	.fn = pmic_service_fn,
	.priority = 1,
};

// These match the register definition.
enum led_mode {
	// Switch to charger-controlled LED
	LED_MODE_AUTO = 0,
	LED_MODE_MANUAL_OFF = 1,
	LED_MODE_MANUAL_1HZ = 3,
	LED_MODE_MANUAL_4HZ = 5,
	LED_MODE_MANUAL_ON = 7,
};

static void set_chgled(const axp192_t *axp, enum led_mode mode)
{
	uint8_t val;
	axp192_read_reg(axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, &val);

	val &= ~(7 << 3);
	val |= (mode << 3);

	axp192_write_reg(axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, val);
}

static void setup_battery_charger(const axp192_t *axp) {
	// 4.2 V, 780 mA
	axp192_write_reg(axp, AXP192_CHARGE_CONTROL_1, 0xC8);
	// Default values - 40 min precharge, 8 hour constant current
	axp192_write_reg(axp, AXP192_CHARGE_CONTROL_2, 0x41);
}

static void power_off(const axp192_t *axp)
{
	// Flash LED
	set_chgled(axp, LED_MODE_MANUAL_4HZ);

	// Save whatever state needs saving...

	// Power off all rails.
	axp192_set_rail_state(axp, AXP192_RAIL_DCDC1, false);
	axp192_set_rail_state(axp, AXP192_RAIL_DCDC2, false);
	axp192_set_rail_state(axp, AXP192_RAIL_LDO2, false);
	axp192_set_rail_state(axp, AXP192_RAIL_LDO3, false);
	axp192_set_rail_state(axp, AXP192_RAIL_EXTEN, false);

	vTaskDelay(1000 / portTICK_PERIOD_MS);

	// Turn off.
	axp192_write_reg(axp, AXP192_SHUTDOWN_BATTERY_CHGLED_CONTROL, 0x80);

	for ( ;; ) {
		// This function does not return
	}
}

// TODO: Once tasks can request voltages, this can be local to pmic_service_fn
const axp192_t axp = {
	.read = &i2c_read,
	.write = &i2c_write,
};

static void pmic_service_fn(void *param)
{
	struct service *service = (struct service *)param;
	uint8_t irqmask[5] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
	uint8_t irqstatus[5] = { 0 };
	gpio_config_t io_conf;

	i2c_init();

	axp192_init(&axp);
	setup_battery_charger(&axp);

	// Flash LED
	set_chgled(&axp, LED_MODE_MANUAL_1HZ);

	// Power off everything we don't need
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC1, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_DCDC2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO2, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_LDO3, false);
	axp192_set_rail_state(&axp, AXP192_RAIL_EXTEN, false);

	set_chgled(&axp, LED_MODE_AUTO);

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
	io_conf.pin_bit_mask = (1ULL << PMIC_IRQ);
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpio_config(&io_conf);

	gpio_handler_add_irq(PMIC_IRQ, service);

	while (1) {
		struct service_message smsg;
		if (service_receive_message(service, &smsg, portMAX_DELAY)) {
			continue;
		}

		switch (smsg.cmd) {
		case SERVICE_CMD_START:
			break;
		case SERVICE_CMD_STOP:
			break;
		case SERVICE_CMD_PAUSE:
			break;
		case SERVICE_CMD_RESUME:
			break;
		case GPIO_IRQ_CMD:
			if (smsg.arg == PMIC_IRQ) {
				axp192_read_irq_status(&axp, irqmask, irqstatus, true);
				if ((irqstatus[2] & (1 << 1))) {
					struct service *main_service = service_lookup("main");
					service_stop(main_service);
					// FIXME: Calling sync across services is probably a bad
					// idea, as it could deadlock - if the service we're syncing
					// on is syncing on us, too.
					// Here we know it's not the case, but that's perhaps not a
					// good general approach.
					service_sync(main_service);

					// TODO: Is this what we want? Or an explicit "power off" command
					// sent by "main"? Or perhaps just power-off in STOP?
					power_off(&axp);
				}
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
