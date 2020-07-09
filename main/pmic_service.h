// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __PMIC_SERVICE_H__
#define __PMIC_SERVICE_H__

#include "axp192.h"
#include "service_manager.h"

#define PMIC_CMD_SCOPE SERVICE_SCOPE('p', 'm')
#define PMIC_CMD(_arg) SERVICE_CMD(PMIC_CMD_SCOPE, _arg)

#define PMIC_CMD_REPORT_BATTERY PMIC_CMD(4)

struct service *pmic_service_register();

int pmic_request_rail(struct service *service, axp192_rail_t rail, uint16_t millivolts);
int pmic_release_rail(struct service *service, axp192_rail_t rail);
int pmic_request_battery(struct service *service, struct service *requestor);

#endif /* __PMIC_SERVICE_H__ */
