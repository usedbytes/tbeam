// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __PMIC_SERVICE_H__
#define __PMIC_SERVICE_H__

#include "axp192.h"
#include "service_manager.h"

extern struct service pmic_service;


int pmic_request_rail(struct service *service, axp192_rail_t rail, uint16_t millivolts);
int pmic_release_rail(struct service *service, axp192_rail_t rail);

#endif /* __PMIC_SERVICE_H__ */
