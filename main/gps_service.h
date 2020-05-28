// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __GPS_SERVICE_H__
#define __GPS_SERVICE_H__

#include "service_manager.h"

struct service *gps_service_register();

#define GPS_CMD_SCOPE SERVICE_SCOPE('g', 'p')
#define GPS_CMD(_x)   SERVICE_CMD(GPS_CMD_SCOPE, _x)

#define GPS_CMD_LOCK_STATUS GPS_CMD(3)

int gps_subscribe_lock_status(struct service *service, struct service *subscriber);
int gps_unsubscribe_lock_status(struct service *service, struct service *subscriber);

#endif /* __GPS_SERVICE_H__ */
