// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __NETWORK_SERVICE_H__
#define __NETWORK_SERVICE_H__

#include "service_manager.h"

#define NETWORK_CMD_SCOPE SERVICE_SCOPE('n', 'e')
#define NETWORK_CMD(_x)   SERVICE_CMD(NETWORK_CMD_SCOPE, _x)

#define NETWORK_CMD_NETWORK_STATUS NETWORK_CMD(3)

enum network_status {
	NETWORK_STATUS_CONNECTED,
	NETWORK_STATUS_FAILED,
};

struct service *network_service_register();

int network_subscribe_network_status(struct service *service, struct service *subscriber);
int network_unsubscribe_network_status(struct service *service, struct service *subscriber);

#endif /* __NETWORK_SERVICE_H__ */
