// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#include <stddef.h>

#include "list.h"

static struct list_node *__list_tail(struct list_node *list)
{
	while (list->next)
		list = list->next;
	return list;
}

void list_add_tail(struct list_node *list, struct list_node *add)
{
	list = __list_tail(list);
	list->next = add;
}

struct list_node *list_del(struct list_node *list, struct list_node *del)
{
	while (list->next) {
		if (list->next == del) {
			list->next = del->next;
			return del;
		}
		list = list->next;
	}

	return NULL;
}

struct list_node *list_find(struct list_node *list, bool (*match)(struct list_node *, void *), void *data)
{
	while ((list = list->next)) {
		if (match(list, data)) {
			return list;
		}
	}

	return NULL;
}
