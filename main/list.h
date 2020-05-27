// SPDX-License-Identifier: MIT
// Copyright (c) 2020 Brian Starkey <stark3y@gmail.com>

#ifndef __LIST_H__
#define __LIST_H__

#include <stdbool.h>

struct list_node {
	struct list_node *next;
};

void list_add_tail(struct list_node *list, struct list_node *add);
struct list_node *list_del(struct list_node *list, struct list_node *del);
struct list_node *list_find(struct list_node *list, bool (*match)(struct list_node *, void *), void *data);

#define container_of(ptr, type, member) (type *)((char *)(ptr) - offsetof(type, member))

#define list_for_each(_list, _type, _member, _cursor) \
	for (_cursor = container_of((_list)->next, _type, _member); \
	     ((_list)->next != NULL) && (_cursor != NULL); \
	     _cursor = _cursor->_member.next == NULL ? NULL : container_of(_cursor->_member.next, _type, _member))

#endif
