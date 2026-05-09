#pragma once

#include "c_base.h"

typedef struct list_node
{
  struct list_node* next;
  size_t size;
} list_node_t;

typedef struct
{
  list_node_t head;
  mutex_t mtx;
} list_t;

static inline void list_node_init(list_node_t* n, size_t size)
{
  n->next = NULL;
  n->size = size;
}

void list_init(list_t* l);
void list_add(list_t* l, list_node_t* n);
uint32_t list_size(list_t* l);
error_code_t list_delete(list_t* l, list_node_t* n);

typedef error_code_t (*list_each_fn)(list_node_t* node, void* user);
error_code_t list_foreach(list_t* l, list_each_fn fn, void* user);

