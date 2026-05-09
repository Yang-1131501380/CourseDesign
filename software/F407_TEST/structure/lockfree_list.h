#pragma once

#include "c_base.h"

typedef struct lf_list_node
{
  _Atomic(struct lf_list_node*) next;
  size_t size;
} lf_list_node_t;

typedef struct
{
  lf_list_node_t head;
} lf_list_t;

static inline void lf_list_node_init(lf_list_node_t* n, size_t size)
{
  atomic_store_explicit(&n->next, NULL, memory_order_relaxed);
  n->size = size;
}

void lf_list_init(lf_list_t* l);
void lf_list_add(lf_list_t* l, lf_list_node_t* n);
uint32_t lf_list_size(lf_list_t* l);

typedef error_code_t (*lf_list_each_fn)(lf_list_node_t* node, void* user);
error_code_t lf_list_foreach(lf_list_t* l, lf_list_each_fn fn, void* user);

