#pragma once

#include "c_base.h"

typedef enum
{
  RBT_RED = 0,
  RBT_BLACK = 1
} rbt_color_t;

typedef struct rbt_node
{
  void* key;
  void* data;
  size_t data_size;
  rbt_color_t color;
  struct rbt_node* left;
  struct rbt_node* right;
  struct rbt_node* parent;
} rbt_node_t;

typedef int (*rbt_compare_fn)(const void* k1, const void* k2);

typedef struct
{
  rbt_node_t* root;
  mutex_t mtx;
  rbt_compare_fn cmp;
} rbtree_t;

void rbt_node_init(rbt_node_t* n, void* key, void* data, size_t data_size);
void rbtree_init(rbtree_t* t, rbt_compare_fn cmp);
void rbtree_insert(rbtree_t* t, rbt_node_t* n);
void rbtree_delete(rbtree_t* t, rbt_node_t* n);
rbt_node_t* rbtree_search(rbtree_t* t, const void* key);
uint32_t rbtree_size(rbtree_t* t);

typedef error_code_t (*rbt_each_fn)(rbt_node_t* node, void* user);
error_code_t rbtree_foreach(rbtree_t* t, rbt_each_fn fn, void* user);

