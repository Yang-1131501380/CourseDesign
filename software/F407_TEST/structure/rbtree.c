#include "rbtree.h"

static void left_rotate(rbtree_t* t, rbt_node_t* x)
{
  rbt_node_t* y = x->right;
  x->right = y->left;
  if (y->left)
  {
    y->left->parent = x;
  }
  y->parent = x->parent;
  if (!x->parent)
  {
    t->root = y;
  }
  else if (x == x->parent->left)
  {
    x->parent->left = y;
  }
  else
  {
    x->parent->right = y;
  }
  y->left = x;
  x->parent = y;
}

static void right_rotate(rbtree_t* t, rbt_node_t* y)
{
  rbt_node_t* x = y->left;
  y->left = x->right;
  if (x->right)
  {
    x->right->parent = y;
  }
  x->parent = y->parent;
  if (!y->parent)
  {
    t->root = x;
  }
  else if (y == y->parent->right)
  {
    y->parent->right = x;
  }
  else
  {
    y->parent->left = x;
  }
  x->right = y;
  y->parent = x;
}

static void insert_fix(rbtree_t* t, rbt_node_t* z)
{
  while (z->parent && z->parent->color == RBT_RED)
  {
    rbt_node_t* gp = z->parent->parent;
    if (z->parent == gp->left)
    {
      rbt_node_t* y = gp->right;
      if (y && y->color == RBT_RED)
      {
        z->parent->color = RBT_BLACK;
        y->color = RBT_BLACK;
        gp->color = RBT_RED;
        z = gp;
      }
      else
      {
        if (z == z->parent->right)
        {
          z = z->parent;
          left_rotate(t, z);
        }
        z->parent->color = RBT_BLACK;
        gp->color = RBT_RED;
        right_rotate(t, gp);
      }
    }
    else
    {
      rbt_node_t* y = gp->left;
      if (y && y->color == RBT_RED)
      {
        z->parent->color = RBT_BLACK;
        y->color = RBT_BLACK;
        gp->color = RBT_RED;
        z = gp;
      }
      else
      {
        if (z == z->parent->left)
        {
          z = z->parent;
          right_rotate(t, z);
        }
        z->parent->color = RBT_BLACK;
        gp->color = RBT_RED;
        left_rotate(t, gp);
      }
    }
  }
  t->root->color = RBT_BLACK;
}

static void transplant(rbtree_t* t, rbt_node_t* u, rbt_node_t* v)
{
  if (!u->parent)
  {
    t->root = v;
  }
  else if (u == u->parent->left)
  {
    u->parent->left = v;
  }
  else
  {
    u->parent->right = v;
  }
  if (v)
  {
    v->parent = u->parent;
  }
}

static rbt_node_t* tree_min(rbt_node_t* x)
{
  while (x && x->left)
  {
    x = x->left;
  }
  return x;
}

static void delete_fix(rbtree_t* t, rbt_node_t* x, rbt_node_t* parent)
{
  while ((x == NULL || x->color == RBT_BLACK) && x != t->root)
  {
    if (x == parent->left)
    {
      rbt_node_t* w = parent->right;
      if (w->color == RBT_RED)
      {
        w->color = RBT_BLACK;
        parent->color = RBT_RED;
        left_rotate(t, parent);
        w = parent->right;
      }
      if ((!w->left || w->left->color == RBT_BLACK) &&
          (!w->right || w->right->color == RBT_BLACK))
      {
        w->color = RBT_RED;
        x = parent;
        parent = x->parent;
      }
      else
      {
        if (!w->right || w->right->color == RBT_BLACK)
        {
          if (w->left)
          {
            w->left->color = RBT_BLACK;
          }
          w->color = RBT_RED;
          right_rotate(t, w);
          w = parent->right;
        }
        w->color = parent->color;
        parent->color = RBT_BLACK;
        if (w->right)
        {
          w->right->color = RBT_BLACK;
        }
        left_rotate(t, parent);
        x = t->root;
        break;
      }
    }
    else
    {
      rbt_node_t* w = parent->left;
      if (w->color == RBT_RED)
      {
        w->color = RBT_BLACK;
        parent->color = RBT_RED;
        right_rotate(t, parent);
        w = parent->left;
      }
      if ((!w->left || w->left->color == RBT_BLACK) &&
          (!w->right || w->right->color == RBT_BLACK))
      {
        w->color = RBT_RED;
        x = parent;
        parent = x->parent;
      }
      else
      {
        if (!w->left || w->left->color == RBT_BLACK)
        {
          if (w->right)
          {
            w->right->color = RBT_BLACK;
          }
          w->color = RBT_RED;
          left_rotate(t, w);
          w = parent->left;
        }
        w->color = parent->color;
        parent->color = RBT_BLACK;
        if (w->left)
        {
          w->left->color = RBT_BLACK;
        }
        right_rotate(t, parent);
        x = t->root;
        break;
      }
    }
  }
  if (x)
  {
    x->color = RBT_BLACK;
  }
}

void rbt_node_init(rbt_node_t* n, void* key, void* data, size_t data_size)
{
  n->key = key;
  n->data = data;
  n->data_size = data_size;
  n->color = RBT_RED;
  n->left = n->right = n->parent = NULL;
}

void rbtree_init(rbtree_t* t, rbt_compare_fn cmp)
{
  t->root = NULL;
  t->cmp = cmp;
  mutex_init(&t->mtx);
}

void rbtree_insert(rbtree_t* t, rbt_node_t* n)
{
  mutex_lock(&t->mtx);
  rbt_node_t* y = NULL;
  rbt_node_t* x = t->root;
  while (x)
  {
    y = x;
    x = (t->cmp(n->key, x->key) < 0) ? x->left : x->right;
  }
  n->parent = y;
  if (!y)
  {
    t->root = n;
  }
  else if (t->cmp(n->key, y->key) < 0)
  {
    y->left = n;
  }
  else
  {
    y->right = n;
  }
  n->color = RBT_RED;
  insert_fix(t, n);
  mutex_unlock(&t->mtx);
}

void rbtree_delete(rbtree_t* t, rbt_node_t* z)
{
  mutex_lock(&t->mtx);
  rbt_node_t* y = z;
  rbt_color_t y_color = y->color;
  rbt_node_t *x = NULL, *x_parent = NULL;
  if (!z->left)
  {
    x = z->right;
    x_parent = z->parent;
    transplant(t, z, z->right);
  }
  else if (!z->right)
  {
    x = z->left;
    x_parent = z->parent;
    transplant(t, z, z->left);
  }
  else
  {
    y = tree_min(z->right);
    y_color = y->color;
    x = y->right;
    x_parent = (y->parent == z) ? y : y->parent;
    if (y->parent != z)
    {
      transplant(t, y, y->right);
      y->right = z->right;
      if (y->right)
      {
        y->right->parent = y;
      }
    }
    transplant(t, z, y);
    y->left = z->left;
    if (y->left)
    {
      y->left->parent = y;
    }
    y->color = z->color;
  }
  if (y_color == RBT_BLACK)
  {
    delete_fix(t, x, x_parent);
  }
  mutex_unlock(&t->mtx);
}

rbt_node_t* rbtree_search(rbtree_t* t, const void* key)
{
  mutex_lock(&t->mtx);
  rbt_node_t* x = t->root;
  while (x)
  {
    int cmp = t->cmp(key, x->key);
    if (cmp == 0)
    {
      break;
    }
    x = (cmp < 0) ? x->left : x->right;
  }
  mutex_unlock(&t->mtx);
  return x;
}

static uint32_t count_rec(rbt_node_t* n)
{
  if (!n)
  {
    return 0;
  }
  return 1 + count_rec(n->left) + count_rec(n->right);
}

uint32_t rbtree_size(rbtree_t* t)
{
  mutex_lock(&t->mtx);
  uint32_t cnt = count_rec(t->root);
  mutex_unlock(&t->mtx);
  return cnt;
}

static error_code_t foreach_rec(rbt_node_t* n, rbt_each_fn fn, void* user)
{
  if (!n)
  {
    return ERR_OK;
  }
  if (foreach_rec(n->left, fn, user) != ERR_OK)
  {
    return ERR_FAILED;
  }
  if (fn && fn(n, user) != ERR_OK)
  {
    return ERR_FAILED;
  }
  return foreach_rec(n->right, fn, user);
}

error_code_t rbtree_foreach(rbtree_t* t, rbt_each_fn fn, void* user)
{
  mutex_lock(&t->mtx);
  error_code_t r = foreach_rec(t->root, fn, user);
  mutex_unlock(&t->mtx);
  return r;
}

