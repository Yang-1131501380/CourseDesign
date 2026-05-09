#include "list.h"

void list_init(list_t* l)
{
  l->head.next = &l->head;
  l->head.size = 0;
  mutex_init(&l->mtx);
}

void list_add(list_t* l, list_node_t* n)
{
  mutex_lock(&l->mtx);
  n->next = l->head.next;
  l->head.next = n;
  mutex_unlock(&l->mtx);
}

uint32_t list_size(list_t* l)
{
  mutex_lock(&l->mtx);
  uint32_t cnt = 0;
  for (list_node_t* p = l->head.next; p != &l->head; p = p->next)
  {
    ++cnt;
  }
  mutex_unlock(&l->mtx);
  return cnt;
}

error_code_t list_delete(list_t* l, list_node_t* n)
{
  mutex_lock(&l->mtx);
  for (list_node_t* p = &l->head; p->next != &l->head; p = p->next)
  {
    if (p->next == n)
    {
      p->next = n->next;
      n->next = NULL;
      mutex_unlock(&l->mtx);
      return ERR_OK;
    }
  }
  mutex_unlock(&l->mtx);
  return ERR_NOT_FOUND;
}

error_code_t list_foreach(list_t* l, list_each_fn fn, void* user)
{
  mutex_lock(&l->mtx);
  for (list_node_t* p = l->head.next; p != &l->head; p = p->next)
  {
    if (fn)
    {
      error_code_t r = fn(p, user);
      if (r != ERR_OK)
      {
        mutex_unlock(&l->mtx);
        return r;
      }
    }
  }
  mutex_unlock(&l->mtx);
  return ERR_OK;
}

