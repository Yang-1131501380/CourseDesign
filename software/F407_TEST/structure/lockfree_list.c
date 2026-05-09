#include "lockfree_list.h"

void lf_list_init(lf_list_t* l)
{
  atomic_store_explicit(&l->head.next, &l->head, memory_order_relaxed);
  l->head.size = 0;
}

void lf_list_add(lf_list_t* l, lf_list_node_t* n)
{
  lf_list_node_t* current;
  do
  {
    current = atomic_load_explicit(&l->head.next, memory_order_acquire);
    atomic_store_explicit(&n->next, current, memory_order_relaxed);
  } while (!atomic_compare_exchange_weak_explicit(&l->head.next, &current, n,
                                                  memory_order_release, memory_order_acquire));
}

uint32_t lf_list_size(lf_list_t* l)
{
  uint32_t cnt = 0;
  for (lf_list_node_t* p =
           atomic_load_explicit(&l->head.next, memory_order_acquire);
       p != &l->head;
       p = atomic_load_explicit(&p->next, memory_order_relaxed))
  {
    ++cnt;
  }
  return cnt;
}

error_code_t lf_list_foreach(lf_list_t* l, lf_list_each_fn fn, void* user)
{
  for (lf_list_node_t* p =
           atomic_load_explicit(&l->head.next, memory_order_acquire);
       p != &l->head;
       p = atomic_load_explicit(&p->next, memory_order_relaxed))
  {
    if (fn)
    {
      error_code_t r = fn(p, user);
      if (r != ERR_OK)
      {
        return r;
      }
    }
  }
  return ERR_OK;
}

