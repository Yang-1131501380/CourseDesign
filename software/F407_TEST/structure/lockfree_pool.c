#include "lockfree_pool.h"

error_code_t lockfree_pool_init(lockfree_pool_t* p, uint32_t slot_count, size_t data_size)
{
  p->slot_count = slot_count;
  p->data_size = data_size;
  p->states = (pool_slot_state_t*)aligned_alloc(alignof(max_align_t),
                                                sizeof(pool_slot_state_t) * slot_count);
  p->data = (uint8_t*)aligned_alloc(alignof(max_align_t), data_size * slot_count);
  if (!p->states || !p->data)
  {
    free(p->states);
    free(p->data);
    return ERR_NO_MEM;
  }
  for (uint32_t i = 0; i < slot_count; ++i)
  {
    atomic_store_explicit(&p->states[i].state, SLOT_FREE, memory_order_relaxed);
  }
  return ERR_OK;
}

void lockfree_pool_deinit(lockfree_pool_t* p)
{
  free(p->states);
  free(p->data);
}

error_code_t lockfree_pool_put(lockfree_pool_t* p, const void* data, uint32_t* start_index)
{
  uint32_t start = start_index ? *start_index : 0;
  for (uint32_t i = start; i < p->slot_count; ++i)
  {
    unsigned int cur = atomic_load_explicit(&p->states[i].state, memory_order_relaxed);
    if (cur == SLOT_FREE || cur == SLOT_RECYCLE)
    {
      if (atomic_compare_exchange_strong_explicit(&p->states[i].state, &cur, SLOT_BUSY,
                                                  memory_order_release, memory_order_relaxed))
      {
        fast_copy(&p->data[i * p->data_size], data, p->data_size);
        atomic_store_explicit(&p->states[i].state, SLOT_READY, memory_order_release);
        if (start_index)
        {
          *start_index = i;
        }
        return ERR_OK;
      }
    }
  }
  return ERR_FULL;
}

error_code_t lockfree_pool_put_to_slot(lockfree_pool_t* p, const void* data, uint32_t index)
{
  unsigned int cur = atomic_load_explicit(&p->states[index].state, memory_order_relaxed);
  if (cur == SLOT_FREE || cur == SLOT_RECYCLE)
  {
    if (atomic_compare_exchange_strong_explicit(&p->states[index].state, &cur, SLOT_BUSY,
                                                memory_order_release, memory_order_relaxed))
    {
      fast_copy(&p->data[index * p->data_size], data, p->data_size);
      atomic_store_explicit(&p->states[index].state, SLOT_READY, memory_order_release);
      return ERR_OK;
    }
  }
  return ERR_FULL;
}

error_code_t lockfree_pool_get(lockfree_pool_t* p, void* out, uint32_t* start_index)
{
  uint32_t start = start_index ? *start_index : 0;
  for (uint32_t i = start; i < p->slot_count; ++i)
  {
    unsigned int cur = atomic_load_explicit(&p->states[i].state, memory_order_acquire);
    if (cur == SLOT_READY)
    {
      if (atomic_compare_exchange_strong_explicit(&p->states[i].state, &cur, SLOT_BUSY,
                                                  memory_order_acquire, memory_order_relaxed))
      {
        fast_copy(out, &p->data[i * p->data_size], p->data_size);
        atomic_store_explicit(&p->states[i].state, SLOT_RECYCLE, memory_order_release);
        if (start_index)
        {
          *start_index = i;
        }
        return ERR_OK;
      }
    }
    if (cur == SLOT_FREE)
    {
      if (start_index)
      {
        *start_index = 0;
      }
      return ERR_EMPTY;
    }
  }
  if (start_index)
  {
    *start_index = 0;
  }
  return ERR_EMPTY;
}

error_code_t lockfree_pool_get_from_slot(lockfree_pool_t* p, void* out, uint32_t index)
{
  unsigned int cur = atomic_load_explicit(&p->states[index].state, memory_order_acquire);
  if (cur == SLOT_READY)
  {
    if (atomic_compare_exchange_strong_explicit(&p->states[index].state, &cur, SLOT_BUSY,
                                                memory_order_acquire, memory_order_relaxed))
    {
      fast_copy(out, &p->data[index * p->data_size], p->data_size);
      atomic_store_explicit(&p->states[index].state, SLOT_RECYCLE, memory_order_release);
      return ERR_OK;
    }
  }
  return ERR_EMPTY;
}

error_code_t lockfree_pool_recycle_slot(lockfree_pool_t* p, uint32_t index)
{
  unsigned int cur = atomic_load_explicit(&p->states[index].state, memory_order_relaxed);
  if (cur == SLOT_READY)
  {
    if (atomic_compare_exchange_strong_explicit(&p->states[index].state, &cur, SLOT_RECYCLE,
                                                memory_order_release, memory_order_relaxed))
    {
      return ERR_OK;
    }
  }
  return ERR_EMPTY;
}

size_t lockfree_pool_size(lockfree_pool_t* p)
{
  size_t cnt = 0;
  for (uint32_t i = 0; i < p->slot_count; ++i)
  {
    if (atomic_load_explicit(&p->states[i].state, memory_order_relaxed) == SLOT_READY)
    {
      ++cnt;
    }
  }
  return cnt;
}

size_t lockfree_pool_empty(lockfree_pool_t* p)
{
  size_t cnt = 0;
  for (uint32_t i = 0; i < p->slot_count; ++i)
  {
    unsigned int s = atomic_load_explicit(&p->states[i].state, memory_order_relaxed);
    if (s == SLOT_FREE || s == SLOT_RECYCLE)
    {
      ++cnt;
    }
  }
  return cnt;
}

uint32_t lockfree_pool_slot_count(const lockfree_pool_t* p) { return p->slot_count; }

