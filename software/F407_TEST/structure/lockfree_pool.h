#pragma once

#include "c_base.h"
#include <stdalign.h>

typedef enum
{
  SLOT_FREE = 0,
  SLOT_BUSY = 1,
  SLOT_READY = 2,
  SLOT_RECYCLE = UINT32_MAX
} slot_state_t;

typedef struct
{
  atomic_uint state;
} pool_slot_state_t;

typedef struct
{
  uint32_t slot_count;
  size_t data_size;
  pool_slot_state_t* states;
  uint8_t* data;
} lockfree_pool_t;

error_code_t lockfree_pool_init(lockfree_pool_t* p, uint32_t slot_count, size_t data_size);
void lockfree_pool_deinit(lockfree_pool_t* p);
error_code_t lockfree_pool_put(lockfree_pool_t* p, const void* data, uint32_t* start_index);
error_code_t lockfree_pool_put_to_slot(lockfree_pool_t* p, const void* data, uint32_t index);
error_code_t lockfree_pool_get(lockfree_pool_t* p, void* out, uint32_t* start_index);
error_code_t lockfree_pool_get_from_slot(lockfree_pool_t* p, void* out, uint32_t index);
error_code_t lockfree_pool_recycle_slot(lockfree_pool_t* p, uint32_t index);
size_t lockfree_pool_size(lockfree_pool_t* p);
size_t lockfree_pool_empty(lockfree_pool_t* p);
uint32_t lockfree_pool_slot_count(const lockfree_pool_t* p);

