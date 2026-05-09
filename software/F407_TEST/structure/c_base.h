#pragma once

#include <assert.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

/* error codes */
typedef enum
{
  ERR_PENDING = 1,
  ERR_OK = 0,
  ERR_FAILED = -1,
  ERR_INIT = -2,
  ERR_ARG = -3,
  ERR_STATE = -4,
  ERR_SIZE = -5,
  ERR_CHECK = -6,
  ERR_NOT_SUPPORT = -7,
  ERR_NOT_FOUND = -8,
  ERR_NO_RESPONSE = -9,
  ERR_NO_MEM = -10,
  ERR_NO_BUFF = -11,
  ERR_TIMEOUT = -12,
  ERR_EMPTY = -13,
  ERR_FULL = -14,
  ERR_BUSY = -15,
  ERR_PTR_NULL = -16,
  ERR_OUT_OF_RANGE = -17
} error_code_t;

typedef enum
{
  SIZE_EQUAL = 0,
  SIZE_LESS = 1,
  SIZE_MORE = 2,
  SIZE_NONE = 3
} size_limit_mode_t;

typedef struct
{
  void* addr;
  size_t size;
} raw_data_t;

static inline void fast_copy(void* dst, const void* src, size_t len)
{
  memcpy(dst, src, len);
}

static inline size_t max_size_t(size_t a, size_t b) { return (a > b) ? a : b; }
static inline size_t min_size_t(size_t a, size_t b) { return (a < b) ? a : b; }

typedef struct
{
  atomic_flag flag;
} mutex_t;

static inline void mutex_init(mutex_t* m) { atomic_flag_clear(&m->flag); }

static inline void mutex_lock(mutex_t* m)
{
  while (atomic_flag_test_and_set_explicit(&m->flag, memory_order_acquire))
  {
  }
}

static inline void mutex_unlock(mutex_t* m)
{
  atomic_flag_clear_explicit(&m->flag, memory_order_release);
}

#ifndef OFFSET_OF
#define OFFSET_OF(type, member) ((size_t) & (((type*)0)->member))
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member) ((type*)((char*)(ptr)-OFFSET_OF(type, member)))
#endif

