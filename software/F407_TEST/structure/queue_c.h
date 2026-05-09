#pragma once

#include "c_base.h"

typedef struct
{
  uint8_t* buf;
  uint16_t elem_size;
  size_t length;
  size_t head;
  size_t tail;
  bool is_full;
  bool own;
} queue_t;

error_code_t queue_init(queue_t* q, uint16_t elem_size, size_t length, uint8_t* buffer);
void queue_deinit(queue_t* q);
void* queue_at(queue_t* q, uint32_t index);
error_code_t queue_push(queue_t* q, const void* data);
error_code_t queue_peek(queue_t* q, void* data);
error_code_t queue_pop(queue_t* q, void* data);
int queue_last_index(const queue_t* q);
int queue_first_index(const queue_t* q);
error_code_t queue_push_batch(queue_t* q, const void* data, size_t count);
error_code_t queue_pop_batch(queue_t* q, void* data, size_t count);
error_code_t queue_peek_batch(queue_t* q, void* data, size_t count);
error_code_t queue_overwrite(queue_t* q, const void* data);
void queue_reset(queue_t* q);
size_t queue_size(const queue_t* q);
size_t queue_empty_size(const queue_t* q);
size_t queue_max_size(const queue_t* q);

