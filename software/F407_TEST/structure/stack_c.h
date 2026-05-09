#pragma once

#include "c_base.h"

typedef struct
{
  void* buf;
  uint32_t top;
  uint32_t depth;
  size_t elem_size;
  mutex_t mtx;
} stack_t;

error_code_t stack_init(stack_t* s, size_t elem_size, uint32_t depth);
void stack_deinit(stack_t* s);
uint32_t stack_size(const stack_t* s);
uint32_t stack_empty(const stack_t* s);
error_code_t stack_push(stack_t* s, const void* data);
error_code_t stack_pop(stack_t* s, void* out);
error_code_t stack_peek(stack_t* s, void* out);
error_code_t stack_insert(stack_t* s, const void* data, uint32_t index);
error_code_t stack_delete(stack_t* s, uint32_t index);
void stack_reset(stack_t* s);

