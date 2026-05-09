#include "stack_c.h"

error_code_t stack_init(stack_t* s, size_t elem_size, uint32_t depth)
{
  s->buf = malloc(elem_size * depth);
  if (!s->buf)
  {
    return ERR_NO_MEM;
  }
  s->elem_size = elem_size;
  s->depth = depth;
  s->top = 0;
  mutex_init(&s->mtx);
  return ERR_OK;
}

void stack_deinit(stack_t* s)
{
  free(s->buf);
}

uint32_t stack_size(const stack_t* s) { return s->top; }
uint32_t stack_empty(const stack_t* s) { return s->depth - s->top; }

error_code_t stack_push(stack_t* s, const void* data)
{
  mutex_lock(&s->mtx);
  if (s->top >= s->depth)
  {
    mutex_unlock(&s->mtx);
    return ERR_FULL;
  }
  fast_copy((uint8_t*)s->buf + s->top * s->elem_size, data, s->elem_size);
  s->top++;
  mutex_unlock(&s->mtx);
  return ERR_OK;
}

error_code_t stack_pop(stack_t* s, void* out)
{
  mutex_lock(&s->mtx);
  if (s->top == 0)
  {
    mutex_unlock(&s->mtx);
    return ERR_EMPTY;
  }
  s->top--;
  if (out)
  {
    fast_copy(out, (uint8_t*)s->buf + s->top * s->elem_size, s->elem_size);
  }
  mutex_unlock(&s->mtx);
  return ERR_OK;
}

error_code_t stack_peek(stack_t* s, void* out)
{
  mutex_lock(&s->mtx);
  if (s->top == 0)
  {
    mutex_unlock(&s->mtx);
    return ERR_EMPTY;
  }
  fast_copy(out, (uint8_t*)s->buf + (s->top - 1) * s->elem_size, s->elem_size);
  mutex_unlock(&s->mtx);
  return ERR_OK;
}

error_code_t stack_insert(stack_t* s, const void* data, uint32_t index)
{
  mutex_lock(&s->mtx);
  if (s->top >= s->depth)
  {
    mutex_unlock(&s->mtx);
    return ERR_FULL;
  }
  if (index > s->top)
  {
    mutex_unlock(&s->mtx);
    return ERR_OUT_OF_RANGE;
  }
  memmove((uint8_t*)s->buf + (index + 1) * s->elem_size,
          (uint8_t*)s->buf + index * s->elem_size,
          (s->top - index) * s->elem_size);
  fast_copy((uint8_t*)s->buf + index * s->elem_size, data, s->elem_size);
  s->top++;
  mutex_unlock(&s->mtx);
  return ERR_OK;
}

error_code_t stack_delete(stack_t* s, uint32_t index)
{
  mutex_lock(&s->mtx);
  if (index >= s->top)
  {
    mutex_unlock(&s->mtx);
    return ERR_OUT_OF_RANGE;
  }
  memmove((uint8_t*)s->buf + index * s->elem_size,
          (uint8_t*)s->buf + (index + 1) * s->elem_size,
          (s->top - index - 1) * s->elem_size);
  s->top--;
  mutex_unlock(&s->mtx);
  return ERR_OK;
}

void stack_reset(stack_t* s)
{
  mutex_lock(&s->mtx);
  s->top = 0;
  mutex_unlock(&s->mtx);
}

