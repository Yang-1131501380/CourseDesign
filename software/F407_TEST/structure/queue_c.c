#include "queue_c.h"

error_code_t queue_init(queue_t* q, uint16_t elem_size, size_t length, uint8_t* buffer)
{
  q->elem_size = elem_size;
  q->length = length;
  q->head = q->tail = 0;
  q->is_full = false;
  q->own = buffer == NULL;
  if (buffer)
  {
    q->buf = buffer;
    return ERR_OK;
  }
  q->buf = (uint8_t*)malloc(elem_size * length);
  return q->buf ? ERR_OK : ERR_NO_MEM;
}

void queue_deinit(queue_t* q)
{
  if (q->own && q->buf)
  {
    free(q->buf);
  }
}

void* queue_at(queue_t* q, uint32_t index)
{
  return q->buf + ((size_t)index * q->elem_size);
}

error_code_t queue_push(queue_t* q, const void* data)
{
  if (q->is_full)
  {
    return ERR_FULL;
  }
  fast_copy(q->buf + q->tail * q->elem_size, data, q->elem_size);
  q->tail = (q->tail + 1) % q->length;
  if (q->tail == q->head)
  {
    q->is_full = true;
  }
  return ERR_OK;
}

error_code_t queue_peek(queue_t* q, void* data)
{
  if (queue_size(q) == 0)
  {
    return ERR_EMPTY;
  }
  fast_copy(data, q->buf + q->head * q->elem_size, q->elem_size);
  return ERR_OK;
}

error_code_t queue_pop(queue_t* q, void* data)
{
  if (queue_size(q) == 0)
  {
    return ERR_EMPTY;
  }
  if (data)
  {
    fast_copy(data, q->buf + q->head * q->elem_size, q->elem_size);
  }
  q->head = (q->head + 1) % q->length;
  q->is_full = false;
  return ERR_OK;
}

int queue_last_index(const queue_t* q)
{
  if (queue_size(q) == 0)
  {
    return -1;
  }
  return (int)((q->tail + q->length - 1) % q->length);
}

int queue_first_index(const queue_t* q)
{
  if (queue_size(q) == 0)
  {
    return -1;
  }
  return (int)q->head;
}

error_code_t queue_push_batch(queue_t* q, const void* data, size_t count)
{
  if (queue_empty_size(q) < count)
  {
    return ERR_FULL;
  }
  const uint8_t* src = (const uint8_t*)data;
  size_t first = min_size_t(count, q->length - q->tail);
  fast_copy(q->buf + q->tail * q->elem_size, src, first * q->elem_size);
  if (count > first)
  {
    fast_copy(q->buf, src + first * q->elem_size, (count - first) * q->elem_size);
  }
  q->tail = (q->tail + count) % q->length;
  if (q->tail == q->head)
  {
    q->is_full = true;
  }
  return ERR_OK;
}

error_code_t queue_pop_batch(queue_t* q, void* data, size_t count)
{
  if (queue_size(q) < count)
  {
    return ERR_EMPTY;
  }
  if (count == 0)
  {
    return ERR_OK;
  }

  uint8_t* dst = (uint8_t*)data;
  size_t first = min_size_t(count, q->length - q->head);
  if (dst)
  {
    fast_copy(dst, q->buf + q->head * q->elem_size, first * q->elem_size);
    if (count > first)
    {
      fast_copy(dst + first * q->elem_size, q->buf, (count - first) * q->elem_size);
    }
  }
  q->head = (q->head + count) % q->length;
  q->is_full = false;
  return ERR_OK;
}

error_code_t queue_peek_batch(queue_t* q, void* data, size_t count)
{
  if (queue_size(q) < count)
  {
    return ERR_EMPTY;
  }
  uint8_t* dst = (uint8_t*)data;
  size_t first = min_size_t(count, q->length - q->head);
  fast_copy(dst, q->buf + q->head * q->elem_size, first * q->elem_size);
  if (count > first)
  {
    fast_copy(dst + first * q->elem_size, q->buf, (count - first) * q->elem_size);
  }
  return ERR_OK;
}

error_code_t queue_overwrite(queue_t* q, const void* data)
{
  fast_copy(q->buf, data, q->elem_size * q->length);
  q->head = 0;
  q->tail = 1 % q->length;
  q->is_full = (q->length == 1);
  return ERR_OK;
}

void queue_reset(queue_t* q)
{
  q->head = q->tail = 0;
  q->is_full = false;
}

size_t queue_size(const queue_t* q)
{
  if (q->is_full)
  {
    return q->length;
  }
  if (q->tail >= q->head)
  {
    return q->tail - q->head;
  }
  return q->length + q->tail - q->head;
}

size_t queue_empty_size(const queue_t* q) { return q->length - queue_size(q); }
size_t queue_max_size(const queue_t* q) { return q->length; }

