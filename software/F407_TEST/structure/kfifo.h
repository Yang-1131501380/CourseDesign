#ifndef __KFIFO_H__
#define __KFIFO_H__

#include <stdint.h>
#include <string.h>

#ifndef KFIFO_USE_ATOMIC
#define KFIFO_USE_ATOMIC 1
#endif

#if (KFIFO_USE_ATOMIC != 0)
#include "rtatomic.h"
#endif

typedef struct {
    uint8_t *buffer;
#if (KFIFO_USE_ATOMIC != 0)
    rt_atomic_t *seq_buffer;
#endif
    uint32_t size;
#if (KFIFO_USE_ATOMIC != 0)
    rt_atomic_t in;
    rt_atomic_t out;
#else
    uint32_t in;
    uint32_t out;
#endif
} kfifo_t;

void kfifo_init(kfifo_t *fifo, uint8_t *buffer, void *seq_buffer, uint32_t size);
uint32_t kfifo_len(kfifo_t *fifo);
uint32_t kfifo_avail(kfifo_t *fifo);
uint32_t kfifo_in(kfifo_t *fifo, const uint8_t *buffer, uint32_t len);
uint32_t kfifo_out(kfifo_t *fifo, uint8_t *buffer, uint32_t len);
uint8_t kfifo_is_empty(kfifo_t *fifo);

uint32_t kfifo_in_nolock(kfifo_t *fifo, const uint8_t *buffer, uint32_t len);
uint32_t kfifo_out_nolock(kfifo_t *fifo, uint8_t *buffer, uint32_t len);
uint32_t kfifo_len_nolock(kfifo_t *fifo);
uint32_t kfifo_avail_nolock(kfifo_t *fifo);
uint8_t kfifo_is_empty_nolock(kfifo_t *fifo);

#endif
