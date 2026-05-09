#include "kfifo.h"

static inline uint8_t is_power_of_2(uint32_t n)
{
    return (uint8_t)((n != 0U) && ((n & (n - 1U)) == 0U));
}

static inline uint32_t roundup_pow_of_two(uint32_t n)
{
    if (n == 0U) {
        return 1U;
    }

    n--;
    n |= n >> 1U;
    n |= n >> 2U;
    n |= n >> 4U;
    n |= n >> 8U;
    n |= n >> 16U;
    n++;
    return n;
}

void kfifo_init(kfifo_t *fifo, uint8_t *buffer, void *seq_buffer, uint32_t size)
{
#if (KFIFO_USE_ATOMIC != 0)
    uint32_t i;
#endif

    if (!is_power_of_2(size)) {
        size = roundup_pow_of_two(size);
    }

    fifo->buffer = buffer;
    fifo->size = size;
#if (KFIFO_USE_ATOMIC != 0)
    fifo->seq_buffer = (rt_atomic_t *)seq_buffer;
    rt_atomic_store(&fifo->in, 0U);
    rt_atomic_store(&fifo->out, 0U);

    for (i = 0U; i < size; ++i) {
        rt_atomic_store(&fifo->seq_buffer[i], i);
    }
#else
    (void)seq_buffer;
    fifo->in = 0U;
    fifo->out = 0U;
#endif
}

uint32_t kfifo_len(kfifo_t *fifo)
{
#if (KFIFO_USE_ATOMIC != 0)
    return rt_atomic_load(&fifo->in) - rt_atomic_load(&fifo->out);
#else
    return fifo->in - fifo->out;
#endif
}

uint32_t kfifo_avail(kfifo_t *fifo)
{
    return fifo->size - kfifo_len(fifo);
}

uint32_t kfifo_in(kfifo_t *fifo, const uint8_t *buffer, uint32_t len)
{
#if (KFIFO_USE_ATOMIC != 0)
    uint32_t written = 0U;

    while (written < len) {
        uint32_t pos = rt_atomic_load(&fifo->in);
        uint32_t index = pos & (fifo->size - 1U);
        uint32_t seq = rt_atomic_load(&fifo->seq_buffer[index]);
        int32_t dif = (int32_t)(seq - pos);
        rt_atomic_t expected = (rt_atomic_t)pos;

        if (dif == 0) {
            if (rt_atomic_compare_exchange_strong(&fifo->in, &expected, pos + 1U) == 0U) {
                continue;
            }

            fifo->buffer[index] = buffer[written];
            rt_atomic_store(&fifo->seq_buffer[index], pos + 1U);
            written++;
            continue;
        }

        if (dif < 0) {
            break;
        }
    }

    return written;
#else
    uint32_t written;
    uint32_t index;

    written = kfifo_avail(fifo);
    if (written > len) {
        written = len;
    }

    index = fifo->in & (fifo->size - 1U);
    len = fifo->size - index;
    if (len > written) {
        len = written;
    }

    (void)memcpy(&fifo->buffer[index], buffer, len);
    (void)memcpy(fifo->buffer, &buffer[len], written - len);
    fifo->in += written;

    return written;
#endif
}

uint32_t kfifo_out(kfifo_t *fifo, uint8_t *buffer, uint32_t len)
{
#if (KFIFO_USE_ATOMIC != 0)
    uint32_t read = 0U;

    while (read < len) {
        uint32_t pos = rt_atomic_load(&fifo->out);
        uint32_t index = pos & (fifo->size - 1U);
        uint32_t seq = rt_atomic_load(&fifo->seq_buffer[index]);
        int32_t dif = (int32_t)(seq - (pos + 1U));
        rt_atomic_t expected = (rt_atomic_t)pos;

        if (dif == 0) {
            if (rt_atomic_compare_exchange_strong(&fifo->out, &expected, pos + 1U) == 0U) {
                continue;
            }

            buffer[read] = fifo->buffer[index];
            rt_atomic_store(&fifo->seq_buffer[index], pos + fifo->size);
            read++;
            continue;
        }

        if (dif < 0) {
            break;
        }
    }

    return read;
#else
    uint32_t read;
    uint32_t index;

    read = kfifo_len(fifo);
    if (read > len) {
        read = len;
    }

    index = fifo->out & (fifo->size - 1U);
    len = fifo->size - index;
    if (len > read) {
        len = read;
    }

    (void)memcpy(buffer, &fifo->buffer[index], len);
    (void)memcpy(&buffer[len], fifo->buffer, read - len);
    fifo->out += read;

    return read;
#endif
}

uint8_t kfifo_is_empty(kfifo_t *fifo)
{
    return (uint8_t)(kfifo_len(fifo) == 0U);
}

uint32_t kfifo_len_nolock(kfifo_t *fifo)
{
    return kfifo_len(fifo);
}

uint32_t kfifo_avail_nolock(kfifo_t *fifo)
{
    return kfifo_avail(fifo);
}

uint32_t kfifo_in_nolock(kfifo_t *fifo, const uint8_t *buffer, uint32_t len)
{
    return kfifo_in(fifo, buffer, len);
}

uint32_t kfifo_out_nolock(kfifo_t *fifo, uint8_t *buffer, uint32_t len)
{
    return kfifo_out(fifo, buffer, len);
}

uint8_t kfifo_is_empty_nolock(kfifo_t *fifo)
{
    return kfifo_is_empty(fifo);
}
