#ifndef __RT_ATOMIC_H__
#define __RT_ATOMIC_H__

#include <stdbool.h>
#include <stdint.h>

typedef int32_t rt_atomic_t;
typedef uint8_t rt_atomic8_t;
typedef uint16_t rt_atomic16_t;

#ifndef __RT_DEF_H__
typedef bool rt_bool_t;
#endif

#ifndef rt_inline
#define rt_inline static inline
#endif

#if defined(__CC_ARM)
#include "stm32f4xx.h"

rt_inline uint32_t rt_atomic_irq_save(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

rt_inline void rt_atomic_irq_restore(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

rt_inline rt_atomic_t rt_hw_atomic_load(volatile rt_atomic_t *ptr)
{
    return *ptr;
}

rt_inline void rt_hw_atomic_store(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    uint32_t level = rt_atomic_irq_save();

    *ptr = val;
    rt_atomic_irq_restore(level);
}

rt_inline rt_atomic8_t rt_hw_atomic_load8(volatile rt_atomic8_t *ptr)
{
    return *ptr;
}

rt_inline void rt_hw_atomic_store8(volatile rt_atomic8_t *ptr, rt_atomic8_t val)
{
    uint32_t level = rt_atomic_irq_save();

    *ptr = val;
    rt_atomic_irq_restore(level);
}

rt_inline rt_atomic16_t rt_hw_atomic_load16(volatile rt_atomic16_t *ptr)
{
    return *ptr;
}

rt_inline void rt_hw_atomic_store16(volatile rt_atomic16_t *ptr, rt_atomic16_t val)
{
    uint32_t level = rt_atomic_irq_save();

    *ptr = val;
    rt_atomic_irq_restore(level);
}

rt_inline rt_atomic_t rt_hw_atomic_add(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic_t old = *ptr;

    *ptr = old + val;
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic_t rt_hw_atomic_sub(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic_t old = *ptr;

    *ptr = old - val;
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic8_t rt_hw_atomic_and8(volatile rt_atomic8_t *ptr, rt_atomic8_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic8_t old = *ptr;

    *ptr = (rt_atomic8_t)(old & val);
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic8_t rt_hw_atomic_or8(volatile rt_atomic8_t *ptr, rt_atomic8_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic8_t old = *ptr;

    *ptr = (rt_atomic8_t)(old | val);
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic16_t rt_hw_atomic_and16(volatile rt_atomic16_t *ptr,
                                           rt_atomic16_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic16_t old = *ptr;

    *ptr = (rt_atomic16_t)(old & val);
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic16_t rt_hw_atomic_or16(volatile rt_atomic16_t *ptr,
                                          rt_atomic16_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic16_t old = *ptr;

    *ptr = (rt_atomic16_t)(old | val);
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic_t rt_hw_atomic_and(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic_t old = *ptr;

    *ptr = old & val;
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic_t rt_hw_atomic_or(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic_t old = *ptr;

    *ptr = old | val;
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic_t rt_hw_atomic_xor(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic_t old = *ptr;

    *ptr = old ^ val;
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline rt_atomic_t rt_hw_atomic_exchange(volatile rt_atomic_t *ptr,
                                            rt_atomic_t val)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic_t old = *ptr;

    *ptr = val;
    rt_atomic_irq_restore(level);
    return old;
}

rt_inline void rt_hw_atomic_flag_clear(volatile rt_atomic_t *ptr)
{
    rt_hw_atomic_store(ptr, 0);
}

rt_inline rt_atomic_t rt_hw_atomic_flag_test_and_set(volatile rt_atomic_t *ptr)
{
    return rt_hw_atomic_exchange(ptr, 1);
}

rt_inline rt_atomic_t rt_hw_atomic_compare_exchange_strong(volatile rt_atomic_t *ptr,
                                                           rt_atomic_t *expected,
                                                           rt_atomic_t desired)
{
    uint32_t level = rt_atomic_irq_save();
    rt_atomic_t matched = 0;

    if (*ptr == *expected) {
        *ptr = desired;
        matched = 1;
    } else {
        *expected = *ptr;
    }
    rt_atomic_irq_restore(level);
    return matched;
}

#else

rt_inline rt_atomic_t rt_hw_atomic_load(volatile rt_atomic_t *ptr)
{
    return __atomic_load_n(ptr, __ATOMIC_ACQUIRE);
}

rt_inline void rt_hw_atomic_store(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    __atomic_store_n(ptr, val, __ATOMIC_RELEASE);
}

rt_inline rt_atomic8_t rt_hw_atomic_load8(volatile rt_atomic8_t *ptr)
{
    return __atomic_load_n(ptr, __ATOMIC_ACQUIRE);
}

rt_inline void rt_hw_atomic_store8(volatile rt_atomic8_t *ptr, rt_atomic8_t val)
{
    __atomic_store_n(ptr, val, __ATOMIC_RELEASE);
}

rt_inline rt_atomic16_t rt_hw_atomic_load16(volatile rt_atomic16_t *ptr)
{
    return __atomic_load_n(ptr, __ATOMIC_ACQUIRE);
}

rt_inline void rt_hw_atomic_store16(volatile rt_atomic16_t *ptr, rt_atomic16_t val)
{
    __atomic_store_n(ptr, val, __ATOMIC_RELEASE);
}

rt_inline rt_atomic_t rt_hw_atomic_add(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    return __atomic_fetch_add(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic_t rt_hw_atomic_sub(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    return __atomic_fetch_sub(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic8_t rt_hw_atomic_and8(volatile rt_atomic8_t *ptr, rt_atomic8_t val)
{
    return __atomic_fetch_and(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic8_t rt_hw_atomic_or8(volatile rt_atomic8_t *ptr, rt_atomic8_t val)
{
    return __atomic_fetch_or(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic16_t rt_hw_atomic_and16(volatile rt_atomic16_t *ptr, rt_atomic16_t val)
{
    return __atomic_fetch_and(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic16_t rt_hw_atomic_or16(volatile rt_atomic16_t *ptr, rt_atomic16_t val)
{
    return __atomic_fetch_or(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic_t rt_hw_atomic_and(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    return __atomic_fetch_and(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic_t rt_hw_atomic_or(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    return __atomic_fetch_or(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic_t rt_hw_atomic_xor(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    return __atomic_fetch_xor(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic_t rt_hw_atomic_exchange(volatile rt_atomic_t *ptr, rt_atomic_t val)
{
    return __atomic_exchange_n(ptr, val, __ATOMIC_ACQ_REL);
}

rt_inline void rt_hw_atomic_flag_clear(volatile rt_atomic_t *ptr)
{
    __atomic_store_n(ptr, 0, __ATOMIC_RELEASE);
}

rt_inline rt_atomic_t rt_hw_atomic_flag_test_and_set(volatile rt_atomic_t *ptr)
{
    return __atomic_exchange_n(ptr, 1, __ATOMIC_ACQ_REL);
}

rt_inline rt_atomic_t rt_hw_atomic_compare_exchange_strong(volatile rt_atomic_t *ptr,
                                                           rt_atomic_t *expected,
                                                           rt_atomic_t desired)
{
    return (rt_atomic_t)__atomic_compare_exchange_n(ptr, expected, desired, 0,
                                                    __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE);
}

#endif

#define rt_atomic_load(ptr) rt_hw_atomic_load(ptr)
#define rt_atomic_store(ptr, v) rt_hw_atomic_store(ptr, v)
#define rt_atomic_load8(ptr) rt_hw_atomic_load8(ptr)
#define rt_atomic_store8(ptr, v) rt_hw_atomic_store8(ptr, v)
#define rt_atomic_load16(ptr) rt_hw_atomic_load16(ptr)
#define rt_atomic_store16(ptr, v) rt_hw_atomic_store16(ptr, v)
#define rt_atomic_add(ptr, v) rt_hw_atomic_add(ptr, v)
#define rt_atomic_sub(ptr, v) rt_hw_atomic_sub(ptr, v)
#define rt_atomic_and8(ptr, v) rt_hw_atomic_and8(ptr, v)
#define rt_atomic_or8(ptr, v) rt_hw_atomic_or8(ptr, v)
#define rt_atomic_and16(ptr, v) rt_hw_atomic_and16(ptr, v)
#define rt_atomic_or16(ptr, v) rt_hw_atomic_or16(ptr, v)
#define rt_atomic_and(ptr, v) rt_hw_atomic_and(ptr, v)
#define rt_atomic_or(ptr, v) rt_hw_atomic_or(ptr, v)
#define rt_atomic_xor(ptr, v) rt_hw_atomic_xor(ptr, v)
#define rt_atomic_exchange(ptr, v) rt_hw_atomic_exchange(ptr, v)
#define rt_atomic_flag_clear(ptr) rt_hw_atomic_flag_clear(ptr)
#define rt_atomic_flag_test_and_set(ptr) rt_hw_atomic_flag_test_and_set(ptr)
#define rt_atomic_compare_exchange_strong(ptr, v, des) \
    rt_hw_atomic_compare_exchange_strong(ptr, v, des)

rt_inline rt_bool_t rt_atomic_dec_and_test(volatile rt_atomic_t *ptr)
{
    return (rt_bool_t)(rt_atomic_sub(ptr, 1) == 1);
}

rt_inline rt_atomic_t rt_atomic_fetch_add_unless(volatile rt_atomic_t *ptr,
                                                 rt_atomic_t a,
                                                 rt_atomic_t u)
{
    rt_atomic_t c = rt_atomic_load(ptr);

    do {
        if (c == u) {
            break;
        }
    } while (!rt_atomic_compare_exchange_strong(ptr, &c, c + a));

    return c;
}

rt_inline rt_bool_t rt_atomic_add_unless(volatile rt_atomic_t *ptr,
                                         rt_atomic_t a,
                                         rt_atomic_t u)
{
    return (rt_bool_t)(rt_atomic_fetch_add_unless(ptr, a, u) != u);
}

rt_inline rt_bool_t rt_atomic_inc_not_zero(volatile rt_atomic_t *ptr)
{
    return rt_atomic_add_unless(ptr, 1, 0);
}

#endif
