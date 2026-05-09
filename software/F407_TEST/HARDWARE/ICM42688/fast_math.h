#ifndef __FAST_MATH_H
#define __FAST_MATH_H

#include <stdint.h>

#define FAST_MATH_PI           3.14159265358979323846f
#define FAST_MATH_HALF_PI      (FAST_MATH_PI * 0.5f)
#define FAST_MATH_DEG_TO_RAD   (FAST_MATH_PI / 180.0f)
#define FAST_MATH_RAD_TO_DEG   (180.0f / FAST_MATH_PI)
#define FAST_MATH_MAGIC_NUMBER 0x5f3759dfu
#define FAST_MATH_HALF         0.5f
#define FAST_MATH_THIRD        0.333333333f
#define FAST_MATH_TWO_THIRDS   0.666666667f
#define FAST_MATH_EPSILON      1.0e-6f

static inline float fast_math_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

static inline float fast_math_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static inline float fast_math_deadzone(float value, float deadzone)
{
    return ((value > -deadzone) && (value < deadzone)) ? 0.0f : value;
}

static inline float fast_math_normalize_deg(float angle)
{
    while (angle > 180.0f) {
        angle -= 360.0f;
    }
    while (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

float fast_math_inv_sqrtf(float x);
float fast_math_sqrtf(float x);
float fast_math_atanf(float x);
float fast_math_atan2f(float y, float x);
float fast_math_asinf(float x);

#endif
