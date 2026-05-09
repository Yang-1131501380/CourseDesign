/**
 * @file fast_math.h
 * @brief 快速数学函数实现，包括平方根倒数、平方根、反正切和反正弦函数。
 */

#include "fast_math.h"

/**
 * @union fast_math_float_bits_t
 * @brief 用于浮点数到位表示的类型转换联合体。
 *
 * 该联合体允许将浮点数与其位表示进行双向转换，以便于位操作。
 */
typedef union {
    float value;       ///< 浮点数值。
    uint32_t bits;     ///< 浮点数对应的位表示。
} fast_math_float_bits_t;

/**
 * @brief 计算浮点数的平方根倒数。
 *
 * 通过牛顿迭代法计算给定浮点数的平方根倒数。
 *
 * @param x 输入的浮点数。
 * @return 输入浮点数的平方根倒数。
 */
float fast_math_inv_sqrtf(float x)
{
    fast_math_float_bits_t conv;
    float halfx;

    if (x <= 0.0f) {
        return 0.0f;
    }

    halfx = 0.5f * x;
    conv.value = x;
    conv.bits = 0x5f3759dfu - (conv.bits >> 1);  // 第一次猜测
    conv.value = conv.value * (1.5f - (halfx * conv.value * conv.value));  // 牛顿迭代法第一次修正
    conv.value = conv.value * (1.5f - (halfx * conv.value * conv.value));  // 牛顿迭代法第二次修正

    return conv.value;
}

/**
 * @brief 计算浮点数的平方根。
 *
 * 利用 fast_math_inv_sqrtf 函数计算平方根倒数，然后取其倒数得到平方根。
 *
 * @param x 输入的浮点数。
 * @return 输入浮点数的平方根。
 */
float fast_math_sqrtf(float x)
{
    if (x <= 0.0f) {
        return 0.0f;
    }
    return x * fast_math_inv_sqrtf(x);
}

/**
 * @brief 计算浮点数的反正切值。
 *
 * 使用泰勒级数展开近似计算反正切值。
 *
 * @param x 输入的浮点数。
 * @return 输入浮点数的反正切值。
 */
float fast_math_atanf(float x)
{
    float abs_x = fast_math_absf(x);  // 计算输入值的绝对值
    float x2;
    float result;

    if (abs_x > 1.0f) {
        abs_x = 1.0f / abs_x;  // 当绝对值大于1时，计算其倒数
        x2 = abs_x * abs_x;    // 计算倒数的平方
        // 泰勒级数展开近似计算反正切
        result = abs_x * (1.0f - x2 * (0.333333333f - x2 * (0.2f - x2 * 0.142857143f)));
        result = FAST_MATH_HALF_PI - result;  // 调整角度范围
    } else {
        x2 = abs_x * abs_x;    // 计算绝对值的平方
        // 泰勒级数展开近似计算反正切
        result = abs_x * (1.0f - x2 * (0.333333333f - x2 * (0.2f - x2 * 0.142857143f)));
    }

    // 返回正确的反正切值，根据输入符号调整正负
    return (x < 0.0f) ? -result : result;
}

/**
 * @brief 计算两个浮点数的反正切值。
 *
 * 利用 fast_math_atanf 函数计算反正切值，考虑输入点 (x, y) 所在象限。
 *
 * @param y 输入的 y 坐标。
 * @param x 输入的 x 坐标。
 * @return 输入点 (x, y) 的反正切值。
 */
float fast_math_atan2f(float y, float x)
{
    float abs_y;
    float angle;

    if ((x == 0.0f) && (y == 0.0f)) {
        return 0.0f;  // 如果 (x, y) 是原点，反正切为0
    }

    if (x == 0.0f) {
        // x=0时，角度为 ±π/2
        return (y > 0.0f) ? FAST_MATH_HALF_PI : -FAST_MATH_HALF_PI;
    }

    abs_y = fast_math_absf(y);  // 计算 y 的绝对值
    if (abs_y < 1.0e-6f) {      // 如果 y 非常接近0
        if (x > 0.0f) {
            return 0.0f;  // x>0时，反正切为0
        }
        // x<0时，角度为 π 或 -π
        return (y < 0.0f) ? -FAST_MATH_PI : FAST_MATH_PI;
    }

    // 计算反正切值，角度范围为 [0, π/2]
    angle = fast_math_atanf(abs_y / fast_math_absf(x));
    if (x < 0.0f) {
        // x<0时，调整角度范围为 [π/2, π] 或 [-π, -π/2]
        angle = (y > 0.0f) ? (FAST_MATH_PI - angle) : (-FAST_MATH_PI + angle);
    } else if (y < 0.0f) {
        // y<0时，角度取负值
        angle = -angle;
    }

    return angle;
}

/**
 * @brief 计算浮点数的反正弦值。
 *
 * 使用泰勒级数展开近似计算反正弦值。
 *
 * @param x 输入的浮点数。
 * @return 输入浮点数的反正弦值。
 */
float fast_math_asinf(float x)
{
    float abs_x = x;
    float result;
    float x2;

    if (x < 0.0f) {
        abs_x = -x;  // 如果输入为负，取其绝对值
    }

    if (abs_x >= 1.0f) {
        // 当绝对值大于等于1时，反正弦为 ±π/2
        return (x < 0.0f) ? -FAST_MATH_HALF_PI : FAST_MATH_HALF_PI;
    }

    if (abs_x <= 0.0f) {
        return 0.0f;  // 当绝对值为0时，反正弦为0
    }

    if (abs_x < 0.1f) {
        x2 = abs_x * abs_x;  // 计算绝对值的平方
        // 泰勒级数展开近似计算反正弦
        result = abs_x * (1.0f + x2 * (0.1666666666666667f +
                                       x2 * (0.0750000000000000f +
                                             x2 * 0.0446428571428571f)));
    } else if (abs_x > 0.5f) {
        float t = (1.0f - abs_x) * 0.5f;  // 计算辅助变量 t
        // 使用递归计算反正弦值
        result = FAST_MATH_HALF_PI - 2.0f * fast_math_asinf(fast_math_sqrtf(t));
    } else {
        x2 = abs_x * abs_x;  // 计算绝对值的平方
        // 泰勒级数展开近似计算反正弦
        result = abs_x * (1.0f + x2 * (0.1666666666666667f +
                                       x2 * (0.0750000000000000f +
                                             x2 * (0.0446428571428571f +
                                                   x2 * 0.0303819444444444f))));
    }

    // 返回正确的反正弦值，根据输入符号调整正负
    return (x < 0.0f) ? -result : result;
}
