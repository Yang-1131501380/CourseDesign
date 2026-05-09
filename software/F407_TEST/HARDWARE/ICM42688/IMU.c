/**
 * @file IMU.c
 * @brief IMU传感器处理类，包含加速度计、陀螺仪和磁力计数据的读取和校准功能。
 *
 * 该文件定义了一个IMU类，提供了初始化、读取传感器数据、校准传感器以及获取姿态角等功能。
 *
 * @author 你的名字
 * @date 2023-10-01
 */

#include "IMU.h"
#include "icm42688.h"
#include "rtthread.h"


#define KP                      0.5f      ///< 比例增益。
#define KI                      0.001f    ///< 积分增益。
#define GYRO_VARIANCE_THRESHOLD 0.02f     ///< 陀螺仪方差阈值。
#define GYRO_CAL_COUNT_MAX      20        ///< 陀螺仪校准计数最大值。
#define VARIANCE_SAMPLE_SIZE    20        ///< 方差采样大小。
#define INTEGRAL_LIMIT          0.5f      ///< 积分限幅。
#define ERROR_MAGNITUDE_TH      1.0e-8f   ///< 错误幅度阈值。
#define ACC_NORM_MIN_SQ         1.0e-6f   ///< 加速度最小范数平方。

#define YAW_ZERO    (-0.11f)   ///< 偏航零位偏移。
#define PITCH_ZERO  (+0.04f)   ///< 俯仰零位偏移。
#define ROLL_ZERO   (0.0f)     ///< 滚转零位偏移。

#define US_TO_HALF_SEC(x) ((float)(x) * 0.0000005f)   ///< 微秒转半秒。

typedef struct _ahrsState {
    float exInt;                                      ///< X轴积分误差。
    float eyInt;                                      ///< Y轴积分误差。
    float ezInt;                                      ///< Z轴积分误差。
    float q0;                                         ///< 四元数q0。
    float q1;                                         ///< 四元数q1。
    float q2;                                         ///< 四元数q2。
    float q3;                                         ///< 四元数q3。
    uint32_t lastUpdate;                              ///< 上次更新时间。
    uint32_t now;                                     ///< 当前时间。
    float gyroAngles[3];                              ///< 陀螺仪角度。
    float gyroOffset[3];                              ///< 陀螺仪偏移。
    float gyroFill[3][VARIANCE_SAMPLE_SIZE];          ///< 陀螺仪数据缓冲区。
    float gyroTotal[3];                               ///< 陀螺仪数据总和。
    float sqrGyroTotal[3];                            ///< 陀螺仪数据平方总和。
    uint8_t gyroInitFlag;                             ///< 陀螺仪初始化标志。
    uint8_t gyroCount;                                ///< 陀螺仪计数器。
    uint8_t calCount;                                 ///< 校准计数器。
} AHRS_STATE_S;

static AHRS_STATE_S g_ahrs = {
    .q0 = 1.0f
};

static const float INV_WIN_SIZE = 1.0f / (float)VARIANCE_SAMPLE_SIZE;   ///< 窗口大小倒数。

uint32_t IMU_GetMicros(void)
{
    return (uint32_t)rt_tick_get() * (1000000U / RT_TICK_PER_SECOND);
}

/**
 * @brief 更新陀螺仪数据窗口。
 *
 * 将新的陀螺仪数据加入到滑动窗口中，并更新总和和平方总和。
 *
 * @param data 新的陀螺仪数据，数组长度为3。
 */
static void updateGyroWindow(const float data[3])
{
    int i;

    if (!g_ahrs.gyroInitFlag) {
        for (i = 0; i < 3; i++) {
            g_ahrs.gyroFill[i][g_ahrs.gyroCount] = data[i];
            g_ahrs.gyroTotal[i] += data[i];
            g_ahrs.sqrGyroTotal[i] += data[i] * data[i];
        }

        g_ahrs.gyroCount++;
        if (g_ahrs.gyroCount >= VARIANCE_SAMPLE_SIZE) {
            g_ahrs.gyroCount = 0;
            g_ahrs.gyroInitFlag = 1;
        }
        return;
    }

    for (i = 0; i < 3; i++) {
        float old = g_ahrs.gyroFill[i][g_ahrs.gyroCount];
        g_ahrs.gyroTotal[i] -= old;
        g_ahrs.sqrGyroTotal[i] -= old * old;
        g_ahrs.gyroFill[i][g_ahrs.gyroCount] = data[i];
        g_ahrs.gyroTotal[i] += data[i];
        g_ahrs.sqrGyroTotal[i] += data[i] * data[i];
    }

    g_ahrs.gyroCount++;
    if (g_ahrs.gyroCount >= VARIANCE_SAMPLE_SIZE) {
        g_ahrs.gyroCount = 0;
    }
}

/**
 * @brief 获取陀螺仪统计数据。
 *
 * 计算当前窗口内陀螺仪数据的平均值和方差。
 *
 * @param avgResult 用于存储平均值的数组，长度为3。
 * @param varResult 用于存储方差的数组，长度为3。
 * @return int 如果成功计算并返回数据，返回1，否则返回0。
 */
static int getGyroStats(float avgResult[3], float varResult[3])
{
    int i;

    if (!g_ahrs.gyroInitFlag) {
        return 0;
    }

    for (i = 0; i < 3; i++) {
        float mean = g_ahrs.gyroTotal[i] * INV_WIN_SIZE;
        avgResult[i] = mean;
        varResult[i] = (g_ahrs.sqrGyroTotal[i] * INV_WIN_SIZE) -
                       (mean * mean);
    }

    return 1;
}

/**
 * @brief 初始化IMU传感器。
 *
 * 该方法初始化IMU传感器，并设置四元数初始值和积分误差。
 * 如果初始化成功，启动定时器；否则，通过USART打印错误信息。
 */
void IMU_init(void)
{
    if (bsp_Icm42688Init() == 0) {
        g_ahrs.q0 = 1.0f;
        g_ahrs.q1 = 0.0f;
        g_ahrs.q2 = 0.0f;
        g_ahrs.q3 = 0.0f;
        g_ahrs.exInt = 0.0f;
        g_ahrs.eyInt = 0.0f;
        g_ahrs.ezInt = 0.0f;
        g_ahrs.lastUpdate = IMU_GetMicros();
        g_ahrs.now = g_ahrs.lastUpdate;
    }
}

/**
 * @brief 读取IMU传感器数据。
 *
 * 读取加速度计和陀螺仪的原始数据，并进行初步处理。
 *
 * @param[out] values 用于存储处理后的传感器数据的数组，长度为6。
 *                     前三个元素为加速度数据（ax, ay, az），后三个元素为陀螺仪数据（gx, gy, gz）。
 */
void IMU_getValues(float *values)
{
    float avg[3];
    float var[3];
    icm42688RealData_t accval;
    icm42688RealData_t gyroval;

    bsp_IcmGetRawData(&accval, &gyroval);

    g_ahrs.gyroAngles[0] = gyroval.x;
    g_ahrs.gyroAngles[1] = gyroval.y;
    g_ahrs.gyroAngles[2] = gyroval.z;

    updateGyroWindow(g_ahrs.gyroAngles);

    if (g_ahrs.gyroInitFlag &&
        (g_ahrs.calCount >= GYRO_CAL_COUNT_MAX)) {
        if (getGyroStats(avg, var) &&
            (var[0] < GYRO_VARIANCE_THRESHOLD) &&
            (var[1] < GYRO_VARIANCE_THRESHOLD) &&
            (var[2] < GYRO_VARIANCE_THRESHOLD)) {
            g_ahrs.gyroOffset[0] = avg[0];
            g_ahrs.gyroOffset[1] = avg[1];
            g_ahrs.gyroOffset[2] = avg[2];
            g_ahrs.exInt = 0.0f;
            g_ahrs.eyInt = 0.0f;
            g_ahrs.ezInt = 0.0f;
            g_ahrs.calCount = 0;
        }
    } else if (g_ahrs.calCount < GYRO_CAL_COUNT_MAX) {
        g_ahrs.calCount++;
    }

    values[0] = accval.x;
    values[1] = accval.y;
    values[2] = accval.z;
    values[3] = g_ahrs.gyroAngles[0] - g_ahrs.gyroOffset[0];
    values[4] = g_ahrs.gyroAngles[1] - g_ahrs.gyroOffset[1];
    values[5] = g_ahrs.gyroAngles[2] - g_ahrs.gyroOffset[2];
}

/**
 * @brief 更新AHRS姿态估计。
 *
 * 使用加速度计和陀螺仪数据更新姿态估计，基于四元数和互补滤波器。
 *
 * @param gx 陀螺仪X轴数据，单位：度/秒。
 * @param gy 陀螺仪Y轴数据，单位：度/秒。
 * @param gz 陀螺仪Z轴数据，单位：度/秒。
 * @param ax 加速度计X轴数据，单位：g。
 * @param ay 加速度计Y轴数据，单位：g。
 * @param az 加速度计Z轴数据，单位：g。
 * @param mx 磁力计X轴数据，单位：高斯。
 * @param my 磁力计Y轴数据，单位：高斯。
 * @param mz 磁力计Z轴数据，单位：高斯。
 */
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,
                    float mx, float my, float mz)
{
    float norm;
    float vx;
    float vy;
    float vz;
    float ex;
    float ey;
    float ez;
    float halfT;
    float tempq0;
    float tempq1;
    float tempq2;
    float tempq3;
    float acc_norm_sq;
    float error_magnitude_sq;

    (void)mx;
    (void)my;
    (void)mz;

    g_ahrs.now = IMU_GetMicros();
    halfT = US_TO_HALF_SEC((uint32_t)(g_ahrs.now - g_ahrs.lastUpdate));
    g_ahrs.lastUpdate = g_ahrs.now;

    acc_norm_sq = ax * ax + ay * ay + az * az;
    if (acc_norm_sq < ACC_NORM_MIN_SQ) {
        return;
    }

    norm = fast_math_inv_sqrtf(acc_norm_sq);
    ax *= norm;
    ay *= norm;
    az *= norm;

    vx = 2.0f * (g_ahrs.q1 * g_ahrs.q3 - g_ahrs.q0 * g_ahrs.q2);
    vy = 2.0f * (g_ahrs.q0 * g_ahrs.q1 + g_ahrs.q2 * g_ahrs.q3);
    vz = g_ahrs.q0 * g_ahrs.q0 - g_ahrs.q1 * g_ahrs.q1 -
         g_ahrs.q2 * g_ahrs.q2 + g_ahrs.q3 * g_ahrs.q3;

    ex = (ay * vz - az * vy);
    ey = (az * vx - ax * vz);
    ez = (ax * vy - ay * vx);

    error_magnitude_sq = ex * ex + ey * ey + ez * ez;
    if (error_magnitude_sq > ERROR_MAGNITUDE_TH) {
        g_ahrs.exInt += ex * halfT;
        g_ahrs.eyInt += ey * halfT;
        g_ahrs.ezInt += ez * halfT;

        g_ahrs.exInt = fast_math_clampf(g_ahrs.exInt, -INTEGRAL_LIMIT,
                                        INTEGRAL_LIMIT);
        g_ahrs.eyInt = fast_math_clampf(g_ahrs.eyInt, -INTEGRAL_LIMIT,
                                        INTEGRAL_LIMIT);
        g_ahrs.ezInt = fast_math_clampf(g_ahrs.ezInt, -INTEGRAL_LIMIT,
                                        INTEGRAL_LIMIT);

        gx += KP * ex + KI * g_ahrs.exInt;
        gy += KP * ey + KI * g_ahrs.eyInt;
        gz += KP * ez + KI * g_ahrs.ezInt;
    } else {
        gx += KP * ex;
        gy += KP * ey;
        gz += KP * ez;
    }

    tempq0 = g_ahrs.q0 + (-g_ahrs.q1 * gx - g_ahrs.q2 * gy -
                          g_ahrs.q3 * gz) * halfT;
    tempq1 = g_ahrs.q1 + (g_ahrs.q0 * gx + g_ahrs.q2 * gz -
                          g_ahrs.q3 * gy) * halfT;
    tempq2 = g_ahrs.q2 + (g_ahrs.q0 * gy - g_ahrs.q1 * gz +
                          g_ahrs.q3 * gx) * halfT;
    tempq3 = g_ahrs.q3 + (g_ahrs.q0 * gz + g_ahrs.q1 * gy -
                          g_ahrs.q2 * gx) * halfT;

    norm = fast_math_inv_sqrtf(tempq0 * tempq0 + tempq1 * tempq1 +
                               tempq2 * tempq2 + tempq3 * tempq3);
    g_ahrs.q0 = tempq0 * norm;
    g_ahrs.q1 = tempq1 * norm;
    g_ahrs.q2 = tempq2 * norm;
    g_ahrs.q3 = tempq3 * norm;
}

/**
 * @brief 获取四元数。
 *
 * 获取当前姿态估计的四元数值。
 *
 * @param[out] q 用于存储四元数值的数组，长度为4。
 */
void IMU_getQ(float *q)
{
    float sensorData[6];

    IMU_getValues(sensorData);
    IMU_AHRSupdate(sensorData[3] * FAST_MATH_DEG_TO_RAD,
                   sensorData[4] * FAST_MATH_DEG_TO_RAD,
                   sensorData[5] * FAST_MATH_DEG_TO_RAD,
                   sensorData[0], sensorData[1], sensorData[2],
                   0.0f, 0.0f, 0.0f);

    q[0] = g_ahrs.q0;
    q[1] = g_ahrs.q1;
    q[2] = g_ahrs.q2;
    q[3] = g_ahrs.q3;
}

/**
 * @brief 获取偏航、俯仰和滚转角度。
 *
 * 获取当前姿态估计的偏航、俯仰和滚转角度。
 *
 * @param[out] angles 用于存储欧拉角的数组，长度为3。
 *                     第一个元素为偏航角（yaw），第二个元素为俯仰角（pitch），第三个元素为滚转角（roll）。
 */
void IMU_getYawPitchRoll(float *angles)
{
    float q[4];
    float pitch_input;

    IMU_getQ(q);

    angles[0] = -fast_math_atan2f(2.0f * q[1] * q[2] + 2.0f * q[0] * q[3],
                                  -2.0f * q[2] * q[2] - 2.0f * q[3] * q[3] + 1.0f) * FAST_MATH_RAD_TO_DEG + YAW_ZERO;

    pitch_input = fast_math_clampf(-2.0f * q[1] * q[3] + 2.0f * q[0] * q[2], -1.0f, 1.0f);
    angles[1] = -fast_math_asinf(pitch_input) * FAST_MATH_RAD_TO_DEG + PITCH_ZERO;

    angles[2] = fast_math_atan2f(2.0f * q[2] * q[3] + 2.0f * q[0] * q[1],
                                 -2.0f * q[1] * q[1] - 2.0f * q[2] * q[2] + 1.0f) * FAST_MATH_RAD_TO_DEG + ROLL_ZERO;
}

/**
 * @brief 获取原始陀螺仪数据。
 *
 * 获取当前的陀螺仪原始数据。
 *
 * @param[out] gyroData 用于存储陀螺仪数据的数组，长度为3。
 */
void IMU_TT_getgyro(float *gyroData)
{
    gyroData[0] = g_ahrs.gyroAngles[0];
    gyroData[1] = g_ahrs.gyroAngles[1];
    gyroData[2] = g_ahrs.gyroAngles[2];
}

