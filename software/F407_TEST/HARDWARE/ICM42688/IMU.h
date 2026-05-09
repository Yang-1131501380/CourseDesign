#ifndef __IMU_H
#define __IMU_H

#include "main.h"
#include "fast_math.h"

#ifndef M_PI
#define M_PI FAST_MATH_PI
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD FAST_MATH_DEG_TO_RAD
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG FAST_MATH_RAD_TO_DEG
#endif

typedef struct {
    float accel[3];
    float gyro[3];
    float mag[3];
} SensorData_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

typedef struct {
    float yaw;
    float pitch;
    float roll;
} EulerAngles_t;

void IMU_init(void);
void IMU_getYawPitchRoll(float *angles);
void IMU_TT_getgyro(float *gyroData);
void MPU6050_InitAng_Offset(void);
void IMU_getQ(float *q);
uint32_t IMU_GetMicros(void);

#endif
