#ifndef QMC5883P_H
#define QMC5883P_H

#include "main.h"
#include <stdint.h>

#define QMC5883P_I2C_ADDR_7BIT         0x2CU
#define QMC5883P_I2C_ADDR              (QMC5883P_I2C_ADDR_7BIT << 1)
#define QMC5883P_TIMEOUT_MS            100U

#define QMC5883P_REG_CHIP_ID           0x00U
#define QMC5883P_REG_X_LSB             0x01U
#define QMC5883P_REG_STATUS            0x09U
#define QMC5883P_REG_CONTROL_1         0x0AU
#define QMC5883P_REG_CONTROL_2         0x0BU
#define QMC5883P_REG_AXIS_CFG          0x29U

#define QMC5883P_STATUS_DRDY           0x01U
#define QMC5883P_STATUS_OVL            0x02U

#define QMC5883P_CHIP_ID_VALUE         0x80U
#define QMC5883P_DECLINATION_RAD       0.0f

#define QMC5883P_AXIS_CFG_DEFAULT      0x06U
#define QMC5883P_CTRL1_NORMAL_8G_200HZ 0xCDU
#define QMC5883P_CTRL2_SETRESET_ON_8G  0x08U

typedef enum {
    QMC5883P_MODE_STANDBY = 0x00U,
    QMC5883P_MODE_NORMAL = 0x01U,
    QMC5883P_MODE_SINGLE = 0x02U,
    QMC5883P_MODE_CONTINUOUS = 0x03U
} qmc5883p_mode_t;

typedef enum {
    QMC5883P_ODR_10HZ = 0x00U,
    QMC5883P_ODR_50HZ = 0x04U,
    QMC5883P_ODR_100HZ = 0x08U,
    QMC5883P_ODR_200HZ = 0x0CU
} qmc5883p_odr_t;

typedef enum {
    QMC5883P_RANGE_30G = 0x00U,
    QMC5883P_RANGE_12G = 0x04U,
    QMC5883P_RANGE_8G = 0x08U,
    QMC5883P_RANGE_2G = 0x0CU
} qmc5883p_range_t;

typedef enum {
    QMC5883P_OSR_8 = 0x00U,
    QMC5883P_OSR_4 = 0x10U,
    QMC5883P_OSR_2 = 0x20U,
    QMC5883P_OSR_1 = 0x30U
} qmc5883p_osr_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} qmc5883p_raw_data_t;

typedef struct {
    float x;
    float y;
    float z;
} qmc5883p_gauss_data_t;

HAL_StatusTypeDef QMC5883P_Init(qmc5883p_mode_t mode,
                                qmc5883p_odr_t odr,
                                qmc5883p_range_t range,
                                qmc5883p_osr_t osr);
HAL_StatusTypeDef QMC5883P_Reset(void);
HAL_StatusTypeDef QMC5883P_ReadRaw(qmc5883p_raw_data_t *data);
HAL_StatusTypeDef QMC5883P_ReadGauss(qmc5883p_gauss_data_t *data);
HAL_StatusTypeDef QMC5883P_ReadYawDeg(float *yaw_deg);
HAL_StatusTypeDef QMC5883P_ReadTemperature(int16_t *temperature_c);
HAL_StatusTypeDef QMC5883P_Probe(void);
uint16_t QMC5883P_GetActiveAddress(void);
void QMC5883P_SetCalibrationEnabled(uint8_t enabled);
uint8_t QMC5883P_IsCalibrationEnabled(void);
uint8_t QMC5883P_ReadStatus(void);
uint8_t QMC5883P_IsDataReady(void);
uint8_t QMC5883P_IsOverflow(void);
void QMC5883P_ResetCalibration(void);

#endif
