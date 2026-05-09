#ifndef __BSP_ICM42688_H__
#define __BSP_ICM42688_H__

#include "main.h"
#include <stdint.h>

/*
 * Default to HAL I2C because the current project config exposes I2C1,
 * while SPI2 is shared with the LCD and is configured as TX-only.
 */
#if !defined(ICM42688_USE_I2C) && !defined(ICM42688_USE_SPI)
#define ICM42688_USE_I2C
#endif

#define ICM42688_I2C_ADDR_7BIT           0x69U
#define ICM42688_I2C_ADDR                (ICM42688_I2C_ADDR_7BIT << 1)
#define ICM42688_TIMEOUT_MS              100U

/* Bank 0 */
#define ICM42688_DEVICE_CONFIG           0x11U
#define ICM42688_DRIVE_CONFIG            0x13U
#define ICM42688_INT_CONFIG              0x14U
#define ICM42688_FIFO_CONFIG             0x16U
#define ICM42688_TEMP_DATA1              0x1DU
#define ICM42688_ACCEL_DATA_X1           0x1FU
#define ICM42688_GYRO_DATA_X1            0x25U
#define ICM42688_SIGNAL_PATH_RESET       0x4BU
#define ICM42688_INTF_CONFIG0            0x4CU
#define ICM42688_INTF_CONFIG1            0x4DU
#define ICM42688_PWR_MGMT0               0x4EU
#define ICM42688_GYRO_CONFIG0            0x4FU
#define ICM42688_ACCEL_CONFIG0           0x50U
#define ICM42688_GYRO_CONFIG1            0x51U
#define ICM42688_GYRO_ACCEL_CONFIG0      0x52U
#define ICM42688_ACCEL_CONFIG1           0x53U
#define ICM42688_TMST_CONFIG             0x54U
#define ICM42688_INT_SOURCE0             0x65U
#define ICM42688_WHO_AM_I                0x75U
#define ICM42688_REG_BANK_SEL            0x76U

/* Bank 1 */
#define ICM42688_INTF_CONFIG4            0x7AU

#define ICM42688_ID                      0x47U

#define AFS_2G                           0x03U
#define AFS_4G                           0x02U
#define AFS_8G                           0x01U
#define AFS_16G                          0x00U

#define GFS_2000DPS                      0x00U
#define GFS_1000DPS                      0x01U
#define GFS_500DPS                       0x02U
#define GFS_250DPS                       0x03U
#define GFS_125DPS                       0x04U
#define GFS_62_5DPS                      0x05U
#define GFS_31_25DPS                     0x06U
#define GFS_15_125DPS                    0x07U

#define AODR_8000Hz                      0x03U
#define AODR_4000Hz                      0x04U
#define AODR_2000Hz                      0x05U
#define AODR_1000Hz                      0x06U
#define AODR_200Hz                       0x07U
#define AODR_100Hz                       0x08U
#define AODR_50Hz                        0x09U
#define AODR_25Hz                        0x0AU
#define AODR_12_5Hz                      0x0BU
#define AODR_6_25Hz                      0x0CU
#define AODR_3_125Hz                     0x0DU
#define AODR_1_5625Hz                    0x0EU
#define AODR_500Hz                       0x0FU

#define GODR_8000Hz                      0x03U
#define GODR_4000Hz                      0x04U
#define GODR_2000Hz                      0x05U
#define GODR_1000Hz                      0x06U
#define GODR_200Hz                       0x07U
#define GODR_100Hz                       0x08U
#define GODR_50Hz                        0x09U
#define GODR_25Hz                        0x0AU
#define GODR_12_5Hz                      0x0BU
#define GODR_500Hz                       0x0FU

#ifdef ICM42688_USE_SPI
#ifndef ICM42688_SPI_HANDLE
#define ICM42688_SPI_HANDLE              hspi2
#endif
#ifndef ICM42688_CS_GPIO_Port
#define ICM42688_CS_GPIO_Port            GPIOA
#endif
#ifndef ICM42688_CS_Pin
#define ICM42688_CS_Pin                  GPIO_PIN_4
#endif
#endif

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} icm42688RawData_t;

typedef struct {
    float x;
    float y;
    float z;
} icm42688RealData_t;

int8_t bsp_Icm42688Init(void);
uint8_t bsp_Icm42688GetLastStatus(void);
uint32_t bsp_Icm42688GetErrorCount(void);
int8_t bsp_IcmGetTemperature(int16_t *pTemp);
int8_t bsp_IcmGetAccelerometer(icm42688RawData_t *accData);
int8_t bsp_IcmGetGyroscope(icm42688RawData_t *gyroData);
int8_t bsp_IcmGetRawData(icm42688RealData_t *accData, icm42688RealData_t *gyroData);

#endif
