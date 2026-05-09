#include "QMC5883P.h"
#include "i2c.h"
#include <math.h>

#ifndef QMC5883P_PI
#define QMC5883P_PI 3.14159265358979323846f
#endif

static qmc5883p_range_t qmc5883p_active_range = QMC5883P_RANGE_2G;
static uint16_t qmc5883p_active_address = QMC5883P_I2C_ADDR;
static uint8_t qmc5883p_calibration_enabled = 1U;
static float qmc5883p_x_min = 0.0f;
static float qmc5883p_x_max = 0.0f;
static float qmc5883p_y_min = 0.0f;
static float qmc5883p_y_max = 0.0f;

static HAL_StatusTypeDef qmc5883p_is_ready(uint16_t address)
{
    return HAL_I2C_IsDeviceReady(&hi2c1, address, 3U, QMC5883P_TIMEOUT_MS);
}

static HAL_StatusTypeDef qmc5883p_write_reg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             qmc5883p_active_address,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             QMC5883P_TIMEOUT_MS);
}

static HAL_StatusTypeDef qmc5883p_read_regs(uint8_t reg, uint8_t *buffer, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            qmc5883p_active_address,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buffer,
                            len,
                            QMC5883P_TIMEOUT_MS);
}

static float qmc5883p_get_lsb_per_gauss(void)
{
    switch (qmc5883p_active_range) {
    case QMC5883P_RANGE_2G:
        return 30000.0f;
    case QMC5883P_RANGE_8G:
        return 8000.0f;
    case QMC5883P_RANGE_12G:
        return 12000.0f;
    case QMC5883P_RANGE_30G:
    default:
        return 3000.0f;
    }
}

static HAL_StatusTypeDef qmc5883p_configure_device(void)
{
    HAL_StatusTypeDef status;

    status = qmc5883p_write_reg(QMC5883P_REG_AXIS_CFG, QMC5883P_AXIS_CFG_DEFAULT);
    if (status != HAL_OK) {
        return status;
    }

    status = qmc5883p_write_reg(QMC5883P_REG_CONTROL_2, QMC5883P_CTRL2_SETRESET_ON_8G);
    if (status != HAL_OK) {
        return status;
    }

    status = qmc5883p_write_reg(QMC5883P_REG_CONTROL_1, QMC5883P_CTRL1_NORMAL_8G_200HZ);
    if (status == HAL_OK) {
        qmc5883p_active_range = QMC5883P_RANGE_8G;
        QMC5883P_ResetCalibration();
    }

    return status;
}

HAL_StatusTypeDef QMC5883P_Probe(void)
{
    uint8_t chip_id = 0U;

    if (qmc5883p_is_ready(QMC5883P_I2C_ADDR) != HAL_OK) {
        return HAL_ERROR;
    }

    qmc5883p_active_address = QMC5883P_I2C_ADDR;
    if (qmc5883p_read_regs(QMC5883P_REG_CHIP_ID, &chip_id, 1U) != HAL_OK) {
        return HAL_ERROR;
    }

    return (chip_id == QMC5883P_CHIP_ID_VALUE) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef QMC5883P_Reset(void)
{
    return qmc5883p_write_reg(QMC5883P_REG_CONTROL_2, 0x80U);
}

HAL_StatusTypeDef QMC5883P_Init(qmc5883p_mode_t mode,
                                qmc5883p_odr_t odr,
                                qmc5883p_range_t range,
                                qmc5883p_osr_t osr)
{
    HAL_StatusTypeDef status;
    (void)mode;
    (void)odr;
    (void)range;
    (void)osr;

    status = QMC5883P_Probe();
    if (status != HAL_OK) {
        return status;
    }

    status = QMC5883P_Reset();
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(10U);
    status = qmc5883p_configure_device();
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(5U);
    return HAL_OK;
}

uint16_t QMC5883P_GetActiveAddress(void)
{
    return qmc5883p_active_address;
}

void QMC5883P_SetCalibrationEnabled(uint8_t enabled)
{
    qmc5883p_calibration_enabled = enabled ? 1U : 0U;
}

uint8_t QMC5883P_IsCalibrationEnabled(void)
{
    return qmc5883p_calibration_enabled;
}

uint8_t QMC5883P_ReadStatus(void)
{
    uint8_t status = 0U;

    (void)qmc5883p_read_regs(QMC5883P_REG_STATUS, &status, 1U);
    return status;
}

uint8_t QMC5883P_IsDataReady(void)
{
    return (uint8_t)((QMC5883P_ReadStatus() & QMC5883P_STATUS_DRDY) != 0U);
}

uint8_t QMC5883P_IsOverflow(void)
{
    return (uint8_t)((QMC5883P_ReadStatus() & QMC5883P_STATUS_OVL) != 0U);
}

void QMC5883P_ResetCalibration(void)
{
    qmc5883p_x_min = 0.0f;
    qmc5883p_x_max = 0.0f;
    qmc5883p_y_min = 0.0f;
    qmc5883p_y_max = 0.0f;
}

HAL_StatusTypeDef QMC5883P_ReadRaw(qmc5883p_raw_data_t *data)
{
    uint8_t buffer[6];
    HAL_StatusTypeDef status;

    status = qmc5883p_read_regs(QMC5883P_REG_STATUS, &buffer[0], 1U);
    if (status != HAL_OK) {
        return status;
    }

    if ((buffer[0] & QMC5883P_STATUS_DRDY) == 0U) {
        return HAL_BUSY;
    }

    status = qmc5883p_read_regs(QMC5883P_REG_X_LSB, buffer, sizeof(buffer));
    if (status != HAL_OK) {
        return status;
    }

    data->x = (int16_t)(((uint16_t)buffer[1] << 8) | buffer[0]);
    data->y = (int16_t)(((uint16_t)buffer[3] << 8) | buffer[2]);
    data->z = (int16_t)(((uint16_t)buffer[5] << 8) | buffer[4]);
    return HAL_OK;
}

HAL_StatusTypeDef QMC5883P_ReadGauss(qmc5883p_gauss_data_t *data)
{
    qmc5883p_raw_data_t raw;
    float lsb_per_gauss;
    HAL_StatusTypeDef status;

    status = QMC5883P_ReadRaw(&raw);
    if (status != HAL_OK) {
        return status;
    }

    lsb_per_gauss = qmc5883p_get_lsb_per_gauss();
    data->x = (float)raw.x / lsb_per_gauss;
    data->y = (float)raw.y / lsb_per_gauss;
    data->z = (float)raw.z / lsb_per_gauss;
    return HAL_OK;
}

HAL_StatusTypeDef QMC5883P_ReadTemperature(int16_t *temperature_c)
{
    (void)temperature_c;
    return HAL_ERROR;
}

HAL_StatusTypeDef QMC5883P_ReadYawDeg(float *yaw_deg)
{
    qmc5883p_raw_data_t raw;
    float x;
    float y;
    float yaw_rad;
    HAL_StatusTypeDef status;

    status = QMC5883P_ReadRaw(&raw);
    if (status != HAL_OK) {
        return status;
    }

    x = (float)raw.x;
    y = (float)raw.y;

    if (qmc5883p_calibration_enabled != 0U) {
        if (x < qmc5883p_x_min) {
            qmc5883p_x_min = x;
        } else if (x > qmc5883p_x_max) {
            qmc5883p_x_max = x;
        }

        if (y < qmc5883p_y_min) {
            qmc5883p_y_min = y;
        } else if (y > qmc5883p_y_max) {
            qmc5883p_y_max = y;
        }
    }

    if ((qmc5883p_x_min != qmc5883p_x_max) && (qmc5883p_y_min != qmc5883p_y_max)) {
        x -= (qmc5883p_x_max + qmc5883p_x_min) * 0.5f;
        y -= (qmc5883p_y_max + qmc5883p_y_min) * 0.5f;
        x /= (qmc5883p_x_max - qmc5883p_x_min);
        y /= (qmc5883p_y_max - qmc5883p_y_min);
    }

    yaw_rad = atan2f(y, x) + QMC5883P_DECLINATION_RAD;
    if (yaw_rad < 0.0f) {
        yaw_rad += 2.0f * QMC5883P_PI;
    } else if (yaw_rad >= (2.0f * QMC5883P_PI)) {
        yaw_rad -= 2.0f * QMC5883P_PI;
    }

    *yaw_deg = yaw_rad * (180.0f / QMC5883P_PI);
    return HAL_OK;
}
