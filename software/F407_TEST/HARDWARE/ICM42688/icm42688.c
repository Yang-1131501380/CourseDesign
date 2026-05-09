#include "icm42688.h"
#include "rtthread.h"

#ifdef ICM42688_USE_I2C
#include "i2c.h"
#endif

#ifdef ICM42688_USE_SPI
#include "spi.h"
#endif

typedef struct _icm42688State {
    float accSensitivity;
    float gyroSensitivity;
    volatile uint8_t lastStatus;
    volatile uint32_t errorCount;
} ICM42688_STATE_S;

static ICM42688_STATE_S g_icm42688State = {
    .accSensitivity = 0.244f,
    .gyroSensitivity = 32.8f,
    .lastStatus = HAL_OK,
    .errorCount = 0U
};

static void icm42688_record_status(HAL_StatusTypeDef status)
{
    g_icm42688State.lastStatus = (uint8_t)status;
    if (status != HAL_OK) {
        g_icm42688State.errorCount++;
    }
}

#ifdef ICM42688_USE_SPI
static inline void icm42688_cs_low(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin, GPIO_PIN_RESET);
}

static inline void icm42688_cs_high(void)
{
    HAL_GPIO_WritePin(ICM42688_CS_GPIO_Port, ICM42688_CS_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef icm42688_spi_rw(uint8_t *tx_buffer, uint8_t *rx_buffer, uint16_t len)
{
    return HAL_SPI_TransmitReceive(&ICM42688_SPI_HANDLE,
                                   tx_buffer,
                                   rx_buffer,
                                   len,
                                   ICM42688_TIMEOUT_MS);
}
#endif

static uint8_t icm42688_read_reg(uint8_t reg)
{
    uint8_t value = 0U;

#ifdef ICM42688_USE_SPI
    uint8_t tx_buffer[2] = { (uint8_t)(reg | 0x80U), 0xFFU };
    uint8_t rx_buffer[2] = { 0U };

    icm42688_cs_low();
    if (icm42688_spi_rw(tx_buffer, rx_buffer, 2U) == HAL_OK) {
        value = rx_buffer[1];
    }
    icm42688_cs_high();
#else
    icm42688_record_status(HAL_I2C_Mem_Read(&hi2c1,
                                            ICM42688_I2C_ADDR,
                                            reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            &value,
                                            1U,
                                            ICM42688_TIMEOUT_MS));
#endif

    return value;
}

static void icm42688_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
#ifdef ICM42688_USE_SPI
    uint8_t header = (uint8_t)(reg | 0x80U);

    icm42688_cs_low();
    (void)HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, &header, 1U, ICM42688_TIMEOUT_MS);
    (void)HAL_SPI_Receive(&ICM42688_SPI_HANDLE, buf, len, ICM42688_TIMEOUT_MS);
    icm42688_cs_high();
#else
    icm42688_record_status(HAL_I2C_Mem_Read(&hi2c1,
                                            ICM42688_I2C_ADDR,
                                            reg,
                                            I2C_MEMADD_SIZE_8BIT,
                                            buf,
                                            len,
                                            ICM42688_TIMEOUT_MS));
#endif
}

static uint8_t icm42688_write_reg(uint8_t reg, uint8_t value)
{
#ifdef ICM42688_USE_SPI
    uint8_t tx_buffer[2] = { reg, value };

    icm42688_cs_low();
    (void)HAL_SPI_Transmit(&ICM42688_SPI_HANDLE, tx_buffer, 2U, ICM42688_TIMEOUT_MS);
    icm42688_cs_high();
#else
    icm42688_record_status(HAL_I2C_Mem_Write(&hi2c1,
                                             ICM42688_I2C_ADDR,
                                             reg,
                                             I2C_MEMADD_SIZE_8BIT,
                                             &value,
                                             1U,
                                             ICM42688_TIMEOUT_MS));
#endif

    return 0;
}

uint8_t bsp_Icm42688GetLastStatus(void)
{
    return g_icm42688State.lastStatus;
}

uint32_t bsp_Icm42688GetErrorCount(void)
{
    return g_icm42688State.errorCount;
}

static float bsp_Icm42688GetAres(uint8_t scale)
{
    switch (scale) {
    case AFS_2G:
        g_icm42688State.accSensitivity = 2000.0f / 32768.0f;
        break;
    case AFS_4G:
        g_icm42688State.accSensitivity = 4000.0f / 32768.0f;
        break;
    case AFS_8G:
        g_icm42688State.accSensitivity = 8000.0f / 32768.0f;
        break;
    case AFS_16G:
    default:
        g_icm42688State.accSensitivity = 16000.0f / 32768.0f;
        break;
    }

    return g_icm42688State.accSensitivity;
}

static float bsp_Icm42688GetGres(uint8_t scale)
{
    switch (scale) {
    case GFS_15_125DPS:
        g_icm42688State.gyroSensitivity = 15.125f / 32768.0f;
        break;
    case GFS_31_25DPS:
        g_icm42688State.gyroSensitivity = 31.25f / 32768.0f;
        break;
    case GFS_62_5DPS:
        g_icm42688State.gyroSensitivity = 62.5f / 32768.0f;
        break;
    case GFS_125DPS:
        g_icm42688State.gyroSensitivity = 125.0f / 32768.0f;
        break;
    case GFS_250DPS:
        g_icm42688State.gyroSensitivity = 250.0f / 32768.0f;
        break;
    case GFS_500DPS:
        g_icm42688State.gyroSensitivity = 500.0f / 32768.0f;
        break;
    case GFS_1000DPS:
        g_icm42688State.gyroSensitivity = 1000.0f / 32768.0f;
        break;
    case GFS_2000DPS:
    default:
        g_icm42688State.gyroSensitivity = 2000.0f / 32768.0f;
        break;
    }

    return g_icm42688State.gyroSensitivity;
}

static int8_t bsp_Icm42688RegCfg(void)
{
    uint8_t reg_val = icm42688_read_reg(ICM42688_WHO_AM_I);

    icm42688_write_reg(ICM42688_REG_BANK_SEL, 0x00U);
    icm42688_write_reg(ICM42688_DEVICE_CONFIG, 0x01U);
    rt_thread_mdelay(100U);

    if (reg_val != ICM42688_ID) {
        return -1;
    }

    bsp_Icm42688GetAres(AFS_4G);
    reg_val = (uint8_t)((AFS_4G << 5) | AODR_100Hz);
    icm42688_write_reg(ICM42688_ACCEL_CONFIG0, reg_val);

    bsp_Icm42688GetGres(GFS_1000DPS);
    reg_val = (uint8_t)((GFS_1000DPS << 5) | GODR_100Hz);
    icm42688_write_reg(ICM42688_GYRO_CONFIG0, reg_val);

    reg_val = icm42688_read_reg(ICM42688_PWR_MGMT0);
    reg_val &= (uint8_t)~(1U << 5);
    reg_val |= (uint8_t)(3U << 2);
    reg_val |= 3U;
    icm42688_write_reg(ICM42688_PWR_MGMT0, reg_val);
    rt_thread_mdelay(1U);

    return 0;
}

int8_t bsp_Icm42688Init(void)
{
#ifdef ICM42688_USE_SPI
    icm42688_cs_high();
#endif
    return bsp_Icm42688RegCfg();
}

int8_t bsp_IcmGetTemperature(int16_t *pTemp)
{
    uint8_t buffer[2] = { 0U };

    icm42688_read_regs(ICM42688_TEMP_DATA1, buffer, 2U);
    *pTemp = (int16_t)(((int16_t)((buffer[0] << 8) | buffer[1])) / 132.48f + 25.0f);

    return 0;
}

int8_t bsp_IcmGetAccelerometer(icm42688RawData_t *accData)
{
    uint8_t buffer[6] = { 0U };

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 6U);

    accData->x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    accData->y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    accData->z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    accData->x = (int16_t)(accData->x * g_icm42688State.accSensitivity);
    accData->y = (int16_t)(accData->y * g_icm42688State.accSensitivity);
    accData->z = (int16_t)(accData->z * g_icm42688State.accSensitivity);

    return 0;
}

int8_t bsp_IcmGetGyroscope(icm42688RawData_t *gyroData)
{
    uint8_t buffer[6] = { 0U };

    icm42688_read_regs(ICM42688_GYRO_DATA_X1, buffer, 6U);

    gyroData->x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    gyroData->y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    gyroData->z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);

    gyroData->x = (int16_t)(gyroData->x * g_icm42688State.gyroSensitivity);
    gyroData->y = (int16_t)(gyroData->y * g_icm42688State.gyroSensitivity);
    gyroData->z = (int16_t)(gyroData->z * g_icm42688State.gyroSensitivity);

    return 0;
}

int8_t bsp_IcmGetRawData(icm42688RealData_t *accData, icm42688RealData_t *gyroData)
{
    uint8_t buffer[12] = { 0U };
    icm42688RawData_t accRaw;
    icm42688RawData_t gyroRaw;

    icm42688_read_regs(ICM42688_ACCEL_DATA_X1, buffer, 12U);

    accRaw.x = (int16_t)(((uint16_t)buffer[0] << 8) | buffer[1]);
    accRaw.y = (int16_t)(((uint16_t)buffer[2] << 8) | buffer[3]);
    accRaw.z = (int16_t)(((uint16_t)buffer[4] << 8) | buffer[5]);
    gyroRaw.x = (int16_t)(((uint16_t)buffer[6] << 8) | buffer[7]);
    gyroRaw.y = (int16_t)(((uint16_t)buffer[8] << 8) | buffer[9]);
    gyroRaw.z = (int16_t)(((uint16_t)buffer[10] << 8) | buffer[11]);

    accData->x = accRaw.x * g_icm42688State.accSensitivity;
    accData->y = accRaw.y * g_icm42688State.accSensitivity;
    accData->z = accRaw.z * g_icm42688State.accSensitivity;

    gyroData->x = gyroRaw.x * g_icm42688State.gyroSensitivity;
    gyroData->y = gyroRaw.y * g_icm42688State.gyroSensitivity;
    gyroData->z = gyroRaw.z * g_icm42688State.gyroSensitivity;

    return 0;
}
