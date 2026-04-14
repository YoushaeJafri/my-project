#include "main.h"
#include "IMU.h"

#include <math.h>

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

#define GYRO_CS_LOW()   HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET)

#define L3GD20_CTRL1          0x20U
#define L3GD20_CTRL4          0x23U
#define L3GD20_OUT_X_L        0x28U

#define LSM303_ACC_ADDR       (0x19U << 1)
#define LSM303_CTRL_REG1_A    0x20U
#define LSM303_CTRL_REG4_A    0x23U
#define LSM303_OUT_X_L_A      0x28U

#define GYRO_SENS_DPS_PER_LSB 0.00875f
#define RAD_TO_DEG            57.2957795f
#define ALPHA                 0.93f
#define DT_S                  0.01f
#define ACC_LPF_BETA          0.80f
#define GYRO_DEADBAND_DPS     0.15f

static float pitch_deg = 0.0f;
static float gyro_bias_x_dps = 0.0f;
static float acc_pitch_offset_deg = 0.0f;

static HAL_StatusTypeDef Gyro_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = {(uint8_t)(reg & 0x7FU), value};

    GYRO_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, tx, 2U, HAL_MAX_DELAY);
    GYRO_CS_HIGH();

    return st;
}

static HAL_StatusTypeDef Gyro_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t cmd = (uint8_t)(reg | 0x80U);
    if (len > 1U)
    {
        cmd |= 0x40U;
    }

    GYRO_CS_LOW();
    HAL_StatusTypeDef st = HAL_SPI_Transmit(&hspi1, &cmd, 1U, HAL_MAX_DELAY);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Receive(&hspi1, data, len, HAL_MAX_DELAY);
    }
    GYRO_CS_HIGH();

    return st;
}

static HAL_StatusTypeDef Accel_WriteReg(uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(&hi2c1, LSM303_ACC_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1U, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef Accel_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t addr = reg;
    if (len > 1U)
    {
        addr |= 0x80U;
    }

    return HAL_I2C_Mem_Read(&hi2c1, LSM303_ACC_ADDR, addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

static int Gyro_ReadRaw(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buffer[6];
    if (Gyro_ReadRegs(L3GD20_OUT_X_L, buffer, 6U) != HAL_OK)
    {
        return -1;
    }

    *gx = (int16_t)(((uint16_t)buffer[1] << 8U) | buffer[0]);
    *gy = (int16_t)(((uint16_t)buffer[3] << 8U) | buffer[2]);
    *gz = (int16_t)(((uint16_t)buffer[5] << 8U) | buffer[4]);
    return 0;
}

static int Accel_ReadRaw(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buffer[6];
    if (Accel_ReadRegs(LSM303_OUT_X_L_A, buffer, 6U) != HAL_OK)
    {
        return -1;
    }

    *ax = (int16_t)((((int16_t)((uint16_t)buffer[1] << 8U | buffer[0])) >> 4));
    *ay = (int16_t)((((int16_t)((uint16_t)buffer[3] << 8U | buffer[2])) >> 4));
    *az = (int16_t)((((int16_t)((uint16_t)buffer[5] << 8U | buffer[4])) >> 4));
    return 0;
}

int IMU_Init(void)
{
    int16_t gx = 0, gy = 0, gz = 0;
    int16_t ax = 0, ay = 0, az = 0;
    int32_t sum_x = 0;
    float sum_acc_pitch = 0.0f;
    const uint16_t samples = 300U;

    GYRO_CS_HIGH();

    if (Gyro_WriteReg(L3GD20_CTRL1, 0xBFU) != HAL_OK)
    {
        return -1;
    }
    if (Gyro_WriteReg(L3GD20_CTRL4, 0x00U) != HAL_OK)
    {
        return -1;
    }

    if (Accel_WriteReg(LSM303_CTRL_REG1_A, 0x57U) != HAL_OK)
    {
        return -1;
    }
    if (Accel_WriteReg(LSM303_CTRL_REG4_A, 0x08U) != HAL_OK)
    {
        return -1;
    }

    HAL_Delay(20);

    for (uint16_t i = 0; i < samples; i++)
    {
        if (Gyro_ReadRaw(&gx, &gy, &gz) != 0)
        {
            return -1;
        }
        if (Accel_ReadRaw(&ax, &ay, &az) != 0)
        {
            return -1;
        }
        sum_x += gx;
        sum_acc_pitch += atan2f((float)ax, (float)az) * RAD_TO_DEG;
        HAL_Delay(2);
    }

    (void)gy;
    (void)gz;

    gyro_bias_x_dps = ((float)sum_x / (float)samples) * GYRO_SENS_DPS_PER_LSB;
    acc_pitch_offset_deg = sum_acc_pitch / (float)samples;

    pitch_deg = 0.0f;
    return 0;
}

int IMU_Update(IMU_Angles_t *angles)
{
    int16_t ax = 0, ay = 0, az = 0;
    int16_t gx = 0, gy = 0, gz = 0;

    if (angles == NULL)
    {
        return -1;
    }

    if (Gyro_ReadRaw(&gx, &gy, &gz) != 0)
    {
        return -1;
    }
    if (Accel_ReadRaw(&ax, &ay, &az) != 0)
    {
        return -1;
    }

    (void)gy;
    (void)gz;

    /* Selected axis: X-axis tilt.
       accel tilt uses atan2(ay, az), gyro tilt-rate uses gyro X. */
    float gyro_x_dps = ((float)gx * GYRO_SENS_DPS_PER_LSB) - gyro_bias_x_dps;

    float axf = (float)ax;
    float azf = (float)az;

    float acc_pitch = (atan2f(axf, azf) * RAD_TO_DEG) - acc_pitch_offset_deg;

    if (fabsf(gyro_x_dps) < GYRO_DEADBAND_DPS)
    {
        gyro_x_dps = 0.0f;
    }

    /* angle = 0.98 * (angle + gyro_x * dt) + 0.02 * acc_x */
    pitch_deg = ALPHA * (pitch_deg + gyro_x_dps * DT_S) + (1.0f - ALPHA) * acc_pitch;

    angles->angle = pitch_deg;
    angles->accel = acc_pitch;
    angles->gyro = gyro_x_dps;

    return 0;
}