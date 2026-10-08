#include "mpu6050.h"

static I2C_HandleTypeDef *s_hi2c;

/* 写一个寄存器 */
static void MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    HAL_I2C_Mem_Write(s_hi2c, MPU6050_ADDR, reg, 1, &data, 1, 100);
}

/* 连续读多个寄存器 */
static void MPU6050_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    HAL_I2C_Mem_Read(s_hi2c, MPU6050_ADDR, reg, 1, buf, len, 100);
}

uint8_t MPU6050_ReadID(I2C_HandleTypeDef *hi2c)
{
    uint8_t id = 0;
    s_hi2c = hi2c;
    HAL_I2C_Mem_Read(s_hi2c, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 1, &id, 1, 100);
    return id;
}

void MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    HAL_Delay(100);

    /* 退出休眠, 选X轴PLL为时钟源 */
    MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01);
    HAL_Delay(10);
}

void MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_Raw_t *raw)
{
    uint8_t buf[14];
    s_hi2c = hi2c;

    /* 连读14字节: accel(6) + temp(2) + gyro(6) */
    MPU6050_ReadRegs(MPU6050_REG_ACCEL_XOUT_H, buf, 14);

    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);

    /* ±2g → 16384 LSB/g */
    raw->ax = ax / 16384.0f;
    raw->ay = ay / 16384.0f;
    raw->az = az / 16384.0f;

    /* ±250°/s → 131 LSB/(°/s) */
    raw->gx = gx / 131.0f;
    raw->gy = gy / 131.0f;
    raw->gz = gz / 131.0f;
}
