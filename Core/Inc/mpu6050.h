#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f1xx_hal.h"

/* MPU6050 7位地址0x68, HAL库需要左移1位 */
#define MPU6050_ADDR        (0x68 << 1)

/* 寄存器地址 */
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_WHO_AM_I       0x75
#define MPU6050_REG_ACCEL_XOUT_H   0x3B

/* 原始数据结构体, 已换算成物理单位 */
typedef struct {
    float ax, ay, az;   /* 加速度, 单位 g */
    float gx, gy, gz;   /* 角速度, 单位 °/s */
} MPU6050_Raw_t;

uint8_t MPU6050_ReadID(I2C_HandleTypeDef *hi2c);
void    MPU6050_Init(I2C_HandleTypeDef *hi2c);
void    MPU6050_ReadRaw(I2C_HandleTypeDef *hi2c, MPU6050_Raw_t *raw);

#endif
