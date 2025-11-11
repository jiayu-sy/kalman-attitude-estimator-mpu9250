//
// Created by dlin2 on 2025/7/18.
//

#ifndef MPU_DRIVER_H
#define MPU_DRIVER_H

#include "i2c.h"

// I2C slave address, same for  MPU-6050 / 6500 / 9250 / 9255
#define MPU_ADD     0xD0 // 0x68 << 1

// MPU6500/9250 Register Map
#define MPU_WHO_AM_I_ADD    0x75
#define	SMPLRT_DIV		0x19 // sample rate
#define	CONFIG			0x1A // [2:0] is DLPF_CFG, digital low-pass filter
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define ACCEL_CONFIG2	0x1D

#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48

#define	PWR_MGMT_1		0x6B // power management
#define	PWR_MGMT_2		0x6C
#define INT_ENABLE      0x38 // Interrupt
#define INT_STATUS      0x3A

#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27
#define INT_PIN_CFG     0x37 // set bypass enable
#define USER_CTRL       0x6A

#define EXT_SENS_DATA_00   0x49
#define EXT_SENS_DATA_01   0x4A
#define EXT_SENS_DATA_02   0x4B
#define EXT_SENS_DATA_03   0x4C
#define EXT_SENS_DATA_04   0x4D
#define EXT_SENS_DATA_05   0x4E

// within AK8963:
#define AK8963_ADD      0x18
#define AK8963_WHO_AM_I_ADD 0x00
#define AK8963_ST1      0x02 // status 1
#define AK8963_ST2      0x09 // status 2
#define AK8963_CNTL1    0x0A // control 1, set continuous measurement mode

#define AK8963_HXL      0x03
#define AK8963_HXH      0x04
#define AK8963_HYL      0x05
#define AK8963_HYH      0x06
#define AK8963_HZL      0x07
#define AK8963_HZH      0x08

void DWT_Init(void);
uint8_t MPU_GetID(I2C_HandleTypeDef *hi2c);
uint8_t MPU_Get_AK8963_ID(I2C_HandleTypeDef *hi2c);
void MPU_Init(I2C_HandleTypeDef *hi2c);
void MPU_INT_Init(I2C_HandleTypeDef *hi2c);
void MPU_ReadAcc(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az);
void MPU_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t *gx, int16_t *gy, int16_t *gz);
void MPU_ReadMag(I2C_HandleTypeDef *hi2c, int16_t *mx, int16_t *my, int16_t *mz);

#endif //MPU_DRIVER_H
