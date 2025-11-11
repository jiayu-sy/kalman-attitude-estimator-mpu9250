#include "mpu_driver.h"

// Update on 7/31, extend to mpu-9250 9-axis IMU

// use DWT
void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable DWT peripheral
    DWT->CYCCNT = 0;                                 // Reset counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Enable the cycle counter
}

// Get Chip WHO_AM_I ID, no need to call MPU_Init first
uint8_t MPU_GetID(I2C_HandleTypeDef *hi2c){
    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(hi2c,MPU_ADD,MPU_WHO_AM_I_ADD,I2C_MEMADD_SIZE_8BIT, &id, 1,100) == HAL_OK){
        return id;
    }
    return 0xFF; // error
}

uint8_t MPU_Get_AK8963_ID(I2C_HandleTypeDef *hi2c) {
    // treat AK9063 as I2C Slave of MPU9250
    // config I2C_SLVx register set
    // this is one method, the alternative would be to use bypass mode

    // config mpu as i2c master
    uint8_t val = 0x20; // I2C Master Mode enable
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &val, 1, 100); // USER_CTRL

    val = 0x8C; // [7] = I2C_SLV0_RNW, [6:0] = I2C_ID_0
    // I2C slave address of AK8963 is 0x0C
    // this is not the same as what we did for mpu i2c slave address
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, I2C_SLV0_ADDR, I2C_MEMADD_SIZE_8BIT, &val, 1, 100); // I2C_SLV0_ADDR

    // config target i2c slave register in AK8963
    val = AK8963_WHO_AM_I_ADD;
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, I2C_SLV0_REG, I2C_MEMADD_SIZE_8BIT, &val, 1, 100); // I2C_SLV0_REG
    val = 0x81; // enable, length=1
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, I2C_SLV0_CTRL, I2C_MEMADD_SIZE_8BIT, &val, 1, 100); // I2C_SLV0_CTRL

    HAL_Delay(10); // wait transmission complete

    uint8_t id = 0;
    if (HAL_I2C_Mem_Read(hi2c, MPU_ADD, EXT_SENS_DATA_00, I2C_MEMADD_SIZE_8BIT, &id, 1, 100) == HAL_OK) {
        return id;
    }
    return 0xFF;
}

// Config Registers
void MPU_Init(I2C_HandleTypeDef *hi2c){
    uint8_t val = 0x01;
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    val = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, PWR_MGMT_2, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    val = 0x31; // sample rate divider
    // sample rate = 1kHz / (1 + SMPLRT_DIV)
    // 1000 / (1 + 49) = 20Hz -> 49 = 0b110001 = 0x31
    // 1000 / (1 + 19) = 50Hz -> 19 = 0x13
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    val = 0x06; // Config
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, CONFIG, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    val = 0x18;
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    val = 0x18;
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);

    // magnetometer init:
    val = 0x00; // disable i2c master
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, USER_CTRL, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    HAL_Delay(10);
    val = 0x02; // enable bypass mode
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, INT_PIN_CFG, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    HAL_Delay(10);
    val = 0x16; // 16-bit output, continuous measurement mode 2
    HAL_I2C_Mem_Write(hi2c, AK8963_ADD, AK8963_CNTL1, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
    HAL_Delay(10);
}

void MPU_INT_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t val = 0x01; // Enable interrupt
    HAL_I2C_Mem_Write(hi2c, MPU_ADD, INT_ENABLE, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}

// raw acc
void MPU_ReadAcc(I2C_HandleTypeDef *hi2c, int16_t *ax, int16_t *ay, int16_t *az) {
    uint8_t buffer[6];
    // Read continuous memory from address 0x3B to 0x40
    HAL_I2C_Mem_Read(hi2c, MPU_ADD, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

    // signed
    *ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    *ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    *az = (int16_t)(buffer[4] << 8 | buffer[5]);
}

// raw gyro
void MPU_ReadGyro(I2C_HandleTypeDef *hi2c, int16_t *gx, int16_t *gy, int16_t *gz) {
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(hi2c, MPU_ADD, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

    *gx = (int16_t)(buffer[0] << 8 | buffer[1]);
    *gy = (int16_t)(buffer[2] << 8 | buffer[3]);
    *gz = (int16_t)(buffer[4] << 8 | buffer[5]);
}

// raw mag
void MPU_ReadMag(I2C_HandleTypeDef *hi2c, int16_t *mx, int16_t *my, int16_t *mz) {
    uint8_t buffer[6];
    HAL_I2C_Mem_Read(hi2c, AK8963_ADD, AK8963_HXL, I2C_MEMADD_SIZE_8BIT, buffer, 6, 100);

    // mind order
    *mx = (int16_t)(buffer[1] << 8 | buffer[0]);
    *my = (int16_t)(buffer[3] << 8 | buffer[2]);
    *mz = (int16_t)(buffer[5] << 8 | buffer[4]);

    uint8_t st2;
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, AK8963_ADD, AK8963_ST2, I2C_MEMADD_SIZE_8BIT, &st2, 1, 100);
    if (status != HAL_OK) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    }
}
