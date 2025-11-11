/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>

#include "mpu_driver.h"
#include "kalman.h"

#define CPU_CLOCK_MHZ 100
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t mpu_data_ready = 0;
KalmanData kf_roll;
KalmanData kf_pitch;
KalmanData kf_yaw;
uint32_t last_cycle = 0;
uint32_t timestamp_us = 0;
int16_t ax, ay, az, gx, gy, gz, mx, my, mz;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  MPU_Init(&hi2c1);
  MPU_INT_Init(&hi2c1);
  DWT_Init();
  Kalman_Init(&kf_roll);
  Kalman_Init(&kf_pitch);
  Kalman_Init_Yaw(&kf_yaw);

  // print WHO_AM_I
  // uint8_t mpu_id = MPU_GetID(&hi2c1);
  // char buf[16];
  // int le = snprintf(buf, sizeof(buf), "MPU ID: %x", mpu_id);
  // HAL_UART_Transmit(&huart2, buf, le, HAL_MAX_DELAY);
  // HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // uint8_t check = 0;
  // HAL_I2C_Mem_Read(&hi2c1, AK8963_ADD, AK8963_CNTL1, I2C_MEMADD_SIZE_8BIT, &check, 1, 100);

  // char buf[64];
  // int l = snprintf(buf, sizeof(buf), "AK8963 CNTL1 = 0x%02X\r\n", check);
  // HAL_UART_Transmit(&huart2, buf, l, HAL_MAX_DELAY);
  // HAL_I2C_Mem_Read(&hi2c1, AK8963_ADD, AK8963_ST1, I2C_MEMADD_SIZE_8BIT, &check, 1, 100);
  // char buf[64];
  // int l = snprintf(buf, sizeof(buf), "AK8963 CNTL1 = 0x%02X\r\n", check);
  // HAL_UART_Transmit(&huart2, buf, l, HAL_MAX_DELAY);

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // HAL_I2C_Mem_Read(&hi2c1, AK8963_ADD, AK8963_ST1, I2C_MEMADD_SIZE_8BIT, &check, 1, 100);
    // char buf[64];
    // int l = snprintf(buf, sizeof(buf), "AK8963 CNTL1 = 0x%02X\r\n", check);
    // HAL_UART_Transmit(&huart2, buf, l, HAL_MAX_DELAY);
    // HAL_Delay(200);

    if (mpu_data_ready) {
      mpu_data_ready = 0;

      float dt = (float)(timestamp_us - last_cycle) / (CPU_CLOCK_MHZ * 1e6f);
      last_cycle = timestamp_us;

      // MPU_ReadMag(&hi2c1, &mx, &my, &mz);

      // char buffer[64];
      // int len = snprintf(buffer, sizeof(buffer), "%d %d %d\r\n", mx, my, mz);
      // HAL_UART_Transmit(&huart2, buffer, len, HAL_MAX_DELAY);

      float acc_roll = atan2f(ay, az) * 180.0f / M_PI; // to degree
      float acc_pitch = -atan2f(ax, az) * 180.0f / M_PI;
      float mag_yaw = atan2f(my, mx) * 180.0f / M_PI;
      if (mag_yaw < 0) mag_yaw += 360.0f;

      float gyro_x = gx / 16.4f;
      float gyro_y = gy / 16.4f;
      float gyro_z = gz / 16.4f;

      Kalman_Update(&kf_roll, dt, acc_roll, gyro_x);
      Kalman_Update(&kf_pitch, dt, acc_pitch, gyro_y);
      Kalman_Update(&kf_yaw, dt, mag_yaw, gyro_z);

      char buffer[128];
      // int len = snprintf(buffer, sizeof(buffer), "%.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %.2f\r\n",
      //                    kf_roll.bias, kf_roll.angle, acc_roll, gyro_x, kf_pitch.bias, kf_pitch.angle, acc_pitch, gyro_y,
      //                    kf_yaw.bias, kf_yaw.angle);
      int len = snprintf(buffer, sizeof(buffer), "%.2f %.2f %.2f\r\n", kf_yaw.bias, kf_yaw.angle, mag_yaw);

      HAL_UART_Transmit(&huart2, buffer, len, HAL_MAX_DELAY);

      // char buffer[64];
      // int len = snprintf(buffer, sizeof(buffer), "%d %d %d\r\n", mx, my, mz);
      // HAL_UART_Transmit(&huart2, buffer, len, HAL_MAX_DELAY);
    }
  }
}
  /* USER CODE END 3 */

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// IMU Interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_0) { // PA0
    mpu_data_ready = 1;

    // char msg[] = "interrupting\r\n";
    // HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg) - 1, HAL_MAX_DELAY);

    //get raw
    MPU_ReadAcc(&hi2c1, &ax, &ay, &az);
    MPU_ReadGyro(&hi2c1, &gx, &gy, &gz);
    MPU_ReadMag(&hi2c1, &mx, &my, &mz);

    // record time
    timestamp_us = DWT->CYCCNT;
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
