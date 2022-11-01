/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"




/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "VL53L1X.h"
#include "stdio.h"
#include "stdio.h"
#include "string.h"


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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* GLOBAL VARIABLES */
VL53L1X sensor1;
VL53L1X sensor2;
VL53L1X sensor3;

enum State {
	DRIVE = 1,
	REVERSE = 0
};

char distanceStr1[200] = "wasd";
char distanceStr2[200] = "wasd";
char distanceStr3[200] = "wasd";
uint16_t dist1 = 0;
uint16_t dist2 = 0;
uint16_t dist3 = 0;

// motors speed (max 1000)
int left_speed = 200;
int right_speed = 200;

// 1 -  forward | 0 - backwards
int left_dir = DRIVE;
int right_dir = DRIVE;


void init() {
	  HAL_TIM_Base_Start(&htim3);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	  TOF_InitStruct(&sensor1, &hi2c1, 0x32, TOF_1_GPIO_Port, TOF_1_Pin);
	  TOF_InitStruct(&sensor2, &hi2c1, 0x33, TOF_2_GPIO_Port, TOF_2_Pin);
	  TOF_InitStruct(&sensor3, &hi2c1, 0x34, TOF_3_GPIO_Port, TOF_3_Pin);

	  TOF_TurnOff(&sensor1);
	  TOF_TurnOff(&sensor2);
	  TOF_TurnOff(&sensor3);

	  TOF_BootSensor(&sensor1);
	  TOF_BootSensor(&sensor2);
	  TOF_BootSensor(&sensor3);
}

int min_dist1 = 50;
int min_dist2 = 100;

int slow_speed = 100;
int fast_speed = 200;

void drive() {
	if (dist2 < min_dist1) {
		left_speed = slow_speed;
		right_speed = slow_speed;
		left_dir = DRIVE;
		right_dir = REVERSE;
	} else if (dist1 < min_dist1) {
		left_speed = slow_speed;
		right_speed = slow_speed;
		left_dir = DRIVE;
		right_dir = REVERSE;
	} else if (dist3 < min_dist1) {
		left_speed = slow_speed;
		right_speed = slow_speed;
		left_dir = REVERSE;
		right_dir = DRIVE;
	} else if (dist1 < min_dist2) {
		left_speed = fast_speed;
		right_speed = slow_speed;
		left_dir = DRIVE;
		right_dir = DRIVE;
	} else if(dist3 < min_dist2) {
		left_speed = slow_speed;
		right_speed = fast_speed;
		left_dir = DRIVE;
		right_dir = DRIVE;
	} else {
		left_speed = fast_speed;
		right_speed = fast_speed;
		left_dir = DRIVE;
		right_dir = DRIVE;
	}
}

void sense() {
	//TOF kuulamine
	dist1 = TOF_GetDistance(&sensor1);
	dist2 = TOF_GetDistance(&sensor2);
	dist3 = TOF_GetDistance(&sensor3);
}

void plan() {
	drive();

}

void act() {
	sprintf(distanceStr1, "Distance 1: %d\n\r", dist1);
	sprintf(distanceStr2, "Distance 2: %d\n\r", dist2);
	sprintf(distanceStr3, "Distance 3: %d\n\r", dist3);

	HAL_UART_Transmit(&huart2, (uint8_t*)distanceStr1, strlen(distanceStr1), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)distanceStr2, strlen(distanceStr2), 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)distanceStr3, strlen(distanceStr3), 100);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_speed);
	HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin, left_dir);

	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_speed);
	HAL_GPIO_WritePin(MOTOR_DIR_2_GPIO_Port, MOTOR_DIR_2_Pin, right_dir);
}
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  sense();
	  plan();
	  act();
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
