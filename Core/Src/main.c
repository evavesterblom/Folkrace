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

// Car states
enum State {
	DRIVE = 1,
	REVERSE = 0
};

// Constants
int SERVO_MIN_VALUE = 1225;
int SERVO_MID_VALUE = 1450;
int SERVO_MAX_VALUE = 1625;

int MAX_TURN_ANGLE = 45;
// how to set servo position: __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, 1220);

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
int left_dir = 1;
int right_dir = 1;

// servo angle
int servo = 1450;
int servoTarget = 1450;

void init() {
	// Motor PWM signals
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	// Servo PWM Signal
	HAL_TIM_Base_Start(&htim14);
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

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

void correct() {
	if (servo > SERVO_MAX_VALUE) {
		servo = SERVO_MAX_VALUE;
	}

	if (servo < SERVO_MIN_VALUE) {
		servo = SERVO_MIN_VALUE;
	}

	if (left_speed > 1000) {
		left_speed = 1000;
	}

	if (right_speed > 1000) {
		right_speed = 1000;
	}

	//left_dir = left_dir == 1 ? 0 : 1;

	left_speed = 400;
	right_speed = 400;
}

int servo_min_dist = 110;
int servo_mid_dist = 150;
int servo_max_dist = 200;

int motor_very_slow_speed = 400;
int motor_slow_speed = 400;
int motor_mid_speed = 400;
int motor_high_speed = 400;

int sensDiff = 0;
int sensDir = 0;

double findTurnMagnitude(int sensorDiffMilliMeters) {
    double sensorDiff = sensorDiffMilliMeters / 10;
    sensDiff = sensorDiff;
    double upper = (sensorDiff - 50);
    upper = upper * upper * upper;
    double lower = 4000;
    double add = 28;
    return upper / lower + add;
}

void setServoTurnLeft(double turnAngle) {
    int turnRange = SERVO_MID_VALUE - SERVO_MIN_VALUE;
    double servoDiff = turnAngle / MAX_TURN_ANGLE * turnRange;
    servoTarget = SERVO_MID_VALUE - (int) servoDiff;
}

void setServoTurnRight(double turnAngle) {
    int turnRange = SERVO_MAX_VALUE - SERVO_MID_VALUE;
    double servoDiff = turnAngle / MAX_TURN_ANGLE * turnRange;
    servoTarget = SERVO_MID_VALUE + (int) servoDiff;
}

void findServoTarget() {
    int sensorDiff = dist1 - dist3;
    int turnDirection = sensorDiff < 0 ? 1 : -1;
    sensDir = turnDirection;
    sensorDiff = sensorDiff < 0 ? -sensorDiff : sensorDiff;
    double turnAngle = findTurnMagnitude(sensorDiff);
    turnAngle = turnAngle < 0 ? 0 : turnAngle;
    turnAngle = turnAngle > MAX_TURN_ANGLE ? MAX_TURN_ANGLE : turnAngle;
    if (turnDirection < 0) setServoTurnLeft(turnAngle);
    else setServoTurnRight(turnAngle);
}

int ADJUSTMENT_LIMIT = 150;

void driveSmoothServo() {
	findServoTarget();
	int servoAdjustment = 1;
	if (dist1 < ADJUSTMENT_LIMIT || dist3 < ADJUSTMENT_LIMIT) servoAdjustment = 5;
	if (servo < servoTarget) servo += servoAdjustment;
	else servo -= servoAdjustment;
//	servo = servoTarget;
	left_dir = REVERSE;
	right_dir = DRIVE;
}

void drive_by_servo() {
	// natuke lähedal vasakule servale, keerab natuke paremale
	if (dist1 < servo_max_dist && dist2 > servo_max_dist && dist3 > servo_max_dist) {
		servo = SERVO_MID_VALUE + 100;
		left_speed = motor_mid_speed;
		right_speed = motor_mid_speed;
		left_dir = REVERSE;
		right_dir = DRIVE;

	// natuke lähedal paremale servale, keerab natuke vasakule
	}else if (dist1 > servo_max_dist && dist2 > servo_max_dist && dist3 < servo_max_dist) {
		servo = SERVO_MID_VALUE - 100;
		left_speed = motor_mid_speed;
		right_speed = motor_mid_speed;
		left_dir = REVERSE;
		right_dir = DRIVE;

	// väga lähedal vasakule servale, keerab rohkem paremale
	} else {
		servo = SERVO_MID_VALUE;
		left_speed = motor_mid_speed;
		right_speed = motor_mid_speed;
		left_dir = REVERSE;
		right_dir = DRIVE;
	}
	// tagurdab kui sõidab nurka kinni
//	} else if (dist1 < servo_min_dist || dist2 < servo_min_dist || dist3 < servo_min_dist) {
//		servo = SERVO_MID_VALUE;
//		left_speed = motor_very_slow_speed;
//		right_speed = motor_very_slow_speed;
//		left_dir = REVERSE;
//		right_dir = REVERSE;
//	}
}

int SENSOR_ADJUSTMENT = 10;
int rawDist1 = 200;
int rawDist2 = 200;
int rawDist3 = 200;

void sense() {
	//TOF kuulamine
	rawDist1 = TOF_GetDistance(&sensor1);
	rawDist2 = TOF_GetDistance(&sensor2);
	rawDist3 = TOF_GetDistance(&sensor3);

	// Smoothing
	if (rawDist1 < dist1) dist1 -= SENSOR_ADJUSTMENT;
	else dist1 += SENSOR_ADJUSTMENT;
	if (rawDist2 < dist2) dist2 -= SENSOR_ADJUSTMENT;
	else dist2 += SENSOR_ADJUSTMENT;
	if (rawDist3 < dist3) dist3 -= SENSOR_ADJUSTMENT;
	else dist3 += SENSOR_ADJUSTMENT;
}

void plan() {
	//drive_by_motor_speed();
//	drive_by_servo();
	driveSmoothServo();

}

void act() {
	correct();

//	sprintf(distanceStr1, "L:%d       M:%d       R:%d \n\r", dist1, dist2, dist3);
	sprintf(distanceStr1, "Diff:%d       Dir:%d       Target:%d \n\r", sensDiff, sensDir, servoTarget);
	//sprintf(distanceStr2, "Distance 2: %d\n\r", dist2);
	//sprintf(distanceStr3, "Distance 3: %d\n\r", dist3);

	HAL_UART_Transmit(&huart2, (uint8_t*)distanceStr1, strlen(distanceStr1), 100);
	//HAL_UART_Transmit(&huart2, (uint8_t*)distanceStr2, strlen(distanceStr2), 100);
	//HAL_UART_Transmit(&huart2, (uint8_t*)distanceStr3, strlen(distanceStr3), 100);


	//set left motor direction and speed
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left_speed);
	HAL_GPIO_WritePin(MOTOR_DIR_1_GPIO_Port, MOTOR_DIR_1_Pin, left_dir);

	//set right motor direction and speed
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right_speed);
	HAL_GPIO_WritePin(MOTOR_DIR_2_GPIO_Port, MOTOR_DIR_2_Pin, right_dir);

	//set servo angle
	__HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, servo);
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
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  sense();
	  plan();
	  //left_dir = 1;
	  //right_dir = 1;
	  //left_speed = 400;
	  //right_speed = 400;
	  act();

	  //HAL_Delay(1000);
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
