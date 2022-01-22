/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include "math.h"
#include "arm_math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// SPEED CALCULATION
#define ENCODER_RESOLUTION 224.4
#define TIMER_CONF_BOTH_EDGE_T1T2 4
#define	TIMER_FREQENCY 10
#define	MINUTE_IN_SECOND 60


// PID CONTROLER CONFIG
#define PID_KP 1.2
#define PID_KI 0.1
#define PID_KD 0.2


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// PWM CONFIG // CounterPeriod = 100
volatile uint16_t duty_A = 0;
volatile uint16_t duty_B = 0;

// UART CONFIG
uint8_t user_val[4]; // [X X X R/L]
const char error_1[] = "WRONG DIR!\r\n";
const char error_2[] = "WRONG SPEED!\r\n";
const char error_3[] = "UART ERROR\r\n";
const char error_4[] = "WRONG STATE! WRITE U OR P!\r\n";

volatile float32_t user_speed = 0;
volatile uint8_t flag = 0;
volatile uint8_t state = 0; // default: 0 - UART, another: 1 - ADC

uint16_t data_msg[64];
int length;

// ADC CONFIG
const uint32_t ADC_REG_MAX = 0xfff; //12-bits
const float32_t ADC_VOLTAGE_MAX = 3.3; //[V]

//ADC wyniki konwersji
volatile uint32_t ADC_measurement = 0; //wartość rejestru
volatile float32_t ADC_voltage = 0; //wartośc napięcia w Voltach


// ENCODER CONFIG
uint32_t counter= 0;
int16_t count = 0;

// SPEED CALCULATION
const float32_t max_speed = 270;
const float32_t min_speed = 30;

float32_t speed = 0;
float32_t reference_speed = 0;

// PID CONTROLER CONFIG
arm_pid_instance_f32 PID; // controller instance
float32_t PID_Output = 0;
float32_t PID_Error = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void SpeedCalculation(int16_t count){
	speed = (float32_t)((count * TIMER_FREQENCY * MINUTE_IN_SECOND)/
			(ENCODER_RESOLUTION*TIMER_CONF_BOTH_EDGE_T1T2));
}

void SetDutyPID(arm_pid_instance_f32* pid, float32_t y_ref, float32_t y){

	PID_Error = y_ref - y; //Error calc
	PID_Output = arm_pid_f32(pid, PID_Error); // Output PID signal

	if(PID_Output > 0){
		// SATURATION
		if (PID_Output > 1000){
			PID_Output = 1000;
		}

		duty_A = (uint16_t)(abs(PID_Output));
		duty_B = 0;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_A); // PA6
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_B); // PC7
	}
	else{
		// SATURATION
		if (PID_Output < -1000){
			PID_Output = -1000;
		}

		duty_A = 0;
		duty_B = (uint16_t)(abs(PID_Output));;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_A); // PA6
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_B); // PC7
	}


}


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

	// PID CONTROLER INIT
	PID.Kp = PID_KP;
	PID.Ki = PID_KI;
	PID.Kd = PID_KD;

	arm_pid_init_f32(&PID, 1);
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // PWM CONFIG // CounterPeriod = 1000
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, duty_A); // PA6
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, duty_B); // PC7

  // UART CONFIG
  HAL_UART_Receive_IT(&huart3, user_val, 4);

  // ENCODER CONFIG
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

  // MAIN TIMER CONFIG
  HAL_TIM_Base_Start_IT(&htim6);

  // DATA TRANSMIT TIMER CONFIG
  HAL_TIM_Base_Start_IT(&htim7);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// UART CONFIG
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3){
		// USER SPEED READ
		if(state == 0){
		user_speed = (float32_t)(atof(user_val));

		if(user_speed >= min_speed && user_speed <= max_speed){
			flag = 1;
			if(user_val[3] == 'R' && flag == 1){
				reference_speed = user_speed;
			}
			else if(user_val[3] == 'L' && flag == 1){
				reference_speed = -(user_speed);
			}
			else{
				flag = 0;
				HAL_UART_Transmit(&huart3, error_1, strlen(error_1), 100);
			}
		}
		else{
			flag = 0;
			HAL_UART_Transmit(&huart3, error_2, strlen(error_2), 100);
		}
		}
		else{
			HAL_UART_Transmit(&huart3, error_3, strlen(error_3), 100);
		}
	}

	HAL_UART_Receive_IT(&huart3, user_val, 4);
}

// ADC CONFIG
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc1){
		ADC_measurement = HAL_ADC_GetValue(&hadc1);
		ADC_voltage = ((float32_t)ADC_measurement / (float32_t)ADC_REG_MAX) * ADC_VOLTAGE_MAX;

		if(state == 1){
			reference_speed = (uint16_t)((ADC_voltage * max_speed)/ADC_VOLTAGE_MAX);

			if(reference_speed < min_speed){
				reference_speed = min_speed;
			}
			else if(reference_speed > max_speed){
				reference_speed = max_speed;
			}
		}

	}
}

// TIMER CONFIG
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// MAIN TIMER CONFIG
	if(htim -> Instance == TIM6)
	{
		HAL_ADC_Start_IT(&hadc1);

		// SPEED CALCULATION
		counter = __HAL_TIM_GET_COUNTER(&htim1);
		count = (int16_t)counter;
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		SpeedCalculation(count);

		// SPEED REGULATION
		SetDutyPID(&PID, reference_speed, speed);
	}

	// DATA TRANSMIT TIMER CONFIG
	if(htim -> Instance == TIM7)
	{
		length = sprintf(data_msg, " POM: %3.3f  , REF: %3.3f  , STER: %3.3f \r\n", (float)speed,  (float)reference_speed, (float)PID_Output);
		HAL_UART_Transmit(&huart3, data_msg, length, 0xffff);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
