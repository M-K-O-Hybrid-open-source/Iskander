/* USER CODE BEGIN Header */
//TIM1_ch1 LEG1_HIGH
//TIM1_ch2 LEG1_LOW 			nd so on 
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "string.h"
#include "stdlib.h"
#include "stdarg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "PID.h"

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.14159265

/* Controller parameters */
#define PID_KP_I  100.0f
#define PID_KI_I  20.0f
#define PID_KD_I  0.0f

#define PID_TAU_I 0.02f

#define PID_LIM_MIN_I  0.0f
#define PID_LIM_MAX_I  1800.0f

#define PID_LIM_MIN_INT_I  0.0f
#define PID_LIM_MAX_INT_I  1700.0f

#define SAMPLE_TIME_S 0.0001f					// f = clk_freq/(Period*INC) = 72e6/(720*10) = 1e4 Hz = .0001 sec




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float map(float x, float in_min, float in_max, float out_min, float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint16_t ADC_VALs[4];
uint8_t sensor[3];
uint8_t STEP=0;


uint32_t time_tehif=0;
double time_diff=0;
double speed=0;
uint8_t Pole_P = 1;

float mes_I=0;
	

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void FIND_MOTOR_STEP(void);
void APPLY_THE_PWMs(uint8_t step);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void printMsg(char *msg, ...){
 char buff[80];
 //#ifdef DEBUG_UART
 
 va_list args;
 va_start(args,msg);
 vsprintf(buff,msg,args);
 
 for(int i=0;i<strlen(buff);i++)
 {
	 USART1->DR = buff[i];
	 while(!(USART1->SR & USART_SR_TXE));
	}
} 

/* Initialisec PID controller */

PIDController pid_I = { PID_KP_I, PID_KI_I, -PID_KD_I,
												PID_TAU_I,
												PID_LIM_MIN_I, PID_LIM_MAX_I,
												PID_LIM_MIN_INT_I, PID_LIM_MAX_INT_I,
												SAMPLE_TIME_S };


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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	HAL_ADC_Start_DMA(&hadc1,( uint32_t *)ADC_VALs,4);

	// TIMER init	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	// turn on complementary channel
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // turn on complementary channel
	
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // turn on complementary channel

	
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)0);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,(uint32_t)0);

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)0);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)0);
	
	HAL_TIM_Base_Start_IT(&htim3);
	PIDController_Init(&pid_I);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		mes_I =  map(ADC_VALs[0],0,4048,0,20);;
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 1800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim2.Init.Period = 1800;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 900;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : HLL_1_Pin HLL_2_Pin HLL_3_Pin */
  GPIO_InitStruct.Pin = HLL_1_Pin|HLL_2_Pin|HLL_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */


// The time update for the controller-one second only for NOW, every 100 micro-second
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	UNUSED(htim);
	if (htim->Instance == TIM3) {
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
		time_tehif++;
		PIDController_Update(&pid_I, 10, mes_I);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)pid_I.out);				// firts leg high switch

	}
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
		
if( (GPIO_Pin == HLL_1_Pin) |  (GPIO_Pin == HLL_2_Pin) | (GPIO_Pin == HLL_3_Pin) ){
	
	// disble the INT
	HAL_TIM_Base_Stop(&htim3);
	
	
	//__HAL_TIM_DISABLE_IT(&htim3);
	sensor[0]=HAL_GPIO_ReadPin(HLL_1_GPIO_Port,HLL_1_Pin);
	sensor[1]=HAL_GPIO_ReadPin(HLL_2_GPIO_Port,HLL_2_Pin);
	sensor[2]=HAL_GPIO_ReadPin(HLL_3_GPIO_Port,HLL_3_Pin);
	
	// time clcultion
	
	time_diff = (TIM3->CNT*1e-6);
	time_diff +=(time_tehif*1e-4);
	speed = (2.0*PI/3.0)/(time_diff*Pole_P);
	
	
	STEP=0;		
	FIND_MOTOR_STEP();
	APPLY_THE_PWMs(STEP);


	//HAL_UART_Transmit(&huart1,"HII",3,10);
	time_tehif = 0;
	TIM3->CNT = 0;
	HAL_TIM_Base_Start(&htim3);

	}else{

	}	
}





// Steps function --> this functions Takes no rguments and returns no arguments, just declare
// a GLOBAL variable called STEP
void FIND_MOTOR_STEP(void){
	
		// Cheack for the step
		if( sensor[0] == 1 && sensor[1] == 0 && sensor[2] == 0 ){         // step 6  //100
			STEP = 6;
			
    }    
    else if( sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 0 ){  // step 4  //010  
			STEP = 4;
			
    }
    else if( sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 0){  // step 5  //110
			STEP = 5;
			
    }
    else if( sensor[0] == 0 && sensor[1] == 0 && sensor[2] ==1 ){   // step 2  //001
			STEP = 2;
			
    }
    else if( sensor[0] == 1 && sensor[1] ==0 && sensor[2] ==1 ){    // step 1  //101
			STEP = 1;
			
    }
    else if( sensor[0] == 0 && sensor[1] == 1 && sensor[2] == 1 ){  // step 3  //011
			STEP = 3;
			
    }
		else{
			STEP = 0;
			
    }	
		
}






void APPLY_THE_PWMs(uint8_t step){
	
    switch (step) {
			
      case 1:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)pid_I.out);				// firts leg high switch
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)0);														// first leg low switch
				
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)0);														// second leg high switch
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)pid_I.out);				// second leg low switch
				
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)0);														// third leg high switch
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,(uint32_t)0);														// third leg low switch
        break;
        
      case 2:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)0);
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)pid_I.out);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)0);
			
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,(uint32_t)pid_I.out);
        break;

      case 3:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)pid_I.out);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)0);
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)0);
			
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,(uint32_t)pid_I.out);
        break;
        
      case 4:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)pid_I.out);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)0);
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)pid_I.out);
			
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,(uint32_t)0);
        break;
        
      case 5:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)0);
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)pid_I.out);
			
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)pid_I.out);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,(uint32_t)0);
        break;
        
      case 6:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)pid_I.out);
			
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)0);
	
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)pid_I.out);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,(uint32_t)0);
        break;
			case 0:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)0);				// firts leg high switch
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)0);				// first leg low switch

				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)0);				// second leg high switch
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)0);				// second leg low switch

				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)0);				// third leg high switch
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,(uint32_t)0);				// third leg low switch				
							
      default:
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,(uint32_t)0);

				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,(uint32_t)0);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4,(uint32_t)0);
        break;
			
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
