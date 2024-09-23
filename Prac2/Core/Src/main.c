/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f0xx.h"
#include "lcd_stm32f0.c"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// TODO: Add values for below variables
#define NS 127       // Number of samples in LUT
#define TIM2CLK 8000000  // STM Clock frequency
#define F_SIGNAL 1800 // Frequency of output analog signal
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */
// TODO: Add code for global variables, including LUTs

uint32_t Sin_LUT[NS] = {-1.0000,
		   -0.8727,
		   -0.7454,
		   -0.6180,
		   -0.4907,
		   -0.3634,
		   -0.2361,
		   -0.1087,
		    0.0186,
		    0.1459,
		    0.2732,
		    0.4006,
		    0.5279,
		    0.6552,
		    0.7825,
		    0.9099,
		    0.9628,
		    0.8355,
		    0.7082,
		    0.5808,
		    0.4535,
		    0.3262,
		    0.1989,
		    0.0715,
		   -0.0558,
		   -0.1831,
		   -0.3104,
		   -0.4377,
		   -0.5651,
		   -0.6924,
		   -0.8197,
		   -0.9470,
		   -0.9256,
		   -0.7983,
		   -0.6710,
		   -0.5437,
		   -0.4163,
		   -0.2890,
		   -0.1617,
		   -0.0344,
		    0.0930,
		    0.2203,
		    0.3476,
		    0.4749,
		    0.6023,
		    0.7296,
		    0.8569,
		    0.9842,
		    0.8885,
		    0.7611,
		    0.6338,
		    0.5065,
		    0.3792,
		    0.2518,
		    0.1245,
		   -0.0028,
		   -0.1301,
		   -0.2575,
		   -0.3848,
		   -0.5121,
		   -0.6394,
		   -0.7668,
		   -0.8941,
		   -0.9786,
		   -0.8513,
		   -0.7239,
		   -0.5966,
		   -0.4693,
		   -0.3420,
		   -0.2146,
		   -0.0873,
		    0.0400,
		    0.1673,
		    0.2946,
		    0.4220,
		    0.5493,
		    0.6766,
		    0.8039,
		    0.9313,
		    0.9414,
		    0.8141,
		    0.6868,
		    0.5594,
		    0.4321,
		    0.3048,
		    0.1775,
		    0.0501,
		   -0.0772,
		   -0.2045,
		   -0.3318,
		   -0.4592,
		   -0.5865,
		   -0.7138,
		   -0.8411,
		   -0.9685,
		   -0.9042,
		   -0.7769,
		   -0.6496,
		   -0.5223,
		   -0.3949,
		   -0.2676,
		   -0.1403,
		   -0.0130,
		    0.1144,
		    0.2417,
		    0.3690,
		    0.4963,
		    0.6237,
		    0.7510,
		    0.8783,
		    0.9944,
		    0.8670,
		    0.7397,
		    0.6124,
		    0.4851,
		    0.3577,
		    0.2304,
		    0.1031,
		   -0.0242,
		   -0.1516,
		   -0.2789,
		   -0.4062,
		   -0.5335,
		   -0.6608,
		   -0.7882,
		   -0.9155,
		   -0.9572,
		   -0.8299};
uint32_t saw_LUT[NS] = {0,
	    0.0613,
	    0.1224,
	    0.1830,
	    0.2430,
	    0.3020,
	    0.3599,
	    0.4164,
	    0.4714,
	    0.5246,
	    0.5758,
	    0.6249,
	    0.6716,
	    0.7157,
	    0.7572,
	    0.7958,
	    0.8315,
	    0.8640,
	    0.8932,
	    0.9191,
	    0.9415,
	    0.9604,
	    0.9757,
	    0.9873,
	    0.9952,
	    0.9993,
	    0.9997,
	    0.9963,
	    0.9892,
	    0.9783,
	    0.9638,
	    0.9456,
	    0.9239,
	    0.8987,
	    0.8701,
	    0.8382,
	    0.8032,
	    0.7652,
	    0.7242,
	    0.6806,
	    0.6344,
	    0.5858,
	    0.5350,
	    0.4822,
	    0.4276,
	    0.3713,
	    0.3137,
	    0.2549,
	    0.1951,
	    0.1346,
	    0.0736,
	    0.0123,
	   -0.0491,
	   -0.1102,
	   -0.1710,
	   -0.2311,
	   -0.2903,
	   -0.3484,
	   -0.4052,
	   -0.4605,
	   -0.5141,
	   -0.5657,
	   -0.6152,
	   -0.6624,
	   -0.7071,
	   -0.7491,
	   -0.7883,
	   -0.8246,
	   -0.8577,
	   -0.8876,
	   -0.9142,
	   -0.9373,
	   -0.9569,
	   -0.9729,
	   -0.9853,
	   -0.9939,
	   -0.9988,
	   -0.9999,
	   -0.9973,
	   -0.9909,
	   -0.9808,
	   -0.9670,
	   -0.9495,
	   -0.9285,
	   -0.9040,
	   -0.8761,
	   -0.8449,
	   -0.8105,
	   -0.7730,
	   -0.7327,
	   -0.6895,
	   -0.6438,
	   -0.5957,
	   -0.5453,
	   -0.4929,
	   -0.4386,
	   -0.3827,
	   -0.3253,
	   -0.2667,
	   -0.2071,
	   -0.1467,
	   -0.0858,
	   -0.0245,
	    0.0368,
	    0.0980,
	    0.1589,
	    0.2191,
	    0.2785,
	    0.3369,
	    0.3940,
	    0.4496,
	    0.5035,
	    0.5556,
	    0.6055,
	    0.6532,
	    0.6984,
	    0.7410,
	    0.7807,
	    0.8176,
	    0.8514,
	    0.8819,
	    0.9092,
	    0.9330,
	    0.9533,
	    0.9700,
	    0.9831,
	    0.9925,
	    0.9981};
uint32_t triangle_LUT[NS] = {-1.0000,
		   -0.9363,
		   -0.8727,
		   -0.8090,
		   -0.7454,
		   -0.6817,
		   -0.6180,
		   -0.5544,
		   -0.4907,
		   -0.4270,
		   -0.3634,
		   -0.2997,
		   -0.2361,
		   -0.1724,
		   -0.1087,
		   -0.0451,
		    0.0186,
		    0.0823,
		    0.1459,
		    0.2096,
		    0.2732,
		    0.3369,
		    0.4006,
		    0.4642,
		    0.5279,
		    0.5915,
		    0.6552,
		    0.7189,
		    0.7825,
		    0.8462,
		    0.9099,
		    0.9735,
		   -0.9628,
		   -0.8992,
		   -0.8355,
		   -0.7718,
		   -0.7082,
		   -0.6445,
		   -0.5808,
		   -0.5172,
		   -0.4535,
		   -0.3899,
		   -0.3262,
		   -0.2625,
		   -0.1989,
		   -0.1352,
		   -0.0715,
		   -0.0079,
		    0.0558,
		    0.1194,
		    0.1831,
		    0.2468,
		    0.3104,
		    0.3741,
		    0.4377,
		    0.5014,
		    0.5651,
		    0.6287,
		    0.6924,
		    0.7561,
		    0.8197,
		    0.8834,
		    0.9470,
		   -0.9893,
		   -0.9256,
		   -0.8620,
		   -0.7983,
		   -0.7346,
		   -0.6710,
		   -0.6073,
		   -0.5437,
		   -0.4800,
		   -0.4163,
		   -0.3527,
		   -0.2890,
		   -0.2254,
		   -0.1617,
		   -0.0980,
		   -0.0344,
		    0.0293,
		    0.0930,
		    0.1566,
		    0.2203,
		    0.2839,
		    0.3476,
		    0.4113,
		    0.4749,
		    0.5386,
		    0.6023,
		    0.6659,
		    0.7296,
		    0.7932,
		    0.8569,
		    0.9206,
		    0.9842,
		   -0.9521,
		   -0.8885,
		   -0.8248,
		   -0.7611,
		   -0.6975,
		   -0.6338,
		   -0.5701,
		   -0.5065,
		   -0.4428,
		   -0.3792,
		   -0.3155,
		   -0.2518,
		   -0.1882,
		   -0.1245,
		   -0.0608,
		    0.0028,
		    0.0665,
		    0.1301,
		    0.1938,
		    0.2575,
		    0.3211,
		    0.3848,
		    0.4485,
		    0.5121,
		    0.5758,
		    0.6394,
		    0.7031,
		    0.7668,
		    0.8304,
		    0.8941,
		    0.9577,
		   -0.9786,
		   -0.9149};

// TODO: Equation to calculate TIM2_Ticks
const int sinSource = (uint32_t)Sin_LUT;
const int sawSource = (uint32_t)saw_LUT;
const int triangleSource = (uint32_t)triangle_LUT;

uint32_t TIM2_Ticks = TIM2CLK/F_SIGNAL*NS; // How often to write new LUT value
uint32_t DestAddress = (uint32_t) &(TIM3->CCR3); // Write LUT TO TIM3->CCR3 to modify PWM duty cycle
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  // TODO: Start TIM3 in PWM mode on channel 3

  	  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  // TODO: Start TIM2 in Output Compare (OC) mode on channel 1.

  	  HAL_TIM_OC_Init(&htim2);
  	  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);

  // TODO: Start DMA in IT mode on TIM2->CH1; Source is LUT and Dest is TIM3->CCR3; start with Sine LUT

  	  HAL_DMA_Init(&hdma_tim2_ch1);
  	  HAL_DMA_Start_IT(&hdma_tim2_ch1, (uint32_t)Sin_LUT, DestAddress, NS);

  // TODO: Write current waveform to LCD ("Sine")

  	  delay(1000);
  	  init_LCD();
  	  lcd_putstring("Sine");

  // TODO: Enable DMA (start transfer from LUT to CCR)

  	  __HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);

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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(8000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

  /**/
  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

  /**/
  LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

  /**/
  LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

  /**/
  EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
  EXTI_InitStruct.LineCommand = ENABLE;
  EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
  EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
  LL_EXTI_Init(&EXTI_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
	// TODO: Debounce using HAL_GetTick()


	if(HAL_GetTick()> 100) //delay of 100 ticks
	{

		// TODO: Disable DMA transfer and abort IT, then start DMA in IT mode with new LUT and re-enable transfer
		// HINT: Consider using C's "switch" function to handle LUT changes

		__HAL_TIM_DISABLE_DMA(&htim2, TIM_DMA_CC1);
		HAL_DMA_Abort_IT(&hdma_tim2_ch1);

		switch(source)
		{

		case 0:

			//(uint32_t)&hdma_tim2_ch1.Instance->CMAR = saw_LUT;
			delay(1000);
			lcd_command(CLEAR);
			lcd_putstring("Sawtooth");
			HAL_DMA_Init(&hdma_tim2_ch1);
			HAL_DMA_Start_IT(&hdma_tim2_ch1, *saw_LUT, DestAddress, NS);
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
			source = source + 1;
	  	break;

		case 1:

			//(uint32_t)&hdma_tim2_ch1.Instance->CMAR = triangle_LUT;
			delay(1000);
			lcd_command(CLEAR);
			lcd_putstring("Triangle");
			HAL_DMA_Init(&hdma_tim2_ch1);
			HAL_DMA_Start_IT(&hdma_tim2_ch1, *triangle_LUT, DestAddress, NS);
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
			source = source + 1;

	  	break;
		case 2:

			//Sin_LUT = (uint32_t)&hdma_tim2_ch1.Instance->CMAR;
			delay(1000);
			lcd_command(CLEAR);
			lcd_putstring("Sine");
			HAL_DMA_Init(&hdma_tim2_ch1);
			HAL_DMA_Start_IT(&hdma_tim2_ch1, *Sin_LUT, DestAddress, NS);
			__HAL_TIM_ENABLE_DMA(&htim2, TIM_DMA_CC1);
			source = 0;

		break;

		}
		HAL_GPIO_EXTI_IRQHandler(Button0_Pin); // Clear interrupt flags
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
