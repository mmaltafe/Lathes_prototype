/* USER CODE BEGIN Header */
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
#include "stdio.h"
#include "stdint.h"
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
 typedef enum
 {
   AQ_BREAK       = 0x00U,
   AQ_INIT        = 0x01U,
   AQ_ENVIA       = 0x02U,
 } Aq_StatusTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

SRAM_HandleTypeDef hsram4;

/* USER CODE BEGIN PV */
Aq_StatusTypeDef AQST;
uint16_t valor=0;
uint8_t RawData[28672];
uint8_t RawData1[28672];
float Taxa_Aq_Hz = 12000;
float Presc = 20;
float Count;   //Count= ((1/Taxa_Aq_Hz)/(1/(168000000/(Presc+1))));
int Count_Tmr1;   //Count_Tmr1 =  floorf(Count);
int i = 0;
int j=0;
int w=0;
int z=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_FSMC_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void AD7606_Reset(void);
void AD7606_StartConvst(void);
void AD7606_SetOS(void);
void AD7606_SetInputRange(void);
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

	Count_Tmr1 =  floorf(Count= ((1/Taxa_Aq_Hz)/(1/(168000000/(Presc+1)))));
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
  MX_TIM1_Init();
  MX_FSMC_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
 //HAL_TIM_Base_Start_IT(&htim1);
 //HAL_TIM_Base_Start_ITBREAK(&htim1);
  AD7606_Reset();
  AD7606_SetOS();
  AD7606_SetInputRange();
  AQST= AQ_BREAK;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  if(AQST == AQ_INIT)
	  {
		  HAL_GPIO_WritePin(GPIOF, D1_Pin, GPIO_PIN_RESET);

			HAL_SRAM_Read_16b(&hsram4,(uint32_t *) 0x6C000000, &valor, 1);
			RawData[i]=((valor>>8) & 255);
			RawData[i+1]=(valor & 255);
			//RawData[i]=1;
			//RawData[i+1]=2;

			HAL_SRAM_Read_16b(&hsram4,(uint32_t *) 0x6C000000, &valor, 1);
			RawData[i+2]=((valor>>8) & 255);
			RawData[i+3]=(valor & 255);
			//RawData[i+2]=3;
			//RawData[i+3]=4;

			HAL_SRAM_Read_16b(&hsram4,(uint32_t *) 0x6C000000, &valor, 1);
			RawData[i+4]=((valor>>8) & 255);
			RawData[i+5]=(valor & 255);
			//RawData[i+4]=5;
			//RawData[i+5]=6;

			HAL_SRAM_Read_16b(&hsram4,(uint32_t *) 0x6C000000, &valor, 1);
			RawData[i+6]=((valor>>8) & 255);
			RawData[i+7]=(valor & 255);
			//RawData[i+6]=7;
			//RawData[i+7]=8;

			HAL_SRAM_Read_16b(&hsram4,(uint32_t *) 0x6C000000, &valor, 1);
			RawData[i+8]=((valor>>8) & 255);
			RawData[i+9]=(valor & 255);
			//RawData[i+8]=9;
			//RawData[i+9]=10;

			HAL_SRAM_Read_16b(&hsram4,(uint32_t *) 0x6C000000, &valor, 1);
			RawData[i+10]=((valor>>8) & 255);
			RawData[i+11]=(valor & 255);
			//RawData[i+10]=11;
			//RawData[i+11]=12;

			HAL_SRAM_Read_16b(&hsram4,(uint32_t *) 0x6C000000, &valor, 1);
			RawData[i+12]=((valor>>8) & 255);
			RawData[i+13]=(valor & 255);
			//RawData[i+12]=13;
			//RawData[i+13]=14;

			HAL_SRAM_Read_16b(&hsram4,(uint32_t *) 0x6C000000, &valor, 1);
			RawData[i+14]=((valor>>8) & 255);
			RawData[i+15]=(valor & 255);
			//RawData[i+14]=15;
			//RawData[i+15]=16;
			i=i+16;
			w=w+1;
			AQST= AQ_BREAK;

			if(w>1791)
			{

				HAL_TIM_Base_Stop_IT(&htim1);
				HAL_GPIO_WritePin(GPIOF, D1_Pin, GPIO_PIN_SET);
				//for(j=0;j<28672;j++)
				//{
					//RawData1[j]= RawData[j];
				//}
				AQST= AQ_ENVIA;
			}

	  }
	  else if(AQST== AQ_ENVIA)
	  {

		  HAL_GPIO_WritePin(GPIOG, Init_Aq_Pin, GPIO_PIN_SET);
		 // if(HAL_GPIO_ReadPin(GPIOB, Init_Conv_Pin) == GPIO_PIN_SET)
		 // {
		  while (HAL_GPIO_ReadPin(GPIOB, Init_Conv_Pin) == GPIO_PIN_SET)
		 {
			  HAL_SPI_Transmit(&hspi1, RawData, 28672, 2000);
		 }


			  HAL_GPIO_WritePin(GPIOG, Init_Aq_Pin, GPIO_PIN_RESET);

		 // }




		  i=0;
		  w=0;
		 // HAL_TIM_Base_Start_IT(&htim1);
		  HAL_GPIO_WritePin(GPIOF, D1_Pin, GPIO_PIN_SET);
		  AQST= AQ_BREAK;
	  }
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
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 20;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 666;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, D1_Pin|D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, OS1_Pin|OS2_Pin|RANGE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OS0_GPIO_Port, OS0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Init_Aq_GPIO_Port, Init_Aq_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CONVST_GPIO_Port, CONVST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : K1_Pin K0_Pin */
  GPIO_InitStruct.Pin = K1_Pin|K0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : D1_Pin D2_Pin */
  GPIO_InitStruct.Pin = D1_Pin|D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : OS1_Pin OS2_Pin RANGE_Pin */
  GPIO_InitStruct.Pin = OS1_Pin|OS2_Pin|RANGE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OS0_Pin */
  GPIO_InitStruct.Pin = OS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OS0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Init_Aq_Pin */
  GPIO_InitStruct.Pin = Init_Aq_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(Init_Aq_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : FRSTD_Pin */
  GPIO_InitStruct.Pin = FRSTD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FRSTD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CONVST_Pin */
  GPIO_InitStruct.Pin = CONVST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(CONVST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Init_Conv_Pin */
  GPIO_InitStruct.Pin = Init_Conv_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Init_Conv_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void)
{

  /* USER CODE BEGIN FSMC_Init 0 */

  /* USER CODE END FSMC_Init 0 */

  FSMC_NORSRAM_TimingTypeDef Timing = {0};

  /* USER CODE BEGIN FSMC_Init 1 */

  /* USER CODE END FSMC_Init 1 */

  /** Perform the SRAM4 memory initialization sequence
  */
  hsram4.Instance = FSMC_NORSRAM_DEVICE;
  hsram4.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
  /* hsram4.Init */
  hsram4.Init.NSBank = FSMC_NORSRAM_BANK4;
  hsram4.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
  hsram4.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
  hsram4.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
  hsram4.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
  hsram4.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
  hsram4.Init.WrapMode = FSMC_WRAP_MODE_DISABLE;
  hsram4.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
  hsram4.Init.WriteOperation = FSMC_WRITE_OPERATION_DISABLE;
  hsram4.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
  hsram4.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
  hsram4.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
  hsram4.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
  hsram4.Init.PageSize = FSMC_PAGE_SIZE_NONE;
  /* Timing */
  Timing.AddressSetupTime = 4;
  Timing.AddressHoldTime = 15;
  Timing.DataSetupTime = 15;
  Timing.BusTurnAroundDuration = 15;
  Timing.CLKDivision = 16;
  Timing.DataLatency = 17;
  Timing.AccessMode = FSMC_ACCESS_MODE_A;
  /* ExtTiming */

  if (HAL_SRAM_Init(&hsram4, &Timing, NULL) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FSMC_Init 2 */

  /* USER CODE END FSMC_Init 2 */
}

/* USER CODE BEGIN 4 */
void AD7606_Reset(void)
{
	HAL_GPIO_WritePin(GPIOB, RESET_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, RESET_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, RESET_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, RESET_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, RESET_Pin, 0);
}

void AD7606_StartConvst(void)
{

	HAL_GPIO_WritePin(GPIOB, CONVST_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, CONVST_Pin, 0);
	HAL_GPIO_WritePin(GPIOB, CONVST_Pin, 1);

}

void AD7606_SetOS(void)
{
	HAL_GPIO_WritePin(GPIOA, OS0_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, OS1_Pin, 0);
	HAL_GPIO_WritePin(GPIOC, OS2_Pin, 0);

}


void AD7606_SetInputRange(void)
{

	HAL_GPIO_WritePin(GPIOC, RANGE_Pin, 1);

}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == BUSY_Pin)

  {
	  AQST= AQ_INIT;
  }
  if(GPIO_Pin == Init_Conv_Pin)
  {


	  HAL_TIM_Base_Start_IT(&htim1);

  }

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{


  if (htim->Instance == TIM1) {

		AD7606_StartConvst();

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
