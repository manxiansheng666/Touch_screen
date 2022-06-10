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
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mbrtuslave.h"
#include "bsp.h"
#include "MultiTimer.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MCUDRECEIVELENGTH 256
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

volatile bool IN[8];
volatile bool OUT[3];
bool pre_out0;
uint64_t time = 0;
MultiTimer timer1;//led状态指示灯
volatile float temper;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint64_t PlatformTicksGetFunc(void);
void Timer1Callback(MultiTimer* timer, void *userData);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
HAL_UART_Receive_DMA(&huart2,rx_buffer,BUFFER_SIZE);
MultiTimerInstall(PlatformTicksGetFunc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(OUT[0] == true)//手动
	  {
		if(pre_out0 == false)
			MultiTimerStop(&timer1);
		  
		HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,(GPIO_PinState)OUT[1]);
		HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,(GPIO_PinState)OUT[2]);
	  }
	  else if(OUT[0] == false)	//自动
	  {
		if(pre_out0 == true)
		{
			HAL_GPIO_WritePin(led1_GPIO_Port,led1_Pin,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led2_GPIO_Port,led2_Pin,GPIO_PIN_SET);
			MultiTimerStart(&timer1, 300, Timer1Callback, NULL);
		}
	  }
	  pre_out0 = OUT[0];
	  IN[0] = HAL_GPIO_ReadPin(key0_GPIO_Port,key0_Pin);
	  IN[1] = HAL_GPIO_ReadPin(key1_GPIO_Port,key1_Pin);
	  
	  //获取温度
	  HAL_ADC_Start(&hadc1);	
	  HAL_ADC_PollForConversion(&hadc1,10);
	  uint16_t AD_Value = HAL_ADC_GetValue(&hadc1);
	  float Vol_Value = AD_Value*(3.3/4096);
	  temper = (1.43 - Vol_Value)/0.0043 + 25;
	  
//	  char buf[10] = {0};	//打印输出测试
//	  snprintf(buf,10,"%.2f ",temper);
//	  uart2_send_by485(buf,strlen(buf));
	  MultiTimerYield();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint16_t respondLength = 0;
	uint8_t respondBytes[MCUDRECEIVELENGTH];
	if(huart2.Instance == USART2)
	{
	    rx_cpt_nums = BUFFER_SIZE - hdma_usart2_rx.Instance->CNDTR;
		respondLength = ParsingMasterAccessCommand(rx_buffer,respondBytes,rx_cpt_nums,1);
		if(respondLength != 65535)
        {
//			uart2_send_by485("hello",5);
            uart2_send_by485(respondBytes,respondLength);
		}
		HAL_UART_Receive_DMA(&huart2,rx_buffer,BUFFER_SIZE);
	}
}

void GetInputStatus(uint16_t startAddress,uint16_t quantity,bool *statusValue)
{
	for(int i = 0;i < quantity;i++)
	{
		statusValue[i] = IN[i + startAddress];
	}
}

void GetCoilStatus(uint16_t startAddress,uint16_t quantity,bool *statusList)
{
	for(int i = 0;i < quantity;i++)
	{
		statusList[i] = OUT[i + startAddress];
	}
}

void SetSingleCoil(uint16_t coilAddress,bool coilValue)
{
	OUT[coilAddress] = coilValue;
}

uint64_t PlatformTicksGetFunc(void)
{
    return time;
}

void GetInputRegister(uint16_t startAddress,uint16_t quantity,uint16_t *registerValue)
{
	transfer_4bytes((uint32_t*)&temper);
	uint16_t* buf = (uint16_t*)&temper;
	for(int i = 0;i < quantity;i++)
	{
		registerValue[i] = buf[startAddress + i];
	}
}

void Timer1Callback(MultiTimer* timer, void *userData)
{
    HAL_GPIO_TogglePin(led1_GPIO_Port,led1_Pin);
	HAL_GPIO_TogglePin(led2_GPIO_Port,led2_Pin);
    MultiTimerStart(timer, 300, Timer1Callback, userData);
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
