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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_LEVEL 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* Definitions for buttonControl */
osThreadId_t buttonControlHandle;
const osThreadAttr_t buttonControl_attributes = {
  .name = "buttonControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ledControl */
osThreadId_t ledControlHandle;
const osThreadAttr_t ledControl_attributes = {
  .name = "ledControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for serialControl */
osThreadId_t serialControlHandle;
const osThreadAttr_t serialControl_attributes = {
  .name = "serialControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
uint8_t msg[100];
uint8_t game[] = {1, 4, 3, 2, 3, 1, 4, 3, 2, 1, 4, 2, 3, 4, 2};
uint8_t isReading = 0;
uint8_t gameLevel = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartButtonControl(void *argument);
void StartLedControl(void *argument);
void StartSerialControl(void *argument);

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of buttonControl */
  buttonControlHandle = osThreadNew(StartButtonControl, NULL, &buttonControl_attributes);

  /* creation of ledControl */
  ledControlHandle = osThreadNew(StartLedControl, NULL, &ledControl_attributes);

  /* creation of serialControl */
  serialControlHandle = osThreadNew(StartSerialControl, NULL, &serialControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED4_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BOT1_Pin BOT2_Pin BOT3_Pin BOT4_Pin */
  GPIO_InitStruct.Pin = BOT1_Pin|BOT2_Pin|BOT3_Pin|BOT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED4_Pin LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartButtonControl */
/**
  * @brief  Function implementing the buttonControl thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartButtonControl */
void StartButtonControl(void *argument)
{
  /* USER CODE BEGIN 5 */
  int wrongSequence;
  /* Infinite loop */
  for(;;)
  {
    if(isReading)
    {
      wrongSequence = 0;
      for(int i = 0; i<gameLevel && !wrongSequence && gameLevel!=MAX_LEVEL+1;)
      {
        if(HAL_GPIO_ReadPin(GPIOC, BOT1_Pin))
        {
            if(game[i]!=4)
            	wrongSequence = 1;
            else
            	i++;
            while (HAL_GPIO_ReadPin(GPIOC, BOT1_Pin)){osDelay(30);}
        }
        else if(HAL_GPIO_ReadPin(GPIOC, BOT2_Pin))
        {
            if(game[i]!=3)
            	wrongSequence = 1;
            else
            	i++;
            while (HAL_GPIO_ReadPin(GPIOC, BOT2_Pin)){osDelay(30);}
        }
        else if(HAL_GPIO_ReadPin(GPIOC, BOT3_Pin))
        {
            if(game[i]!=2)
            	wrongSequence = 1;
            else
            	i++;
            while (HAL_GPIO_ReadPin(GPIOC, BOT3_Pin)){osDelay(30);}
        }
        else if(HAL_GPIO_ReadPin(GPIOC, BOT4_Pin))
        {
            if(game[i]!=1)
            	wrongSequence = 1;
            else
            	i++;
            while (HAL_GPIO_ReadPin(GPIOC, BOT4_Pin)){osDelay(30);}
        }
        else{osDelay(30);}
      }
      gameLevel = wrongSequence ? 1 : (gameLevel+1);
      isReading = 0;
      osDelay(1);
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedControl */
/**
* @brief Function implementing the ledControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedControl */
void StartLedControl(void *argument)
{
  /* USER CODE BEGIN StartLedControl */
  /* Infinite loop */
  for(;;)
  {
    if (!isReading){
      if(gameLevel!=MAX_LEVEL+1)
      {
          for(int x=0;x<gameLevel;x++){
            HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_RESET);
            switch (game[x])
            {
            case 1:
              HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_SET);
              break;
            case 2:
              HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_SET);
              break;
            case 3:
              HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_SET);
              break;
            case 4:
              HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_SET);
              break;
            }
            osDelay(1000);
          }
          HAL_GPIO_WritePin(GPIOB, LED1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, LED2_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, LED3_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_RESET);
          isReading=1;
      }
      else
      {
    	  HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
    	  HAL_GPIO_TogglePin(GPIOB, LED2_Pin);
    	  HAL_GPIO_TogglePin(GPIOB, LED3_Pin);
    	  HAL_GPIO_TogglePin(GPIOB, LED4_Pin);
          osDelay(500);
      }
    }
    osDelay(1);
  }
  /* USER CODE END StartLedControl */
}

/* USER CODE BEGIN Header_StartSerialControl */
/**
* @brief Function implementing the serialControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerialControl */
void StartSerialControl(void *argument)
{
  /* USER CODE BEGIN StartSerialControl */
  int level = 0;
  /* Infinite loop */
  for(;;)
  {
	if(level!=gameLevel)
	{
		if(gameLevel!=MAX_LEVEL+1)
		{
			sprintf((char *) msg, "Dificuldade: %d\r\n", gameLevel);
			HAL_UART_Transmit(&huart2, msg, strlen((const char *)msg), 10);
		}
		else
		{
			sprintf((char *) msg, "PARAB%cNS! Voc%c ganhou. Pressione Reset para iniciar um novo jogo.\r\n",144,136);
			HAL_UART_Transmit(&huart2, msg, strlen((const char *)msg), 10);
		}
		level = gameLevel;
	}
    osDelay(1);
  }
  /* USER CODE END StartSerialControl */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
