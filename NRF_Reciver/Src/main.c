/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "RF24.h"
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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin	== IRQ_Pin)
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"IRQ\n", strlen("IRQ\n"), 1000);
	}
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init(); // счётчик для микросекундных пауз

  //const uint64_t pipe0 = 0x787878787878;
  //const uint64_t pipe1 = 0xE8E8F0F0E2LL; // адрес первой трубы
  //const uint64_t pipe2 = 0xE8E8F0F0A2LL;
  //const uint64_t pipe3 = 0xE8E8F0F0D1LL;
  //const uint64_t pipe4 = 0xE8E8F0F0C3LL;
  //const uint64_t pipe5 = 0xE8E8F0F0E7LL;
  const uint64_t pipe1 = 0xE8E8F0F0ABLL;

  uint8_t res = isChipConnected(); // проверяет подключён ли модуль к SPI

  char str[64] = {0,};
  snprintf(str, 64, "Connected: %s\n", 1 ? "OK" : "NOT OK");
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

  res = NRF_Init(); // инициализация

  snprintf(str, 64, "Init: %s\n", res > 0 && res < 255 ? "OK" : "NOT OK");
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

  ////////////// SET ////////////////
  setAutoAck(false);				//автоподтверждение
  ///setAutoAckPipe(1, true);		//автоподтверждение для 1-й трубы
  ///enableAckPayload();			//разрешить добавлять полезную нагрузку в атоподтверждении
  enableAckPayload();  // Разрешение отправки нетипового ответа передатчику;
  // Автоматически включает динамический размер полезной нагрузки для труб №0 и №1,
  //для остальных труб нужно запускать — enableDynamicPayloads().
  ///enableDynamicPayloads();  //включить динамический размер полезной нагрузки.
  ///disableDynamicPayloads();   //oтключает динамическую полезную нагрузку во всей системе.
  //disableCRC(); 				// отключить CRC,
  // перед этим запустить setAutoAck(false) и disableDynamicPayloads().
  // setRetries(2, 3);			//кол-во повторных попыток при неудачной отправке
  setDataRate(RF24_250KBPS); 		//скорость передачи данных
  //RF24_250KBPS
  //RF24_1MBPS
  //RF24_2MBPS
  setPALevel(RF24_PA_MAX);		//мощность передатчика
  //RF24_PA_MIN = -18dBm
  //RF24_PA_LOW = -12dBm
  //RF24_PA_HIGH = -6dBM
  //RF24_PA_MAX = 0dBm
  setCRCLength(RF24_CRC_16);
  //RF24_CRC_8
  //RF24_CRC_16
  setChannel(0x5); 				//частотный канал
  ///setPayloadSize(6);			//размер полезной нагрузки в байтах
  //при отключённой динамической полезной нагрузке
  //powerDown();				//переводит модуль в режим низкого энергопотребления
  //powerUp();					//возвращает к жизни
  setAddressWidth(5);			//установить длину идентификатора трубы
  //3 — 24 бита
  //4 — 32 бита
  //5 — 40 бит
  ///openWritingPipe(pipe1);   //открытие трубы на передатчике (Только для передатчика)
  openReadingPipe(1, pipe1);	//открыть трубу №1 на приёмнике. В скобках номер трубы и индетификатор
//Если нужно открыть несколько труб, то просто добавить функции с другим номером и идентификатором.
  startListening(); 			//включить прослушивание труб на приёмнике
  ///////////////////////////////////

  ////////////////////////// Вывод всяких статусов, для работы не нужно /////////////////////////////
  uint8_t status = get_status();
  snprintf(str, 64, "get_status: 0x%02x\n", status);
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

  status = getPALevel();
  snprintf(str, 64, "getPALevel: 0x%02x  ", status);
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x00)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_PA_MIN\n", strlen("RF24_PA_MIN\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_PA_LOW\n", strlen("RF24_PA_LOW\n"), 1000);
  }
  else if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_PA_HIGH\n", strlen("RF24_PA_HIGH\n"), 1000);
  }
  else if(status == 0x03)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_PA_MAX\n", strlen("RF24_PA_MAX\n"), 1000);
  }

  status = getChannel();
  snprintf(str, 64, "getChannel: 0x%02x № %d\n", status, status);
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

  status = getDataRate();
  snprintf(str, 64, "getDataRate: 0x%02x  ", status);
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_250KBPS\n", strlen("RF24_250KBPS\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_2MBPS\n", strlen("RF24_2MBPS\n"), 1000);
  }
  else
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_1MBPS\n", strlen("RF24_1MBPS\n"), 1000);
  }

  status = getPayloadSize();
  snprintf(str, 64, "getPayloadSize: %d\n", status);
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

  status = getCRCLength();
  snprintf(str, 64, "getCRCLength: 0x%02x  ", status);
  HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

  if(status == 0x00)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_CRC_DISABLED\n", strlen("RF24_CRC_DISABLED\n"), 1000);
  }
  else if(status == 0x01)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_CRC_8\n", strlen("RF24_CRC_8\n"), 1000);
  }
  else if(status == 0x02)
  {
	  HAL_UART_Transmit(&huart1, (uint8_t*)"RF24_CRC_16\n", strlen("RF24_CRC_16\n"), 1000);
  }
  /////////////////////////////////////////////////////////////////////////////////////////////////////
  maskIRQ(true, true, true); // маскируем прерывания
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	///////////////////////////////////// ПРИЁМ /////////////////////////////////////////////
//	uint8_t nrf_data[32] = {0,}; // буфер указываем максимального размера
	int16_t nrf_data[32] = {0,}; // буфер указываем максимального размера
//	static uint8_t remsg = 0;
	uint8_t	count=32;			//количество принимаемых байт
	uint8_t pipe_num = 1;
	/////////////////////////////////////////////////////////////////////////////////////////////////////
	uint32_t message = 111;  //Вот какой потенциальной длины сообщение - uint32_t!
	//туда можно затолкать значение температуры от датчика или еще что-то полезное.
	writeAckPayload( 1, &message, sizeof(message) ); // Грузим сообщение для автоотправки;
	/////////////////////////////////////////////////////////////////////////////////////////////////////

	if(available(&pipe_num)) // проверяем пришло ли что-то
	        {
			HAL_GPIO_TogglePin(LEDPIN_GPIO_Port, LEDPIN_Pin);

			read(&nrf_data, count); // Читаем данные в массив nrf_data и указываем сколько байт читать

			snprintf(str, 64, "data[0]=%d data[1]=%d data[2]=%d data[3]%d\n", nrf_data[0], nrf_data[1], nrf_data[2],nrf_data[3]);
			HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);
//			HAL_UART_Transmit(&huart1, (uint8_t*)"pipe 1\n", strlen("pipe 1\n"), 1000);
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

  /** Initializes the CPU, AHB and APB busses clocks
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  huart1.Init.BaudRate = 1000000;
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDPIN_GPIO_Port, LEDPIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, CE_Pin|CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LEDPIN_Pin */
  GPIO_InitStruct.Pin = LEDPIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDPIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IRQ_Pin */
  GPIO_InitStruct.Pin = IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CE_Pin CSN_Pin */
  GPIO_InitStruct.Pin = CE_Pin|CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
