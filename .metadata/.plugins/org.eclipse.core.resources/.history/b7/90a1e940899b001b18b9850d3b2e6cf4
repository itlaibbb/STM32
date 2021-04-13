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
#include "string.h"
#include <stdio.h>
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
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t aTxBuffer[1]={0};	//буфер для передачи данных по SPI

#define cs_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET)
#define cs_strob() cs_reset();cs_set()

#define data_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)
#define data_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET)

#define spi_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define spi_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)
#define spi_strob() spi_reset();spi_set();spi_reset()



RTC_TimeTypeDef sTime = {0};
RTC_DateTypeDef DateToUpdate = {0};

volatile	int	count=0;
//volatile	int	c_num=0;
volatile	int c_key_dr=0;
volatile	int keyscan=0;
volatile	int key[]={0,0,0,0};
			int key_f[]={0,0,0,0};
			int	c_mde=0;			//режим работы: 0 - индикация времени, 1 - установка часов, 2 - установка минут
//			int	dindonf=0;
//volatile	uint8_t Hours_L=0;
//volatile	uint8_t Hours_H=0;
//volatile	uint8_t Minutes_L=0;
//volatile	uint8_t Minutes_H=0;
			int	Tmp=0;
			int co=0;

volatile	int Hours_L=0;
volatile	int Hours_H=0;
volatile	int Minutes_L=0;
volatile	int Minutes_H=0;
volatile	int Seconds_L=0;
volatile	int Seconds_H=0;

//			int	num_ind[]={0b0000111111111111,0b0010111111111111,0b0100111111111111,0b1000111111111111};
// volatile	int	num_ind[]={0b0001111111111111,0b0010111111111111,0b1000111111111111, 0b0100111111111111};

//volatile	uint8_t	sgm_out[4]={0,};	//массив для 4-х выводимых на индикатор цифр в коде 7seg
//volatile	int	sgm_out[4]={0,};	//массив для 4-х выводимых на индикатор цифр в коде 7seg
		uint8_t sgm_out[5]={0,};	//массив для 6-х выводимых на индикатор цифр в коде 7seg
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//Прерывание от TMR1
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM1) //проверяем какой таймер вызвал колбек
   {
//прерываемя каждую 0,5 мсек.
	 count++;
/*
	  GPIOB->ODR &= 0b0000111111111111;		//гасим все разряды
//	  GPIOB->ODR |= 0b0100111111111111;
	  GPIOB->ODR |= num_ind[c_num];			//

	  GPIOA->ODR &= 0b1111111110000000;		//сбрасываем младшие 7 бит А в 0
	  GPIOA->ODR |= sgm_out[c_num];			//устанавливаем значения сегментов
	  c_num++;
//

	  if(c_num==4)
	  {
		  c_num=0;
	  }
*/
	  c_key_dr++;
	  if(c_key_dr>=50)						//опрос кнопок с защитой от дребезга (значение 50 = 25 мсек)
	  {
		  keyscan=GPIOB->IDR;
		  	  if((keyscan & 1) == 0)
		  	  {
		  		key[0]=1;
		  	  }
		  	  else
		  	  {
		  		key[0]=0;
		  	  }
		  	  if((keyscan & 2) == 0)
		  	  {
		  		key[1]=1;
		  	  }
		  	  else
		  	  {
		  		key[1]=0;
		  	  }
		  	  if((keyscan & 1024) == 0)
		  	  {
		  		key[2]=1;
		  	  }
		  	  else
		  	  {
		  		key[2]=0;
		  	  }
		  	  if((keyscan & 2048) == 0)
		  	  {
		  		key[3]=1;
		  	  }
		  	  else
		  	  {
		  		key[3]=0;
		  	  }
		c_key_dr=0;
	  }

   }
}
//
//
//процедура заполнения массива кода клавиш
//void key_inp (void)
//{
//
//}
//Процедура подучения времени и даты из RTC
//
//Вход: null
//Выход:
// переменные int Minutes_H, Minutes_L, Hours_H Hours_L с единицами/десятками часов и минут
void rtc_get(void)
{
	///////////////////////////////////////////////////////////
	//Получаем значение времени из RTC
		  	  	  	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
	///////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////
	//Преобразуем значение времени в BCD формат
		  	  	  	Seconds_H=sTime.Seconds/10;
		  	  	  	Seconds_L=sTime.Seconds-(Seconds_H*10);
		            Minutes_H=sTime.Minutes/10;
		            Minutes_L=sTime.Minutes-(Minutes_H*10);
		            Hours_H=sTime.Hours/10;
		            Hours_L=sTime.Hours-(Hours_H*10);
	///////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////
	//Считываем значение даты (нужно для нормальной работы HAL_RTC)
		            HAL_RTC_GetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN);
	///////////////////////////////////////////////////////////
#ifdef debug_0
	//4 DEBUG
		            char trans_str[64] = {0,};
	//END 4 DEBUG
	//4 DEBUG
	//Выводим значение часов, минут и секунд в UART1
		            snprintf(trans_str, 63, "Time %d:%d:%d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
		            HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
	//END 4 DEBUG
	//4 DEBUG
	//Выводим BCD значения времени в UART1
		            snprintf(trans_str, 63, "H_H %d H_L %d M_H %d M_L %d\n", Hours_H, Hours_L, Minutes_H,  Minutes_L);
		            HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
	//END 4 DEBUG
	//4 DEBUG
	//	            snprintf(trans_str, 63, "Date %d-%d-20%d\n", DateToUpdate.Date, DateToUpdate.Month, DateToUpdate.Year);
	//	            HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
	//END 4 DEBUG
#endif
}
//
//
//Процедура преобразования BCD цифры в 7 сегментный код
//
//Вход: BCD цифра
//Выход 7 сегментны йкод цифры
//
uint8_t	bcd27seg (uint8_t bcd)
{
	static uint8_t sgm_m[]={
			0b00111111,			//0
			0b00000110,			//1
			0b01011011,			//2
			0b01001111,			//3
			0b01100110,			//4
			0b01101101,			//5
			0b01111101,			//6
			0b00000111,			//7
			0b01111111,			//8
			0b01101111,			//9
			0b00000000			//ZERRO
	};
	return (sgm_m[bcd]);
}
//
/*
//Процедура очистки индикатора
void	clr_all (void)
{
 	 aTxBuffer[0]=0x00;
 	 cs_set();
 	 for(int i=0;i<6;i++)
 	 {
 		 HAL_SPI_Transmit(&hspi1,(uint8_t*)aTxBuffer, 1, 500);
 	 }
//	    	 HAL_SPI_Transmit(&hspi1,(uint8_t*) sgm_out, 4, 5000);
 	 cs_strob();
//
}
*/
//Вывод цифры
void	num_out(int num)
{
		Tmp=1;
	 	cs_set();
	 	 for(int i=0;i<7;i++)
	 	 {
	 	 if(sgm_out[num] & (64/Tmp))
	 	{
	 		data_set();
	 	}
	 	else
	 	{
	 		data_reset();
	 	}
	 	spi_strob();
	 	Tmp=Tmp*2;
	 	 }
	    data_reset();
	    spi_strob();
}
//
//загружаем цифры для индикации
void	out_num	(void)
{
	  for(int i=0;i<6;i++)
	  {
	  num_out(i);
	  }
	    cs_strob();		//включаем индикацию
}
//
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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1); // передаём в функцию указатель на структуру TIM1
//	  clr_all();				//очистка дисплея

  for(int i=0;i<8;i++)
  	  {
  	  	data_reset();
  	 	spi_strob();
  	  }

  	    cs_strob();
  	    c_mde=0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  //
	  //КНОПКИ
	  //
	  //нажатие на KEY1 (установка режимов)
	  //
	  	  if((key[0]==1) && (key_f[0]==0))
	  	  {
	  		  key_f[0]=1;
	  		  c_mde++;					//переход на следующий режим
	  	  }
	  	  if((key[0]==0) && (key_f[0]==1))
	  	  {
	  		  key_f[0]=0;
	  	  }
	  	  if(c_mde==3)					//всего режимов 3
	  	  {
	  		  c_mde=0;					//если переход на 4-й режим, то возврат в режим 0
	  	  }
	  //
	  //
	  //нажатие на KEY2 (уменьшение)
	  //
	  	  if((key[1]==1) && (key_f[1]==0))
	  	  {
	  		  key_f[1]=1;
	  		  if(c_mde==1)				//уменьшение часов
	  		  {
	  			  if(sTime.Hours > 1)
	  			  {
	  				  sTime.Hours--;
	  			  }
	  			  else
	  			  {
	  				  sTime.Hours=24;
	  			  }
	  		  }
	  		  if(c_mde==2)				//уменьшение минут
	  		  {
	  			  if(sTime.Minutes > 1)
	  			  {
	  				  sTime.Minutes--;
	  			  }
	  			  else
	  			  {
	  				  sTime.Minutes=59;
	  			  }
	  		  }
	  		sTime.Seconds=0;
	  		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
	  	  }
	  	  if((key[1]==0) && (key_f[1]==1))
	  	  {
	  		  key_f[1]=0;
	  	  }
	  //
	  //
	  //нажатие на KEY3 (увеличение)
	  //
	  	  	  if((key[2]==1) && (key_f[2]==0))
	  	  	  {
	  	  		  key_f[2]=1;
	  	  		  if(c_mde==1)				//увеличение часов
	  	  		  {
	  	  			  if(sTime.Hours < 23)
	  	  			  {
	  	  				  sTime.Hours++;
	  	  			  }
	  	  			  else
	  	  			  {
	  	  				  sTime.Hours=0;
	  	  			  }
	  	  		  }
	  	  		  if(c_mde==2)				//увеличение минут
	  	  		  {
	  	  			  if(sTime.Minutes < 59)
	  	  			  {
	  	  				  sTime.Minutes++;
	  	  			  }
	  	  			  else
	  	  			  {
	  	  				  sTime.Minutes=0;
	  	  			  }
	  	  		  }
	  	  		sTime.Seconds=0;
	  	  		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN); // RTC_FORMAT_BIN , RTC_FORMAT_BCD
	  	  	  }
	  	  	  if((key[2]==0) && (key_f[2]==1))
	  	  	  {
	  	  		  key_f[2]=0;
	  	  	  }
	  //
	  //
	  //нажатие на KEY4 (переключение рекуператора)
	  //
	  	  	  if((key[3]==1) && (key_f[3]==0))
	  	  	  {
	  	  		  key_f[3]=1;
//////	  	  		  dindonf=!dindonf;
	  	  	  }
	  	  	  if((key[3]==0) && (key_f[3]==1))
	  	  	  {
	  	  		  key_f[3]=0;
	  	  	  }


	  //
	  //
	  //РЕЖИМЫ
	  //
	  //мигающие режимы (1,2)
	  //
	    if((count>=500) && (count<1000))
	  	{
	  	  if(c_mde==1)							//установка часов
	  	  {
	  		  sgm_out[5]=bcd27seg(11);			//преобразуем время в 7seg код
	  		  sgm_out[4]=bcd27seg(11);			//--""--
	  		  out_num();

	  	  }
	  	  if(c_mde==2)							//установка минут
	  	  {
	  		  sgm_out[3]=bcd27seg(11);			//преобразуем время в 7seg код
	  		  sgm_out[2]=bcd27seg(11);			//--""--
	  		  out_num();

	  	  }
	  	}
	  //





	  if(count>=1000)
  {
//1 раз в 0,5 секунды
	  count=0;
      HAL_GPIO_TogglePin(led13_GPIO_Port, led13_Pin); // переключаем пин мигалки в противоположное состояние
	  rtc_get();		//получаем текущее время и преобразуем его d BCD Hours_L, Hours_Н, Minutes_L, Minutes_Н
	  sgm_out[5]=bcd27seg(Hours_H);			//преобразуем время в 7seg код
	  sgm_out[4]=bcd27seg(Hours_L);			//--""--
	  sgm_out[3]=bcd27seg(Minutes_H);		//--""--
	  sgm_out[2]=bcd27seg(Minutes_L);		//--""--
	  sgm_out[1]=bcd27seg(Seconds_H);		//--""--
	  sgm_out[0]=bcd27seg(Seconds_L);		//--""--



//
	  out_num();							//вывод цифр для индикации
//
//Работа рекуператора
	    co++;
	    	if(co == 1)
	    	{
	    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);		//выдув
	    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	    	}
	    	if(co == 80)
	    	{
	    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //вдув
	    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	    		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

	    	}
	    	if(co == 160)
	    	{
	    		co = 0;
	    	}
//
//
  }

#ifdef debug_1
	//4 DEBUG
		            char trans_str[80] = {0,};
	//Выводим значение кнопок
		            snprintf(trans_str, 80, "key[0] = %d key[1] = %d key[2] = %d key[3] = %d keyscan = %d\n", key[0], key[1], key[2], key[3],keyscan);
		            HAL_UART_Transmit(&huart1, (uint8_t*)trans_str, strlen(trans_str), 1000);
#endif
//
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  //RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 17;
  sTime.Minutes = 38;
  sTime.Seconds = 0;

  //if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  //{
//    Error_Handler();
//  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 20;

  BKP->RTCCR |= 28;                                                                        //калибровка RTC
  RTC->PRLL  = 0x7FFE;                                                                    //Настроит делитель на 32768 (32767+1)

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  htim1.Init.Prescaler = 719;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  HAL_GPIO_WritePin(led13_GPIO_Port, led13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin|B_1A_Pin|B_1B_Pin
                          |A_1A_Pin|A_1B_Pin|LED_SOUND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_5|ST_CP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : led13_Pin */
  GPIO_InitStruct.Pin = led13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(led13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin B_1A_Pin B_1B_Pin
                           A_1A_Pin A_1B_Pin LED_SOUND_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|B_1A_Pin|B_1B_Pin
                          |A_1A_Pin|A_1B_Pin|LED_SOUND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY_1_Pin KEY_2_Pin KEY_3_Pin KEY_4_Pin */
  GPIO_InitStruct.Pin = KEY_1_Pin|KEY_2_Pin|KEY_3_Pin|KEY_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ST_CP_Pin */
  GPIO_InitStruct.Pin = ST_CP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(ST_CP_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
