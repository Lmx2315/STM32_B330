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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;
SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_tx;

TIM_HandleTypeDef htim4;

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};

/* USER CODE BEGIN PV */

volatile  uint16_t adcBuffer[5]; // Buffer for store the results of the ADC conversion
float adc_ch[12];

uint8_t transmitBuffer[32];
uint8_t receiveBuffer[32];

uint32_t TIMER1;
u32 TIMER_BP_PWM;
volatile u32  SysTickDelay; 

u32 FLAG_T1;
u32 FLAG_T2;
u8 FLAG_FAPCH_ON;

uint8_t RX_uBUF[1];

unsigned int timer_DMA2;
u8 flag_pachka_TXT; //
uint16_t  text_lengh;
uint8_t text_buffer[Bufer_size];

volatile char          rx_buffer1[RX_BUFFER_SIZE1];
volatile unsigned int rx_wr_index1,rx_rd_index1,rx_counter1;
volatile u8  rx_buffer_overflow1;


char sr[BUFFER_SR+1];
unsigned char lsr;
unsigned char lk;
unsigned int led_tick;


char  strng[buf_IO];
char      InOut[BUFFER_SR+1];
char      Word [buf_Word];    //
char DATA_Word [buf_DATA_Word];    //
char DATA_Word2[buf_DATA_Word];    //
 
char Master_flag; // 
char lsym;
char  sym;
char flag;
char    NB;
char Adress;  //
char packet_sum;
char crc,comanda;
      
char In_data[BUF_STR];
char ink1; //
char data_in;

u16 lenght;
u16 SCH_LENGHT_PACKET;
	
unsigned     int index1;
unsigned     char crc_ok;
unsigned     char packet_ok;
unsigned     char packet_flag;
unsigned     int indexZ; 
unsigned     int index_word;
unsigned     int index_data_word;
unsigned     int index_data_word2;
unsigned     int lenght_data;//
unsigned     char data_flag;
unsigned     char data_flag2;
unsigned     char FLAG_lenght;//
unsigned     int sch_lenght_data;
unsigned     char FLAG_DATA;
unsigned char FLAG_CW;
float time_uart; //
unsigned char flag_pcf;
char lsym1;
char pack_ok1;
char pack_sum1;
char sym1;


u32 sch_rx_byte;
  
u8  Adress_ATT1;
u8  Adress_ATT2;
u8  Adress_ATT3;
u8  Adress_ATT4;
  
u8  Adress_SWITCH;
u8  Adress_KONTROL_WIRE;
u8  Adress_KONTROL_1;
u8  Adress_KONTROL_2;

u8 PWRDN_DAC1;
u8 PWRDN_DAC2;
u8 PWRDN_ADC1;
u8 PWRDN_ADC2;

u8 RESET_DAC1;
u8 RESET_DAC2;
u8 RESET_ADC1;
u8 RESET_ADC2;

u8 EVENT_INT0=0;
u8 EVENT_INT1=0;
u8 EVENT_INT2=0;
u8 EVENT_INT3=0;
u8 EVENT_INT4=0;
u8 EVENT_INT5=0;
u8 EVENT_INT6=0;
u8 EVENT_INT7=0;
u8 FLAG_DMA_ADC=0;

u32 TEST_LED=0;
u64 TIME_SYS=0;//переменная хранить системное время в милисекундах
u32 TIME_TEST=0;
u32 ID_CMD=0;						//переменная хранить текущий ID наших квитанций 

u8 PWR_CHANNEL=255;

u8 LM_ID_CN[8];
//-----------------------------------------------------------------------------
//                           описание структур управления и квитанций
/* USER CODE END PV */

 Frame INVOICE[quantity_SENDER];//структура квитанций о состоянии дел, по числу потенциальный адресатов
 Frame FRM1[quantity_SENDER];	//принятый фрейм, по числу потенциальный адресатов
 SERVER SERV1;					//структура "Хранилище"
 ID_SERVER ID_SERV1;			//структура указатель для "Хранилище"
 CMD_RUN CMD1;
 ADR_SENDER ADDR_SNDR;			//структура хранит массив адресов ОТПРАВИТЕЛЕЙ 

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);

void Initialization_wiznet(void);

/* USER CODE BEGIN PFP */
u8 START_BP=0;

u8  PWR_072      (u8);
u8  LM_MFR_MODEL (u8);
float LM_TEMP    (u8);
float LM_v       (u8);
float LM_in_p    (u8);
float LM_in_i    (u8);
float LM_aux_u   (u8);
u8 TCA_WR    (u32);
u8 LM_MFR_ID (u8);
u8 TCA_test (void);
void UART_DMA_TX (void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  
  /*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;//
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;//
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  */
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV16;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
 
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

   /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 PC1 */
  GPIO_InitStruct.Pin   = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_1;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PE4 PE10 PE11 PE0 */
  GPIO_InitStruct.Pin   = GPIO_PIN_4|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_0;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB9 */
  GPIO_InitStruct.Pin   = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 PD12 PD14 
                           PD15 PD0 PD1 PD4 
                           PD7 */
  GPIO_InitStruct.Pin   = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4 
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 */
  GPIO_InitStruct.Pin  = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin  = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  
    /*Configure GPIO pin :  */
  GPIO_InitStruct.Pin  = GPIO_PIN_5|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin  = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE1 */
  GPIO_InitStruct.Pin  = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_IT_FALLING
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
//  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim4.Init.AutoReloadPreload = TIM_MASTERSLAVEMODE_DISABLE;//TIM_AUTORELOAD_PRELOAD_ENABLE
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}


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


void spi4send32 (u32 d) //32 бита
{
	u8 a1;
	u8 a2;
	u8 a3;
	u8 a4;
	
	u8 b;

  a1 = (d >> 24)&0xff;
  a2 = (d >> 16)&0xff;
  a3 = (d >>  8)&0xff;
  a4 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi4, &a1, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi4, &a2, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi4, &a3, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi4, &a4, &b,1, 5000);  
}

u8 spi4send8 (u8 d) //8 бит
{
	u8 a1;
	u8  b;

  a1 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi4, &a1, &b,1, 5000); 
  return b; 
}

u8 spi5send8 (u8 d) //8 бит
{
	u8 a1;
	u8  b;

  a1 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi5, &a1, &b,1, 5000); 
  return b; 
}

void Delay( unsigned int Val)  
{  
   SysTickDelay = Val;  
   while (SysTickDelay != 0) {};  
}
//----------------------------------------------
volatile void delay_us( uint32_t time_delay)
{	
	time_delay=time_delay*10;
    while(time_delay--)	;
} 
//-------------------------------------------
unsigned int leng ( char *s)
{
  unsigned  char i=0;
  while ((s[i]!='\0')&&(i<120)) { i++;}
  return i;
}

void Transf(const char* s)  // процедура отправки строки символов в порт
{
  u32 l=0;
  u32 i=0;
         
  if ((flag_pachka_TXT==0) )
  {
    l=strlen(s);
    if ((text_lengh+l)>Bufer_size-5) text_lengh=0u;
    for (i=text_lengh;i<(text_lengh+l);i++) text_buffer[i]=s[i-text_lengh];
    text_lengh=text_lengh+l;
  } 
}

void itoa(int val,  char *bufstr, int base) //
{
    u8 buf[32] = {0};
    int i = 30;
    int j;
    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];
    i++; j=0;
    while (buf[i]!=0){ bufstr[j]=buf[i]; i++; j++;}
}

void f_out (char s[],float a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%.2f",a);
   Transf(strng);
   Transf ("\r\n");
}

void d_out (char s[],int a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%d",a);
   Transf(strng);
   Transf ("\r");
}

void u_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
   Transf ("\r");
}

void un_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
}

void nu_out (char s[],u32 a)
{
   Transf (s);
   //itoa (a,strng,10);
   sprintf (strng,"%u",a);
   Transf(strng);
   //Transf ("\r\n");
}

void x32_out (char s[],u32 a)
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   Transf ("\r\n");
}

void x_out (char s[],u32 a)//было u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   Transf ("\r\n");
}

void xn_out (char s[],u32 a)//было u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);   
}

void in_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%d",c);
   Transf (" ");
   Transf(strng);   
}

void i_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%d",c);
   Transf (" ");
   Transf(strng);
   Transf ("\r\n");   
}

void hn_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%X",c);
   Transf (" ");
   if (c<0x10) 
   {
   Transf ("0");
   } 
   Transf(strng);   
}

void h_out (u64 a,u8 n)
{
   u8 c=0;
   c=(a>>n)&0xff;	
   sprintf (strng,"%X",c);
   Transf (" ");
   if (c<0x10) 
   {
   Transf ("0");
   } 
   Transf(strng);  
   Transf ("\r\n");   
}

void u64_out (char s[],u64 a)
{
   u64 z=0;	
   Transf (s);
   z=a>>32;
   a=a&0xffffffff;
   sprintf (strng,"%u",z);
   Transf(strng);
   sprintf (strng,"%u",a);
   Transf(strng);
   Transf ("\r");
}

void UART_IT_TX (void)
{
 uint16_t k;
/*
if ((flag_pachka_TXT==0)&&(text_lengh>1u))
{ 
    k = text_lengh;
  	HAL_UART_Transmit_IT(&huart1,text_buffer,k);
    text_lengh=0u;  //обнуление счётчика буфера 
    flag_pachka_TXT=1; //устанавливаем флаг передачи
  }
  */
}
 
void UART_DMA_TX (void)
{
 uint16_t k;

if (HAL_UART_GetState(&huart1)!=HAL_UART_STATE_BUSY_TX )
	{
		if ((flag_pachka_TXT==0)&&(text_lengh>1u)&&(timer_DMA2>250))
		 {
			k = text_lengh;
			HAL_UART_Transmit_DMA(&huart1,(uint8_t *)text_buffer,k);
			text_lengh=0u;  //обнуление счётчика буфера 
			flag_pachka_TXT=1; //устанавливаем флаг передачи
			timer_DMA2=0;
		  }
	}	
} 

void spisend32 (u32 d) //32 бита
{
	u8 a1;
	u8 a2;
	u8 a3;
	u8 a4;
	
	u8 b;
  //HAL_SPI_TransmitReceive(&hspi3, &address, &data, sizeof(data), 5000);

  a1 = (d >> 24)&0xff;
  a2 = (d >> 16)&0xff;
  a3 = (d >>  8)&0xff;
  a4 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi3, &a1, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a2, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a3, &b,1, 5000);
  HAL_SPI_TransmitReceive(&hspi3, &a4, &b,1, 5000);  
}

u8 spisend8 (u8 d) //8 бит
{
	u8 a1;
	u8  b;
  //HAL_SPI_TransmitReceive(&hspi3, &address, &data, sizeof(data), 5000);

  a1 = (d)      &0xff;
  
  HAL_SPI_TransmitReceive(&hspi3, &a1, &b,1, 5000); 
  return b; 
}
  
  
u64 FPGA_rSPI (u8 size,u8 adr)
{
   u64 d[8];
   u8 i,k;
   u8 adr_a=0;
   u64 value;
   
   k=size/8;
   adr_a=adr;

//   delay_us(1);
 
   spisend8 (adr_a); //
   for (i=0;i<(size/8);i++) d[i] = spisend8 (0);  //считываем данные
//   delay_us(1);  
   
   if (k==1) value =   d[0];
   if (k==2) value =  (d[0]<< 8)+ d[1];
   if (k==3) value =  (d[0]<<16)+(d[1]<< 8)+ d[2];
   if (k==4) value =  (d[0]<<24)+(d[1]<<16)+(d[2]<< 8)+ d[3];
   if (k==5) value =  (d[0]<<32)+(d[1]<<24)+(d[2]<<16)+(d[3]<< 8)+ d[4];
   if (k==6) value =  (d[0]<<40)+(d[1]<<32)+(d[2]<<24)+(d[3]<<16)+(d[4]<< 8)+ d[5];
   if (k==7) value =  (d[0]<<48)+(d[1]<<40)+(d[2]<<32)+(d[3]<<24)+(d[4]<<16)+(d[5]<< 8)+ d[6];
   if (k==8) value =  (d[0]<<56)+(d[1]<<48)+(d[2]<<40)+(d[3]<<32)+(d[4]<<24)+(d[5]<<16)+(d[6]<<8)+d[7];

   return value;
}
//-----------------------------------------------------------------
//             тестовый вывод "Хранилища"
void PRINT_SERV (void)
{
	u32 i=0;
	u32 j=0;
	Transf("\r\n");
	
	for (i=0;i<SIZE_SERVER;i++)
	{
		if (j<4) 
		{
			if (SERV1.MeM[i]<10) xn_out("  ",SERV1.MeM[i]); 
			else				 xn_out(" " ,SERV1.MeM[i]);
		    if (j==3) {Transf("\r\n");j=0;}	else j++;		
		}
	}	
	Transf("\r\n");	
}


void PRINT_SERV_ID (void)
{
	u32 i=0;
	
	Transf("\r\n");
	x_out("N_sch    :",ID_SERV1.N_sch);
	
	for (i=0;i<SIZE_ID;i++)
	{
		if (ID_SERV1.INDEX[i]!=0xffffffff) 
		{		
			Transf("\r\n");	
			u_out("INDEX    :",ID_SERV1.INDEX      [i]);
			x_out("CMD_TYPE :0x",ID_SERV1.CMD_TYPE [i]);
			x_out("SENDER_ID:0x",ID_SERV1.SENDER_ID[i]);
			u_out("TIME     :",ID_SERV1.TIME       [i]);			
		}
	}		
}
//----------------------------------------------------------
void info ()
{

}


u32 crc_input=0u; 
u32 crc_comp=0u;

u32 IO ( char* str)      // функция обработки протокола обмена
{

 unsigned int i=0;
	uint8_t z1=0;
  i = lenght;//длинна принятой пачки
  if (lenght==0) i = leng(str);
  lenght = 0;
 
  indexZ = 0;
  
  if ((time_uart>50u)||(SCH_LENGHT_PACKET>MAX_PL))
  {
	  //-------------------
		packet_flag=0; 
		//-------------------
		time_uart=0u;  //обнуление счётчика тайм аута
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;
  }
  
  while (i>0u)   //перегрузка принятого пакета в массив обработки
  
  {  

	if ((str[indexZ]==0x7e)&&(packet_flag==0))// обнаружено начало пакета
	  {  
		//-------------------
		packet_flag=1; 
		//-------------------
		time_uart=0u;  //обнуление счётчика тайм аута
		index1=0u; 
		crc_ok=0; 
		packet_ok=0; 
		index_word=0u; 
		index_data_word =1u;
		index_data_word2=1u;
		data_flag =0;
		data_flag2=0;
		FLAG_lenght=0u;
		lenght_data=0u;
		sch_lenght_data=0u;
		DATA_Word [0]=' ';
		DATA_Word2[0]=' ';
		FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;		
	  }

	 InOut[index1]=str[indexZ];
	 SCH_LENGHT_PACKET++;//подсчитываем длинну пакета
		 
	if (( InOut[index1]==';')&&(FLAG_DATA==0u)&&(packet_flag==1))  {packet_flag=0;packet_ok=1u;FLAG_CW=1u;}
    
	if (((InOut[index1]=='=')||(InOut[index1]==':'))&&(data_flag==0)) {data_flag=1u;FLAG_CW=1u;}

	if (( InOut[index1]=='.')&&(data_flag2==0)&&(FLAG_DATA==0))   {data_flag2=1u; FLAG_CW=1u;}
	
	if (( InOut[index1]=='$')&&(FLAG_lenght==0u)) {FLAG_lenght=2u;FLAG_CW=1u;}
    
	if ((index1>2u)&&(InOut[2]==' ')&&(FLAG_CW==0u)&&(FLAG_lenght<2u))  
            {
                             if   (data_flag!=1u) {Word[index_word]=InOut[index1];} // заполняем командное слово
                      
                             if  ((data_flag==1u)&&(data_flag2==0u))     DATA_Word[index_data_word]=InOut[index1];// заполняем  слово данных1
                             if  ((data_flag==1u)&&(data_flag2==1u))     DATA_Word2[index_data_word2]=InOut[index1]; // заполняем  слово данных2
                    
                             if  (data_flag!=1u)
                                     {if (index_word<buf_Word) index_word++;} 
                                   else 
                                            {
                                             if ((data_flag==1u)&&(data_flag2==0u)) if (index_data_word<buf_DATA_Word)  {index_data_word++;sch_lenght_data++;}
                                            
                                             if ((data_flag==1u)&&(data_flag2==1u)) if (index_data_word2<buf_DATA_Word) index_data_word2++;
                                            }
			}
	
		if ((FLAG_lenght==2u)&&(FLAG_CW==0u)) {lenght_data = (u8)(InOut[index1]);FLAG_lenght=1u;} //запоминаем длинну пакета данных после ":"
	
		if ((sch_lenght_data<lenght_data)&&(FLAG_lenght==1u)) FLAG_DATA = 1u; else {FLAG_DATA = 0u;}
	 
		if (index1<BUFFER_SR)  index1++;
		if (indexZ <BUFFER_SR)  indexZ ++;
		i--;
		FLAG_CW=0u;	
  }
 

if (packet_ok==1u) 
  {    
      if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   // проверка первого условия пакета - начало пакета
      if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   // проверка второго условия пакета - адресат назначения
 
if (crc_ok==0x3)  //обработка команд адресатом которых является хозяин 
{

	if (strcmp(Word,"JTAG_TST")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("принял JTAG_TST\r"    );
     Transf("\r"); 
	 TST ();
   } else	
if (strcmp(Word,"JTAG2_SCAN")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("принял JTAG_SCAN\r"    );
     Transf("\r"); 
	 //--------------------
		 u8 list[2]; //массив длин IR регистров - их число должно быть по числу устройств на шине 
			list[0]=10;
			list[1]=10;
			list[2]=10;
			list[3]=10;
			list[4]=10;
			list[5]=10;
			list[6]=10;
			list[7]=10;
			list[1]=0;
//	 crc_comp=jtag_scan(list,crc_comp); 
	 crc_comp=jtag_scan(NULL,crc_comp); 
   } else
if (strcmp(Word,"JTAG1_SCAN")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("принял JTAG_SCAN\r"    );
     Transf("\r"); 
     JTAG_SCAN();
   } else
if (strcmp(Word,"JTAG_ID")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("принял JTAG_ID\r"    );
     Transf("\r");  
     ID_recive (crc_comp);
   } else		 
if (strcmp(Word,"BatPG")==0) //
   {
      u_out ("принял BatPG:",0); 
	  crc_comp=DS4520_read();
	  u_out("POWER_GOOD:",crc_comp);
   } else		
if (strcmp(Word,"time")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
//    u_out ("принял time:",crc_comp); 
//	  u_out("TIME_SYS:",TIME_SYS);
	  u64_out("TIME_SYS:",TIME_SYS);
//	  u_out("TIMER1  :",TIMER1);
//	  u_out("TIME_TEST:",TIME_TEST);
	  
   } else	
	
if (strcmp(Word,"mem")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял mem:",crc_comp); 
	     PRINT_SERV();
   } else
if (strcmp(Word,"MSG")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял MSG:",crc_comp); 
	  MSG_SHOW ();
   } else
 if (strcmp(Word,"ID_SERV")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял ID_SERV:",crc_comp); 
	  STATUS_ID (&ID_SERV1);
   } else
	 if (strcmp(Word,"spi5")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял spi5:",crc_comp); 
	  CS_5_MK(0);
	  spi5send8(crc_comp);
	  CS_5_MK(1);
   } else	
	 if (strcmp(Word,"nss")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял nss:",crc_comp); 
	  NSS_4(crc_comp);
   } else
	 if (strcmp(Word,"eth_init")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял eth_init:",crc_comp); 
	  Set_network();
   } else
	 if (strcmp(Word,"tca_rst")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял tca_rst:",crc_comp); 
       RESET_TCA6424A_MK(crc_comp); 
   } else
	   
   if (strcmp(Word,"tca_test")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял tca_test:",crc_comp); 
	TEST_LED=crc_comp;
   } else
 if (strcmp(Word,"tca_w")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял tca_w:",crc_comp); 
      TCA_WR(crc_comp); 
   } else
 if (strcmp(Word,"enable_lm")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял enable_lm:",crc_comp); 
      ENABLE_LM25056_MK(crc_comp); 
   } else	
 if (strcmp(Word,"lm")==0) //
   {
	  crc_input =atoi  (DATA_Word); 
      u_out ("принял lm:",crc_input); 
      LM_MFR_MODEL(crc_input); //если есть ответ i2c - будет выводить данные!!!
	  /*
	  crc_comp=LM_in_p (crc_input);
	  u_out("p=",crc_comp);
	  crc_comp=LM_in_i (crc_input);
	  u_out("i=",crc_comp);
	  crc_comp=LM_aux_u(crc_input);
	  u_out("u=",crc_comp);
	  */
   } else
if (strcmp(Word,"lm_ID")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял lm_ID:",crc_comp); 
      LM_MFR_ID(crc_comp);
	  x_out("0:",LM_ID_CN[0]);
	  x_out("1:",LM_ID_CN[1]);
	  x_out("2:",LM_ID_CN[2]);
   } else
 if (strcmp(Word,"lm_temp")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял lm_temp:",crc_comp); 
	  crc_comp=LM_TEMP(crc_input);
      u_out("temp:",crc_comp);
   } else	
 if (strcmp(Word,"lm_v")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял lm_v:",crc_comp);       
	  crc_comp=LM_v(crc_input);;
      u_out("v:",crc_comp);
   } else	   
 if (strcmp(Word,"pwr_072")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял pwr_072:",crc_comp); 
      PWR_072(crc_comp);
   } else		
 if (strcmp(Word,"i2c_adr")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял i2c_adr:",crc_comp); 
      u_out("i2c:",HAL_I2C_IsDeviceReady(&hi2c1, crc_comp,1, 1000));
   } else	
 if (strcmp(Word,"start")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял start:",crc_comp); 
      START_BP=crc_comp;
   } else
 if (strcmp(Word,"pwm")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял pwm:",crc_comp); 
      PWM(crc_comp);
   } else
 if (strcmp(Word,"pwrdn")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял pwrdn:",crc_comp); 
      PWDN_4(crc_comp);
   } else
 if (strcmp(Word,"ADC_test")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял ADC_test:",crc_comp); 
      ADC_test ();
   } else
if (strcmp(Word,"help")==0)                     
   {
     Transf ("принял help\r\n"    );
     Menu1(0);
   } else
//-------------------i2c----------------	   
if (strcmp(Word,"i2c_status")==0)                     
   {
     Transf ("принял i2c_status\r\n"    );
	 crc_comp =atoi(DATA_Word);	
     if (HAL_I2C_IsDeviceReady(&hi2c1,crc_comp,1,10) ==HAL_OK) Transf("i2c_OK\r\n");else Transf("i2c_error\r\n");	 
   } else
if (strcmp(Word,"i2c_transmit")==0)                     
   {
     Transf ("принял i2c_transmit\r\n"    );
	 crc_comp =atoi(DATA_Word);	
 	 crc_input=atoi(DATA_Word2);
	 z1=crc_input;
     HAL_I2C_Master_Transmit(&hi2c1,crc_comp,&z1,1,10);
	 
   } else
if (strcmp(Word,"i2c_recive")==0)                     
   {
     Transf ("принял i2c_recive\r\n"    );
	 crc_comp =atoi(DATA_Word);	
     HAL_I2C_Master_Receive(&hi2c1,crc_comp,&z1,1,10);
	 x_out("data=",z1);
	 
   } else
if (strcmp(Word,"info")==0)                     
   {
     Transf ("принял info\r\n"    );
 
     info();
   } else

if (strcmp(Word,"menu")==0)                     
   {
     Transf ("принял menu\r\n"    );
     Menu1(0);
   }  
 } 
	  for (i=0u;i<buf_Word;i++)               Word[i]     =0x0;
      for (i=0u;i<buf_DATA_Word;  i++)   DATA_Word[i]     =0x0;
      for (i=0u;i<buf_DATA_Word;  i++)  DATA_Word2[i]     =0x0;  
      for (i=0u;i<BUFFER_SR;i++)  
      {
        InOut[i]     =0x0;
      }  
      
	  time_uart=0;  //обнуление счётчика тайм аута
      packet_flag=0; 
      index1=0u; 
      crc_ok=0; 
      i=0;
      packet_ok=0; 
      index_word=0u; 
      index_data_word=0u;
      data_flag=0;
      index_data_word2=0u;
      data_flag2=0;
      indexZ =0u;
      FLAG_lenght=0u;
      lenght_data=0u;
      sch_lenght_data=0u;
      FLAG_CW = 0u; //флаг управляющего байта, снимается сразу после исполнения
      FLAG_DATA = 0u;	  
      	  
      DATA_Word [0]=' ';
      DATA_Word2[0]=' ';
	  SCH_LENGHT_PACKET=0;
  }

  if ((packet_ok==1)&&(crc_ok==0x1))     //обработка команд адресатом которых является слейв

  {
    
    if (Master_flag==0)

      {            
         
      }
  }  
  return  crc_input;         
} 

void Menu1(char a) 
 {
//***************************************************************************
    int i;  
 
  for (i=0;i<20;i++) Transf("\r");    // очистка терминала
  for (i=0; i<20; i++) Transf ("-");  // вывод приветствия
  Transf("\r");
  Transf("..........Terminal Тестовой платы.........\r\n");
  Transf("\r");
  Transf("MENU :\r");
  Transf("-------\r");
  Transf("Расшифровка структуры команды:\r");
  Transf("~ - стартовый байт\r");
  Transf("1 - адрес абонента\r");
  Transf(";- конец пачки \r");
  Transf(".............. \r");
  Transf("---------------------------------------------\r\n");
  Transf("IP  :192.168.1.163 - IP адрес    блока\r");
  Transf("PORT:2054          - номер порта блока\r");
  Transf("~0 help; - текущее меню\r");
  Transf("~0 info; - информация \r");
  Transf("~0 dac1_init; - \r");
  Transf("~0 dac1_r:0;   - чтение регистра\r");
  Transf("~0 dac1_w:0.0; - запись регистра\r");
  Transf("~0 dac1_serdes_pll:1; - очистка регистра сигнала захвата PLL Serdes\r");
  Transf("~0 dac1_info:0; \r");
  Transf("~0 dac1_init:0; \r");
  Transf("~0 dac1_phy_wr:0; \r");
  Transf("~0 dac1_phy_info; \r");
  Transf("~0 lmk_sync; - sync на LMK\r");
  Transf("~0 init_lmk; - init на LMK\r");
  Transf("-------------------------------------------\r");
  Transf("\r");
  Transf("\r");
  Transf("++++++++++++++++++++\r");
  Transf("\r");
  Transf("\r");
  //for (i=0; i<64; i++) zputs ("*",1);  // вывод приветствия
  //for (i=0;i<10;i++) puts("\r",1);  // очистка терминала
  Transf("\r");
  //*******************************************************************************
}


char getchar1(void)
{
   uint8_t data;
   while (rx_counter1 == 0);
   data = rx_buffer1[ rx_rd_index1++ ];
   if (rx_rd_index1 == RX_BUFFER_SIZE1) rx_rd_index1 = 0;
    --rx_counter1;
    return data;
}
      
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 //-------------------------  
   rx_buffer1[rx_wr_index1++]= (uint8_t) (RX_uBUF[0]& 0xFF); //считываем данные в буфер, инкрементируя хвост буфера
   if ( rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0; //идем по кругу
   	 
	  if (++rx_counter1 == RX_BUFFER_SIZE1) //переполнение буфера
      {
        rx_counter1=0; //начинаем сначала (удаляем все данные)
        rx_buffer_overflow1=1;  //сообщаем о переполнении
      }
	  
 //--------------------------  
   HAL_UART_Receive_IT(&huart1,RX_uBUF,1);
}



void UART_conrol (void)
{
 u16 i=0;
 u16 j=0;

  if (rx_counter1!=0u)
    {   
      if (rx_counter1<BUFFER_SR) j = rx_counter1; else j=BUFFER_SR;

      for (i=0u;i<j; i++) 
         {
           sr[i]=getchar1();
           lenght=i+1;  
           if (sr[i]==';') {break;}
          }
            sr[lenght]=0x00;
            IO (sr);
        };
}


void LED (void)
{
	
	if ((TIMER1<100)&&(FLAG_T1==0)) 
	{
		
		VD3(1);
		VD4(0);
		VD5(0);
		FLAG_T1=1;
		if (TEST_LED!=0)	TCA_WR (TEST_LED);
		TEST_LED=TEST_LED<<1;
	}
	
	if ((TIMER1>200)&&(FLAG_T2==0)) 
	{
		VD3(0);
		VD4(1);
		VD5(0);
		FLAG_T2=1;
	}
	
	if ((TIMER1>400))
	{
		VD3(0);
		VD4(0);
		VD5(1);
	}

	if (TIMER1>500)	
	{ 
		TIMER1=0;
		FLAG_T1=0;
		FLAG_T2=0;
	}	
}



volatile  u32 adc_cntl=0;
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc1)

{
	adc_cntl++;
//	HAL_ADC_Start_IT(hadc1);
}

float DBm[335];

void Massiv_dbm(void)
{
u16 i;
float e;
 DBm[  9]=-25;
 DBm[ 10]=-24;
 DBm[ 11]=-23;
 DBm[ 12]=-22;
 DBm[ 13]=-21;
 DBm[ 15]=-20;
 DBm[ 24]=-15;
 DBm[ 41]=-10;
 DBm[ 70]=-5;
 DBm[130]= 0;
 DBm[230]= 5;
 DBm[330]= 8;

for (i=  0;i<  9;i++) { e=-600+(i- 9)*6  ;   DBm[i]= e;}
for (i=  9;i< 16;i++) { e=-250+(i- 9)*8.3;   DBm[i]= e;}
for (i= 15;i< 25;i++) { e=-200+(i-15)* 5;    DBm[i]= e;}
for (i= 24;i< 42;i++) { e=-150+(i-24)*2.7;   DBm[i]= e;}
for (i= 41;i< 71;i++) { e=-100+(i-41)*1.7;   DBm[i]= e;}
for (i= 70;i<131;i++) { e=-50 +(i- 70)*0.83; DBm[i]= e;}
for (i=130;i<231;i++) { e=-0 +(i-130)*0.5;   DBm[i]= e;}
for (i=230;i<334;i++) { e= 50 +(i-230)*0.29; DBm[i]= e;}

for (i=0;i<334;i++) 
         {
           DBm[i]= DBm[i]/10;
//		   un_out("DBm[",i);
//	       f_out("]=",DBm[i]);
         }
}

double ADC_Temp(double t)
{
double TCelsius;	
TCelsius = ((t - 0.760) / 0.0025) + 25.0 ;
return TCelsius;
} 

void ADC_test (void)
{
//	u32 i=0;

	adc_cntl=0;	
	
	adc_ch[ 0] = ((float)adcBuffer[0])*3.3/4096*2;
	adc_ch[ 1] = ((float)adcBuffer[1])*3.3/4096*2;
	adc_ch[ 2] = ((float)adcBuffer[2])*3.3*5.71428/4096;//12 Вольт делитель 47к-10к
	adc_ch[ 3] = ((float)adcBuffer[3])*3.3/4096*2;
	adc_ch[ 4] = ((float)adcBuffer[4])*3.3/4096;
	
	
	Transf("\r\n---------\r\n");
	f_out("adc0_5.0V     :",adc_ch[0 ]);
	f_out("adc1_3.3V     :",adc_ch[1 ]);
	f_out("adc2_12V      :",adc_ch[2 ]);
	f_out("adc3_3.3V     :",adc_ch[3 ]);
	f_out("temp_sens(С)  :",ADC_Temp(adc_ch[4]));
	
	Transf("\r\r\r");
	u_out("adc0_5.0V     :",adcBuffer[0 ]);
	u_out("adc1_3.3V     :",adcBuffer[1 ]);
	u_out("adc2_12V      :",adcBuffer[2 ]);
	u_out("adc3_3.3V     :",adcBuffer[3 ]);
	u_out("temp_sens(С)  :",adcBuffer[4 ]);
	
}

u8 FLAG_WDG=0;

void DMA_ADC (void)
{
	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA  (&hadc1,(uint32_t*)&adcBuffer,5); // Start ADC in DMA 	
}

void WATCH_DOG (void)
{
	
	if (FLAG_WDG==0) {FLAG_WDG=1;} else FLAG_WDG=0;
	
	WDI_MK(FLAG_WDG);	
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
  
void BP_start (u16 a)
{
	static u8 step=0;
	
	if ((a==1)&&(step==0))
	{
		step=1;
	} else if ((step>4)&&(a==0)) step=0;
	
	if ((a==0)&&(step==0))
	{
		PWR_072 (255);//выключаем каналы питания и сбрасываем состояния переменных говорящих о каналах питания - в ноль
		PWM (0);		
	} 
	
	
	if (step==1)
	{
		TIMER_BP_PWM=100;
		step=2;
		Transf("step1\r\n");
		PWM (100);
	}
	
	if ((step==2)&&(TIMER_BP_PWM==0))
	{
		PWM (1000);
		TIMER_BP_PWM=100;
		step=3;
		Transf("step2\r");
	}
	
	if ((step==3)&&(TIMER_BP_PWM==0))
	{
		PWM (3000);
		TIMER_BP_PWM=100;
		step=4;
		Transf("step3\r");
	}
	
	if ((step==4)&&(TIMER_BP_PWM==0))
	{
		PWM (7000);
		TIMER_BP_PWM=100;
		step=5;
		Transf("step4\r");
	}
	
	if ((step==5)&&(TIMER_BP_PWM==0))
	{
		PWM (9000);
		TIMER_BP_PWM=100;
		step=6;
		Transf("step5\r");
		
		IO("~0 pwr_072:0;"); //подаём питание на все каналы!!! - без этого не работает i2c
			
		IO("~0 enable_lm:1;");//включаем все м/мы LM
	}
	
}  
  
void PWM (u16 a)
{
  sConfigOC.Pulse      = a;
  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;

  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4,TIM_CHANNEL_1);
}  


u8 TCA_test (void)
{
 return 0;
}

u8 TCA_WR (u32 z)
{
	uint16_t DevAddress=0x22;//адрес 
	 u8  a[4];
	 uint8_t state=0;
	 
	 HAL_I2C_Init(&hi2c1);
	
	a[0] =   0x8c;  //Configuration Port 0
	a[1] =   0x00;
	a[2] =   0x00;
	a[3] =   0x00;
	
	state=HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,4,1000);//конфигурируем порты мк как выходы
	
	a[0] =   0x84;  //Output Port 0
	
	a[1] = (z>> 0)&0xff;
	a[2] = (z>> 8)&0xff;
	a[3] = (z>>16)&0xff;
	
	state=HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,4,1000);
	
	HAL_I2C_DeInit(&hi2c1);

	return state;
}

u8 PWR_072 (u8 z)
{
	uint16_t DevAddress=0x53;//адрес DD10 ds4520
	 u8  a[2];
	 uint8_t state=0;
	 uint8_t v=0;
	 u32 error=0;
	
	HAL_I2C_Init(&hi2c1);	

	if (z&(1<<0)) v=v|(1<<6);
	if (z&(1<<1)) v=v|(1<<4);
	if (z&(1<<2)) v=v|(1<<5);
	if (z&(1<<3)) v=v|(1<<3);
	if (z&(1<<4)) v=v|(1<<2);
	if (z&(1<<5)) v=v|(1<<1);
	if (z&(1<<6)) v=v|(1<<0);
	if (z&(1<<7)) v=v|(1<<7);
	
	a[0] =   0xf0;  //команда-регистр включение подтяжки для 0-7 выходы
	a[1] =   v;
	
	HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,2,1000);
		
	a[0] =   0xf2;  //команда-регистр записи 0-7 выходы (либо ноль либо третье состояние!!!)
	a[1] =      v;
	
//	u_out("z:",z);
//	state=      HAL_I2C_IsDeviceReady  (&hi2c1,(DevAddress<<1),1,  1000);
		error = HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,2,1000);
//	if (state)  HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,2,1000);

		PWR_CHANNEL=z;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return state;
}

u8 DS4520_read (void)  //проверяем состояние входа расширителя - 15 нога расширителя DD10 (8порт)  - сигнал POWERGOOD от конденсаторов
{
	uint16_t DevAddress=0x53;//адрес DD10 ds4520
	 u8  c[1]; 
	 u8  a[2];
	 uint8_t state=0;
	 uint8_t v=0;
	 u32 error=0;
	
	HAL_I2C_Init(&hi2c1);	

	c[0] =   0xf9;  //команда-чтение I/O Status 1 
	
	state  = HAL_I2C_IsDeviceReady  (&hi2c1,(DevAddress<<1),1,  1000);
if (!state)  
	{
		HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,1,1);
	}
	
	HAL_I2C_DeInit(&hi2c1);	
	v=a[0]&0x01;	
	return v;
}



u8 LM_MFR_MODEL (u8 z)
{
	 uint16_t DevAddress=0x00;//
	 uint8_t a[32];
	 uint16_t Size2=8;
	 uint8_t state=1;
	 uint8_t c[2];
	 
	 c[0]=0x9a;//To read the manufacturer ID команда-чтение manufacturer ID in ASCII characters (NSC).
	 
	HAL_I2C_Init(&hi2c1); 
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-го канала
	if (z==7)  DevAddress=0x56;//LM25056 - 2-го канала
	if (z==6)  DevAddress=0x51;//LM25056 - 3-го канала
	if (z==5)  DevAddress=0x57;//LM25056 - 4-го канала
	if (z==4)  DevAddress=0x59;//LM25056 - 5-го канала
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-го канала
	if (z==2)  DevAddress=0x15;//LM25056 - 7-го канала
	if (z==1)  DevAddress=0x16;//LM25056 - 8-го канала
	
	state  = HAL_I2C_IsDeviceReady  (&hi2c1,(DevAddress<<1),1,  1000);
//	x_out("state:",state);
	if (!state)  
	{
		HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size2,1);
		
		x_out("a[0]=",a[0]);
		x_out("a[1]=",a[1]);
		x_out("a[2]=",a[2]);		
		x_out("a[3]=",a[3]);
		x_out("a[4]=",a[4]);
		x_out("a[5]=",a[5]);
		x_out("a[6]=",a[6]);
		x_out("a[7]=",a[7]);
	}	
	
	HAL_I2C_DeInit(&hi2c1);
	
	return state;
}



u8 LM_MFR_ID (u8 z)
{
	 uint16_t DevAddress=0x00;//
	 uint8_t a[32];
	 uint16_t Size2=3;
	 uint8_t state=1;
	 uint8_t v=0;
	 uint8_t c[2];
	 u32 error=0;
	 
	 HAL_I2C_Init(&hi2c1);
	 
	 c[0]=0x99;//To read the manufacturer ID команда-чтение manufacturer ID in ASCII characters (NSC).
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-го канала
	if (z==7)  DevAddress=0x56;//LM25056 - 2-го канала
	if (z==6)  DevAddress=0x51;//LM25056 - 3-го канала
	if (z==5)  DevAddress=0x57;//LM25056 - 4-го канала
	if (z==4)  DevAddress=0x59;//LM25056 - 5-го канала
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-го канала
	if (z==2)  DevAddress=0x15;//LM25056 - 7-го канала
	if (z==1)  DevAddress=0x16;//LM25056 - 8-го канала
	
//	state  = HAL_I2C_IsDeviceReady  (&hi2c1,(DevAddress<<1),1,  1000);
//	x_out("state:",state);
//	if (!state)  
	{
	 error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size2,1);
		
//		x_out("a[0]=",a[0]);
//		x_out("a[1]=",a[1]);
//		x_out("a[2]=",a[2]);
		if (error==1) 
		{
			LM_ID_CN[0]=0xff;
			LM_ID_CN[1]=0xff;
			LM_ID_CN[2]=0xff;
		} else
		{
			LM_ID_CN[0]=a[0];
			LM_ID_CN[1]=a[1];
			LM_ID_CN[2]=a[2];	
		}
	}	
	
	HAL_I2C_DeInit(&hi2c1);
	return state;
}

u8 D_TEMP[4];

float LM_TEMP (u8 z)
{
	 uint16_t DevAddress=0x00;//
	 uint8_t  a[32];
	 uint8_t  c[1];
	 uint16_t Size=2;//размер считываемого массива
	 uint8_t state=1;
	 uint8_t v=0;
	 u32 error=0;
	 int value=0;
	 
	 float x=0;
	 float m=1580;
	 float y=0;
	 float r=2;
	 float b=-14500;
	 int   p=0;
	 
	 HAL_I2C_Init(&hi2c1);
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-го канала
	if (z==7)  DevAddress=0x56;//LM25056 - 2-го канала
	if (z==6)  DevAddress=0x51;//LM25056 - 3-го канала
	if (z==5)  DevAddress=0x57;//LM25056 - 4-го канала
	if (z==4)  DevAddress=0x59;//LM25056 - 5-го канала
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-го канала
	if (z==2)  DevAddress=0x15;//LM25056 - 7-го канала
	if (z==1)  DevAddress=0x16;//LM25056 - 8-го канала
	
	c[0]=0x8d;//Retrieves temperature measurement.
	
//	state  = HAL_I2C_IsDeviceReady  (&hi2c1,(DevAddress<<1),1,  1000);
	
//	x_out("state:",state);
//	if (!state)  
	{
		error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size,1);
		
	//	x_out("a[0]=",a[0]);
	//	x_out("a[1]=",a[1]);
	//	x_out("a[2]=",a[2]);		
	//	x_out("a[3]=",a[3]);
	//	x_out("a[4]=",a[4]);
	//	x_out("a[5]=",a[5]);
	//	x_out("a[6]=",a[6]);
	//	x_out("a[7]=",a[6]);
	}	
	
	p=(a[1]<<8)+a[0];
	
//	x_out("p=",p);
	
	y=p;
	
//	f_out("y=",y);
	
	x=(1/m)*(y*(powf(10,r))-b);
	
//	f_out("t=",x);
	
	value=x*100;
	
	if (error==1) value=0xffffffff;
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
/* 	u_out("D_TEMP[0]",D_TEMP[0]);
	u_out("D_TEMP[1]",D_TEMP[1]);
	u_out("D_TEMP[2]",D_TEMP[2]);
	u_out("D_TEMP[3]",D_TEMP[3]); */
	
	HAL_I2C_DeInit(&hi2c1);
	
	return x;
}

float LM_v (u8 z)
{
	 uint16_t DevAddress=0x00;//
	 uint8_t  a[32];
	 uint8_t  c[1];
	 uint16_t Size=2;//размер считываемого массива
	 uint8_t state=1;
	 uint8_t v=0;
	 u32 error=0;
	 int value=0;
	 
	 float x=0;
	 float m=16296;
	 float y=0;
	 float r=2;
	 float b=1343;
	 int  p=0;
	 
	 HAL_I2C_Init(&hi2c1);
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-го канала
	if (z==7)  DevAddress=0x56;//LM25056 - 2-го канала
	if (z==6)  DevAddress=0x51;//LM25056 - 3-го канала
	if (z==5)  DevAddress=0x57;//LM25056 - 4-го канала
	if (z==4)  DevAddress=0x59;//LM25056 - 5-го канала
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-го канала
	if (z==2)  DevAddress=0x15;//LM25056 - 7-го канала
	if (z==1)  DevAddress=0x16;//LM25056 - 8-го канала
	
	c[0]=0x88;//Retrieves temperature measurement.
	
//	state  = HAL_I2C_IsDeviceReady  (&hi2c1,(DevAddress<<1),1,  1000);
	
//	x_out("state:",state);
//	if (!state)  
	{
		error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size,1);
		
	//	x_out("a[0]=",a[0]);
	//	x_out("a[1]=",a[1]);
	//	x_out("a[2]=",a[2]);		
	//	x_out("a[3]=",a[3]);
	//	x_out("a[4]=",a[4]);
	//	x_out("a[5]=",a[5]);
	//	x_out("a[6]=",a[6]);
	//	x_out("a[7]=",a[6]);
	}	
	
	p=(a[1]<<8)+a[0];
	
//	x_out("p=",p);
	
	y=p;
	
//	f_out("y=",y);
	
	x=(1/m)*(y*(powf(10,r))-b);
	
//	f_out("U=",x);
	
    value=x*100;
	
	if (error==1) value=0xffffffff;
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return x;
}
  
float LM_aux_u (u8 z)
{
	 uint16_t DevAddress=0x00;//
	 uint8_t  a[32];
	 uint8_t  c[1];
	 uint16_t Size=2;//размер считываемого массива
	 uint8_t state=1;
	 uint8_t v=0;
	 u32 error=0;
	 int value=0;
	 
	 float x=0;
	 float m=3416;
	 float y=0;
	 float r=0;
	 float b=-4;
	 int  p=0;
	 
	 HAL_I2C_Init(&hi2c1);
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-го канала
	if (z==7)  DevAddress=0x56;//LM25056 - 2-го канала
	if (z==6)  DevAddress=0x51;//LM25056 - 3-го канала
	if (z==5)  DevAddress=0x57;//LM25056 - 4-го канала
	if (z==4)  DevAddress=0x59;//LM25056 - 5-го канала
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-го канала
	if (z==2)  DevAddress=0x15;//LM25056 - 7-го канала
	if (z==1)  DevAddress=0x16;//LM25056 - 8-го канала
	
	c[0]=0xd0;//Retrieves auxiliary voltage measurement.

	error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size,1);
	
	p=(a[1]<<8)+a[0];
	y=p;

	x=(1/m)*(y*(powf(10,r))-b);
	
//	f_out("U=",x);
	
	value=x*100;
	
	if (error==1) value=0xffffffff;
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return x;
}

float LM_in_i (u8 z)
{
	 uint16_t DevAddress=0x00;//
	 uint8_t  a[32];
	 uint8_t  c[1];
	 uint16_t Size=2;//размер считываемого массива
	 uint8_t state=1;
	 uint8_t v=0;
	 u32 error=0;
	 int value=0;
	 
	 float x=0;
	 float m=13797;
	 float y=0;
	 float r=2;
	 float b=-1833;
	 int  p=0;
	 
	HAL_I2C_Init(&hi2c1);
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-го канала
	if (z==7)  DevAddress=0x56;//LM25056 - 2-го канала
	if (z==6)  DevAddress=0x51;//LM25056 - 3-го канала
	if (z==5)  DevAddress=0x57;//LM25056 - 4-го канала
	if (z==4)  DevAddress=0x59;//LM25056 - 5-го канала
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-го канала
	if (z==2)  DevAddress=0x15;//LM25056 - 7-го канала
	if (z==1)  DevAddress=0x16;//LM25056 - 8-го канала
	
	c[0]=0xd1;//Retrieves  input current measurement

	error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size,1);
	
	p=(a[1]<<8)+a[0];
	y=p;

	x=(1/m)*(y*(powf(10,r))-b);
	
//	f_out("I=",x);
	
	value=x*100;
	
	if (error==1) value=0xffffffff;
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return x;
}

float LM_in_p (u8 z)
{
	 uint16_t DevAddress=0x00;//
	 uint8_t  a[32];
	 uint8_t  c[1];
	 uint16_t Size=2;//размер считываемого массива
	 uint8_t state=1;
	 uint8_t v=0;
	 u32 error=0;
	 int value;
	 
	 float x=0;
	 float m=5501;
	 float y=0;
	 float r=3;
	 float b=-2908;
	 int  p=0;
	 
	HAL_I2C_Init(&hi2c1);
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-го канала
	if (z==7)  DevAddress=0x56;//LM25056 - 2-го канала
	if (z==6)  DevAddress=0x51;//LM25056 - 3-го канала
	if (z==5)  DevAddress=0x57;//LM25056 - 4-го канала
	if (z==4)  DevAddress=0x59;//LM25056 - 5-го канала
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-го канала
	if (z==2)  DevAddress=0x15;//LM25056 - 7-го канала
	if (z==1)  DevAddress=0x16;//LM25056 - 8-го канала
	
	c[0]=0xd2;//Retrieves   input power measurement.

	error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size,1);
	
	p=(a[1]<<8)+a[0];
	y=p;

	x=(1/m)*(y*(powf(10,r))-b);
	
//	f_out("P=",x);
	
	value=x*100;
	
	if (error==1) value=0xffffffff;
	
//	u_out("i2c:",error);
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return x;
}

u32 idx_srv(
u32 a,//текущий индекс команды в массиве хранилище
u32 k //смещение данных в байтах от их начального положения
) //функция высчитывает индекс данных в массиве "Хранилище" 
{
 u32 idx=0;
 idx=a+24+k;//
 if (idx>(SIZE_SERVER-1)) idx=idx-SIZE_SERVER;
 return idx;
}

void CMD_search (ID_SERVER *id,SERVER *srv)
{
	u32 i=0;
	u32 idx0=0;
	u32 idx1=0;
	u32 idx2=0;
	u32 idx3=0;
	u32 idx4=0;
	u32 idx5=0;
	u32 idx6=0;
	u32 idx7=0;
	
	u64 ADR=0;
	u32 data=0;
	u8 D[4];

	for (i=0;i<SIZE_ID;i++)
	{		
		if (id->CMD_TYPE[i]==CMD_TIME_SETUP) 
		{
			Transf("\r\n------\r\n");
			Transf("Команда:установка времени!\r\n");
			
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище"
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище"
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
			idx4=idx_srv(id->INDEX[i],4);//индекс расположения данных в "хранилище"
			idx5=idx_srv(id->INDEX[i],5);//индекс расположения данных в "хранилище"
			idx6=idx_srv(id->INDEX[i],6);//индекс расположения данных в "хранилище"
			idx7=idx_srv(id->INDEX[i],7);//индекс расположения данных в "хранилище"
			
			TIME_SYS=((u64)srv->MeM[idx0]<<56)|((u64)srv->MeM[idx1]<<48)|
					 ((u64)srv->MeM[idx2]<<40)|((u64)srv->MeM[idx3]<<32)|
					 ((u64)srv->MeM[idx4]<<24)|((u64)srv->MeM[idx5]<<16)|
					 ((u64)srv->MeM[idx6]<< 8)|((u64)srv->MeM[idx7]<< 0);
			IO("~0 time;");
			Transf("\r\n------\r\n");		
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда
			u_out("Номер отправителя:",ADR);
			ERROR_CMD_MSG ( //заполняем квитанцию о выполнении команды
			id,			    //указатель на реестр
			&INVOICE[ADR],  //указатель на структуру квитанции
			i, 			    //индекс команды в реестре
			MSG_CMD_OK,	    //сообщение квитанции
			0,				//данные квитанции
			TIME_SYS	    //текущее системное время 
			);	
			SERV_ID_DEL (id,i);//удаляем команду из реестра
		}
		
		if (id->CMD_TYPE[i]==CMD_STATUS)//команда запроса состояни
		{
			//нет данных у команды
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда

			ERROR_CMD_MSG ( //заполняем квитанцию о выполнении команды
			id,			    //указатель на реестр
			&INVOICE[ADR],  //указатель на структуру квитанции
			i, 			    //индекс команды в реестре
			MSG_CMD_OK,	    //сообщение квитанции
			0,				//данные квитанции
			TIME_SYS	    //текущее системное время 
			);	
			
			//------------------------------------
			if (START_BP==1)
			{
				LM_MFR_ID(1);
				
				SYS_CMD_MSG(
				id,				//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH1,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM_ID_CN,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_MFR_ID(2);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH2,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM_ID_CN,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_MFR_ID(3);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH3,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM_ID_CN,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_MFR_ID(4);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH4,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM_ID_CN,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_MFR_ID(5);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH5,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM_ID_CN,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				
				LM_MFR_ID(6);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH6,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM_ID_CN,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_MFR_ID(7);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH7,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM_ID_CN,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_MFR_ID(8);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH8,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM_ID_CN,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
			}
			//-----------------------------------
			if (START_BP==1)
			{
				LM_TEMP(1);
							
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH1,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_TEMP(2);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH2,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_TEMP(3);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH3,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_TEMP(4);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH4,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_TEMP(5);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH5,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_TEMP(6);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH6,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_TEMP(7);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH7,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				LM_TEMP(8);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH8,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
			}
			//-----------------------------------
			
			LM_in_p(1);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH1,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_p(2);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH2,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_p(3);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH3,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_p(4);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH4,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_p(5);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH5,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_p(6);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH6,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_p(7);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH7,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_p(8);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH8,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			//-----------------------------------
			
			LM_in_i(1);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH1,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_i(2);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH2,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_i(3);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH3,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_i(4);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH4,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_i(5);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH5,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_i(6);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH6,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_i(7);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH7,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_in_i(8);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH8,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			//-----------------------------------
			
			LM_v(1);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH1,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_v(2);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH2,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_v(3);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH3,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_v(4);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH4,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_v(5);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH5,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_v(6);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH6,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_v(7);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH7,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			LM_v(8);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH8,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			//-----------------------------------
			//квитанция о состоянии питания каналов
			D_TEMP[0]=0;
			D_TEMP[1]=0;
			D_TEMP[2]=START_BP;
			D_TEMP[3]=PWR_CHANNEL;
	//		u_out("PWR:",PWR_CHANNEL);
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_PWR_CHANNEL,//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,    		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			//-----------------------------------
			SERV_ID_DEL (id,i);//удаляем команду из реестра
			
//			Transf("Запрос состояния!\r\n");	
//			un_out("[",TIME_SYS);Transf("]\r\n");
		}	
		
		if (id->CMD_TYPE[i]==CMD_LED)//команда управления светодиодами на лицевой панели
		{
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище"
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище"
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			TCA_WR(data);//выполняем команду
			
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда

			ERROR_CMD_MSG ( //заполняем квитанцию о выполнении команды
			id,			    //указатель на реестр
			&INVOICE[ADR],  //указатель на структуру квитанции
			i, 			    //индекс команды в реестре
			MSG_CMD_OK,	    //сообщение квитанции
			0,				//данные квитанции
			TIME_SYS	    //текущее системное время 
			);	
			SERV_ID_DEL (id,i);//удаляем команду из реестра
			
			Transf("Светодиоды!\r\n");			
		}
		
		if (id->CMD_TYPE[i]==CMD_12V)//команда включения источника +12V
		{
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище"
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище"
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			START_BP=data;//выполняем команду
			
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда

			ERROR_CMD_MSG ( //заполняем квитанцию о выполнении команды
			id,			    //указатель на реестр
			&INVOICE[ADR],  //указатель на структуру квитанции
			i, 			    //индекс команды в реестре
			MSG_CMD_OK,	    //сообщение квитанции
			0,				//данные квитанции
			TIME_SYS	    //текущее системное время 
			);	
			SERV_ID_DEL (id,i);//удаляем команду из реестра
			
			Transf("Управление питанием +12V!\r\n");
			u_out("START_BP:",START_BP);
		}
		
		if (id->CMD_TYPE[i]==CMD_CH_UP)//команда включения канала питания, исправить длинну данных команды!!!
		{
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище"
		//--------------Убрать в соответствии с протоколом из ТО на РЭС!!!!-----------------
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище"
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
		//----------------------------------------------------------------------------------	
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			PWR_072 (data);//выполняем команду
		
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда

			ERROR_CMD_MSG ( //заполняем квитанцию о выполнении команды
			id,			    //указатель на реестр
			&INVOICE[ADR],  //указатель на структуру квитанции
			i, 			    //индекс команды в реестре
			MSG_CMD_OK,	    //сообщение квитанции
			0,				//данные квитанции
			TIME_SYS	    //текущее системное время 
			);	
			SERV_ID_DEL (id,i);//удаляем команду из реестра
			u_out("управляем питание каналов:",data);
		}
		
	//	if (id->TIME<TIME_SYS)
	}
}

u8 PIN_control_PB5 (void)
{
  static u8 pn_old;
  u8 pn;
  u8 flag;
  pn=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5);
  if (pn_old!=pn) {pn_old=pn;flag=1;} else flag=0;
  return flag;
}
  
int main(void)
{
	int i=0;
  /* USER CODE BEGIN 1 */
  Adress=0x30; //адресс кассеты 
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
  MX_I2C1_Init();
  MX_GPIO_Init();

  MX_DMA_Init ();
  MX_ADC1_Init();
  MX_TIM4_Init();
//MX_SPI2_Init();
//MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
//MX_USART2_UART_Init();
//MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

//  Delay(1000);
  
  Transf("-------------\r\n");
  Transf("    Б330\r\n");
  Transf("-------------\r\n");
  
  PWDN_4(1);
  
  ENABLE_LM25056_MK(0);//enable 
  PWR_072 (255);	   //снять питание на кассеты	
  RESET_TCA6424A_MK(1); 
  
  VD3(0);
  VD4(0);
  VD5(0);
  
 WDI_MK(0); 
 Massiv_dbm();
 
HAL_UART_Receive_IT(&huart1,RX_uBUF,1);
HAL_ADC_Start_DMA  (&hadc1,(uint32_t*)&adcBuffer,5); // Start ADC in DMA 

//--------init wiz820------------------
 PWDN_4(0);//снимаем повердаун с wiz820
 NSS_4(1);
 RES_4(0);
 Delay(2);
 RES_4(1);
 INIT_SERV_ARCHIV (&SERV1,&ID_SERV1,&ADDR_SNDR);//инициилизируем хранилище и реестр
 
 while (i<200)
 {
	 i++;
	 Delay(1);
	 WATCH_DOG ();
 }
 
 Set_network();
 RECEIVE_udp (0, 3001,1);

  while (1)
  {
    /* USER CODE END WHILE */

	WATCH_DOG ();
	LED();
	UART_conrol();
	BP_start(START_BP);
	
	if (EVENT_INT1==1)
	{
		EVENT_INT1=0;
	//	Transf("event 1!\r");
		RECEIVE_udp (0, 3001,1);		
	}; 
	CMD_search (&ID_SERV1,&SERV1);
	SEND_UDP_MSG ();
  	  UART_DMA_TX();
	//	if (FLAG_DMA_ADC==1) {DMA_ADC();FLAG_DMA_ADC=0;}	
  }

}
