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
#include <stdio.h>

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
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_tx;

TIM_HandleTypeDef htim4;

TIM_ClockConfigTypeDef sClockSourceConfig = {0};
TIM_MasterConfigTypeDef sMasterConfig = {0};
TIM_OC_InitTypeDef sConfigOC = {0};

/* USER CODE BEGIN PV */
#define LED_INTERVAL 500  		 // Интервал обновления индикации светодиодов
#define SYS_INTERVAL 250

u64 STM32_VERSION = 0x110620211803;//номер версии прошивки 12-02 время и 24-05-2021 дата
u32 IP_my=0;
u16 PORT_my=0;

u32 TIMER_CONTROL_SYS;   		//переменная таймера контроля состояния системы блока питания

volatile  uint16_t adcBuffer[5]; // Buffer for store the results of the ADC conversion
float adc_ch[12];

uint8_t transmitBuffer[32];
uint8_t receiveBuffer[32];

uint32_t TIMER1;
u32 TIMER_BP_PWM;
volatile u32  SysTickDelay; 

u32 FLAG_T1;
u32 FLAG_T2;
u8  FLAG_FAPCH_ON;

uint8_t RX_uBUF[1];
uint8_t RX2_uBUF[1];

//----------------UART1---------------------------------
unsigned int timer_DMA2;
u8 flag_pachka_TXT; //
uint16_t  text_lengh;
uint8_t text_buffer[Bufer_size];

volatile char          rx_buffer1[RX_BUFFER_SIZE1];
volatile unsigned int rx_wr_index1,rx_rd_index1,rx_counter1;
volatile u8  rx_buffer_overflow1;
//----------------UART2---------------------------------
unsigned int timer_DMA1_Stream6;
u8 flag_pachka_TXT2; //
uint16_t  text_lengh2;
uint8_t text_buffer2[Bufer_size2];

volatile char          rx_buffer2[RX_BUFFER_SIZE2];
volatile unsigned int rx_wr_index2,rx_rd_index2,rx_counter2;
volatile u8  rx_buffer_overflow2;

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

u32 TEST_LED     =0;
u64 TIME_SYS     =0;//переменная хранить системное время в милисекундах
u32 TIME_TEST    =0;
u32 ID_CMD       =0;//переменная хранить текущий ID наших квитанций 
u32 FLAG_T1HZ_MK =0;//Это флаг показывает что в течении пары секунд была одна секундная метка
u32 TIMER_T1HZ_MK=0;//Это контрольный таймер для проверки наличия секундной метки
u8  PWR_CHANNEL=255;

//флаги севтодиодов
u8  LED_ISPRAV_AC=0;
u8  LED_PROGR    =0;
u8  LED_OFCH     =0;
u8  LED_SINHR    =0;
u8  LED_LS       =0;
u8  LED_ISPR_J330=0;
u8  LED_OTKL_AC  =0;
u8  LED_TEMP	 =0;
u32 TIMER_LED    =0;

LM_struct LM1,LM2,LM3,LM4,LM5,LM6,LM7,LM8;

u8 LM_ID_CN[8];
u8 D_TEMP[4];
int TMP_v=0;

u8 FLAG_TIME_OF_WORK_WRITE=0; //флаг отложеной записи во флеш для нарабоки блока
u32 TIME_OF_SECOND=0;         //cчётчик секунд с начала работы
u32 TIME_OF_WORK=0;           //время наработки блока в десятках минут
SYS_STATE_BOARD B330;         //структура содержащая состояние блока Б330
//----------Адреса кассет 072 на бекплейне--------------
u32 MASTER_IP0     =0x0103023c;
u32 MASTER_IP1     =0x0103023d;
u32 MASTER_DEST_IP0=0x01030201;
u32 MASTER_DEST_IP1=0x01030202;

u32 SLAVE_IP0      =0x0103013c;
u32 SLAVE_IP1      =0x0103013d;
u32 SLAVE_DEST_IP0 =0x01030101;
u32 SLAVE_DEST_IP1 =0x01030102;
//------------------------------------------------------
u8  FLAG_ASQ_TEST_485 =0;    //флаг ответа на запрос теста по 485 шине
u8  FLAG_ASQ_TEST_JTAG=0;    //флаг ответа на запрос теста по SPI шине
u8  FLAG_ASQ_TEST_SPI =0;    //флаг ответа на запрос теста по JTAG шине

POINTER * PNT[PNT_BUF];  //массив указателей на отложенные задачи

POINTER  POINTER_TEST_485_REQ;  // 0  флаг запускающий ожидание ответа на запрос по шине 485 
POINTER  POINTER_TEST_485;      // 1  флаг запускающий тест проверки шины 485
POINTER  POINTER_TEST_SPI_REQ;  // 2  флаг запускающий ожидание ответа на запрос по шине SPI 
POINTER  POINTER_TEST_SPI;      // 2  флаг запускающий тест проверки шины SPI
POINTER  POINTER_TEST_JTAG_REQ;  // 3  флаг запускающий ожидание ответа на запрос по шине JTAG 
POINTER  POINTER_TEST_JTAG;     // 3  флаг запускающий тест проверки шины JTAG
POINTER  POINTER_ADR_COLLECT;   // 4  флаг запускающий сбор адресов с бекплейнов
POINTER  POINTER_MASTER_IP0_SETUP;     // 5  флаг запускающий раздачу IP0 адресов кассетам 072 на бекплейне
POINTER  POINTER_SLAVE_IP0_SETUP;
POINTER  POINTER_MASTER_IP1_SETUP;     // 6  флаг запускающий раздачу IP1 адресов кассетам 072 на бекплейне
POINTER  POINTER_SLAVE_IP1_SETUP;
POINTER  POINTER_DEST_MASTER_IP0_SETUP;// 7  флаг запускающий раздачу DEST_IP0 адресов кассетам 072 на бекплейне
POINTER  POINTER_DEST_SLAVE_IP0_SETUP;
POINTER  POINTER_DEST_MASTER_IP1_SETUP;// 8  флаг запускающий раздачу DEST_IP1 адресов кассетам 072 на бекплейне
POINTER  POINTER_DEST_SLAVE_IP1_SETUP;
POINTER  POINTER_ADR_REQ;       // 9  флаг запроса адреса от слейва на бэкплейне, поднимается когда запрашивается 
POINTER  POINTER_ETHERNET_RERUN;// 10 флаг по которому отсылается команда для переконфигурирования маков ячеек 072

u8  ADR_SLAVE [8];          //тут храним адреса кассет из блока АЦ
u8  NUMBER_OF_B072;         //число блоков Б072 на бекплейне
u32 TIMER_TIMEOUT=0;        //таймер таймаута ожидания ответов

u8 SCH_TST1=0;//счётчик числа попыток
u8 SCH_TST2=0;
u8 SCH_TST3=0;
int index_ch;
float TMP_f=0;
u8 FLAG_CMD=0;
u8 DAT_REQ[BUF_DATA_SZ];//транспортный массив
//-----------------------------------------------------------------------------
//                           описание структур управления и квитанций
/* USER CODE END PV */

 Frame INVOICE[quantity_SENDER];//структура квитанций о состоянии дел, по числу потенциальный адресатов
 Frame FRM1[quantity_SENDER];	//принятый фрейм, по числу потенциальный адресатов
 SERVER SERV1;					//структура "Хранилище"
 ID_SERVER ID_SERV1;			//структура указатель для "Хранилище"
 CMD_RUN CMD1;
 ADR_SENDER ADDR_SNDR;			//структура хранит массив адресов ОТПРАВИТЕЛЕЙ 

u64 ADRES_SENDER_CMD=0; //тут храним адрес компьютера управления, кто запрашивает у нас квитанции
u8 FLAG_ADRES_SENDER_CMD=0;//флаг того что есть куда отправлять квитанции
//-----------------------------------------------------------------------------
//                       JTAG
#include "jtagtap.h"
#include "jtag_scan.h"
#include "jtag_devs.h"

extern jtag_dev_t jtag_devs[JTAG_MAX_DEVS+1];
extern int jtag_dev_count;

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
int LM_TEMP    (u8);
int LM_v       (u8);
int LM_in_p    (u8);
int LM_in_i    (u8);
int LM_aux_u   (u8);
u8 TCA_WR    (u32);
u8 LM_MFR_ID (u8);
u8 TCA_test (void);
void SYS_INFO (u8);
void UART_DMA_TX  (void);
void UART_DMA_TX2 (void);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;// это для работы от внутреннего источника тактов
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;//
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  */
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;// это для работы от кварца 16 МГц
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  
  /*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;// это для работы от кварца 8 МГц
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  */
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
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }

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
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  huart1.Init.BaudRate = 256000;//115200
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
    /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

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

    /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  GPIO_InitStruct.Pin   = GPIO_PIN_0|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 PD12 PD14 
                           PD15 PD0 PD1 PD4 
                           PD7 */
  GPIO_InitStruct.Pin   = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15;
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

    /*Configure GPIO pin :  */
  GPIO_InitStruct.Pin  = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
  
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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
//--------SPI2--------------------
u8 SPI_Send(u8 data) 
{
  u8 a1;
  u8  b;
  a1 = (data)&0xff;  
  HAL_SPI_TransmitReceive(&hspi3, &a1, &b,1, 5000); 
  return b; 
}

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

void Transf2(const char* s)  // процедура отправки строки символов в порт
{
  u32 l=0;
  u32 i=0;
         
  if ((flag_pachka_TXT2==0) )
  {
    l=strlen(s);
    if ((text_lengh2+l)>Bufer_size2-5) text_lengh2=0u;
    for (i=text_lengh2;i<(text_lengh2+l);i++) text_buffer2[i]=s[i-text_lengh2];
    text_lengh2=text_lengh2+l;
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

void un64_out (char s[],u64 a)
{
   u64 z=0; 
   Transf (s);
   z=a>>32;
   a=a&0xffffffff;
   sprintf (strng,"%u",z);
   Transf(strng);
   sprintf (strng,"%u",a);
   Transf(strng);
}

//выводим десятичные числа побайтно для IP адреса
void xun_out (char s[],u32 a)//было u64 
{
   u8 tmp0=0;
   Transf (s);
   tmp0=(a>>24)&0xff;
   sprintf (strng,"%u",tmp0);
   Transf(strng);   
   Transf(".");
   tmp0=(a>>16)&0xff;
   sprintf (strng,"%u",tmp0);
   Transf(strng);   
   Transf(".");
   tmp0=(a>> 8)&0xff;
   sprintf (strng,"%u",tmp0);
   Transf(strng);   
   Transf(".");
   tmp0=(a>> 0)&0xff;
   sprintf (strng,"%u",tmp0);
   Transf(strng);   
}

void ARRAY_DATA (u32 a)  //разбиваем 32 битную переменную на байты и суём в транспортный массив
{
	D_TEMP[0]=a>>24;
	D_TEMP[1]=a>>16;
	D_TEMP[2]=a>> 8;
	D_TEMP[3]=a;
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

void UART_DMA_TX2 (void)
{
 uint16_t k;

if (HAL_UART_GetState(&huart2)!=HAL_UART_STATE_BUSY_TX )
	{
		if ((flag_pachka_TXT2==0)&&(text_lengh2>1u)&&(timer_DMA1_Stream6>250))
		 {
			NRE_RS485(1);//выключаем буфер 485 на приём 
			DE_RS485(1);//включаем буфер 485 на передачу

			k = text_lengh2;
			HAL_UART_Transmit_DMA(&huart2,(uint8_t *)text_buffer2,k);
			text_lengh2=0u;  //обнуление счётчика буфера 
			flag_pachka_TXT2=1; //устанавливаем флаг передачи
			timer_DMA1_Stream6=0;
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
  
   CS_5_MK(0);
//   delay_us(1);
 
   spi5send8 (adr_a); //
   for (i=0;i<(size/8);i++) d[i] = spi5send8 (0x00);  //считываем данные
//   delay_us(1);  
   CS_5_MK(1);
   
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

u32 FPGA_wSPI (u8 size,u8 adr,u64 data)
{
   u8 d[8];
   u8 i,k;
   
   k=size/8;
    
   if (k==1)  d[3]=data;
   
   if (k==2) {d[2]=data;
        d[3]=data>>8;
        }
        
   if (k==3) {d[1]=data;
        d[2]=data>>8;
        d[3]=data>>16;
        }
		
   if (k==4) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
        }
		
   if (k==5) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
        }
		
	if (k==6) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
        }
	if (k==7) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
		d[6]=data>>48;
        }
	if (k==8) {d[0]=data;
        d[1]=data>>8;
        d[2]=data>>16;
        d[3]=data>>24;
		d[4]=data>>32;
		d[5]=data>>40;
		d[6]=data>>48;
		d[7]=data>>56;
        }
  
   CS_5_MK(0);
   spi5send8 (adr|0x80); //передаём адресс
   k--;
   for (i=0;i<(size/8);i++) spi5send8 (d[k-i]); //пишем данные
   CS_5_MK(1);

   return 0;
}  

//-----------------------------------------------------------------
void TIME_cons (void)
{
  un64_out("[",TIME_SYS);Transf("]\r\n");
}
//-----------------------------------------------------------------
//  EPCS128 flash

u8 spi_EPCS1_STATUS (void)			//счтывает статусный байт во флеш , нулевой бит - бит записи, его проверЯем
{
	u8 m[1];
	spi_EPCS1_rd(READ_STATUS,m,1);
	return m[0];
}

u8 spi_EPCS2_STATUS (void)			//счтывает статусный байт во флеш , нулевой бит - бит записи, его проверЯем
{
	u8 m[1];
	spi_EPCS2_rd(READ_STATUS,m,1);
	return m[0];
}
void spi_EPCS1_rd (u8 cmd,u8 d[],u32 n) //чтение данных
{  
   u32 i=0;
   CS_EPCS1(0);
   spisend8(cmd);//
   for (i=0;i<n;i++)  
   {
	   d[i]=spisend8(0);  
   }
   CS_EPCS1(1);
}

void spi_EPCS2_rd (u8 cmd,u8 d[],u32 n) //чтение данных
{  
   u32 i=0;
   CS_EPCS2(0);
   spisend8(cmd);//
   for (i=0;i<n;i++)  
   {
	   d[i]=spisend8(0);  
   }
   CS_EPCS2(1);
}

void spi_EPCS1_read (u8 cmd,u32 adr,u8 d[],u32 n) //чтение данных
{  
   u32 i=0;
   CS_EPCS1(0);
   spisend8(cmd);//
   spisend8((adr>>16)&0xff);//
   spisend8((adr>> 8)&0xff);//
   spisend8( adr     &0xff);//
   for (i=0;i<n;i++)  
   {
	   Transf(".");
	   d[i]=spisend8(0);  
   }
   CS_EPCS1(1);
}

void spi_EPCS2_read (u8 cmd,u32 adr,u8 d[],u32 n) //чтение данных
{  
   u32 i=0;
   CS_EPCS2(0);
   spisend8(cmd);//
   spisend8((adr>>16)&0xff);//
   spisend8((adr>> 8)&0xff);//
   spisend8( adr     &0xff);//
   for (i=0;i<n;i++)  
   {
	   Transf(".");
	   d[i]=spisend8(0);  
   }
   CS_EPCS2(1);
}

void spi_EPCS1_write (u8 cmd,u32 adr,u8 d[],u32 n) //запись данных в один сектор - 256 байт!!!
{  
   u32 i=0;
   CS_EPCS1(0);  
   spisend8(cmd);//
   spisend8((adr>>16)&0xff);//
   spisend8((adr>> 8)&0xff);//
   spisend8( adr     &0xff);//
   
   for (i=0;i<n;i++)  
   {
	   Transf(".");
	   spisend8(d[i]);  
   }
   CS_EPCS1(1);
  Transf("\r\n");
}

void spi_EPCS2_write (u8 cmd,u32 adr,u8 d[],u32 n) //запись данных в один сектор - 256 байт!!!
{  
   u32 i=0;
   CS_EPCS2(0);  
   spisend8(cmd);//
   spisend8((adr>>16)&0xff);//
   spisend8((adr>> 8)&0xff);//
   spisend8( adr     &0xff);//
   
   for (i=0;i<n;i++)  
   {
	   Transf(".");
	   spisend8(d[i]);  
   }
   CS_EPCS2(1);
  Transf("\r\n");
}

void spi_EPCS1_ERASE_BULK (void) //разрешение записи во флеш
{  
   CS_EPCS1(0);  
   spisend8(ERASE_BULK);//
   CS_EPCS1(1);
}

void spi_EPCS2_ERASE_BULK (void) //разрешение записи во флеш
{  
   CS_EPCS2(0);  
   spisend8(ERASE_BULK);//
   CS_EPCS2(1);
}

void spi_EPCS1_wr_ENABLE (void) //разрешение записи во флеш
{  
   CS_EPCS1(0);  
   spisend8(WRITE_ENABLE);//
   CS_EPCS1(1);
}

void spi_EPCS2_wr_ENABLE (void) //разрешение записи во флеш
{  
   CS_EPCS2(0);  
   spisend8(WRITE_ENABLE);//
   CS_EPCS2(1);
}

void spi_EPCS1_wr_DISABLE (void) //разрешение записи во флеш
{  
   CS_EPCS1(0);  
   spisend8(WRITE_DISABLE);//
   CS_EPCS1(1);
}

void spi_EPCS2_wr_DISABLE (void) //разрешение записи во флеш
{  
   CS_EPCS2(0);  
   spisend8(WRITE_DISABLE);//
   CS_EPCS2(1);
}

//-----------------------------------------------------------------
 void Console_corr(void)
 {
   int i=0;
    Transf("\r\n-------\r\n");
   for (i=0;i<8;i++)
   {
     un_out("KI[",i); f_out("]=",B330.Corr_I[i]);
   }
    Transf("\r\n-------\r\n");
   for (i=0;i<8;i++)
   {
     un_out("KU[",i); f_out("]=",B330.Corr_U[i]);
   }
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

void SPI_BP_WRITE (u32 adr,u32 data)
{
  adr=(adr<<4)|0x2;
  FPGA_wSPI (32,adr,data);
}

u32 SPI_BP_READ (u32 adr)
{
  u32 tmp0;
  adr=(adr<<4)|0x3;
  tmp0=FPGA_rSPI (32,adr);
  return tmp0;
}
//----------------------------------------------------------
void info ()
{

}

void BUS_485_TEST (u8 a)
{
   u8 u[17];
   int n=0;

   u[n++]=' ';   //0
   u[n++]='~';   //1
   u[n++]=0x30+a;//2
   u[n++]=' ';   //3
   u[n++]='r';   //4
   u[n++]='s';   //5
   u[n++]='4';   //6
   u[n++]='8';   //7
   u[n++]='5';   //8
   u[n++]='_';   //9
   u[n++]='t';   //10
   u[n++]='e';   //11
   u[n++]='s';   //12
   u[n++]='t';   //13
   u[n++]=';';   //14
   u[n++]=';';   //15
   u[n++]=0x00;  //16

   Transf2(u);
   Transf("Послан код: ");
   Transf(u);
   Transf("\r\n");
}

u32 crc_input=0u; 
u32 crc_comp=0u;
u8 Str[64];
u8 mas[300];

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
  if (strcmp(Word,"versiya")==0) //
   {
    Transf ("принял versiya\r\n"); 
    REQ_VERSIYA();       
   } else
if (strcmp(Word,"setup_IP0")==0) // отсылаем IP адрес по шине 485 в 072
   {

	 crc_comp =atoi(DATA_Word);	//первое число - адресс касеты на бекплейне
 	 crc_input=atoi(DATA_Word2);//второе число - IP адресс
	 
	 Str[ 0]='~';
	 Str[ 1]=0x30+(crc_comp&0xff);
	 Str[ 2]='s';
	 Str[ 3]='s';
	 Str[ 4]='e';
	 Str[ 5]='t';
	 Str[ 6]='u';
	 Str[ 7]='p';
	 Str[ 8]='_';
	 Str[ 9]='I';
	 Str[10]='P';
	 Str[11]='0';
	 Str[12]=':';
	 Str[13]=(crc_input>>24);
	 Str[14]=(crc_input>>16);
	 Str[15]=(crc_input>> 8);
	 Str[16]=(crc_input>> 0);
	 Str[17]=';';
	 Str[18]=0x00;

	 u_out("принял setup_IP0:",crc_input);
	 Transf2(Str);
   } else
if (strcmp(Word,"rst_072")==0) // 
   {
	 crc_comp =atoi(DATA_Word);	
	 u_out("принял rst_072:",crc_comp);
	 RESET_072(crc_comp);
   } else   
if (strcmp(Word,"rs485_test_OK")==0) // проверяем шину 485! Должен прийти ответ.
   {
	 u_out("принял rs485_test_OK:",crc_comp);
	 Transf("тест 485 шины пройден успешно!\r\n");
	 FLAG_ASQ_TEST_485=1;
   } else	
if (strcmp(Word,"rs485_test")==0) // проверяем шину 485! Должен прийти ответ.
   {
	 u_out("принял rs485_test:",crc_comp);	 
     BUS_485_TEST (crc_comp);
   } else
 if (strcmp(Word,"rs485_help")==0) // ~0 spi_write:adr.code;
   {
	 u_out("принял rs485_help:",crc_comp);
	 Transf2("~0 help;");
   } else		
 if (strcmp(Word,"upr")==0) // ~0 spi_write:adr.code;
   {
	 crc_comp =atoi(DATA_Word);	//первое число - адресс касеты на бекплейне
     UPR_HDS_MK (crc_comp);
	 u_out("принял upr:",crc_comp);
   } else	
 if (strcmp(Word,"spi_write")==0) // ~0 spi_write:adr.code;
   {
	 crc_comp =atoi(DATA_Word);	//первое число - адресс касеты на бекплейне
 	 crc_input=atoi(DATA_Word2);//записываемый 32-битный код 
	 crc_comp=(crc_comp<<4)|0x2;
	 FPGA_wSPI (32,crc_comp,crc_input);
	 Transf("принял FPGA spi_write\r\n");
   } else
 if (strcmp(Word,"spi_read")==0) //
   {	
    crc_comp=atoi(DATA_Word);//тут адресс кассеты на бекплейне
	  crc_comp=(crc_comp<<4)|0x3;
	  crc_comp=FPGA_rSPI (32,crc_comp);
	  x_out("CODE FPGA:",crc_comp);
   } else	
 if (strcmp(Word,"spi_rd_tst")==0) //
   {	
    crc_comp=atoi(DATA_Word);//тут адресс кассеты на бекплейне
	  crc_comp=(crc_comp<<4)|0x1;
	  crc_comp=FPGA_rSPI (32,crc_comp);
	  x_out("CODE FPGA:",crc_comp);
   } else
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
     TIME_cons ();
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
   } else
if (strcmp(Word,"sys_info")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("принял sys_info:",crc_comp); 
      SYS_INFO(crc_comp);
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
if (strcmp(Word,"led")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     u_out ("принял led:",crc_comp);
	 TCA_WR(crc_comp);
   }else  
if (strcmp(Word,"menu")==0)                     
   {
     Transf ("принял menu\r\n"    );
     Menu1(0);
   } else
if (strcmp(Word,"REQ_COL")==0)    //запрос количества блоков 072 и их адресов                 
   {
     Transf ("принял REQ_COL\r\n"    );
     req_col();
   } else
if (strcmp(Word,"ANS")==0) //пришёл ответ по бекплейну (485) на ранее заданый вопрос                     
   {     
     crc_comp =atoi(DATA_Word); 
     u_out ("принял ANS:",crc_comp);
     answer_translated (crc_comp);     
   } else   
 if (strcmp(Word,"Show_worktime")==0) //
   {
    u_out ("принял Show_worktime:",TIME_OF_WORK); 
   } else
  if (strcmp(Word,"MSG_ADR")==0) //пришло сообщение с позицией на бекплейне, какая-то касета перезагрузилась и просит реинициализации
   {
    crc_comp =atoi(DATA_Word); 
    u_out ("принял MSG_ADR:",crc_comp); 
    FUNC_FLAG_UP (&POINTER_ADR_COLLECT,100);//ставим отложенную задачу для опроса кассет на бекплейне
   } else
      if (strcmp(Word,"EPCS1_DEV_ID")==0) //only for EPCS128 !!!
        {
          Transf ("\r\nпринял EPCS1_DEV_ID:\r\n");
          spi_EPCS1_rd(READ_DEV_ID,mas,3);
          Transf ("\r\n");
          if (mas[2]==0x18) Transf("recive: EPCS128\r\n");
          x_out("mas[2]:",mas[2]);
        } else       
      if (strcmp(Word,"EPCS2_DEV_ID")==0) //only for EPCS128 !!!
        {
          Transf ("\r\nпринял EPCS2_DEV_ID:\r\n");
          spi_EPCS2_rd(READ_DEV_ID,mas,3);
          Transf ("\r\n");
          if (mas[2]==0x18) Transf("recive: EPCS128\r\n");
          x_out("mas[2]:",mas[2]);
        } else
      if (strcmp(Word,"EPCS1_ID")==0) //
        {
          Transf ("\r\nпринял EPCS_ID:\r\n");
          spi_EPCS1_rd(0x20,mas,3);
          Transf ("\r\n");
          x_out("mas[0]:",mas[0]);
          x_out("mas[1]:",mas[1]);
          x_out("mas[2]:",mas[2]);
          }  else
      if (strcmp(Word,"EPCS1_STATUS")==0) //
        {
          crc_comp =atoi(DATA_Word);
          crc_input=atoi(DATA_Word2);
          Transf ("\r\nпринял EPCS1_STATUS:\r\n");
          spi_EPCS1_rd(READ_STATUS,mas,4);
          Transf ("\r\n");
          x_out("mas[0]:",mas[0]);
          x_out("mas[1]:",mas[1]);
          x_out("mas[2]:",mas[2]);
          x_out("mas[3]:",mas[3]);
          }  else
     if (strcmp(Word,"EPCS2_STATUS")==0) //
        {
          crc_comp =atoi(DATA_Word);
          crc_input=atoi(DATA_Word2);
          Transf ("\r\nпринял EPCS2_STATUS:\r\n");
          spi_EPCS2_rd(READ_STATUS,mas,4);
          Transf ("\r\n");
          x_out("mas[0]:",mas[0]);
          x_out("mas[1]:",mas[1]);
          x_out("mas[2]:",mas[2]);
          x_out("mas[3]:",mas[3]);
          }  else
        if (strcmp(Word,"EPCS1_READ")==0) //
          {
            crc_comp =atoi(DATA_Word); //адрес чтения
            crc_input=atoi(DATA_Word2);//количество байт
            x_out ("\r\nпринял EPCS1_READ:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ            
            spi_EPCS1_read(READ_BYTES,crc_comp,mas,crc_input);//чтение 256 байт данных
            Transf("\r\n---------------------------\r\n");
            TABL_CONS (mas,crc_input);
          }
            else
        if (strcmp(Word,"EPCS2_READ")==0) //
          {
            crc_comp =atoi(DATA_Word); //адрес чтения
            crc_input=atoi(DATA_Word2);//количество байт
            x_out ("\r\nпринял EPCS2_READ:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ            
            spi_EPCS2_read(READ_BYTES,crc_comp,mas,crc_input);//чтение 256 байт данных
            Transf("\r\n---------------------------\r\n");
            TABL_CONS (mas,crc_input);
          }
    else
    if (strcmp(Word,"EPCS1_WRITE_TEST")==0) //
      {
        crc_comp =atoi(DATA_Word);
        crc_input=atoi(DATA_Word2);
        x_out ("\r\nпринял EPCS1_WRITE_TEST:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ
        Transf("\r\n---------------------------\r\n");        
        for (i=0;i<256;i++) mas[i]=crc_input;          
        EPCS1_WRITE_BUF(crc_comp,mas,256);
        }else
    if (strcmp(Word,"EPCS2_WRITE_TEST")==0) //
      {
        crc_comp =atoi(DATA_Word);
        crc_input=atoi(DATA_Word2);
        x_out ("\r\nпринял EPCS2_WRITE_TEST:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ
        Transf("\r\n---------------------------\r\n");        
        for (i=0;i<256;i++) mas[i]=crc_input;          
        EPCS2_WRITE_BUF(crc_comp,mas,256);
        }else
    if (strcmp(Word,"EPCS1_ERASE_SECTOR")==0) //
      {
        crc_comp =atoi(DATA_Word);
        x_out ("\r\nпринял EPCS1_ERASE_SECTOR:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ
        EPCS1_ERASE_SECTOR(crc_comp);
        }else 
    if (strcmp(Word,"EPCS2_ERASE_SECTOR")==0) //
      {
        crc_comp =atoi(DATA_Word);
        x_out ("\r\nпринял EPCS2_ERASE_SECTOR:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ
        EPCS2_ERASE_SECTOR(crc_comp);
        }else 
    if (strcmp(Word,"EPCS1_ERASE_ALL")==0) //
      {
        crc_comp =atoi(DATA_Word);
        x_out ("\r\nпринял EPCS_ERASE_ALL:",crc_comp);//crc_comp - тут 24-х битный адрес чтениЯ
        EPCS1_ERASE_ALL();
        } else 
    if (strcmp(Word,"serial_w")==0) //
        {
          crc_comp =atoi(DATA_Word);
          u_out ("принял serial_w:",crc_comp);  
          SERIAL_NUMBER_WR (crc_comp);                  
        } else
    if (strcmp(Word,"corr")==0) //
        {
          crc_comp =atoi(DATA_Word);
          u_out ("принял corr:",crc_comp);  
          Console_corr();                  
        }  else
    if (strcmp(Word,"corr_I_write")==0) //
        {
          crc_comp =atoi(DATA_Word);
          u_out ("принял corr_I_write:",crc_comp);  
          CorrI_write ();                
        } 
 } 
      for (i=0u;i<(BUFFER_SR+1);i++)         str[i]=0x00;
	    for (i=0u;i<buf_Word     ;i++)        Word[i]=0x00;
      for (i=0u;i<buf_DATA_Word;i++)   DATA_Word[i]=0x00;
      for (i=0u;i<buf_DATA_Word;i++)  DATA_Word2[i]=0x00;  
      for (i=0u;i<BUFFER_SR    ;i++)       InOut[i]=0x00;  
      
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
  Transf("..........Terminal 330 кассеты.........\r\n");
  Transf("\r");
  Transf("MENU :\r");
  Transf("-------\r");
  Transf("Расшифровка структуры команды:\r");
  Transf("~ - стартовый байт\r");
  un_out("",Adress-0x30);
  Transf(" - адрес абонента\r");
  Transf(";- конец пачки \r");
  Transf(".............. \r");
  Transf("---------------------------------------------\r\n");
  Transf("PORT:");
  un_out("",PORT_my);
  Transf("       - номер порта блока\r");
  Transf("IP  :");
  xun_out("",IP_my);
  Transf(" - IP адрес    блока\r"); 
  Transf("~0 help; - текущее меню\r");
  Transf("~0 info; - информация \r");
  Transf("~0 rs485_test; - \r");
  Transf("~0 JTAG_TST;   - тестирование JTAG\r");
  Transf("~0 JTAG2_SCAN; - сканирование цепи JTAG\r");
  Transf("~0 JTAG1_SCAN; - сканирование цепи JTAG\r");
  Transf("~0 JTAG_ID; \r");
  Transf("~0 time;       - вывод системного времени\r");
  Transf("~0 sys_info; \r");
  Transf("~0 start:1; \r");
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
  REQ_VERSIYA(); //выводим версию прошивки
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

char getchar2(void)
{
   uint8_t data;
   while (rx_counter2 == 0);
   data = rx_buffer2[ rx_rd_index2++ ];
   if (rx_rd_index2 == RX_BUFFER_SIZE2) rx_rd_index2 = 0;
    --rx_counter2;
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
   		HAL_UART_Receive_IT(huart,RX_uBUF,1);
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
		//Transf2("~0 help;\r\n");
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
	static u8 flag=0;
	if (a==0)
	{
	    B330.INIT=0;
		PWR_072    (255);//выключаем каналы питания и сбрасываем состояния переменных говорящих о каналах питания - в ноль
		UPR_HDS_MK (1);	
		TIMER_BP_PWM=1000;
		if (flag==0) {Transf("Выключаем ПИТАНИЕ!\r\n");flag=1;}
	} else
	if ((TIMER_BP_PWM==0)&&(flag==1))
	{
		B330.INIT=1;
		flag=0;
		UPR_HDS_MK (0);			
		Transf("Включаем ПИТАНИЕ!\r\n");
		IO("~0 pwr_072:0;"); //подаём питание на все каналы!!! - без этого не работает i2c			
		IO("~0 enable_lm:1;");//включаем все м/мы LM
    FUNC_FLAG_UP (&POINTER_ADR_COLLECT,4000);//ставим отложенную задачу для опроса кассет на бекплейне
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

void SYS_INFO (u8 a)
{
	Transf("\r\n");
	if (a>0)
	{
		u_out("LM1.TEMP:",LM1.TEMP);
		u_out("LM2.TEMP:",LM2.TEMP);
		u_out("LM3.TEMP:",LM3.TEMP);
		u_out("LM4.TEMP:",LM4.TEMP);
		u_out("LM5.TEMP:",LM5.TEMP);
		u_out("LM6.TEMP:",LM6.TEMP);
		u_out("LM7.TEMP:",LM7.TEMP);
		u_out("LM8.TEMP:",LM8.TEMP);
			Transf("\r\n");
	}
	if (a>1)
	{
		u_out("LM1.P:",LM1.P);
		u_out("LM2.P:",LM2.P);
		u_out("LM3.P:",LM3.P);
		u_out("LM4.P:",LM4.P);
		u_out("LM5.P:",LM5.P);
		u_out("LM6.P:",LM6.P);
		u_out("LM7.P:",LM7.P);
		u_out("LM8.P:",LM8.P);
			Transf("\r\n");
	}
	if (a>2)
	{
		u_out("LM1.I:",LM1.I);
		u_out("LM2.I:",LM2.I);
		u_out("LM3.I:",LM3.I);
		u_out("LM4.I:",LM4.I);
		u_out("LM5.I:",LM5.I);
		u_out("LM6.I:",LM6.I);
		u_out("LM7.I:",LM7.I);
		u_out("LM8.I:",LM8.I);
			Transf("\r\n");
	}
	if (a>3)
	{
		u_out("LM1.U:",LM1.U);
		u_out("LM2.U:",LM2.U);
		u_out("LM3.U:",LM3.U);
		u_out("LM4.U:",LM4.U);
		u_out("LM5.U:",LM5.U);
		u_out("LM6.U:",LM6.U);
		u_out("LM7.U:",LM7.U);
		u_out("LM8.U:",LM8.U);
			Transf("\r\n");
	}
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

  if (z==1) 
  {
    LM1.ID[0]=a[0];
    LM1.ID[1]=a[1];
    LM1.ID[2]=a[2];
  }
  if (z==2) 
  {
    LM2.ID[0]=a[0];
    LM2.ID[1]=a[1];
    LM2.ID[2]=a[2];
  }
  if (z==3) 
  {
    LM3.ID[0]=a[0];
    LM3.ID[1]=a[1];
    LM3.ID[2]=a[2];
  }
  if (z==4) 
  {
    LM4.ID[0]=a[0];
    LM4.ID[1]=a[1];
    LM4.ID[2]=a[2];
  }
  if (z==5) 
  {
    LM5.ID[0]=a[0];
    LM5.ID[1]=a[1];
    LM5.ID[2]=a[2];
  }
  if (z==6) 
  {
    LM6.ID[0]=a[0];
    LM6.ID[1]=a[1];
    LM6.ID[2]=a[2];
  }
  if (z==7) 
  {
    LM7.ID[0]=a[0];
    LM7.ID[1]=a[1];
    LM7.ID[2]=a[2];
  }
  if (z==8) 
  {
    LM8.ID[0]=a[0];
    LM8.ID[1]=a[1];
    LM8.ID[2]=a[2];
  }
	
	HAL_I2C_DeInit(&hi2c1);
	return state;
}

float okrug(float chislo, long znaki)
{
    float res;
    return round(chislo * pow(10, znaki)) / pow(10, znaki);
}

int LM_TEMP (u8 z)
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
  x=okrug(x,0);
  value=x;
  value=value*100;

	
	if (error==1) value=0xffffffff;

	HAL_I2C_DeInit(&hi2c1);
	
	return value;
}

int LM_v (u8 z)
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
//	x=okrug(x,2);
  value=x*100*B330.Corr_U[z-1];
	
	if (error==1) value=0xffffffff;
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return value;
}
  
int LM_aux_u (u8 z)
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
	
//	 x=okrug(x,2);
  value=x*100;
	
	if (error==1) value=0xffffffff;
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return value;
}

int LM_in_i (u8 z)
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
	 float m=68985.849;//5 мОм
	 float y=0;
	 float r=2;
	 float b=0;//-1833
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
	
	c[0]=0xd1;//Retrieves  input current measurement

	error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size,1);
	
	p=(a[1]<<8)+a[0];
	y=p;

//	x=(1/m)*((y*(powf(10,r)))-b);
	x=(1/m)*((y*(powf(10,r)))-b)/1.1;//с поправочным коэффициентом
//	f_out("I=",x);
	
	// x=okrug(x,2);
  value=x*100*B330.Corr_I[z-1];
	
	if (error==1) value=0xffffffff;
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return value;
}

int LM_in_p (u8 z)
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
	 float m=27.45;
	 float y=0;
	 float r=0;
	 float b=0;
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
	
	c[0]=0xd2;//Retrieves   input power measurement.

	error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size,1);
	
	p=(a[1]<<8)+a[0];
	y=p;

	x=(1/m)*(y*(powf(10,r))-b);
	
//	f_out("P=",x);
	
	 x=okrug(x,2);
  value=x*100;

	if (error==1) value=0xffffffff;
	
//	u_out("i2c:",error);
	
	D_TEMP[0]=value>>24;
	D_TEMP[1]=value>>16;
	D_TEMP[2]=value>> 8;
	D_TEMP[3]=value;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return value;
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

void DAT_CORR_REQ_FORM (void)
{
  u32 tmp=0;
  u32 i=0;
  u32 j=0;

  for (i=0;i<8;i++)
  {
    tmp=B330.Corr_I[i]*1000;
    DAT_REQ[j+0]=(tmp>>24)&0xff;
    DAT_REQ[j+1]=(tmp>>16)&0xff;
    DAT_REQ[j+2]=(tmp>> 8)&0xff;
    DAT_REQ[j+3]=(tmp>> 0)&0xff;
    j=j+4;
  }

  for (i=0;i<8;i++)
  {
    tmp=B330.Corr_U[i]*1000;
    DAT_REQ[j+0]=(tmp>>24)&0xff;
    DAT_REQ[j+1]=(tmp>>16)&0xff;
    DAT_REQ[j+2]=(tmp>> 8)&0xff;
    DAT_REQ[j+3]=(tmp>> 0)&0xff;
    j=j+4;
  }
  
}

//функция отслыает ...
void MSG_SEND_UDP (ID_SERVER *id,SERVER *srv,u32 msg_type)
{
  u32 i=0;
  
  u64 ADR=ADRES_SENDER_CMD;
  u32 data=0;
  u8 D[4];
        
 if (msg_type==MSG_REQ_TEST_485)
 {
    ARRAY_DATA(FLAG_ASQ_TEST_485);
 }else
 if (msg_type==MSG_REQ_TEST_SPI)
 {
    ARRAY_DATA(FLAG_ASQ_TEST_SPI);
 }else
 if (msg_type==MSG_REQ_TEST_JTAG)
 {
    ARRAY_DATA(FLAG_ASQ_TEST_JTAG);
 }
       

  SYS_CMD_MSG(
        id,//реестр
        &INVOICE[ADR],  //структура квитанций 
        i,        //индекс в реестре
        msg_type, //тип сообщения
        4,        //объём данных сообщения в байтах
        D_TEMP,   //данные сообщения - массив данных
        TIME_SYS  //время составления квитанции
        );
}


#define COL 37
u8 DATA_TR [COL];

void ARR_Z ()
{
  u8 n=0;
  u32 tmp0=0;
  
  DATA_TR[n++]=B330.INIT;			//0
  DATA_TR[n++]=B330.TEMP_MAX>>8;	//1	
  DATA_TR[n++]=B330.TEMP_MAX&0xff;  //2	 

  DATA_TR[n++]=B330.I>>24;			//3
  DATA_TR[n++]=B330.I>>16;			//4
  DATA_TR[n++]=B330.I>> 8;			//5
  DATA_TR[n++]=B330.I&0xff;			//6

  DATA_TR[n++]=B330.U_min>>24;		//7	
  DATA_TR[n++]=B330.U_min>>16;		//8
  DATA_TR[n++]=B330.U_min>> 8;		//9
  DATA_TR[n++]=B330.U_min&0xff;		//10

  DATA_TR[n++]=B330.U_max>>24;		//11  
  DATA_TR[n++]=B330.U_max>>16;		//12 
  DATA_TR[n++]=B330.U_max>> 8;		//13
  DATA_TR[n++]=B330.U_max&0xff;		//14

  DATA_TR[n++]=B330.P>>24;			//15
  DATA_TR[n++]=B330.P>>16;			//16 
  DATA_TR[n++]=B330.P>> 8;			//17
  DATA_TR[n++]=B330.P&0xff; 		//18

  DATA_TR[n++]=B330.FLAG_1HZ;		//19

  DATA_TR[n++]=B330.B330_NUMBER>>24;//20
  DATA_TR[n++]=B330.B330_NUMBER>>16;//21
  DATA_TR[n++]=B330.B330_NUMBER>> 8;//22
  DATA_TR[n++]=B330.B330_NUMBER;	//23
  DATA_TR[n++]=B330.PRG_VERSIYA>>56;//24
  DATA_TR[n++]=B330.PRG_VERSIYA>>48;//25
  DATA_TR[n++]=B330.PRG_VERSIYA>>40;//26
  DATA_TR[n++]=B330.PRG_VERSIYA>>32;//27
  DATA_TR[n++]=B330.PRG_VERSIYA>>24;//28
  DATA_TR[n++]=B330.PRG_VERSIYA>>16;//29
  DATA_TR[n++]=B330.PRG_VERSIYA>> 8;//30
  DATA_TR[n++]=B330.PRG_VERSIYA>> 0;//31
  DATA_TR[n++]=B330.WORK_TIME>>24;	//32
  DATA_TR[n++]=B330.WORK_TIME>>16;	//33
  DATA_TR[n++]=B330.WORK_TIME>> 8;	//34
  DATA_TR[n++]=B330.WORK_TIME>> 0;  //35
  DATA_TR[n++]=B330.STATUS_OK; 		  //36
}

void SYS_INFO_SEND_UDP (ID_SERVER *id,SERVER *srv)
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
	
	u64 ADR=ADRES_SENDER_CMD;
	u32 data=0;
	u8 D[4];

	if (START_BP==1)
			{
			//	LM_MFR_ID(1);
				
				SYS_CMD_MSG(
				id,				//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH1,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM1.ID,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
			//	LM_MFR_ID(2);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH2,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM2.ID,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
			//	LM_MFR_ID(3);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH3,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM3.ID,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
			//	LM_MFR_ID(4);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH4,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM4.ID,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
			//	LM_MFR_ID(5);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH5,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM5.ID,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				
			//	LM_MFR_ID(6);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH6,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM6.ID,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
			//	LM_MFR_ID(7);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH7,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM7.ID,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
			//	LM_MFR_ID(8);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_ID_CH8,		//тип сообщения
				3,		 		//объём данных сообщения в байтах
				LM8.ID,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
			}
			//-----------------------------------
			if (START_BP==1)
			{
				ARRAY_DATA(LM1.TEMP);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH1,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,			//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				ARRAY_DATA(LM2.TEMP);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH2,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				ARRAY_DATA(LM3.TEMP);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH3,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				ARRAY_DATA(LM4.TEMP);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH4,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				ARRAY_DATA(LM5.TEMP);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH5,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				ARRAY_DATA(LM6.TEMP);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH6,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				ARRAY_DATA(LM7.TEMP);
				
				SYS_CMD_MSG(
				id,//реестр
				&INVOICE[ADR], 	//структура квитанций	
				i,	 			//индекс в реестре
				MSG_TEMP_CH7,	//тип сообщения
				4,		 		//объём данных сообщения в байтах
				D_TEMP,		//данные сообщения - массив данных
				TIME_SYS	  	//время составления квитанции
				);
				
				ARRAY_DATA(LM8.TEMP);
				
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
			
			ARRAY_DATA(LM1.P);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH1,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM2.P);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH2,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM3.P);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH3,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM4.P);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH4,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM5.P);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH5,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM6.P);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH6,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM7.P);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_P_CH7,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM8.P);
			
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
			
			ARRAY_DATA(LM1.I);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH1,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM2.I);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH2,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM3.I);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH3,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM4.I);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH4,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM5.I);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH5,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM6.I);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH6,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM7.I);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_I_CH7,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM8.I);
			
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
			
			ARRAY_DATA(LM1.U);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH1,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM2.U);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH2,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM3.U);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH3,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM4.U);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH4,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM5.U);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH5,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM6.U);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH6,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM7.U);
			
			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_U_CH7,	//тип сообщения
			4,		 		//объём данных сообщения в байтах
			D_TEMP,		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
			
			ARRAY_DATA(LM8.U);
			
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
//----------------------------------
//квитанция об общем состоянии кассеты Б330
	ARR_Z ();//заполняем транспортный массив

			SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR], 	//структура квитанций	
			i,	 			//индекс в реестре
			MSG_STATUS_OK,  //тип сообщения
			COL,		    //объём данных сообщения в байтах
			DATA_TR,  		//данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);

}

int adr_BPL=0;


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
			FLAG_CMD=1;
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
			TIME_cons ();
			Transf("\r\n------\r\n");					
		} else		
		if (id->CMD_TYPE[i]==CMD_STATUS)//команда запроса состояни
		{
			//нет данных у команды
      FLAG_CMD=1;
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда
			ADRES_SENDER_CMD=ADR;
			//------------------------------------
			SYS_INFO_SEND_UDP (id,srv);
		}	else		
		if (id->CMD_TYPE[i]==CMD_LED)//команда управления светодиодами на лицевой панели
		{
      FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище"
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище"
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			TCA_WR(data);//выполняем команду
		} else    			
		if (id->CMD_TYPE[i]==CMD_NUMB_BLOCK_WR)//команда управления светодиодами на лицевой панели
		{
      FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище"
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище"
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			 SERIAL_NUMBER_WR (data);//выполняем команду
		} else			
		if (id->CMD_TYPE[i]==CMD_12V)//команда включения источника +12V
		{
      FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище"
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище"
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			START_BP=data;//выполняем команду			
			FLAG_ADRES_SENDER_CMD=1;//поднимаем флаг того что у нас есть куда отправлять квитанции
		} else		
		if (id->CMD_TYPE[i]==CMD_CH_UP)//команда включения канала питания, исправить длинну данных команды!!!
		{
      FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище"
		//--------------Убрать в соответствии с протоколом из ТО на РЭС!!!!-----------------
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище"
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
		//----------------------------------------------------------------------------------	
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			PWR_072 (data);//выполняем команду
		} else
		if (id->CMD_TYPE[i]==CMD_SETUP_IP0)//команда установки IP0 адреса определённой кассеты 072 
		{
      FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище" 
			idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище" //в младшем адресе находятся старшие байты числа!!!
			idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
			idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
			idx4=idx_srv(id->INDEX[i],4);//индекс расположения данных в "хранилище"
		//----------------------------------------------------------------------------------	
			data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      int adr_BPL=srv->MeM[idx0];//адрес 072 на бекплейне

			//SETUP_IP0_072 (adr_BPL,data);//выполняем команду
      SETUP_IP0_072 (ADR_SLAVE[0],data);//выполняем команду
      Transf("Пришёл IP0 для Мастера 072!\r\n");
      x_out("отправляем IP адрес:",data);
      u_out("В блок 072 №",adr_BPL);			
		} else
      if (id->CMD_TYPE[i]==CMD_SETUP_IP1)//команда установки IP0 адреса определённой кассеты 072 
    {
      FLAG_CMD=1;
      idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище" 
      idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище" //в младшем адресе находятся старшие байты числа!!!
      idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
      idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
      idx4=idx_srv(id->INDEX[i],4);//индекс расположения данных в "хранилище"
    //----------------------------------------------------------------------------------  
      data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      adr_BPL=srv->MeM[idx0];//адрес 072 на бекплейне

      SETUP_IP1_072 (ADR_SLAVE[0],data);//выполняем команду
      Transf("Пришёл IP1 для Мастера 072!\r\n");
      x_out("отправляем IP адрес:",data);
      u_out("В блок 072 №",adr_BPL);      
    } else
    if (id->CMD_TYPE[i]==CMD_REQ_NUM_SLAVE)//команда запроса о количестве блоков 072 и их адресах 
    {
      FLAG_CMD=1;
      Transf("Запрашиваем кассеты 072 в АЦ на предмет их адресов!\r\n");
  //    req_col();
      FUNC_FLAG_UP (&POINTER_ADR_COLLECT,1000);//ставим отложенную задачу для опроса кассет на бекплейне
    } else
       if (id->CMD_TYPE[i]==CMD_TEST_485)//команда начала теста проверки шины 485
    {
      FLAG_CMD=1;
      Transf("начинаем тест проверку шины 485!\r\n");
      FLAG_ASQ_TEST_485=0;//сбрасываем флаг ответа на тест 485
      FUNC_FLAG_UP (&POINTER_TEST_485,10);//ставим отложенную задачу для опроса кассет на бекплейне, нельзя ставить 0 в задержку!!!
    }else
       if (id->CMD_TYPE[i]==CMD_TEST_SPI)//команда начала теста проверки шины SPI
    {
      FLAG_CMD=1;
      Transf("начинаем тест проверку шины SPI!\r\n");
      FLAG_ASQ_TEST_SPI=0;//сбрасываем флаг ответа на тест 485
      FUNC_FLAG_UP (&POINTER_TEST_SPI,10);//ставим отложенную задачу для опроса кассет на бекплейне, нельзя ставить 0 в задержку!!!
    }else
       if (id->CMD_TYPE[i]==CMD_TEST_JTAG)//команда начала теста проверки шины JTAG
    {
      FLAG_CMD=1;
      Transf("начинаем тест проверку шины JTAG!\r\n");
      FLAG_ASQ_TEST_JTAG=0;//сбрасываем флаг ответа на тест 485
      FUNC_FLAG_UP (&POINTER_TEST_JTAG,10);//ставим отложенную задачу для опроса кассет на бекплейне, нельзя ставить 0 в задержку!!!
    }else
       if (id->CMD_TYPE[i]==CMD_Corr_I)//команда переустановки коэффициента коррекции тока
    {
      FLAG_CMD=1;
      idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище" 
      idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище" //в младшем адресе находятся старшие байты числа!!!
      idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
      idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
      idx4=idx_srv(id->INDEX[i],4);//индекс расположения данных в "хранилище"
    //----------------------------------------------------------------------------------  
      data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      index_ch=srv->MeM[idx0];//индекс канала для корректора
      u_out("Принят коэффициент коррекции измерения тока:",data);
      u_out("в канале:",index_ch);
      TMP_f=data;      
      B330.Corr_I[index_ch]=TMP_f/1000;
      CorrI_write (); 
    }else
       if (id->CMD_TYPE[i]==CMD_Corr_U)//команда переустановки коэффициента коррекции напряжения
    {
      FLAG_CMD=1;
      idx0=idx_srv(id->INDEX[i],0);//индекс расположения данных в "хранилище" 
      idx1=idx_srv(id->INDEX[i],1);//индекс расположения данных в "хранилище" //в младшем адресе находятся старшие байты числа!!!
      idx2=idx_srv(id->INDEX[i],2);//индекс расположения данных в "хранилище"
      idx3=idx_srv(id->INDEX[i],3);//индекс расположения данных в "хранилище"
      idx4=idx_srv(id->INDEX[i],4);//индекс расположения данных в "хранилище"
    //----------------------------------------------------------------------------------  
      data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      index_ch=srv->MeM[idx0];//индекс канала для корректора
      u_out("Принят коэффициент коррекции измерения напряжения:",data);
      u_out("в канале:",index_ch);
      TMP_f=data;      
      B330.Corr_U[index_ch]=TMP_f/1000;
      CorrU_write (); 
    }else
       if (id->CMD_TYPE[i]==CMD_Corr_REQ)//команда запроса списка корректирующих коэффициентов
    {
      FLAG_CMD=1;
      TIME_cons ();
      Transf("\r\nПринят запрос на список корректирющих коэффициентов.\r\n");
      Console_corr();      
      DAT_CORR_REQ_FORM();//заполняем транспортный массив
      //ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда
      ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);
      SYS_CMD_MSG(
			id,//реестр
			&INVOICE[ADR],//структура квитанций	
			i,	 		     	//индекс в реестре
			MSG_Corr_REQ,	//тип сообщения
			64,		 		    //объём данных сообщения в байтах
			DAT_REQ,	    //данные сообщения - массив данных
			TIME_SYS	  	//время составления квитанции
			);
    }
		
    //----------------------------------------------------------
    //---------отсылаем квитанцию и очищаем буфер---------------
    if (FLAG_CMD==1)
    {
      FLAG_CMD=0;
	  //------------------------------ 
      ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//ищем порядковый номер отправителя в структуре отправителей, если его там нет  - то заносим туда     
			ERROR_CMD_MSG ( //заполняем квитанцию о выполнении команды
			id,			        //указатель на реестр
			&INVOICE[ADR],  //указатель на структуру квитанции
			i, 			        //индекс команды в реестре
			MSG_CMD_OK,	    //сообщение квитанции
			0,				      //данные квитанции
			TIME_SYS	      //текущее системное время 
			);	
			SERV_ID_DEL (id,i);//удаляем команду из реестра
    //------------------------------
    }

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

void CONTROL_T1HZ_MK (void)
{
	u8 val=0;
	if ((TIMER_T1HZ_MK>1500)&&(FLAG_T1HZ_MK==1)) 
	{
		FLAG_T1HZ_MK=0;
		Transf("ПРОПАЛ СИГНАЛ T1HZ_MK!!!\r\n");
	}
	B330.FLAG_1HZ=FLAG_T1HZ_MK;
}
  
void LED_CONTROL (void)
{
       u32 z =0;
static u32 z0; 

z=LED_ISPRAV_AC+((LED_PROGR    &3)<< 4)+
				((LED_OFCH     &3)<<21)+
				((LED_SINHR    &3)<<18)+
				((LED_LS       &3)<<16)+
				((LED_ISPR_J330&3)<<14)+
				((LED_OTKL_AC  &1)<<12)+
				((LED_TEMP     &3)<<8);

if (START_BP==0)  z=0;

if (TIMER_LED>LED_INTERVAL) 
	{
		if (z!=z0)
		{
			TCA_WR(z);//выполняем команду
			z0=z;
//			x_out("z:",z);
		}
		TIMER_LED=0;
	}
}

void CONTROL_POK (void)
{
	static u8 var=0;
	if (POK_HDS_OUT!=var)
	{
		var=POK_HDS_OUT;
		u_out("POK_HDS_OUT:",POK_HDS_OUT);
	}
}

//--------------------
int comp (const int *, const int *);
/* сравнение двух целых */
int comp (const int *i, const int *j)
{
  return *i - *j;
}
//--------------------

int FILTR (int data,int *p,u8 n)
{
  int z=0;
  int tmp[32];
  int i=0;
  int k=n/2;

  for (i=0;i<(n-1);i++) p[n-i-1]=p[n-i-2];
  p[0]=data;
  for (i=0;i<n;i++) tmp[i]=p[i];

  qsort(tmp, n, sizeof (int), (int(*) (const void *, const void *)) comp);//сортируем временный массив
  z=tmp[k];

  return z;
}

int cnvrt (int i,int u)
{
  int z=0;
  float x0,x1,y;
  x0=i;
  x1=u;
  x0=x0/100;
  x1=x1/100;
  y=x0*x1;
  z=y*100;
  return z;
}

void CONTROL_SYS (void)
{
	static u8 flag=0;
	u32 tmp0=0;   
	
  if ((START_BP==1)&&(TIMER_CONTROL_SYS>SYS_INTERVAL))
  {
	flag=1;
    TIMER_CONTROL_SYS=0;
    //проверка ID
    LM_MFR_ID(1);
    LM_MFR_ID(2);
    LM_MFR_ID(3);
    LM_MFR_ID(4);
    LM_MFR_ID(5);
    LM_MFR_ID(6);
    LM_MFR_ID(7);
    LM_MFR_ID(8);
    //измерение температуры
    LM1.TEMP=LM_TEMP(1);
    LM2.TEMP=LM_TEMP(2);
    LM3.TEMP=LM_TEMP(3);
    LM4.TEMP=LM_TEMP(4);
    LM5.TEMP=LM_TEMP(5);
    LM6.TEMP=LM_TEMP(6);
    LM7.TEMP=LM_TEMP(7);
    LM8.TEMP=LM_TEMP(8);

    if (LM1.TEMP>tmp0) tmp0=LM1.TEMP;
    if (LM2.TEMP>tmp0) tmp0=LM2.TEMP;
    if (LM3.TEMP>tmp0) tmp0=LM3.TEMP;
    if (LM4.TEMP>tmp0) tmp0=LM4.TEMP;
    if (LM5.TEMP>tmp0) tmp0=LM5.TEMP;
    if (LM6.TEMP>tmp0) tmp0=LM6.TEMP;
    if (LM7.TEMP>tmp0) tmp0=LM7.TEMP;
    if (LM8.TEMP>tmp0) tmp0=LM8.TEMP;

    B330.TEMP_MAX=tmp0;tmp0=0;//4-ре значащих разряда
    //B330.P = 1000;
    //Измерение потребляемого тока
    LM1.I=FILTR (LM_in_i(1),LM1.FI,12);
    LM2.I=FILTR (LM_in_i(2),LM2.FI,12);
    LM3.I=FILTR (LM_in_i(3),LM3.FI,12);
    LM4.I=FILTR (LM_in_i(4),LM4.FI,12);
    LM5.I=FILTR (LM_in_i(5),LM5.FI,12);
    LM6.I=FILTR (LM_in_i(6),LM6.FI,12);
    LM7.I=FILTR (LM_in_i(7),LM7.FI,12);
    LM8.I=FILTR (LM_in_i(8),LM8.FI,12);

    if (LM1.I>tmp0) tmp0=LM1.I;
    if (LM2.I>tmp0) tmp0=LM2.I;
    if (LM3.I>tmp0) tmp0=LM3.I;
    if (LM4.I>tmp0) tmp0=LM4.I;
    if (LM5.I>tmp0) tmp0=LM5.I;
    if (LM6.I>tmp0) tmp0=LM6.I;
    if (LM7.I>tmp0) tmp0=LM7.I;
    if (LM8.I>tmp0) tmp0=LM8.I;

    B330.I = tmp0;tmp0=0;//3-три значащих разряда
//    B330.I = 1000;
    //Измерение напряжения в каналах
    LM1.U=FILTR (LM_v(1),LM1.FU,12);
    LM2.U=FILTR (LM_v(2),LM2.FU,12);
    LM3.U=FILTR (LM_v(3),LM3.FU,12);
    LM4.U=FILTR (LM_v(4),LM4.FU,12);
    LM5.U=FILTR (LM_v(5),LM5.FU,12);
    LM6.U=FILTR (LM_v(6),LM6.FU,12);
    LM7.U=FILTR (LM_v(7),LM7.FU,12);
    LM8.U=FILTR (LM_v(8),LM8.FU,12);

        //Измерение потребляемой мощности
    LM1.P=FILTR (cnvrt (LM1.I,LM1.U),LM1.FP,12);
    LM2.P=FILTR (cnvrt (LM2.I,LM2.U),LM2.FP,12);
    LM3.P=FILTR (cnvrt (LM3.I,LM3.U),LM3.FP,12);
    LM4.P=FILTR (cnvrt (LM4.I,LM4.U),LM4.FP,12);
    LM5.P=FILTR (cnvrt (LM5.I,LM5.U),LM5.FP,12);
    LM6.P=FILTR (cnvrt (LM6.I,LM6.U),LM6.FP,12);
    LM7.P=FILTR (cnvrt (LM7.I,LM7.U),LM7.FP,12);
    LM8.P=FILTR (cnvrt (LM8.I,LM8.U),LM8.FP,12);

    B330.P = LM1.P+LM2.P+LM3.P+LM4.P+LM5.P+LM6.P+LM7.P+LM8.P;//4-ре значащих разряда

    if (LM1.U>tmp0) tmp0=LM1.U;
    if (LM2.U>tmp0) tmp0=LM2.U;
    if (LM3.U>tmp0) tmp0=LM3.U;
    if (LM4.U>tmp0) tmp0=LM4.U;
    if (LM5.U>tmp0) tmp0=LM5.U;
    if (LM6.U>tmp0) tmp0=LM6.U;
    if (LM7.U>tmp0) tmp0=LM7.U;
    if (LM8.U>tmp0) tmp0=LM8.U;  

    B330.U_max=tmp0;tmp0=100000;//4-ре значащих разряда
  //B330.U_max=1000;

    if (LM1.U<tmp0) tmp0=LM1.U;
    if (LM2.U<tmp0) tmp0=LM2.U;
    if (LM3.U<tmp0) tmp0=LM3.U;
    if (LM4.U<tmp0) tmp0=LM4.U;
    if (LM5.U<tmp0) tmp0=LM5.U;
    if (LM6.U<tmp0) tmp0=LM6.U;
    if (LM7.U<tmp0) tmp0=LM7.U;
    if (LM8.U<tmp0) tmp0=LM8.U;   

    B330.U_min=tmp0;tmp0=0; //4-ре значащих разряда
 //   B330.U_min=1000;   

  } else
	  if ((START_BP==0)&&(flag==1))
  {
	flag=0;
  	LM1.TEMP=0xffffffff;
    LM2.TEMP=0xffffffff;
    LM3.TEMP=0xffffffff;
    LM4.TEMP=0xffffffff;
    LM5.TEMP=0xffffffff;
    LM6.TEMP=0xffffffff;
    LM7.TEMP=0xffffffff;
    LM8.TEMP=0xffffffff;
	
  	LM1.U=0;
    LM2.U=0;
    LM3.U=0;
    LM4.U=0;
    LM5.U=0;
    LM6.U=0;
    LM7.U=0;
    LM8.U=0;
	
	  LM1.I=0;
    LM2.I=0;
    LM3.I=0;
    LM4.I=0;
    LM5.I=0;
    LM6.I=0;
    LM7.I=0;
    LM8.I=0;
	
  	LM1.P=0;
    LM2.P=0;
    LM3.P=0;
    LM4.P=0;
    LM5.P=0;
    LM6.P=0;
    LM7.P=0;
    LM8.P=0;
  }
  
  u8 err=0;
  if (B330.TEMP_MAX>5000) err++;
  if (B330.I>2500)        err++;
  if (B330.U_min<1100)    err++;
  if (B330.U_max>1250)    err++;
  if (B330.P    >30000)   err++;
  if (B330.FLAG_1HZ==0)   err++;

  if (err!=0) B330.STATUS_OK=0; else B330.STATUS_OK=1;
}

void ALARM_SYS_TEMP (void)  
{
	u8 var=0;
	if (LM1.TEMP>LM1.TEMP_max) var=var|(1<<0);
	if (LM2.TEMP>LM2.TEMP_max) var=var|(1<<1);
	if (LM3.TEMP>LM3.TEMP_max) var=var|(1<<2);
	if (LM4.TEMP>LM4.TEMP_max) var=var|(1<<3);
	if (LM5.TEMP>LM5.TEMP_max) var=var|(1<<4);
	if (LM6.TEMP>LM6.TEMP_max) var=var|(1<<5);
	if (LM7.TEMP>LM7.TEMP_max) var=var|(1<<6);
	if (LM8.TEMP>LM8.TEMP_max) var=var|(1<<7);
	
	if (var!=0) LED_TEMP=2; else LED_TEMP=1; 
}

void UART_CNTR (UART_HandleTypeDef *huart)
{
	if (huart->gState != HAL_UART_STATE_BUSY_TX)
	{
		NRE_RS485(0);//включаем буфер 485 на приём 
		DE_RS485 (0);//выключаем буфер 485 на передачу
	}
}  

void SETUP_IP0_072 (u8 adr,u32 ip)
{
  u8 a[64];
  for (int i=0;i<64;i++) a[i]=0;

   strcpy(a,"~0 setup_IP0:"); 
   a[1]= adr+0x30;
   sprintf (strng,"%d",ip);
   strcat(a,strng);
   strcat(a,";\r\n");


   Transf ("Отправляем на бекплейн:");
   Transf (a);
   Transf ("\r\n");
   Transf2(a);
}

void SETUP_IP1_072 (u8 adr,u32 ip)
{
  u8 a[64];
  for (int i=0;i<64;i++) a[i]=0;

   strcpy(a,"~0 setup_IP1:"); 
   a[1]= adr+0x30; 
   sprintf (strng,"%d",ip);
   strcat(a,strng);
   strcat(a,";\r\n");


   Transf ("Отправляем на бекплейн:");
   Transf (a);
   Transf ("\r\n");
   Transf2(a);
}

void SETUP_DEST_IP0_072 (u8 adr,u32 ip)
{
  u8 a[64];
  for (int i=0;i<64;i++) a[i]=0;

   strcpy(a,"~0 dest_IP0:"); 
   a[1]= adr+0x30; 
   sprintf (strng,"%d",ip);
   strcat(a,strng);
   strcat(a,";\r\n");


   Transf ("Отправляем на бекплейн:");
   Transf (a);
   Transf ("\r\n");
   Transf2(a);
}

void SETUP_DEST_IP1_072 (u8 adr,u32 ip)
{
  u8 a[64];
  for (int i=0;i<64;i++) a[i]=0;

   strcpy(a,"~0 dest_IP1:"); 
   a[1]= adr+0x30; 
   sprintf (strng,"%d",ip);
   strcat(a,strng);
   strcat(a,";\r\n");


   Transf ("Отправляем на бекплейн:");
   Transf (a);
   Transf ("\r\n");
   Transf2(a);
}

void REQ_VERSIYA (void)
{
  u32 tmp0,tmp1;
  tmp0=STM32_VERSION>>16;   //тут дата
  tmp1=STM32_VERSION&0xffff;//тут время
  Transf("----------------------\r\n");
  Transf("версия прошивки STM32:\r\n");
  xn_out("Дата:",tmp0);Transf("  ");x_out("Время:",tmp1); 
}

//разбираем ответ от кассеты на бекплейне
void answer_translated (u32 dat)
{
  u8 adr=0;
  if (POINTER_ADR_REQ.timer>0)//флаг запроса ставится не равным нулю!
  {
    adr=NUMBER_OF_B072++;
    ADR_SLAVE[adr]=dat;
  }
}

//процедура сбора адресов с слейвов - блоков 072 на бекплейне обмена с таймаутом
void SLAVE_COUNT ()
{
  int i=0;
  int tmp0;

   tmp0=NUMBER_OF_B072;//сколько пришло ответов
   u_out("Всего блоков:",tmp0);
   Transf("-------------\r\n");
   for (i=0;i<tmp0;i++)
    {
      u_out("Блок 072:",ADR_SLAVE[i]);
    }
}

//посылаем запрос на бекплейн про адреса блоков 072
void req_col (void)
{
  Transf2("~0 REQ_ADR;");//отсылаем запрос 
  NUMBER_OF_B072=0;//сбрасываем счётчик числа блоков
}

//посылаем команду на переконфигурирование ETH MAC ячеек 072
void CMD_MAC_RECONF ()
{
  Transf2("~0 eth_config;");//отсылаем запрос 
}

//запускает отложенные задачи
void DISPATCHER (u32 timer)
{
	u32 tmp0=0;
	int i=0;

      if (FLAG_DWN(&POINTER_ADR_COLLECT))
      {
        TIME_cons ();
        Transf("Идёт сбор адресов!\r\n");
        req_col ();//запрашиваем адреса
        FUNC_FLAG_UP (&POINTER_ADR_REQ,3000);//поднимаем флаг следующей задачи - вывод количества блоков 072 в консоль и определения их количества
        return;
      } else
      if (FLAG_DWN(&POINTER_ADR_REQ))
      {
        TIME_cons ();
        SLAVE_COUNT ();
        if (NUMBER_OF_B072>0) FUNC_FLAG_UP (&POINTER_MASTER_IP0_SETUP,200);//поднимаем флаг следующей задачи - установка блокам 072 IP0 адресов, если эти блоки есть
        return;
      } else
      if (FLAG_DWN(&POINTER_MASTER_IP0_SETUP))
      {
        TIME_cons ();
        Transf("Устанавливаем мастер IP0!\r\n");
        SETUP_IP0_072 (ADR_SLAVE[0],MASTER_IP0);//отсылаем IP0 мастеру , он всегда стоит раньше всех на бекплейне
        FUNC_FLAG_UP (&POINTER_SLAVE_IP0_SETUP,100);     //поднимаем флаг следующей задачи - установка блокам 072 IP1 адресов
        return;
      } else
       if (FLAG_DWN(&POINTER_SLAVE_IP0_SETUP))
      {
        TIME_cons ();
        Transf("Устанавливаем слейв IP0!\r\n");
        SETUP_IP0_072 (ADR_SLAVE[1],SLAVE_IP0); //отсылаем IP0 слейву, он стоит позже по бекплейну
        FUNC_FLAG_UP (&POINTER_MASTER_IP1_SETUP,100);     //поднимаем флаг следующей задачи - установка блокам 072 IP1 адресов
        return;
      } else
      if (FLAG_DWN(&POINTER_MASTER_IP1_SETUP))
      {
        TIME_cons ();
        Transf("Устанавливаем мастер IP1!\r\n");
        SETUP_IP1_072 (ADR_SLAVE[0],MASTER_IP1);//отсылаем IP1 мастеру , он всегда стоит раньше всех на бекплейне
        FUNC_FLAG_UP  (&POINTER_SLAVE_IP1_SETUP,100);//поднимаем флаг следующей задачи - установка блокам 072 IP1 адресов
        return;
      } else
        if (FLAG_DWN(&POINTER_SLAVE_IP1_SETUP))
      {
        TIME_cons ();
        Transf("Устанавливаем слейв IP1!\r\n");
        SETUP_IP1_072 (ADR_SLAVE[1],SLAVE_IP1); //отсылаем IP1 слейву, он стоит позже по бекплейну
        FUNC_FLAG_UP  (&POINTER_DEST_MASTER_IP0_SETUP,100);//поднимаем флаг следующей задачи - установка блокам 072 IP1 адресов
        return;
      } else
      if (FLAG_DWN(&POINTER_DEST_MASTER_IP0_SETUP))
      {
        TIME_cons ();
        Transf("Устанавливаем мастер DEST_IP0!\r\n");
        SETUP_DEST_IP0_072 (ADR_SLAVE[0],MASTER_DEST_IP0);//отсылаем IP1 мастеру , он всегда стоит раньше всех на бекплейне
        FUNC_FLAG_UP       (&POINTER_DEST_SLAVE_IP0_SETUP,100);    //поднимаем флаг следующей задачи - установка блокам 072 DEST_IP1 адресов
        return;
      } else
      if (FLAG_DWN(&POINTER_DEST_SLAVE_IP0_SETUP))
      {
        TIME_cons ();
        Transf("Устанавливаем слейв DEST_IP0!\r\n");
        SETUP_DEST_IP0_072 (ADR_SLAVE[1], SLAVE_DEST_IP0);//отсылаем IP1 слейву, он стоит позже по бекплейну
        FUNC_FLAG_UP       (&POINTER_DEST_MASTER_IP1_SETUP,100);    //поднимаем флаг следующей задачи - установка блокам 072 DEST_IP1 адресов
        return;
      } else
      if (FLAG_DWN(&POINTER_DEST_MASTER_IP1_SETUP))
      {
        TIME_cons ();
        Transf("Устанавливаем мастер DEST_IP1!\r\n");
        SETUP_DEST_IP1_072 (ADR_SLAVE[0],MASTER_DEST_IP1);//отсылаем IP1 мастеру , он всегда стоит раньше всех на бекплейне
        FUNC_FLAG_UP       (&POINTER_DEST_SLAVE_IP1_SETUP,100);    //поднимаем флаг следующей задачи - реконфиг мак-ков 072 по ранее установленным IP
        return;
      } else
      if (FLAG_DWN(&POINTER_DEST_SLAVE_IP1_SETUP))
      {
        TIME_cons ();
        Transf("Устанавливаем слейв DEST_IP1!\r\n");
        SETUP_DEST_IP1_072 (ADR_SLAVE[1], SLAVE_DEST_IP1);//отсылаем IP1 слейву, он стоит позже по бекплейну
        FUNC_FLAG_UP       (&POINTER_ETHERNET_RERUN,100); //поднимаем флаг следующей задачи - реконфиг мак-ков 072 по ранее установленным IP
        return;
      } else
      if (FLAG_DWN(&POINTER_ETHERNET_RERUN))
      {
        TIME_cons ();
        Transf("Рестарт МАК ячеек 072!\r\n");
        CMD_MAC_RECONF ();
        return;
      } else
       if (FLAG_DWN(&POINTER_TEST_485))
      {
		    FUNC_FLAG_UP (&POINTER_TEST_485_REQ,500);//ставим отложенную задачу для проверки наличия ответа на тест 485
        TIME_cons ();
        Transf("Посылаем код по шине 485!\r\n");
        tmp0=ADR_SLAVE[1];//
        BUS_485_TEST (tmp0);
        return;
      } else		
	    if (FLAG_DWN(&POINTER_TEST_485_REQ))
      {
        TIME_cons ();
        Transf("Проверяем результат теста шины 485!\r\n");
        if (FLAG_ASQ_TEST_485==1) Transf("Тест пройден!\r\n");
		    else
        {
          if (SCH_TST1<3)
          {
            SCH_TST1++;
            FUNC_FLAG_UP (&POINTER_TEST_485,10);//повтор теста
          } else Transf("Тест не пройден!\r\n");
        }    

        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_485);//готовим квитанцию серверу по результатам теста
        return;
      } else
       if (FLAG_DWN(&POINTER_TEST_SPI))
      {
		    FUNC_FLAG_UP (&POINTER_TEST_SPI_REQ,10);//ставим отложенную задачу для проверки наличия ответа на тест 485
        TIME_cons ();
        Transf("Посылаем код по шине SPI!\r\n");
        FLAG_ASQ_TEST_SPI=0;
        tmp0=ADR_SLAVE[1];//
        u_out("Adr:",ADR_SLAVE[1]);
		    if (tmp0==8) tmp0=0;//поправка для 8-го адресного места на шине бекплейна!
        SPI_BP_WRITE (tmp0,0xDEEDBEEF);//посылаем код по адресу ADR_SLAVE[1]
        return;
      }else		
	    if (FLAG_DWN(&POINTER_TEST_SPI_REQ))
      {
        TIME_cons ();
        Transf("Проверяем результат теста шины SPI!\r\n");
        tmp0=ADR_SLAVE[1];//
		    if (tmp0==8) tmp0=0;//поправка для 8-го адресного места на шине бекплейна!
        tmp0=SPI_BP_READ (tmp0);//считываем код из кассеты
        x_out("Ancwer:",tmp0);
        if (tmp0==0xDEEDBEEF) {Transf("Тест пройден!\r\n");FLAG_ASQ_TEST_SPI=1;}
		    else               Transf("Тест не пройден!\r\n");
        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_SPI);//готовим квитанцию серверу по результатам теста
        return;
      } else
       if (FLAG_DWN(&POINTER_TEST_JTAG))
      {
		    FUNC_FLAG_UP (&POINTER_TEST_JTAG_REQ,500);//ставим отложенную задачу для проверки наличия ответа на тест 485
        TIME_cons ();
        Transf("Опрашиваем шину JTAG!\r\n");
        FLAG_ASQ_TEST_JTAG=0;
        tmp0=jtag_scan(NULL,0);
        u_out("tmp0:",tmp0);
        return;
      }else		
	    if (FLAG_DWN(&POINTER_TEST_JTAG_REQ))
      {
        TIME_cons ();
        Transf("Проверяем результат теста шины JTAG!\r\n");
        u_out("Количество устройств:",jtag_dev_count);
        for(i=0;i<jtag_dev_count;i++)
        {
          un_out("",i);Transf("] ");xn_out("",jtag_devs[i].idcode);Transf(" - ");Transf(jtag_devs[i].descr);Transf("\r\n");
        }        

        if (jtag_dev_count==2)
        {
        	if (jtag_devs[1].idcode==0x2a020dd)
        	{     Transf("Тест пройден!\r\n"   );FLAG_ASQ_TEST_JTAG=1;}
		    else  Transf("Тест не пройден!\r\n");
        } 

        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_JTAG);//готовим квитанцию серверу по результатам теста
        return;
      } 	 

  }

//эта процедура поднимает переданный ей флаг и устанавливает таймаут на заданое время
void FUNC_FLAG_UP (POINTER *p,u32 time)
{
  int i=0;
  p->FLAG=0;
  p->timer=time;
  while (PNT[i]!=0) {i++;};//ищем свободное место в массиве указателей
  PNT[i]=p;

  Transf("Поднимаем флаг отложенного задания!\r\n");
  TIME_cons ();
  u_out("timeout:",time);
}

//эта процедура снимает переданный ей флаг
u8 FLAG_DWN (POINTER *p)
{
  if (p->FLAG==1)
  {
  	  p->FLAG=0;
  	  return 1;
  } else return 0;
}

void FTIME_OF_WORK ()
{
	if (TIME_OF_SECOND>600) 
	{
		TIME_OF_SECOND=0;
		TIME_OF_WORK++;
		FLAG_TIME_OF_WORK_WRITE=1;
	}
}

u32 buf_to_data (u8 *p)
{
	u32 a=0;
	a=(p[0]<<24)|(p[1]<<16)|(p[2]<<8)|(p[3]<<0);//старший байт в младшем адресе!!!
	return a;
}

void TABL_CONS (u8 *p,u32 n)
{
  int i=0;
   for (i=0;i<n;i++)
   {
     hn_out (p[i+0],0);hn_out (p[i+1],0);hn_out (p[i+2],0);hn_out (p[i+3],0);
     i=i+3;
     Transf("\r\n");	
   }	
}

void EPCS1_READ (u32 adr,u8 *p,u32 n)
{
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//проверяем что флеш не занята
  spi_EPCS1_read(READ_BYTES,adr,p,n);//чтение n байт данных
}

void EPCS2_READ (u32 adr,u8 *p,u32 n)
{
  while ((spi_EPCS2_STATUS()&1)==1){WATCH_DOG ();};	//проверяем что флеш не занята
  spi_EPCS2_read(READ_BYTES,adr,p,n);//чтение n байт данных
}

void EPCS1_WRITE_BUF(u32 adr,u8 *p,u32 n)
{
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//проверЯем что флеш не занЯта
  spi_EPCS1_wr_ENABLE(); //разрешаем запись во флеш
  spi_EPCS1_write(WRITE_BYTES,adr,p,n);
  spi_EPCS1_wr_DISABLE();//запрещаем запись во флеш
}

void EPCS2_WRITE_BUF(u32 adr,u8 *p,u32 n)
{
  while ((spi_EPCS2_STATUS()&1)==1){WATCH_DOG ();};	//проверЯем что флеш не занЯта
  spi_EPCS2_wr_ENABLE(); //разрешаем запись во флеш
  spi_EPCS2_write(WRITE_BYTES,adr,p,n);
  spi_EPCS2_wr_DISABLE();//запрещаем запись во флеш
}

void EPCS1_ERASE_SECTOR(u32 adr)
{
  u8 a[1]={0xff};
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//проверяем что флеш не занята
  spi_EPCS1_wr_ENABLE();//разрешаем запись во флеш
  spi_EPCS1_write(ERASE_SECTOR,adr,a,0);
  spi_EPCS1_wr_DISABLE();//запрещаем запись во флеш
}

void EPCS2_ERASE_SECTOR(u32 adr)
{
  u8 a[1]={0xff};
  while ((spi_EPCS2_STATUS()&1)==1){WATCH_DOG ();};	//проверяем что флеш не занята
  spi_EPCS2_wr_ENABLE();//разрешаем запись во флеш
  spi_EPCS2_write(ERASE_SECTOR,adr,a,0);
  spi_EPCS2_wr_DISABLE();//запрещаем запись во флеш
}

void EPCS2_ERASE_ALL(void)
{
  while ((spi_EPCS2_STATUS()&1)==1){WATCH_DOG ();};	//проверяем что флеш не занята
  spi_EPCS2_wr_ENABLE ();//разрешаем запись во флеш
  spi_EPCS2_ERASE_BULK();//стираем всЮ во флеш
  spi_EPCS2_wr_DISABLE();//запрещаем запись во флеш
}

void EPCS1_ERASE_ALL(void)
{
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//проверЯем что флеш не занЯта
  spi_EPCS1_wr_ENABLE ();//разрешаем запись во флеш
  spi_EPCS1_ERASE_BULK();//стираем всЮ во флеш
  spi_EPCS1_wr_DISABLE();//запрещаем запись во флеш
}

void SERIAL_NUMBER_WR (u32 data)
{
	u8 buf[4];
  EPCS1_ERASE_SECTOR(SERIAL_ADR_FLASH);//стираем сектор коэффициентов коррекции тока
	u_out("Записываем серийный номер:",data);
	buf[0]=(data>>24)&0xff;
	buf[1]=(data>>16)&0xff;
	buf[2]=(data>> 8)&0xff;
	buf[3]=(data>> 0)&0xff;
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//проверЯем что флеш не занЯта
  spi_EPCS1_wr_ENABLE(); //разрешаем запись во флеш
  spi_EPCS1_write(WRITE_BYTES,SERIAL_ADR_FLASH,buf,4);
  spi_EPCS1_wr_DISABLE();//запрещаем запись во флеш
}

//запись коэффициентов коррекции измерения тока
void PACK_ARR_FLOAT (float *p,u8 *a,u32 n)
{
  int i=0;
  int j=0;
  int tmp=0;
  for (i=0;i<n;i++)
  {
     tmp=p[i]*1000;
     a[j+0]=(tmp>>24)&0xff;
     a[j+1]=(tmp>>16)&0xff;
     a[j+2]=(tmp>> 8)&0xff;
     a[j+3]=(tmp>> 0)&0xff;
     j=j+4;
  }
}

void CorrIU_write (void) 
{
  int x=0;
  u8 Arr[32];
  EPCS1_ERASE_SECTOR(CorrI_ADR_FLASH);//стираем сектор коэффициентов коррекции тока
  PACK_ARR_FLOAT (B330.Corr_I,Arr,8);
  x=8*4;
  EPCS1_WRITE_BUF(CorrI_ADR_FLASH,Arr,x);//записываем весь массив коэффициентов на флеш
  PACK_ARR_FLOAT (B330.Corr_U,Arr,8);
  x=8*4;
  EPCS1_WRITE_BUF(CorrU_ADR_FLASH,Arr,x);//записываем весь массив коэффициентов на флеш
}

void CorrI_write (void) 
{
  u8 Arr[32];
  EPCS1_ERASE_SECTOR(CorrI_ADR_FLASH);//стираем сектор коэффициентов коррекции тока
  PACK_ARR_FLOAT (B330.Corr_I,Arr,8);
  int x=8*4;
  EPCS1_WRITE_BUF(CorrI_ADR_FLASH,Arr,x);//записываем весь массив коэффициентов на флеш
}

 //запись коэффициентов коррекции измерения напряжения
void CorrU_write (void)
{
   u8 Arr[32];
   EPCS1_ERASE_SECTOR(CorrU_ADR_FLASH);//стираем сектор коэффициентов коррекции тока
   PACK_ARR_FLOAT (B330.Corr_U,Arr,8);
   int x=8*4;
   EPCS1_WRITE_BUF(CorrU_ADR_FLASH,Arr,x);//записываем весь массив коэффициентов на флеш
}

void CorrI_read (void)
{
  u8 a[64];
  int x=8*4;//размер массива в байтах
  int i=0;
  int   tmp =0;
  float tmp1=0;
  EPCS1_READ (CorrI_ADR_FLASH,a,x);
  for (i=0;i<8;i++) 
  {
    tmp =(a[4*i]<<24)+(a[4*i+1]<<16)+(a[4*i+2]<<8)+(a[4*i+3]<<0);
    tmp1=tmp;
    B330.Corr_I[i]=tmp1/1000;
  }
} 

void CorrU_read (void)
{
  u8 a[64];
  int x=8*4;//размер массива в байтах
  int i=0;
  int   tmp =0;
  float tmp1=0;
  EPCS1_READ (CorrU_ADR_FLASH,a,x);
  for (i=0;i<8;i++) 
  {
    tmp =(a[4*i]<<24)+(a[4*i+1]<<16)+(a[4*i+2]<<8)+(a[4*i+3]<<0);
    tmp1=tmp;
    B330.Corr_U[i]=tmp1/1000;
  }
} 

//Считывает серийный номер из флешпамяти
u64 SERIAL_NUMBER ()
{
   u8 buf[4];//{}
   int adr=SERIAL_ADR_FLASH;
   u64 data=0;

   spi_EPCS1_read(READ_BYTES,adr,buf,4);
   data=buf_to_data(buf);
   u_out("Серийный номер блока:",data);
   return data;
}

int main(void)
{
	int i=0;
  /* USER CODE BEGIN 1 */
  Adress=0x30; //адресс кассеты 

  for (i=0;i<PNT_BUF;i++) PNT[i]=NULL;//обнуляем ассив указателей
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
//MX_TIM4_Init();  НЕ НУЖЕН, В СХЕМЕ НЕТ БОЛЬШЕ ШИМа!!!
//MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
//MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

//  Delay(1000);
	RESET_072(0);
//------SETUP----------
LM1.TEMP_max=5000;//первые два числа десятки и еденицы, вторы два числа десятые и сотые
LM2.TEMP_max=5000;
LM3.TEMP_max=5000;
LM4.TEMP_max=5000;
LM5.TEMP_max=5000;
LM6.TEMP_max=5000;
LM7.TEMP_max=5000;
LM8.TEMP_max=5000;
//---------------------
  
  Transf("-------------\r\n");
  Transf("    Б330\r\n");
  Transf("-------------\r\n");
  DE_RS485(1);
  CS_5_MK(1);
  CS_EPCS1(1);
  CS_EPCS2(1);  
  //---убрать!!!--------
  //PE14_0;
  //PE12_0;
  //--------------------
  
  PWDN_4(1);
  
  ENABLE_LM25056_MK(1);//enable 
  Delay(2);
  ENABLE_LM25056_MK(0);//enable 
  PWR_072 (255);	   //снять питание на кассеты	
  RESET_TCA6424A_MK(0);
  Delay(2);
  RESET_TCA6424A_MK(1); 
  
  VD3(0);
  VD4(0);
  VD5(0);
  
 WDI_MK(0); 
 
HAL_UART_Receive_IT(&huart1,RX_uBUF,1);
HAL_UART_Receive_IT(&huart2,RX_uBUF,1);
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
 RECEIVE_udp(0, 3001,1); 
 
 for (i=0;i<8;i++) ADR_SLAVE[i]=0xff;//очищаем массив с адресами слейвов (тут храним адреса для обмена по 485 шине)

//-------------------------------------
//           Для сдачи по ТУ

//  TCA_WR(255);//зажигаем все светодиоды на лицевой панели
//-------------------------------------
  RESET_072(1);//снимаем ресет 072 кассет
 
  B330.B330_NUMBER=SERIAL_NUMBER ();
  CorrI_read  ();  //загружаем поправочные коэффициенты для коррекции измерений тока
  CorrU_read  ();  //загружаем поправочные коэффициенты для коррекции измерений напряжения
  Console_corr();//выводим загруженные коэффициенты
  B330.PRG_VERSIYA=STM32_VERSION;
  B330.WORK_TIME=TIME_OF_WORK; //время наработки блока в десятках минут;

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
	
	if (EVENT_INT3==1)//контроль секундной метки T1HZ_MK
	{
		//Transf("ЕСТЬ   СИГНАЛ T1HZ_MK!\r");
		if (FLAG_T1HZ_MK==0) Transf("ЕСТЬ   СИГНАЛ T1HZ_MK!\r");
		if (FLAG_ADRES_SENDER_CMD==1) SYS_INFO_SEND_UDP(&ID_SERV1,&SERV1);//отсылаем квитанцию о нашем состоянии
		EVENT_INT3=0;
		FLAG_T1HZ_MK=1;
		TIMER_T1HZ_MK=0;
		TIME_OF_SECOND++;//подсчитываем число секунд с момента включения			
	}; 

  DISPATCHER        (TIMER_TIMEOUT);//выполняет отложенные задачи
	ALARM_SYS_TEMP    ();//сравниваем измеренную температуру с пороговым значением  
  CONTROL_SYS       ();//проверяем параметры системы: температуру , ток потребление и т.д.
	CONTROL_POK 	    ();
	LED_CONTROL 	    ();
	CONTROL_T1HZ_MK   ();
	CMD_search        (&ID_SERV1,&SERV1);
	SEND_UDP_MSG 	    ();
  UART_DMA_TX  	    ();
	UART_DMA_TX2  	  ();
	UART_CNTR         (&huart2);//тут управляем драйвером 485
	FTIME_OF_WORK     ();     //тут следим за временем наработки

  }

}
