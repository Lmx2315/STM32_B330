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
#define LED_INTERVAL 500  		 // �������� ���������� ��������� �����������
#define SYS_INTERVAL 250

u64 STM32_VERSION = 0x190720211536;//����� ������ �������� 12-41 ����� � 18-06-2021 ����
u32 IP_my=0;
u16 PORT_my=0;

u32 TIMER_CONTROL_SYS;   		//���������� ������� �������� ��������� ������� ����� �������

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
u8   Adress_REZ='0';
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
u64 TIME_SYS     =0;//���������� ������� ��������� ����� � ������������
u32 TIME_TEST    =0;
u32 ID_CMD       =0;//���������� ������� ������� ID ����� ��������� 
u32 FLAG_T1HZ_MK =0;//��� ���� ���������� ��� � ������� ���� ������ ���� ���� ��������� �����
u32 TIMER_T1HZ_MK=0;//��� ����������� ������ ��� �������� ������� ��������� �����
u8  PWR_CHANNEL=255;
u32 TIMER_LS=0;
u8 FLAG_LS=0;


//����� �����������
u8  LED_ISPRAV_AC=0;
u8  LED_PROGR    =0;
u8  LED_OFCH     =0;
u8  LED_SINHR    =0;
u8  LED_LS       =0;
u8  LED_ISPR_J330=0;
u8  LED_OTKL_AC  =0;
u8  LED_TEMP	 =0;
u32 TIMER_LED    =0;

LM_struct LM[8];

u8 LM_ID_CN[8];
u8 D_TEMP[4];
int TMP_v=0;

u8 FLAG_TIME_OF_WORK_WRITE=0; //���� ��������� ������ �� ���� ��� �������� �����
u32 TIME_OF_SECOND=0;         //c������ ������ � ������ ������
u32 TIME_OF_WORK=0;           //����� ��������� ����� � �������� �����
SYS_STATE_BOARD B330;         //��������� ���������� ��������� ����� �330
//----------������ ������ 072 �� ���������--------------
/*
u32 MASTER_IP0     =0x0103073d;
u32 MASTER_IP1     =0x0103063c;
u32 MASTER_DEST_IP0=0x01030701;
u32 MASTER_DEST_IP1=0x01030601;

u32 SLAVE_IP0      =0x0103023d;
u32 SLAVE_IP1      =0x0103013c;
u32 SLAVE_DEST_IP0 =0x01030201;
u32 SLAVE_DEST_IP1 =0x01030101;
*/
u32 MASTER_IP0     =0xffffffff;
u32 MASTER_IP1     =0xffffffff;
u32 MASTER_DEST_IP0=0xffffffff;
u32 MASTER_DEST_IP1=0xffffffff;

u32 SLAVE_IP0      =0xffffffff;
u32 SLAVE_IP1      =0xffffffff;
u32 SLAVE_DEST_IP0 =0xffffffff;
u32 SLAVE_DEST_IP1 =0xffffffff;
//------------------------------------------------------
u8  FLAG_ASQ_TEST_485  =0;    //���� ������ �� ������ ����� �� 485 ����
u8  FLAG_ASQ_TEST_JTAG =0;    //���� ������ �� ������ ����� �� SPI ����
u8  FLAG_ASQ_TEST_SPI  =0;    //���� ������ �� ������ ����� �� JTAG ����
u8  FLAG_ASQ_TEST_RESET=0;    //���� ������ �� ������ �������� ������� RESET

u8  FLAG_CHECK_RST_072=0;    //���� �������� ������� ����� ��� ������ 072

POINTER * PNT[PNT_BUF];  //������ ���������� �� ���������� ������

POINTER  POINTER_TEST_485_REQ;         // 0  ���� ����������� �������� ������ �� ������ �� ���� 485 
POINTER  POINTER_TEST_485;             // 1  ���� ����������� ���� �������� ���� 485
POINTER  POINTER_TEST_SPI_REQ;         // 2  ���� ����������� �������� ������ �� ������ �� ���� SPI 
POINTER  POINTER_TEST_SPI;             // 3  ���� ����������� ���� �������� ���� SPI
POINTER  POINTER_TEST_JTAG_REQ;        // 4  ���� ����������� �������� ������ �� ������ �� ���� JTAG 
POINTER  POINTER_TEST_JTAG;            // 5  ���� ����������� ���� �������� ���� JTAG
POINTER  POINTER_ADR_COLLECT;          // 6  ���� ����������� ���� ������� � ����������
POINTER  POINTER_MASTER_IP0_SETUP;     // 7  ���� ����������� ������� IP0 ������� �������� 072 �� ���������
POINTER  POINTER_SLAVE_IP0_SETUP;      // 8
POINTER  POINTER_MASTER_IP1_SETUP;     // 9  ���� ����������� ������� IP1 ������� �������� 072 �� ���������
POINTER  POINTER_SLAVE_IP1_SETUP;      // 10 
POINTER  POINTER_DEST_MASTER_IP0_SETUP;// 11  ���� ����������� ������� DEST_IP0 ������� �������� 072 �� ���������
POINTER  POINTER_DEST_SLAVE_IP0_SETUP; // 12
POINTER  POINTER_DEST_MASTER_IP1_SETUP;// 13  ���� ����������� ������� DEST_IP1 ������� �������� 072 �� ���������
POINTER  POINTER_DEST_SLAVE_IP1_SETUP; // 14
POINTER  POINTER_ADR_REQ;              // 15  ���� ������� ������ �� ������ �� ���������, ����������� ����� ������������� 
POINTER  POINTER_ETHERNET_RERUN;       // 16  ���� �� �������� ���������� ������� ��� �������������������� ����� ����� 072
POINTER  POINTER_RESET_072_0;          // 17  ���� ��������������� ����� ��� ������ 072
POINTER  POINTER_RESET_072_1;          // 18  ���� ��������� ����� ��� ������ 072    
POINTER  POINTER_CHECK_RESET_072;      // 19  ���� ����������� ��� ��������� �������� ����� ������� 072  
POINTER  POINTER_LED_TEST0;            // 20  
POINTER  POINTER_LED_TEST1;            // 21 
POINTER  POINTER_LED_TEST2;            // 22
POINTER  POINTER_LED_TEST3;            // 23    
POINTER  POINTER_RESET;                // 24  
POINTER  POINTER_START_072;            // 25 

u8  ADR_SLAVE [8];          //��� ������ ������ ������ �� ����� ��
u8  NUMBER_OF_B072;         //����� ������ �072 �� ���������
u32 TIMER_TIMEOUT=0;        //������ �������� �������� �������
u8 FLAG_LED_TEST=0;
u8 SCH_TST1=0;//������� ����� �������
u8 SCH_TST2=0;
u8 SCH_TST3=0;
int index_ch;
float TMP_f=0;
u8 FLAG_CMD=0;
u8 DAT_REQ[BUF_DATA_SZ];//������������ ������
SYS_STATE_072 B072[8];  //������ �������� ��������� 072 �����
u16 TEMP_MAX=5500;//����������� ���������� ����������� 50 ����
#define COL 512
u8 DATA_TR [COL];


//-----------------------------------------------------------------------------
//                           �������� �������� ���������� � ���������
/* USER CODE END PV */

 Frame INVOICE[quantity_SENDER];//��������� ��������� � ��������� ���, �� ����� ������������� ���������
 Frame FRM1[quantity_SENDER];	//�������� �����, �� ����� ������������� ���������
 SERVER SERV1;					//��������� "���������"
 ID_SERVER ID_SERV1;			//��������� ��������� ��� "���������"
 CMD_RUN CMD1;
 ADR_SENDER ADDR_SNDR;			//��������� ������ ������ ������� ������������ 

u64 ADRES_SENDER_CMD=0; //��� ������ ����� ���������� ����������, ��� ����������� � ��� ���������
u8 FLAG_ADRES_SENDER_CMD=0;//���� ���� ��� ���� ���� ���������� ���������
u8 FLAG_REQ_STATUS=0;
u8 FLAG_REJ_OSNPAR=0; //���� ������ � ������ ��������� �������� ����������
u8 FLAG_ERROR_REJ_OSNPAR=0;//���� ����������� ���� � ������ �������� �������� ���������� ���-�� ������� �������
ERROR_STRUCT ERROR_FLAG;   //������ ������ 
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
u8 PWR_ALL=255;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;// ��� ��� ������ �� ����������� ��������� ������
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;//
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  */
  
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;// ��� ��� ������ �� ������ 16 ���
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  
  /*
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;// ��� ��� ������ �� ������ 8 ���
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
  GPIO_InitStruct.Pull  = GPIO_PULLUP;
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

void spi4send32 (u32 d) //32 ����
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

u8 spi4send8 (u8 d) //8 ���
{
	u8 a1;
	u8  b;
  a1 = (d)      &0xff;  
  HAL_SPI_TransmitReceive(&hspi4, &a1, &b,1, 5000); 
  return b; 
}

u8 spi5send8 (u8 d) //8 ���
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

void Transf(const char* s)  // ��������� �������� ������ �������� � ����
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

void Transf2(const char* s)  // ��������� �������� ������ �������� � ����
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

void x_out (char s[],u32 a)//���� u64 
{
   Transf (s);
   sprintf (strng,"%X",a);
   Transf(strng);
   Transf ("\r\n");
}

void xn_out (char s[],u32 a)//���� u64 
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

//������� ���������� ����� �������� ��� IP ������
void xun_out (char s[],u32 a)//���� u64 
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

void ARRAY_DATA (u32 a)  //��������� 32 ������ ���������� �� ����� � ��� � ������������ ������
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
    text_lengh=0u;  //��������� �������� ������ 
    flag_pachka_TXT=1; //������������� ���� ��������
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
			text_lengh=0u;  //��������� �������� ������ 
			flag_pachka_TXT=1; //������������� ���� ��������
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
			NRE_RS485(1);//��������� ����� 485 �� ���� 
			DE_RS485(1);//�������� ����� 485 �� ��������

			k = text_lengh2;
			HAL_UART_Transmit_DMA(&huart2,(uint8_t *)text_buffer2,k);
			text_lengh2=0u;  //��������� �������� ������ 
			flag_pachka_TXT2=1; //������������� ���� ��������
			timer_DMA1_Stream6=0;
		  }
	}	
} 

void spisend32 (u32 d) //32 ����
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

u8 spisend8 (u8 d) //8 ���
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
   for (i=0;i<(size/8);i++) d[i] = spi5send8 (0x00);  //��������� ������
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
   spi5send8 (adr|0x80); //������� ������
   k--;
   for (i=0;i<(size/8);i++) spi5send8 (d[k-i]); //����� ������
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

u8 spi_EPCS1_STATUS (void)			//�������� ��������� ���� �� ���� , ������� ��� - ��� ������, ��� ���������
{
	u8 m[1];
	spi_EPCS1_rd(READ_STATUS,m,1);
	return m[0];
}

u8 spi_EPCS2_STATUS (void)			//�������� ��������� ���� �� ���� , ������� ��� - ��� ������, ��� ���������
{
	u8 m[1];
	spi_EPCS2_rd(READ_STATUS,m,1);
	return m[0];
}
void spi_EPCS1_rd (u8 cmd,u8 d[],u32 n) //������ ������
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

void spi_EPCS2_rd (u8 cmd,u8 d[],u32 n) //������ ������
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

void spi_EPCS1_read (u8 cmd,u32 adr,u8 d[],u32 n) //������ ������
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

void spi_EPCS2_read (u8 cmd,u32 adr,u8 d[],u32 n) //������ ������
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

void spi_EPCS1_write (u8 cmd,u32 adr,u8 d[],u32 n) //������ ������ � ���� ������ - 256 ����!!!
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

void spi_EPCS2_write (u8 cmd,u32 adr,u8 d[],u32 n) //������ ������ � ���� ������ - 256 ����!!!
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

void spi_EPCS1_ERASE_BULK (void) //���������� ������ �� ����
{  
   CS_EPCS1(0);  
   spisend8(ERASE_BULK);//
   CS_EPCS1(1);
}

void spi_EPCS2_ERASE_BULK (void) //���������� ������ �� ����
{  
   CS_EPCS2(0);  
   spisend8(ERASE_BULK);//
   CS_EPCS2(1);
}

void spi_EPCS1_wr_ENABLE (void) //���������� ������ �� ����
{  
   CS_EPCS1(0);  
   spisend8(WRITE_ENABLE);//
   CS_EPCS1(1);
}

void spi_EPCS2_wr_ENABLE (void) //���������� ������ �� ����
{  
   CS_EPCS2(0);  
   spisend8(WRITE_ENABLE);//
   CS_EPCS2(1);
}

void spi_EPCS1_wr_DISABLE (void) //���������� ������ �� ����
{  
   CS_EPCS1(0);  
   spisend8(WRITE_DISABLE);//
   CS_EPCS1(1);
}

void spi_EPCS2_wr_DISABLE (void) //���������� ������ �� ����
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
//             �������� ����� "���������"
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

void SPI_BP_WRITE_1 (u32 adr,u8 dop,u32 data)
{
  adr=(adr<<4)|dop;
  FPGA_wSPI (32,adr,data);
}

void SPI_BP_WRITE (u32 adr,u32 data)
{
  adr=(adr<<4)|0x2;
  FPGA_wSPI (32,adr,data);
}

u32 SPI_BP_READ_TEST (u32 adr)//������������ ���������� �� ������� �������� ������������ ����� 0xdeedbeef
{
  u32 tmp0;
  adr=(adr<<4)|0x1;
  tmp0=FPGA_rSPI (32,adr);
  return tmp0;
}

u32 SPI_BP_READ (u32 adr)
{
  u32 tmp0;
  adr=(adr<<4)|0x3;
  tmp0=FPGA_rSPI (32,adr);
  return tmp0;
}

u64 SPI_BP64_READ (u32 adr)
{
  u64 tmp0;
  adr=(adr<<4)|0x8;//0x8 - ����� �������� ������
  tmp0=FPGA_rSPI (64,adr);
  return tmp0;
}
//----------------------------------------------------------
void info ()
{

}

void AVARIYA_OTKL ()
{
  PWR_072 (255);
  START_BP=0;//	
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
   u[n++]=0x00;   //15
   u[n++]=0x00;  //16

   Transf2(u);
   Transf("������ ���: ");
   Transf(u);
   Transf("\r\n");
}

u32 crc_input=0u; 
u32 crc_comp=0u;
u8 Str[64];
u8 mas[300];

u32 IO ( char* str)      // ������� ��������� ��������� ������
{
 unsigned int i=0;
	uint8_t z1=0;
  i = lenght;//������ �������� �����
  if (lenght==0) i = leng(str);
  lenght = 0; 
  indexZ = 0;
  
  if ((time_uart>50u)||(SCH_LENGHT_PACKET>MAX_PL))
  {
	  //-------------------
		packet_flag=0; 
		//-------------------
		time_uart=0u;  //��������� �������� ���� ����
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
		FLAG_CW = 0u; //���� ������������ �����, ��������� ����� ����� ����������
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;
  }
  
  while (i>0u)   //���������� ��������� ������ � ������ ���������  
  {  

	if ((str[indexZ]==0x7e)&&(packet_flag==0))// ���������� ������ ������
	  {  
		//-------------------
		packet_flag=1; 
		//-------------------
		time_uart=0u;  //��������� �������� ���� ����
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
		FLAG_CW = 0u; //���� ������������ �����, ��������� ����� ����� ����������
		FLAG_DATA = 0u;
		SCH_LENGHT_PACKET=0;		
	  }

	 InOut[index1]=str[indexZ];
	 SCH_LENGHT_PACKET++;//������������ ������ ������
		 
	if (( InOut[index1]==';')&&(FLAG_DATA==0u)&&(packet_flag==1))  {packet_flag=0;packet_ok=1u;FLAG_CW=1u;}
    
	if (((InOut[index1]=='=')||(InOut[index1]==':'))&&(data_flag==0)) {data_flag=1u;FLAG_CW=1u;}

	if (( InOut[index1]=='.')&&(data_flag2==0)&&(FLAG_DATA==0))   {data_flag2=1u; FLAG_CW=1u;}
	
	if (( InOut[index1]=='$')&&(FLAG_lenght==0u)) {FLAG_lenght=2u;FLAG_CW=1u;}
    
	if ((index1>2u)&&(InOut[2]==' ')&&(FLAG_CW==0u)&&(FLAG_lenght<2u))  
            {
                             if   (data_flag!=1u) {Word[index_word]=InOut[index1];} // ��������� ��������� �����
                      
                             if  ((data_flag==1u)&&(data_flag2==0u))     DATA_Word[index_data_word]=InOut[index1];// ���������  ����� ������1
                             if  ((data_flag==1u)&&(data_flag2==1u))     DATA_Word2[index_data_word2]=InOut[index1]; // ���������  ����� ������2
                    
                             if  (data_flag!=1u)
                                     {if (index_word<buf_Word) index_word++;} 
                                   else 
                                            {
                                             if ((data_flag==1u)&&(data_flag2==0u)) if (index_data_word<buf_DATA_Word)  {index_data_word++;sch_lenght_data++;}
                                            
                                             if ((data_flag==1u)&&(data_flag2==1u)) if (index_data_word2<buf_DATA_Word) index_data_word2++;
                                            }
			}
	
		if ((FLAG_lenght==2u)&&(FLAG_CW==0u)) {lenght_data = (u8)(InOut[index1]);FLAG_lenght=1u;} //���������� ������ ������ ������ ����� ":"
	
		if ((sch_lenght_data<lenght_data)&&(FLAG_lenght==1u)) FLAG_DATA = 1u; else {FLAG_DATA = 0u;}
	 
		if (index1<BUFFER_SR)  index1++;
		if (indexZ <BUFFER_SR)  indexZ ++;
		i--;
		FLAG_CW=0u;	
  } 

if (packet_ok==1u) 
  {    
      if (InOut[0]==0x7e)   crc_ok=crc_ok|0x1;   // �������� ������� ������� ������ - ������ ������
      if (InOut[1]==Adress) crc_ok=crc_ok|0x2;   // �������� ������� ������� ������ - ������� ����������
	  if (InOut[1]==Adress_REZ) crc_ok=crc_ok|0x2;   // �������� ������� ������� ������ - ������� ����������
 
if (crc_ok==0x3)  //��������� ������ ��������� ������� �������� ������ 
{
  if (strcmp(Word,"versiya")==0) //
   {
    Transf ("������ versiya\r\n"); 
    REQ_VERSIYA();       
   } else
if (strcmp(Word,"setup_IP0")==0) // �������� IP ����� �� ���� 485 � 072
   {

	 crc_comp =atoi(DATA_Word);	//������ ����� - ������ ������ �� ���������
 	 crc_input=atoi(DATA_Word2);//������ ����� - IP ������
	 
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

	 u_out("������ setup_IP0:",crc_input);
	 Transf2(Str);
   } else
if (strcmp(Word,"rst_072")==0) // 
   {
	 crc_comp =atoi(DATA_Word);	
	 u_out("������ rst_072:",crc_comp);
	 //RESET_072(crc_comp);
   } else   
if (strcmp(Word,"rs485_test_OK")==0) // ��������� ���� 485! ������ ������ �����.
   {
	 u_out("������ rs485_test_OK:",crc_comp);
	 Transf("���� 485 ���� ������� �������!\r\n");
	 FLAG_ASQ_TEST_485=1;
   } else	
if (strcmp(Word,"rs485_test")==0) // ��������� ���� 485! ������ ������ �����.
   {
	 u_out("������ rs485_test:",crc_comp);	 
     BUS_485_TEST (crc_comp);
   } else
 if (strcmp(Word,"rs485_help")==0) // ~0 spi_write:adr.code;
   {
	 u_out("������ rs485_help:",crc_comp);
	 Transf2("~0 help;");
   } else		
 if (strcmp(Word,"upr")==0) // ~0 spi_write:adr.code;
   {
	 crc_comp =atoi(DATA_Word);	//������ ����� - ������ ������ �� ���������
     UPR_HDS_MK (crc_comp);
	 u_out("������ upr:",crc_comp);
   } else	
 if (strcmp(Word,"spi_write")==0) // ~0 spi_write:adr.code;
   {
	 crc_comp =atoi(DATA_Word);	//������ ����� - ������ ������ �� ���������
 	 crc_input=atoi(DATA_Word2);//������������ 32-������ ��� 
	 crc_comp=(crc_comp<<4)|0x2;
	 FPGA_wSPI (32,crc_comp,crc_input);
	 Transf("������ FPGA spi_write\r\n");
   } else
 if (strcmp(Word,"spi_read")==0) //
   {	
    crc_comp=atoi(DATA_Word);//��� ������ ������� �� ���������
	  crc_comp=(crc_comp<<4)|0x3;
	  crc_comp=FPGA_rSPI (32,crc_comp);
	  x_out("CODE FPGA:",crc_comp);
   } else	
 if (strcmp(Word,"spi_rd_tst")==0) //
   {	
    crc_comp=atoi(DATA_Word);//��� ������ ������� �� ���������
	  crc_comp=(crc_comp<<4)|0x1;
	  crc_comp=FPGA_rSPI (32,crc_comp);
	  x_out("CODE FPGA:",crc_comp);
   } else
if (strcmp(Word,"JTAG_TST")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("������ JTAG_TST\r"    );
     Transf("\r"); 
	   TST ();
   } else	
if (strcmp(Word,"JTAG2_SCAN")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("������ JTAG_SCAN\r"    );
     Transf("\r"); 
	   crc_comp=jtag_scan(NULL,crc_comp); 
   } else
if (strcmp(Word,"JTAG1_SCAN")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("������ JTAG_SCAN\r"    );
     Transf("\r"); 
     JTAG_SCAN();
   } else
if (strcmp(Word,"JTAG_ID")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     Transf ("������ JTAG_ID\r"    );
     Transf("\r");  
     ID_recive (crc_comp);
   } else		 
if (strcmp(Word,"BatPG")==0) //
   {
      u_out ("������ BatPG:",0); 
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
      u_out ("������ mem:",crc_comp); 
	     PRINT_SERV();
   } else
if (strcmp(Word,"MSG")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ MSG:",crc_comp); 
	  MSG_SHOW ();
   } else
 if (strcmp(Word,"ID_SERV")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ ID_SERV:",crc_comp); 
	  STATUS_ID (&ID_SERV1);
   } else
	 if (strcmp(Word,"spi5")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ spi5:",crc_comp); 
	  CS_5_MK(0);
	  spi5send8(crc_comp);
	  CS_5_MK(1);
   } else	
	 if (strcmp(Word,"nss")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ nss:",crc_comp); 
	  NSS_4(crc_comp);
   } else
	 if (strcmp(Word,"eth_init")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ eth_init:",crc_comp); 
	  Set_network();
   } else
	 if (strcmp(Word,"tca_rst")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ tca_rst:",crc_comp); 
       RESET_TCA6424A_MK(crc_comp); 
   } else
	   
   if (strcmp(Word,"tca_test")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ tca_test:",crc_comp); 
	TEST_LED=crc_comp;
   } else
 if (strcmp(Word,"tca_w")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ tca_w:",crc_comp); 
      TCA_WR(crc_comp); 
   } else
 if (strcmp(Word,"enable_lm")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ enable_lm:",crc_comp); 
      ENABLE_LM25056_MK(crc_comp); 
   } else	
 if (strcmp(Word,"lm")==0) //
   {
	  crc_input =atoi  (DATA_Word); 
      u_out ("������ lm:",crc_input); 
      LM_MFR_MODEL(crc_input); //���� ���� ����� i2c - ����� �������� ������!!!
   } else
if (strcmp(Word,"sys_info")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ sys_info:",crc_comp); 
      SYS_INFO(crc_comp);
   } else
if (strcmp(Word,"lm_ID")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ lm_ID:",crc_comp); 
      LM_MFR_ID(crc_comp);
	  x_out("0:",LM_ID_CN[0]);
	  x_out("1:",LM_ID_CN[1]);
	  x_out("2:",LM_ID_CN[2]);
   } else
 if (strcmp(Word,"lm_temp")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ lm_temp:",crc_comp); 
	  crc_comp=LM_TEMP(crc_input);
      u_out("temp:",crc_comp);
   } else	
 if (strcmp(Word,"lm_v")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ lm_v:",crc_comp);       
	  crc_comp=LM_v(crc_input);;
      u_out("v:",crc_comp);
   } else	   
 if (strcmp(Word,"pwr_072")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ pwr_072:",crc_comp); 
      PWR_072(crc_comp);
   } else		
 if (strcmp(Word,"i2c_adr")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ i2c_adr:",crc_comp); 
      u_out("i2c:",HAL_I2C_IsDeviceReady(&hi2c1, crc_comp,1, 1000));
   } else	
 if (strcmp(Word,"start")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ start:",crc_comp); 
      START_BP=crc_comp;
	  PWR_ALL=0;
   } else
 if (strcmp(Word,"pwrdn")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ pwrdn:",crc_comp); 
      PWDN_4(crc_comp);
   } else
 if (strcmp(Word,"ADC_test")==0) //
   {
	  crc_comp =atoi  (DATA_Word); 
      u_out ("������ ADC_test:",crc_comp); 
      ADC_test ();
   } else
if (strcmp(Word,"help")==0)                     
   {
     Transf ("������ help\r\n"    );
     Menu1(0);
   } else
//-------------------i2c----------------	   
if (strcmp(Word,"i2c_status")==0)                     
   {
     Transf ("������ i2c_status\r\n"    );
	 crc_comp =atoi(DATA_Word);	
     if (HAL_I2C_IsDeviceReady(&hi2c1,crc_comp,1,10) ==HAL_OK) Transf("i2c_OK\r\n");else Transf("i2c_error\r\n");	 
   } else
if (strcmp(Word,"i2c_transmit")==0)                     
   {
     Transf ("������ i2c_transmit\r\n"    );
	 crc_comp =atoi(DATA_Word);	
 	 crc_input=atoi(DATA_Word2);
	 z1=crc_input;
     HAL_I2C_Master_Transmit(&hi2c1,crc_comp,&z1,1,10);	 
   } else
if (strcmp(Word,"i2c_recive")==0)                     
   {
     Transf ("������ i2c_recive\r\n"    );
	   crc_comp =atoi(DATA_Word);	
     HAL_I2C_Master_Receive(&hi2c1,crc_comp,&z1,1,10);
	 x_out("data=",z1);	 
   } else
if (strcmp(Word,"info")==0)                     
   {
     Transf ("������ info\r\n"    ); 
     info();
   } else
if (strcmp(Word,"led")==0)                     
   {
	 crc_comp =atoi(DATA_Word);
     u_out ("������ led:",crc_comp);
	 TCA_WR(crc_comp);
   }else  
if (strcmp(Word,"menu")==0)                     
   {
     Transf ("������ menu\r\n"    );
     Menu1(0);
   } else
if (strcmp(Word,"REQ_COL")==0)    //������ ���������� ������ 072 � �� �������                 
   {
     Transf ("������ REQ_COL\r\n"    );
     req_col();
   } else
if (strcmp(Word,"ANS")==0) //������ ����� �� ��������� (485) �� ����� ������� ������                     
   {     
     crc_comp =atoi(DATA_Word); 
     u_out ("������ ANS:",crc_comp);
     answer_translated (crc_comp);     
   } else   
 if (strcmp(Word,"Show_worktime")==0) //
   {
    u_out ("������ Show_worktime:",TIME_OF_WORK); 
   } else
  if (strcmp(Word,"MSG_ADR")==0) //������ ��������� � �������� �� ���������, �����-�� ������ ��������������� � ������ ���������������
   {
    crc_comp =atoi(DATA_Word); 
    u_out ("������ MSG_ADR:",crc_comp); 
	FUNC_FLAG_UP (&POINTER_ADR_COLLECT,50);//���� ������� � ��������� �� ���������
	if (FLAG_REJ_OSNPAR==1)	
	{
		ERROR_FLAG.ALERT=1;//��������� ���� ������ �� ����� ������ ��������� �������� ����������
		ERROR_FLAG.CODE=ERROR_CODE_REJ;
		ERROR_FLAG.INDEX=crc_comp;
		FLAG_REJ_OSNPAR=0;//��������� ����� ��������� �������� ����������
	}

   } else
      if (strcmp(Word,"EPCS1_DEV_ID")==0) //only for EPCS128 !!!
        {
          Transf ("\r\n������ EPCS1_DEV_ID:\r\n");
          spi_EPCS1_rd(READ_DEV_ID,mas,3);
          Transf ("\r\n");
          if (mas[2]==0x18) Transf("recive: EPCS128\r\n");
          x_out("mas[2]:",mas[2]);
        } else       
      if (strcmp(Word,"EPCS2_DEV_ID")==0) //only for EPCS128 !!!
        {
          Transf ("\r\n������ EPCS2_DEV_ID:\r\n");
          spi_EPCS2_rd(READ_DEV_ID,mas,3);
          Transf ("\r\n");
          if (mas[2]==0x18) Transf("recive: EPCS128\r\n");
          x_out("mas[2]:",mas[2]);
        } else
      if (strcmp(Word,"EPCS1_ID")==0) //
        {
          Transf ("\r\n������ EPCS_ID:\r\n");
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
          Transf ("\r\n������ EPCS1_STATUS:\r\n");
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
          Transf ("\r\n������ EPCS2_STATUS:\r\n");
          spi_EPCS2_rd(READ_STATUS,mas,4);
          Transf ("\r\n");
          x_out("mas[0]:",mas[0]);
          x_out("mas[1]:",mas[1]);
          x_out("mas[2]:",mas[2]);
          x_out("mas[3]:",mas[3]);
          }  else
        if (strcmp(Word,"EPCS1_READ")==0) //
          {
            crc_comp =atoi(DATA_Word); //����� ������
            crc_input=atoi(DATA_Word2);//���������� ����
            x_out ("\r\n������ EPCS1_READ:",crc_comp);//crc_comp - ��� 24-� ������ ����� ������            
            spi_EPCS1_read(READ_BYTES,crc_comp,mas,crc_input);//������ 256 ���� ������
            Transf("\r\n---------------------------\r\n");
            TABL_CONS (mas,crc_input);
          }
            else
        if (strcmp(Word,"EPCS2_READ")==0) //
          {
            crc_comp =atoi(DATA_Word); //����� ������
            crc_input=atoi(DATA_Word2);//���������� ����
            x_out ("\r\n������ EPCS2_READ:",crc_comp);//crc_comp - ��� 24-� ������ ����� ������            
            spi_EPCS2_read(READ_BYTES,crc_comp,mas,crc_input);//������ 256 ���� ������
            Transf("\r\n---------------------------\r\n");
            TABL_CONS (mas,crc_input);
          }
    else
    if (strcmp(Word,"EPCS1_WRITE_TEST")==0) //
      {
        crc_comp =atoi(DATA_Word);
        crc_input=atoi(DATA_Word2);
        x_out ("\r\n������ EPCS1_WRITE_TEST:",crc_comp);//crc_comp - ��� 24-� ������ ����� ������
        Transf("\r\n---------------------------\r\n");        
        for (i=0;i<256;i++) mas[i]=crc_input;          
        EPCS1_WRITE_BUF(crc_comp,mas,256);
        }else
    if (strcmp(Word,"EPCS2_WRITE_TEST")==0) //
      {
        crc_comp =atoi(DATA_Word);
        crc_input=atoi(DATA_Word2);
        x_out ("\r\n������ EPCS2_WRITE_TEST:",crc_comp);//crc_comp - ��� 24-� ������ ����� ������
        Transf("\r\n---------------------------\r\n");        
        for (i=0;i<256;i++) mas[i]=crc_input;          
        EPCS2_WRITE_BUF(crc_comp,mas,256);
        }else
    if (strcmp(Word,"EPCS1_ERASE_SECTOR")==0) //
      {
        crc_comp =atoi(DATA_Word);
        x_out ("\r\n������ EPCS1_ERASE_SECTOR:",crc_comp);//crc_comp - ��� 24-� ������ ����� ������
        EPCS1_ERASE_SECTOR(crc_comp);
        }else 
    if (strcmp(Word,"EPCS2_ERASE_SECTOR")==0) //
      {
        crc_comp =atoi(DATA_Word);
        x_out ("\r\n������ EPCS2_ERASE_SECTOR:",crc_comp);//crc_comp - ��� 24-� ������ ����� ������
        EPCS2_ERASE_SECTOR(crc_comp);
        }else 
    if (strcmp(Word,"EPCS1_ERASE_ALL")==0) //
      {
        crc_comp =atoi(DATA_Word);
        x_out ("\r\n������ EPCS_ERASE_ALL:",crc_comp);//crc_comp - ��� 24-� ������ ����� ������
        EPCS1_ERASE_ALL();
        } else 
    if (strcmp(Word,"serial_w")==0) //
        {
          crc_comp =atoi(DATA_Word);
          u_out ("������ serial_w:",crc_comp);  
          SERIAL_NUMBER_WR (crc_comp);                  
        } else
    if (strcmp(Word,"corr")==0) //
        {
          crc_comp =atoi(DATA_Word);
          u_out ("������ corr:",crc_comp);  
          Console_corr();                  
        }  else
    if (strcmp(Word,"corr_I_write")==0) //
        {
          crc_comp =atoi(DATA_Word);
          u_out ("������ corr_I_write:",crc_comp);  
          CorrI_write ();                
        } 
 } 
      for (i=0u;i<(BUFFER_SR+1);i++)         str[i]=0x00;
	    for (i=0u;i<buf_Word     ;i++)        Word[i]=0x00;
      for (i=0u;i<buf_DATA_Word;i++)   DATA_Word[i]=0x00;
      for (i=0u;i<buf_DATA_Word;i++)  DATA_Word2[i]=0x00;  
      for (i=0u;i<BUFFER_SR    ;i++)       InOut[i]=0x00;  
      
	      time_uart=0;  //��������� �������� ���� ����
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
      FLAG_CW = 0u; //���� ������������ �����, ��������� ����� ����� ����������
      FLAG_DATA = 0u;	  
      	  
      DATA_Word [0]=' ';
      DATA_Word2[0]=' ';
	  SCH_LENGHT_PACKET=0;
  }

  if ((packet_ok==1)&&(crc_ok==0x1))     //��������� ������ ��������� ������� �������� �����

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
 
  for (i=0;i<20;i++) Transf("\r");    // ������� ���������
  for (i=0; i<20; i++) Transf ("-");  // ����� �����������
  Transf("\r");
  Transf("..........Terminal 330 �������.........\r\n");
  Transf("\r");
  Transf("MENU :\r");
  Transf("-------\r");
  Transf("����������� ��������� �������:\r");
  Transf("~ - ��������� ����\r");
  un_out("",Adress-0x30);
  Transf(" - ����� ��������\r");
  Transf(";- ����� ����� \r");
  Transf(".............. \r");
  Transf("---------------------------------------------\r\n");
  Transf("PORT:");
  un_out("",PORT_my);
  Transf("       - ����� ����� �����\r");
  Transf("IP  :");
  xun_out("",IP_my);
  Transf(" - IP �����    �����\r"); 
  Transf("~0 help; - ������� ����\r");
  Transf("~0 info; - ���������� \r");
  Transf("~0 rs485_test; - \r");
  Transf("~0 JTAG_TST;   - ������������ JTAG\r");
  Transf("~0 JTAG2_SCAN; - ������������ ���� JTAG\r");
  Transf("~0 JTAG1_SCAN; - ������������ ���� JTAG\r");
  Transf("~0 JTAG_ID; \r");
  Transf("~0 time;       - ����� ���������� �������\r");
  Transf("~0 sys_info; \r");
  Transf("~0 start:1; \r");
  Transf("~0 lmk_sync; - sync �� LMK\r");
  Transf("~0 init_lmk; - init �� LMK\r");
  Transf("-------------------------------------------\r");
  Transf("\r");
  Transf("\r");
  Transf("++++++++++++++++++++\r");
  Transf("\r");
  Transf("\r");
  //for (i=0; i<64; i++) zputs ("*",1);  // ����� �����������
  //for (i=0;i<10;i++) puts("\r",1);  // ������� ���������
  Transf("\r");
  REQ_VERSIYA(); //������� ������ ��������
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
   		rx_buffer1[rx_wr_index1++]= (uint8_t) (RX_uBUF[0]& 0xFF); //��������� ������ � �����, ������������� ����� ������
   		if ( rx_wr_index1 == RX_BUFFER_SIZE1) rx_wr_index1=0; //���� �� �����
   	 
	  	if (++rx_counter1 == RX_BUFFER_SIZE1) //������������ ������
      	{
        	rx_counter1=0; //�������� ������� (������� ��� ������)
        	rx_buffer_overflow1=1;  //�������� � ������������
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
	adc_ch[ 2] = ((float)adcBuffer[2])*3.3*5.71428/4096;//12 ����� �������� 47�-10�
	adc_ch[ 3] = ((float)adcBuffer[3])*3.3/4096*2;
	adc_ch[ 4] = ((float)adcBuffer[4])*3.3/4096;	
	
	Transf("\r\n---------\r\n");
	f_out("adc0_5.0V     :",adc_ch[0 ]);
	f_out("adc1_3.3V     :",adc_ch[1 ]);
	f_out("adc2_12V      :",adc_ch[2 ]);
	f_out("adc3_3.3V     :",adc_ch[3 ]);
	f_out("temp_sens(�)  :",ADC_Temp(adc_ch[4]));
	
	Transf("\r\r\r");
	u_out("adc0_5.0V     :",adcBuffer[0 ]);
	u_out("adc1_3.3V     :",adcBuffer[1 ]);
	u_out("adc2_12V      :",adcBuffer[2 ]);
	u_out("adc3_3.3V     :",adcBuffer[3 ]);
	u_out("temp_sens(�)  :",adcBuffer[4 ]);	
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
  
void BP_start (u16 a,u8 pwr)
{
	static u8 flag=0;
	if (a==0)
	{
	    B330.INIT=0;
		PWR_072    (255);//��������� ������ ������� � ���������� ��������� ���������� ��������� � ������� ������� - � ����
		UPR_HDS_MK (1);	
		TIMER_BP_PWM=1000;
		RESET_072(0); //������ �����
		if (flag==0) {Transf("��������� �������!\r\n");flag=1;}
	} else
	if ((TIMER_BP_PWM==0)&&(flag==1))
	{
		B330.INIT=1;
		flag=0;
		UPR_HDS_MK (0);			
		Transf("�������� �������!\r\n");
		PWR_072(pwr); //����� ������� �� ��� ������!!! - ��� ����� �� �������� i2c			
		ENABLE_LM25056_MK(1); //�������� ��� �/�� LM
        FUNC_FLAG_UP (&POINTER_RESET,3000);//������ ���������� ������ ��� ������ ������ �� ���������
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
	uint16_t DevAddress=0x22;//����� 
	 u8  a[4];
	 uint8_t state=0;
	 
	 HAL_I2C_Init(&hi2c1);
	
	a[0] =   0x8c;  //Configuration Port 0
	a[1] =   0x00;
	a[2] =   0x00;
	a[3] =   0x00;
	
	state=HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,4,1000);//������������� ����� �� ��� ������
	
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
	uint16_t DevAddress=0x53;//����� DD10 ds4520
	 u8  a[2];
	 uint8_t state=0;
	 uint8_t v=0;
	 u32 error=0;
	 int i=0; 
	 
	for (i=0;i<8;i++) B330.CH[i]=1;
	
	HAL_I2C_Init(&hi2c1);	

	if (z&(1<<0)) {v=v|(1<<6); B330.CH[7]=0;}
	if (z&(1<<1)) {v=v|(1<<4); B330.CH[6]=0;}
	if (z&(1<<2)) {v=v|(1<<5); B330.CH[5]=0;}
	if (z&(1<<3)) {v=v|(1<<3); B330.CH[4]=0;}
	if (z&(1<<4)) {v=v|(1<<2); B330.CH[3]=0;}
	if (z&(1<<5)) {v=v|(1<<1); B330.CH[2]=0;}
	if (z&(1<<6)) {v=v|(1<<0); B330.CH[1]=0;}
	if (z&(1<<7)) {v=v|(1<<7); B330.CH[0]=0;}
	
	a[0] =   0xf0;  //�������-������� ��������� �������� ��� 0-7 ������
	a[1] =   v;
	
	HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,2,1000);
		
	a[0] =   0xf2;  //�������-������� ������ 0-7 ������ (���� ���� ���� ������ ���������!!!)
	a[1] =      v;
	
//	u_out("z:",z);
//	state=      HAL_I2C_IsDeviceReady  (&hi2c1,(DevAddress<<1),1,  1000);
		error = HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,2,1000);
//	if (state)  HAL_I2C_Master_Transmit(&hi2c1,(DevAddress<<1),a,2,1000);

		PWR_CHANNEL=z;
	
	HAL_I2C_DeInit(&hi2c1);
	
	return state;
}

u8 DS4520_read (void)  //��������� ��������� ����� ����������� - 15 ���� ����������� DD10 (8����)  - ������ POWERGOOD �� �������������
{
	uint16_t DevAddress=0x53;//����� DD10 ds4520
	 u8  c[1]; 
	 u8  a[2];
	 uint8_t state=0;
	 uint8_t v=0;
	 u32 error=0;
	
	HAL_I2C_Init(&hi2c1);	

	c[0] =   0xf9;  //�������-������ I/O Status 1 
	
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
	 uint8_t a[8];
	 uint16_t Size2=8;
	 uint8_t state=1;
	 uint8_t c[2];
	 
	 c[0]=0x9a;//To read the manufacturer ID �������-������ manufacturer ID in ASCII characters (NSC).
	 
	HAL_I2C_Init(&hi2c1); 
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-�� ������
	if (z==7)  DevAddress=0x56;//LM25056 - 2-�� ������
	if (z==6)  DevAddress=0x51;//LM25056 - 3-�� ������
	if (z==5)  DevAddress=0x57;//LM25056 - 4-�� ������
	if (z==4)  DevAddress=0x59;//LM25056 - 5-�� ������
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-�� ������
	if (z==2)  DevAddress=0x15;//LM25056 - 7-�� ������
	if (z==1)  DevAddress=0x16;//LM25056 - 8-�� ������
	
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
	int i=0;
	Transf("\r\n");
	if (a>0)
	{
		for (i=0;i<8;i++) {un_out("LM[",i); u_out("].TEMP:",LM[i].TEMP);}
		Transf("\r\n");
	}
	if (a>1)
	{
		for (i=0;i<8;i++) {un_out("LM[",i); u_out("].P:",LM[i].P);}
		Transf("\r\n");
	}
	if (a>2)
	{
		for (i=0;i<8;i++) {un_out("LM[",i); u_out("].I:",LM[i].I);}
			Transf("\r\n");
	}
	if (a>3)
	{
		for (i=0;i<8;i++) {un_out("LM[",i); u_out("].U:",LM[i].U);}
			Transf("\r\n");
	}
}

u8 LM_MFR_ID (u8 z)
{
	 uint16_t DevAddress=0x00;//
	 uint8_t a[4];
	 uint16_t Size2=3;
	 uint8_t state=1;
	 uint8_t v=0;
	 uint8_t c[2];
	 u32 error=0;
	 
	 HAL_I2C_Init(&hi2c1);
	 
	 c[0]=0x99;//To read the manufacturer ID �������-������ manufacturer ID in ASCII characters (NSC).
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-�� ������
	if (z==7)  DevAddress=0x56;//LM25056 - 2-�� ������
	if (z==6)  DevAddress=0x51;//LM25056 - 3-�� ������
	if (z==5)  DevAddress=0x57;//LM25056 - 4-�� ������
	if (z==4)  DevAddress=0x59;//LM25056 - 5-�� ������
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-�� ������
	if (z==2)  DevAddress=0x15;//LM25056 - 7-�� ������
	if (z==1)  DevAddress=0x16;//LM25056 - 8-�� ������
	
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
    LM[0].ID[0]=a[0];
    LM[0].ID[1]=a[1];
    LM[0].ID[2]=a[2];
  }
  if (z==2) 
  {
    LM[1].ID[0]=a[0];
    LM[1].ID[1]=a[1];
    LM[1].ID[2]=a[2];
  }
  if (z==3) 
  {
    LM[2].ID[0]=a[0];
    LM[2].ID[1]=a[1];
    LM[2].ID[2]=a[2];
  }
  if (z==4) 
  {
    LM[3].ID[0]=a[0];
    LM[3].ID[1]=a[1];
    LM[3].ID[2]=a[2];
  }
  if (z==5) 
  {
    LM[4].ID[0]=a[0];
    LM[4].ID[1]=a[1];
    LM[4].ID[2]=a[2];
  }
  if (z==6) 
  {
    LM[5].ID[0]=a[0];
    LM[5].ID[1]=a[1];
    LM[5].ID[2]=a[2];
  }
  if (z==7) 
  {
    LM[6].ID[0]=a[0];
    LM[6].ID[1]=a[1];
    LM[6].ID[2]=a[2];
  }
  if (z==8) 
  {
    LM[7].ID[0]=a[0];
    LM[7].ID[1]=a[1];
    LM[7].ID[2]=a[2];
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
	 uint8_t  a[8];
	 uint8_t  c[1];
	 uint16_t Size=2;//������ ������������ �������
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
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-�� ������
	if (z==7)  DevAddress=0x56;//LM25056 - 2-�� ������
	if (z==6)  DevAddress=0x51;//LM25056 - 3-�� ������
	if (z==5)  DevAddress=0x57;//LM25056 - 4-�� ������
	if (z==4)  DevAddress=0x59;//LM25056 - 5-�� ������
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-�� ������
	if (z==2)  DevAddress=0x15;//LM25056 - 7-�� ������
	if (z==1)  DevAddress=0x16;//LM25056 - 8-�� ������
	
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
	 uint8_t  a[8];
	 uint8_t  c[1];
	 uint16_t Size=2;//������ ������������ �������
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
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-�� ������
	if (z==7)  DevAddress=0x56;//LM25056 - 2-�� ������
	if (z==6)  DevAddress=0x51;//LM25056 - 3-�� ������
	if (z==5)  DevAddress=0x57;//LM25056 - 4-�� ������
	if (z==4)  DevAddress=0x59;//LM25056 - 5-�� ������
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-�� ������
	if (z==2)  DevAddress=0x15;//LM25056 - 7-�� ������
	if (z==1)  DevAddress=0x16;//LM25056 - 8-�� ������
	
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
	 uint8_t  a[8];
	 uint8_t  c[1];
	 uint16_t Size=2;//������ ������������ �������
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
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-�� ������
	if (z==7)  DevAddress=0x56;//LM25056 - 2-�� ������
	if (z==6)  DevAddress=0x51;//LM25056 - 3-�� ������
	if (z==5)  DevAddress=0x57;//LM25056 - 4-�� ������
	if (z==4)  DevAddress=0x59;//LM25056 - 5-�� ������
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-�� ������
	if (z==2)  DevAddress=0x15;//LM25056 - 7-�� ������
	if (z==1)  DevAddress=0x16;//LM25056 - 8-�� ������
	
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
	 uint8_t  a[8];
	 uint8_t  c[1];
	 uint16_t Size=2;//������ ������������ �������
	 uint8_t state=1;
	 uint8_t v=0;
	 u32 error=0;
	 int value=0;
	 
	 float x=0;
	 float m=68985.849;//5 ���
	 float y=0;
	 float r=2;
	 float b=0;//-1833
	 int   p=0;
	 
	HAL_I2C_Init(&hi2c1);
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-�� ������
	if (z==7)  DevAddress=0x56;//LM25056 - 2-�� ������
	if (z==6)  DevAddress=0x51;//LM25056 - 3-�� ������
	if (z==5)  DevAddress=0x57;//LM25056 - 4-�� ������
	if (z==4)  DevAddress=0x59;//LM25056 - 5-�� ������
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-�� ������
	if (z==2)  DevAddress=0x15;//LM25056 - 7-�� ������
	if (z==1)  DevAddress=0x16;//LM25056 - 8-�� ������
	
	c[0]=0xd1;//Retrieves  input current measurement

	error=HAL_I2C_Master_SMBA_block_recieve(&hi2c1,(DevAddress<<1),c,1,a,Size,1);
	
	p=(a[1]<<8)+a[0];
	y=p;

//	x=(1/m)*((y*(powf(10,r)))-b);
	x=(1/m)*((y*(powf(10,r)))-b)/1.1;//� ����������� �������������
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
	 uint8_t  a[8];
	 uint8_t  c[1];
	 uint16_t Size=2;//������ ������������ �������
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
	 
	if (z==8)  DevAddress=0x50;//LM25056 - 1-�� ������
	if (z==7)  DevAddress=0x56;//LM25056 - 2-�� ������
	if (z==6)  DevAddress=0x51;//LM25056 - 3-�� ������
	if (z==5)  DevAddress=0x57;//LM25056 - 4-�� ������
	if (z==4)  DevAddress=0x59;//LM25056 - 5-�� ������
	if (z==3)  DevAddress=0x5a;//LM25056 - 6-�� ������
	if (z==2)  DevAddress=0x15;//LM25056 - 7-�� ������
	if (z==1)  DevAddress=0x16;//LM25056 - 8-�� ������
	
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
u32 a,//������� ������ ������� � ������� ���������
u32 k //�������� ������ � ������ �� �� ���������� ���������
) //������� ����������� ������ ������ � ������� "���������" 
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

//������� �������� ...
void MSG_SEND_UDP (ID_SERVER *id,SERVER *srv,u32 msg_type)
{
  u32 i=0;
  
  u64 ADR=ADRES_SENDER_CMD;
  u32 data=0;
//u8 D[4];
        
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
 }else
 if (msg_type==MSG_REQ_TEST_RESET)
 {
    ARRAY_DATA(FLAG_ASQ_TEST_RESET);
 }
       

  SYS_CMD_MSG(
        id,//������
        &INVOICE[ADR],  //��������� ��������� 
        0,        //������ � �������
        msg_type, //��� ���������
        4,        //����� ������ ��������� � ������
        D_TEMP,   //������ ��������� - ������ ������
        TIME_SYS  //����� ����������� ���������
        );
}

int ARR_Z (void)
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
  DATA_TR[n++]=B330.STATUS_OK; 		//36
  return n;
}


u16 ARR_B072 (void)
{
  u16 n=0;
  u32 tmp0=0;
  u16 j=0;
  
      DATA_TR[n++]=NUMBER_OF_B072;//�������� � ���������� ������������ ����� �072 � ��
for (j=0;j<8;j++)
	{
	  DATA_TR[n++]=B072[j].TEMP>>8;	
	  DATA_TR[n++]=B072[j].TEMP>>0;	
	  DATA_TR[n++]=B072[j].REF;
	  DATA_TR[n++]=B072[j].SYNC_1HZ;
	  DATA_TR[n++]=B072[j].ADC_0;
	  DATA_TR[n++]=B072[j].ADC_1;
	  DATA_TR[n++]=B072[j].DAC_0;
	  DATA_TR[n++]=B072[j].DAC_1;
	  DATA_TR[n++]=B072[j].SYNC0;
	  DATA_TR[n++]=B072[j].SYNC1;
	  DATA_TR[n++]=B072[j].ERROR_1HZ>>16;
	  DATA_TR[n++]=B072[j].ERROR_1HZ>>8;
	  DATA_TR[n++]=B072[j].ERROR_1HZ;
	}		
   
return n;
}

u16 ARR_B330 (void)
{
  u16 n=0;
  u32 tmp0=0;
  u16 j=0;
  u16 i=0;
  
  for (i=0;i<8;i++)
  {
	  DATA_TR[n++]=LM[i].TEMP>>24;	
	  DATA_TR[n++]=LM[i].TEMP>>16;
      DATA_TR[n++]=LM[i].TEMP>>8;	
	  DATA_TR[n++]=LM[i].TEMP>>0;
	  
	  DATA_TR[n++]=LM[i].P>>24;	
	  DATA_TR[n++]=LM[i].P>>16;
      DATA_TR[n++]=LM[i].P>>8;	
	  DATA_TR[n++]=LM[i].P>>0;
	  
	  DATA_TR[n++]=LM[i].I>>24;	
	  DATA_TR[n++]=LM[i].I>>16;
      DATA_TR[n++]=LM[i].I>>8;	
	  DATA_TR[n++]=LM[i].I>>0;
	  
	  DATA_TR[n++]=LM[i].U>>24;	
	  DATA_TR[n++]=LM[i].U>>16;
      DATA_TR[n++]=LM[i].U>>8;	
	  DATA_TR[n++]=LM[i].U>>0;
	  
	  DATA_TR[n++]=LM[i].P_max>>24;	
	  DATA_TR[n++]=LM[i].P_max>>16;
      DATA_TR[n++]=LM[i].P_max>>8;	
	  DATA_TR[n++]=LM[i].P_max>>0;
	  
	  DATA_TR[n++]=LM[i].TEMP_max>>24;	
	  DATA_TR[n++]=LM[i].TEMP_max>>16;
      DATA_TR[n++]=LM[i].TEMP_max>>8;	
	  DATA_TR[n++]=LM[i].TEMP_max>>0;
	  
	  DATA_TR[n++]=LM[i].U_min>>24;	
	  DATA_TR[n++]=LM[i].U_min>>16;
      DATA_TR[n++]=LM[i].U_min>>8;	
	  DATA_TR[n++]=LM[i].U_min>>0; 
	  
  }	
   
return n;
}

void SYS_INFO_SEND_UDP (ID_SERVER *id,SERVER *srv)
{
	u32 i=0;
	u16 n=0;	
	u64 ADR=ADRES_SENDER_CMD;
	u32 data=0;
	u16 Caunt=0;
	u8 D[4];

	if (START_BP==1)
			{
			
			
						Caunt=ARR_B330 ();//��������� ������ ������
						
						SYS_CMD_MSG(
						id,//������
						&INVOICE[ADR], //��������� ���������	
						0,	 		   //������ � �������
						MSG_STATE_B330,//��� ���������
						Caunt,		   //����� ������ ��������� � ������
						DATA_TR,       //������ ��������� - ������ ������
						TIME_SYS  	   //����� ����������� ���������
						);
					
						//-----------------------------------
						//��������� � ��������� ������� �������
						D_TEMP[0]=0;
						D_TEMP[1]=0;
						D_TEMP[2]=START_BP;
						D_TEMP[3]=PWR_CHANNEL;
						//u_out("PWR:",PWR_CHANNEL);
						SYS_CMD_MSG(
						id,//������
						&INVOICE[ADR], 	//��������� ���������	
						0,	 			//������ � �������
						MSG_PWR_CHANNEL,//��� ���������
						4,		 		//����� ������ ��������� � ������
						D_TEMP,    		//������ ��������� - ������ ������
						TIME_SYS	  	//����� ����������� ���������
						);
			//----------------------------------
			//��������� �� ����� ��������� ������� �330
				n=ARR_Z ();//��������� ������������ ������

						SYS_CMD_MSG(
						id,//������
						&INVOICE[ADR], 	//��������� ���������	
						0,	 			//������ � �������
						MSG_STATUS_OK,  //��� ���������
						n,		    //����� ������ ��������� � ������
						DATA_TR,  		//������ ��������� - ������ ������
						TIME_SYS	  	//����� ����������� ���������
						);
			
			
			}

}

void STATE_B072_INFO_SEND_UDP (ID_SERVER *id,SERVER *srv)
{
	u32 i=0;
	u16 n=0; 
    u64 ADR=ADRES_SENDER_CMD;
//---------------------------------
//��������� � ��������� ����� B072
	n=ARR_B072();//��������� ������������ ������
	
	SYS_CMD_MSG(
			id,//������
			&INVOICE[ADR], 	//��������� ���������	
			i,	 			//������ � �������
			MSG_STATE_B072,  //��� ���������
			n,		        //����� ������ ��������� � ������
			DATA_TR,  		//������ ��������� - ������ ������
			TIME_SYS	  	//����� ����������� ���������
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
	u32 tmp0=0;
	u32 tmp1=0;
	u8  D[4];
	int adr_BPL=0;

	for (i=0;i<SIZE_ID;i++)
	{		
		if (id->CMD_TYPE[i]==CMD_TIME_SETUP) 
		{
			Transf("\r\n------\r\n");
			Transf("�������:��������� �������!\r\n");
			FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
			idx4=idx_srv(id->INDEX[i],4);//������ ������������ ������ � "���������"
			idx5=idx_srv(id->INDEX[i],5);//������ ������������ ������ � "���������"
			idx6=idx_srv(id->INDEX[i],6);//������ ������������ ������ � "���������"
			idx7=idx_srv(id->INDEX[i],7);//������ ������������ ������ � "���������"
			
			TIME_SYS=((u64)srv->MeM[idx0]<<56)|((u64)srv->MeM[idx1]<<48)|
					 ((u64)srv->MeM[idx2]<<40)|((u64)srv->MeM[idx3]<<32)|
					 ((u64)srv->MeM[idx4]<<24)|((u64)srv->MeM[idx5]<<16)|
					 ((u64)srv->MeM[idx6]<< 8)|((u64)srv->MeM[idx7]<< 0);
			TIME_cons ();
			Transf("\r\n------\r\n");					
		} else		
		if (id->CMD_TYPE[i]==CMD_STATUS)//������� ������� ��������
		{
			//��� ������ � �������
		//	Transf("�������:CMD_STATUS!\r\n");
            FLAG_CMD=1;
			ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����
			ADRES_SENDER_CMD=ADR;
		//	SYS_INFO_SEND_UDP(&ID_SERV1,&SERV1);
			FLAG_REQ_STATUS=1;
		}	else		
		if (id->CMD_TYPE[i]==CMD_LED)//������� ���������� ������������ �� ������� ������
		{
            FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			TCA_WR(data);//��������� �������
		} else  
		if (id->CMD_TYPE[i]==CMD_LED_TEST)//������� ���������� ������������ �� ������� ������
		{
            FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			FUNC_FLAG_UP (&POINTER_LED_TEST0,10);
		} else  			
		if (id->CMD_TYPE[i]==CMD_NUMB_BLOCK_WR)//�������
		{
            FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			
			 SERIAL_NUMBER_WR (data);//��������� �������
			 B330.B330_NUMBER=SERIAL_NUMBER ();
		} else			
		if (id->CMD_TYPE[i]==CMD_12V)//������� ��������� ��������� +12V
		{
            FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
			
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			tmp0= (srv->MeM[idx2]);
			START_BP=data;//��������� �������	
		    PWR_ALL=tmp0; //����� ������ �������� �����
			FLAG_ADRES_SENDER_CMD=1;//��������� ���� ���� ��� � ��� ���� ���� ���������� ���������
		} else		
		if (id->CMD_TYPE[i]==CMD_CH_UP)//������� ��������� ������ �������, ��������� ������ ������ �������!!!
		{
            FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
		//--------------������ � ������������ � ���������� �� �� �� ���!!!!-----------------
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
		//----------------------------------------------------------------------------------	
			data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
			x_out("CH:",data);
			PWR_072 (data);//��������� �������
		} else
		if (id->CMD_TYPE[i]==CMD_SETUP_IP0)//������� ��������� IP0 ������ ����������� ������� 072 
		{
          FLAG_CMD=1;
			idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������" 
			idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������" //� ������� ������ ��������� ������� ����� �����!!!
			idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
			idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
			idx4=idx_srv(id->INDEX[i],4);//������ ������������ ������ � "���������"
		//----------------------------------------------------------------------------------	
			data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
            adr_BPL=srv->MeM[idx0];//����� 072 �� ���������
			SETUP_IP0_072 (adr_BPL,data);//��������� �������
			Transf("������ IP0\r\n");
			x_out("���������� IP �����:",data);
			u_out("� ���� 072 �",adr_BPL);			
		} else
      if (id->CMD_TYPE[i]==CMD_SETUP_IP1)//������� ��������� IP1 ������ ����������� ������� 072 
    {
      FLAG_CMD=1;
      idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������" 
      idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������" //� ������� ������ ��������� ������� ����� �����!!!
      idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
      idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
      idx4=idx_srv(id->INDEX[i],4);//������ ������������ ������ � "���������"
    //----------------------------------------------------------------------------------  
      data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      adr_BPL=srv->MeM[idx0];//����� 072 �� ���������
      SETUP_IP1_072 (adr_BPL,data);//��������� �������
      Transf("������ IP1\r\n");
      x_out("���������� IP �����:",data);
      u_out("� ���� 072 �",adr_BPL);      
    } else 
	if (id->CMD_TYPE[i]==CMD_SETUP_DEST_IP0)//������� ��������� IP0 ������ ����������� ������� 072 
    {
      FLAG_CMD=1;
      idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������" 
      idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������" //� ������� ������ ��������� ������� ����� �����!!!
      idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
      idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
      idx4=idx_srv(id->INDEX[i],4);//������ ������������ ������ � "���������"
    //----------------------------------------------------------------------------------  
      data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      adr_BPL=srv->MeM[idx0];//����� 072 �� ���������
      SETUP_DEST_IP0_072 (adr_BPL,data);//��������� �������
      Transf("������ dest_IP0\r\n");
      x_out("���������� IP �����:",data);
      u_out("� ���� 072 �",adr_BPL);      
    } else
	if (id->CMD_TYPE[i]==CMD_SETUP_DEST_IP1)//������� ��������� IP1 ������ ����������� ������� 072 
    {
      FLAG_CMD=1;
      idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������" 
      idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������" //� ������� ������ ��������� ������� ����� �����!!!
      idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
      idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
      idx4=idx_srv(id->INDEX[i],4);//������ ������������ ������ � "���������"
    //----------------------------------------------------------------------------------  
      data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      adr_BPL=srv->MeM[idx0];//����� 072 �� ���������
      SETUP_DEST_IP1_072 (adr_BPL,data);//��������� �������
      Transf("������ dest_IP1\r\n");
      x_out("���������� IP �����:",data);
      u_out("� ���� 072 �",adr_BPL);      
    } else 
    if (id->CMD_TYPE[i]==CMD_REQ_NUM_SLAVE)//������� ������� � ���������� ������ 072 � �� ������� 
    {
      FLAG_CMD=1;
      Transf("����������� ������� 072 � �� �� ������� �� �������!\r\n");
  //    req_col();
      FUNC_FLAG_UP (&POINTER_ADR_COLLECT,1000);//������ ���������� ������ ��� ������ ������ �� ���������
    } else		 
    if (id->CMD_TYPE[i]==CMD_REJ_OSNPAR)//������� �������� � ����� �������� �������� ����������
    {
        FLAG_CMD=1; 
	    idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
    	idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
		idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
		idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
      data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
	  FLAG_REJ_OSNPAR=data;
      if (FLAG_REJ_OSNPAR==1) Transf("��������� ������ �������� ���������!\r\n");
	  else Transf("���������� ������ �������� ���������!\r\n");
      
    } else
       if (id->CMD_TYPE[i]==CMD_TEST_485)//������� ������ ����� �������� ���� 485
    {
      FLAG_CMD=1;
      Transf("�������� ���� �������� ���� 485!\r\n");
      FLAG_ASQ_TEST_485=0;//���������� ���� ������ �� ���� 485
	  SCH_TST1=0;
      FUNC_FLAG_UP (&POINTER_TEST_485,10);//������ ���������� ������ ��� ������ ������ �� ���������, ������ ������� 0 � ��������!!!
    }else
       if (id->CMD_TYPE[i]==CMD_TEST_SPI)//������� ������ ����� �������� ���� SPI
    {
      FLAG_CMD=1;
      Transf("�������� ���� �������� ���� SPI!\r\n");
      FLAG_ASQ_TEST_SPI=0;//���������� ���� ������ �� ���� 485
      FUNC_FLAG_UP (&POINTER_TEST_SPI,10);//������ ���������� ������ ��� ������ ������ �� ���������, ������ ������� 0 � ��������!!!
    }else
       if (id->CMD_TYPE[i]==CMD_TEST_JTAG)//������� ������ ����� �������� ���� JTAG
    {
      FLAG_CMD=1;
      Transf("�������� ���� �������� ���� JTAG!\r\n");
      FLAG_ASQ_TEST_JTAG=0;//���������� ���� ������ �� ���� 485
      FUNC_FLAG_UP (&POINTER_TEST_JTAG,10);//������ ���������� ������ ��� ������ ������ �� ���������, ������ ������� 0 � ��������!!!
    }else
       if (id->CMD_TYPE[i]==CMD_Corr_I)//������� ������������� ������������ ��������� ����
    {
      FLAG_CMD=1;
      idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������" 
      idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������" //� ������� ������ ��������� ������� ����� �����!!!
      idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
      idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
      idx4=idx_srv(id->INDEX[i],4);//������ ������������ ������ � "���������"
    //----------------------------------------------------------------------------------  
      data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      index_ch=srv->MeM[idx0];//������ ������ ��� ����������
      u_out("������ ����������� ��������� ��������� ����:",data);
      u_out("� ������:",index_ch);
      TMP_f=data;      
      B330.Corr_I[index_ch]=TMP_f/1000;
      CorrI_write (); 
    }else
       if (id->CMD_TYPE[i]==CMD_Corr_U)//������� ������������� ������������ ��������� ����������
    {
      FLAG_CMD=1;
      idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������" 
      idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������" //� ������� ������ ��������� ������� ����� �����!!!
      idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
      idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
      idx4=idx_srv(id->INDEX[i],4);//������ ������������ ������ � "���������"
    //----------------------------------------------------------------------------------  
      data=((srv->MeM[idx1])<<24)|((srv->MeM[idx2])<<16)|((srv->MeM[idx3])<< 8)|((srv->MeM[idx4]));
      index_ch=srv->MeM[idx0];//������ ������ ��� ����������
      u_out("������ ����������� ��������� ��������� ����������:",data);
      u_out("� ������:",index_ch);
      TMP_f=data;      
      B330.Corr_U[index_ch]=TMP_f/1000;
      CorrU_write (); 
    }else 
       if (id->CMD_TYPE[i]==CMD_Corr_REQ)//������� ������� ������ �������������� �������������
    {
      FLAG_CMD=1;
      TIME_cons ();
      Transf("\r\n������ ������ �� ������ ������������� �������������.\r\n");
      Console_corr();      
      DAT_CORR_REQ_FORM();//��������� ������������ ������
      //���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����
      ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);
      SYS_CMD_MSG(
			id,//������
			&INVOICE[ADR],//��������� ���������	
			i,	 		     	//������ � �������
			MSG_Corr_REQ,	//��� ���������
			64,		 		//����� ������ ��������� � ������
			DAT_REQ,	    //������ ��������� - ������ ������
			TIME_SYS	  	//����� ����������� ���������
			);
    }else 
	if (id->CMD_TYPE[i]==CMD_B072_START)//������� ��� ������ ����� �072
    {
      FLAG_CMD=1;
      TIME_cons ();
      Transf("\r\n������ ������ �� ������ ����� �072\r\n");
      Transf2("~0 b072_start:1;");	  
	  FUNC_FLAG_UP (&POINTER_START_072,100);//������ ���������� ������ 
      ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);      
    }else 
       if (id->CMD_TYPE[i]==CMD_RESET_072)//������� �������� ������� RESET ��� 072
    {
      FLAG_CMD=1;
      TIME_cons ();
      Transf("\r\n������ ������ �� �������� ������� RESET ��� 072\r\n");
      idx0=idx_srv(id->INDEX[i],0);//������ ������������ ������ � "���������"
   	  idx1=idx_srv(id->INDEX[i],1);//������ ������������ ������ � "���������"
	  idx2=idx_srv(id->INDEX[i],2);//������ ������������ ������ � "���������"
	  idx3=idx_srv(id->INDEX[i],3);//������ ������������ ������ � "���������"
	  data=((srv->MeM[idx0])<<24)|((srv->MeM[idx1])<<16)|((srv->MeM[idx2])<< 8)|((srv->MeM[idx3]));
		
	  if (data==3) //���� ������ ������� CMD_RESET_072 ����� 3 - �� ��� ���� �� �������� ������� RESET
	  {
		  FLAG_ASQ_TEST_RESET=0;//������� ���� ������ �� ������ ���������� ����� ��� ������� �����
		  FLAG_CHECK_RST_072 =1;//��������� ���� �������� ������� ����� 		  //
		  FUNC_FLAG_UP (&POINTER_RESET_072_0,2);//������ ���������� ������ ��� ��������� ������� RESET ��� 072 ������, ������ ������� 0 � ��������!!!
	  }
	  
	  if (data==1) //���� ������ ������� CMD_RESET_072 ����� 1 - �� ��� ������ RESET
	  {
		  FUNC_FLAG_UP (&POINTER_RESET_072_0,2);//������ ���������� ������ ��� ��������� ������� RESET ��� 072 ������, ������ ������� 0 � ��������!!!
		  FUNC_FLAG_UP (&POINTER_ADR_COLLECT,100);//������ ���������� ������ ��� ������ ������ �� ���������
	  }
    }
		
    //----------------------------------------------------------
    //---------�������� ��������� � ������� �����---------------
    if (FLAG_CMD==1)
    {
      FLAG_CMD=0;
	  //------------------------------ 
	  /*
      ADR=ADR_FINDER(id->SENDER_ID[i],&ADDR_SNDR);//���� ���������� ����� ����������� � ��������� ������������, ���� ��� ��� ���  - �� ������� ����     
			ERROR_CMD_MSG ( //��������� ��������� � ���������� �������
			id,			        //��������� �� ������
			&INVOICE[ADR],  //��������� �� ��������� ���������
			i, 			        //������ ������� � �������
			MSG_CMD_OK,	    //��������� ���������
			0,				      //������ ���������
			TIME_SYS	      //������� ��������� ����� 
			);	
		*/	
			SERV_ID_DEL (id,i);//������� ������� �� �������
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
	if ((TIMER_T1HZ_MK>1050)&&(FLAG_T1HZ_MK==1)) 
	{
		FLAG_T1HZ_MK=0;
		Transf("������ ������ T1HZ_MK!!!\r\n");
	}
	B330.FLAG_1HZ=FLAG_T1HZ_MK;
	if (FLAG_T1HZ_MK==1) LED_SINHR=1; else LED_SINHR=2;
	
}
  
void LED_CONTROL (void)
{
       u32 z =0;
static u32 z0; 

if (TIMER_LS>2000) {LED_LS=2;}

if (FLAG_LED_TEST==0)
	{
	z=LED_ISPRAV_AC+((LED_PROGR    &3)<< 4)+
					((LED_OFCH     &3)<<21)+
					((LED_SINHR    &3)<<18)+
					((LED_LS       &3)<<16)+
					((LED_ISPR_J330&3)<<14)+
					((LED_OTKL_AC  &1)<<12)+
					((LED_TEMP     &3)<<8);	
	} else
if (FLAG_LED_TEST==1)
	{
		LED_ISPRAV_AC=0;
		LED_PROGR    =0;
		LED_OFCH     =0;
		LED_SINHR    =0;
		LED_LS       =0;
		LED_ISPR_J330=0;
		LED_OTKL_AC  =0;
		LED_TEMP     =0;
	z=LED_ISPRAV_AC+((LED_PROGR    &3)<< 4)+
					((LED_OFCH     &3)<<21)+
					((LED_SINHR    &3)<<18)+
					((LED_LS       &3)<<16)+
					((LED_ISPR_J330&3)<<14)+
					((LED_OTKL_AC  &1)<<12)+
					((LED_TEMP     &3)<<8);
		
	}else
if (FLAG_LED_TEST==2)
	{
		LED_ISPRAV_AC=1;
		LED_PROGR    =1;
		LED_OFCH     =1;
		LED_SINHR    =1;
		LED_LS       =1;
		LED_ISPR_J330=1;
		LED_OTKL_AC  =1;
		LED_TEMP     =1;
	z=LED_ISPRAV_AC+((LED_PROGR    &3)<< 4)+
					((LED_OFCH     &3)<<21)+
					((LED_SINHR    &3)<<18)+
					((LED_LS       &3)<<16)+
					((LED_ISPR_J330&3)<<14)+
					((LED_OTKL_AC  &1)<<12)+
					((LED_TEMP     &3)<<8);
		
	}else
if (FLAG_LED_TEST==3)
	{
		LED_ISPRAV_AC=2;
		LED_PROGR    =2;
		LED_OFCH     =2;
		LED_SINHR    =2;
		LED_LS       =2;
		LED_ISPR_J330=2;
		LED_OTKL_AC  =2;
		LED_TEMP     =2;
	z=LED_ISPRAV_AC+((LED_PROGR    &3)<< 4)+
					((LED_OFCH     &3)<<21)+
					((LED_SINHR    &3)<<18)+
					((LED_LS       &3)<<16)+
					((LED_ISPR_J330&3)<<14)+
					((LED_OTKL_AC  &1)<<12)+
					((LED_TEMP     &3)<<8);
		
	}


if (START_BP==0)  z=0;

if (TIMER_LED>LED_INTERVAL) 
	{
		if (z!=z0)
		{
			TCA_WR(z);//��������� �������
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
/* ��������� ���� ����� */
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

  qsort(tmp, n, sizeof (int), (int(*) (const void *, const void *)) comp);//��������� ��������� ������
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
	int i=0;
	int tmp=0;
	u32 tmp0=0;   
	
  if ((START_BP==1)&&(TIMER_CONTROL_SYS>SYS_INTERVAL))
  {
	flag=1;
    TIMER_CONTROL_SYS=0;
    //�������� ID
    LM_MFR_ID(1);
    LM_MFR_ID(2);
    LM_MFR_ID(3);
    LM_MFR_ID(4);
    LM_MFR_ID(5);
    LM_MFR_ID(6);
    LM_MFR_ID(7);
    LM_MFR_ID(8);
    //��������� �����������
    if (B330.CH[0]==1) LM[0].TEMP=LM_TEMP(1); else  LM[0].TEMP=20;
    if (B330.CH[1]==1) LM[1].TEMP=LM_TEMP(2); else  LM[1].TEMP=20;
    if (B330.CH[2]==1) LM[2].TEMP=LM_TEMP(3); else  LM[2].TEMP=20;
    if (B330.CH[3]==1) LM[3].TEMP=LM_TEMP(4); else  LM[3].TEMP=20;
    if (B330.CH[4]==1) LM[4].TEMP=LM_TEMP(5); else  LM[4].TEMP=20; 
    if (B330.CH[5]==1) LM[5].TEMP=LM_TEMP(6); else  LM[5].TEMP=20;
    if (B330.CH[6]==1) LM[6].TEMP=LM_TEMP(7); else  LM[6].TEMP=20; 
    if (B330.CH[7]==1) LM[7].TEMP=LM_TEMP(8); else  LM[7].TEMP=20;

    if ((LM[0].TEMP>tmp0)&&(B330.CH[0]==1)) tmp0=LM[0].TEMP;
    if ((LM[1].TEMP>tmp0)&&(B330.CH[1]==1)) tmp0=LM[1].TEMP;
    if ((LM[2].TEMP>tmp0)&&(B330.CH[2]==1)) tmp0=LM[2].TEMP;
    if ((LM[3].TEMP>tmp0)&&(B330.CH[3]==1)) tmp0=LM[3].TEMP;
    if ((LM[4].TEMP>tmp0)&&(B330.CH[4]==1)) tmp0=LM[4].TEMP;
    if ((LM[5].TEMP>tmp0)&&(B330.CH[5]==1)) tmp0=LM[5].TEMP;
    if ((LM[6].TEMP>tmp0)&&(B330.CH[6]==1)) tmp0=LM[6].TEMP;
    if ((LM[7].TEMP>tmp0)&&(B330.CH[7]==1)) tmp0=LM[7].TEMP;

    B330.TEMP_MAX=tmp0;tmp0=0;//4-�� �������� �������
    //B330.P = 1000;
    //��������� ������������� ����
	for (i=0;i<8;i++) 
	{
		LM[i].I=FILTR (LM_in_i(i+1),LM[i].FI,12);
		
		if (LM[i].I==0xffffffff) LM[i].I=0;
			
		if (LM[i].I>LM[i].I_max) 
		{
			ERROR_FLAG.ALERT=1;//���� ���� ���������� �� ���� ��������� ���� ������ ����
			ERROR_FLAG.INDEX=i;
			ERROR_FLAG.CODE = ERROR_CODE_Imax;//��������� ��� - ���������� ���� � ������
			LM[i].ERROR=1; //��������� ���� ������ ����� ������
		}
	}	
	
    for (i=0;i<8;i++) if ((LM[i].I>tmp0)&&(B330.CH[i]==1)) tmp0=LM[i].I;//���� ������������ ��� � �����

    B330.I = tmp0;tmp0=0;//3-��� �������� �������

    //��������� ���������� � �������
	for (i=0;i<8;i++) 
	{
		LM[i].U=FILTR (LM_v(i+1),LM[i].FU,12);
		if (LM[i].U==0xffffffff) LM[i].U=0;
	}

    //��������� ������������ ��������
	for (i=0;i<8;i++) LM[i].P=FILTR (cnvrt (LM[i].I,LM[i].U),LM[i].FP,12);

    //4-�� �������� �������
	for (i=0;i<8;i++)  tmp+=LM[i].P;
	B330.P =tmp;

	for (i=0;i<8;i++) if ((LM[i].U>tmp0)&&(B330.CH[i]==1)) tmp0=LM[i].U;

    B330.U_max=tmp0;tmp0=100000;//4-�� �������� �������

    for (i=0;i<8;i++) if ((LM[i].U<tmp0)&&(B330.CH[i]==1)) tmp0=LM[i].U;

    B330.U_min=tmp0;tmp0=0; //4-�� �������� �������

  } else
	  if ((START_BP==0)&&(flag==1))
  {
	flag=0;
	for (i=0;i<8;i++) 
	{
	 LM[i].TEMP=0xffffffff;
     LM[i].U=0;
	 LM[i].I=0;
	 LM[i].P=0;
	}

  }
  
  u8 err=0;
  if  (B330.TEMP_MAX>TEMP_MAX) err++;
  if ((B330.I    >500)&&(B330.I    !=4294967295))    err++;
  if  (B330.U_min<1100)    err++;
  if ((B330.U_max>1250)&&(B330.U_max!=4294967295))    err++;
  if  (B330.P    >40000)   err++;
  if  (B330.FLAG_1HZ==0)   err++;
  
 /*
  Transf("\r\n");
  u_out("B330.CH[0]:",B330.CH[0]);
  u_out("B330.CH[1]:",B330.CH[1]);
  u_out("B330.CH[2]:",B330.CH[2]);
  u_out("B330.CH[3]:",B330.CH[3]);
  u_out("B330.CH[4]:",B330.CH[4]);
  u_out("B330.CH[5]:",B330.CH[5]);
  u_out("B330.CH[6]:",B330.CH[6]);
  u_out("B330.CH[7]:",B330.CH[7]);
  
  u_out("B330.TEMP_MAX:",B330.TEMP_MAX);
  u_out("B330.I       :",B330.I);
  u_out("B330.U_min   :",B330.U_min);
  u_out("B330.U_max   :",B330.U_max);
  u_out("B330.P       :",B330.P);
  u_out("B330.FLAG_1HZ:",B330.FLAG_1HZ);
*/
  if (err!=0) B330.STATUS_OK=0; else B330.STATUS_OK=1;
  if (B330.STATUS_OK==1) LED_ISPR_J330=1; else LED_ISPR_J330=2;
  //u_out("B330.STATUS_OK:",B330.STATUS_OK);
}

void ALARM_SYS_TEMP (void)  
{
	u16 var=0;
	int tmp=TEMP_MAX/100;
	
	if (LM[0].TEMP>LM[0].TEMP_max) var=var|(1<<0);
	if (LM[1].TEMP>LM[1].TEMP_max) var=var|(1<<1);
	if (LM[2].TEMP>LM[2].TEMP_max) var=var|(1<<2);
	if (LM[3].TEMP>LM[3].TEMP_max) var=var|(1<<3);
	if (LM[4].TEMP>LM[4].TEMP_max) var=var|(1<<4);
	if (LM[5].TEMP>LM[5].TEMP_max) var=var|(1<<5);
	if (LM[6].TEMP>LM[6].TEMP_max) var=var|(1<<6);
	if (LM[7].TEMP>LM[7].TEMP_max) var=var|(1<<7);
	
	if ((B072[0].TEMP>tmp)&&(B072[0].TEMP!=65535)) var=var|(1<< 8);
	if ((B072[1].TEMP>tmp)&&(B072[0].TEMP!=65535)) var=var|(1<< 9);
	if ((B072[2].TEMP>tmp)&&(B072[0].TEMP!=65535)) var=var|(1<<10);
	if ((B072[3].TEMP>tmp)&&(B072[0].TEMP!=65535)) var=var|(1<<11);
	if ((B072[4].TEMP>tmp)&&(B072[0].TEMP!=65535)) var=var|(1<<12);
	if ((B072[5].TEMP>tmp)&&(B072[0].TEMP!=65535)) var=var|(1<<13);
	if ((B072[6].TEMP>tmp)&&(B072[0].TEMP!=65535)) var=var|(1<<14);
	if ((B072[7].TEMP>tmp)&&(B072[0].TEMP!=65535)) var=var|(1<<15);
	
	if ((var!=0)||(NUMBER_OF_B072==0)) LED_TEMP=2; else LED_TEMP=1; 
}

void UART_CNTR (UART_HandleTypeDef *huart)
{
	if (huart->gState != HAL_UART_STATE_BUSY_TX)
	{
		NRE_RS485(0);//�������� ����� 485 �� ���� 
		DE_RS485 (0);//��������� ����� 485 �� ��������
	}
}  

void SETUP_IP0_072 (u8 adr,u32 ip)
{
  u8 tmp0;
/* 
  u8 a[64];
  for (int i=0;i<64;i++) a[i]=0;
   strcpy(a,"~0 setup_IP0:"); 
   a[1]= adr+0x30;
   sprintf (strng,"%d",ip);
   strcat(a,strng);
   strcat(a,";\r\n");
   Transf ("���������� �� ��������:");
   Transf (a);
   Transf ("\r\n");
   Transf2(a);
*/   
   tmp0=adr;//
   if (tmp0==8) tmp0=0;//�������� ��� 8-�� ��������� ����� �� ���� ���������!
   SPI_BP_WRITE_1 (tmp0,4,ip);//�������� ��� �� ������ ADR_SLAVE[1]
}

void SETUP_IP1_072 (u8 adr,u32 ip)
{

  u8 tmp0;
/*  
  u8 a[64];
  for (int i=0;i<64;i++) a[i]=0;
   strcpy(a,"~0 setup_IP1:"); 
   a[1]= adr+0x30; 
   sprintf (strng,"%d",ip);
   strcat(a,strng);
   strcat(a,";\r\n");
   Transf ("���������� �� ��������:");
   Transf (a);
   Transf ("\r\n");
   Transf2(a);
*/   
   tmp0=adr;//
   if (tmp0==8) tmp0=0;//�������� ��� 8-�� ��������� ����� �� ���� ���������!
   SPI_BP_WRITE_1 (tmp0,5,ip);//�������� ��� �� ������ ADR_SLAVE[1]
}

void SETUP_DEST_IP0_072 (u8 adr,u32 ip)
{

  u8 tmp0;
/*  
  u8 a[64];
  for (int i=0;i<64;i++) a[i]=0;
   strcpy(a,"~0 dest_IP0:"); 
   a[1]= adr+0x30; 
   sprintf (strng,"%d",ip);
   strcat(a,strng);
   strcat(a,";\r\n");
   Transf ("���������� �� ��������:");
   Transf (a);
   Transf ("\r\n");
   Transf2(a);
  */ 
   tmp0=adr;//
   if (tmp0==8) tmp0=0;//�������� ��� 8-�� ��������� ����� �� ���� ���������!
    SPI_BP_WRITE_1 (tmp0,6,ip);//�������� ��� �� ������ ADR_SLAVE[1]
}

void SETUP_DEST_IP1_072 (u8 adr,u32 ip)
{

  u8 tmp0;
 /* 
  u8 a[64];
  for (int i=0;i<64;i++) a[i]=0;
   strcpy(a,"~0 dest_IP1:"); 
   a[1]= adr+0x30; 
   sprintf (strng,"%d",ip);
   strcat(a,strng);
   strcat(a,";\r\n");
   Transf ("���������� �� ��������:");
   Transf (a);
   Transf ("\r\n");
   Transf2(a);
 */  
   tmp0=adr;//
   if (tmp0==8) tmp0=0;//�������� ��� 8-�� ��������� ����� �� ���� ���������!
    SPI_BP_WRITE_1 (tmp0,7,ip);//�������� ��� �� ������ ADR_SLAVE[1]
}

void REQ_VERSIYA (void)
{
  u32 tmp0,tmp1;
  tmp0=STM32_VERSION>>16;   //��� ����
  tmp1=STM32_VERSION&0xffff;//��� �����
  Transf("----------------------\r\n");
  Transf("������ �������� STM32:\r\n");
  xn_out("����:",tmp0);Transf("  ");x_out("�����:",tmp1); 
}

//��������� ����� �� ������� �� ���������
void answer_translated (u32 dat)
{
  u8 adr=0;
  if (POINTER_ADR_REQ.timer>0)//���� ������� �������� �� ������ ����!
  {
    adr=NUMBER_OF_B072++;
    ADR_SLAVE[adr]=dat;
  }
}

//��������� ����� ������� � ������� - ������ 072 �� ��������� ������ � ���������
void SLAVE_COUNT ()
{
  int i=0;
  int tmp0;

   tmp0=NUMBER_OF_B072;//������� ������ �������
   u_out("����� ������:",tmp0);
   Transf("-------------\r\n");
   for (i=0;i<tmp0;i++)
    {
      u_out("���� 072:",ADR_SLAVE[i]);
    }
}

//�������� ������ �� �������� ��� ������ ������ 072
void req_col (void)
{
   int i=0;
   u32 tmp0=0;
   NUMBER_OF_B072=0;//���������� ������� ����� ������
   for (i=0;i<8;i++)
    {
      ADR_SLAVE[i]=0xff;//������� ������ �������
    }
  
//  Transf2("~0 REQ_ADR;");//�������� ������ 

   for (i=1;i<9;i++)
    {
       if (i!=8) tmp0=SPI_BP_READ_TEST (i);//��������� ��� �� �������
	   else      tmp0=SPI_BP_READ_TEST (0);
	   
	   if (tmp0==0xDEEDBEEF) 
	   {
		   ADR_SLAVE[NUMBER_OF_B072]=i; 
		   NUMBER_OF_B072++;
		}
    }       
       
}

//�������� ������� �� �������������������� ETH MAC ����� 072
void CMD_MAC_RECONF ()
{
  Transf2("~0 eth_config;");//�������� ������ 
}

//��������� ��������� � ����� 072 �� ��������� spi
SYS_STATE_072 BP_072_READ (u8 adr)
{
  u8  tmp0;
  u64 tmp1;
  SYS_STATE_072 tmp2;
  
  tmp0=adr;
  if (tmp0==8) tmp0=0;          //�������� ��� 8-�� ��������� ����� �� ���� ���������!
      tmp1=SPI_BP64_READ (tmp0);//��������� ��� �� �������

  tmp2.TEMP     = tmp1     &0xffff;
  tmp0          =(tmp1>>56)&0xff;
  
  tmp2.REF      =(tmp0>>5)&1;
  tmp2.SYNC_1HZ =(tmp0>>4)&1;
  tmp2.ADC_0    =(tmp0>>0)&1;
  tmp2.ADC_1    =(tmp0>>1)&1;
  tmp2.DAC_0    =(tmp0>>2)&1;
  tmp2.DAC_1    =(tmp0>>3)&1;
  
  tmp2.SYNC0    =(tmp1>>48)&0xff;
  tmp2.SYNC1    =(tmp1>>40)&0xff;
  tmp2.ERROR_1HZ=(tmp1>>16)&0xffffff; 
  
 /* 
  Transf("-----------\r\n");
  un_out("B072[",adr);Transf("]:\r\n");
  u_out("TEMP:", tmp2.TEMP);
  u_out("REF:", tmp2.REF);
  u_out("SYNC_1HZ:", tmp2.SYNC_1HZ);
  */

return tmp2;
}

//��������� ���������� ������ 
void DISPATCHER (u32 timer) 
{
	u32 tmp0=0;
	int i=0;

	  if (FLAG_DWN(&POINTER_START_072)) //��������� ��� 072 �������
      {	
		Transf2("~0 b072_start:1;");
        return;
      } else
	  if (FLAG_DWN(&POINTER_RESET)) //������� �����
      {
        RESET_072(1);//
		NUMBER_OF_B072=0;//���������� ������� ����� ������
        FUNC_FLAG_UP (&POINTER_ADR_COLLECT,3000);//��������� ���� ��������� ������ - ������ ������� RESET
        return;
      } else
      if (FLAG_DWN(&POINTER_RESET_072_0))
      {
        TIME_cons ();
        Transf("ON RESET for 072.\r\n");
        RESET_072(0);//������������� ������ RESET �� ���� ���������
		NUMBER_OF_B072=0;//���������� ������� ����� ������
		FLAG_ASQ_TEST_RESET=0;
        FUNC_FLAG_UP (&POINTER_RESET_072_1,10);//��������� ���� ��������� ������ - ������ ������� RESET
        return;
      } else
	  if (FLAG_DWN(&POINTER_RESET_072_1))
      {
        TIME_cons ();
        Transf("OFF RESET for 072.\r\n");
        RESET_072(1);//������� ������ RESET �� ���� ���������		
		FUNC_FLAG_UP (&POINTER_ADR_COLLECT,3000);//���� ������� � ��������� �� ���������
		return;
      } else
	  if (FLAG_DWN(&POINTER_CHECK_RESET_072))
      {
        TIME_cons ();
        Transf("�������� ������� RESET ��� 072.\r\n");
		if (NUMBER_OF_B072==0) NUMBER_OF_B072=FUNC_FIND_072 ();
		u_out("���������� ������ 072 �� ���������:",NUMBER_OF_B072);
        if (NUMBER_OF_B072>0) FLAG_ASQ_TEST_RESET=1;//���� ������� �������
		else FLAG_ASQ_TEST_RESET=0;
		FUNC_FLAG_UP (&POINTER_MASTER_IP0_SETUP,200);
        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_RESET);//������� ��������� ������� �� ����������� ����� ������� RESET
        return;
      } else
      if (FLAG_DWN(&POINTER_ADR_COLLECT))
      {
        TIME_cons ();
        Transf("��� ���� �������!\r\n");
        req_col ();//����������� ������
        FUNC_FLAG_UP (&POINTER_ADR_REQ,300);//��������� ���� ��������� ������ - ����� ���������� ������ 072 � ������� � ����������� �� ����������
        return;
      } else
      if (FLAG_DWN(&POINTER_ADR_REQ))
      {
        TIME_cons ();
        SLAVE_COUNT ();
		if (FLAG_CHECK_RST_072==1) FUNC_FLAG_UP (&POINTER_CHECK_RESET_072,10);//��������� ���� ��������� ������ - �������� ����������������� ������� RESET
		else 
		if (NUMBER_OF_B072>0) FUNC_FLAG_UP (&POINTER_MASTER_IP0_SETUP,200);//��������� ���� ��������� ������ - ��������� ������ 072 IP0 �������, ���� ��� ����� ����
            
			FLAG_CHECK_RST_072=0;        
        return;
      } else
      if (FLAG_DWN(&POINTER_MASTER_IP0_SETUP))
      {
        TIME_cons ();
        Transf("������������� ������ IP0!\r\n");
        SETUP_IP0_072 (1,MASTER_IP0);//�������� IP0 ������� , �� ������ ����� ������ ���� �� ���������
        FUNC_FLAG_UP (&POINTER_SLAVE_IP0_SETUP,10);     //��������� ���� ��������� ������ - ��������� ������ 072 IP1 �������
		if (NUMBER_OF_B072>0) FLAG_ASQ_TEST_RESET=1;//���� ������� �������
		MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_RESET);//������� ��������� ������� �� ����������� ����� ������� RESET
        return;
      } else
       if (FLAG_DWN(&POINTER_SLAVE_IP0_SETUP))
      {
        TIME_cons ();
        Transf("������������� ����� IP0!\r\n");
        SETUP_IP0_072 (ADR_SLAVE[NUMBER_OF_B072-1],SLAVE_IP0); //�������� IP0 ������, �� ����� ����� �� ���������
        FUNC_FLAG_UP (&POINTER_MASTER_IP1_SETUP,10);     //��������� ���� ��������� ������ - ��������� ������ 072 IP1 �������
		if (NUMBER_OF_B072>0) FLAG_ASQ_TEST_RESET=1;//���� ������� �������
		MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_RESET);//������� ��������� ������� �� ����������� ����� ������� RESET
        return;
      } else
      if (FLAG_DWN(&POINTER_MASTER_IP1_SETUP))
      {
        TIME_cons ();
        Transf("������������� ������ IP1!\r\n");
        SETUP_IP1_072 (1,MASTER_IP1);//�������� IP1 ������� , �� ������ ����� ������ ���� �� ���������
        FUNC_FLAG_UP  (&POINTER_SLAVE_IP1_SETUP,10);//��������� ���� ��������� ������ - ��������� ������ 072 IP1 �������
        return;
      } else
        if (FLAG_DWN(&POINTER_SLAVE_IP1_SETUP))
      {
        TIME_cons ();
        Transf("������������� ����� IP1!\r\n");
        SETUP_IP1_072 (ADR_SLAVE[NUMBER_OF_B072-1],SLAVE_IP1); //�������� IP1 ������, �� ����� ����� �� ���������
        FUNC_FLAG_UP  (&POINTER_DEST_MASTER_IP0_SETUP,10);//��������� ���� ��������� ������ - ��������� ������ 072 IP1 �������
        return;
      } else
      if (FLAG_DWN(&POINTER_DEST_MASTER_IP0_SETUP))
      {
        TIME_cons ();
        Transf("������������� ������ DEST_IP0!\r\n");
        SETUP_DEST_IP0_072 (1,MASTER_DEST_IP0);//�������� IP1 ������� , �� ������ ����� ������ ���� �� ���������
        FUNC_FLAG_UP       (&POINTER_DEST_SLAVE_IP0_SETUP,10);    //��������� ���� ��������� ������ - ��������� ������ 072 DEST_IP1 �������
        return;
      } else
      if (FLAG_DWN(&POINTER_DEST_SLAVE_IP0_SETUP))
      {
        TIME_cons ();
        Transf("������������� ����� DEST_IP0!\r\n");
        SETUP_DEST_IP0_072 (ADR_SLAVE[NUMBER_OF_B072-1], SLAVE_DEST_IP0);//�������� IP1 ������, �� ����� ����� �� ���������
        FUNC_FLAG_UP       (&POINTER_DEST_MASTER_IP1_SETUP,10);    //��������� ���� ��������� ������ - ��������� ������ 072 DEST_IP1 �������
        return;
      } else
      if (FLAG_DWN(&POINTER_DEST_MASTER_IP1_SETUP))
      {
        TIME_cons ();
        Transf("������������� ������ DEST_IP1!\r\n");
        SETUP_DEST_IP1_072 (1,MASTER_DEST_IP1);//�������� IP1 ������� , �� ������ ����� ������ ���� �� ���������
        FUNC_FLAG_UP       (&POINTER_DEST_SLAVE_IP1_SETUP,10);    //��������� ���� ��������� ������ - �������� ���-��� 072 �� ����� ������������� IP
        return;
      } else
      if (FLAG_DWN(&POINTER_DEST_SLAVE_IP1_SETUP))
      {
        TIME_cons ();
        Transf("������������� ����� DEST_IP1!\r\n");
        SETUP_DEST_IP1_072 (ADR_SLAVE[NUMBER_OF_B072-1], SLAVE_DEST_IP1);//�������� IP1 ������, �� ����� ����� �� ���������
        FUNC_FLAG_UP       (&POINTER_ETHERNET_RERUN,10); //��������� ���� ��������� ������ - �������� ���-��� 072 �� ����� ������������� IP
        if (NUMBER_OF_B072>0) FLAG_ASQ_TEST_RESET=1;//���� ������� �������
		MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_RESET);//������� ��������� ������� �� ����������� ����� ������� RESET
		return;
      } else
      if (FLAG_DWN(&POINTER_ETHERNET_RERUN))
      {
        TIME_cons ();
        Transf("������� ��� ����� 072!\r\n");
        CMD_MAC_RECONF ();
        return;
      } else
       if (FLAG_DWN(&POINTER_TEST_485))
      {
		FUNC_FLAG_UP (&POINTER_TEST_485_REQ,300);//������ ���������� ������ ��� �������� ������� ������ �� ���� 485
        TIME_cons ();
        Transf("�������� ��� �� ���� 485!\r\n");
        tmp0=ADR_SLAVE[NUMBER_OF_B072-1];//
		u_out("REQ_ADR:",tmp0);
        BUS_485_TEST (tmp0);	
        return;
      } else		
	    if (FLAG_DWN(&POINTER_TEST_485_REQ))
      {
        TIME_cons ();
        Transf("��������� ��������� ����� ���� 485!\r\n");
        if (FLAG_ASQ_TEST_485==1) Transf("���� �������!\r\n");
		    else
        {
          if (SCH_TST1<5)
          {
            SCH_TST1++;
			u_out("�������:",SCH_TST1+1);
            FUNC_FLAG_UP (&POINTER_TEST_485,10);//������ �����
          } else Transf("���� �� �������!\r\n");
        }    

        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_485);//������� ��������� ������� �� ����������� �����
        return;
      } else
       if (FLAG_DWN(&POINTER_TEST_SPI))
      {
		    FUNC_FLAG_UP (&POINTER_TEST_SPI_REQ,10);//������ ���������� ������ ��� �������� ������� ������ �� ���� SPI
        TIME_cons ();
        Transf("�������� ��� �� ���� SPI!\r\n");
        FLAG_ASQ_TEST_SPI=0;
        tmp0=ADR_SLAVE[NUMBER_OF_B072-1];//
        u_out("Adr:",ADR_SLAVE[1]);
		    if (tmp0==8) tmp0=0;//�������� ��� 8-�� ��������� ����� �� ���� ���������!
        SPI_BP_WRITE (tmp0,0xDEEDBEEF);//�������� ��� �� ������ ADR_SLAVE[1]
        return;
      }else		
	    if (FLAG_DWN(&POINTER_TEST_SPI_REQ))
      {
        TIME_cons ();
        Transf("��������� ��������� ����� ���� SPI!\r\n");
        tmp0=ADR_SLAVE[NUMBER_OF_B072-1];//
		    if (tmp0==8) tmp0=0;//�������� ��� 8-�� ��������� ����� �� ���� ���������!
        tmp0=SPI_BP_READ (tmp0);//��������� ��� �� �������
        x_out("Ancwer:",tmp0);
        if (tmp0==0xDEEDBEEF) {Transf("���� �������!\r\n");FLAG_ASQ_TEST_SPI=1;}
		    else               Transf("���� �� �������!\r\n");
        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_SPI);//������� ��������� ������� �� ����������� �����
        return;
      } else
       if (FLAG_DWN(&POINTER_TEST_JTAG))
      {
		FUNC_FLAG_UP (&POINTER_TEST_JTAG_REQ,500);//
        TIME_cons ();
        Transf("���������� ���� JTAG!\r\n");
        FLAG_ASQ_TEST_JTAG=0;
        tmp0=jtag_scan(NULL,0);
        u_out("tmp0:",tmp0);
        return;
      }else		
	    if (FLAG_DWN(&POINTER_TEST_JTAG_REQ))
      {
        TIME_cons ();
        Transf("��������� ��������� ����� ���� JTAG!\r\n");
        u_out("���������� ���������:",jtag_dev_count);
        for(i=0;i<jtag_dev_count;i++)
        {
          un_out("",i);Transf("] ");xn_out("",jtag_devs[i].idcode);Transf(" - ");Transf(jtag_devs[i].descr);Transf("\r\n");
        }        

        if (jtag_dev_count==NUMBER_OF_B072)
        {
        	if (jtag_devs[NUMBER_OF_B072-1].idcode==0x2a020dd)
        	{     Transf("���� �������!\r\n"   );FLAG_ASQ_TEST_JTAG=1;}
		    else  Transf("���� �� �������!\r\n");
        } 

        MSG_SEND_UDP (&ID_SERV1,&SERV1,MSG_REQ_TEST_JTAG);//������� ��������� ������� �� ����������� �����
        return;
      } if (FLAG_DWN(&POINTER_LED_TEST0))
      {
        Transf("C���������:0\r\n");
		FLAG_LED_TEST=1;
        FUNC_FLAG_UP (&POINTER_LED_TEST1,2000);
        return;
      } else
		if (FLAG_DWN(&POINTER_LED_TEST1))
      {
        Transf("C���������:1\r\n");
		FLAG_LED_TEST=2;
        FUNC_FLAG_UP (&POINTER_LED_TEST2,2000);
        return;
      } else
		if (FLAG_DWN(&POINTER_LED_TEST2))
      {
        Transf("C���������:2\r\n");
		FLAG_LED_TEST=3;
        FUNC_FLAG_UP (&POINTER_LED_TEST3,2000);
        return;
      } else
		if (FLAG_DWN(&POINTER_LED_TEST3))
      {
		Transf("C���������:�����\r\n");
		LED_ISPRAV_AC=1;
		LED_PROGR    =0;
		LED_OFCH     =1;
		LED_SINHR    =1;
		LED_LS       =2;
		LED_ISPR_J330=1;
		LED_OTKL_AC  =1;
		LED_TEMP	 =1;
		FLAG_LED_TEST=0;
        return;
      }   

  }

//��� ��������� ���������� ������ �� ��������� ����� ���������� �� �������
void FUNC_TEST_FIND (void)
{
	int i=0;
	u32 tmp=0;
	if (FLAG_REJ_OSNPAR==1)
	{
		for (i=0;i<8;i++)
		{
			tmp=SPI_BP_READ_TEST (i);//��������� ��� �� �������
			if (tmp==0xDEEDBEEF) 
			{
				u_out("������� ������:",i);
				ERROR_FLAG.ALERT=1;
				ERROR_FLAG.INDEX=i-1;
				ERROR_FLAG.CODE=ERROR_CODE_REJ;
				FLAG_REJ_OSNPAR=0;
			}
		}		
	}	
}

//������� ������������ ���������� ������ �� ���������
int FUNC_FIND_072 (void)
{
	int i=0;
	int n=0;
	u32 tmp=0;

		for (i=0;i<8;i++)
		{
			tmp=SPI_BP_READ_TEST (i);//��������� ��� �� �������
			if (tmp==0xDEEDBEEF) 
			{
				n++;
			}
		}
		return n;
}

//��� ��������� ��������� ���������� �� ���� � ������������� ������� �� ������� �����
void FUNC_FLAG_UP (POINTER *p,u32 time)
{
  int i=0;
  p->FLAG=0;
  p->timer=time;
  while (PNT[i]!=0) {i++;};//���� ��������� ����� � ������� ����������
  PNT[i]=p;
  //Transf("��������� ���� ����������� �������!\r\n");
  //TIME_cons ();
  //u_out("timeout:",time);
}

//��� ��������� ������� ���������� �� ����
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
	a=(p[0]<<24)|(p[1]<<16)|(p[2]<<8)|(p[3]<<0);//������� ���� � ������� ������!!!
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
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS1_read(READ_BYTES,adr,p,n);//������ n ���� ������
}

void EPCS2_READ (u32 adr,u8 *p,u32 n)
{
  while ((spi_EPCS2_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS2_read(READ_BYTES,adr,p,n);//������ n ���� ������
}

void EPCS1_WRITE_BUF(u32 adr,u8 *p,u32 n)
{
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS1_wr_ENABLE(); //��������� ������ �� ����
  spi_EPCS1_write(WRITE_BYTES,adr,p,n);
  spi_EPCS1_wr_DISABLE();//��������� ������ �� ����
}

void EPCS2_WRITE_BUF(u32 adr,u8 *p,u32 n)
{
  while ((spi_EPCS2_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS2_wr_ENABLE(); //��������� ������ �� ����
  spi_EPCS2_write(WRITE_BYTES,adr,p,n);
  spi_EPCS2_wr_DISABLE();//��������� ������ �� ����
}

void EPCS1_ERASE_SECTOR(u32 adr)
{
  u8 a[1]={0xff};
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS1_wr_ENABLE();//��������� ������ �� ����
  spi_EPCS1_write(ERASE_SECTOR,adr,a,0);
  spi_EPCS1_wr_DISABLE();//��������� ������ �� ����
}

void EPCS2_ERASE_SECTOR(u32 adr)
{
  u8 a[1]={0xff};
  while ((spi_EPCS2_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS2_wr_ENABLE();//��������� ������ �� ����
  spi_EPCS2_write(ERASE_SECTOR,adr,a,0);
  spi_EPCS2_wr_DISABLE();//��������� ������ �� ����
}

void EPCS2_ERASE_ALL(void)
{
  while ((spi_EPCS2_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS2_wr_ENABLE ();//��������� ������ �� ����
  spi_EPCS2_ERASE_BULK();//������� ��� �� ����
  spi_EPCS2_wr_DISABLE();//��������� ������ �� ����
}

void EPCS1_ERASE_ALL(void)
{
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS1_wr_ENABLE ();//��������� ������ �� ����
  spi_EPCS1_ERASE_BULK();//������� ��� �� ����
  spi_EPCS1_wr_DISABLE();//��������� ������ �� ����
}

void SERIAL_NUMBER_WR (u32 data)
{
	u8 buf[4];
  EPCS1_ERASE_SECTOR(SERIAL_ADR_FLASH);//������� ������ 
	u_out("���������� �������� �����:",data);
	buf[0]=(data>>24)&0xff;
	buf[1]=(data>>16)&0xff;
	buf[2]=(data>> 8)&0xff;
	buf[3]=(data>> 0)&0xff;
  while ((spi_EPCS1_STATUS()&1)==1){WATCH_DOG ();};	//��������� ��� ���� �� ������
  spi_EPCS1_wr_ENABLE(); //��������� ������ �� ����
  spi_EPCS1_write(WRITE_BYTES,SERIAL_ADR_FLASH,buf,4);
  spi_EPCS1_wr_DISABLE();//��������� ������ �� ����
}

//������ ������������� ��������� ��������� ����
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
  EPCS1_ERASE_SECTOR(CorrI_ADR_FLASH);//������� ������ ������������� ��������� ����
  PACK_ARR_FLOAT (B330.Corr_I,Arr,8);
  x=8*4;
  EPCS1_WRITE_BUF(CorrI_ADR_FLASH,Arr,x);//���������� ���� ������ ������������� �� ����
  PACK_ARR_FLOAT (B330.Corr_U,Arr,8);
  x=8*4;
  EPCS1_WRITE_BUF(CorrU_ADR_FLASH,Arr,x);//���������� ���� ������ ������������� �� ����
}

void CorrI_write (void) 
{
  u8 Arr[32];
  EPCS1_ERASE_SECTOR(CorrI_ADR_FLASH);//������� ������ ������������� ��������� ����
  PACK_ARR_FLOAT (B330.Corr_I,Arr,8);
  int x=8*4;
  EPCS1_WRITE_BUF(CorrI_ADR_FLASH,Arr,x);//���������� ���� ������ ������������� �� ����
}

 //������ ������������� ��������� ��������� ����������
void CorrU_write (void)
{
   u8 Arr[32];
   EPCS1_ERASE_SECTOR(CorrU_ADR_FLASH);//������� ������ ������������� ��������� ����
   PACK_ARR_FLOAT (B330.Corr_U,Arr,8);
   int x=8*4;
   EPCS1_WRITE_BUF(CorrU_ADR_FLASH,Arr,x);//���������� ���� ������ ������������� �� ����
}

void CorrI_read (void)
{
  u8 a[64];
  int x=8*4;//������ ������� � ������
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
  int x=8*4;//������ ������� � ������
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

//��������� �������� ����� �� ����������
u64 SERIAL_NUMBER (void)
{
   u8 buf[4];//{}
   int adr=SERIAL_ADR_FLASH;
   u64 data=0;

   spi_EPCS1_read(READ_BYTES,adr,buf,4);
   data=buf_to_data(buf);
   u_out("�������� ����� �����:",data);
   return data;
}

void SYS_072_STATE (void)
{
	int i=0;
	int error=0;
	int msg=0;
	int tmp=TEMP_MAX/100; 
	
			            B072[0]=BP_072_READ (ADR_SLAVE[0]);//������� ������		
  if (NUMBER_OF_B072>1) B072[1]=BP_072_READ (ADR_SLAVE[1]);//������� ����
  if (NUMBER_OF_B072>2) B072[2]=BP_072_READ (ADR_SLAVE[2]);//������� ����
  if (NUMBER_OF_B072>3) B072[3]=BP_072_READ (ADR_SLAVE[3]);//������� ����
  if (NUMBER_OF_B072>4) B072[4]=BP_072_READ (ADR_SLAVE[4]);//������� ����
  if (NUMBER_OF_B072>5) B072[5]=BP_072_READ (ADR_SLAVE[5]);//������� ����
  if (NUMBER_OF_B072>6) B072[6]=BP_072_READ (ADR_SLAVE[6]);//������� ����
  if (NUMBER_OF_B072>7) B072[7]=BP_072_READ (ADR_SLAVE[7]);//������� ����
  
  if (B072[NUMBER_OF_B072-1].REF==1) LED_OFCH=1; else LED_OFCH=2;
  
  for (i=0;i<NUMBER_OF_B072;i++)
  {
	  if (B072[i].ADC_0!=1) error++;
	  if (B072[i].ADC_1!=1) error++;
	  if (B072[i].DAC_0!=1) error++;
	  if (B072[i].DAC_1!=1) error++;
  }
  
  if ((error>0)||(NUMBER_OF_B072==0)) LED_ISPRAV_AC=2; else LED_ISPRAV_AC=1;
  
  if ((LED_TEMP==2)&&(NUMBER_OF_B072!=0))
  {
	  for (i=0;i<8;i++) 
		  if (B072[i].TEMP>tmp) 
		  {
			nu_out("B072[",i); u_out("]:",B072[i].TEMP);
		  }
  }
}

void ALARM_SYSTEM (ID_SERVER *id,SERVER *srv)//�������� �� ��������� ����������
{
	int i=0;
    u64 ADR=ADRES_SENDER_CMD;	
	
	if (ERROR_FLAG.ALERT==1) //�������� ����� ������
	{
		AVARIYA_OTKL ();
		ERROR_FLAG.ALERT=0;
		
		D_TEMP[0]=0;
		D_TEMP[1]=0;
		D_TEMP[2]=ERROR_FLAG.INDEX;//���������� ����� ������ ��� ���� ������
		D_TEMP[3]=ERROR_FLAG.CODE; //���������� ��� ������
		
			SYS_CMD_MSG(
			id,//������
			&INVOICE[ADR], //��������� ���������	
			0,	 		   //������ � �������
			MSG_ERROR_REJ, //��� ���������
			4,		       //����� ������ ��������� � ������
			D_TEMP,        //������ ��������� - ������ ������
			TIME_SYS  	   //����� ����������� ���������
			);
	}
}

int main(void)
{
	int i=0;
  /* USER CODE BEGIN 1 */
  Adress='b'; //������ ������� 

  for (i=0;i<PNT_BUF;i++) PNT[i]=NULL;//�������� ����� ����������
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
//MX_TIM4_Init();  �� �����, � ����� ��� ������ ����!!!
//MX_SPI2_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_SPI5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
//MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

//  Delay(1000);
	RESET_072(1);
//------SETUP----------
LM[0].TEMP_max=5000;//������ ��� ����� ������� � �������, ����� ��� ����� ������� � �����
LM[1].TEMP_max=5000;
LM[2].TEMP_max=5000;
LM[3].TEMP_max=5000;
LM[4].TEMP_max=5000;
LM[5].TEMP_max=5000;
LM[6].TEMP_max=5000;
LM[7].TEMP_max=5000;

LM[0].I_max=499;//4.99 �����
LM[1].I_max=499;
LM[2].I_max=499;
LM[3].I_max=499;
LM[4].I_max=499;
LM[5].I_max=499;
LM[6].I_max=499;
LM[7].I_max=499;

//---------------------
  
  Transf("-------------\r\n");
  Transf("    �330\r\n");
  Transf("-------------\r\n");
  DE_RS485(1);
  CS_5_MK(1);
  CS_EPCS1(1);
  CS_EPCS2(1);  
  //---������!!!--------
  //PE14_0;
  //PE12_0;
  //--------------------
  
  PWDN_4(1);
  
  ENABLE_LM25056_MK(1);//enable 
  Delay(2);
  ENABLE_LM25056_MK(0);//enable 
  PWR_072 (255);	   //����� ������� �� �������	
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
 PWDN_4(0);//������� ��������� � wiz820
 NSS_4(1);
 RES_4(0);
 Delay(2);
 RES_4(1);
 INIT_SERV_ARCHIV (&SERV1,&ID_SERV1,&ADDR_SNDR);//�������������� ��������� � ������
 
 while (i<200)
 {
	 i++;
	 Delay(1);
	 WATCH_DOG ();
 }
 
 Set_network();
 RECEIVE_udp(0, 3001,1); 
 
 for (i=0;i<8;i++) ADR_SLAVE[i]=0xff;//������� ������ � �������� ������� (��� ������ ������ ��� ������ �� 485 ����)

//-------------------------------------
//           ��� ����� �� ��

//  TCA_WR(255);//�������� ��� ���������� �� ������� ������
//------------------------------------- 
  B330.B330_NUMBER=SERIAL_NUMBER ();
  CorrI_read  ();  //��������� ����������� ������������ ��� ��������� ��������� ����
  CorrU_read  ();  //��������� ����������� ������������ ��� ��������� ��������� ����������
  Console_corr();//������� ����������� ������������
  B330.PRG_VERSIYA=STM32_VERSION;
  B330.WORK_TIME=TIME_OF_WORK; //����� ��������� ����� � �������� �����;
  
  LED_ISPRAV_AC=1;
  LED_PROGR    =0;
  LED_OFCH     =1;
  LED_SINHR    =1;
  LED_LS       =2;
  LED_ISPR_J330=1;
  LED_OTKL_AC  =1;
  LED_TEMP	   =1;
  
  RESET_072(0);//������� ����� 072 ������


  while (1)
  {
    /* USER CODE END WHILE */

	WATCH_DOG ();
	LED();
	UART_conrol();
	BP_start(START_BP,PWR_ALL);
	
	if (EVENT_INT1==1)
	{
		EVENT_INT1=0;
	//	Transf("event 1!\r");
		RECEIVE_udp (0, 3001,1);	
		LED_LS =1;//���� ����� �� ��
		TIMER_LS=0;
	}; 
	
	if (EVENT_INT3==1)//�������� ��������� ����� T1HZ_MK
	{
		if (FLAG_T1HZ_MK==0) Transf("����   ������ T1HZ_MK!\r");
        FLAG_REQ_STATUS=1;		
	
		EVENT_INT3=0;
		FLAG_T1HZ_MK=1;
		TIMER_T1HZ_MK=0;
		TIME_OF_SECOND++;//������������ ����� ������ � ������� ���������			
	}; 
	
	if (FLAG_REQ_STATUS==1) //�������� ��������� � ����� ���������
	{
		SYS_072_STATE ();//����� ��������� 072 ������	
		SYS_INFO_SEND_UDP(&ID_SERV1,&SERV1);
		STATE_B072_INFO_SEND_UDP (&ID_SERV1,&SERV1);//�������� � ��������� ����� �072
		FLAG_REQ_STATUS=0;
	}

    DISPATCHER          (TIMER_TIMEOUT);//��������� ���������� ������
	ALARM_SYS_TEMP      ();//���������� ���������� ����������� � ��������� ���������  
	ALARM_SYSTEM        (&ID_SERV1,&SERV1);//�������� �� �������� ����������
    CONTROL_SYS         ();//��������� ��������� �������: ����������� , ��� ����������� � �.�.
	CONTROL_POK 	    ();
	LED_CONTROL 	    ();
	CONTROL_T1HZ_MK     ();
	CMD_search          (&ID_SERV1,&SERV1);
	SEND_UDP_MSG 	    ();
    UART_DMA_TX  	    ();
	UART_DMA_TX2  	    ();
	UART_CNTR           (&huart2);//��� ��������� ��������� 485
	FTIME_OF_WORK       ();     //��� ������ �� �������� ���������
	FUNC_TEST_FIND      ();//�������� � ������ ��������� �������� ����������! ��������� ���������� ������ �� ��������� ����� ���������� �� �������

  }

}
