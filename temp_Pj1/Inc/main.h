/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */


/**
 * The 8-bit signed data type.
 */
typedef char int8;
/**
 * The volatile 8-bit signed data type.
 */
typedef volatile char vint8;
/**
 * The 8-bit unsigned data type.
 */
typedef unsigned char uint8;
/**
 * The volatile 8-bit unsigned data type.
 */
typedef volatile unsigned char vuint8;

/**
 * The 16-bit signed data type.
 */
typedef int int16;
/**
 * The volatile 16-bit signed data type.
 */
typedef volatile int vint16;
/**
 * The 16-bit unsigned data type.
 */
typedef unsigned short uint16;
/**
 * The volatile 16-bit unsigned data type.
 */
typedef volatile unsigned int vuint16;
/**
 * The 32-bit signed data type.
 */
typedef long int32;
/**
 * The volatile 32-bit signed data type.
 */
typedef volatile long vint32;
/**
 * The 32-bit unsigned data type.
 */
typedef unsigned long uint32;


/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

#define PB0_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_RESET)
#define PB0_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,GPIO_PIN_SET)

#define PB5_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET)
#define PB5_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET)

#define PB6_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_RESET)
#define PB6_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6,GPIO_PIN_SET)

#define PB7_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET)
#define PB7_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET)

#define PB9_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,GPIO_PIN_RESET)
#define PB9_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,GPIO_PIN_SET)

#define PB10_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_RESET)
#define PB10_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10,GPIO_PIN_SET)

#define PB12_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_RESET)
#define PB12_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12,GPIO_PIN_SET)

#define PB13_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_RESET)
#define PB13_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13,GPIO_PIN_SET)

#define PB15_0  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_RESET)
#define PB15_1  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15,GPIO_PIN_SET)

#define PC13_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_RESET)
#define PC13_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,GPIO_PIN_SET)

#define PC3_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_RESET)
#define PC3_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3,GPIO_PIN_SET)

#define PC1_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_RESET)
#define PC1_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1,GPIO_PIN_SET)

#define PC2_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_RESET)
#define PC2_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2,GPIO_PIN_SET)

#define PC8_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_RESET)
#define PC8_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,GPIO_PIN_SET)

#define PC14_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_RESET)
#define PC14_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14,GPIO_PIN_SET)

#define PC15_0  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_RESET)
#define PC15_1  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15,GPIO_PIN_SET)


#define PD0_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_RESET)
#define PD0_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0,GPIO_PIN_SET)

#define PD1_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,GPIO_PIN_RESET)
#define PD1_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1,GPIO_PIN_SET)

#define PD2_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_RESET)
#define PD2_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2,GPIO_PIN_SET)

#define PD3_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,GPIO_PIN_RESET)
#define PD3_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3,GPIO_PIN_SET)

#define PD4_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,GPIO_PIN_RESET)
#define PD4_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4,GPIO_PIN_SET)

#define PD5_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,GPIO_PIN_RESET)
#define PD5_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5,GPIO_PIN_SET)

#define PD7_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,GPIO_PIN_RESET)
#define PD7_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7,GPIO_PIN_SET)

#define PD8_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,GPIO_PIN_RESET)
#define PD8_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8,GPIO_PIN_SET)

#define PD9_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,GPIO_PIN_RESET)
#define PD9_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9,GPIO_PIN_SET)

#define PD10_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_RESET)
#define PD10_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10,GPIO_PIN_SET)

#define PD11_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,GPIO_PIN_RESET)
#define PD11_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11,GPIO_PIN_SET)

#define PD12_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_RESET)
#define PD12_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12,GPIO_PIN_SET)

#define PD13_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_RESET)
#define PD13_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13,GPIO_PIN_SET)

#define PD14_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_RESET)
#define PD14_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,GPIO_PIN_SET)

#define PD15_0  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_RESET)
#define PD15_1  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15,GPIO_PIN_SET)

//---------------------------
#define PE0_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,GPIO_PIN_RESET)
#define PE0_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0,GPIO_PIN_SET)

#define PE1_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_RESET)
#define PE1_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1,GPIO_PIN_SET)

#define PE2_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,GPIO_PIN_RESET)
#define PE2_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2,GPIO_PIN_SET)

#define PE3_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_RESET)
#define PE3_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3,GPIO_PIN_SET)

#define PE4_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,GPIO_PIN_RESET)
#define PE4_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,GPIO_PIN_SET)

#define PE5_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,GPIO_PIN_RESET)
#define PE5_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5,GPIO_PIN_SET)

#define PE6_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,GPIO_PIN_RESET)
#define PE6_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6,GPIO_PIN_SET)

#define PE7_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,GPIO_PIN_RESET)
#define PE7_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7,GPIO_PIN_SET)

#define PE8_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,GPIO_PIN_RESET)
#define PE8_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8,GPIO_PIN_SET)

#define PE9_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_RESET)
#define PE9_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,GPIO_PIN_SET)

#define PE10_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10,GPIO_PIN_RESET)
#define PE10_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10,GPIO_PIN_SET)

#define PE11_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_RESET)
#define PE11_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_SET)

#define PE12_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,GPIO_PIN_RESET)
#define PE12_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,GPIO_PIN_SET)

#define PE13_0  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,GPIO_PIN_RESET)
#define PE13_1  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13,GPIO_PIN_SET)

#define PE14_0 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14,GPIO_PIN_RESET)
#define PE14_1 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14,GPIO_PIN_SET)

#define PE15_0 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,GPIO_PIN_RESET)
#define PE15_1 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15,GPIO_PIN_SET)

//-----

//----------------------

#define RESET_072(a) ((a==1)?PB0_1 : PB0_0)

#define VD3(a) ((a==1)?PC13_1 : PC13_0)
#define VD4(a) ((a==1)?PC14_1 : PC14_0)
#define VD5(a) ((a==1)?PC15_1 : PC15_0)

#define PWDN_4(a) ((a==1)?PC1_1 : PC1_0)
  
#define CS_5_MK(a)     ((a==1)?PE10_1 : PE10_0)
#define CS_SLAVE_5_MK(a) ((a==1)?PE11_1 : PE11_0)
#define CS_2_MK(a)       ((a==1)?PB12_1 : PB12_0)


#define OE_SCLK_SLAVE_5_MK(a)   ((a==1)?PD10_1 : PD10_0)
#define UPR_HDS_MK(a)       ((a==1)?PD12_1 : PD12_0)
#define ENABLE_LM25056_MK(a)    ((a==1)?PD14_1 : PD14_0)
#define RESET_TCA6424A_MK(a)    ((a==1)?PD15_1 : PD15_0)
#define SMBA_LM25056_MK(a)    ((a==1)?PC6_1 : PC6_0)

#define POK_HDS_OUT         HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_9)
#define INT_TCA6424A_MK     HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7)
#define FLAG_MIC29151       HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_8)

#define CS_EPCS1(a)   ((a==1)?PD0_1 : PD0_0)
#define CS_EPCS2(a)   ((a==1)?PD1_1 : PD1_0)

#define DE_RS485(a)   ((a==1)?PD4_1 : PD4_0)
#define NRE_RS485(a)    ((a==1)?PD7_1 : PD7_0)

#define WDI_MK(a)     ((a==1)?PB9_1 : PB9_0)
#define RES_4(a)      ((a==1)?PE0_1 : PE0_0)

#define NSS_4(a)        ((a==1)?PE4_1 : PE4_0)

//------------------------------------------------
//                 JTAG

#define TDO()  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)
#define TDI(a)  ((a==1)?PB15_1 : PB15_0)
#define TCK(a)  ((a==1)?PB13_1 : PB13_0)
#define TMS(a)  ((a==1)?PB12_1 : PB12_0)

#define ALIGN(x, n) (((x) + (n) - 1) & ~((n) - 1))
#undef MIN
#define MIN(x, y)  (((x) < (y)) ? (x) : (y))
#undef MAX
#define MAX(x, y)  (((x) > (y)) ? (x) : (y))

//typedef unsigned int bool;


#define ssize_t long
#define mode_t unsigned int

//------------------------------------------------

#define u64 unsigned long long
#define u32 unsigned int
#define u16 unsigned short
#define u8  uint8_t

//---------------------------------------------------------------------

// USART1 Receiver buffer
#define RX_BUFFER_SIZE1 64u
#define Bufer_size   8192u   //16384

// USART2 Receiver buffer
#define RX_BUFFER_SIZE2 64u
#define Bufer_size2   1024u   //16384

#define buf_IO   32u 
#define buf_Word 32u 
#define buf_DATA_Word 200u 
#define BUFFER_SR 200u
#define BUF_STR 64
#define MAX_PL 157u

//------------------------------------------------

/* typedef struct     //структура ответной квитанции
{
  u32 Cmd_size;   //размер данных
  u32 Cmd_type;   //тип квитанции
  u32 Cmd_id;     //ID квитанции
  u32 Cmd_time;   //время формирования квитанции (сразу после выполнения команды)
  u8  Cmd_data[32]; //данные квитанции - 0-выполненна , 1 - не выполненна
  u32 N_sch;      //число подготовленных квитанций
}INVOICE;
 */


#define MSG_REPLY       1
#define MSG_ERROR       2
#define ERROR_CMD_BUF   1 //были затёртые команды в буфере
#define MSG_CMD_OK      3 //команда выполненна успешно
#define MSG_STATUS_OK 100 //Квитация на статус

#define MSG_ID_CH1    101 //сообщаем ID миксросхемы LM в канале 1
#define MSG_ID_CH2    102 //
#define MSG_ID_CH3    103 //
#define MSG_ID_CH4    104 //
#define MSG_ID_CH5    105 //
#define MSG_ID_CH6    106 //
#define MSG_ID_CH7    107 //
#define MSG_ID_CH8    108 //

#define MSG_TEMP_CH1    111 //сообщаем температуру миксросхемы LM в канале 1
#define MSG_TEMP_CH2    112 //
#define MSG_TEMP_CH3    113 //
#define MSG_TEMP_CH4    114 //
#define MSG_TEMP_CH5    115 //
#define MSG_TEMP_CH6    116 //
#define MSG_TEMP_CH7    117 //
#define MSG_TEMP_CH8    118 //

#define MSG_U_CH1     121 //сообщаем напряжение миксросхемы LM в канале 1
#define MSG_U_CH2     122 //
#define MSG_U_CH3     123 //
#define MSG_U_CH4     124 //
#define MSG_U_CH5     125 //
#define MSG_U_CH6     126 //
#define MSG_U_CH7     127 //
#define MSG_U_CH8     128 //

#define MSG_I_CH1     131 //сообщаем напряжение миксросхемы LM в канале 1
#define MSG_I_CH2     132 //
#define MSG_I_CH3     133 //
#define MSG_I_CH4     134 //
#define MSG_I_CH5     135 //
#define MSG_I_CH6     136 //
#define MSG_I_CH7     137 //
#define MSG_I_CH8     138 //

#define MSG_P_CH1     141 //сообщаем напряжение миксросхемы LM в канале 1
#define MSG_P_CH2     142 //
#define MSG_P_CH3     143 //
#define MSG_P_CH4     144 //
#define MSG_P_CH5     145 //
#define MSG_P_CH6     146 //
#define MSG_P_CH7     147 //
#define MSG_P_CH8     148 //

#define MSG_PWR_CHANNEL   150 //сообщаем состояние линий питания каналов
#define MSG_REQ_VERSION   109 //сообщаем 	номер версии
#define MSG_REQ_NUM_SLAVE 110 //сообщаем  количество слейвов в блоке и их адреса
#define MSG_REQ_TEST_485  111 //сообщаем  результат теста проверки шины 485
#define MSG_REQ_TEST_SPI  112 //сообщаем  результат теста проверки шины SPI
#define MSG_REQ_TEST_JTAG 113 //сообщаем  результат теста проверки шины JTAG

//---------команды управления---------------------------------------------
#define CMD_TIME_SETUP       1   //команда установки точного времени, реалтайм.
#define CMD_HELP             2   //вывести в консоль HELP()
#define CMD_TIME             0   //вывести в консоль текущее время
#define CMD_12V              3   //включить - 1 , выключить - 0 питание +12 Вольт
#define CMD_STATUS         100 //сообщите состояние
#define CMD_LED            200 //команда управления светодиодами лицевой панели
#define CMD_xxx            300
#define CMD_CH_UP            4   //команда включения/выключения каналов питания , канал в данных передаётся (вкл/выкл - инверсные коды)
#define CMD_SETUP_IP0        5   //команда установки IP0 адреса блока 072
#define CMD_SETUP_IP1        6   //команда установки IP0 адреса блока 072
#define CMD_SETUP_DEST_IP0   7   //команда установки DEST_IP0 адреса блока 072
#define CMD_SETUP_DEST_IP1   8   //команда установки DEST_IP1 адреса блока 072
#define CMD_REQ_VERSION      9   //команда запроса номера версии ПО
#define CMD_REQ_NUM_SLAVE   10   //команда запроса количества блоков 072 в АЦ и их адресов
#define CMD_TEST_485        11   //команда начала теста проверки шины 485
#define CMD_TEST_SPI        12   //команда начала теста проверки шины SPI
#define CMD_TEST_JTAG       13   //команда начала теста проверки шины JTAG
//------------------------------------------------------------------------
 
#define SIZE_SERVER   4096//размер буфера "Хранилище"  тут хранятся данные команды пришедших пакетов сами команды хранятся в реестре
#define SIZE_ID        32 //размер реестра  
#define quantity_CMD   64 //максимальное количество команд 
#define quantity_DATA  16 //максимальная длинна данных
#define quantity_SENDER 2 //максимальное количество адресатов

//-----------------------------------------------------------------
typedef struct    //структура параметров LM
{
  u32 TEMP;   //температура
  u32 P;      //измеренная мощность
  u32 I;      //измеренный ток
  u32 U;      //измеренное напряжение
  u8 ID[3];   //ID микросхемы
  u32 P_max;    //максимально допустимая потребляемая мощность
  u32 TEMP_max; //максимально допустимая температура
  u32 U_min;    //минимально допустимое напряжение
}LM_struct;

//----------------------------------------------------------------- 
typedef struct //структура команды
{
  u32 Cmd_size;
  u32 Cmd_type;
  u64 Cmd_id;
  u64 Cmd_time;
  u8  Cmd_data[quantity_DATA];
}Command;

typedef struct //структура сообщения с описанием команд
{
  u32 Msg_size;
  u32 Msg_type;
  u64 Num_cmd_in_msg;
    Command CMD[quantity_CMD];
}Message;

typedef struct //структура фрейма
{
  u16 Frame_size;
  u16 Frame_number;
  u8  Stop_bit;
  u32 Msg_uniq_id;
  u64 Sender_id;
  u64 Receiver_id;
    Message MSG;    
}Frame;

typedef struct //структура "Хранилище"
{
  u8 MeM[SIZE_SERVER];
  u32 INDEX;
  u32 INDEX_LAST;
  u64 CMD_ID;
  u64 SENDER_ID;
  u64 TIME;
  u32 x1;//начало диапазона удалённых индексов
  u32 x2;//конец  диапазона удалённых индексов
}SERVER;


typedef struct //структура "реестр" (ID)
{
  u32 INDEX       [SIZE_ID];
  u64 CMD_TYPE    [SIZE_ID];
  u64 CMD_ID      [SIZE_ID];
  u64 SENDER_ID   [SIZE_ID];
  u64 TIME        [SIZE_ID];
  u8  FLAG_REAL_TIME[SIZE_ID]; //флаг реального времени
  u32 N_sch;//число записей в структуре
}ID_SERVER;

typedef struct   //структура команды на исполнение
{
  u32 INDEX;  //индекс команды в структуре SERVER 
  u64 TIME; //время исполнения
}CMD_RUN;

typedef struct
{
  u32 IDX;        //подсчитанное количество отправителей
  u64 A[quantity_SENDER]; //масств адресов ОТПРАВ�
}ADR_SENDER;
//---------------------------
typedef struct  //эта структура описывает указатель для отложенных команд
{ 
  u8  FLAG;
  u32 timer;
} POINTER;

#define PNT_BUF 11
//---------------------------

u32 IO ( char* );
void test_delay (u32 );
u8 PIN_control_PA8 (void);
u8 PIN_control (void);
void ATT_upr (u8 ,u8 );
void FAPCH_INIT (void);
void LED (void);
void SDRAM_test_wr (u32,u16);
u16 SDRAM_test_rd (u32 ,u16); 
void test3_sdram(u16 );
void test2_SDRAM(u16 );
void test_SDRAM (u16 );
char getchar1(void);
void Menu1(char );
u64 FPGA_rSPI (u8,u8);
u64 FPGA2_rSPI(u8,u8);
u32 FPGA_wSPI (u8,u8,u64);
u32 FPGA2_wSPI(u8,u8,u64);
void spisend_FPGA (u8,u8);
u8 spisend8 (u8);
void spisend32 (u32);
void h_out (u64,u8);
void hn_out(u64,u8);
void i_out (u64,u8);
void in_out(u64,u8);
void xn_out (char *,u32);
void x_out (char *,u32);
void x32_out (char *,u32);
void nu_out (char *,u32);
void un_out (char *,u32);
void u_out (char *,u32);
void d_out (char *,int);
void f_out (char *,float);
void Transf(const char* );
unsigned int leng ( char *);
void itoa(int ,  char *, int);
volatile void delay_us( uint32_t );
void Delay( unsigned int ) ;
void ADC_test (void);
void PWM (u16);
void spi4send32 (u32 );
u8 spi4send8 (u8 );
void FRAME_DECODE (uint8 *,u32 );
void INIT_SERV_ARCHIV (SERVER *,ID_SERVER *,ADR_SENDER *);
void SERV_WR (u8 ,SERVER *);
void PRINT_SERV (void);
void PRINT_SERV_ID (void);
void STATUS_ID (ID_SERVER *);
u32 ADR_FINDER (u64 ,ADR_SENDER *); //ищем адрес отправителя в структуре адресов (его порядковый номер)
void Set_network(void);
void MSG_SHOW (void);
u32 SEND_UDP_MSG (void);
u8 DS4520_read (void);
void SETUP_IP0_072 (u8 ,u32);
void SETUP_IP1_072 (u8 ,u32);
void REQ_VERSIYA (void);
void answer_translated (u32 );
void req_col ();
void FUNC_FLAG_UP (POINTER *,u32);

//-------------JTAG--------------
void JTAG_SCAN (void);
u8 SCAN_N (void);
void ID_recive (u8 );
//-------------------------------


u32 ERROR_CMD_MSG //формирует структуру квитанции уровня CMD
(
ID_SERVER *,  //указатель на структуру "реестр" 
Frame *,    //указатель на структуру квитанции
u32 ,     //индекс в реестре
u32 ,     //тип сообщения
u32 ,     //данные сообщения
u32       //время составления квитанции
);

u32 SERV_ID_WR  //функция заполнения реестра команд
(
Frame  *,    //указатель на структуру квитанции
SERVER *,        //указатель на структуру "Хранилище"
ID_SERVER *,   //указатель на структуру "реестр"
u32 ,      //индекс в Хранилище
u32 ,      //CMD_ID
u32 ,      //SENDER_ID
u64 ,      //TIME
u64        //CMD_ID
); 

void SERV_ID_DEL //процедура удаления команды из реестра
(
ID_SERVER *,     //указатель на структуру "реестр"
u32        //индекс команды в реестре
);

u32 TX_MSG_BUFF (//заполняем транспортный массив
Frame *,      //структура квитанции
uint8 *,      //транспортный массив
u32         //максимальный размер транспортного массива 
);

u32 SYS_CMD_MSG(
ID_SERVER *,  //реестр
Frame *,    //структура квитанций 
u32 ,     //индекс в реестре
u32 ,     //тип сообщения
u32 ,     //объём данных сообщения в байтах
u8 *,     //данные сообщения - массив данных
u32         //время составления квитанции
);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
