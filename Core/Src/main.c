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
#include "string.h"

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
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim23;
TIM_HandleTypeDef htim24;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
/*initialize*/
int ini = 1;
int ini_1 = 1;
int ini_2 = 1;
int ini_3 = 1;
/*Show off*/
int Demo_1 = 0;
int Demo_2 = 0;
int Demo_3 = 0;
int Cube = 100;
int step = 1;

int down_p = 0;
int down_n = 0;
int mid_p = 0;
int mid_n = 0;
int up_p = 0;
int up_n = 0;
int rotate_p = 0;
int rotate_n = 0;
int grab = 0;
int loose = 0;
/*Reset*/
int Reset = 0;
/*All at Once*/
double Once = -0.0;
double TEMP = 0.0;
/*Switch*/
int Switch = 1;
/*Stepper*/
int done_1 = 0;
int done_2 = 0;
int done_3 = 0;
int deceleration_1 = 0;
double temp_target_1 = 0.0;
int directioned_1 = 1;
double target_1 = 0.0;
double target_2 = 0.0;
double target_3 = 0.0;
double targeted_1 = 0.0;
double targeted_2 = 0.0;
double targeted_3 = 0.0;
double position_1 = 0.0;
double position_2 = 0.0;
double position_3 = 0.0;
double angle_1 = 0.0;
double angle_2 = 0.0;
double angle_3 = 0.0;
double ratio_1 = 27.0;
double ratio_2 = 27.0;
double ratio_3 = 4.658;
int direction_1 = 1;
int direction_2 = 1;
int direction_3 = 1;
double relate_1 = 0.0;
double relate_2 = 0.0;
double relate_3 = 0.0;
int i = 0;
int j = 0;
int k = 0;
int max_speed_1 = 199;
int max_speed_2 = 299;
int max_speed_3 = 999;
int initial_speed = 1999;
int ARR_1 = 1999;
int ARR_2 = 1999;
int ARR_3 = 1999;
int CNT_1 = 0;
int CNT_2 = 0;
int CNT_3 = 0;
int error_1 = 1000;
int error_2 = 1000;
int error_3 = 1000;
double percent = 0.25;
int x = 5;
double y = 12;
/*Servo*/
double degree_1 = 0.0;
double degree_2 = 0.0;
double pulse_1 = 0.0;
double pulse_2 = 0.0;
double Deg_1 = 0.0;
double Deg_2 = 0.0;
int a = 0;
int b = 0;
double temp_1 = 0.0;
double temp_2 = 0.0;
/*int grab = 0;*/
/*tighten duration*/
/*int Time_a = 0;
int Time_b = 0;*/
/*relocation*/
/*int Time_A = 0;
int Time_B = 0;
int reset_a = 1;
int reset_b = 1;
int stop_1 = 0;
int stop_2 = 0;*/
/*pressure time*/
/*int Time_1 = 0;
int Time_2 = 0;*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_HS_USB_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM23_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM24_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM12_Init(void);
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_HS_USB_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM23_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM24_Init();
  MX_DMA_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim12);
  HAL_TIM_Base_Start_IT(&htim15);
  HAL_TIM_Base_Start_IT(&htim24);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim23, TIM_CHANNEL_1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/*Button*/
	/*if(down_p == 1)
	{
		target_1++;
		if(target_1>180)
			target_1 = 180;
	}
	if(down_n == 1)
	{
		target_1--;
		if(target_1<0)
			target_1 = 0;
	}
	if(mid_p == 1)
	{
		target_2++;
		if(target_2>180)
			target_2 = 180;
	}
	if(mid_n == 1)
	{
		target_2--;
		if(target_2<0)
			target_2 = 0;
	}
	if(up_p == 1)
	{
		target_3++;
		if(target_3>180)
			target_3 = 180;
	}
	if(up_n == 1)
	{
		target_3--;
		if(target_3<0)
			target_3 = 0;
	}
	if(rotate_p == 1)
	{
		degree_1++;
		if(degree_1>90)
			target_3 = 90;
	}
	if(rotate_n == 1)
	{
		degree_1--;
		if(degree_1<-90)
			target_3 = -90;
	}*/
	if(grab == 1)
	{
		degree_2 = Cube;
		grab = 0;
	}
	if(loose == 1)
	{
		degree_2 = 0;
		loose = 0;
	}
	/*Demo1*/
	/*switch(Demo_1)
	{
		case 1:
			if(ini)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				ini = 0;
			}
			target_1 = 0;
			target_2 = -138;
			target_3 = -55;
			if(done_2 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_1 = 2;
				}
			}
			break;
		case 2:
			HAL_Delay(50);
			degree_2 = 90;
			HAL_Delay(1500);
			Demo_1 = 3;
			break;
		case 3:
			HAL_Delay(50);
			target_1 = 90;
			target_2 = -50;
			if(done_1 == 1 && done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_1 = 4;
			}
			break;
		case 4:
			HAL_Delay(50);
			target_2 = -115;
			target_3 = -45;
			if(done_2 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_1 = 5;
				}
			}
			break;
		case 5:
			HAL_Delay(50);
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_1 = 6;
			break;
		case 6:
			HAL_Delay(50);
			target_2 = -95;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_1 = 7;
			}
			break;
		case 7:
			HAL_Delay(50);
			target_1 = 45;
			if(done_1 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_1 = 8;
			}
			break;
		case 8:
			HAL_Delay(50);
			target_2 = -138;
			target_3 = -55;
			if(done_2 == 1 && done_3 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_1 = 9;
			}
			break;
		case 9:
			HAL_Delay(50);
			degree_2 = 90;
			HAL_Delay(1500);
			Demo_1 = 10;
			break;
		case 10:
			HAL_Delay(50);
			target_2 = -95;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_1 = 11;
			}
			break;
		case 11:
			HAL_Delay(50);
			degree_1 = 90;
			HAL_Delay(1500);
			Demo_1 = 12;
			break;
		case 12:
			HAL_Delay(50);
			target_1 = 90;
			degree_1 = 0;
			if(done_1 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_1 = 13;
			}
			break;
		case 13:
			HAL_Delay(50);
			target_2 = -90;
			target_3 = -35;
			if(done_2 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_1 = 14;
				}
			}
			break;
		case 14:
			HAL_Delay(50);
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_1 = 15;
			break;
		case 15:			// OLD
			HAL_Delay(50);
			target_2 = -25;
			target_3 = -45;
			if(done_2 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_1 = 16;
				}
			}
			break;
		case 16:
			HAL_Delay(50);
			target_1 = 0;
			target_2 = 0;
			target_3 = 0;
			if(done_1 == 1 && done_2 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_1 = 0;
				}
			}
			break;
	}*/
	/*Demo2*/
	/*switch(Demo_2)
	{
		case 1:
			if(ini)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				ini = 0;
			}
			target_1 = -188;
			target_2 = -170;
			target_3 = -75;
			if(done_1 == 1 && done_2 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_2 = 2;
				}
			}
			break;
		case 2:
			HAL_Delay(50);
			degree_2 = Cube;
			HAL_Delay(1500);
			Demo_2 = 3;
			break;
		case 3:
			HAL_Delay(50);
			target_2 = -140;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 4;
			}
			break;
		case 4:
			HAL_Delay(50);
			target_1 = -148;
			HAL_Delay(1000);
			target_2 = -165;
			target_3 = -85;
			if(done_1 == 1 && done_2 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_2 = 5;
				}
			}
			break;
		case 5:
			HAL_Delay(50);
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_2 = 6;
			break;
		case 6:
			HAL_Delay(50);
			target_2 = -155;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 7;
			}
			break;
		case 7:
			HAL_Delay(50);
			target_1 = -170;
			target_3 = -110;
			if(done_1 == 1 && done_3 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 8;
			}
			break;
		case 8:
			HAL_Delay(50);
			target_2 = -177;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 9;
			}
			break;
		case 9:
			HAL_Delay(50);
			degree_2 = Cube;
			HAL_Delay(1500);
			Demo_2 = 10;
			break;
		case 10:
			HAL_Delay(50);
			target_2 = -158;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 11;
			}
			break;
		case 11:
			HAL_Delay(50);
			target_1 = -149;
			target_2 = -147;
			target_3 = -80;
			if(done_1 == 1 && done_2 == 1 && done_3 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 12;
			}
			break;
		case 12:
			HAL_Delay(50);
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_2 = 13;
			break;
		case 13:
			HAL_Delay(50);
			target_2 = -137;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 14;
			}
			break;
		case 14:
			HAL_Delay(50);
			degree_1 = -15;
			HAL_Delay(1500);
			Demo_2 = 15;
			break;
		case 15:
			HAL_Delay(50);
			target_1 = -199;
			target_3 = -117;
			if(done_1 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_2 = 16;
				}
			}
			break;
		case 16:
			HAL_Delay(50);
			target_2 = -179;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 17;
			}
			break;
		case 17:
			HAL_Delay(50);
			degree_2 = Cube;
			HAL_Delay(1500);
			Demo_2 = 18;
			break;
		case 18:
			HAL_Delay(50);
			target_2 = -129;
			if(done_2 == 1)
			{
				done_1 = 0;
				done_2 = 0;
				done_3 = 0;
				Demo_2 = 19;
			}
			break;
		case 19:
			HAL_Delay(50);
			target_1 = -149;
			target_3 = -72;
			if(done_1 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_2 = 20;
				}
			}
			break;
		case 20:
			HAL_Delay(50);
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_2 = 21;
			break;
		case 21:
			HAL_Delay(50);
			target_2 = 0;
			HAL_Delay(750);
			target_1 = 0;
			target_3 = 0;
			degree_1 = 0;
			if(done_1 == 1 && done_2 == 1)
			{
				if(done_3 == 1)
				{
					done_1 = 0;
					done_2 = 0;
					done_3 = 0;
					Demo_2 = 0;
				}
			}
			break;
	}*/
	/*Demo3*/
	switch(Demo_3)
	{
		case 1:
			degree_2 = 100;
			HAL_Delay(1500);
			Demo_3 = 2;
			break;
		case 2:
			target_3 = -50;
			if(position_3 == -50)
				Demo_3 = 3;
			break;
		case 3:
			target_2 = -165;
			if(position_2 == -165)
				Demo_3 = 4;
			break;
		case 4:
			target_3 = -80;
			if(position_3 == -80)
				Demo_3 = 5;
			break;
		case 5:
			target_3 = -55;
			if(position_3 == -55)
				Demo_3 = 6;
			break;
		case 6:
			target_1 = -25;
			if(position_1 == -25)
				Demo_3 = 7;
			break;
		case 7:
			degree_1 = -25;
			HAL_Delay(1500);
			Demo_3 = 8;
			break;
		case 8:
			target_3 = -90;
			if(position_3 == -90)
				Demo_3 = 9;
			break;
		case 9:
			target_2 = -155;
			if(position_2 == -155)
				Demo_3 = 10;
			break;
		case 10:
			degree_1 = 0;
			HAL_Delay(1500);
			Demo_3 = 11;
			break;
		case 11:
			target_1 = 0;
			if(position_1 == 0)
				Demo_3 = 12;
			break;
		case 12:
			degree_2 = 50;
			HAL_Delay(1500);
			Demo_3 = 13;
			break;
		case 13:
			target_3 = -115;
			if(position_3 == -115)
				Demo_3 = 14;
			break;
		case 14:
			target_2 = -175;
			if(position_2 == -175)
				Demo_3 = 15;
			break;
		case 15:
			target_3 = -100;
			if(position_3 == -100)
				Demo_3 = 16;
			break;
		case 16:
			target_2 = -155;
			if(position_2 == -155)
				Demo_3 = 17;
			break;
		case 17:
			target_1 = -25;
			if(position_1 == -25)
				Demo_3 = 18;
			break;
		case 18:
			target_3 = -125;
			degree_1 = 25;
			if(position_3 == -125)
				Demo_3 = 19;
			break;
		case 19:
			target_1 = -20;
			if(position_1 == -20)
				Demo_3 = 20;
			break;
		case 20:
			degree_2 = 50;
			HAL_Delay(1500);
			Demo_3 = 21;
			break;
		case 21:
			target_2 = -180;
			if(position_2 == -180)
				Demo_3 = 22;
			break;
		case 22:
			degree_1 = 0;
			HAL_Delay(1500);
			Demo_3 = 23;
			break;
		case 23:
			target_3 = -90;
			if(position_3 == -90)
				Demo_3 = 24;
			break;
		case 24:
			target_2 = -160;
			if(position_2 == -160)
				Demo_3 = 25;
			break;
		case 25:
			target_1 = -40;
			if(position_1 == -40)
				Demo_3 = 26;
			break;
		case 26:
			degree_1 = -35;
			HAL_Delay(1500);
			Demo_3 = 27;
			break;
		case 27:
			target_2 = -170;
			degree_1 = -10;
			if(position_2 == -170)
				Demo_3 = 28;
			break;
		case 28:
			target_1 = -25;
			if(position_1 == -25)
				Demo_3 = 29;
			break;
		case 29:

			target_2 = -160;
			if(position_2 == -160)
				Demo_3 = 30;
			break;
		case 30:
			target_1 = 35;
			degree_1 = 35;
			if(position_1 == 35)
				Demo_3 = 31;
			break;
		case 31:
			target_2 = -170;
			if(position_2 == -170)
				Demo_3 = 32;
			break;
		case 32:
			target_1 = 20;
			if(position_1 == 20)
				Demo_3 = 33;
			break;
		case 33:
			target_2 = -155;
			if(position_2 == -155)
				Demo_3 = 34;
			break;
		case 34:
			degree_1 = 90;
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_3 = 35;
			break;
		case 35:
			target_1 = -8;
			if(position_1 == -8)
				Demo_3 = 36;
			break;
		case 36:
			target_3 = -75;
			if(position_3 == -75)
				Demo_3 = 37;
			break;
		case 37:
			target_2 = -170;
			if(position_2 == -170)
				Demo_3 = 38;
			break;
		case 38:
			degree_2 = 100;
			HAL_Delay(1500);
			Demo_3 = 39;
			break;
		case 39:
			target_2 = -130;
			if(position_2 == -130)
				Demo_3 = 40;
			break;
		case 40:
			target_1 = 30;
			if(position_1 == 30)
				Demo_3 = 41;
			break;
		case 41:
			target_3 = -95;
			if(position_3 == -95)
				Demo_3 = 42;
			break;
		case 42:
			target_2 = -170;
			if(position_2 == -170)
				Demo_3 = 43;
			break;
		case 43:
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_3 = 44;
			break;
		case 44:
			target_2 = -150;
			if(position_2 == -150)
				Demo_3 = 45;
			break;
		case 45:
			target_1 = 5;
			if(position_1 == 5)
				Demo_3 = 46;
			break;
		case 46:
			degree_1 = 0;
			HAL_Delay(1500);
			Demo_3 = 47;
			break;
		case 47:
			target_3 = -90;
			if(position_3 == -90)
				Demo_3 = 48;
			break;
		case 48:
			target_2 = -170;
			if(position_2 == -170)
				Demo_3 = 49;
			break;
		case 49:
			degree_2 = 100;
			HAL_Delay(1500);
			Demo_3 = 50;
			break;
		case 50:
			target_2 = -145;
			if(position_2 == -145)
				Demo_3 = 51;
			break;
		case 51:
			target_1 = 30;
			if(position_1 == 30)
				Demo_3 = 52;
			break;
		case 52:
			target_3 = -80;
			if(position_3 == -80)
				Demo_3 = 53;
			break;
		case 53:
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_3 = 54;
			break;
		case 54:
			target_2 = -135;
			if(position_2 == -135)
				Demo_3 = 55;
			break;
		case 55:
			target_1 = -20;
			if(position_1 == -20)
				Demo_3 = 56;
			break;
		case 56:
			degree_1 = -150;
			HAL_Delay(1500);
			Demo_3 = 57;
			break;
		case 57:
			target_3 = -100;
			if(position_3 == -100)
				Demo_3 = 58;
			break;
		case 58:
			target_2 = -170;
			if(position_2 == -170)
				Demo_3 = 59;
			break;
		case 59:
			degree_2 = 100;
			HAL_Delay(1500);
			Demo_3 = 60;
			break;
		case 60:
			target_2 = -130;
			if(position_2 == -130)
				Demo_3 = 61;
			break;
		case 61:
			target_1 = 30;
			degree_1 = 0;
			if(position_1 == 30)
				Demo_3 = 62;
			break;
		case 62:
			target_3 = -70;
			if(position_3 == -70)
				Demo_3 = 63;
			break;
		case 63:
			degree_2 = 0;
			HAL_Delay(1500);
			Demo_3 = 64;
			break;
		case 64:
			target_2 = 0;
			target_3 = 0;
			HAL_Delay(5000);
			target_1 = 0;
			break;
	}

	/*Done*/
	if(target_1 == position_1)
	{
		/*if(ini_1 == 1)
		{*/
			done_1 = 1;
			/*ini_1 = 0;
		}*/
	}
	/*else
	{
		done_1 = 0;
		ini_1 = 1;
	}*/
	if(target_2 == position_2)
	{
		/*if(ini_2 == 1)
		{*/
			done_2 = 1;
			/*ini_2 = 0;
		}*/
	}
	/*else
	{
		done_2 = 0;
		ini_2 = 1;
	}*/
	if(target_3 == position_3)
	{
		/*if(ini_3 == 1)
		{*/
			done_3 = 1;
			/*ini_3 = 0;
		}*/
	}
	/*else
	{
		done_3 = 0;
		ini_3 = 1;
	}*/

	/*Limit*/
	if(target_2 < -200)
	{
		target_2 = -200;
	}
	else if(target_2 > 0)
	{
		target_2 = 0;
	}

	if(target_3 < -270)
	{
		target_3 = -270;
	}
	else if(target_3 > 0)
	{
		target_3 = 0;
	}

	/*Reset*/
	if(Reset == 1)
	{
		HAL_NVIC_SystemReset();
		Reset = 0;
	}

	/*Relative target*/
	target_1 += relate_1;
	relate_1 = 0;
	target_2 += relate_2;
	relate_2 = 0;
	target_3 += relate_3;
	relate_3 = 0;

	/*All at Once*/
	if(Once != TEMP )
	{
		target_1 = Once;
		target_2 = Once;
		target_3 = Once;
		if(Once <= 180 && Once >=0)
		{
			degree_1 = 0;
			degree_2 = 0;
		}
	}

	TEMP = Once;

	/*Enable*/
	HAL_GPIO_WritePin(GPIOG,GPIO_PIN_2,Switch);


	/*stepper_1*/
	/*Record position & reset i (Considering the circumstances which target changed while operating)*/
	if(i >= (fabs(targeted_1-position_1)/1.8*8*2*ratio_1) || target_1 != targeted_1)
	{
		/*if(deceleration_1 == 1)
		{
			target_1 = temp_target_1;
			deceleration_1 = 0;
		}*/
		if(direction_1 == 1)
			position_1 += round(1.8/8/2/ratio_1*i);
		else if(direction_1 == 0)
			position_1 -= round(1.8/8/2/ratio_1*i);
		/*if(directioned_1 != direction_1)
		{
			if(ARR_1<initial_speed)
			{
				ARR_1+=10;
			}
			__HAL_TIM_SET_AUTORELOAD(&htim3,ARR_1);
			deceleration_1 = 1;
		}*/
		i = 0;
	}
	/*Record target & direction*/
	targeted_1 = target_1;
	/*if((target_1-position_1)>=0)
	{
		directioned_1 = 1;
	}
	else
	{
		directioned_1 = 0;
	}*/
	/*DIR*/
	if((targeted_1-position_1)>=0)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_SET);
		direction_1 = 1;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
		direction_1 = 0;
	}
	/*Record target*/
	targeted_1 = target_1;
	/*angle*/
	angle_1 = targeted_1-position_1;

	/*stepper_2*/
	/*Record position & reset j (Considering the circumstances which target changed while operating)*/
	if(j >= (fabs(targeted_2-position_2)/1.8*8*2*ratio_2) || target_2 != targeted_2)
	{
		if(direction_2 == 1)
			position_2 += round(1.8/8/2/ratio_2*j);
		else if(direction_2 == 0)
			position_2 -= round(1.8/8/2/ratio_2*j);
		j = 0;
	}
	/*Record target*/
	targeted_2 = target_2;
	/*DIR*/
	if((targeted_2-position_2)>=0)
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_SET);
		direction_2 = 1;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_11,GPIO_PIN_RESET);
		direction_2 = 0;
	}
	/*Record target*/
	targeted_2 = target_2;
	/*angle*/
	angle_2 = targeted_2-position_2;

	/*stepper_3*/
	/*Record position & reset k (Considering the circumstances which target changed while operating)*/
	if(k >= (fabs(targeted_3-position_3)/1.8*8*2*ratio_3) || target_3 != targeted_3)
	{
		if(direction_3 == 1)
			position_3 += round(1.8/8/2/ratio_3*k);
		else if(direction_3 == 0)
			position_3 -= round(1.8/8/2/ratio_3*k);
		k = 0;
	}
	/*Record target*/
	targeted_3 = target_3;
	/*DIR*/
	if((targeted_3-position_3)>=0)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_RESET);
		direction_3 = 1;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_2,GPIO_PIN_SET);
		direction_3 = 0;
	}
	/*Record target*/
	targeted_3 = target_3;
	/*angle*/
	angle_3 = targeted_3-position_3;

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 68;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 6144;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  htim1.Init.Prescaler = 274;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 274;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 274;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 274;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 274;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 274;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 274;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 49999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 274;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM23 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM23_Init(void)
{

  /* USER CODE BEGIN TIM23_Init 0 */

  /* USER CODE END TIM23_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM23_Init 1 */

  /* USER CODE END TIM23_Init 1 */
  htim23.Instance = TIM23;
  htim23.Init.Prescaler = 274;
  htim23.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim23.Init.Period = 19999;
  htim23.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim23.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim23) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim23, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim23, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM23_Init 2 */

  /* USER CODE END TIM23_Init 2 */
  HAL_TIM_MspPostInit(&htim23);

}

/**
  * @brief TIM24 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM24_Init(void)
{

  /* USER CODE BEGIN TIM24_Init 0 */

  /* USER CODE END TIM24_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM24_Init 1 */

  /* USER CODE END TIM24_Init 1 */
  htim24.Instance = TIM24;
  htim24.Init.Prescaler = 274;
  htim24.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim24.Init.Period = 999;
  htim24.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim24.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim24) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim24, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim24, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM24_Init 2 */

  /* USER CODE END TIM24_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_HS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_HS_USB_Init(void)
{

  /* USER CODE BEGIN USB_OTG_HS_Init 0 */

  /* USER CODE END USB_OTG_HS_Init 0 */

  /* USER CODE BEGIN USB_OTG_HS_Init 1 */

  /* USER CODE END USB_OTG_HS_Init 1 */
  /* USER CODE BEGIN USB_OTG_HS_Init 2 */

  /* USER CODE END USB_OTG_HS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_FS_PWR_EN_Pin|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_FS_PWR_EN_Pin PD11 PD12 PD13
                           PD2 */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PG2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_VBUS_Pin */
  GPIO_InitStruct.Pin = USB_FS_VBUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/*void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == GPIO_PIN_6)
	{
		HAL_NVIC_SystemReset();
	}
}*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*Stepper step control*/
	/*Stepper_1*/
	if(htim->Instance == TIM3)
	{
		/*STEP*/
		if(i < (fabs(angle_1)/1.8*8*2*ratio_1))
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_8);
			i++;
		}
	}
	/*Stepper_2*/
	if(htim->Instance == TIM15)
	{
		/*STEP*/
		if(j < (fabs(angle_2)/1.8*8*2*ratio_2))
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_10);
			j++;
		}
	}
	/*Stepper_3*/
	if(htim->Instance == TIM1)
	{
		/*STEP*/
		if(k < (fabs(angle_3)/1.8*8*2*ratio_3))
		{
			HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_12);
			k++;
		}
	}
	/*Stepper velocity control*/
	/*Grab*/
	if(htim->Instance == TIM4)
	{
		/*Stepper_1*/
		if(i < (fabs(target_1-position_1)/1.8*8*2*ratio_1)*percent)
 		{
			if(ARR_1>max_speed_1)
			{
				ARR_1-=x;
				if(ARR_1<=max_speed_1)
				{
					CNT_1 = i;
				}
			}
			__HAL_TIM_SET_AUTORELOAD(&htim3,ARR_1);
		}
		if(i > (fabs(target_1-position_1)/1.8*8*2*ratio_1)-(CNT_1+error_1) && (target_1-position_1)!=0)
		{
			if(ARR_1<initial_speed)
			{
				ARR_1+=x;
				if(ARR_1>=initial_speed)
				{
					CNT_1 = 0;
				}
			}
			__HAL_TIM_SET_AUTORELOAD(&htim3,ARR_1);
		}
	}
	if(htim->Instance == TIM5)
	{
		/*Stepper_2*/
		if(j < (fabs(target_2-position_2)/1.8*8*2*ratio_2)*percent)
		{
			if(ARR_2>max_speed_2)
			{
				ARR_2-=x;
				if(ARR_2<=max_speed_2)
				{
					CNT_2 = j;
				}
			}
			__HAL_TIM_SET_AUTORELOAD(&htim15,ARR_2);
		}
		if(j > (fabs(target_2-position_2)/1.8*8*2*ratio_2)-(CNT_2+error_2) && (target_2-position_2)!=0)
		{
			if(ARR_2<initial_speed)
			{
				ARR_2+=x;
				if(ARR_2>=initial_speed)
				{
					CNT_2 = 0;
				}
			}
			__HAL_TIM_SET_AUTORELOAD(&htim15,ARR_2);
		}
	}
	if(htim->Instance == TIM24)
	{
		/*Stepper_3*/
		if(k < (fabs(target_3-position_3)/1.8*8*2*ratio_3)*percent)
		{
			if(ARR_3>max_speed_3)
			{
				ARR_3-=x;
				if(ARR_3<=max_speed_3)
				{
					CNT_3 = k;
				}
			}
			__HAL_TIM_SET_AUTORELOAD(&htim1,ARR_3);
		}
		if(k > (fabs(target_3-position_3)/1.8*8*2*ratio_3)-(CNT_3+error_3) && (target_3-position_3)!=0)
		{
			if(ARR_3<initial_speed)
			{
				ARR_3+=x;
				if(ARR_3>=initial_speed)
				{
					CNT_3 = 0;
				}
			}
			__HAL_TIM_SET_AUTORELOAD(&htim1,ARR_3);
		}
	}
	/*Servo Control*/
	if(htim->Instance == TIM12)
	{
		/*speed_control*/
		/*Servo 1*/
		/*if (a < 500) a+=y;
		else a = 0;
		if(degree_1 != temp_1)
		{
			if((degree_1-temp_1)>=0)
				pulse_1 = 2000;
			else
				pulse_1 = 1000;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_1);
			a = 0;
		}
		if(a > fabs(degree_1-Deg_1))
		{
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			Deg_1 = degree_1;
		}*/

		/*Servo 2*/
		/*if (b < 500) b+=y;
		else b = 0;
		if(degree_2 != temp_2)
		{
			if((degree_2-temp_2)>=0)
				pulse_2 = 2000;
			else
				pulse_2 = 1000;
			__HAL_TIM_SET_COMPARE(&htim23, TIM_CHANNEL_1, pulse_2);
			b = 0;
		}
		if(b > fabs(degree_2-Deg_2))
		{
			__HAL_TIM_SET_COMPARE(&htim23, TIM_CHANNEL_1, 0);
			Deg_2 = degree_2;
		}

		temp_1 = degree_1;
		temp_2 = degree_2;*/

		/*angle_control*/
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 350+(degree_1+96)*(2250-350)/180);
		if(Switch)
		{
			__HAL_TIM_SET_COMPARE(&htim23, TIM_CHANNEL_1, 0);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim23, TIM_CHANNEL_1, 350+(degree_2)*(2300-350)/180);
		}
	}

}
/*Grab*/
/*tighten*/
/*if(grab == 1)
{
	stop_1 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12);
	stop_2 = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10);

	if(stop_1 == 0)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		Time_a++;
	}
	else
	{
		//ratio = 100;
		if(Time_1==1000)
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		}
		else
		{
			Time_1++;
		}
	}
	if(stop_2 == 0)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
		Time_b++;
	}
	else
	{
		if(Time_2==1000)
		{
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
		}
		else
		{
			Time_2++;
		}
	}

	reset_a = 0;
	reset_b = 0;
}
//loosen
else if(grab == 0)
{
	//reset pressure time
	Time_1 = 0;
	Time_2 = 0;

	if(reset_a==0&&Time_A<Time_a)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_SET);
		Time_A++;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_13,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_12,GPIO_PIN_RESET);
		Time_A = 0;
		reset_a = 1;
		Time_a = 0;
	}
	if(reset_b==0&&Time_B<Time_b)
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
		Time_B++;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOD,GPIO_PIN_11,GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_RESET);
		Time_B = 0;
		reset_b = 1;
		Time_b = 0;
	}
}*/
/*interruption*/
/*else
{
	Time_A = 0;
	Time_B = 0;
	Time_a = 0;
	Time_b = 0;
	Time_1 = 0;
	Time_2 = 0;
	stop_1 = 0;
	stop_2 = 0;
	reset_a = 1;
	reset_b = 1;
}*/
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
