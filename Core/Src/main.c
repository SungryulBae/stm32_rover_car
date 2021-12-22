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
#include "main.h"
#include "stm32f4xx_hal.h" //

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "mpu6050.h"
#include "lcd.h"
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
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
osThreadId myTask02Handle;
/* USER CODE BEGIN PV */
MPU6050_t MPU6050;

volatile double Duty = 0.9;
// lcd
//I2C_HandleTypeDef hi2c1;
//UART_HandleTypeDef huart3;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM5_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
void StartDefaultTask(void const * argument);
void StartTask02(void const * argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile uint8_t dist_trig = 0;

//---------------------------- UART ----------------

uint8_t rx; 	// 1 byte
uint8_t buf[50], buf_index=0;

void PutString(char *str)
{
	HAL_UART_Transmit_IT(&huart3, (uint8_t*)str, strlen(str));
}
void PutString_Bluetooth(char *str)
{
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)str, strlen(str));
}

void DispCommand(void)
{
	char buffer[50];
	strcpy(buffer, "\r\n> ");
	HAL_UART_Transmit(&huart3, (uint8_t*)buffer, strlen(buffer), 1000);
}

//---------------------------UART_interrupt------------------------------------------------------------------------

unsigned char Receive_Buffer[1];
bool RxFlag;

int _write(int file,unsigned char* p, int len)
{
	HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3,p,len,100);
	return (status == HAL_OK ? len :0);
}


//-----------------------------------------Supersonic & Remote Controller--------------------------------------------

uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

#define TRIG_PIN GPIO_PIN_13
#define TRIG_PORT GPIOF


//////////Remote Controller//////////

//tim9 ch2
uint32_t IC_Val1_ch1 = 0;
uint32_t IC_Val2_ch1 = 0;
uint32_t DF1 = 0;
uint32_t DF1_old = 0;
uint8_t Is_First_Captured_ch1 = 0;

//tim3 ch4
uint32_t IC_Val1_ch2 = 0;
uint32_t IC_Val2_ch2 = 0;
uint32_t DF2 = 0;
uint32_t DF2_old = 0;
uint8_t Is_First_Captured_ch2 = 0;

//tim2 ch1
uint32_t IC_Val1_ch3 = 0;
uint32_t IC_Val2_ch3 = 0;
uint32_t DF3 = 0;
uint32_t DF3_old = 0;
uint8_t Is_First_Captured_ch3 = 0;

//tim12 ch1
uint32_t IC_Val1_ch4 = 0;
uint32_t IC_Val2_ch4 = 0;
uint32_t DF4 = 0;
uint32_t DF4_old = 0;
uint8_t Is_First_Captured_ch4 = 0;

/////////////////////////////////////

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM1)  // if the interrupt source is channel1
	{
		if (Is_First_Captured==0) // if the first value is not captured
		{
			IC_Val1 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);  // read second value
			__HAL_TIM_SET_COUNTER(&htim1, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}

			else if (IC_Val1 > IC_Val2)
			{
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034/2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim1, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}

	////////////////////////////////////////Remote Controller////////////////////////////////////////

	// PA0
	if(htim->Instance == TIM2)  // if the interrupt source is channel2
	{
		if (Is_First_Captured_ch1==0) // if the first value is not captured
		{

			IC_Val1_ch1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1); // read the first value


			Is_First_Captured_ch1 = 1;  // set the first captured as true
			// Now change the polarity to falling edge

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);

		}

		else if (Is_First_Captured_ch1==1)   // if the first is already captured
		{

			IC_Val2_ch1 = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);  // read second value


			__HAL_TIM_SET_COUNTER(&htim2, 0);  // reset the counter


			if (IC_Val2_ch1 > IC_Val1_ch1)
			{
				DF1 = IC_Val2_ch1 - IC_Val1_ch1;
			}
			else if (IC_Val1_ch1 > IC_Val2_ch1)
			{
				DF1 = (0xffff - IC_Val1_ch1) + IC_Val2_ch1;
			}


			if (DF1>800 && DF1<2400) // 1100~1800
			{
				DF1_old = DF1;
			}


			Is_First_Captured_ch1 = 0; // set it back to false

			// set polarity to rising edge

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim2, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);


			__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);


		}
	}

	// PB1
	if(htim->Instance == TIM3)  // if the interrupt source is channel3
	{
		if (Is_First_Captured_ch2==0) // if the first value is not captured
		{

			IC_Val1_ch2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4); // read the first value


			Is_First_Captured_ch2 = 1;  // set the first captured as true
			// Now change the polarity to falling edge

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_FALLING);

		}

		else if (Is_First_Captured_ch2==1)   // if the first is already captured
		{

			IC_Val2_ch2 = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_4);  // read second value


			__HAL_TIM_SET_COUNTER(&htim3, 0);  // reset the counter


			if (IC_Val2_ch2 > IC_Val1_ch2)
			{
				DF2 = IC_Val2_ch2 - IC_Val1_ch2;
			}
			else if (IC_Val1_ch2 > IC_Val2_ch2)
			{
				DF2 = (0xffff - IC_Val1_ch2) + IC_Val2_ch2;
			}

			if (DF2>800 && DF2<2500) // 1000 ~ 1900
			{
				DF2_old = DF2;
			}


			Is_First_Captured_ch2 = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim3, TIM_CHANNEL_4, TIM_INPUTCHANNELPOLARITY_RISING);

			__HAL_TIM_DISABLE_IT(&htim3, TIM_IT_CC4);

		}
	}



	// ch4 PA3
	if(htim->Instance == TIM9)  // if the interrupt source is channel9
	{
		if (Is_First_Captured_ch4==0) // if the first value is not captured
		{
			IC_Val1_ch4 = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_2); // read the first value



			Is_First_Captured_ch4 = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim9, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_FALLING);

		}

		else if (Is_First_Captured_ch4==1)   // if the first is already captured
		{
			IC_Val2_ch4 = HAL_TIM_ReadCapturedValue(&htim9, TIM_CHANNEL_2);  // read second value



			__HAL_TIM_SET_COUNTER(&htim9, 0);  // reset the counter


			if (IC_Val2_ch4 > IC_Val1_ch4)
			{
				DF4 = IC_Val2_ch4 - IC_Val1_ch4;
			}
			else if (IC_Val1_ch4 > IC_Val2_ch4)
			{
				DF4 = (0xffff - IC_Val1_ch4) + IC_Val2_ch4;
			}


			if (DF4>800 && DF4<2200)
			{
				DF4_old = DF4;
			}


			Is_First_Captured_ch4 = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(&htim9, TIM_CHANNEL_2, TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim9, TIM_IT_CC2);


		}
	}

	// ch3 PA2
	if(htim->Instance == TIM5)  // if the interrupt source is channel2
	{
		if (Is_First_Captured_ch3==0) // if the first value is not captured
		{

			IC_Val1_ch3 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3); // read the first value


			Is_First_Captured_ch3 = 1;  // set the first captured as true
			// Now change the polarity to falling edge

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);

		}

		else if (Is_First_Captured_ch3==1)   // if the first is already captured
		{

			IC_Val2_ch3 = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_3);  // read second value


			__HAL_TIM_SET_COUNTER(&htim5, 0);  // reset the counter


			if (IC_Val2_ch3 > IC_Val1_ch3)
			{
				DF3 = IC_Val2_ch3 - IC_Val1_ch3;
			}
			else if (IC_Val1_ch3 > IC_Val2_ch3)
			{
				DF3 = (0xffff - IC_Val1_ch3) + IC_Val2_ch3;
			}




			if (DF3>800 && DF3<2400)
			{
				DF3_old = DF3;
			}


			Is_First_Captured_ch3 = 0; // set it back to false

			// set polarity to rising edge

			__HAL_TIM_SET_CAPTUREPOLARITY(&htim5, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);


			__HAL_TIM_DISABLE_IT(&htim5, TIM_IT_CC3);


		}
	}



	/////////////////////////////////////////////////////////////////////////////////////////////////


}

void HCSR04_Read (void)
{
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
	HAL_Delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
}

void RemoteController_Read (void)
{
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC4); // ch1
	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1); // ch2
	__HAL_TIM_ENABLE_IT(&htim9, TIM_IT_CC2); // ch4
	__HAL_TIM_ENABLE_IT(&htim5, TIM_IT_CC3); // ch5
}

//-------------------------------------------------------------------------------------

/*
#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz;


void MPU6050_Init (void)
{
	uint8_t check, Data;

	// check device ID WHO_AM_I
	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);



	if (check == 104) // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +- 2g
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +- 250 degree/s
		Data = 0x00;
		HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
	}
}

void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

//	 convert the RAW values into acceleration in 'g'
//	     we have to divide according to the Full scale value set in FS_SEL
//	     I have configured FS_SEL = 0. So I am dividing by 16384.0
//	     for more details check ACCEL_CONFIG Register

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

//	     convert the RAW values into dps (degree/s)
//	     we have to divide according to the Full scale value set in FS_SEL
//	     I have configured FS_SEL = 0. So I am dividing by 131.0
//	     for more details check GYRO_CONFIG Register

	Gx = Gyro_X_RAW/131.0;
	Gy = Gyro_Y_RAW/131.0;
	Gz = Gyro_Z_RAW/131.0;
}

*/


//----------------------------------TIM4 (PWM)---------------------------------------------------
volatile uint16_t value1 = 25, value2 = 25;
uint16_t add = 10;


//-------------------------------systick------------------------------------------------------
uint32_t timeout = 1000; // 1 sec
bool timeout_trig = 0;
uint32_t one_cnt = 0;
uint32_t five_cnt = 0;

//----------------------------------------------------

#define STOP 0
#define NONSTOP 1


uint8_t run = 5;



/*
      0 = stop
 	  5 = turn left
	 10 = turn right


	 direction
 	 \ | /    1 2 3
 	 - 0 -    4 0 6
	 / | \    7 8 9
*/


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
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM9_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_I2C2_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1); //supersonic


  HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // ch1 FB
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_4); // ch2 LR
  HAL_TIM_IC_Start_IT(&htim9, TIM_CHANNEL_2); // ch4 turn
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_3); // ch3 speed

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  TIM4->ARR=50-1;
  // speed
  // CCR1 = RF
  // CCR2 = RB
  // CCR3 = LF
  // CCR4 = LB
  TIM4->CCR1=50*0.1; // fast
  TIM4->CCR2=50*0.1; // fast
  TIM4->CCR3=50*0.1; // fast
  TIM4->CCR4=50*0.1; // fast





  //MPU6050_Init();
  while (MPU6050_Init(&hi2c1) == 1);



  //lcd
  init();


  //---------------------------UART_interrupt------------------------------------------------------------------------
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_RXNE);
  __HAL_UART_ENABLE_IT(&huart3,UART_IT_TC);
  printf("ex6_2 Echo UART");

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask02 */
  osThreadDef(myTask02, StartTask02, osPriorityIdle, 0, 256);
  myTask02Handle = osThreadCreate(osThread(myTask02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM1_BRK_TIM9_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
  /* TIM1_CC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  /* TIM5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* I2C1_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
  /* I2C1_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* I2C2_EV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  /* I2C2_ER_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
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
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 1;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 1;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 400-1;
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 65535;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 16-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim9, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PF13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG2 PG3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void UART_Interrupt_Processing(UART_HandleTypeDef*huart)
{
	if(huart->Instance == huart3.Instance)
	{
		if(HAL_UART_Receive_IT(&huart3,Receive_Buffer,1)==HAL_OK)
		{
			RxFlag = TRUE;
		}
	}
}


//======================= SYSTICK_Callback ================================

// you must put "HAL_SYSTICK_IRQHandler()" into void Systick_Handler(void)



void HAL_SYSTICK_Callback(void)
{
	if(timeout != 0)
	{
		timeout--;
	}
	else
	{
		timeout_trig = 1;




		timeout = 1000;
		one_cnt++;
		if ( (one_cnt != 0) && (one_cnt % 5 == 0) )
		{
			 five_cnt++;
		}
	}


}





/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HCSR04_Read();
	  RemoteController_Read();


	  // 20kHz PWM (D=0.1~0.9)

	  /*

	  // speed test

	  TIM4->CCR1=value1;
	  TIM4->CCR2=value1;
	  TIM4->CCR3=value2;
	  TIM4->CCR4=value2;


	  if(value1>=50*0.9)
	  {
		  add = -1;
	  }
	  else if(value1<=50*0.1)
	  {
		  add = 1;
	  }

	  value1+=add;
	  value2-=add;
	  */



	  /*
	  if 	  ( (five_cnt % 6) == 1)
	  {
		  run = 1; // left


	  }
	  else if ( (five_cnt % 6) == 3)
	  {
		  run = 2; // right


	  }
	  else if ( (five_cnt % 6) == 5)
	  {
		  run = 3; // straight slow


	  }
	  else
	  {
		  run = 0; // straight fast
	  }



	  if 	  (run == 0) // straight fast
	  {
		  // left motor
		  TIM4->CCR1=50*0.2; // fast
		  TIM4->CCR2=50*0.2; // fast
		  // right motor
		  TIM4->CCR3=50*0.2; // fast
		  TIM4->CCR4=50*0.2; // fast
	  }
	  else if (run == 1) // left
	  {
		  // left motor
		  TIM4->CCR1=50*0.8; // slow
		  TIM4->CCR2=50*0.8; // slow
		  // right motor
		  TIM4->CCR3=50*0.2; // fast
		  TIM4->CCR4=50*0.2; // fast
	  }
	  else if (run == 2) // right
	  {
		  // left motor
		  TIM4->CCR1=50*0.2; // fast
		  TIM4->CCR2=50*0.2; // fast
		  // right motor
		  TIM4->CCR3=50*0.8; // slow
		  TIM4->CCR4=50*0.8; // slow
	  }
	  else if (run == 3) // straight slow
	  {
		  // left motor
		  TIM4->CCR1=50*0.8; // slow
		  TIM4->CCR2=50*0.8; // slow
		  // right motor
		  TIM4->CCR3=50*0.8; // slow
		  TIM4->CCR4=50*0.8; // slow
	  }

	  */

	  //////////////

	  /*
	        5 = stop
	   	  0 = turn left
	  	 10 = turn right


	  	 direction
	   	 \ | /    1 2 3
	   	 - 0 -    4 5 6
	  	 / | \    7 8 9

	      // straight forward (run == 2 )
	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

	  	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, 1); // nonstop
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, 1);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, 1);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, 1);
	  */


	  // Direction
	  // DF1 1150 1760 2010 foward<--> back
	  // DF2 1890 1280 1000 left<--> right

	  // DF4 1100 1500 1800 turn left<--> right

	  // SPEED
	  // DF3 1000~1900 Low --> HIGH


	  // Direction
	  if ( DF1_old < 1500 ) // FOWARD
	  {
		  if (DF2_old > 1400)
		  {
			  run = 1;
		  }
		  else if (DF2_old > 1150)
		  {
			  run = 2;
		  }
		  else
		  {
			  run = 3;
		  }
	  }
	  else if (DF1_old < 1850 )
	  {
		  if (DF2_old > 1400)
		  {
			  run = 4;
		  }
		  else if (DF2_old < 1150)
		  {
			  run = 6;
		  }
		  else
		  {
			  if (DF4_old < 1250)
			  {
				  run = 0; // turn left
			  }
			  else if (DF4_old < 1650)
			  {
				  run = 5; // stop
			  }
			  else
			  {
				  run = 10; // turn right
			  }
		  }
	  }
	  else // BACK
	  {
		  if (DF2_old > 1400)
		  {
			  run = 7;
		  }
		  else if (DF2_old > 1150)
		  {
			  run = 8;
		  }
		  else
		  {
			  run = 9;
		  }

	  }


	// SPEED
	  // map 1000~1900 to 0.9 ~ 0.1
	  Duty = (-1*0.8/900*(DF3_old-1000))+0.9;

	  if (Duty>0 && Duty<1)
	  {
		  TIM4->CCR1=50*Duty;
		  TIM4->CCR2=50*Duty;
		  TIM4->CCR3=50*Duty;
		  TIM4->CCR4=50*Duty;
	  }





	  //run = five_cnt % 11; // auto run mode


	  if 	   (run == 1) // left forward
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, NONSTOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, STOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, STOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, NONSTOP);
	  }
	  else if  (run == 2) // straight forward
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, NONSTOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, NONSTOP);
	  }
	  else if  (run == 3) // right forward
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, STOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, STOP);
	  }
	  else if  (run == 4) // left
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, !1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, NONSTOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, NONSTOP);
	  }
	  else if  (run == 5) // stop
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, STOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, STOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, STOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, STOP);
	  }
	  else if  (run == 6) // right
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, !1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, NONSTOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, NONSTOP);
	  }
	  else if  (run == 7) // left back
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, !1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, !1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, STOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, STOP);
	  }
	  else if  (run == 8) // straight back
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, !1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, !1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, NONSTOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, NONSTOP);
	  }
	  else if  (run == 9) // right back
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, !1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, !1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, NONSTOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, STOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, STOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, NONSTOP);
	  }
	  else if  (run == 10) // turn right
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, !0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, NONSTOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, NONSTOP);
	  }
	  else if  (run == 0) // turn left
	  {
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, 0);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, !1);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, !1);

		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, NONSTOP);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, NONSTOP);
		  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, NONSTOP);
	  }



	  // CCR1=%d CCR2= %d CCR3= %d CCR4= %d      TIM4->CCR1,TIM4->CCR2,TIM4->CCR3,TIM4->CCR4
	  // timeout=%d, five_cnt=%d,                timeout,five_cnt

	  //sprintf(temp1, "\r\n IC_Val1=%d IC_Val2=%d Difference=%d \r\n df1=%d df2=%d df3=%d df4=%d",IC_Val1, IC_Val2, Difference,Difference_tim2_ch1,Difference_tim2_ch2,Difference_tim2_ch3,Difference_tim2_ch4);
	  //strcat(buffer,temp1);


	  //sprintf(temp1, "\r\n (Ax,Ay,Az)=(%.2f, %.2f ,%.2f)",MPU6050.Ax,MPU6050.Ay,MPU6050.Az);
	  //strcat(buffer,temp1);
	  //sprintf(temp2, "\r\n (Gx,Gy,Gz)=(%.2f, %.2f ,%.2f)",MPU6050.Gx,MPU6050.Gy,MPU6050.Gz);
	  //strcat(buffer,temp2);
    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {

	  //MPU6050_Read_Accel();
	  //MPU6050_Read_Gyro();
	  MPU6050_Read_All(&hi2c1, &MPU6050);


	  /*
      if(RxFlag)
      {
    	  printf("rx=%d[%c]\r\n",Receive_Buffer[0],Receive_Buffer[0]);
    	  RxFlag=FALSE;
      }
      */

	  //------------------------ LCD -------------------------------------------------------
		char str_UP[17],str_DOWN[17];
		sprintf(str_UP,"D=%02d(%+04d,%+04d)",Distance,(int)MPU6050.KalmanAngleX,(int)MPU6050.KalmanAngleY);
		sprintf(str_DOWN,"M=%02d DT=%02d%%     ",run,(int)(Duty*100));

		// clear display (optional here)
		//LCD_SendCommand(LCD_ADDR, 0b00000001);

		// set address to 0x00
		LCD_SendCommand(LCD_ADDR, 0b10000000);
		LCD_SendString(LCD_ADDR, str_UP);

		// set address to 0x40
		LCD_SendCommand(LCD_ADDR, 0b11000000);
		LCD_SendString(LCD_ADDR, str_DOWN);


	  // debug
	  char buffer[50]={""}, temp1[30],temp2[30];
	  sprintf(buffer, "\r\n (D,ROLL,PITCH) = %d,%.2f,%.2f,  run=%d, DFch1=%d DFch2=%d DFch3=%d (Duty=%.2f) DFch4=%d, timeout=%d",Distance,MPU6050.KalmanAngleX,MPU6050.KalmanAngleY,run,DF1_old,DF2_old,DF3_old,Duty,DF4_old,timeout);
	  PutString(buffer);
	  PutString_Bluetooth(buffer);

	  osDelay(20);

  }
  /* USER CODE END StartTask02 */
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
