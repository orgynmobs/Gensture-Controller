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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75




//Definición esttructuras
typedef struct {
	float Gx;
	float Gy;
	float Gz;
}Gyro;

typedef struct{
	float Ax;
	float Ay;
	float Az;
} Accel;


 typedef struct {
	 int hi2c;
	 Gyro MPUgyro;
	 Accel MPUaccel;
	 float offsetx;
	 float offsetY;
}MPU6050;






MPU6050 mpu1,mpu2;


//valores en RAW temporales de GYRO Y ACCEL
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;
//valores del las salidas de las medidas
float Ax, Ay, Az, Gx, Gy, Gz;
// datos en valores de graved y velocidad para MPU1 1
float Ax1, Ay1, Az1, Gx1, Gy1, Gz1;
//datos en valores de graved y velocidad para MPU1 2
float Ax2, Ay2, Az2, Gx2, Gy2, Gz2;

//angulos reales MPU 1
float gyro_y1,gyro_x1,gyro_z1;
//angulos reales MPU 2
float gyro_y2,gyro_x2,gyro_z2;
//aceleraciones reales Mpu 1
float accel_x1,accel_y1,accel_z1;
//aceleraciones reales mpu 2
float accel_x2,accel_y2,accel_z2;



//salidas procesadas
float angulo_y,angulo_x;
float angulo_y2,angulo_x2;
char palabra[32];
char palabra2[32];
//integradores
uint32_t time;
uint32_t dt;
//kalman
int X, X_estimate;
float ADC_val;  // valor de lectura real
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int MPU6050_Init (I2C_HandleTypeDef hi2c)
{
	uint8_t check;
		uint8_t Data;

		// check device ID WHO_AM_I
		Data = 0;
		check= 0;
		HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

		if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
		{
			// power management register 0X6B we should write all 0's to wake the sensor up


			// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
			Data = 0x07;
			HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

			// Set accelerometer configuration in ACCEL_CONFIG Register
			// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

			// Set Gyroscopic configuration in GYRO_CONFIG Register
			// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
			return 1;
		}
 return 0;
	}

Accel MPU6050_Read_Accel (int selector)
{
	uint8_t Rec_Data[6];
Accel lectura;

I2C_HandleTypeDef hi2c;

if (selector == 1) hi2c = hi2c1;
if (selector == 2) hi2c = hi2c2;

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	lectura.Ax = Accel_X_RAW/16384.0;
	lectura.Ay = Accel_Y_RAW/16384.0;
	lectura.Az = Accel_Z_RAW/16384.0;
	return lectura;
}


Gyro MPU6050_Read_Gyro (int selector)
{
Gyro lectura;
	uint8_t Rec_Data[6];
	I2C_HandleTypeDef hi2c;

	if (selector == 1) hi2c = hi2c1;
	if (selector == 2) hi2c = hi2c2;
	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	lectura.Gx = Gyro_X_RAW/131.0;
	lectura.Gy = Gyro_Y_RAW/131.0;
	lectura.Gz = Gyro_Z_RAW/131.0;

return lectura;
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
int start = 0;
mpu1.hi2c= 1;
mpu2.hi2c= 2;

mpu1.offsetY =  2.579;
mpu1.offsetx =  0.3;

mpu2.offsetY =  -3;
mpu2.offsetx = 0;


//valores stadisticos
	float desv_tipica = 2.0231;
	float desv_estado = 2.0231;
	float var = desv_tipica*desv_tipica;
	float var_estado = desv_estado*desv_estado;
	float P = var_estado;
	//valores edl filtro
	float Kalman ;

	float P_previa;


	char info_real[32], info_kalman[32];
		char ln[] = "\n\r";
		char comma[] = ",";
/*char clear[] = "CLEARDATA";
char columns[] = "LABEL,Gx,Gy,Gz,Ax,Ay,Az";
char timer [] ="RESETTIMER";

HAL_UART_Transmit(&huart5, clear, sizeof(clear), HAL_MAX_DELAY);
HAL_UART_Transmit(&huart5, columns, sizeof(columns), HAL_MAX_DELAY);
HAL_UART_Transmit(&huart5, timer, sizeof(timer), HAL_MAX_DELAY);
*/
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
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	while(start == 0){
		 start = MPU6050_Init(hi2c1) && MPU6050_Init(hi2c2);
/*		 char clear[] = "CLEARDATA";
		 char columns[] = "LABEL,Gx,Gy,Gz,Ax,Ay,Az";
		 char timer [] ="RESETTIMER";

		 HAL_UART_Transmit(&huart5, clear, sizeof(clear), HAL_MAX_DELAY);
		 HAL_UART_Transmit(&huart5, columns, sizeof(columns), HAL_MAX_DELAY);
		 HAL_UART_Transmit(&huart5, timer, sizeof(timer), HAL_MAX_DELAY);
		 */
	}

	  // read the Accelerometer and Gyro values and tranform into angles


	// 	  MPU6050_Read_Accel(hi2c2,Ax1,Ay1,Az1); // error AL asignar? proque no coge las void
		 mpu2.MPUaccel = MPU6050_Read_Accel(mpu2.hi2c);

		 // Ax1 = Ax;
		 // Ay1 = Ay;
		 // Az1 = Az;
		 mpu1.MPUaccel =  MPU6050_Read_Accel(mpu1.hi2c);
			//Ax2 = Ax;
		//	Ay2 = Ay;
		//	 Az2 = Az;

	 	mpu2.MPUgyro =   MPU6050_Read_Gyro(mpu2.hi2c);
	 	//  Gx1 = Gx;
	 	//  Gy1 = Gy;
	 	//  Gz1 = Gz;
	 	mpu1.MPUgyro = MPU6050_Read_Gyro(mpu1.hi2c);
	 	//	 	  Gx2 = Gx;
	 	//	 	  Gy2 = Gy;
	 	//	 	  Gz2 = Gz;

accel_x1= atan(mpu1.MPUaccel.Ay/sqrt(pow(mpu1.MPUaccel.Ax,2) + pow(mpu1.MPUaccel.Az,2)))*(180.0/3.14);
accel_y1=atan(-mpu1.MPUaccel.Ax/sqrt(pow(mpu1.MPUaccel.Ay,2) + pow(mpu1.MPUaccel.Az,2)))*(180.0/3.14);

accel_x2= atan(mpu2.MPUaccel.Ay/sqrt(pow(mpu2.MPUaccel.Ax,2) + pow(mpu2.MPUaccel.Az,2)))*(180.0/3.14);
accel_y2=atan(-mpu2.MPUaccel.Ax/sqrt(pow(mpu2.MPUaccel.Ay,2) + pow(mpu2.MPUaccel.Az,2)))*(180.0/3.14);

	  dt =  HAL_GetTick()-time;
	  time = HAL_GetTick();
	 	gyro_y1 += dt*(mpu1.MPUgyro.Gy-mpu1.offsetY)/1000;  //2.579
	 	gyro_x1 += dt*(mpu1.MPUgyro.Gx-mpu1.offsetx)/1000;    //0.3
	 	gyro_z1 += dt*(mpu1.MPUgyro.Gz)/1000;

	 	gyro_y2 += dt*(mpu2.MPUgyro.Gy-mpu2.offsetY)/1000;
	 	gyro_x2 += dt*(mpu2.MPUgyro.Gx-mpu2.offsetx)/1000;
	 	gyro_z2 += dt*(mpu2.MPUgyro.Gz)/1000;



angulo_y = 0.01*(accel_y1) + 0.9*gyro_y1;
angulo_x = 0.01*(accel_x1) + 0.9*gyro_x1;

angulo_y2 = 0.01*(accel_y2) + 0.9*gyro_y2;
angulo_x2 = 0.01*(accel_x2) + 0.9*gyro_x2;

//ADC read

P_previa =  P;


	 	  Kalman = P/(P+var);


HAL_ADC_Start(&hadc1);
	 if(HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY)==HAL_OK){
		 ADC_val=HAL_ADC_GetValue(&hadc1) ;  // entre 2500 y 1500 . quzás ajusatble tocando la resolucion
	 }
	 X_estimate = X + Kalman*(ADC_val-X);
		 	  P = (1-Kalman)*P_previa + fabs(X - X_estimate)*0.01;
		 	  X = X_estimate;


//UART
		 	 gcvt(angulo_x,10,palabra);
		 	 gcvt(angulo_x2,10,palabra2);
		 	  itoa(X_estimate,info_kalman,10);
		 	  itoa(ADC_val,info_real,10);
		 	 HAL_UART_Transmit(&huart5, (uint8_t*)info_real, sizeof(int), 100);
		 	 HAL_UART_Transmit(&huart5, (uint8_t*)comma, sizeof(comma), 100);
		 	 HAL_UART_Transmit(&huart5, (uint8_t*)info_kalman, sizeof(int), 100);
		 	HAL_UART_Transmit(&huart5, (uint8_t*)comma, sizeof(comma), 100);
		 	HAL_UART_Transmit(&huart5,(uint8_t*)palabra,sizeof(float ), 100);//palabra
		 	HAL_UART_Transmit(&huart5, (uint8_t*)comma, sizeof(comma), 100);
			HAL_UART_Transmit(&huart5,(uint8_t*)palabra2,sizeof(float ), 100);//palabra
		 	 HAL_UART_Transmit(&huart5, (uint8_t*)ln, sizeof(comma), 100);

/*
 * OBTENCION DEL OFFSET EN EXCEL
gcvt(angulo_x,10,palabra);
//sprintf(palabra,"Y %f \n\r",angulo_y);   //offset y , 2.579
char espacio[] = "\r";
char separacion[] = " ";
HAL_UART_Transmit(&huart5,&palabra,sizeof(float ), 100);//palabra
HAL_UART_Transmit(&huart5,&separacion,sizeof(char ), 100);
HAL_UART_Transmit(&huart5,&ADC_val,sizeof(int ), 100);//palabra
HAL_UART_Transmit(&huart5,&espacio,sizeof(espacio ), 100);
*/


//__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,(angulo_y*200/180)+50);


//HAL_Delay(100);


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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  hi2c1.Init.ClockSpeed = 100000;
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
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

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
