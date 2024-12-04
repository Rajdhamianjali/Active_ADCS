/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
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
//#include "ADCS_Debug.h"
#include "OBC_Handshake.h"
#include "IMU.h"
#include "ADCS_Debug.h"
#include "estimator.h"
#include "Bdot.h"
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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t OBC_HANDSHAKE_FLAG;

MPU6500_t DEFAULT_MPU6500;
lsm9ds1_t DEFAULT_LSM9DS1;

imu_filter imu_filter_data;

sat_attitude attitude_sat;

sat_att_combined combined_sat_att;

extern mag_moment mag_moment_bdot;

uint8_t SAT_IMU_REG[17];
float IMU_SEN_DATA[9];

uint32_t mSEC = 0;
uint32_t sec = 0;
uint8_t min = 0;

int count = 0;

int mode = 0;

int IS_TAKE_DATA = 1;

float rxToOBC_temp[23]; //Data to be sent to OBC
int16_t rxToOBC_1[23]; //Arranged data in int16 format
uint8_t rxToOBC[46]; //Buffer to hold transmitted data
#define DATA_POINTS 10
uint8_t fullDataBuffer[DATA_POINTS][9];

int16_t decode1[23];
float decode2[23];

#define DATA_COLLECTION_PERIOD 10  // Data collection period in seconds
#define TOTAL_RUN_TIME 60         // Total run time in seconds

int counter = 0,counter2=0;  // Counter for tracking the 10-second intervals
int data_index = 0;
int total_time = 0; // Counter for total run time

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void arrangeData();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim1) {
		mSEC++;
		if (mSEC > 999) {
			sec++;
			count++;
			mSEC = 0;
		}

		if (count == 60) {
			MTQ_Disable();
			count = 0;
			myDebug("MTQ Disable\r\n");

			HAL_TIM_Base_Stop_IT(&htim1);

			rxToOBC_temp[9] = mag_moment_bdot.MomentX;
			rxToOBC_temp[10] = mag_moment_bdot.MomentY;
			rxToOBC_temp[11] = mag_moment_bdot.MomentZ;
			rxToOBC_temp[12] = mag_moment_bdot.Dy_per;
			rxToOBC_temp[13] = mag_moment_bdot.Dz_per;

			imu_filter_data = IMU_Get_Data(&DEFAULT_MPU6500, &DEFAULT_LSM9DS1); //IMU filtered data
			process_IMU_filt(imu_filter_data);

			rxToOBC_temp[14] = imu_filter_data.p_rps;
			rxToOBC_temp[15] = imu_filter_data.q_rps;
			rxToOBC_temp[16] = imu_filter_data.r_rps;

			rxToOBC_temp[17] = imu_filter_data.mx_ut;
			rxToOBC_temp[18] = imu_filter_data.my_ut;
			rxToOBC_temp[19] = imu_filter_data.mz_ut;

			rxToOBC_temp[20] = combined_sat_att.roll;
			rxToOBC_temp[21] = combined_sat_att.pitch;
			rxToOBC_temp[22] = combined_sat_att.yaw;

			arrangeData();
			if (HAL_UART_Transmit(&huart3, rxToOBC, 45, 1000) == HAL_OK) {
				myDebug("Transmitted to OBC\r\n");
				for (int i = 0; i < 46; i++) {
					myDebug("%02x ", rxToOBC[i]);
				}
				myDebug("\n");
			}
			memset(rxToOBC, '\0', sizeof(rxToOBC));
			memset(rxToOBC_temp, '\0', sizeof(rxToOBC_temp));

			HAL_Delay(1000);

			mSEC = 0;
			sec = 0;
			IS_TAKE_DATA = 1;
		}
	}
}

void arrangeData() {

	for (int i = 0; i < 23; i++) {
		myDebug("%.2f ", rxToOBC_temp[i]);
		rxToOBC_1[i] = (int16_t) round(rxToOBC_temp[i] * 100.0f);
	}
	myDebug("\n");

	myDebug("arranged \n");
	for (int i = 0; i < 23; i++) {
		myDebug("%d ", rxToOBC_1[i]);

	}
	myDebug("\n");

	for (int i = 0; i < 46; i++) {
		rxToOBC[2 * i] = (rxToOBC_1[i] >> 8) & 0xff;
		rxToOBC[2 * i + 1] = rxToOBC_1[i] & 0xff;
	}

//	rxToOBC[46] = 0xff;
//	rxToOBC[47] = 0xd9;

	/* if (HAL_UART_Transmit(&huart3, rxToOBC, 100, 1000) == HAL_OK) {
	 myDebug("Transmitted to OBC\r\n");
	 for (int i = 0; i < 100; i++) {
	 myDebug("%02x ", rxToOBC[i]);
	 }
	 myDebug("\n");

	 //decoding the data
	 //		for (int i = 0; i < 23; i++) {
	 //			decode1[i] = (int16_t) ((rxToOBC[2 * i] << 8)
	 //					| (rxToOBC[2 * i + 1] & 0xff));
	 //			decode2[i] = decode1[i] / 100.0f;
	 //			myDebug("Decoded values:\n");
	 //			for (int i = 0; i < 23; i++) {
	 //				myDebug("%.2f ", rxToOBC_temp[i]);
	 //			}
	 //			myDebug("\n");
	 //		}

	 //		uint8_t foot[2] = { 0xff, 0xd9 };

	 //		if (HAL_UART_Transmit(&huart3, foot, sizeof(foot), 1000) == HAL_OK) {
	 //			myDebug("ffd9 TX 1\n");
	 //			if (HAL_UART_Transmit(&huart3, foot, 2, 1000) == HAL_OK) {
	 //				myDebug("ffd9 TX 2\n", sizeof(foot));
	 //			}
	 //		}
	 }
	 */
	myDebug("\n");

//	memset(rxToOBC, '\0', sizeof(rxToOBC));
//	memset(rxToOBC_temp, '\0', sizeof(rxToOBC_temp));
//
//	HAL_Delay(1000);
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_SPI1_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_TIM1_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	myDebug("##### Welcome to Active ADCS Debug Zone #####\r\n");
	myDebug("----- Waiting for Handshake command from OBC -----\r\n");
//	OBC_HANDSHAKE_FLAG = 1;
	while (OBC_HANDSHAKE_FLAG == 0) {
		WAIT_FOR_HANDSHAKE();
	}

	myDebug("----- Waiting Enable command from OBC -----\r\n");
//	mode = 2;
	while (mode == 0) {
		GET_COMMAND_OBC();
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (total_time < TOTAL_RUN_TIME) {

			if (IS_TAKE_DATA == 1) {

				if (mode == 0) {
					GET_COMMAND_OBC();
				}

				if (mode == 1 || mode == 2) {

					IMU_Setup(&DEFAULT_MPU6500); //initialization and calibration
					HAL_Delay(1000);
					imu_filter_data = IMU_Get_Data(&DEFAULT_MPU6500,
							&DEFAULT_LSM9DS1); //IMU filtered data
					HAL_Delay(1000);
					process_IMU_filt(imu_filter_data);

					memset(rxToOBC_temp, '\0', sizeof(rxToOBC_temp));

					rxToOBC_temp[0] = imu_filter_data.p_rps;
					rxToOBC_temp[1] = imu_filter_data.q_rps;
					rxToOBC_temp[2] = imu_filter_data.r_rps;

					rxToOBC_temp[3] = imu_filter_data.mx_ut;
					rxToOBC_temp[4] = imu_filter_data.my_ut;
					rxToOBC_temp[5] = imu_filter_data.mz_ut;

					rxToOBC_temp[6] = combined_sat_att.roll;
					rxToOBC_temp[7] = combined_sat_att.pitch;
					rxToOBC_temp[8] = combined_sat_att.yaw;

					memcpy(fullDataBuffer[data_index], rxToOBC_temp,
							sizeof(rxToOBC_temp));

					counter++;
					data_index++;
					if (counter >= DATA_COLLECTION_PERIOD) {


						if (mode == 1) {

							arrangeData();
							if (HAL_UART_Transmit(&huart3, rxToOBC,
									sizeof(rxToOBC), 1000) == HAL_OK) {
								myDebug("Transmitted to OBC\r\n");
								for (int i = 0; i < sizeof(rxToOBC); i++) {
									myDebug("%02x ", rxToOBC[i]);
								}
								myDebug("\n");
							}
						}
						memset(rxToOBC,'\0', sizeof(rxToOBC));

						counter = 0;  // Reset counter after 10 seconds
						data_index = 0;

					}
					if (mode == 2) {

						IS_TAKE_DATA = 0;

						CalTorque(imu_filter_data, &DEFAULT_LSM9DS1,
								combined_sat_att);
					}
					HAL_Delay(1000);
				}
			}
		}
	}

	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

}
/* USER CODE END 3 */

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

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
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 72 - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 2000 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 36000 - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 2000 - 1;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 36000 - 1;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CS_MAG_GPIO_Port, CS_MAG_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, CS_MPU_Pin | MTQEN_5V_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CS_MAG_Pin */
	GPIO_InitStruct.Pin = CS_MAG_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CS_MAG_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : CS_MPU_Pin MTQEN_5V_Pin */
	GPIO_InitStruct.Pin = CS_MPU_Pin | MTQEN_5V_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
