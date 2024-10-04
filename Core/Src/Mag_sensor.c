/*
 * Mag_sensor.c
 *
 *  Created on: Sep 7, 2024
 *      Author: Dell
 */
#include "Mag_sensor.h"

/**
 * @brief  Read data from Specific Register address of LSM9DS1
 * @param  hspi1 pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  add Register address from which data is to be read
 */
uint8_t LSM9DS1_ReadReg(SPI_HandleTypeDef *hspi1, uint8_t add) {
	uint8_t val;
	add |= 0x80;  // set the MSB to indicate a read operation
	HAL_GPIO_WritePin(GPIOA, CS_MAG_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, &add, 1, 100);
	HAL_SPI_Receive(hspi1, &val, 1, 100);
	HAL_GPIO_WritePin(GPIOA, CS_MAG_Pin, GPIO_PIN_SET);
	return val;
}

/**
 * @brief  Write on Specific Register address of LSM9DS1
 * @param  hspi1 pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  add Register address where certain value is to be written
 */
void LSM9DS1_WriteReg(SPI_HandleTypeDef *hspi1, uint8_t add, uint8_t val) {
	add &= 0x7F;  // clear the MSB to indicate a write operation
	HAL_GPIO_WritePin(GPIOA, CS_MAG_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspi1, &add, 1, 100);
	HAL_SPI_Transmit(hspi1, &val, 1, 100);
	HAL_GPIO_WritePin(GPIOA, CS_MAG_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Initialize LSM9DS1 to work in 16-bit, 1.25Hz ODR, ±4 Gauss and Continuous conversion Mode
 * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
 *               the configuration information for SPI module.
 * @param  ctrl2 Control Register Value to choose LSM9DS1 sensor Scale
 */
int LSM9DS1_Init(SPI_HandleTypeDef *hspi1, uint8_t ctrl2) {
	uint8_t ctrl1 = 0x74; // set the magnetic resolution to 16-bit, 20 Hz ODR, UHP mode in X-Y axis
	LSM9DS1_WriteReg(hspi1, LSM9DS1_CTRL_REG1_M, ctrl1);

	/* Change the full-scale range to ±4 Gauss */

	//value to set the full-scale range
	LSM9DS1_WriteReg(hspi1, LSM9DS1_CTRL_REG2_M, ctrl2);

	/* Change the control register 3 to continuous conversion mode */

	uint8_t ctrl3 = 0x00; // value to set the continuous conversion mode
	LSM9DS1_WriteReg(hspi1, LSM9DS1_CTRL_REG3_M, ctrl3);

	uint8_t ctrl4 = 0x0C; // value to set the UHP mode on Z-axis
	LSM9DS1_WriteReg(hspi1, LSM9DS1_CTRL_REG4_M, ctrl4);

	return 1;
}

void LSM9DS1_ReadData(lsm9ds1_t *pLSM9DS1) {
	uint8_t Mag_Data[6];
	Mag_Data[0] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTX_L_M);
	Mag_Data[1] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTX_H_M);
	Mag_Data[2] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTY_L_M);
	Mag_Data[3] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTY_H_M);
	Mag_Data[4] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTZ_L_M);
	Mag_Data[5] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTZ_H_M);

	pLSM9DS1->m_raw_data.mx = (int16_t) ((Mag_Data[1] << 8) | Mag_Data[0]);
	pLSM9DS1->m_raw_data.my = (int16_t) ((Mag_Data[3] << 8) | Mag_Data[2]);
	pLSM9DS1->m_raw_data.mz = (int16_t) ((Mag_Data[5] << 8) | Mag_Data[4]);

	pLSM9DS1->m_sensor_data.mx = (float) (pLSM9DS1->m_raw_data.mx
			* LSM9DS1_SENSITIVITY);
	pLSM9DS1->m_sensor_data.my = (float) (pLSM9DS1->m_raw_data.my
			* LSM9DS1_SENSITIVITY);
	pLSM9DS1->m_sensor_data.mz = (float) (pLSM9DS1->m_raw_data.mz
			* LSM9DS1_SENSITIVITY);

	pLSM9DS1->m_sensor_data.total_mag = sqrt(
			pLSM9DS1->m_sensor_data.mx * pLSM9DS1->m_sensor_data.mx
					+ pLSM9DS1->m_sensor_data.my * pLSM9DS1->m_sensor_data.my
					+ pLSM9DS1->m_sensor_data.mz * pLSM9DS1->m_sensor_data.mz);

	//myDebug("-----Without filter, Sensor Data of Magnetometer-----\r\n");
	myDebug("MAG (uT)\r\n");
	myDebug(" mx = %f \tmy = %f \tmz = %f\r\nTotal Magenotometer = %f uT\r\n",
			pLSM9DS1->m_sensor_data.mx, pLSM9DS1->m_sensor_data.my,
			pLSM9DS1->m_sensor_data.mz, pLSM9DS1->m_sensor_data.total_mag);
}

