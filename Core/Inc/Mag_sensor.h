/*
 * Mag_sensor.h
 *
 *  Created on: Sep 7, 2024
 *      Author: Dell
 */

#ifndef INC_MAG_SENSOR_H_
#define INC_MAG_SENSOR_H_

#include "main.h"
#include "ADCS_Debug.h"
#include "math.h"

extern SPI_HandleTypeDef hspi1;
extern float IMU_SEN_DATA[9];

#define LSM9DS1_CTRL_REG1_M		0x20
#define LSM9DS1_CTRL_REG2_M		0x21
#define LSM9DS1_CTRL_REG3_M		0x22
#define LSM9DS1_CTRL_REG4_M		0x23
#define LSM9DS1_SENSITIVITY		((float)0.058f) //±16 Gauss and a resolution of 16-bits

#define LSM9DS1_OUTX_L_M 0x28
#define LSM9DS1_OUTY_L_M 0x2A
#define LSM9DS1_OUTZ_L_M 0x2C
#define LSM9DS1_OUTX_H_M 0x29
#define LSM9DS1_OUTY_H_M 0x2B
#define LSM9DS1_OUTZ_H_M 0x2D

//Magnetometer Functions
typedef struct LSM9DS1 {

	struct Mag_RawData {
		int16_t mx, my, mz;
	} m_raw_data;

	struct Mag_SensorData {
		float mx, my, mz;
		double total_mag;
	} m_sensor_data;

} lsm9ds1_t;

void LSM9DS1_ReadData(lsm9ds1_t *pLSM9DS1);

uint8_t LSM9DS1_ReadReg(SPI_HandleTypeDef *hspi1, uint8_t add);
void LSM9DS1_WriteReg(SPI_HandleTypeDef *hspi1, uint8_t add, uint8_t val);
int LSM9DS1_Init(SPI_HandleTypeDef *hspi1, uint8_t ctrl2);

#endif /* INC_MAG_SENSOR_H_ */
