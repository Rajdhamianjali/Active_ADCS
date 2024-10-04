/*
 * IMU.h
 *
 *  Created on: Sep 8, 2024
 *      Author: Dell
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "main.h"
#include "Mag_sensor.h"
#include "MPU_sensor.h"
#include "ADCS_Debug.h"
#include "RCFilter.h"

extern SPI_HandleTypeDef hspi1;

extern uint8_t SAT_IMU_REG[17];

typedef struct imu_filtered_data {
	//filtered accelermeter data
	float ax_mps2;
	float ay_mps2;
	float az_mps2;

	//filtered gyroscope data
	float p_rps;
	float q_rps;
	float r_rps;

	//filtered magnotometer
	float mx_ut;
	float my_ut;
	float mz_ut;

	double total_mag;

} imu_filter;

imu_filter IMU_RCFilter();

void IMU_Sensor_Data(MPU6500_t *DEFAULT_MPU6500, lsm9ds1_t *DEFAULT_LSM9DS1);
void IMU_REG_Data();
void IMU_Setup(MPU6500_t *DEFAULT_MPU6500);
imu_filter IMU_Get_Data(MPU6500_t *DEFAULT_MPU6500, lsm9ds1_t *DEFAULT_LSM9DS1);

#endif /* INC_IMU_H_ */
