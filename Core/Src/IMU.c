/*
 * IMU.c
 *
 *  Created on: Sep 8, 2024
 *      Author: Dell
 */

#include "IMU.h"

void IMU_Setup(MPU6500_t *DEFAULT_MPU6500) {

	if (LSM9DS1_Init(&hspi1, 0x60) == 1) { // ±16 Gauss full scale, 20Hz ODR, Continuous conversion mode

		myDebug("\nLSM9DS1 Initialization Complete..\n");

		HAL_Delay(500);

		if (MPU_begin(DEFAULT_MPU6500) != 1) {
			myDebug("MPU6500 Initialization Failed..\n");
		} else {
			myDebug("MPU6500 Initialization Complete..\n");

			HAL_Delay(500);
			//	 Calibrate the IMU
//			myDebug("Now, Calibrating MPU6500.\r\n");
//			HAL_Delay(1);
//			MPU_calibrateGyro(DEFAULT_MPU6500, 2000);
//
//			HAL_Delay(500);
//			myDebug("Calibration  Complete!!!\n");
//			myDebug("******************************\r\n");
		}
	} else {
		myDebug("LSM9DS1 Initialization Failed..\n");
	}
}

void IMU_REG_Data() {
	//must be same as the variable '_buffer' of 'MPU6500_GetData()'
	SAT_IMU_REG[0] = MPU6500_ReadReg(&hspi1, ACCEL_XOUT_H);
	SAT_IMU_REG[1] = MPU6500_ReadReg(&hspi1, ACCEL_XOUT_L);
	SAT_IMU_REG[2] = MPU6500_ReadReg(&hspi1, ACCEL_YOUT_H);
	SAT_IMU_REG[3] = MPU6500_ReadReg(&hspi1, ACCEL_YOUT_L);
	SAT_IMU_REG[4] = MPU6500_ReadReg(&hspi1, ACCEL_ZOUT_H);
	SAT_IMU_REG[5] = MPU6500_ReadReg(&hspi1, ACCEL_ZOUT_L);

	SAT_IMU_REG[6] = MPU6500_ReadReg(&hspi1, GYRO_XOUT_H);
	SAT_IMU_REG[7] = MPU6500_ReadReg(&hspi1, GYRO_XOUT_L);
	SAT_IMU_REG[8] = MPU6500_ReadReg(&hspi1, GYRO_YOUT_H);
	SAT_IMU_REG[9] = MPU6500_ReadReg(&hspi1, GYRO_YOUT_L);
	SAT_IMU_REG[10] = MPU6500_ReadReg(&hspi1, GYRO_ZOUT_H);
	SAT_IMU_REG[11] = MPU6500_ReadReg(&hspi1, GYRO_ZOUT_L);

	//must be same as the 'Mag_Data[]' of 'LSM9DS1_ReadData'
	SAT_IMU_REG[12] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTX_L_M);
	SAT_IMU_REG[13] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTX_H_M);
	SAT_IMU_REG[14] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTY_L_M);
	SAT_IMU_REG[15] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTY_H_M);
	SAT_IMU_REG[16] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTZ_L_M);
	SAT_IMU_REG[17] = LSM9DS1_ReadReg(&hspi1, LSM9DS1_OUTZ_H_M);
}

void IMU_Sensor_Data(MPU6500_t *DEFAULT_MPU6500, lsm9ds1_t *DEFAULT_LSM9DS1) {

	// combined of MPU, MAG and all non filtered sensor data
	IMU_SEN_DATA[0] = DEFAULT_MPU6500->sensorData.ax;
	IMU_SEN_DATA[1] = DEFAULT_MPU6500->sensorData.ay;
	IMU_SEN_DATA[2] = DEFAULT_MPU6500->sensorData.az;

	IMU_SEN_DATA[3] = DEFAULT_MPU6500->sensorData.gx;
	IMU_SEN_DATA[4] = DEFAULT_MPU6500->sensorData.gy;
	IMU_SEN_DATA[5] = DEFAULT_MPU6500->sensorData.gz;

	IMU_SEN_DATA[6] = DEFAULT_LSM9DS1->m_sensor_data.mx;
	IMU_SEN_DATA[7] = DEFAULT_LSM9DS1->m_sensor_data.my;
	IMU_SEN_DATA[8] = DEFAULT_LSM9DS1->m_sensor_data.mz;
}

imu_filter IMU_RCFilter() {
	RCFilter lpfAcc[3];
	RCFilter lpfGyr[3];
	RCFilter lpfMag[3];

	imu_filter filt_imu;

	for (int n = 0; n < 3; n++) {
		RCFilter_Init(&lpfAcc[n], 5.0f, 0.01f);
		RCFilter_Init(&lpfGyr[n], 25.0f, 0.01f);
		RCFilter_Init(&lpfMag[n], 1.0f, 0.01f);
	}

	/* Filter accelerometer data */
	RCFilter_Update(&lpfAcc[0], IMU_SEN_DATA[0]); // sensor data ax
	RCFilter_Update(&lpfAcc[1], IMU_SEN_DATA[1]); // sensor data ay
	RCFilter_Update(&lpfAcc[2], IMU_SEN_DATA[2]);	// sensor data az

	/* Filter gyroscope data */
	RCFilter_Update(&lpfGyr[0], IMU_SEN_DATA[3]);	//sensor data gx
	RCFilter_Update(&lpfGyr[1], IMU_SEN_DATA[4]);	//sensor data gy
	RCFilter_Update(&lpfGyr[2], IMU_SEN_DATA[5]);	//sensor data gz

	/* Filter magmetometer data */
	RCFilter_Update(&lpfMag[0], IMU_SEN_DATA[6]);   // mx
	RCFilter_Update(&lpfMag[1], IMU_SEN_DATA[7]);   // my
	RCFilter_Update(&lpfMag[2], IMU_SEN_DATA[8]);   // mz

	//Filtered accelerometer measurement
	filt_imu.ax_mps2 = lpfAcc[0].out[0];
	filt_imu.ay_mps2 = lpfAcc[1].out[0];
	filt_imu.az_mps2 = lpfAcc[2].out[0];
	//Filtered Gyroscope measurement
	filt_imu.p_rps = lpfGyr[0].out[0];
	filt_imu.q_rps = lpfGyr[1].out[0];
	filt_imu.r_rps = lpfGyr[2].out[0];
	//Filtered Magnetometer measurement
	filt_imu.mx_ut = lpfMag[0].out[0];
	filt_imu.my_ut = lpfMag[1].out[0];
	filt_imu.mz_ut = lpfMag[2].out[0];

	// if no use of filter
	filt_imu.ax_mps2 = IMU_SEN_DATA[0];
	filt_imu.ay_mps2 = IMU_SEN_DATA[1];
	filt_imu.az_mps2 = IMU_SEN_DATA[2];

	filt_imu.p_rps = IMU_SEN_DATA[3];
	filt_imu.q_rps = IMU_SEN_DATA[4];
	filt_imu.r_rps = IMU_SEN_DATA[5];

	filt_imu.mx_ut = IMU_SEN_DATA[6];
	filt_imu.my_ut = IMU_SEN_DATA[7];
	filt_imu.mz_ut = IMU_SEN_DATA[8];

	filt_imu.total_mag = sqrt(
			filt_imu.mx_ut * filt_imu.mx_ut + filt_imu.my_ut * filt_imu.my_ut
					+ filt_imu.mz_ut * filt_imu.mz_ut);

	myDebug("\n-----RC filtered, Sensor Data-----\r\n");
	myDebug("ACCEL (m/s^2)\r\n");
	myDebug(" ax = %.2f \tay = %.2f \taz = %.2f \r\n", filt_imu.ax_mps2,
			filt_imu.ay_mps2, filt_imu.az_mps2);
	myDebug("GYRO (rad/s) \r\n");
	myDebug(" gx = %f \tgy = %f \tgz = %f \r\n", filt_imu.p_rps, filt_imu.q_rps,
			filt_imu.r_rps);
	myDebug("MAG (uT) \r\n");
	myDebug(" mx = %f \tmy = %f \tmz = %f \r\nTotal Magenotometer = %f uT\r\n",
			filt_imu.mx_ut, filt_imu.my_ut, filt_imu.mz_ut, filt_imu.total_mag);

	return filt_imu;
}

imu_filter IMU_Get_Data(MPU6500_t *DEFAULT_MPU6500, lsm9ds1_t *DEFAULT_LSM9DS1) {
	IMU_REG_Data();   //register data
	MPU_calcAttitude(DEFAULT_MPU6500);	//calculate ACC, GRYO sensor data
	LSM9DS1_ReadData(DEFAULT_LSM9DS1);	//calculate MAG sensor data
	IMU_Sensor_Data(DEFAULT_MPU6500, DEFAULT_LSM9DS1); // sensor data of IMU without filter
	return (IMU_RCFilter());
}

