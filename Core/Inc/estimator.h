/*
 * estimator.h
 *
 *  Created on: Sep 9, 2024
 *      Author: Dell
 */

#ifndef INC_ESTIMATOR_H_
#define INC_ESTIMATOR_H_

#include "IMU.h"
#include "EKF.h"
#include "ADCS_Debug.h"
#include "main.h"

typedef struct sat_attitude_ {
	float accel_phiHat;		//accel roll data
	float accel_thetaHat;	//accel pitch data
	float gyro_phiHat;		//gyro roll data
	float gyro_thetaHat;	//gyro pitch data
} sat_attitude;

typedef struct sat_att_combined_ {
	float roll;
	float pitch;
	float yaw;
} sat_att_combined;

void Attitude_genEstimate(imu_filter *filt, sat_attitude *att);

void Attitude_compleEstimate(imu_filter *filt, sat_att_combined *att);

void Attitude_ekfEstimate(imu_filter *filt, sat_att_combined *att);

void process_IMU_filt(imu_filter filt_imu);

#endif /* INC_ESTIMATOR_H_ */
