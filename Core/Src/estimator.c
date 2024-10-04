/*
 * estimator.c
 *
 *  Created on: Sep 9, 2024
 *      Author: Dell
 */
#include "estimator.h"

#define G_MPS2 				9.8100000000f

#define SAMPLE_TIME_MS_USB_ 		1000
#define COMP_FILT_ALPHA 		0.0500000000f

#define KALMAN_PREDICT_PERIOD_MS 	10
#define KALMAN_UPDATE_PERIOD_MS 	100

#define epsilon 1e-6f

extern sat_attitude attitude_sat;
extern sat_att_combined combined_sat_att;
extern imu_filter imu_filter_data;

void Attitude_genEstimate(imu_filter *filt, sat_attitude *att) {
	float phiHat_deg_ = 0.0f;
	float thetaHat_deg_ = 0.0f;

	/*Calculate roll (phi) and pitch(theta) angle estimates using filtered accelerometer readings*/
	phiHat_deg_ = atanf(filt->ay_mps2 / (filt->az_mps2 + epsilon)) * RAD2DEG;
	thetaHat_deg_ = asinf(filt->ax_mps2 / G_MPS2) * RAD2DEG;

//	phiHat_deg_ = atanf(filt->ay_mps2 / filt->az_mps2);
//	thetaHat_deg_ = asinf(filt->ax_mps2 / G_MPS2);

//	myDebug("Accel phiHat_deg : %.2f\n", (double) phiHat_deg_);
//	myDebug("Accel thetahat_deg : %.2f\n", (double) thetaHat_deg_);

	att->accel_phiHat = phiHat_deg_;
	att->accel_thetaHat = thetaHat_deg_;

	phiHat_deg_ = 0.0f;    //just try need to be actually on code
	thetaHat_deg_ = 0.0f;

//Transform body rates to Euler rates to get estimate of roll and pitch angles using filtered gyroscope reading
	float phiDot_rps = filt->p_rps
			+ tanf(thetaHat_deg_ * DEG2RAD)
					* (sinf(phiHat_deg_ * DEG2RAD) * filt->q_rps
							+ cosf(phiHat_deg_ * DEG2RAD) * filt->r_rps);
	float thetaDot_rps = cosf(phiHat_deg_ * DEG2RAD) * filt->q_rps
			- sinf(phiHat_deg_ * DEG2RAD) * filt->r_rps;

	//Integrate Euler rates to get estimate of roll and pitch angles
	phiHat_deg_ = (phiHat_deg_ * DEG2RAD
			+ (SAMPLE_TIME_MS_USB_ / 1000.0f) * phiDot_rps) * RAD2DEG;
	thetaHat_deg_ = (thetaHat_deg_ * DEG2RAD
			+ (SAMPLE_TIME_MS_USB_ / 1000.0F) * thetaDot_rps) * RAD2DEG;

//	myDebug("Gyro phiHat_deg: %.2f\n", (double) phiHat_deg_);
//	myDebug("Gyro thetaHat_deg: %.2f\n", (double) thetaHat_deg_);

	att->gyro_phiHat = phiHat_deg_;
	att->gyro_thetaHat = thetaHat_deg_;
	return;
}

void Attitude_compleEstimate(imu_filter *filt, sat_att_combined *att) {

//	float thetaHat_rad_comb = 0.0f;
//	float phiHat_rad_comb = 0.0f;

	float thetaHat_rad_comb = att->pitch;   //use just to try
	float phiHat_rad_comb = att->roll;

	/*Calculate roll (phi) and pitch(theta) angle estimates using filtered accelerometer readings*/
	float phiHat_acc_rad = atanf(filt->ay_mps2 / (filt->az_mps2 + epsilon));
	float thetaHat_acc_rad = asinf(filt->ax_mps2 / G_MPS2);

//	myDebug("accel phiHat_rad : %.2f\n", (double) phiHat_acc_rad);
//	myDebug("accel thetaHat_rad : %.2f\n", (double) thetaHat_acc_rad);

//Transform body rates to Euler rates to get estimate of roll and pitch angles using filtered gyroscope readings
	float phiDot_rps = filt->p_rps
			+ tanf(thetaHat_rad_comb)
					* (sinf(phiHat_rad_comb) * filt->q_rps
							+ cosf(phiHat_rad_comb) * filt->r_rps);
	float thetaDot_rps = cosf(phiHat_rad_comb) * filt->q_rps
			- sinf(phiHat_rad_comb) * filt->r_rps;

//	myDebug("Gryo phidot_rps : %.2f\n", (double) phiDot_rps);
//	myDebug("Gyro thetadot_rps : %.2f\n", (double) thetaDot_rps);

//Combining Accel and Gyro data for complementary filter
	phiHat_rad_comb = (COMP_FILT_ALPHA * phiHat_acc_rad
			+ (1.0f - COMP_FILT_ALPHA)
					* (phiHat_rad_comb
							+ (SAMPLE_TIME_MS_USB_ / 1000.0f) * phiDot_rps));
	thetaHat_rad_comb = (COMP_FILT_ALPHA * thetaHat_acc_rad
			+ (1.0f - COMP_FILT_ALPHA)
					* (thetaHat_rad_comb
							+ (SAMPLE_TIME_MS_USB_ / 1000.0f) * thetaDot_rps));
//	att->pitch = RAD2DEG * thetaHat_rad_comb;
//	att->roll = RAD2DEG * phiHat_rad_comb;

	att->pitch = thetaHat_rad_comb;
	att->roll = phiHat_rad_comb;
	myDebug("\nComplementary filter roll: %.2f rad\n", (double) att->roll);
	myDebug("Complementary filter pitch: %.2f rad\n", (double) att->pitch);

}

void Attitude_ekfEstimate(imu_filter *filt, sat_att_combined *att) {

	float KALMAN_P_INIT = 0.1f;
	float KALMAN_Q = 0.001f;
	float KALMAN_R = 0.011f;
	float KalmanQ[2] = { KALMAN_Q, KALMAN_Q };
	float KalmanR[3] = { KALMAN_R, KALMAN_R, KALMAN_R };
	float Kalman_P_Init[2] = { KALMAN_P_INIT, KALMAN_P_INIT };
	Eekf ekf_1;

	//Filtered accelerometer measurement
	float ax_mps2 = 0.0f;
	float ay_mps2 = 0.0f;
	float az_mps2 = 0.0f;
	//Filtered gyroscope measurement
	float p_rps = 0.0f;
	float q_rps = 0.0f;
	float r_rps = 0.0f;

	//Remapping axis data of Accel and Gyro
	ax_mps2 = (filt->ax_mps2);
	ay_mps2 = filt->ay_mps2;
	az_mps2 = (filt->az_mps2);
	p_rps = (filt->p_rps);
	q_rps = filt->q_rps;
	r_rps = (filt->r_rps);

	//Initialize kalman filter
	EKF_Init(&ekf_1, Kalman_P_Init, KalmanQ, KalmanR);
	//Prediction step using filtered gyro data
	EKF_Predict(&ekf_1, p_rps, q_rps, r_rps, 0.001f * KALMAN_PREDICT_PERIOD_MS);

	//Update step using Accel data
	EKF_Update(&ekf_1, ax_mps2, ay_mps2, az_mps2);

	att->pitch = ekf_1.theta_r;
	att->roll = ekf_1.phi_r;
	float Xm = imu_filter_data.mx_ut * cos(att->pitch)
			- imu_filter_data.my_ut * sin(att->roll) * sin(att->pitch)
			+ imu_filter_data.mz_ut * cos(att->roll) * sin(att->pitch);

	float Ym = imu_filter_data.my_ut * cos(att->roll)
			+ imu_filter_data.mz_ut * sin(att->roll);

	att->yaw = atan2(Ym, Xm);

	att->pitch = RAD2DEG * ekf_1.theta_r;
	att->roll = RAD2DEG * ekf_1.phi_r;
	att->yaw = RAD2DEG * att->yaw;

	myDebug("\nEKF Update : pitch : %.2f deg\n", (double) att->pitch);
	myDebug("EKF Update : roll : %.2f deg\n", (double) att->roll);
	myDebug("EKF Update : yaw : %.2f deg\n", (double) att->yaw);
}
void process_IMU_filt(imu_filter filt_imu) {
	Attitude_genEstimate(&filt_imu, &attitude_sat);
	Attitude_compleEstimate(&filt_imu, &combined_sat_att);
	Attitude_ekfEstimate(&filt_imu, &combined_sat_att);

}

