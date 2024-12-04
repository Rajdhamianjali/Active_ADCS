/*
 * Bdot.c
 *
 *  Created on: Sep 13, 2024
 *      Author: Dell
 */

#ifndef SRC_BDOT_C_
#define SRC_BDOT_C_

#include "Bdot.h"

void CalAttitudeError(sat_att_combined *pcombined_sat_att) {

	float desired_attitude[3] = { desired_roll, desired_pitch, desired_yaw }; //Desired Attitude

	float current_attitude[3] = { pcombined_sat_att->roll,
			pcombined_sat_att->pitch, pcombined_sat_att->yaw }; //Current Attitude

	for (int i = 0; i < 3; i++) {
		attitude_error[i] = desired_attitude[i] - current_attitude[i];
	}

	//print attitude errors
	myDebug(" roll error = %.2f\r\n", attitude_error[0]);
	myDebug(" pitch error = %.2f\r\n", attitude_error[1]);
	myDebug(" yaw error = %.2f\r\n", attitude_error[2]);
}

void att_control(sat_att_combined *pcombined_sat_att, imu_filter *pfilt_att) {

	CalAttitudeError(pcombined_sat_att);

	/*
	 * If the attitude error exceeds the threshold, enable MTQ
	 */

	if (fabs(attitude_error[0]) > ERROR_THRESHOLD
			|| fabs(attitude_error[1]) > ERROR_THRESHOLD
			|| fabs(attitude_error[2]) > ERROR_THRESHOLD) {

		SET_PWM_FORWARD_Y(Dy);
		SET_PWM_FORWARD_Z(Dz);
		myDebug("MTQ Active");

	}

	else {

		MTQ_Disable();
	}

}

/*
 * B-dot algorithm implementation to calculate required magnetic moment
 *
 */
void CalTorque(imu_filter pfilt_att, lsm9ds1_t *pBdata,
		sat_att_combined pcombined_sat_att) {

	float sp = sinf(pcombined_sat_att.roll * DEG2RAD);
	float cp = cosf(pcombined_sat_att.roll * DEG2RAD);
	float st = sinf(pcombined_sat_att.pitch * DEG2RAD);
	float ct = cosf(pcombined_sat_att.pitch * DEG2RAD);

	// Compute angular velocities
	float omega_x = pfilt_att.p_rps
			+ st * (pfilt_att.q_rps * sp + pfilt_att.r_rps * cp);
	float omega_y = pfilt_att.q_rps * cp - pfilt_att.r_rps * sp;
	float omega_z = (pfilt_att.q_rps * sp + pfilt_att.r_rps * cp) / ct;

	float Wx = omega_x;
	float Wy = omega_y;
	float Wz = omega_z;

	myDebug(" wx = %.2f\r\n", Wx);
	myDebug(" wy = %.2f\r\n", Wy);
	myDebug(" wz = %.2f\r\n", Wz);

	angular_error[0] = desired_Wx - Wx;
	angular_error[1] = desired_Wy - Wy;
	angular_error[2] = desired_Wz - Wz;

	if (fabs(angular_error[0]) > ERROR_THRESHOLD
			|| fabs(angular_error[1]) > ERROR_THRESHOLD
			|| fabs(angular_error[2]) > ERROR_THRESHOLD) {

		//magnetic fields from magnetometer
		float Bx = pfilt_att.mx_ut;
		float By = pfilt_att.my_ut;
		float Bz = pfilt_att.mz_ut;

		myDebug(" Bx = %.2f\r\n", Bx);
		myDebug(" By = %.2f\r\n", By);
		myDebug(" Bz = %.2f\r\n", Bz);

		//calculate rate of change of magnetic field (dB/dt)
		float dBx_dt = Wy * Bz - Wz * By;
		float dBy_dt = Wz * Bx - Wx * Bz;
		float dBz_dt = Wx * By - Wy * Bx;

		myDebug("Desired Magnetic Moment\n");
		//calculate the desired magnetic moment

		mag_moment_bdot.MomentX = -Kp * dBx_dt;
		mag_moment_bdot.MomentY = -Kp * dBy_dt;
		mag_moment_bdot.MomentZ = -Kp * dBz_dt;

		myDebug(" MomentX = %.2f\r\n", mag_moment_bdot.MomentX);
		myDebug(" MomentY = %.2f\r\n", mag_moment_bdot.MomentY);
		myDebug(" MomentZ = %.2f\r\n", mag_moment_bdot.MomentZ);
		myDebug(" MAX_MOMENT_MTQ = %.2f\r\n", MAX_MOMENT_MTQ);
		myDebug(" MAX_DUTY_CYCLE = %.2f\r\n", MAX_DUTY_CYCLE);

		mag_moment_bdot.MomentY = 0.2;
		mag_moment_bdot.MomentZ = 0.2;

		// Constrain magnetic moments within allowable limits
//		MomentY = fminf(fmaxf(MomentY, -MAX_MOMENT_MTQ), MAX_MOMENT_MTQ);
//		MomentZ = fminf(fmaxf(MomentZ, -MAX_MOMENT_MTQ), MAX_MOMENT_MTQ);

//calculate PWM duty cycles based on maximum moment

		float maxDutyCycle = 28800.00;

		mag_moment_bdot.Dy = (mag_moment_bdot.MomentY / MAX_MOMENT_MTQ)
				* MAX_DUTY_CYCLE;
		mag_moment_bdot.Dz = (mag_moment_bdot.MomentZ / MAX_MOMENT_MTQ)
				* MAX_DUTY_CYCLE;

		mag_moment_bdot.Dy_per = (mag_moment_bdot.Dy / maxDutyCycle);
		mag_moment_bdot.Dz_per = (mag_moment_bdot.Dz / maxDutyCycle);

		myDebug("Required Duty Cycle\n");
		myDebug(" Dy = %.2f\r\n", mag_moment_bdot.Dy);
		myDebug(" Dz = %.2f\r\n", mag_moment_bdot.Dz);
		myDebug("----- MTQ enabled !!! -----\n");

		// Constrain PWM duty cycle to allowable range
//		Dy = fminf(fmaxf(Dy, 0.0f), MAX_DUTY_CYCLE);
//		Dz = fminf(fmaxf(Dz, 0.0f), MAX_DUTY_CYCLE);

		// Apply PWM based on the sign of the moments

		HAL_TIM_Base_Start_IT(&htim1);

//		if (mag_moment_bdot.MomentY > mag_moment_bdot.MomentZ) {
		if (mag_moment_bdot.MomentY < 0) {
			SET_PWM_REVERSE_Y(fabs(mag_moment_bdot.Dy));
		} else {
			SET_PWM_FORWARD_Y((int) mag_moment_bdot.Dy);
		}
		myDebug("----- Both ON !!! -----\n");
	}
	if (mag_moment_bdot.MomentZ < 0) {
		SET_PWM_REVERSE_Z(fabs(mag_moment_bdot.Dz));
	} else {
		SET_PWM_FORWARD_Z((int) mag_moment_bdot.Dz);
	}

	return;
}

// Function to enable MTQ_OCP
void MTQ_Enable() {
	HAL_GPIO_WritePin(GPIOB, MTQEN_5V_Pin, SET);
}

// Function to disable MTQ_OCP
void MTQ_Disable() {
	HAL_GPIO_WritePin(GPIOB, MTQEN_5V_Pin, RESET);
}

// Function to set PWM duty cycle for Y-axis MTQ
void SET_PWM_FORWARD_Z(uint32_t Dy) {

	TIM3->CCR1 = Dy;
	TIM3->CCR2 = 0;
	MTQ_Enable();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

}

void SET_PWM_REVERSE_Z(uint32_t Dy) {

	TIM3->CCR1 = 0;
	TIM3->CCR2 = Dy;
	MTQ_Enable();
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void SET_PWM_FORWARD_Y(uint32_t Dz) {

	TIM4->CCR3 = 0;
	TIM4->CCR4 = Dz;
	MTQ_Enable();
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

}

void SET_PWM_REVERSE_Y(uint32_t Dz) {

	TIM4->CCR3 = Dz;
	TIM4->CCR4 = 0;
	MTQ_Enable();
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

}

#endif /* SRC_BDOT_C_ */
