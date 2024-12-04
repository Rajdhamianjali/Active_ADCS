/*
 * Bdot.h
 *
 *  Created on: Sep 13, 2024
 *      Author: Dell
 */

#ifndef INC_BDOT_H_
#define INC_BDOT_H_

#include "main.h"
#include "estimator.h"
#include "IMU.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern int count;


//Define Constants
#define desired_roll  0.5f
#define desired_pitch  0.5f
#define desired_yaw  0.5f

#define  desired_Wx  0.5f
#define  desired_Wy  0.5f
#define  desired_Wz  0.5f

#define DT 0.05  //sampling interval: duration between consecutive sensor readings
#define Kp 0.1f  //proportional gain for B-Dot algorithm
#define ERROR_THRESHOLD 0.2f //threshold angular velocity in rad/s

#define MAX_MOMENT_MTQ 0.2f	//max moment for MTQ in Am2
#define MAX_DUTY_CYCLE 28800.0f //Maximum allowed duty cycle

//Global variables
float attitude_error[3];
float angular_error[3];
float Dx, Dy, Dz;

typedef struct mag_moment_data {
	float MomentX;
	float MomentY;
	float MomentZ;
	float Dy;
	float Dz;
	float Dy_per;
	float Dz_per;
} mag_moment;

mag_moment mag_moment_bdot;

void CalAttitudeError(sat_att_combined *pcombined_sat_att);
void CalTorque(imu_filter imu_filter_data, lsm9ds1_t *pBdata, sat_att_combined pcombined_sat_att);
void att_control(sat_att_combined *pcombined_sat_att, imu_filter *pfilt_att);

void MTQ_Enable();
void MTQ_Disable();
void SET_PWM_FORWARD_Y(uint32_t Dy);
void SET_PWM_REVERSE_Y(uint32_t Dy);
void SET_PWM_FORWARD_Z(uint32_t Dz);
void SET_PWM_REVERSE_Z(uint32_t Dz);


#endif /* INC_BDOT_H_ */
