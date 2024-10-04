/*
 * MPU_sensor.h
 *
 *  Created on: Sep 7, 2024
 *      Author: Dell
 */

#ifndef INC_MPU_SENSOR_H_
#define INC_MPU_SENSOR_H_

#include "main.h"
#include "ADCS_Debug.h"
#include "math.h"

extern SPI_HandleTypeDef hspi1;
extern uint8_t tdata;
extern uint8_t rdata;
extern float IMU_SEN_DATA[9];

#define WHO_AM_I_MPU_ADDR				0x75
#define WHO_AM_I_MPU_REG_DEFAULT		0x70

#define MPU_READ_CMD					0x80
#define PWR_MGMT_1						0x6B
#define USER_CTRL						0x6A

#define ACCEL_OUT 0x3B
#define GYRO_OUT 0x43
#define TEMP_OUT 0x41

#define ACCEL_CONFIG					0X1C
#define ACCEL_CONFIG_2					0x1D
#define CONFIG							0x1A
#define GYRO_CONFIG						0x1B

#define ACCEL_XOUT_H      0x3B
#define ACCEL_XOUT_L      0x3C
#define ACCEL_YOUT_H      0x3D
#define ACCEL_YOUT_L      0x3E
#define ACCEL_ZOUT_H      0x3F
#define ACCEL_ZOUT_L      0x40

#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48

typedef struct MPU_6500 {
	struct RawData {
		int16_t ax, ay, az, gx, gy, gz;
	} rawData;

	struct SensorData {
		float aScaleFactor, gScaleFactor;
		float ax, ay, az, gx, gy, gz;
	} sensorData;

	struct GyroCal {
		float x, y, z;
		float Ax, Ay, Az;
	} gyroCal;

	struct Attitude {
		float tau, dt;
		float r, p, y;
	} attitude;

	struct Settings {
		uint8_t aFullScaleRange, gFullScaleRange;
		GPIO_TypeDef *CS_PORT;
		uint8_t CS_PIN;
	} settings;
} MPU6500_t;

// Full scale ranges
enum gyroscopeFullScaleRange {
	GFSR_250DPS, GFSR_500DPS, GFSR_1000DPS, GFSR_2000DPS
};

enum accelerometerFullScaleRange {
	AFSR_2G, AFSR_4G, AFSR_8G, AFSR_16G
};

typedef enum DLPFBandwidth_ {
	DLPF_BANDWIDTH_250HZ = 0,
	DLPF_BANDWIDTH_184HZ,
	DLPF_BANDWIDTH_92HZ,
	DLPF_BANDWIDTH_41HZ,
	DLPF_BANDWIDTH_20HZ,
	DLPF_BANDWIDTH_10HZ,
	DLPF_BANDWIDTH_5HZ
} DLPFBandwidth;

#define RAD2DEG 57.2957795131
#define DEG2RAD 0.01745329251

uint8_t MPU6500_ReadReg(SPI_HandleTypeDef *hspi, uint8_t add);

void MPU_readProcessedData(MPU6500_t *pMPU6500);
void MPU_calcAttitude(MPU6500_t *pMPU6500);

void MPU6500_GetData(MPU6500_t *pMPU6500);
void MPU_calibrateGyro(MPU6500_t *pMPU6500, uint16_t numCalPoints);

void MPU_writeAccFullScaleRange(MPU6500_t *pMPU6500, uint8_t aScale);
void MPU_writeGyroFullScaleRange(MPU6500_t *pMPU6500, uint8_t gScale);
void MPU6500_SetDLPFBandwidth(DLPFBandwidth bandwidth);
uint8_t SPIx_WriteRead(uint8_t Byte);
void MPU_SPI_Write(uint8_t writeAddress, uint8_t numByteToWrite, uint8_t *data);
void writeRegister(uint8_t writeAddress, uint8_t numByteToWrite, uint8_t data);
int MPU_begin(MPU6500_t *pMPU6500);

void MPU_CS(uint8_t state);
void MPU_SPI_Read(uint8_t readAddress, uint8_t numByteToRead, uint8_t *r_buffer);
void readRegisters(uint8_t readAddress, uint8_t numByteToRead, uint8_t *buffer);
uint8_t whoAmI();

#endif /* INC_MPU_SENSOR_H_ */
