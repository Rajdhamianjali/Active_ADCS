/*
 * MPU_sensor.c
 *
 *  Created on: Sep 7, 2024
 *      Author: Dell
 */

#include "MPU_sensor.h"

//static uint8_t _buffer[14];

/* gets the MPU6500 WHO_AM_I register value, expected to be 0x70 */
uint8_t whoAmI() {
	uint8_t readRegBuffer;
	// read the WHO AM I register
	readRegisters(WHO_AM_I_MPU_ADDR, 1, &readRegBuffer);

	// return the register value
	return readRegBuffer;
}

/* reads registers from MPU6500 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t readAddress, uint8_t numByteToRead,
		uint8_t *r_buffer) {
	MPU_SPI_Read(readAddress, numByteToRead, r_buffer);
}

void MPU_SPI_Read(uint8_t readAddress, uint8_t numByteToRead, uint8_t *r_buffer) {
	MPU_CS(CS_SEL);
	uint8_t tData = readAddress | MPU_READ_CMD;
	HAL_SPI_Transmit(&hspi1, &tData, 1, 100);
	HAL_SPI_Receive(&hspi1, r_buffer, numByteToRead, 100);
	MPU_CS(CS_DES);
}

void MPU_CS(uint8_t state) {
	HAL_GPIO_WritePin(CS_MPU_GPIO_Port, CS_MPU_Pin, state);
}

int MPU_begin(MPU6500_t *pMPU6500) {

	uint8_t addr, val;

	whoAmI();

	if (whoAmI() == WHO_AM_I_MPU_REG_DEFAULT) {

		// configuring MPU6500
		addr = PWR_MGMT_1;
		val = 0x00;
		writeRegister(addr, 1, val);

		// Disable I2C (SPI only)
		addr = USER_CTRL;
		val = 0x10;
		writeRegister(addr, 1, val);

		// Configure DLPF value()
		val = 0x11;
		MPU6500_SetDLPFBandwidth(DLPF_BANDWIDTH_20HZ);

		// Set the full scale ranges
		MPU_writeAccFullScaleRange(pMPU6500,
				pMPU6500->settings.aFullScaleRange);
		MPU_writeGyroFullScaleRange(pMPU6500,
				pMPU6500->settings.gFullScaleRange);
		return 1;
	} else {
		myDebug("WHO AM I Failed!!!-> %x \r\n", whoAmI());
		return 0;
	}
}

/* writes a byte to MPU6500 register given a register address and data */
void writeRegister(uint8_t writeAddress, uint8_t numByteToWrite, uint8_t data) {
	MPU_SPI_Write(writeAddress, numByteToWrite, &data);
	HAL_Delay(10);
}

void MPU_SPI_Write(uint8_t writeAddress, uint8_t numByteToWrite, uint8_t *data) {
	MPU_CS(CS_SEL);
	SPIx_WriteRead(writeAddress);
	while (numByteToWrite >= 0x01) {
		SPIx_WriteRead(*data);
		numByteToWrite--;
		data++;
	}
	MPU_CS(CS_DES);
}

uint8_t SPIx_WriteRead(uint8_t Byte) {
	uint8_t receivedbyte = 0;
	if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t*) &Byte,
			(uint8_t*) &receivedbyte, 1, 0x1000) != HAL_OK) {
		return -1;
	} else {
		return receivedbyte;
	}
}

/* sets the DLPF bandwidth to values other than default */
void MPU6500_SetDLPFBandwidth(DLPFBandwidth bandwidth) {
	writeRegister(ACCEL_CONFIG_2, 1, bandwidth);
	writeRegister(CONFIG, 1, bandwidth);
}

/// @brief Set the accelerometer full scale range
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
/// @param aScale Set 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g
void MPU_writeAccFullScaleRange(MPU6500_t *pMPU6500, uint8_t aScale) {
	// Variable init
	uint8_t addr = ACCEL_CONFIG;
	uint8_t val;

	// Set the value
	switch (aScale) {
	case AFSR_2G:
		pMPU6500->sensorData.aScaleFactor = 16384.0;
		val = 0x00;
		writeRegister(addr, 1, val);
		break;
	case AFSR_4G:
		pMPU6500->sensorData.aScaleFactor = 8192.0;
		val = 0x08;
		writeRegister(addr, 1, val);
		break;
	case AFSR_8G:
		pMPU6500->sensorData.aScaleFactor = 4096.0;
		val = 0x10;
		writeRegister(addr, 1, val);
		break;
	case AFSR_16G:
		pMPU6500->sensorData.aScaleFactor = 2048.0;
		val = 0x18;
		writeRegister(addr, 1, val);
		break;
	default:
		pMPU6500->sensorData.aScaleFactor = 8192.0;
		val = 0x08;
		writeRegister(addr, 1, val);
		break;
	}
}

/// @brief Set the gyroscope full scale range
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
/// @param gScale Set 0 for ±250°/s, 1 for ±500°/s, 2 for ±1000°/s, and 3 for ±2000°/s
void MPU_writeGyroFullScaleRange(MPU6500_t *pMPU6500, uint8_t gScale) {
	// Variable init
	uint8_t addr = GYRO_CONFIG;
	uint8_t val;

	// Set the value
	switch (gScale) {
	case GFSR_250DPS:
		pMPU6500->sensorData.gScaleFactor = 131.0;
		val = 0x00;
		writeRegister(addr, 1, val);
		break;
	case GFSR_500DPS:
		pMPU6500->sensorData.gScaleFactor = 65.5;
		val = 0x08;
		writeRegister(addr, 1, val);
		break;
	case GFSR_1000DPS:
		pMPU6500->sensorData.gScaleFactor = 32.8;
		val = 0x10;
		writeRegister(addr, 1, val);
		break;
	case GFSR_2000DPS:
		pMPU6500->sensorData.gScaleFactor = 16.4;
		val = 0x18;
		writeRegister(addr, 1, val);
		break;
	default:
		pMPU6500->sensorData.gScaleFactor = 65.5;
		val = 0x08;
		writeRegister(addr, 1, val);
		break;
	}
}

/// @brief Find offsets for each axis of gyroscope
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
/// @param numCalPoints Number of data points to average
void MPU_calibrateGyro(MPU6500_t *pMPU6500, uint16_t numCalPoints) {
	// Init
	int32_t x = 0;
	int32_t y = 0;
	int32_t z = 0;
	int32_t Ax = 0;
	int32_t Ay = 0;
	int32_t Az = 0;

	// Zero guard
	if (numCalPoints == 0) {
		numCalPoints = 1;
	}

	// Save specified number of points
	for (uint16_t ii = 0; ii < numCalPoints; ii++) {
		MPU6500_GetData(pMPU6500);

		x += pMPU6500->rawData.gx;
		y += pMPU6500->rawData.gy;
		z += pMPU6500->rawData.gz;

		Ax += pMPU6500->rawData.ax;
		Ay += pMPU6500->rawData.ay;
		Az += pMPU6500->rawData.az;

		HAL_Delay(3);
	}

	// Average the saved data points to find the gyroscope offset
	pMPU6500->gyroCal.x = (float) x / (float) numCalPoints;
	pMPU6500->gyroCal.y = (float) y / (float) numCalPoints;
	pMPU6500->gyroCal.z = (float) z / (float) numCalPoints;

	pMPU6500->gyroCal.Ax = (float) Ax / (float) numCalPoints;
	pMPU6500->gyroCal.Ay = (float) Ay / (float) numCalPoints;
	pMPU6500->gyroCal.Az = (float) Az / (float) numCalPoints;
}

/* read the data, each argument should point to a array for x, y, and z */
void MPU6500_GetData(MPU6500_t *pMPU6500) {
	// grab the data from the MPU6500

	uint8_t _buffer[14];

	_buffer[0] = MPU6500_ReadReg(&hspi1, ACCEL_XOUT_H);
	_buffer[1] = MPU6500_ReadReg(&hspi1, ACCEL_XOUT_L);
	_buffer[2] = MPU6500_ReadReg(&hspi1, ACCEL_YOUT_H);
	_buffer[3] = MPU6500_ReadReg(&hspi1, ACCEL_YOUT_L);
	_buffer[4] = MPU6500_ReadReg(&hspi1, ACCEL_ZOUT_H);
	_buffer[5] = MPU6500_ReadReg(&hspi1, ACCEL_ZOUT_L);

	_buffer[6] = MPU6500_ReadReg(&hspi1, GYRO_XOUT_H);
	_buffer[7] = MPU6500_ReadReg(&hspi1, GYRO_XOUT_L);
	_buffer[8] = MPU6500_ReadReg(&hspi1, GYRO_YOUT_H);
	_buffer[9] = MPU6500_ReadReg(&hspi1, GYRO_YOUT_L);
	_buffer[10] = MPU6500_ReadReg(&hspi1, GYRO_ZOUT_H);
	_buffer[11] = MPU6500_ReadReg(&hspi1, GYRO_ZOUT_L);

	//readRegisters(ACCEL_OUT, 14, _buffer);

	// combine into 16 bit values
	pMPU6500->rawData.ax = (((int16_t) _buffer[0]) << 8) | _buffer[1];
	pMPU6500->rawData.ay = (((int16_t) _buffer[2]) << 8) | _buffer[3];
	pMPU6500->rawData.az = (((int16_t) _buffer[4]) << 8) | _buffer[5];

	pMPU6500->rawData.gx = (((int16_t) _buffer[6]) << 8) | _buffer[7];
	pMPU6500->rawData.gy = (((int16_t) _buffer[8]) << 8) | _buffer[9];
	pMPU6500->rawData.gz = (((int16_t) _buffer[10]) << 8) | _buffer[11];

}

/// @brief Calculate the attitude of the sensor in degrees using a complementary filter
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
void MPU_calcAttitude(MPU6500_t *pMPU6500) {
	// Read processed data
	MPU_readProcessedData(pMPU6500);
}

/// @brief Calculate the real world sensor values
/// @param SPIx Pointer to SPI structure config
/// @param pMPU6500 Pointer to master MPU6500 struct
void MPU_readProcessedData(MPU6500_t *pMPU6500) {
	// Get raw values from the IMU
	MPU6500_GetData(pMPU6500);

	// Compensate for accel offset
	pMPU6500->sensorData.ax = pMPU6500->rawData.ax - 540.2096;
	pMPU6500->sensorData.ay = pMPU6500->rawData.ay - 196.684;
	pMPU6500->sensorData.az = pMPU6500->rawData.az - 1538.457;

// Convert accelerometer values to g's
	pMPU6500->sensorData.ax = pMPU6500->rawData.ax
			/ pMPU6500->sensorData.aScaleFactor;
	pMPU6500->sensorData.ay = pMPU6500->rawData.ay
			/ pMPU6500->sensorData.aScaleFactor;
	pMPU6500->sensorData.az = pMPU6500->rawData.az
			/ pMPU6500->sensorData.aScaleFactor;

// Compensate for gyro offset
	pMPU6500->sensorData.gx = pMPU6500->rawData.gx - (-249.209);
	pMPU6500->sensorData.gy = pMPU6500->rawData.gy - 283.064;
	pMPU6500->sensorData.gz = pMPU6500->rawData.gz - (-3.174);

// Convert gyro values to deg/s
//	pMPU6500->sensorData.gx = (pMPU6500->sensorData.gx
//			/ pMPU6500->sensorData.gScaleFactor);
//	pMPU6500->sensorData.gy = (pMPU6500->sensorData.gy
//			/ pMPU6500->sensorData.gScaleFactor);
//	pMPU6500->sensorData.gz = (pMPU6500->sensorData.gz
//			/ pMPU6500->sensorData.gScaleFactor);

// Convert gyro values to rad/s
	pMPU6500->sensorData.gx = (pMPU6500->sensorData.gx
			/ pMPU6500->sensorData.gScaleFactor) * DEG2RAD;
	pMPU6500->sensorData.gy = (pMPU6500->sensorData.gy
			/ pMPU6500->sensorData.gScaleFactor) * DEG2RAD;
	pMPU6500->sensorData.gz = (pMPU6500->sensorData.gz
			/ pMPU6500->sensorData.gScaleFactor) * DEG2RAD;

	myDebug("-----Without filter, Sensor Data-----\r\n");
	myDebug("ACCEL (m/s^2)\r\n");
	myDebug(" ax = %.0f \tay = %.0f \taz = %.0f \r\n", pMPU6500->sensorData.ax,
			pMPU6500->sensorData.ay, pMPU6500->sensorData.az);
	myDebug("GYRO (rad/s) \r\n");
	myDebug(" gx = %.0f \tgy = %.0f \tgz = %.0f \r\n", pMPU6500->sensorData.gx,
			pMPU6500->sensorData.gy, pMPU6500->sensorData.gz);
}

uint8_t MPU6500_ReadReg(SPI_HandleTypeDef *hspi, uint8_t add) {
	uint8_t val;
	add |= 0x80;  // set the MSB to indicate a read operation
	MPU_CS(CS_SEL);
	HAL_SPI_Transmit(hspi, &add, 1, 100);
	HAL_SPI_Receive(hspi, &val, 1, 100);
	MPU_CS(CS_DES);
	return val;
}

