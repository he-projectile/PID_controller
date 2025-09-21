#ifndef __MPU9255_H
#define __MPU9255_H

#include "main.h"

typedef enum{
	MPU6050_STATE_INACTIVE = 0,
	MPU6050_STATE_ACTIVE,
	MPU6050_STATE_ERROR	
} MPU6050_STATE;

typedef enum{
	MPU6050_RAW_DATA_REQUEST_INACTIVE = 0,
	MPU6050_RAW_DATA_REQUEST_TRANSLATING,
	MPU6050_RAW_DATA_REQUEST_RECEIVED,
} MPU6050_RAW_DATA_REQUEST;

typedef struct{
	uint8_t accOverload : 1;
	uint8_t gyroOverload : 1;	
} MPU6050_OVERLOAD;

typedef struct{
	uint8_t accDataReady : 1;
	uint8_t gyroDataReady : 1;
} MPU6050_PROCESSED_DATA_READY;

typedef struct{
	float multiplicative;
	float additive;
} MPU6050_ERROR_COMPENSATOR;

typedef struct {
	MPU6050_STATE MPU6050state;
	uint8_t i2cAddress;
	MPU6050_OVERLOAD overload;
	float ACx, ACy, ACz, GYx, GYy, GYz;
	MPU6050_ERROR_COMPENSATOR \
		compensatorACx, compensatorACy, compensatorACz, \
		compensatorGYx, compensatorGYy, compensatorGYz;
	MPU6050_PROCESSED_DATA_READY processedDataReady;
	MPU6050_RAW_DATA_REQUEST accDataRequest, gyroDataRequest;
	uint16_t accRequestPeriodUs, gyroRequestPeriodUs;
	uint64_t accPrevRequestUs, gyroPrevRequestUs;
	uint16_t filterTauUs;
	uint64_t accPrevFilterUs, gyroPrevFilterUs;
	uint8_t accRawDataBuffer[6], gyroRawDataBuffer[6];
	
	#ifdef DEBUG	
		uint16_t accMesCnt, gyroMesCnt; uint64_t initMicros;
	#endif
	
	I2C_HandleTypeDef *i2cHandle;
	
} MPU6050_DATA_STRUCTURE;

void MPU6050setup(const MPU6050_DATA_STRUCTURE *dataStructure);
void MPU6050_ReceiveIT(MPU6050_DATA_STRUCTURE *dataStructure);
uint8_t MPU6050_GYRO_isReady(MPU6050_DATA_STRUCTURE *dataStructure);
uint8_t MPU6050_ACC_isReady(MPU6050_DATA_STRUCTURE *dataStructure);
void MPU6050_cycle(MPU6050_DATA_STRUCTURE *dataStructure, uint64_t micros);
void gyroMeasureOffset(MPU6050_DATA_STRUCTURE *dataStructure);

#endif /* __MPU9255_H */
