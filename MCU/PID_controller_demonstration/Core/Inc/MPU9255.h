

#ifndef __MPU9255_H
#define __MPU9255_H

#include "main.h"

void MPU9255setup(I2C_HandleTypeDef* i2c, float* _ACx, float* _ACy, float* _ACz, float* _GYx,\
	float* _GYy, float* _GYz, uint8_t *overload);
void MPU9255_IT(void);
uint8_t MPU9255_ACC_GYRO_isReady(void);
uint8_t MPU9255_cycle(uint64_t micros);
void gyroCalib(void);

#endif /* __MPU9255_H */
