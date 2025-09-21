#define MPU_ADDR 0x68<<1
#define MPU_WHO_AM_I 0x75
#define MPU_SENS_START 0x3B
#define MPU_SENS_LEN 14
#define MPU_FIFO_EN 0x23
#define MPU_FIFO_SET 0x78
#define MPU_PWR_MGMT_1 0x6B
#define MPU_PWR_MGMT_1_SET 0x00
#define MPU_SMPLRT_DIV 0x19
#define MPU_SMPLRT_DIV_SET 0x07
#define MPU_GYRO_CONFIG	0x1B
#define MPU_GYRO_CONFIG_SET 0x10
#define MPU_I2C_CONFIG 55
#define MPU_I2C_CONFIG_SET 0x02
#define MPU_INTERRUPT_ENABLE 0x38
#define _RAW_DATA_READY 0x01
#define _INT_ANYRD_2CLEAR_ANY_READ 1<<4
#define	_ACTL_LOW 1<<7
#define _OPEN_DRAIN 1<<6
#define _LATCH_INT_EN_UNTIL_CLEARED 1<<5

#include "MPU6050.h"
#include "math.h"
	
static void I2C_writeByte(I2C_HandleTypeDef* i2c, uint8_t devAddr, uint8_t regAddr, uint8_t data){
	HAL_I2C_Mem_Write(i2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}
static uint8_t I2C_readByte(I2C_HandleTypeDef* i2c, uint8_t devAddr, uint8_t regAddr){
	uint8_t data;
	HAL_I2C_Mem_Read(i2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	return data;
}

static uint8_t overloadCheck(const uint8_t dataBuffer[], uint8_t highIndex, uint8_t lowIndex, uint16_t threshold){
	int16_t value = (int16_t)(dataBuffer[highIndex]<<8)|dataBuffer[lowIndex];
	return (value >= threshold ||  value <= -threshold);
};
	
static float convertRawDataToFloat(const uint8_t dataBuffer[], uint8_t highIndex, uint8_t lowIndex, float divider){
return ((float) ((int16_t)(dataBuffer[highIndex]<<8) | dataBuffer[lowIndex])) / divider;
}	

static float compensateError(MPU6050_ERROR_COMPENSATOR compensator, float rawValue){
	return (rawValue + compensator.additive)*compensator.multiplicative;
}

// ФНЧ без нормализации временного интервала
static void LPfilter_IntervalPrepared(float *filteredValue, float unfilteredValue, float normalizedInterval){
// защита от ошибки FPU-coпроцессора 
unfilteredValue = (fabs(unfilteredValue) < 1e-20) ? ((unfilteredValue>=0)? 1e-20 : -1e-20) : unfilteredValue;
*filteredValue = normalizedInterval * unfilteredValue + (1-normalizedInterval)* *filteredValue;	
}


void MPU6050setup(const MPU6050_DATA_STRUCTURE *MPU6050dataStructure){
		I2C_writeByte(MPU6050dataStructure->i2cHandle, MPU6050dataStructure->i2cAddress, MPU_FIFO_EN, MPU_FIFO_SET);
		I2C_writeByte(MPU6050dataStructure->i2cHandle, MPU6050dataStructure->i2cAddress, MPU_PWR_MGMT_1, MPU_PWR_MGMT_1_SET);
		I2C_writeByte(MPU6050dataStructure->i2cHandle, MPU6050dataStructure->i2cAddress, MPU_SMPLRT_DIV, MPU_SMPLRT_DIV_SET);
		I2C_writeByte(MPU6050dataStructure->i2cHandle, MPU6050dataStructure->i2cAddress, MPU_GYRO_CONFIG, MPU_GYRO_CONFIG_SET);	
}
	
void MPU6050_ReceiveIT(MPU6050_DATA_STRUCTURE *MPU6050dataStructure){
	if ( MPU6050dataStructure->i2cHandle->State == 0x20 // Порт готов
		&& MPU6050dataStructure->accDataRequest == MPU6050_RAW_DATA_REQUEST_TRANSLATING) // Принимались данные акселерометра
		MPU6050dataStructure->accDataRequest = MPU6050_RAW_DATA_REQUEST_RECEIVED;
	
	if ( MPU6050dataStructure->i2cHandle->State == 0x20 // Порт готов
		&& MPU6050dataStructure->gyroDataRequest == MPU6050_RAW_DATA_REQUEST_TRANSLATING) // Принимались данные гироскопа
		MPU6050dataStructure->gyroDataRequest = MPU6050_RAW_DATA_REQUEST_RECEIVED;
}

uint8_t MPU6050_GYRO_isReady(MPU6050_DATA_STRUCTURE *MPU6050dataStructure){
	uint8_t ready = MPU6050dataStructure->processedDataReady.gyroDataReady;
	MPU6050dataStructure->processedDataReady.gyroDataReady = 0;
	return ready;
}

uint8_t MPU6050_ACC_isReady(MPU6050_DATA_STRUCTURE *MPU6050dataStructure){
	uint8_t ready = MPU6050dataStructure->processedDataReady.accDataReady;
	MPU6050dataStructure->processedDataReady.accDataReady = 0;
	return ready;
}

void MPU6050_cycle(MPU6050_DATA_STRUCTURE *MPU6050dataStructure, uint64_t micros){	

	#ifdef DEBUG	
		// Для проверки правильности частоты измерений
		if (MPU6050dataStructure->initMicros == 0) MPU6050dataStructure->initMicros = micros;
	#endif	
	{// Запрос данных акселерометра
	
	if (micros - MPU6050dataStructure->accPrevRequestUs >= MPU6050dataStructure->accRequestPeriodUs && // Пора запрашивать ускорения
			MPU6050dataStructure->i2cHandle->State == 0x20 && // Порт свободен
			MPU6050dataStructure->accDataRequest == MPU6050_RAW_DATA_REQUEST_INACTIVE) // Предыдущие данные обработаны)
	{
		if ( HAL_OK == HAL_I2C_Mem_Read_IT(MPU6050dataStructure->i2cHandle, MPU6050dataStructure->i2cAddress, 0x3B, I2C_MEMADD_SIZE_8BIT, MPU6050dataStructure->accRawDataBuffer, 6) ) 
		{
			MPU6050dataStructure->accDataRequest = MPU6050_RAW_DATA_REQUEST_TRANSLATING;
			MPU6050dataStructure->accPrevRequestUs = micros;
		}
	}

	// Обработка данных акселерометра
	if (MPU6050dataStructure->accDataRequest == MPU6050_RAW_DATA_REQUEST_RECEIVED){
		float unfilteredValue, filterNormalizedInterval;
		
		#ifdef DEBUG	
		// Для проверки правильности частоты измерений
		MPU6050dataStructure->accMesCnt ++;
		#endif		

		// Подготовка для цифрового ФНЧ
		filterNormalizedInterval = (float)(micros - MPU6050dataStructure->accPrevFilterUs) / (MPU6050dataStructure->filterTauUs + micros - MPU6050dataStructure->accPrevFilterUs);
		MPU6050dataStructure->accPrevFilterUs = micros;

		 
		if (  overloadCheck(MPU6050dataStructure->accRawDataBuffer, 0, 1, 32000) ||
					overloadCheck(MPU6050dataStructure->accRawDataBuffer, 2, 3, 32000) ||
					overloadCheck(MPU6050dataStructure->accRawDataBuffer, 4, 5, 32000) )
			MPU6050dataStructure->overload.accOverload = 1;
		else 
			MPU6050dataStructure->overload.accOverload = 0;
		 
		unfilteredValue = convertRawDataToFloat(MPU6050dataStructure->accRawDataBuffer, 0, 1, 1638.4);
		unfilteredValue = compensateError(MPU6050dataStructure->compensatorACx, unfilteredValue);
		LPfilter_IntervalPrepared(&(MPU6050dataStructure->ACx), unfilteredValue, filterNormalizedInterval);

		unfilteredValue = convertRawDataToFloat(MPU6050dataStructure->accRawDataBuffer, 2, 3, 1638.4);
		unfilteredValue = compensateError(MPU6050dataStructure->compensatorACy, unfilteredValue);
		LPfilter_IntervalPrepared(&(MPU6050dataStructure->ACy), unfilteredValue, filterNormalizedInterval);

		unfilteredValue = convertRawDataToFloat(MPU6050dataStructure->accRawDataBuffer, 4, 5, 1638.4);
		unfilteredValue = compensateError(MPU6050dataStructure->compensatorACz, unfilteredValue);
		LPfilter_IntervalPrepared(&(MPU6050dataStructure->ACz), unfilteredValue, filterNormalizedInterval);	

		MPU6050dataStructure->processedDataReady.accDataReady = 1; // Можно читать новое ускорение
		MPU6050dataStructure->accDataRequest = MPU6050_RAW_DATA_REQUEST_INACTIVE; // Сырые данные обработаны

	}	
  }
	
	{// Запрос данных гироскопа
	if (micros - MPU6050dataStructure->gyroPrevRequestUs >= MPU6050dataStructure->gyroRequestPeriodUs && // Пора запрашивать угловые скорости
			MPU6050dataStructure->i2cHandle->State == 0x20 && // Порт свободен
			MPU6050dataStructure->gyroDataRequest == MPU6050_RAW_DATA_REQUEST_INACTIVE) // Предыдущие данные обработаны
	{
		if ( HAL_OK == HAL_I2C_Mem_Read_IT(MPU6050dataStructure->i2cHandle, MPU6050dataStructure->i2cAddress, 0x43, I2C_MEMADD_SIZE_8BIT, MPU6050dataStructure->gyroRawDataBuffer, 6) ) 
		{
			MPU6050dataStructure->gyroDataRequest = MPU6050_RAW_DATA_REQUEST_TRANSLATING;
			MPU6050dataStructure->gyroPrevRequestUs = micros;
		}
	}
	
	if (MPU6050dataStructure->gyroDataRequest == MPU6050_RAW_DATA_REQUEST_RECEIVED){
		float unfilteredValue, filterNormalizedInterval;
		
		#ifdef DEBUG	
			// Для проверки правильности частоты измерений
			MPU6050dataStructure->gyroMesCnt ++;
		#endif
		
		// Подготовка для цифрового ФНЧ
		filterNormalizedInterval = (float)(micros - MPU6050dataStructure->gyroPrevFilterUs) / (MPU6050dataStructure->filterTauUs + micros - MPU6050dataStructure->gyroPrevFilterUs);
		MPU6050dataStructure->gyroPrevFilterUs = micros;

		 
		if (  overloadCheck(MPU6050dataStructure->gyroRawDataBuffer, 0, 1, 32000) ||
					overloadCheck(MPU6050dataStructure->gyroRawDataBuffer, 2, 3, 32000) ||
					overloadCheck(MPU6050dataStructure->gyroRawDataBuffer, 4, 5, 32000) )
			MPU6050dataStructure->overload.gyroOverload = 1;
		else 
			MPU6050dataStructure->overload.gyroOverload = 0;
		 
		unfilteredValue = convertRawDataToFloat(MPU6050dataStructure->gyroRawDataBuffer, 0, 1, 32.8);
		unfilteredValue = compensateError(MPU6050dataStructure->compensatorGYx, unfilteredValue);
		LPfilter_IntervalPrepared(&(MPU6050dataStructure->GYx), unfilteredValue, filterNormalizedInterval);

		unfilteredValue = convertRawDataToFloat(MPU6050dataStructure->gyroRawDataBuffer, 2, 3, 32.8);
		unfilteredValue = compensateError(MPU6050dataStructure->compensatorGYy, unfilteredValue);
		LPfilter_IntervalPrepared(&(MPU6050dataStructure->GYy), unfilteredValue, filterNormalizedInterval);

		unfilteredValue = convertRawDataToFloat(MPU6050dataStructure->gyroRawDataBuffer, 4, 5, 32.8);
		unfilteredValue = compensateError(MPU6050dataStructure->compensatorGYz, unfilteredValue);
		LPfilter_IntervalPrepared(&(MPU6050dataStructure->GYz), unfilteredValue, filterNormalizedInterval);	

		MPU6050dataStructure->processedDataReady.gyroDataReady = 1; // Можно читать новую угловую скорость
		MPU6050dataStructure->gyroDataRequest = MPU6050_RAW_DATA_REQUEST_INACTIVE; // Сырые данные обработаны

	}
	}	
	// всё обработали
}


void gyroMeasureOffset(MPU6050_DATA_STRUCTURE *MPU6050dataStructure){
	float gyroXoffs = 0, gyroYoffs = 0, gyroZoffs = 0;
	
	for( uint8_t i = 0; i < 100; i ++){
		if ( HAL_OK == HAL_I2C_Mem_Read(MPU6050dataStructure->i2cHandle, MPU6050dataStructure->i2cAddress, 0x43, I2C_MEMADD_SIZE_8BIT, MPU6050dataStructure->gyroRawDataBuffer, 6, 1000) ){

			gyroXoffs += convertRawDataToFloat(MPU6050dataStructure->gyroRawDataBuffer, 0, 1, 32.8);
			gyroYoffs += convertRawDataToFloat(MPU6050dataStructure->gyroRawDataBuffer, 2, 3, 32.8);
			gyroZoffs += convertRawDataToFloat(MPU6050dataStructure->gyroRawDataBuffer, 4, 5, 32.8);
		}
		HAL_Delay(5);
	}
	
	MPU6050dataStructure->compensatorGYx.additive = -gyroXoffs/100;
	MPU6050dataStructure->compensatorGYy.additive = -gyroYoffs/100;
	MPU6050dataStructure->compensatorGYz.additive = -gyroZoffs/100;
}


#undef MPU_WHO_AM_I
#undef MPU_SENS_START
#undef MPU_SENS_LEN
#undef MPU_FIFO_EN
#undef MPU_FIFO_SET
#undef MPU_PWR_MGMT_1
#undef MPU_PWR_MGMT_1_SET
#undef MPU_SMPLRT_DIV
#undef MPU_SMPLRT_DIV_SET
#undef MPU_GYRO_CONFIG
#undef MPU_GYRO_CONFIG_SET
#undef MPU_I2C_CONFIG
#undef MPU_I2C_CONFIG_SET
#undef INTERRUPT_ENABLE
#undef _RAW_DATA_READY
#undef INT_PIN
#undef _INT_ANYRD_2CLEAR_ANY_READ
#undef	_ACTL_LOW
#undef _OPEN_DRAIN
#undef _LATCH_INT_EN_UNTIL_CLEARED

