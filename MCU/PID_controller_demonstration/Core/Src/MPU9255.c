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
#define INTERRUPT_ENABLE 0x38
#define _RAW_DATA_READY 0x01
#define INT_PIN 55
#define _INT_ANYRD_2CLEAR_ANY_READ 1<<4
#define	_ACTL_LOW 1<<7
#define _OPEN_DRAIN 1<<6
#define _LATCH_INT_EN_UNTIL_CLEARED 1<<5
#define ACC_GYRO_READ_PERIOD_US 4500

#define accXoffs 0
#define accYoffs 0
#define accZoffs 0

#define accXmult 1
#define accYmult 0.9975
#define accZmult 0.992

//#define gyroXoffs 1.5807 -2.85640097
//#define gyroYoffs 1.6857 1.79603696
//#define gyroZoffs 0.3176 -0.427743942

#define gyroXmult 1
#define gyroYmult 1
#define gyroZmult 1

#include "MPU9255.h"
#include "math.h"

static I2C_HandleTypeDef* mpu9255i2c;
static float* ACx, *ACy, *ACz, *GYx, *GYy, *GYz;
static uint8_t i2cAccReceivedFlag = 0, i2cMagReceivedFlag = 0;
static uint8_t MPUdata[16]={0};
static uint64_t magTime=0, accGyroTime = 0;
//static const uint8_t ASAX=178, ASAY=178, ASAZ=167;
static uint8_t magDataReady = 0, accGyroDataReady = 0;
static uint8_t* overload;

static float gyroXoffs = -2.85640097;
static float gyroYoffs = 1.79603696;
static float gyroZoffs = -0.427743942;

static float filterTauUs = 1000;
	
static void I2C_writeByte(I2C_HandleTypeDef* i2c, uint8_t devAddr, uint8_t regAddr, uint8_t data){
	HAL_I2C_Mem_Write(i2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}
static uint8_t I2C_readByte(I2C_HandleTypeDef* i2c, uint8_t devAddr, uint8_t regAddr){
	uint8_t data;
	HAL_I2C_Mem_Read(i2c, devAddr, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
	return data;
}

void MPU9255setup(I2C_HandleTypeDef* i2c, float* _ACx, float* _ACy, float* _ACz, float* _GYx,\
		float* _GYy, float* _GYz, uint8_t *_overload){
		mpu9255i2c = i2c;
	
		HAL_Delay(10);
		I2C_writeByte(mpu9255i2c, MPU_ADDR, MPU_FIFO_EN, MPU_FIFO_SET);
		I2C_writeByte(mpu9255i2c, MPU_ADDR, MPU_PWR_MGMT_1, MPU_PWR_MGMT_1_SET);
		I2C_writeByte(mpu9255i2c, MPU_ADDR, MPU_SMPLRT_DIV, MPU_SMPLRT_DIV_SET);
		I2C_writeByte(mpu9255i2c, MPU_ADDR, MPU_GYRO_CONFIG, MPU_GYRO_CONFIG_SET);
			
		overload = _overload;

		ACx = _ACx;
		ACy = _ACy;
		ACz = _ACz;

		GYx = _GYx;
		GYy = _GYy;
		GYz = _GYz;		
}
	
void MPU9255_IT(){
	if ( mpu9255i2c->State == 0x20 && i2cAccReceivedFlag == 1 )
		i2cAccReceivedFlag = 2;
	if ( mpu9255i2c->State == 0x20 && i2cMagReceivedFlag == 1 )
		i2cMagReceivedFlag = 2;		
}

uint8_t MPU9255_ACC_GYRO_isReady(){
	uint8_t ready = accGyroDataReady;
	accGyroDataReady = 0;
	return ready;
}

uint8_t MPU9255_cycle(uint64_t micros){
	uint8_t status = 0;
	if(micros - accGyroTime >= ACC_GYRO_READ_PERIOD_US && mpu9255i2c->State == 0x20 && i2cAccReceivedFlag == 0){		
		HAL_I2C_Mem_Read_IT(mpu9255i2c, MPU_ADDR, MPU_SENS_START, I2C_MEMADD_SIZE_8BIT, MPUdata, MPU_SENS_LEN);
		i2cAccReceivedFlag = 1;
		
		accGyroTime = micros;
		
		status |= 1<<0;
	}		
	
 if (i2cAccReceivedFlag == 2){
	float unfilteredValue = 0;
	static uint64_t microsPrev = 0;
	float filterNormedKoef = 0; 
	uint8_t overloadTmp = 0;
	i2cAccReceivedFlag = 0;	 
	
	filterNormedKoef = (float)(micros - microsPrev)/(filterTauUs + (float)(micros - microsPrev));
	microsPrev = micros;
	
	 
	unfilteredValue = (int16_t)(MPUdata[0]<<8)|MPUdata[1];
	if( fabs(unfilteredValue) > 32000) overloadTmp |= 1 << 2;	
	unfilteredValue = (unfilteredValue/1638.4 - accXoffs)*accXmult;
	unfilteredValue = (fabs(unfilteredValue) < 1e-20) ? ((unfilteredValue>=0)? 1e-20 : -1e-20) : unfilteredValue; // защита от ошибки процессора 
	*ACx = filterNormedKoef * unfilteredValue + (1-filterNormedKoef)* *ACx;
	 
	unfilteredValue = (int16_t)(MPUdata[2]<<8)|MPUdata[3];
	if( fabs(unfilteredValue) > 32000) overloadTmp |= 1 << 2;	
	unfilteredValue = (unfilteredValue/1638.4 - accYoffs)*accYmult;
	unfilteredValue = (fabs(unfilteredValue) < 1e-20) ? ((unfilteredValue>=0)? 1e-20 : -1e-20) : unfilteredValue; // защита от ошибки процессора 
	*ACy = filterNormedKoef * unfilteredValue + (1-filterNormedKoef)* *ACy;
	 
	unfilteredValue = (int16_t)(MPUdata[4]<<8)|MPUdata[5];
	if( fabs(unfilteredValue) > 32000) overloadTmp |= 1 << 2;	
	unfilteredValue = (unfilteredValue/1638.4 - accZoffs)*accZmult;
	unfilteredValue = (fabs(unfilteredValue) < 1e-20) ? ((unfilteredValue>=0)? 1e-20 : -1e-20) : unfilteredValue; // защита от ошибки процессора 
	*ACz = filterNormedKoef * unfilteredValue + (1-filterNormedKoef)* *ACz;


	unfilteredValue = (int16_t)(MPUdata[8]<<8)|MPUdata[9];
	if( fabs(unfilteredValue) > 32000) overloadTmp |= 1 << 1;	
	unfilteredValue = (unfilteredValue/32.8 - gyroXoffs)*gyroXmult;
	unfilteredValue = (fabs(unfilteredValue) < 1e-20) ? ((unfilteredValue>=0)? 1e-20 : -1e-20) : unfilteredValue; // защита от ошибки процессора 
	*GYx = filterNormedKoef * unfilteredValue + (1-filterNormedKoef)* *GYx;
	 
	unfilteredValue = (int16_t)(MPUdata[10]<<8)|MPUdata[11];
	if( fabs(unfilteredValue) > 32000) overloadTmp |= 1 << 1;	
	unfilteredValue = (unfilteredValue/32.8 - gyroYoffs)*gyroYmult;
	unfilteredValue = (fabs(unfilteredValue) < 1e-20) ? ((unfilteredValue>=0)? 1e-20 : -1e-20) : unfilteredValue; // защита от ошибки процессора 
	*GYy = filterNormedKoef * unfilteredValue + (1-filterNormedKoef)* *GYy;
	 
	unfilteredValue = (int16_t)(MPUdata[12]<<8)|MPUdata[13];
	if( fabs(unfilteredValue) > 32000) overloadTmp |= 1 << 1;	
	unfilteredValue = (unfilteredValue/32.8 - gyroZoffs)*gyroZmult;
	unfilteredValue = (fabs(unfilteredValue) < 1e-20) ? ((unfilteredValue>=0)? 1e-20 : -1e-20) : unfilteredValue; // защита от ошибки процессора 
	*GYz = filterNormedKoef * unfilteredValue + (1-filterNormedKoef)* *GYz;

//	GYxTmp = (int16_t)(MPUdata[8]<<8)|MPUdata[9];
//	GYyTmp = (int16_t)(MPUdata[10]<<8)|MPUdata[11];
//	GYzTmp = (int16_t)(MPUdata[12]<<8)|MPUdata[13];	
//	

//	GYxTmp = (GYxTmp/32.8 - gyroXoffs)*gyroXmult;
//	GYyTmp = (GYyTmp/32.8 - gyroYoffs)*gyroYmult;
//	GYzTmp = (GYzTmp/32.8 - gyroZoffs)*gyroZmult;
	
	
	*overload = overloadTmp;
	
	 
	accGyroDataReady = 1;
	
	status |= 1<<2;
 }	
 return status;
}


void gyroCalib(void){
	gyroXoffs = 0;
	gyroYoffs = 0;
	gyroZoffs = 0;
	for( uint8_t i = 0; i < 100; i ++){
		HAL_I2C_Mem_Read(mpu9255i2c, MPU_ADDR, MPU_SENS_START, I2C_MEMADD_SIZE_8BIT, MPUdata, MPU_SENS_LEN, 1000);

		gyroXoffs += ((int16_t)(MPUdata[8]<<8)|MPUdata[9])/32.8;
		gyroYoffs += ((int16_t)(MPUdata[10]<<8)|MPUdata[11])/32.8;
		gyroZoffs += ((int16_t)(MPUdata[12]<<8)|MPUdata[13])/32.8;		
		
		HAL_Delay(5);
	}
	
	gyroXoffs /= 100;
	gyroYoffs /= 100;
	gyroZoffs /= 100;

}


#undef MPU_ADDR
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

#undef ACC_GYRO_READ_PERIOD_US

#undef accXoffs
#undef accYoffs
#undef accZoffs

#undef accXmult
#undef accYmult
#undef accZmult

#undef gyroXoffs
#undef gyroYoffs
#undef gyroZoffs

#undef gyroXmult
#undef gyroYmult
#undef gyroZmult

#undef magXOffs
#undef magYOffs
#undef magZOffs

#undef magXmult
#undef magYmult
#undef magZmult
