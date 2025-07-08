/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "MATRIX.h"
#include "MICROS.h"
#include "MPU9255.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float ACx, ACy, ACz, GYx, GYy, GYz, GYxPrev, GYyPrev, GYzPrev, MAGx, MAGy, MAGz,
	accTotal, gyroTotal, magTotal, 
	beamAngle, beamAngleTau = 0.001;
uint8_t accGyroMagOverload;
const uint16_t PCazimuth = 345;

struct microsPeriod cyclePeriod = {0, 0}, accGyroPeriod = {0, 0}; 
										
float periodS;				

const uint16_t timMin = 13200, timMax = 16800, timSpan = timMax - timMin;
float kp = 10, ki = 0.001, kd = 5, ks = 700;
float P = 0, I = 0, D = 0, S = 0;
float reqAngle = 90, error = 0, errorPrev = 0;	
float Ilim = 200;
float Plim = 400;
float Dsigma = 45;
float PIDsumTau = 0.01, PIDsum = 0, PIDnormedKoef = 0, PIDsumTmp = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void transmitBasis(float* floatArray, uint64_t time, uint8_t overload);
void transmitBeamAngle(float angle, uint64_t time);
void initState(struct matrix *basis, struct matrix *gravity, struct matrix* north);
void restoreVertical(struct matrix* bas, struct matrix* grav, struct matrix* acc);
void PID(uint64_t time);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	microsInit(&htim3, 72000000);	// start micros timer
	
	htim4.Instance->PSC = 5;
	htim4.Instance->ARR = 48000;
	htim4.Instance->CCR2 = 12000;		
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//	HAL_Delay(1000);
//	htim4.Instance->CCR2 = 1000;
	
	// startup the sensor, assign pointers of ACC, GYRO, MAG and overload flag variables	
	MPU9255setup(&hi2c1, &ACx, &ACy, &ACz, &GYx, &GYy, &GYz, &accGyroMagOverload);
	
	struct matrix accVec,	gyroVec, magVec, rotMatrix, basis, gravityVec, northVec, velocityVec, coordinate, basisStep1, basisStep1Inv, plug, transitionMatrix;
	
	matrixZeroes(&accVec , 3, 1);
	matrixZeroes(&gyroVec , 3, 1);
	matrixZeroes(&magVec , 3, 1);
	matrixZeroes(&rotMatrix , 3, 3);
	matrixEye(&basis, 3);
	matrixZeroes(&gravityVec, 3,1);
	matrixZeroes(&northVec, 3, 1);
	matrixZeroes(&velocityVec, 3,1);
	matrixZeroes(&coordinate, 3, 1);
	matrixZeroes(&basisStep1, 3, 3);
	matrixZeroes(&basisStep1Inv, 3, 3);
	matrixZeroes(&plug, 3, 3);
	matrixZeroes(&transitionMatrix, 3, 3);
	
	matrixSE(&northVec, 0, 0, 1);
	matrixRotZ(&northVec, PCazimuth);
	// indication before the ongoing calibration. Sensor should be kept still!
	HAL_Delay(500);
	for(uint8_t v = 0; v<10; v++){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		HAL_Delay(150);
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	
	gyroCalib();
	
	initState(&basis, &gravityVec, &northVec);
	
	matrixCopy(&basis, &basisStep1);
	matrixInvOrth(&basisStep1, &basisStep1Inv);
	
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(2000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	microsStart(&accGyroPeriod);	
  while (1)
  {
		static uint32_t ledToggleCnt = 0;
		float GYxTmp, GYyTmp, GYzTmp;
			
		MPU9255_cycle(micros());
			
		if( MPU9255_ACC_GYRO_isReady() == 1){
			float normedKoef, beamAngleTmp;
			matrixSE(&accVec, 0, 0, ACx);
			matrixSE(&accVec, 1, 0, ACy);
			matrixSE(&accVec, 2, 0, ACz);
			accTotal = matrixVecLen(&accVec);
			
			microsUpdate(&accGyroPeriod);
			periodS = (float) accGyroPeriod.period*1e-6;
			GYxTmp = (GYx+GYxPrev)/2 * periodS;
			GYyTmp = (GYy+GYyPrev)/2 * periodS;
			GYzTmp = (GYz+GYzPrev)/2 * periodS;
			GYxPrev = GYx;
			GYyPrev = GYy;
			GYzPrev = GYz;	
			
			matrixRot3D(&rotMatrix, GYxTmp, GYyTmp, GYzTmp);
			matrixMultiply(&basis, &rotMatrix, &basis);
			
			matrixMultiply(&basis, &accVec, &accVec);
			restoreVertical(&basis, &gravityVec, &accVec);
			matrixSum(&accVec, &gravityVec, &accVec);
			
			matrixMultiply(&basis, &basisStep1Inv, &transitionMatrix);
			beamAngleTmp = rad2deg*matrixRotMatrixRotAngle(&transitionMatrix);
			
			normedKoef = periodS/(beamAngleTau+periodS);
      beamAngle = normedKoef*beamAngleTmp+(1-normedKoef)*beamAngle;  
		}				
		
		// transmitBasis(basis.arr, micros(), accGyroMagOverload);
		transmitBeamAngle(beamAngle, micros());
		

		PID(micros());
		
		
		if(ledToggleCnt == 10000){// I hope this can indicate the load of the CPU
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);	
				ledToggleCnt = 0;
		}
		ledToggleCnt++;

		microsUpdate(&cyclePeriod);		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// this fcn is needed to orient the MPU sensor
void initState(struct matrix *basis, struct matrix *gravity, struct matrix* north){
	struct matrix	rotVec, Rinit, acc, imGravity, magInit;
	float c, s, lx, ly, lz, theta;
	uint8_t magCnt = 0;
	
	matrixZeroes(&rotVec, 3,1);
	matrixZeroes(&Rinit, 3,3);
	matrixZeroes(&acc, 3,1);
	matrixZeroes(&imGravity, 3,1);
	matrixZeroes(&magInit, 3, 1);
	// acquisition ACC GYRO MAG meawurements
	for(uint8_t i = 0; i < 100; i++){
		while ( MPU9255_cycle(micros()) && 1<<0 == 0)
			HAL_Delay(1);
		
		while (MPU9255_ACC_GYRO_isReady() == 0){
			HAL_Delay(1);
			MPU9255_cycle(micros());
		}
			
		acc.arr[0] += ACx;
		acc.arr[1] += ACy;
		acc.arr[2] += ACz;
	}
	//// vertical alignment
	matrixMultiplyByNumb(&acc, gravity, -0.01);
	imGravity.arr[2] = -matrixVecLen(gravity);
	matrixCross(gravity, &imGravity, &rotVec);
	matrixNormVec(&rotVec, &rotVec);
	
	theta = acos(matrixDot(gravity, &imGravity)/(matrixVecLen(&imGravity)*matrixVecLen(gravity)));
	matrixRotArbVec(&Rinit, &rotVec, theta);
	
	matrixMultiply(&Rinit, basis , basis);
	matrixMultiply(basis, gravity, gravity);	
	
	//// azimuth alignment
/*	matrixMultiplyByNumb(&magInit, &magInit, (float) 1/magCnt);
	matrixMultiply(basis, &magInit, &magInit);	
	magInit.arr[2] = 0;
	matrixNormVec(&magInit, &magInit);
	theta = -matrixVecAzimuth(north, &magInit);
	
	matrixSE(&Rinit,	0,	0, cos(theta));
	matrixSE(&Rinit,	0,	1, -sin(theta));
	matrixSE(&Rinit,	0,	2, 0);
	matrixSE(&Rinit,	1,	0, sin(theta));
	matrixSE(&Rinit,	1,	1, cos(theta));
	matrixSE(&Rinit,	1,	2, 0);
	matrixSE(&Rinit,	2,	0, 0);
	matrixSE(&Rinit,	2,	1, 0);
	matrixSE(&Rinit,	2,	2, 1);	
	
	matrixMultiply(&Rinit, basis , basis);*/
}
// transmits array of vectors and sensor overload flag to PC via COM
void transmitBasis(float* floatArray, uint64_t time, uint8_t overload){
	static uint64_t prewTime = 0;
	if(time - prewTime < 50000) return;
	
	uint8_t data[10];
	
	prewTime = time;
	for(uint8_t i = 0; i<9; i++){
		data[i] = (int8_t) (floatArray[i]*100);
	}
	data[9] = overload;
		
	CDC_Transmit_FS(data, 10);
}

void transmitBeamAngle(float angle, uint64_t time){
	static uint64_t prewTime = 0;
	static uint8_t data[20];
	static uint8_t counter = 0;
	
	if(time - prewTime < 20000) return;
	angle = angle*100;
	data[counter*2+1] = ((uint16_t)(angle)>>8)&0x00ff;
	data[counter*2] = ((uint16_t)(angle))&0x00ff;
//	counter ++;
	prewTime = time;
	CDC_Transmit_FS(data, 2);
	
//	if (counter == 10){
//		CDC_Transmit_FS(data, 20);
//		counter = 0;
//	}
}

void PID(uint64_t time){
	static uint64_t prewTime = 0;
	uint32_t PIDperiod = time - prewTime;
	float periodS;
	static uint8_t firstIteration = 1;
	static float anglePrev = 0, da = 0;
	
	if(PIDperiod < 5000) return;
	prewTime = time;
		
	periodS = (float)PIDperiod*1e-6;
	
	if (firstIteration){
		error = reqAngle - beamAngle;
		errorPrev = error;
		firstIteration = 0;
		anglePrev = beamAngle;
		return;
	}
	
	errorPrev = error;
	error = reqAngle - beamAngle;
	S = +ks*sin(deg2rad*reqAngle);
	P = 	kp*error;
	P = (P > Plim) ? Plim : P;
	P = (P < -Plim) ? -Plim: P;
	I += 	ki*periodS*error;
	I = (I > Ilim) ? Ilim : I;
	I = (I < -Ilim) ? -Ilim: I;
	
	da = beamAngle-anglePrev;
	anglePrev = beamAngle;
//	D = kd*da*fabs(da)/periodS * exp(-pow(error/Dsigma, 2));
	
	if (da >= 0)
		D = kd*da/periodS;
	else 
		D = kd*da/periodS * exp(-pow(error/Dsigma, 2));
	PIDsumTmp = (P+I-D+S)/1000;	
	
	PIDsumTmp = (PIDsumTmp > 1) ? 1 : PIDsumTmp;
	PIDsumTmp = (PIDsumTmp < 0) ? 0: PIDsumTmp;
	
	PIDsumTmp = (PIDsumTmp < 1e-6) ? 1e-6: PIDsumTmp; // защита от ошибки процессора
	PIDnormedKoef = periodS/(PIDsumTau+periodS);
	PIDsum = PIDnormedKoef*PIDsumTmp+(1-PIDnormedKoef)*PIDsum;  
	
	htim4.Instance->CCR2 = (uint16_t)(PIDsum * timSpan + timMin);
	
}

// allign vertical orientation to mitigate drift of gyro sensors or after a loss of angles due to overload
void restoreVertical(struct matrix* bas, struct matrix* grav, struct matrix* acc){
	struct matrix	rotVec, Rinit;
	float accLen, angle;
	
	matrixZeroes(&rotVec, 3,1);
	matrixZeroes(&Rinit, 3,3);
	accLen = matrixVecLen(acc);
	
	if( accLen > 9.8 && accLen <  10.2){
		matrixCross(acc, grav, &rotVec);
		matrixNormVec(&rotVec, &rotVec);
		angle = -acos(-matrixDot(acc, grav)/(accLen*matrixVecLen(grav)));
//		if( angle < -0.03 || angle > 0.03){
			matrixRotArbVec(&Rinit, &rotVec, angle/50);
			matrixMultiply(&Rinit, bas, bas);
//		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
