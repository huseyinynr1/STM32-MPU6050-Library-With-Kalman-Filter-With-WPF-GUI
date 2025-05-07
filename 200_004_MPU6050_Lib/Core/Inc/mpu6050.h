/*
 * mpu6050.h
 *
 *  Created on: Apr 7, 2025
 *      Author: husey
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32f4xx_hal.h"
#include "mpu6050_register.h"
#include <math.h>

typedef struct{
	float ax;
	float ay;
	float az;

	float gx;
	float gy;
	float gz;
}SensorOffset_t;

typedef struct{
	double roll;
	double pitch;

	double previous_roll;
	double previous_pitch;

	uint32_t previous_time;

}AngleState_t;

typedef struct{
	double angle;            // Son tahmin edilen açı.
	double previous_angle;
	float bias;              // Jiroskop drift değeri.
	float Q_angle;           // Filtrenin açı tahminine güveni.
	float Q_bias;            // Bias tahminine güven (jiroskop sapması).
	float R_measure;         // İvmeölçer ölçümlerine duyulan güven (gürültü seviyesi)
	float P[2][2];           // 2x2 hata kovaryans matrisi
	uint32_t previous_time;  // Son zaman
}KalmanState_t;


HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *h12c);
HAL_StatusTypeDef MPU6050_READ_ACCEL_ALL_AXIS(I2C_HandleTypeDef *hi2c, int16_t *aX,int16_t *aY,int16_t *aZ);
int16_t MPU6050_READ_ACCEL_X_AXIS(I2C_HandleTypeDef *hi2c);
int16_t MPU6050_READ_ACCEL_Y_AXIS(I2C_HandleTypeDef *hi2c);
int16_t MPU6050_READ_ACCEL_Z_AXIS(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_READ_GYRO_ALL_AXIS(I2C_HandleTypeDef *hi2c, int16_t *gX,int16_t *gY,int16_t *gZ);
int16_t MPU6050_READ_GYRO_X_AXIS(I2C_HandleTypeDef *hi2c);
int16_t MPU6050_READ_GYRO_Y_AXIS(I2C_HandleTypeDef *hi2c);
int16_t MPU6050_READ_GYRO_Z_AXIS(I2C_HandleTypeDef *hi2c);
int16_t MPU6050_READ_TEMP(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_CONVERT_ACCELL(I2C_HandleTypeDef *hi2c, float *ax_g, float *ay_g, float *az_g);
HAL_StatusTypeDef MPU6050_CONVERT_GYRO(I2C_HandleTypeDef *hi2c, float *gx_dps, float *gy_dps, float *gz_dps);
HAL_StatusTypeDef MPU6050_CONVERT_TEMP(I2C_HandleTypeDef *hi2c, float *temp);
void Offset_Calibration_Accel(I2C_HandleTypeDef *hi2c, SensorOffset_t *offset);
void Offset_Calibration_Gyro(I2C_HandleTypeDef *hi2c, SensorOffset_t *offset);
void Offset_Calibration_All(I2C_HandleTypeDef *hi2c, SensorOffset_t *offset);
HAL_StatusTypeDef MPU6050_GET_REAL_ACCEL(I2C_HandleTypeDef *h12c, SensorOffset_t *offset, float *realAx, float *realAy, float *realAz);
HAL_StatusTypeDef MPU6050_GET_REAL_GYRO(I2C_HandleTypeDef *h12c, SensorOffset_t *offset, float *realGx, float *realGz, float *realGy);
HAL_StatusTypeDef GET_ANGLE_WITH_ACCELL(I2C_HandleTypeDef *h12c, SensorOffset_t *offset, double *roll_degree, double *pitch_degree);
HAL_StatusTypeDef GET_ANGLE_WITH_GYRO(I2C_HandleTypeDef *hi2c, SensorOffset_t *offset, AngleState_t *angle);
void Kalman_Init(KalmanState_t *kalman,	double angle, float bias, float Q_angle,float Q_bias, float R_measure, float P[2][2]);
double Kalman_Prediction(KalmanState_t *kalman, I2C_HandleTypeDef *hi2c, double *angle_out, float gx_dps);
double Kalman_Update(I2C_HandleTypeDef *hi2c, KalmanState_t *kalman, SensorOffset_t *offset, double *angle_out, float measured_angle);

#endif /* INC_MPU6050_H_ */
