/*
 * MPU6050.c
 *
 *  Created on: Apr 7, 2025
 *      Author: husey
 */

#include "mpu6050.h"

static int timeout = 100;

HAL_StatusTypeDef MPU6050_WriteRegister(I2C_HandleTypeDef *hi2c, uint8_t writeAddress, uint8_t writeRegister, uint8_t writeValue)
{
    uint8_t txData[2] = {writeRegister, writeValue};
    return HAL_I2C_Master_Transmit(hi2c, writeAddress, txData, sizeof(txData), timeout);
}

HAL_StatusTypeDef MPU6050_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t readAddress, uint8_t readRegister, uint8_t *data, uint8_t size)
{
    return HAL_I2C_Mem_Read(hi2c, readAddress, readRegister, 1, data, size, timeout);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *h12c)
{
    HAL_StatusTypeDef ret;
    uint8_t readData;

    ret = MPU6050_WriteRegister(h12c, WRITE_ADDR, PWR_MGMT_1, 0x00);
    if(ret != HAL_OK) return ret;

    ret = MPU6050_ReadRegister(h12c, READ_ADDR, WHO_AM_I, &readData, 1);
    if(ret != HAL_OK || readData != MPU6050_ADDR) return HAL_ERROR;

    ret = MPU6050_WriteRegister(h12c, WRITE_ADDR, CONFIG, 0x03);
    if(ret != HAL_OK) return ret;

    ret = MPU6050_WriteRegister(h12c, WRITE_ADDR, SMPRT_DIV, 9);
    if(ret != HAL_OK) return ret;

    ret = MPU6050_WriteRegister(h12c, WRITE_ADDR, GYRO_CONFIG, 0x10);
    if(ret != HAL_OK) return ret;

    ret = MPU6050_WriteRegister(h12c, WRITE_ADDR, ACCEL_CONFIG, 0x10);
    if(ret != HAL_OK) return ret;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_READ_ACCEL_ALL_AXIS(I2C_HandleTypeDef *hi2c, int16_t *aX, int16_t *aY, int16_t *aZ)
{
    HAL_StatusTypeDef ret;
    uint8_t accelData[6];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, ACCEL_XOUT_H, accelData, 6);
    if(ret != HAL_OK) return ret;

    *aX = (int16_t)(accelData[0] << 8 | accelData[1]);
    *aY = (int16_t)(accelData[2] << 8 | accelData[3]);
    *aZ = (int16_t)(accelData[4] << 8 | accelData[5]);

    return HAL_OK;
}

int16_t MPU6050_READ_ACCEL_X_AXIS(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t data[2];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, ACCEL_XOUT_H, data, 2);
    if(ret != HAL_OK) return INT16_MIN;

    return (int16_t)(data[0] << 8 | data[1]);
}

int16_t MPU6050_READ_ACCEL_Y_AXIS(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t data[2];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, ACCEL_YOUT_H, data, 2);
    if(ret != HAL_OK) return INT16_MIN;

    return (int16_t)(data[0] << 8 | data[1]);
}

int16_t MPU6050_READ_ACCEL_Z_AXIS(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t data[2];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, ACCEL_ZOUT_H, data, 2);
    if(ret != HAL_OK) return INT16_MIN;

    return (int16_t)(data[0] << 8 | data[1]);
}

HAL_StatusTypeDef MPU6050_READ_GYRO_ALL_AXIS(I2C_HandleTypeDef *hi2c, int16_t *gX, int16_t *gY, int16_t *gZ)
{
    HAL_StatusTypeDef ret;
    uint8_t gyroData[6];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, GYRO_XOUT_H, gyroData, 6);
    if(ret != HAL_OK) return ret;

    *gX = (int16_t)(gyroData[0] << 8 | gyroData[1]);
    *gY = (int16_t)(gyroData[2] << 8 | gyroData[3]);
    *gZ = (int16_t)(gyroData[4] << 8 | gyroData[5]);

    return HAL_OK;
}

int16_t MPU6050_READ_GYRO_X_AXIS(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t data[2];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, GYRO_XOUT_H, data, 2);
    if(ret != HAL_OK) return INT16_MIN;

    return (int16_t)(data[0] << 8 | data[1]);
}

int16_t MPU6050_READ_GYRO_Y_AXIS(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t data[2];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, GYRO_YOUT_H, data, 2);
    if(ret != HAL_OK) return INT16_MIN;

    return (int16_t)(data[0] << 8 | data[1]);
}

int16_t MPU6050_READ_GYRO_Z_AXIS(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t data[2];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, GYRO_ZOUT_H, data, 2);
    if(ret != HAL_OK) return INT16_MIN;

    return (int16_t)(data[0] << 8 | data[1]);
}

int16_t MPU6050_READ_TEMP(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t tempRawData[2];

    ret = MPU6050_ReadRegister(hi2c, READ_ADDR, TEMP_OUT_H, tempRawData, 2);
    if(ret != HAL_OK) return 0;

    return (int16_t)(tempRawData[0] << 8 | tempRawData[1]);
}


HAL_StatusTypeDef MPU6050_CONVERT_ACCELL(I2C_HandleTypeDef *hi2c, float *ax_g, float *ay_g, float *az_g)
{
	HAL_StatusTypeDef ret;
	uint8_t config_reg;
	int16_t raw_ax, raw_ay, raw_az;
	float scale;

	ret = MPU6050_ReadRegister(hi2c, READ_ADDR, ACCEL_CONFIG, &config_reg, 1);
	if(ret != HAL_OK) return ret;

	ret = MPU6050_READ_ACCEL_ALL_AXIS(hi2c, &raw_ax, &raw_ay, &raw_az);
	if(ret != HAL_OK) return ret;

	uint8_t afs_sel = (config_reg >> 3) & 0x03;

	switch (afs_sel) {
		case 0:
			scale = 16384.0f;
			break;
		case 1:
			scale = 8192.0f;
			break;
		case 2:
			scale = 4096.0f;
			break;
		case 3:
			scale = 2048.0f;
			break;
		default:
			return HAL_ERROR;
	}

	*ax_g = (float)raw_ax / scale;
	*ay_g = (float)raw_ay / scale;
	*az_g = (float)raw_az / scale;

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CONVERT_GYRO(I2C_HandleTypeDef *hi2c, float *gx_dps, float *gy_dps, float *gz_dps)
{
	HAL_StatusTypeDef ret;
	uint8_t config_reg;
	int16_t raw_gx, raw_gy, raw_gz;
	float scale;

	ret = MPU6050_ReadRegister(hi2c, READ_ADDR, GYRO_CONFIG, &config_reg, 1);
	if(ret != HAL_OK) return ret;

	ret = MPU6050_READ_GYRO_ALL_AXIS(hi2c, &raw_gx, &raw_gy, &raw_gz);
	if(ret != HAL_OK) return ret;

	uint8_t fs_sel = (config_reg >> 3) & 0x03;

	switch (fs_sel) {
		case 0:
			scale = 131.0f;
			break;
		case 1:
			scale = 65.5f;
			break;
		case 2:
			scale = 32.8f;
			break;
		case 3:
			scale = 16.4f;
			break;
		default:
			return HAL_ERROR;
	}

	*gx_dps = raw_gx / scale;
	*gy_dps = raw_gy / scale;
	*gz_dps = raw_gz / scale;

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CONVERT_TEMP(I2C_HandleTypeDef *hi2c, float *temp)
{
	int16_t raw_temp;

	raw_temp = MPU6050_READ_TEMP(hi2c);
	if(raw_temp == INT16_MIN) return HAL_ERROR;

	*temp = (float)(raw_temp / 340.0f) + 36.53f;

	return HAL_OK;
}


void Offset_Calibration_Accel(I2C_HandleTypeDef *hi2c, SensorOffset_t *offset)
{
	float totalAccelMeasurementX = 0, totalAccelMeasurementY = 0, totalAccelMeasurementZ = 0;
	float ax, ay, az;


	for (int i = 0 ; i < CALIBRATION_SAMPLE_COUNT; i++)
	{
		MPU6050_CONVERT_ACCELL(hi2c, &ax, &ay, &az);

		totalAccelMeasurementX += ax;

		totalAccelMeasurementY += ay;

		totalAccelMeasurementZ += az;

		HAL_Delay(2);
	}

	offset->ax = totalAccelMeasurementX / CALIBRATION_SAMPLE_COUNT;
	offset->ay = totalAccelMeasurementY / CALIBRATION_SAMPLE_COUNT;
	offset->az = totalAccelMeasurementZ / CALIBRATION_SAMPLE_COUNT;
}

void Offset_Calibration_Gyro(I2C_HandleTypeDef *hi2c, SensorOffset_t *offset)
{
	float totalGyroMeasurementX = 0, totalGyroMeasurementY = 0, totalGyroMeasurementZ = 0;
	float gx, gy, gz;

		for (int i = 0 ; i < CALIBRATION_SAMPLE_COUNT; i++)
		{
			MPU6050_CONVERT_GYRO(hi2c, &gx, &gy, &gz);

			totalGyroMeasurementX += gx;

			totalGyroMeasurementY += gy;

			totalGyroMeasurementZ += gz;

			HAL_Delay(2);
		}

		offset->gx = totalGyroMeasurementX / CALIBRATION_SAMPLE_COUNT;
		offset->gy = totalGyroMeasurementY / CALIBRATION_SAMPLE_COUNT;
		offset->gz = totalGyroMeasurementZ / CALIBRATION_SAMPLE_COUNT;
}

void Offset_Calibration_All(I2C_HandleTypeDef *hi2c, SensorOffset_t *offset)
{
	Offset_Calibration_Accel(hi2c, offset);
	Offset_Calibration_Gyro(hi2c, offset);
}

HAL_StatusTypeDef MPU6050_GET_REAL_ACCEL(I2C_HandleTypeDef *h12c, SensorOffset_t *offset, float *realAx, float *realAy, float *realAz)
{
	HAL_StatusTypeDef ret;
	float raw_ax, raw_ay, raw_az;

	ret = MPU6050_CONVERT_ACCELL(h12c, &raw_ax, &raw_ay, &raw_az);
	if(ret != HAL_OK) return ret;

	*realAx = raw_ax - offset->ax;
	*realAy = raw_ay - offset->ay;
	*realAz = raw_az - offset->az;

	return HAL_OK;
}

HAL_StatusTypeDef MPU6050_GET_REAL_GYRO(I2C_HandleTypeDef *h12c, SensorOffset_t *offset, float *realGx, float *realGy, float *realGz)
{
	HAL_StatusTypeDef ret;
	float raw_gx, raw_gy, raw_gz;

	ret = MPU6050_CONVERT_GYRO(h12c, &raw_gx, &raw_gy, &raw_gz);
	if(ret != HAL_OK) return ret;

	*realGx = raw_gx - offset->gx;
	*realGy = raw_gy - offset->gy;
	*realGz = raw_gz - offset->gz;

	return HAL_OK;
}

HAL_StatusTypeDef GET_ANGLE_WITH_ACCELL(I2C_HandleTypeDef *h12c, SensorOffset_t *offset, double *roll_degree, double *pitch_degree)
{
	HAL_StatusTypeDef ret;
	float realAx, realAy, realAz;
	double radian_roll, radian_pitch;

	ret = MPU6050_GET_REAL_ACCEL(h12c, offset, &realAx, &realAy, &realAz);
	if(ret != HAL_OK) return ret;

	radian_roll = (double)atan2(realAy, realAz) ;

	radian_pitch = (double)atan2(-realAx, sqrt(pow(realAy , 2) + pow(realAz, 2)));

	*roll_degree = radian_roll * (180.0f / M_PI);

	*pitch_degree = radian_pitch * (180.0f / M_PI);

	return HAL_OK;
}

HAL_StatusTypeDef GET_ANGLE_WITH_GYRO(I2C_HandleTypeDef *hi2c, SensorOffset_t *offset, AngleState_t *angle)
{
	HAL_StatusTypeDef ret;
	float realGx, realGy, realGz;
	uint32_t current_time, delta_time;

	ret = MPU6050_GET_REAL_GYRO(hi2c, offset, &realGx, &realGy, &realGz);
	if(ret != HAL_OK) return ret;

	current_time = HAL_GetTick();
	delta_time = (current_time - angle->previous_time) / 1000.0f;

	angle->roll  = angle->previous_roll  + (realGx * delta_time);
	angle->pitch = angle->previous_pitch + (realGy * delta_time);

	angle->previous_roll = angle->roll;
    angle->previous_pitch = angle->pitch;
    angle->previous_time = current_time;

    return HAL_OK;
}

void Kalman_Init(KalmanState_t *kalman,	double angle, float bias, float Q_angle,float Q_bias,
float R_measure, float P[2][2])
{
	kalman->angle = angle;

	kalman->bias = bias;

	kalman->Q_angle = Q_angle;

	kalman->Q_bias = Q_bias;

	kalman->R_measure = R_measure;

	kalman->previous_time = HAL_GetTick();

	for(int i = 0 ; i < 2; i++)
	{
		for(int j = 0; j < 2; j++)
		{
			kalman->P[i][j] = P[i][j];
		}
	}
}

// Kalman filtresi tahmin aşaması
double Kalman_Prediction(KalmanState_t *kalman, I2C_HandleTypeDef *hi2c, double *angle_out, float gx_dps)
{
	uint32_t current_time;
	float rate, dt;

	current_time = HAL_GetTick();

	dt = (float)(current_time - kalman->previous_time) / 1000.0f;

	rate = gx_dps - kalman->bias;

	kalman->angle = kalman->previous_angle + (dt * rate);

	kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
	kalman->P[0][1] -= dt * kalman->P[1][1];
	kalman->P[1][0] -= dt * kalman->P[1][1];
	kalman->P[1][1] += kalman->Q_bias * dt;

	kalman->previous_time = current_time;
	kalman->previous_angle = kalman->angle;

	*angle_out = kalman->angle;
	return *angle_out;

}

// Kalman filtresi güncelleme aşaması
double Kalman_Update(I2C_HandleTypeDef *hi2c, KalmanState_t *kalman, SensorOffset_t *offset, double *angle_out, float measured_angle)
{
	float y, S;

	y = measured_angle - kalman->angle;       // Tahmin ile ölçüm arasındaki fark.

	S = kalman->P[0][0] + kalman->R_measure;  //

	float K[2];
	K[0] = kalman->P[0][0] / S;
	K[1] = kalman->P[1][0] / S;

	kalman->angle += K[0] * y;               // açıyı düzeltmek için kazanç
	kalman->bias  += K[1] * y;               // bias düzeltmesi için kazanç

	float P00_temp = kalman->P[0][0];
	float P01_temp = kalman->P[0][1];

	kalman->P[0][0] -= K[0] * P00_temp;
	kalman->P[0][1] -= K[0] * P01_temp;
	kalman->P[1][0] -= K[1] * P00_temp;
	kalman->P[1][1] -= K[1] * P01_temp;

	*angle_out = kalman->angle;

	return *angle_out;
}
