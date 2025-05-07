/*
 * mpu6050_register.h
 *
 *  Created on: Apr 7, 2025
 *      Author: husey
 */

#ifndef INC_MPU6050_REGISTER_H_
#define INC_MPU6050_REGISTER_H_

#define MPU6050_ADDR  0x68
#define WRITE_ADDR   (MPU6050_ADDR << 1)
#define READ_ADDR    (MPU6050_ADDR << 1 | 1)

#define CALIBRATION_SAMPLE_COUNT 500

#define PWR_MGMT_1     0x6B    // Güç yönetim registerı
#define CONFIG         0x1A    //
#define SMPRT_DIV      0x19    // Örnekleme hızı belirleme registerı
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C    // İvmeölçer konfigrasyon registerı
#define WHO_AM_I       0x75    // Cihaz kimliğini doğrulama registerı
#define ACCEL_XOUT_H   0x3B    // X ekseni ivme değerini barındıran (yüksek bit) registerı
#define ACCEL_XOUT_L   0x3C    // X ekseni ivme değerini barındıran (düşük bit) registerı
#define ACCEL_YOUT_H   0x3D    // Y ekseni ivme değerini barındıran (yüksek bit) registerı
#define ACCEL_YOUT_L   0x3E    // Y ekseni ivme değerini barındıran (düşük bit) registerı
#define ACCEL_ZOUT_H   0x3F    // Z ekseni ivme değerini barındıran (yüksek bit) registerı
#define ACCEL_ZOUT_L   0x40    // Z ekseni ivme değerini barındıran (düşük bit) registerı
#define TEMP_OUT_H     0x41    // MPU6050 sıcaklık değerini barındıran (yüksek bit) registerı
#define TEMP_OUT_L     0x42    // MPU6050 sıcaklık değerini barındıran (düşük bit) registerı
#define GYRO_XOUT_H    0x43    // X ekseni açısal hız değerini barındıran (yüksek bit) registerı
#define GYRO_XOUT_L    0x44    // X ekseni açısal hız değerini barındıran (düşük bit) registerı
#define GYRO_YOUT_H    0x45    // Y ekseni açısal hız değerini barındıran (yüksek bit) registerı
#define GYRO_YOUT_L    0x46    // Y ekseni açısal hız değerini barındıran (düşük bit) registerı
#define GYRO_ZOUT_H    0x47    // Z ekseni açısal hız değerini barındıran (yüksek bit) registerı
#define GYRO_ZOUT_L    0x48    // Z ekseni açısal hız değerini barındıran (düşük bit) registerı

#endif /* INC_MPU6050_REGISTER_H_ */
