/*
 * mpu6050_constants.h
 *
 *  Created on: 21 cze 2022
 *      Author: adamp
 */

#ifndef INC_MPU6050_CONSTANTS_H_
#define INC_MPU6050_CONSTANTS_H_

typedef enum {
	mpuGenericError = -128,
	mpuConvertError = -4,
	mpuAbsentError = -3,
	mpuReadError = -2,
	mpuWriteError = -1,
	mpuOk = 0
} MpuStatus;

#define MPU6050_ADDRESS_1             (0xD0)
#define MPU6050_ADDRESS_2             (0xD2)

/* Registers defines */
#define MPU6050_REG_SMPLRT_DIV		  (0x19)
#define MPU6050_REG_GYRO_CONFIG       (0x1B)
#define MPU6050_REG_ACCEL_CONFIG      (0x1C)
#define MPU6050_REG_ACCEL_XOUT_H      (0x3B)
#define MPU6050_REG_ACCEL_YOUT_H      (0x3D)
#define MPU6050_REG_ACCEL_ZOUT_H      (0x3F)
#define MPU6050_REG_GYRO_XOUT_H       (0x43)
#define MPU6050_REG_GYRO_YOUT_H       (0x45)
#define MPU6050_REG_GYRO_ZOUT_H       (0x47)
#define MPU6050_REG_PWR_MGMT_1        (0x6B)
#define MPU6050_REG_WHO_AM_I          (0x75)

#define RESET_VALUE 1U<<7
#define WHO_AM_I	0x68U

#define MPU_ACCEL_2G_FSR 		0
#define MPU_ACCEL_4G_FSR		1U<<3
#define MPU_ACCEL_8G_FSR		2U<<3
#define MPU_ACCEL_16G_FSR		3U<<3
#define MPU_2G_CONV_FACTOR		16384
#define MPU_4G_CONV_FACTOR		8192
#define MPU_8G_CONV_FACTOR		4096
#define MPU_16G_CONV_FACTOR		2048

#define MPU_GYRO_250_FSR 		0
#define MPU_GYRO_500_FSR		1U<<3
#define MPU_GYRO_1000_FSR		2U<<3
#define MPU_GYRO_2000_FSR		3U<<3
/* CONV_FACTOR = FSR/32696 */
#define MPU_250_DEG_CONV_FACTOR		0.007633
#define MPU_500_DEG_CONV_FACTOR		0.015267
#define MPU_1000_DEG_CONV_FACTOR	0.030487
#define MPU_2000_DEG_CONV_FACTOR	0.060975

#endif /* INC_MPU6050_CONSTANTS_H_ */
