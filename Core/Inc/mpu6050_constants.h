/*
 * mpu6050_constants.h
 *
 *  Created on: 21 cze 2022
 *      Author: adamp
 */

#ifndef INC_MPU6050_CONSTANTS_H_
#define INC_MPU6050_CONSTANTS_H_

typedef enum{
	mpuReadError = -3,
	mpuWriteError = -2,
	mpuInitError = -1,
	mpuOk = 0
}MpuStatus;

#endif /* INC_MPU6050_CONSTANTS_H_ */
