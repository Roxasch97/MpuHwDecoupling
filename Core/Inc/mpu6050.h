/*
 * mpu6050.h
 *
 *  Created on: Jun 13, 2022
 *      Author: adamp
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdbool.h>
#include <stdint.h>
#include "mpu6050_interface.h"
#include "mpu6050_constants.h"

typedef struct {
	mpu_interface 	interface;
	uint8_t 		address;
	MpuStatus 		status;
	uint8_t 		samplerateDiv;
	uint8_t 		accelFullScaleRange;
	uint16_t 		accelConvFactor;
	uint8_t 		gyroFullScaleRange;
	float 			gyroConvFactor;
	uint8_t 		samplerateDivider;
} MpuType;

MpuStatus MpuReset(const MpuType *mpu);
MpuStatus MpuInitialize(MpuType *mpu);
MpuStatus MpuSetAccelFSR(MpuType *mpu);
MpuStatus MpuSetGyroFSR(MpuType *mpu);
MpuStatus MpuSetSamplerateDivider(const MpuType *mpu);
float MpuConvertAccel(const MpuType *mpu, int16_t rawAccel);
int16_t MpuReadAccelXRaw(const MpuType *mpu);
int16_t MpuReadAccelYRaw(const MpuType *mpu);
int16_t MpuReadAccelZRaw(const MpuType *mpu);
float MpuConvertGyro(const MpuType *mpu, int16_t rawGyro);
int16_t MpuReadGyroXRaw(const MpuType *mpu);
int16_t MpuReadGyroYRaw(const MpuType *mpu);
int16_t MpuReadGyroZRaw(const MpuType *mpu);
void MpuHandleErrors(MpuStatus status);

#endif /* INC_MPU6050_H_ */
