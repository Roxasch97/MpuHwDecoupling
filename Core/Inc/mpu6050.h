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
	mpu_interface interface;
	uint8_t address;
	MpuStatus status;
	uint8_t accelFullScaleRange;
	uint16_t accelConvFactor;
	uint8_t gyroFullScaleRange;
	float gyroConvFactor;
	uint8_t samplerateDivider;

	/*Basic registers values*/
	uint8_t PWR_MGMT_1;
	uint8_t SMPRT_DIV;
	uint8_t GYRO_CONFIG;
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
void MpuHandleErrors(MpuStatus status);

#endif /* INC_MPU6050_H_ */
