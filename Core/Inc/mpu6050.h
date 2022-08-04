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

typedef struct
{
	mpu_interface interface;
	uint8_t address;
	MpuStatus status;

	/*Basic registers values*/
	uint8_t PWR_MGMT_1;
	uint8_t SMPRT_DIV;
	uint8_t ACCEL_CONFIG;
	uint8_t GYRO_CONFIG;
	uint8_t INT_ENABLE;
	uint8_t FullScaleRange;
} MpuType;

void MpuInitialize(MpuType *mpu);
int16_t MpuReadAccelXRaw (MpuType *mpu);
void MpuHandleErrors(MpuType *mpu);

#endif /* INC_MPU6050_H_ */
