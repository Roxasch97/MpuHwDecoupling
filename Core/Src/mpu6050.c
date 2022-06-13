/*
 * mpu6050.c
 *
 *  Created on: Jun 13, 2022
 *      Author: adamp
 */


#include "mpu6050.h"


void mpuWrite(uint8_t regAddress, uint8_t value, MpuType *mpu)
{
	mpu->interface.MpuWriteCallback(regAddress, value, mpu->address);
}

void mpuRead(uint8_t regAddress, uint16_t size, MpuType *mpu, uint8_t *destination)
{
	mpu->interface.MpuReadCallback(regAddress, size, destination, mpu->address);
}

uint8_t MpuWhoAmI(MpuType *MpuAccel)
{
	uint8_t WhoAmIVal=0;
	mpuRead(MPU6050_REG_WHO_AM_I, 1, MpuAccel, &WhoAmIVal);
	return WhoAmIVal;
}
