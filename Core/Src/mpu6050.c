/*
 * mpu6050.c
 *
 *  Created on: Jun 13, 2022
 *      Author: adamp
 */


#include "mpu6050.h"


void MpuWrite(MpuType *mpu , uint8_t regAddress, uint8_t value)
{
	mpu->interface.MpuWriteCallback(mpu->address, regAddress, &value);
}

void MpuRead(MpuType *mpu ,uint8_t regAddress, uint16_t size, uint8_t *destination)
{
	mpu->interface.MpuReadCallback(regAddress, size, destination, mpu->address);
}

uint8_t MpuWhoAmI(MpuType *mpu)
{
	uint8_t WhoAmIVal=0;
	MpuRead(mpu, MPU6050_REG_WHO_AM_I, 1, &WhoAmIVal);
	return WhoAmIVal;
}

void MpuMemoryWrite(MpuType *mpu, uint8_t regAddress, uint8_t data){

	if(MpuWhoAmI(mpu)==0x68U)
	{
		MpuWrite(mpu, regAddress, data);
	}
	else
	{
	/* Error handling to be done eventually */
	}

}

void MpuMemoryRead(MpuType *mpu ,uint8_t regAddress, uint16_t size, uint8_t *destination){

	if(MpuWhoAmI(mpu)==0x68U)
	{
		MpuRead(mpu, regAddress, size, destination);
	}
	else
	{
	/* Error handling to be done eventually */
	}

}



void MpuReset(MpuType *mpu)
{
	MpuMemoryWrite(mpu, MPU6050_REG_PWR_MGMT_1, RESET_VALUE);
}

void MpuSetFullScaleRange(MpuType *mpu){
	uint8_t value = 0;
	MpuMemoryRead(mpu, MPU6050_REG_GYRO_CONFIG, 1, &value);
	value >>= 3U;
	value &= 3U;
	mpu->FullScaleRange = LSB_Sensitivity >> value;
}

void MpuInitialize(MpuType *mpu)
{
	if(MpuWhoAmI(mpu)==0x68U)
		{
			MpuReset(mpu);
			MpuMemoryWrite(mpu, MPU6050_REG_SMPLRT_DIV, mpu->SMPRT_DIV);
			MpuMemoryWrite(mpu, MPU6050_REG_GYRO_CONFIG, mpu->GYRO_CONFIG);
			MpuMemoryWrite(mpu, MPU6050_REG_ACCEL_CONFIG, mpu->ACCEL_CONFIG);
			MpuMemoryWrite(mpu, MPU6050_REG_INT_ENABLE, mpu->INT_ENABLE);
			MpuMemoryWrite(mpu, MPU6050_REG_PWR_MGMT_1, mpu->PWR_MGMT_1);
			MpuSetFullScaleRange(mpu);
		}
		else
		{
		/* Error handling to be done eventually */
		}

}

int16_t MpuReadAccelXRaw (MpuType *mpu)
{
	uint8_t data[2];
	MpuMemoryRead(mpu, MPU6050_REG_ACCEL_XOUT_H, 2, &data[0]);

	return (int16_t)(data[0]<<8 | data[1]);
}

int16_t MpuReadAccelYRaw (MpuType *mpu)
{
	uint8_t data[2];
	MpuMemoryRead(mpu, MPU6050_REG_ACCEL_YOUT_H, 2, &data[0]);

	return (int16_t)(data[0]<<8 | data[1]);
}

int16_t MpuReadAccelZRaw (MpuType *mpu)
{
	uint8_t data[2];
	MpuMemoryRead(mpu, MPU6050_REG_ACCEL_ZOUT_H, 2, &data[0]);

	return (int16_t)(data[0]<<8 | data[1]);
}

float MpuConvertAccel(MpuType *mpu, int16_t rawAccel){
	if(mpu->FullScaleRange != 0)
	{
	return rawAccel/mpu->FullScaleRange;
	}
	else
	{
	/* Error handling to be done eventually */
	return 0;
	}
}




