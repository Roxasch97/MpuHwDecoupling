/*
 * mpu6050.c
 *
 *  Created on: Jun 13, 2022
 *      Author: adamp
 */

#include "mpu6050.h"

#define CheckStatus() if(status != mpuOk){return status;}
#define UNUSED(x) (void)(x)

MpuStatus MpuWrite(const MpuType *mpu, uint8_t regAddress, uint8_t value) {
	if (mpu->interface.MpuWriteCallback == 0) {
		return mpuNullptrError;
	}
	return mpu->interface.MpuWriteCallback(mpu->address, regAddress, &value);
}

MpuStatus MpuRead(const MpuType *mpu, uint8_t regAddress, uint16_t size,
		uint8_t *destination) {
	if (mpu->interface.MpuReadCallback == 0) {
		return mpuNullptrError;
	}
	return mpu->interface.MpuReadCallback(mpu->address, regAddress, destination,
			size);
}

uint8_t MpuWhoAmI(const MpuType *mpu) {
	uint8_t WhoAmIVal = 0;
	MpuRead(mpu, MPU6050_REG_WHO_AM_I, 1, &WhoAmIVal);
	return WhoAmIVal;
}

MpuStatus MpuMemoryWrite(const MpuType *mpu, uint8_t regAddress, uint8_t data) {

	if (MpuWhoAmI(mpu) == WHO_AM_I) {
		return MpuWrite(mpu, regAddress, data);
	} else {
		return mpuAbsentError;
	}
}

MpuStatus MpuMemoryRead(const MpuType *mpu, const uint8_t regAddress,
		uint16_t size, uint8_t *destination) {

	if (MpuWhoAmI(mpu) == WHO_AM_I) {
		return MpuRead(mpu, regAddress, size, destination);
	} else {
		return mpuAbsentError;
	}

}

MpuStatus MpuReset(const MpuType *mpu) {
	return MpuMemoryWrite(mpu, MPU6050_REG_PWR_MGMT_1, RESET_VALUE);
}

MpuStatus MpuInitialize(MpuType *mpu) {
	MpuStatus status = mpuGenericError;
	if (MpuWhoAmI(mpu) == WHO_AM_I) {
		status = MpuReset(mpu);
		CheckStatus();
		status = MpuSetSamplerateDivider(mpu);
		CheckStatus();
		status = MpuSetGyroFSR(mpu);
		CheckStatus();
		status = MpuSetAccelFSR(mpu);
		CheckStatus();
		status = MpuMemoryWrite(mpu, MPU6050_REG_PWR_MGMT_1, 0);
		return status;
	} else {
		return mpuAbsentError;
	}
}

int16_t MpuReadAccelRaw(const MpuType *mpu, uint8_t regAddress) {
	uint8_t data[2] = { 0 };
	if (MpuMemoryRead(mpu, regAddress, 2, data) != mpuOk) {
		return 0;
	}
	return (int16_t) (data[0] << 8 | data[1]);
}

int16_t MpuReadAccelXRaw(const MpuType *mpu) {
	return MpuReadAccelRaw(mpu, MPU6050_REG_ACCEL_XOUT_H);
}

int16_t MpuReadAccelYRaw(const MpuType *mpu) {
	return MpuReadAccelRaw(mpu, MPU6050_REG_ACCEL_YOUT_H);
}

int16_t MpuReadAccelZRaw(const MpuType *mpu) {
	return MpuReadAccelRaw(mpu, MPU6050_REG_ACCEL_ZOUT_H);
}

float MpuConvertAccel(const MpuType *mpu, int16_t rawAccel) {

	if (mpu->accelConvFactor == 0) {
		return 0;
	}
	return (float) rawAccel / (float) mpu->accelConvFactor;
}

MpuStatus MpuSetAccelFSR(MpuType *mpu) {

	MpuStatus status = mpuGenericError;
	if (mpu->accelFullScaleRange != MPU_ACCEL_2G_FSR
			&& mpu->accelFullScaleRange != MPU_ACCEL_4G_FSR
			&& mpu->accelFullScaleRange != MPU_ACCEL_8G_FSR
			&& mpu->accelFullScaleRange != MPU_ACCEL_16G_FSR) {
		return mpuGenericError;
	}

	uint8_t tmp;
	status = MpuMemoryRead(mpu, MPU6050_REG_ACCEL_CONFIG, 1, &tmp);
	CheckStatus();
	tmp &= 0xE7; //to clear AFS_SEL bits
	tmp |= mpu->accelFullScaleRange;
	status = MpuMemoryWrite(mpu, MPU6050_REG_ACCEL_CONFIG, tmp);
	CheckStatus();

	switch (mpu->accelFullScaleRange) {
	case MPU_ACCEL_2G_FSR:
		mpu->accelConvFactor = MPU_2G_CONV_FACTOR;
		break;
	case MPU_ACCEL_4G_FSR:
		mpu->accelConvFactor = MPU_4G_CONV_FACTOR;
		break;
	case MPU_ACCEL_8G_FSR:
		mpu->accelConvFactor = MPU_8G_CONV_FACTOR;
		break;
	case MPU_ACCEL_16G_FSR:
		mpu->accelConvFactor = MPU_16G_CONV_FACTOR;
		break;
	default:
		mpu->gyroConvFactor = 0;
	}

	return mpuOk;
}

MpuStatus MpuSetGyroFSR(MpuType *mpu) {

	MpuStatus status = mpuGenericError;

	if (mpu->gyroFullScaleRange != MPU_GYRO_250_FSR
			&& mpu->gyroFullScaleRange != MPU_GYRO_500_FSR
			&& mpu->gyroFullScaleRange != MPU_GYRO_1000_FSR
			&& mpu->gyroFullScaleRange != MPU_GYRO_2000_FSR) {
		return mpuGenericError;
	}

	uint8_t tmp;
	status = MpuMemoryRead(mpu, MPU6050_REG_GYRO_CONFIG, 1, &tmp);
	CheckStatus();
	tmp &= 0xE7; //to clear AFS_SEL bits
	tmp |= mpu->gyroFullScaleRange;
	status = MpuMemoryWrite(mpu, MPU6050_REG_GYRO_CONFIG, tmp);
	CheckStatus();
	switch (mpu->gyroFullScaleRange) {
	case MPU_GYRO_250_FSR:
		mpu->gyroConvFactor = MPU_250_DEG_CONV_FACTOR;
		break;
	case MPU_GYRO_500_FSR:
		mpu->gyroConvFactor = MPU_500_DEG_CONV_FACTOR;
		break;
	case MPU_GYRO_1000_FSR:
		mpu->gyroConvFactor = MPU_1000_DEG_CONV_FACTOR;
		break;
	case MPU_GYRO_2000_FSR:
		mpu->gyroConvFactor = MPU_2000_DEG_CONV_FACTOR;
		break;
	default:
		mpu->gyroConvFactor = 0;
	}
	return mpuOk;
}

MpuStatus MpuSetSamplerateDivider(const MpuType *mpu) {
	return MpuMemoryWrite(mpu, MPU6050_REG_SMPLRT_DIV, mpu->samplerateDiv);
}

__attribute__((weak)) void MpuHandleErrors(MpuStatus status) {
	UNUSED(status);
}

