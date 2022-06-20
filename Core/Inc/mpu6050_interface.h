/*
 * mpu6050_interface.h
 *
 *  Created on: Jun 13, 2022
 *      Author: adamp
 */

#ifndef INC_MPU6050_INTERFACE_H_
#define INC_MPU6050_INTERFACE_H_

typedef void (*mpu_write_fun_t)(uint16_t deviceAddress , uint8_t regAddress, uint8_t *value);
typedef void (*mpu_read_fun_t)(uint16_t deviceAddress, uint8_t regAddress,uint8_t *destination, uint16_t size);


typedef struct{
	mpu_write_fun_t MpuWriteCallback;
	mpu_read_fun_t MpuReadCallback;
}mpu_interface;

#endif /* INC_MPU6050_INTERFACE_H_ */
