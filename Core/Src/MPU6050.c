/*
 * MPU6050.c
 *
 *  Created on: Sep 5, 2023
 *  @author: kashish Singh
 *  @file:MPU6050.c
 *  @brief: MPU6050 DRIVER WITH STM32F401
 */

#include "MPU6050.h"
#include <assert.h>
#include <stdbool.h>

/*
* @brief  This register is used to verify the identity of the device.
 *@brief  The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0’s 7-bit I2C address.
* @retval  mpu6050_status_t
*/
HAL_StatusTypeDef MPU6050_Test(void){
	uint8_t tmp;
	errorstatus = HAL_I2C_Mem_Read(MPU6050_I2C_Address & 0x7f) << 1,WHO_AM_I ,&tmp ,1);
	if(tmp != (uint8_t)0x68){
		return Error_Recognising_MPU6050;
	}
	return MPU6050_No_Error;
}


uint8_t MPU6050_Init(MPU6050 *status , I2C_HandleTypeDef* hi2c1){
	status->hi2c1  =I2Chandle;
	status->temperature = 0.0;

	//set initial value for acceleration
	status->acceleration[0] =0;
	status->acceleration[1] =0;
	status->acceleration[2] =0;

	//set initial value for gyroscope
	status->gyroscope[0] =0;
	status->gyroscope[1] =0;
	status->gyroscope[2] =0;

}

//Checks if target device is ready for communication.
HAL_StatusTypeDef ref =HAL_I2C_IsDeviceReady(status->hi2c1,0x68<<1+0,1,1,100){


}

/*
 * @brief   Read current Gyro configuration
 * @param   gyroconfig: Pointer to buffer where configuration will be stored
 * @retval  mpu6050_status_t
 */
mpu6050_status_t mpu6050_gyro_read_config(uint8_t *gyroconfig) {
  if (mpu6050_reg_read(MPU6050_GYRO_CONFIG, gyroconfig) != MPU6050_OK)
    return MPU6050_ERROR;
  return MPU6050_OK;
}

/*
 * @brief   Read current Accel configuration
 * @param   accelconfig: Pointer to buffer where configuration will be stored
 * @retval  mpu6050_status_t
 */
mpu6050_status_t mpu6050_accel_read_config(uint8_t *accelconfig) {
  if (mpu6050_reg_read(MPU6050_ACCEL_CONFIG, accelconfig) != MPU6050_OK)
    return MPU6050_ERROR;
  return MPU6050_OK;
}

/*
 *@brief : read MPU6050 register
 *@param  MPU6050_I2C_Address : address of register to read
 *@param  data pointer to buffer where data will be stored
 *@retval status
*/

HAL_StatusTypeDef MPU6050_readRegister(MPU6050 *status,uint8_t reg , uint8_t *data){ //reads 1 byte of data
//HAL I2C register Read wrapper
	return HAL_I2C_Mem_Read(status->hi2c1 ,MPU6050_I2C_Address,reg,data,1,HAL_MAX_DELAY);
	}

/*
 *@brief : Burst read MPU6050 registers
 *@param  MPU6050_I2C_Address : address of first register to read
 *@param  data pointer to buffer will where data will be stored
 *@param length : amount of data to be read 
 *@retval status
*/

HAL_StatusTypeDef MPU6050_burstRead(MPU6050 *status,uint8_t reg , uint8_t *data , uint8_t length){

	return HAL_I2C_Mem_Read(status->hi2c1,MPU6050_I2C_Address,reg,data,length,HAL_MAX_DELAY);
} 

/*
@brief :  Write MPU6050 register
@param  MPU6050_I2C_Address : address of register to write
@param  data pointer to buffer with value to write
@retval status
*/

HAL_StatusTypeDef MPU6050_writeresgister(MPU6050 *status , uint8_t reg, uint8_t *data){
//write register wrapper
	return HAL_I2C_Men_Write(status->hi2c1 ,MPU6050_I2C_Address ,reg,uint8_t,data,1,HAL_MAX_DELAY);
	
}

/*
 * @brief   MPU-6050 Non-blocking burst read DMA
 * @param   reg_address: Address of first register to read
 * @param   pdata: Pointer to buffer where received data will be stored
 * @retval  mpu6050_status_t
 */
static mpu6050_status_t mpu6050_nonblocking_read(MPU6050 *status,uint8_t *data,uint16_t length) {
  /* MPU6050 non-blocking register read wrapper */
  return HAL_I2C_Mem_Read_DMA((uint16_t)hmpu1.address << 1, reg_address, data, data_amount);
}

/*
 * @brief read raw accelerator data from from buffer
 */
void MPU6050_Read_Accel (float* Ax, float* Ay, float* Az)
{  uint8_t Rec_Data[6];
  if (mpu6050_burst_read(MPU6050_ACCEL_XOUT_H, reg_value, 6) != MPU6050_OK)
    return MPU6050_ERROR;
	uint8_t Rec_Data[6];

	HAL_I2C_Mem_Read (&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);
	//Adding 2 BYTES into 16 bit integer 
	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);
  return MPU6050_OK;
}

mpu6050_status_t mpu6050_accel_fetch(void) {
  if (mpu6050_nonblocking_read(MPU6050_ACCEL_XOUT_H, rxbuffer, 6) != MPU6050_OK)
    return MPU6050_ERROR;
  return MPU6050_OK;
}
 
/*
 * @brief read raw gyro from from buffer
 */
}
mpu6050_status_t mpu6050_gyro_read_raw(uint16_t *pgyrox, uint16_t *gyroy, uint16_t *pgyroz) {
  uint8_t reg_value[6];
  if (mpu6050_burst_read(MPU6050_GYRO_XOUT_H, reg_value, 6) != MPU6050_OK)
    return MPU6050_ERROR;

  *pgyrox = (reg_value[0] << 8) | reg_value[1];
  *pgyroy = (reg_value[2] << 8) | reg_value[3];
  *pgyroz = (reg_value[4] << 8) | reg_value[5];

  return MPU6050_OK;
}

mpu6050_status_t mpu6050_gyro_fetch(void) {
  if (mpu6050_nonblocking_read(MPU6050_GYRO_XOUT_H, rxbuffer, 6) != MPU6050_OK)
    return MPU6050_ERROR;
  return MPU6050_OK;
}
/*
 * @brief   Temperature Measurement
 * @param   ptemp: Pointer to buffer where temperature measurement will be stored

 */
mpu6050_status_t MPU6050_temp_read_raw(uint16_t *ptemp) {
  uint8_t reg_value[6];
  if (mpu6050_burst_read(MPU6050_TEMP_OUT_H, reg_value, 2) != MPU6050_OK)
    return MPU6050_ERROR;

  *ptemp = (reg_value[0] << 8) | reg_value[1];

  return MPU6050_OK;
}
