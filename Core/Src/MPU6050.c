/*
 * MPU6050.c
 *
 *  Created on: Sep 5, 2023
 *      Author: kashish Singh
 *      MPU6050 DRIVER WITH STM32F401
 */

/* This register is used to verify the identity of the device.
The contents of WHO_AM_I are the upper 6 bits of the MPU-60X0’s 7-bit I2C address.
*/

HAL_StatusTypeDef MPU6050_Test(void){
	uint8_t tmp;
	errorstatus = MPU6050_ReadRegister(MPU6050_I2C_Address & 0x7f) << 1,WHO_AM_I ,&tmp ,1);
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
	////HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout)

HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050 *status,uint8_t reg , uint8_t *data){ //reads 1 byte of data
// HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	return HAL_I2C_Mem_Read(status->hi2c1 ,MPU6050_I2C_Address,reg,uint8_t,data,1,HAL_MAX_DELAY);
	//READ FROM MPU6050_I2C_ADDRESS ,  READ REG , READ 8 BITS OF DATA , STORE THE DATA AT data , 1 BYTE OF DATA TO BE READ  , WAIT HAL_MAX_DELAY for the data to be read
	}

HAL_StatusTypeDef MPU6050_ReadRegisters(MPU6050 *status,uint8_t reg , uint8_t *data , uint8_t length){

	return HAL_I2C_Mem_Read(status->hi2c1,MPU6050_I2C_Address,reg,uint8_t,data,length,HAL_MAX_DELAY);
} //reads multiple bytes of data

HAL_StatusTypeDef MPU6050_writeresgister(MPU6050 *status , uint8_t reg, uint8_t *data){
//HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
	return HAL_I2C_Men_Write(status->hi2c1 ,MPU6050_I2C_Address ,reg,uint8_t,data,1,HAL_MAX_DELAY);
	//write data to data
}
