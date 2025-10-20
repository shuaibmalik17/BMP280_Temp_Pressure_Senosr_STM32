/*
 * BMP280_STM32.c
 *
 *  Created on: Jan 18, 2025
 *      Author: shuaib
 */

#include "BMP280_STM32.h"

extern I2C_HandleTypeDef hi2c1;
#define BMP280_I2C &hi2c1

#define BMP280_ADDRESS 0xEC

extern float Temperature;
extern float Pressure;

uint8_t chipID;

uint8_t TrimParam[36];
int32_t tRaw;
int32_t pRaw;

uint16_t dig_T1, \
		 dig_P1;

int16_t  dig_T2, dig_T3, \
		 dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// Read the Trimming parameters saved in the NVM ROM of the device
void TrimRead(void)
{
	uint8_t trimdata[24];
	// Read NVM from 0x88 to 0xA1
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, 0x88, 1, trimdata, 24, HAL_MAX_DELAY);

	// Arrange the data as per the datasheet (page no. 24)
	dig_T1 = (trimdata[1]<<8) | trimdata[0];
	dig_T2 = (trimdata[3]<<8) | trimdata[2];
	dig_T3 = (trimdata[5]<<8) | trimdata[4];
	dig_P1 = (trimdata[7]<<8) | trimdata[5];
	dig_P2 = (trimdata[9]<<8) | trimdata[6];
	dig_P3 = (trimdata[11]<<8) | trimdata[10];
	dig_P4 = (trimdata[13]<<8) | trimdata[12];
	dig_P5 = (trimdata[15]<<8) | trimdata[14];
	dig_P6 = (trimdata[17]<<8) | trimdata[16];
	dig_P7 = (trimdata[19]<<8) | trimdata[18];
	dig_P8 = (trimdata[21]<<8) | trimdata[20];
	dig_P9 = (trimdata[23]<<8) | trimdata[22];
}

int BMP280_Config (uint8_t osrs_t, uint8_t osrs_p, uint8_t mode, uint8_t t_sb, uint8_t filter)
{
	// Read the Trimming parameters
	TrimRead();


	uint8_t datatowrite = 0;
	uint8_t datacheck = 0;

	// Reset the device
	datatowrite = 0xB6;  // reset sequence
	if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS, RESET_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
	{
		return -1;
	}

	HAL_Delay (100);


	// write the standby time and IIR filter coeff to 0xF5
	datatowrite = (t_sb <<5) |(filter << 2);
	if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS, CONFIG_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
	{
		return -1;
	}
	HAL_Delay (100);
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, CONFIG_REG, 1, &datacheck, 1, 1000);
	if (datacheck != datatowrite)
	{
		return -1;
	}


	// write the pressure and temp oversampling along with mode to 0xF4
	datatowrite = (osrs_t <<5) |(osrs_p << 2) | mode;
	if (HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS, CTRL_MEAS_REG, 1, &datatowrite, 1, 1000) != HAL_OK)
	{
		return -1;
	}
	HAL_Delay (100);
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, CTRL_MEAS_REG, 1, &datacheck, 1, 1000);
	if (datacheck != datatowrite)
	{
		return -1;
	}

	return 0;
}

int BMPReadRaw(void)
{
	uint8_t RawData[6];

	// Check the chip ID before reading
	HAL_I2C_Mem_Read(&hi2c1, BMP280_ADDRESS, ID_REG, 1, &chipID, 1, 1000);

	if (chipID == 0x58)
	{
		// Read the Registers 0xF7 to 0xFE
		HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, PRESS_MSB_REG, 1, RawData, 6, HAL_MAX_DELAY);

		/* Calculate the Raw data for the parameters
		 * Here the Pressure and Temperature are in 20 bit format and humidity in 16 bit format
		 */
		pRaw = (RawData[0]<<12)|(RawData[1]<<4)|(RawData[2]>>4);
		tRaw = (RawData[3]<<12)|(RawData[4]<<4)|(RawData[5]>>4);

		return 0;
	}

	else return -1;
}

void BMP280_WakeUP(void)
{
	uint8_t datatowrite = 0;

	// first read the register
	HAL_I2C_Mem_Read(BMP280_I2C, BMP280_ADDRESS, CTRL_MEAS_REG, 1, &datatowrite, 1, 1000);

	// modify the data with the forced mode
	datatowrite = datatowrite | MODE_FORCED;

	// write the new data to the register
	HAL_I2C_Mem_Write(BMP280_I2C, BMP280_ADDRESS, CTRL_MEAS_REG, 1, &datatowrite, 1, 1000);

	HAL_Delay (100);
}

int32_t t_fine;
int32_t BMP280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))>> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

uint32_t BMP280_compensate_P_int32(int32_t adc_P)
{
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) *var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t)var1);
	}
	else
	{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	return p;
}

void BMP280_Measure (void)
{
	if (BMPReadRaw() == 0)
	{
		  if (tRaw == 0x800000) Temperature = 0; // value in case temp measurement was disabled
		  else
		  {
			  Temperature = (BMP280_compensate_T_int32 (tRaw))/100.0;  // as per datasheet, the temp is x100
		  }
		  if (pRaw == 0x800000) Pressure = 0; // value in case temp measurement was disabled
		  else
		  {
			  Pressure = (BMP280_compensate_P_int32 (pRaw))/256.0;  // as per datasheet, the pressure is Pa

		  }

	}
	else
	{
		Temperature = Pressure = 0;
	}
}
