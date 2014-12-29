/*
 * LSM9DS0_tiva.h
 *
 *  Created on: Dec 28, 2014
 *      Author: freddy
 *      email:	freddy12120@gmail.com
 *
 *	this file based on:
 *	https://github.com/sparkfun/LSM9DS0_Breakout
 *
 *	it defines every register in the LSM9DS0 for gyro, accelerometer and magnetometer
 *
 *	this file only implemented the I2C communication, what it's very simple to set up on the
 *	LSM9DS0 breakout board
 *
 *	Development environment specifics:
 *			IDE: Code Composer studio v6
 *			Hardware platform: TIVA C series Launchpad 3.3V/40MHz
 *			LSM9DS0 breakout Version: 1.0
 *
 *
 *	Only for I2C, the hardware configuration is very simple
 *
 *	LSM9DS0 --------- TIVA C launchpad
 *
 *	 SCL ------------- 	PB2 (SCL)
 *	 SDA ------------- 	PB3 (SDA)
 *	 VDD ------------- 	3.3V
 *	 GND ------------- 	GND
 *
 * you don't need to worry about others pins because the board help you in this task for only for I2C configuration,
 * if you need set up SPI configuration, you can read the datasheet of the board, this task is not covered in this
 * little simple library.
 *
 */

#ifndef LSM9DS0_TIVA_H_
#define LSM9DS0_TIVA_H_


#include <stdint.h>
#include <stdbool.h>

////////////////////////////
// LSM9DS0 Gyro Registers //
////////////////////////////
#define WHO_AM_I_G			0x0F
#define CTRL_REG1_G			0x20
#define CTRL_REG2_G			0x21
#define CTRL_REG3_G			0x22
#define CTRL_REG4_G			0x23
#define CTRL_REG5_G			0x24
#define REFERENCE_G			0x25
#define STATUS_REG_G		0x27
#define OUT_X_L_G			0x28
#define OUT_X_H_G			0x29
#define OUT_Y_L_G			0x2A
#define OUT_Y_H_G			0x2B
#define OUT_Z_L_G			0x2C
#define OUT_Z_H_G			0x2D
#define FIFO_CTRL_REG_G		0x2E
#define FIFO_SRC_REG_G		0x2F
#define INT1_CFG_G			0x30
#define INT1_SRC_G			0x31
#define INT1_THS_XH_G		0x32
#define INT1_THS_XL_G		0x33
#define INT1_THS_YH_G		0x34
#define INT1_THS_YL_G		0x35
#define INT1_THS_ZH_G		0x36
#define INT1_THS_ZL_G		0x37
#define INT1_DURATION_G		0x38



//////////////////////////////////////////
// LSM9DS0 Accel/Magneto (XM) Registers //
//////////////////////////////////////////
#define OUT_TEMP_L_XM		0x05
#define OUT_TEMP_H_XM		0x06
#define STATUS_REG_M		0x07
#define OUT_X_L_M			0x08
#define OUT_X_H_M			0x09
#define OUT_Y_L_M			0x0A
#define OUT_Y_H_M			0x0B
#define OUT_Z_L_M			0x0C
#define OUT_Z_H_M			0x0D
#define WHO_AM_I_XM			0x0F
#define INT_CTRL_REG_M		0x12
#define INT_SRC_REG_M		0x13
#define INT_THS_L_M			0x14
#define INT_THS_H_M			0x15
#define OFFSET_X_L_M		0x16
#define OFFSET_X_H_M		0x17
#define OFFSET_Y_L_M		0x18
#define OFFSET_Y_H_M		0x19
#define OFFSET_Z_L_M		0x1A
#define OFFSET_Z_H_M		0x1B
#define REFERENCE_X			0x1C
#define REFERENCE_Y			0x1D
#define REFERENCE_Z			0x1E
#define CTRL_REG0_XM		0x1F
#define CTRL_REG1_XM		0x20
#define CTRL_REG2_XM		0x21
#define CTRL_REG3_XM		0x22
#define CTRL_REG4_XM		0x23
#define CTRL_REG5_XM		0x24
#define CTRL_REG6_XM		0x25
#define CTRL_REG7_XM		0x26
#define STATUS_REG_A		0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D
#define FIFO_CTRL_REG		0x2E
#define FIFO_SRC_REG		0x2F
#define INT_GEN_1_REG		0x30
#define INT_GEN_1_SRC		0x31
#define INT_GEN_1_THS		0x32
#define INT_GEN_1_DURATION	0x33
#define INT_GEN_2_REG		0x34
#define INT_GEN_2_SRC		0x35
#define INT_GEN_2_THS		0x36
#define INT_GEN_2_DURATION	0x37
#define CLICK_CFG			0x38
#define CLICK_SRC			0x39
#define CLICK_THS			0x3A
#define TIME_LIMIT			0x3B
#define TIME_LATENCY		0x3C
#define TIME_WINDOW			0x3D
#define ACT_THS				0x3E
#define ACT_DUR				0x3F

// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW



	enum gyro_scale
	{
		G_SCALE_245DPS,		// 00: +/- 245 degrees per second
		G_SCALE_500DPS,		// 01: +/- 500 dps
		G_SCALE_2000DPS,	// 10: +/- 2000 dps
	}gScl;
	// accel_scale defines all possible FSR's of the accelerometer:
	enum accel_scale
	{
		A_SCALE_2G,	// 000: +/- 2g
		A_SCALE_4G,	// 001: +/- 4g
		A_SCALE_6G,	// 010: +/- 6g
		A_SCALE_8G,	// 011: +/- 8g
		A_SCALE_16G	// 100: +/- 16g
	}aScl;
	// mag_scale defines all possible FSR's of the magnetometer:
	enum mag_scale
	{
		M_SCALE_2GS,	// 00: +/- 2Gs
		M_SCALE_4GS, 	// 01: +/- 4Gs
		M_SCALE_8GS,	// 10: +/- 8Gs
		M_SCALE_12GS,	// 11: +/- 12Gs
	}mScl;
	// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
	enum gyro_odr
	{								// ODR (Hz) --- Cutoff
		G_ODR_95_BW_125  = 0x0, 	//   95         12.5
		G_ODR_95_BW_25   = 0x1, 	//   95          25
		// 0x2 and 0x3 define the same data rate and bandwidth
		G_ODR_190_BW_125 = 0x4, 	//   190        12.5
		G_ODR_190_BW_25  = 0x5, 	//   190         25
		G_ODR_190_BW_50  = 0x6, 	//   190         50
		G_ODR_190_BW_70  = 0x7, 	//   190         70
		G_ODR_380_BW_20  = 0x8, 	//   380         20
		G_ODR_380_BW_25  = 0x9, 	//   380         25
		G_ODR_380_BW_50  = 0xA, 	//   380         50
		G_ODR_380_BW_100 = 0xB, 	//   380         100
		G_ODR_760_BW_30  = 0xC, 	//   760         30
		G_ODR_760_BW_35  = 0xD, 	//   760         35
		G_ODR_760_BW_50  = 0xE, 	//   760         50
		G_ODR_760_BW_100 = 0xF, 	//   760         100
	}gRate;
	// accel_oder defines all possible output data rates of the accelerometer:
	enum accel_odr
	{
		A_POWER_DOWN, 	// Power-down mode (0x0)
		A_ODR_3125,		// 3.125 Hz	(0x1)
		A_ODR_625,		// 6.25 Hz (0x2)
		A_ODR_125,		// 12.5 Hz (0x3)
		A_ODR_25,		// 25 Hz (0x4)
		A_ODR_50,		// 50 Hz (0x5)
		A_ODR_100,		// 100 Hz (0x6)
		A_ODR_200,		// 200 Hz (0x7)
		A_ODR_400,		// 400 Hz (0x8)
		A_ODR_800,		// 800 Hz (9)
		A_ODR_1600		// 1600 Hz (0xA)
	}aRate;
	// accel_oder defines all possible output data rates of the magnetometer:
	enum mag_odr
	{
		M_ODR_3125,	// 3.125 Hz (0x00)
		M_ODR_625,	// 6.25 Hz (0x01)
		M_ODR_125,	// 12.5 Hz (0x02)
		M_ODR_25,	// 25 Hz (0x03)
		M_ODR_50,	// 50 (0x04)
		M_ODR_100,	// 100 Hz (0x05)
	}mRate;


	int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
	int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
	int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer



	void readGyro();
	void readAccel();
	void readMag();
	float calcGyro(int16_t gyro);
	float calcAccel(int16_t accel);
	float calcMag(int16_t mag);
	void setGyroScale();
	void setAccelScale();
	void setMagScale();
	void setGyroODR();

	void setAccelODR();
	void setMagODR();
	void configGyroInt(uint8_t int1Cfg, uint16_t int1ThsX, uint16_t int1ThsY, uint16_t int1ThsZ, uint8_t duration);


	uint8_t xmAddress, gAddress;

	float gRes, aRes, mRes;

	void initGyro();
	void initAccel();
	void initMag();
	uint8_t gReadByte(uint8_t subAddress);
	void gReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	void gWriteByte(uint8_t subAddress, uint8_t data);
	uint8_t xmReadByte(uint8_t subAddress);
	void xmReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	void xmWriteByte(uint8_t subAddress, uint8_t data);
	void calcgRes();
	void calcmRes();
	void calcaRes();


	///////////////////
	// I2C Functions //
	///////////////////

	void initI2C();
	void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);
	uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);
	void I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);

#endif /* LSM9DS0_TIVA_H_ */
