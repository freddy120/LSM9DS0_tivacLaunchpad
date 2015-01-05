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
 *	Only for I2C, the hardware configuration is very simple, any interrupt on the IMU is enabled
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


// gyro_scale defines the possible full-scale ranges of the gyroscope:
#define G_SCALE_245DPS 		0x00		// 00: +/- 245 degrees per second
#define G_SCALE_500DPS 		0x01		// 01: +/- 500 dps
#define G_SCALE_2000DPS		0x02		// 10: +/- 2000 dps

// accel_scale defines all possible FSR's (full-scale range) of the accelerometer:
#define A_SCALE_2G 			0x00	// 000: +/- 2g
#define A_SCALE_4G 			0x01	// 001: +/- 4g
#define A_SCALE_6G 			0x02 	// 010: +/- 6g
#define A_SCALE_8G			0x03	// 011: +/- 8g
#define A_SCALE_16G			0x04	// 100: +/- 16g

// mag_scale defines all possible FSR's of the magnetometer:

#define M_SCALE_2GS 		0x00	// 00: +/- 2Gs
#define M_SCALE_4GS 		0x01	// 01: +/- 4Gs
#define M_SCALE_8GS			0x02	// 10: +/- 8Gs
#define M_SCALE_12GS		0x03	// 11: +/- 12Gs

// gyro_odr defines all possible data rate/bandwidth combos of the gyro:
												// ODR (Hz) --- Cutoff
#define G_ODR_95_BW_125  			0x0 		//   95         12.5
#define G_ODR_95_BW_25   			0x1 		//   95          25
// 0x2 and 0x3 define the same data rate and bandwidth
#define G_ODR_190_BW_125 			0x4 		//   190        12.5
#define G_ODR_190_BW_25  			0x5 		//   190         25
#define G_ODR_190_BW_50  			0x6 		//   190         50
#define G_ODR_190_BW_70  			0x7 		//   190         70
#define G_ODR_380_BW_20  			0x8 		//   380         20
#define G_ODR_380_BW_25  			0x9 		//   380         25
#define G_ODR_380_BW_50  			0xA 		//   380         50
#define G_ODR_380_BW_100 			0xB 		//   380         100
#define G_ODR_760_BW_30  			0xC 		//   760         30
#define G_ODR_760_BW_35  			0xD 		//   760         35
#define G_ODR_760_BW_50  			0xE 		//   760         50
#define G_ODR_760_BW_100 			0xF 		//   760         100

//high-pass filter cutoff frequency gyroscopy depend of ODR
												// Cutoff     ODR 95(Hz) 	ODR 190(Hz)		ODR 380(Hz)  ODR 760(Hz)
#define HPCF_0			0x00 					//      		7.2				13.5			27			51.4
#define HPCF_1			0x01 					//      		3.5				7.2			    13.5		27
#define HPCF_2			0x02 					//      		1.8				3.5				7.2			13.5
#define HPCF_3			0x03 					//      		0.9				1.8				3.5			7.2
#define HPCF_4			0x04 					//      		0.45			0.9			    1.8			3.5
#define HPCF_5			0x05 					//      		0.18			0.45			0.9			1.8
#define HPCF_6			0x06 					//      		0.09			0.18			0.45		0.9
#define HPCF_7			0x07 					//      		0.045			0.09			0.18		0.18
#define HPCF_8			0x08 					//      		0.018			0.045			0.09		0.18
#define HPCF_9			0x09 					//      		0.009			0.018			0.045		0.09



// accel_oder defines all possible output data rates of the accelerometer:
#define	A_POWER_DOWN 		0x00 	// Power-down mode (0x0)
#define A_ODR_3125			0x01	// 3.125 Hz	(0x1)
#define A_ODR_625			0x02	// 6.25 Hz (0x2)
#define A_ODR_125			0x03	// 12.5 Hz (0x3)
#define A_ODR_25			0x04	// 25 Hz (0x4)
#define A_ODR_50			0x05	// 50 Hz (0x5)
#define A_ODR_100			0x06	// 100 Hz (0x6)
#define A_ODR_200			0x07	// 200 Hz (0x7)
#define A_ODR_400			0x08	// 400 Hz (0x8)
#define A_ODR_800			0x09	// 800 Hz (9)
#define A_ODR_1600			0x0A	// 1600 Hz (0xA)


// accel_abw defines all possible anti-aliasing filter rates of the accelerometer:
#define A_ABW_773			0x00		// 773 Hz (0x0)
#define A_ABW_194			0x01		// 194 Hz (0x1)
#define A_ABW_362			0x02		// 362 Hz (0x2)
#define A_ABW_50			0x03		//  50 Hz (0x3)


// accel_oder defines all possible output data rates of the magnetometer:
#define M_ODR_3125			0x00	// 3.125 Hz (0x00)
#define M_ODR_625			0x01	// 6.25 Hz (0x01)
#define M_ODR_125			0x02	// 12.5 Hz (0x02)
#define M_ODR_25			0x03	// 25 Hz (0x03)
#define M_ODR_50			0x04	// 50 (0x04)
#define M_ODR_100			0x05	// 100 Hz (0x05)


// SDO_XM and SDO_G are both grounded, so our addresses are:
#define LSM9DS0_ADDRESS_XM  0x1D
#define LSM9DS0_ADDRESS_G   0x6B



 	//We'll store the gyro, accel, and magnetometer readings in a series of
	// public class variables. Each sensor gets three variables -- one for each
	// axis. Call readGyro(), readAccel(), and readMag() first, before using
	// these variables!

	// These values are the RAW signed 16-bit readings from the sensors.
	int16_t gx, gy, gz; // x, y, and z axis readings of the gyroscope
	int16_t ax, ay, az; // x, y, and z axis readings of the accelerometer
	int16_t mx, my, mz; // x, y, and z axis readings of the magnetometer
    int16_t temperature;
    float tempf;

	float abias[3];
	float gbias[3];

	// I2C address
	uint8_t xmAddress, gAddress;

	// gRes, aRes, and mRes store the current resolution for each sensor.
	// Units of these values would be DPS (or g's or Gs's) per ADC tick.
	// This value is calculated as (sensor scale) / (2^15).
	float gRes, aRes, mRes;

	uint8_t  gScale;
	uint8_t  aScale;
	uint8_t  mScale;



	// InitIMU() -- Initialize the gyro, accelerometer, and magnetometer.
	// This will set up the scale and output rate of each sensor. It'll also
	// "turn on" every sensor and every axis of every sensor.
	// Input:
	//	- gScl = The scale of the gyroscope.
	//	- aScl = The scale of the accelerometer.
	//	- mScl = The scale of the magnetometer.
	//	- gODR = Output data rate of the gyroscope.
	//	- aODR = Output data rate of the accelerometer.
	//	- mODR = Output data rate of the magnetometer.
	// Output: The function will return an unsigned 16-bit value. The most-sig
	//		bytes of the output are the WHO_AM_I reading of the accel. The
	//		least significant two bytes are the WHO_AM_I reading of the gyro.
	// All parameters have a defaulted value.
	// Default values are FSR's of:  245DPS, 2g, 2Gs; ODRs of 95 Hz for
	// gyro, 100 Hz for accelerometer, 100 Hz for magnetometer.
	// Use the return value of this function to verify communication.
	uint16_t InitIMU(uint8_t gScl, uint8_t aScl, uint8_t mScl, uint8_t gODR, uint8_t aODR, uint8_t mODR);


	void calLSM9DS0(float *gbias, float *abias);

	// readGyro() -- Read the gyroscope output registers.
	// This function will read all six gyroscope output registers.
	// The readings are stored in the gx, gy, and gz variables. Read
	// those _after_ calling readGyro().
	void readGyro();


	// readAccel() -- Read the accelerometer output registers.
	// This function will read all six accelerometer output registers.
	// The readings are stored in the  ax, ay, and az variables. Read
	// those _after_ calling readAccel().
	void readAccel();


	// readMag() -- Read the magnetometer output registers.
	// This function will read all six magnetometer output registers.
	// The readings are stored in the  mx, my, and mz variables. Read
	// those _after_ calling readMag().
	void readMag();

	// readTemp() -- Read the temperature output register.
	// This function will read two temperature output registers.
	// The combined readings are stored in the temperature variables. Read
	// those _after_ calling readTemp().
	void readTemp();


	// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
	// This function reads in a signed 16-bit value and returns the scaled
	// DPS. This function relies on gScale and gRes being correct.
	// Input:
	//	- gyro = A signed 16-bit raw reading from the gyroscope.
	float calcGyro(int16_t gyro);


	// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
	// This function reads in a signed 16-bit value and returns the scaled
	// g's. This function relies on aScale and aRes being correct.
	// Input:
	//	- accel = A signed 16-bit raw reading from the accelerometer.
	float calcAccel(int16_t accel);


	// calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
	// This function reads in a signed 16-bit value and returns the scaled
	// Gs. This function relies on mScale and mRes being correct.
	// Input:
	//	- mag = A signed 16-bit raw reading from the magnetometer.
	float calcMag(int16_t mag);


	// setGyroScale() -- Set the full-scale range of the gyroscope.
	// This function can be called to set the scale of the gyroscope to
	// 245, 500, or 200 degrees per second.
	// Input:
	// 	- gScl = The desired gyroscope scale.
	void setGyroScale(uint8_t gScl);

	// setAccelScale() -- Set the full-scale range of the accelerometer.
	// This function can be called to set the scale of the accelerometer to
	// 2, 4, 6, 8, or 16 g's.
	// Input:
	// 	- aScl = The desired accelerometer scale.
	void setAccelScale(uint8_t aScl);


	// setMagScale() -- Set the full-scale range of the magnetometer.
	// This function can be called to set the scale of the magnetometer to
	// 2, 4, 8, or 12 Gs.
	// Input:
	// 	- mScl = The desired magnetometer scale.
	void setMagScale(uint8_t mScl);



	// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
	// Input:
	//	- gRate = The desired output rate and cutoff frequency of the gyro.
	void setGyroODR(uint8_t gRate);

	// setGyroHPF(uint8_t hpcf) -- set high-pass filter cutoff frequency
	// Input:
	//	-  hpcf = high-pass filter cutoff frequency of the gyro .
	void setGyroHPF(uint8_t hpcf);

	// setAccelODR() -- Set the output data rate of the accelerometer
	// Input:
	//	- aRate = The desired output rate of the accel.
	void setAccelODR(uint8_t aRate);


	// setAccelABW() -- Set the anti-aliasing filter rate of the accelerometer
	// Input:
	//	- abwRate = The desired anti-aliasing filter rate of the accel.
	void setAccelABW(uint8_t abwRate);


	// setMagODR() -- Set the output data rate of the magnetometer
	// Input:
	//	- mRate = The desired output rate of the mag.
	void setMagODR(uint8_t mRate);


	// initGyro() -- Sets up the gyroscope to begin reading.
	// This function steps through all five gyroscope control registers.
	// Upon exit, the following parameters will be set:
	//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled.
	//		95 Hz ODR, 12.5 Hz cutoff frequency.
	//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
	//		set to 7.2 Hz (depends on ODR).
	//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
	//		active high). Data-ready output enabled on DRDY_G.
	//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
	//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
	//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled
	void initGyro();


	// initAccel() -- Sets up the accelerometer to begin reading.
	// This function steps through all accelerometer related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
	//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
	//		all axes enabled.
	//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
	//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
	void initAccel();


	// initMag() -- Sets up the magnetometer to begin reading.
	// This function steps through all magnetometer-related control registers.
	// Upon exit these registers will be set as:
	//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
	//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
	//		requests don't latch. Temperature sensor disabled.
	//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
	//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
	//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
	void initMag();


	// initTemp() -- enable sensor temperature
	void initTemp();


	// gReadByte() -- Reads a byte from a specified gyroscope register.
	// Input:
	// 	- subAddress = Register to be read from.
	// Output:
	// 	- An 8-bit value read from the requested address.
	uint8_t gReadByte(uint8_t subAddress);

	// gReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the gyroscope.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	void gReadBytes(uint8_t subAddress, uint8_t dest[], uint8_t count);

	// gWriteByte() -- Write a byte to a register in the gyroscope.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void gWriteByte(uint8_t subAddress, uint8_t data);


	// xmReadByte() -- Read a byte from a register in the accel/mag sensor
	// Input:
	//	- subAddress = Register to be read from.
	// Output:
	//	- An 8-bit value read from the requested register.
	uint8_t xmReadByte(uint8_t subAddress);


	// xmReadBytes() -- Reads a number of bytes -- beginning at an address
	// and incrementing from there -- from the accelerometer/magnetometer.
	// Input:
	// 	- subAddress = Register to be read from.
	// 	- * dest = A pointer to an array of uint8_t's. Values read will be
	//		stored in here on return.
	//	- count = The number of bytes to be read.
	// Output: No value is returned, but the `dest` array will store
	// 	the data read upon exit.
	void xmReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count);


	// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
	// Input:
	//	- subAddress = Register to be written to.
	//	- data = data to be written to the register.
	void xmWriteByte(uint8_t subAddress, uint8_t data);


	// calcgRes() -- Calculate the resolution of the gyroscope.
	// This function will set the value of the gRes variable. gScale must
	// be set prior to calling this function.
	void calcgRes();

	// calcmRes() -- Calculate the resolution of the magnetometer.
	// This function will set the value of the mRes variable. mScale must
	// be set prior to calling this function.
	void calcmRes();


	// calcaRes() -- Calculate the resolution of the accelerometer.
	// This function will set the value of the aRes variable. aScale must
	// be set prior to calling this function.
	void calcaRes();


	///////////////////
	// I2C Functions //
	///////////////////

	// initI2C() -- Initialize the I2C hardware.
	// This function will setup all I2C pins and related hardware.
	void initI2C();

	// I2CwriteByte() -- Write a byte out of I2C to a register in the device
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be written to.
	//	- data = Byte to be written to the register.
	void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data);



	// I2CreadByte() -- Read a single byte from a register over I2C.
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to be read from.
	// Output:
	//	- The byte read from the requested address.
	uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);


	// I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
	// Input:
	//	- address = The 7-bit I2C address of the slave device.
	//	- subAddress = The register to begin reading.
	// 	- * dest = Pointer to an array where we'll store the readings.
	//	- count = Number of registers to be read.
	// Output: No value is returned by the function, but the registers read are
	// 		all stored in the *dest array given.
	void I2CreadBytes(uint8_t address, uint8_t subAddress,uint8_t *dest, uint8_t count);

#endif /* LSM9DS0_TIVA_H_ */
