/*
 * LSM9DS0_tiva.c
 *
 *  Created on: Dec 28, 2014
 *      Author: freddy
 *
 * this file based on:
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


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pin_map.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"
#include "driverlib/i2c.h"

#include "LSM9DS0_tiva.h"

// Init conf of IMU LSM9DS0
uint16_t InitIMU(uint8_t gScl, uint8_t aScl, uint8_t mScl, uint8_t gODR, uint8_t aODR, uint8_t mODR){

	initI2C();
	xmAddress = LSM9DS0_ADDRESS_XM;
	gAddress = LSM9DS0_ADDRESS_G;

	// are used throughout to calculate the actual g's, DPS,and Gs's.
	gScale = gScl;
	aScale = aScl;
	mScale = mScl;

	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable


	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t gTest = gReadByte(WHO_AM_I_G);		// Read the gyro WHO_AM_I
	uint8_t xmTest = xmReadByte(WHO_AM_I_XM);	// Read the accel/mag WHO_AM_I

	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
	setGyroODR(gODR); // Set the gyro output data rate and bandwidth.
	setGyroScale(gScale); // Set the gyro range

	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	setAccelODR(aODR); // Set the accel data rate.
	setAccelScale(aScale); // Set the accel range.
	//setAccelABW(A_ABW_773);

	// Magnetometer initialization stuff:
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
	setMagODR(mODR); // Set the magnetometer output data rate.
	setMagScale(mScale); // Set the magnetometer's range.

	//enable Temperature
    initTemp();

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return (xmTest << 8) | gTest;
}



// gyro read Byte. only I2C mode
uint8_t gReadByte(uint8_t subAddress){

	return I2CreadByte(gAddress, subAddress);
}

// gyro read Bytes. only I2C mode
void gReadBytes(uint8_t subAddress, uint8_t dest[], uint8_t count){

	I2CreadBytes(gAddress,subAddress,dest,count);

}
// gyro write Byte. only I2C mode
void gWriteByte(uint8_t subAddress, uint8_t data){

	I2CwriteByte(gAddress,subAddress,data);
}

// accel/mag read Byte. only I2C mode
uint8_t xmReadByte(uint8_t subAddress){

	return I2CreadByte(xmAddress, subAddress);
}

// accel/mag read Bytes. only I2C mode
void xmReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count){

	I2CreadBytes(xmAddress,subAddress,dest,count);

}

// accel/mag write Byte. only I2C mode
void xmWriteByte(uint8_t subAddress, uint8_t data){

	I2CwriteByte(xmAddress,subAddress, data);
}


//init gyroscope
void initGyro(){

	/* CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
	Bits[7:0]: DR1 DR0 BW1 BW0 PD Zen Xen Yen
	DR[1:0] - Output data rate selection
		00=95Hz, 01=190Hz, 10=380Hz, 11=760Hz
	BW[1:0] - Bandwidth selection (sets cutoff frequency)
		 Value depends on ODR. See datasheet table 21.
	PD - Power down enable (0=power down mode, 1=normal or sleep mode)
	Zen, Xen, Yen - Axis enable (o=disabled, 1=enabled)	*/
	gWriteByte(CTRL_REG1_G, 0x0F); // Normal mode, enable all axes


	/* CTRL_REG2_G sets up the HPF
	Bits[7:0]: 0 0 HPM1 HPM0 HPCF3 HPCF2 HPCF1 HPCF0
	HPM[1:0] - High pass filter mode selection
		00=normal (reset reading HP_RESET_FILTER, 01=ref signal for filtering,
		10=normal, 11=autoreset on interrupt
	HPCF[3:0] - High pass filter cutoff frequency
		Value depends on data rate. See datasheet table 26.
	*/
	gWriteByte(CTRL_REG2_G, 0x00); // Normal mode, high cutoff frequency


	/* CTRL_REG3_G sets up interrupt and DRDY_G pins
	Bits[7:0]: I1_IINT1 I1_BOOT H_LACTIVE PP_OD I2_DRDY I2_WTM I2_ORUN I2_EMPTY
	I1_INT1 - Interrupt enable on INT_G pin (0=disable, 1=enable)
	I1_BOOT - Boot status available on INT_G (0=disable, 1=enable)
	H_LACTIVE - Interrupt active configuration on INT_G (0:high, 1:low)
	PP_OD - Push-pull/open-drain (0=push-pull, 1=open-drain)
	I2_DRDY - Data ready on DRDY_G (0=disable, 1=enable)
	I2_WTM - FIFO watermark in on DRDY_G (0=disable 1=enable)
	I2_ORUN - FIFO overrun interrupt on DRDY_G (0=disable 1=enable)
	I2_EMPTY - FIFO empty interrupt on DRDY_G (0=disable 1=enable) */
	// Int1 enabled (pp, active low), data read on DRDY_G:
	gWriteByte(CTRL_REG3_G, 0x88);


	/* CTRL_REG4_G sets the scale, update mode
	Bits[7:0] - BDU BLE FS1 FS0 - ST1 ST0 SIM
	BDU - Block data update (0=continuous, 1=output not updated until read
	BLE - Big/little endian (0=data LSB @ lower address, 1=LSB @ higher add)
	FS[1:0] - Full-scale selection
		00=245dps, 01=500dps, 10=2000dps, 11=2000dps
	ST[1:0] - Self-test enable
		00=disabled, 01=st 0 (x+, y-, z-), 10=undefined, 11=st 1 (x-, y+, z+)
	SIM - SPI serial interface mode select
		0=4 wire, 1=3 wire */
	gWriteByte(CTRL_REG4_G, 0x00); // Set scale to 245 dps


	/* CTRL_REG5_G sets up the FIFO, HPF, and INT1
	Bits[7:0] - BOOT FIFO_EN - HPen INT1_Sel1 INT1_Sel0 Out_Sel1 Out_Sel0
	BOOT - Reboot memory content (0=normal, 1=reboot)
	FIFO_EN - FIFO enable (0=disable, 1=enable)
	HPen - HPF enable (0=disable, 1=enable)
	INT1_Sel[1:0] - Int 1 selection configuration
	Out_Sel[1:0] - Out selection configuration */
	gWriteByte(CTRL_REG5_G, 0x10); //enabla  HPF


}

//init Accelerometer
void initAccel(){

	/* CTRL_REG0_XM (0x1F) (Default value: 0x00)
	Bits (7-0): BOOT FIFO_EN WTM_EN 0 0 HP_CLICK HPIS1 HPIS2
	BOOT - Reboot memory content (0: normal, 1: reboot)
	FIFO_EN - Fifo enable (0: disable, 1: enable)
	WTM_EN - FIFO watermark enable (0: disable, 1: enable)
	HP_CLICK - HPF enabled for click (0: filter bypassed, 1: enabled)
	HPIS1 - HPF enabled for interrupt generator 1 (0: bypassed, 1: enabled)
	HPIS2 - HPF enabled for interrupt generator 2 (0: bypassed, 1 enabled)   */
	xmWriteByte(CTRL_REG0_XM, 0x00);

	/* CTRL_REG1_XM (0x20) (Default value: 0x07)
	Bits (7-0): AODR3 AODR2 AODR1 AODR0 BDU AZEN AYEN AXEN
	AODR[3:0] - select the acceleration data rate:
		0000=power down, 0001=3.125Hz, 0010=6.25Hz, 0011=12.5Hz,
		0100=25Hz, 0101=50Hz, 0110=100Hz, 0111=200Hz, 1000=400Hz,
		1001=800Hz, 1010=1600Hz, (remaining combinations undefined).
	BDU - block data update for accel AND mag
		0: Continuous update
		1: Output registers aren't updated until MSB and LSB have been read.
	AZEN, AYEN, and AXEN - Acceleration x/y/z-axis enabled.
		0: Axis disabled, 1: Axis enabled									 */
	xmWriteByte(CTRL_REG1_XM, 0x67); // 100Hz data rate, x/y/z all enabled

	/* CTRL_REG2_XM (0x21) (Default value: 0x00)
	Bits (7-0): ABW1 ABW0 AFS2 AFS1 AFS0 AST1 AST0 SIM
		ABW[1:0] - Accelerometer anti-alias filter bandwidth
			00=773Hz, 01=194Hz, 10=362Hz, 11=50Hz
		AFS[2:0] - Accel full-scale selection
			000=+/-2g, 001=+/-4g, 010=+/-6g, 011=+/-8g, 100=+/-16g
		AST[1:0] - Accel self-test enable
			00=normal (no self-test), 01=positive st, 10=negative st, 11=not allowed
		SIM - SPI mode selection
			0=4-wire, 1=3-wire													 */
	xmWriteByte(CTRL_REG2_XM, 0x00); // Set scale to 2g

	/* CTRL_REG3_XM is used to set interrupt generators on INT1_XM
	Bits (7-0): P1_BOOT P1_TAP P1_INT1 P1_INT2 P1_INTM P1_DRDYA P1_DRDYM P1_EMPTY
	*/
	// Accelerometer data ready on INT1_XM (0x04)
	xmWriteByte(CTRL_REG3_XM, 0x04);


}

// init Magnetometer
void initMag(){

	/* CTRL_REG5_XM enables temp sensor, sets mag resolution and data rate
	Bits (7-0): TEMP_EN M_RES1 M_RES0 M_ODR2 M_ODR1 M_ODR0 LIR2 LIR1
		TEMP_EN - Enable temperature sensor (0=disabled, 1=enabled)
		M_RES[1:0] - Magnetometer resolution select (0=low, 3=high)
		M_ODR[2:0] - Magnetometer data rate select
			000=3.125Hz, 001=6.25Hz, 010=12.5Hz, 011=25Hz, 100=50Hz, 101=100Hz
		LIR2 - Latch interrupt request on INT2_SRC (cleared by reading INT2_SRC)
			0=interrupt request not latched, 1=interrupt request latched
		LIR1 - Latch interrupt request on INT1_SRC (cleared by readging INT1_SRC)
			0=irq not latched, 1=irq latched 									 */
	xmWriteByte(CTRL_REG5_XM, 0xF0); // high resolution Mag data rate - 50 Hz, enable temperature sensor

	/* CTRL_REG6_XM sets the magnetometer full-scale
	Bits (7-0): 0 MFS1 MFS0 0 0 0 0 0
		MFS[1:0] - Magnetic full-scale selection
		00:+/-2Gauss, 01:+/-4Gs, 10:+/-8Gs, 11:+/-12Gs							 */
	xmWriteByte(CTRL_REG6_XM, 0x00); // Mag scale to +/- 2GS

	/* CTRL_REG7_XM sets magnetic sensor mode, low power mode, and filters
	AHPM1 AHPM0 AFDS 0 0 MLP MD1 MD0
		AHPM[1:0] - HPF mode selection
			00=normal (resets reference registers), 01=reference signal for filtering,
			10=normal, 11=autoreset on interrupt event
		AFDS - Filtered acceleration data selection
			0=internal filter bypassed, 1=data from internal filter sent to FIFO
		MLP - Magnetic data low-power mode
			0=data rate is set by M_ODR bits in CTRL_REG5
			1=data rate is set to 3.125Hz
		MD[1:0] - Magnetic sensor mode selection (default 10)
			00=continuous-conversion, 01=single-conversion, 10 and 11=power-down */
	xmWriteByte(CTRL_REG7_XM, 0x00); // Continuous conversion mode

	/* CTRL_REG4_XM is used to set interrupt generators on INT2_XM
	Bits (7-0): P2_TAP P2_INT1 P2_INT2 P2_INTM P2_DRDYA P2_DRDYM P2_Overrun P2_WTM
	*/
	xmWriteByte(CTRL_REG4_XM, 0x00); // accelerometer data ready on INT2_XM (0x08) disable

	/* INT_CTRL_REG_M to set push-pull/open drain, and active-low/high
	Bits[7:0] - XMIEN YMIEN ZMIEN PP_OD IEA IEL 4D MIEN
		XMIEN, YMIEN, ZMIEN - Enable interrupt recognition on axis for mag data
		PP_OD - Push-pull/open-drain interrupt configuration (0=push-pull, 1=od)
		IEA - Interrupt polarity for accel and magneto
			0=active-low, 1=active-high
		IEL - Latch interrupt request for accel and magneto
			0=irq not latched, 1=irq latched
		4D - 4D enable. 4D detection is enabled when 6D bit in INT_GEN1_REG is set
		MIEN - Enable interrupt generation for magnetic data
			0=disable, 1=enable) */
	xmWriteByte(INT_CTRL_REG_M, 0x00); // Enable interrupts for mag, active-low, push-pull
	//disable any interruption
}
void initTemp(){

	uint8_t temp;
	temp  = xmReadByte(CTRL_REG5_XM);
	xmWriteByte(CTRL_REG5_XM,temp | (1<<7));

}


void setGyroODR(uint8_t gRate){

	// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
	uint8_t temp = gReadByte(CTRL_REG1_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (gRate << 4);
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(CTRL_REG1_G, temp);

}

void setGyroScale(uint8_t gScl){
	// We need to preserve the other bytes in CTRL_REG4_G. So, first read it:
	uint8_t temp = gReadByte(CTRL_REG4_G);
	// Then mask out the gyro scale bits:
	temp &= 0xFF^(0x3 << 4);
	// Then shift in our new scale bits:
	temp |= gScl << 4;
	// And write the new register value back into CTRL_REG4_G:
	gWriteByte(CTRL_REG4_G, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update gScale:
	gScale = gScl;
	// Then calculate a new gRes, which relies on gScale being set correctly:
	calcgRes();

}

// table 26. High-pass filter cutoff frecuency configuration
void setGyroHPF(uint8_t hpcf){// HPCF 4-bits:  0-9

	// default hpcf = 0;

	uint8_t temp = gReadByte(CTRL_REG2_G);
	// Then mask out the gyro ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= hpcf;
	// And write the new register value back into CTRL_REG1_G:
	gWriteByte(CTRL_REG2_G, temp);

}
void setAccelODR(uint8_t aRate){

	// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG1_XM);
	// Then mask out the accel ODR bits:
	temp &= 0xFF^(0xF << 4);
	// Then shift in our new ODR bits:
	temp |= (aRate << 4);
	// And write the new register value back into CTRL_REG1_XM:
	xmWriteByte(CTRL_REG1_XM, temp);

}


void setAccelScale(uint8_t aScl){

	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG2_XM);
	// Then mask out the accel scale bits:
	temp &= 0xFF^(0x3 << 3);
	// Then shift in our new scale bits:
	temp |= aScl << 3;
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update aScale:
	aScale = aScl;
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();

}


void setAccelABW(uint8_t abwRate)
{
	// We need to preserve the other bytes in CTRL_REG2_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG2_XM);
	// Then mask out the accel ABW bits:
	temp &= 0xFF^(0x3 << 6);
	// Then shift in our new ODR bits:
	temp |= (abwRate << 6);
	// And write the new register value back into CTRL_REG2_XM:
	xmWriteByte(CTRL_REG2_XM, temp);
}

void setMagODR(uint8_t mRate){

	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG5_XM);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= (mRate << 2);
	// And write the new register value back into CTRL_REG5_XM:
	xmWriteByte(CTRL_REG5_XM, temp);

}


void setMagScale(uint8_t mScl){

	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = xmReadByte(CTRL_REG6_XM);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	// Then shift in our new scale bits:
	temp |= mScl << 5;
	// And write the new register value back into CTRL_REG6_XM:
	xmWriteByte(CTRL_REG6_XM, temp);

	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();

}


// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
// calibration LSM9DS0
void calLSM9DS0(float *gbias, float *abias)
{
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  int16_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
  int samples, ii;

  // First get gyro bias
  uint8_t c = gReadByte(CTRL_REG5_G);
  gWriteByte(CTRL_REG5_G, c | 0x40);         // Enable gyro FIFO
  SysCtlDelay(270000);                       // Wait for change to take effect 20ms
  gWriteByte(FIFO_CTRL_REG_G, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
  SysCtlDelay(14000000);  					 // delay 1000 milliseconds to collect FIFO samples

  samples = (gReadByte(FIFO_SRC_REG_G) & 0x1F); // Read number of stored samples

  for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO

	data[0]= gReadByte(OUT_X_L_G);
	data[1]= gReadByte(OUT_X_H_G);

	data[2]= gReadByte(OUT_Y_L_G);
	data[3]= gReadByte(OUT_Y_H_G);

	data[4]= gReadByte(OUT_Z_L_G);
	data[5]= gReadByte(OUT_Z_H_G);

    gyro_bias[0] += (((int16_t)data[1] << 8) | data[0]);
    gyro_bias[1] += (((int16_t)data[3] << 8) | data[2]);
    gyro_bias[2] += (((int16_t)data[5] << 8) | data[4]);
  }

  gyro_bias[0] /= samples; // average the data
  gyro_bias[1] /= samples;
  gyro_bias[2] /= samples;

  gbias[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
  gbias[1] = (float)gyro_bias[1]*gRes;
  gbias[2] = (float)gyro_bias[2]*gRes;

  c = gReadByte(CTRL_REG5_G);
  gWriteByte(CTRL_REG5_G, c & ~0x40);  // Disable gyro FIFO
  SysCtlDelay(270000);
  gWriteByte(FIFO_CTRL_REG_G, 0x00);   // Enable gyro bypass mode


  //  Now get the accelerometer biases
  c = xmReadByte(CTRL_REG0_XM);
  xmWriteByte(CTRL_REG0_XM, c | 0x40);      // Enable accelerometer FIFO
  SysCtlDelay(270000);                      // Wait for change to take effect
  xmWriteByte(FIFO_CTRL_REG, 0x20 | 0x1F);  // Enable accelerometer FIFO stream mode and set watermark at 32 samples
  SysCtlDelay(14000000);   // delay 1000 milliseconds to collect FIFO samples

  samples = (xmReadByte(FIFO_SRC_REG) & 0x1F); // Read number of stored accelerometer samples

  for(ii = 0; ii < samples ; ii++) {          // Read the accelerometer data stored in the FIFO

	data[0]= xmReadByte(OUT_X_L_A);
    data[1]= xmReadByte(OUT_X_H_A);

    data[2]= xmReadByte(OUT_Y_L_A);
    data[3]= xmReadByte(OUT_Y_H_A);

    data[4]= xmReadByte(OUT_Z_L_A);
    data[5]= xmReadByte(OUT_Z_L_A);

    accel_bias[0] += (((int16_t)data[1] << 8) | data[0]);
    accel_bias[1] += (((int16_t)data[3] << 8) | data[2]);
    accel_bias[2] += (((int16_t)data[5] << 8) | data[4]) - (int16_t)(1./aRes); // Assumes sensor facing up!
  }

  accel_bias[0] /= samples; // average the data
  accel_bias[1] /= samples;
  accel_bias[2] /= samples;

  abias[0] = (float)accel_bias[0]*aRes; // Properly scale data to get gs
  abias[1] = (float)accel_bias[1]*aRes;
  abias[2] = (float)accel_bias[2]*aRes;

  c = xmReadByte(CTRL_REG0_XM);
  xmWriteByte(CTRL_REG0_XM, c & ~0x40);    // Disable accelerometer FIFO
  SysCtlDelay(270000);
  xmWriteByte(FIFO_CTRL_REG, 0x00);       // Enable accelerometer bypass mode
}



void readGyro(){

	uint8_t tlo;
	int16_t thi;
	tlo = gReadByte(OUT_X_L_G);
	thi = gReadByte(OUT_X_H_G);
	thi <<= 8; thi |= tlo;
	gx = thi;

	tlo = gReadByte(OUT_Y_L_G);
	thi = gReadByte(OUT_Y_H_G);
	thi <<= 8; thi |= tlo;
	gy = thi;

	tlo = gReadByte(OUT_Z_L_G);
	thi = gReadByte(OUT_Z_H_G);
	thi <<= 8; thi |= tlo;
	gz = thi;



}
void readAccel(){
	//uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp
	///xmReadBytes(OUT_X_L_A, temp, 6); // Read 6 bytes, beginning at OUT_X_L_A

	uint8_t tlo;
	int16_t thi;
	tlo = xmReadByte(OUT_X_L_A);
	thi = xmReadByte(OUT_X_H_A);
	thi <<= 8; thi |= tlo;
	ax = thi;

	tlo = xmReadByte(OUT_Y_L_A);
	thi = xmReadByte(OUT_Y_H_A);
	thi <<= 8; thi |= tlo;
	ay = thi;

	tlo = xmReadByte(OUT_Z_L_A);
	thi = xmReadByte(OUT_Z_H_A);
	thi <<= 8; thi |= tlo;
	az = thi;

}
void readMag(){

	uint8_t tlo;
	int16_t thi;
	tlo = xmReadByte(OUT_X_L_M);
	thi = xmReadByte(OUT_X_H_M);
	thi <<= 8; thi |= tlo;
	mx = thi;

	tlo = xmReadByte(OUT_Y_L_M);
	thi = xmReadByte(OUT_Y_H_M);
	thi <<= 8; thi |= tlo;
	my = thi;

	tlo = xmReadByte(OUT_Z_L_M);
	thi = xmReadByte(OUT_Z_H_M);
	thi <<= 8; thi |= tlo;
	mz = thi;


}


void readTemp()
{
	//12 bit signed int two complement
    int16_t temp;
    uint8_t  aux;

    temp = xmReadByte(OUT_TEMP_H_XM);

    aux  = xmReadByte(OUT_TEMP_L_XM);

    temp <<= 8; temp |= aux;

    temperature=temp;

	//temperature = (((int16_t) temp[1] << 12) | temp[0] << 4 ) >> 4; // Temperature is a 12-bit signed integer
}

float calcGyro(int16_t gyro){

	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
		return gRes * gyro;

}
float calcAccel(int16_t accel){

	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
		return aRes * accel;

}
float calcMag(int16_t mag){

	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
		return mRes * mag;

}




void calcgRes()
{
	// Possible gyro scales (and their register bit settings) are:
	// 245 DPS (00), 500 DPS (01), 2000 DPS (10). Here's a bit of an algorithm
	// to calculate DPS/(ADC tick) based on that 2-bit value:

	//G_So: angular rate sensitivity (DPS/digit)

	switch (gScale)
	{
	case G_SCALE_245DPS:
		//gRes = 245.0 / 32768.0;
		gRes = 0.00875;
		break;
	case G_SCALE_500DPS:
		//gRes = 500.0 / 32768.0;
		gRes = 0.01750;
		break;
	case G_SCALE_2000DPS:
		//gRes = 2000.0 / 32768.0;
		gRes = 0.070;
		break;
	}
}

void calcaRes()
{
	// Possible accelerometer scales (and their register bit settings) are:
	// 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an
	// algorithm to calculate g/(ADC tick) based on that 3-bit value:


	switch (aScale)
	{
	case A_SCALE_2G:
		aRes = 0.061/1000.0;
		break;

	case A_SCALE_4G:
		aRes = 0.122/1000.0;
		break;
	case A_SCALE_6G:
		aRes = 0.183/1000.0;
		break;

	case A_SCALE_8G:
		aRes = 0.244/1000.0;
		break;

	case A_SCALE_16G:
		aRes = 0.732/1000.0;
		break;

	}

}

void calcmRes()
{
	// Possible magnetometer scales (and their register bit settings) are:
	// 2 Gs (00), 4 Gs (01), 8 Gs (10) 12 Gs (11). Here's a bit of an algorithm
	// to calculate Gs/(ADC tick) based on that 2-bit value:
	//mRes = mScale == M_SCALE_2GS ? 2.0 / 32768.0 :
	//       (float) (mScale << 2) / 32768.0;

	// Gauss/LSB
	switch (mScale)
	{
	case M_SCALE_2GS:
		mRes = 0.08/1000.0;
		break;
	case M_SCALE_4GS:
		mRes = 0.16/1000.0;
		break;
	case M_SCALE_8GS:
		mRes = 0.32/1000.0;
		break;
	case M_SCALE_12GS:
		mRes = 0.48/1000.0;
		break;
	}
}



// init I2C tiva c series launchpad
void initI2C(){

	    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
	    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
	    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

	    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
	    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

	    // If false the data rate is set to 100kbps and if true the data rate will
	    // be set to 400kbps.
	    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

	    //clear I2C FIFOs
	    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;




}
// write a subAdress and data, transfer when master is writing one byte to slave
void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data){

	// Tell the master module what address it will place on the bus when
	// communicating with the slave.
	I2CMasterSlaveAddrSet(I2C0_BASE, address, false); // write to bus
	I2CMasterDataPut(I2C0_BASE, subAddress);

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    while(I2CMasterBusy(I2C0_BASE));

    I2CMasterDataPut(I2C0_BASE, data);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
 	while(I2CMasterBusy(I2C0_BASE));

}

// read a single byte, transfer when master is receiving (reading) one byte of data from slave
uint8_t I2CreadByte(uint8_t address, uint8_t subAddress){


		I2CMasterSlaveAddrSet(I2C0_BASE, address, false);
		I2CMasterDataPut(I2C0_BASE, subAddress);

		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);
		while(I2CMasterBusy(I2C0_BASE));

		I2CMasterSlaveAddrSet(I2C0_BASE, address, true);
		I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
		while(I2CMasterBusy(I2C0_BASE));

		return( I2CMasterDataGet(I2C0_BASE));

}
// transfer when master is receiving (reading) multiple bytes of data from slave
/* Count: number of bytes requested to slave
 * dest:  array of data reading
 * */
void I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t *dest, uint8_t count){

	I2CMasterSlaveAddrSet(I2C0_BASE, address, false);
	I2CMasterDataPut(I2C0_BASE, subAddress);

	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
	while(I2CMasterBusy(I2C0_BASE));

	I2CMasterSlaveAddrSet(I2C0_BASE, address, true);
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
    while(I2CMasterBusy(I2C0_BASE));

    dest[0] = I2CMasterDataGet(I2C0_BASE);

    if (count == 1){

    	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while(I2CMasterBusy(I2C0_BASE));

    }else{
        int i = 0;

        for( i=1 ; i< count; i++){

           	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
           	while(I2CMasterBusy(I2C0_BASE));

            dest[i] = I2CMasterDataGet(I2C0_BASE);

         }

       	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
       	while(I2CMasterBusy(I2C0_BASE));

    }
    //send control byte and read from the register from the MCU
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

}



