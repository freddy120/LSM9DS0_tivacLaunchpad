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


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
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
// write a byte
void I2CwriteByte(uint8_t address, uint8_t subAddress, uint8_t data){

	// Tell the master module what address it will place on the bus when
	// communicating with the slave.
	I2CMasterSlaveAddrSet(I2C0_BASE, address, false);

	//put data to be sent into FIFO
	I2CMasterDataPut(I2C0_BASE, subAddress);
	//put data to be sent into FIFO
	I2CMasterDataPut(I2C0_BASE, data);

	//Initiate send of data from the MCU
	I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

	// Wait until MCU is done transferring.
	 while(I2CMasterBusy(I2C0_BASE));

}





