/* Simple example of use LSM9DS0_tiva.h
 *
 * Author: Freddy Rodrigo Mendoza Ticona
 *
 * Afiliacion: UNI, Lima-Peru.
 *		Created on: Dec 28, 2014
 * 		main.c
 *
 * based on
 * https://github.com/sparkfun/LSM9DS0_Breakout/blob/master/Libraries/Arduino/SFE_LSM9DS0/examples/LSM9DS0_Simple/
 *
 *
 * Only for I2C, the hardware configuration is very simple
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
#include "driverlib/uart.h"
#include "driverlib/i2c.h"
#include "utils/uartstdio.h"

#include "LSM9DS0_tiva.h"

// functions prototypes
void configureUART(void);
void configureClock(void);


#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif



int main(void) {
	
	return 0;
}


// config Clock source
void configureClock(void){
	// system control
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //40MHz
}

// config Init UART
void configureUART(void){

	// habilita Periferico UART
	 UARTStdioConfig(0,115200,SysCtlClockGet());

	 ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	 ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	 ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
	 ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
	 ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	 // UART 115200 bps
	 ROM_UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
			 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

}
