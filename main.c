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

#ifndef M_PI
#define M_PI                    3.14159265358979323846
#endif



#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif


// functions prototypes
void configureUART(void);
void configureClock(void);
void configureFPU(void);

void printTestAdafruitAHRS(void);
void printValuesRAW(void);


void ftoa(float f,char *buf);
void UARTstringf(char *car,uint8_t len);


// get orientation
float heading;
float pitch;
float roll;
void getOrientation(float hx, float hy, float hz, float x, float y, float z);


// compass calibration - accelerometer and magnetometer
float axf, ayf, azf, gxf, gyf, gzf;
void printValuesCalib(void); //printf values - bias

int main(void) {
	
	configureFPU();
	configureClock();
	configureUART();

	//set up LSM9DSO IMU
	uint16_t status = InitIMU(G_SCALE_245DPS, A_SCALE_2G, M_SCALE_2GS, G_ODR_95_BW_125,A_ODR_100,M_ODR_100);





	// begin() returns a 16-bit value which includes both the gyro
	// and accelerometers WHO_AM_I response. You can check this to
	// make sure communication was successful.
	  UARTprintf("LSM9DS0 WHO_AM_I's returned: 0x");
	  UARTprintf("%x",status);
	  UARTprintf("\n");
	  UARTprintf(" Should be 0x49D4");

	  SysCtlDelay(6700000); // delay 500ms
	  SysCtlDelay(6700000); // delay 500ms
	  SysCtlDelay(6700000); // delay 500ms
	  SysCtlDelay(6700000); // delay 500ms

	  calLSM9DS0(gbias, abias);

	while(1){


		//printGyro();  // Print "G: gx, gy, gz"
		//printAccel(); // Print "A: ax, ay, az"
		//printMag();   // Print "M: mx, my, mz"

		 //printValuesRAW();
		 printValuesCalib();

	    // printTestAdafruitAHRS();
		//SysCtlDelay(6700000); // delay 500ms
		 SysCtlDelay(1340000); // delay 100ms


	}
}


// enabled FPU
void configureFPU(void){
	ROM_FPULazyStackingEnable();
    ROM_FPUEnable();

}
// config Clock source
void configureClock(void){
	// system control
	ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //40MHz
   // SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ);
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
	 //
	 // Use the internal 16MHz oscillator as the UART clock source.
	 //
	 //ROM_UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	 ROM_UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
			 (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));

	 //UARTStdioConfig(0, 115200, 16000000);
}



// Compute orientation based on accelerometer and magnetometer data.
void getOrientation(float hx, float hy, float hz, float x, float y, float z)
{

	// roll: Rotation around the X-axis. -180 <= roll <= 180
	// a positive roll angle is defined to be a clockwise rotation about the positive X-axis
	 //
	 //                    y
	 //      roll = atan2(---)
	 //                    z
	 //

    roll = atan2f(y, z);


    // pitch: Rotation around the Y-axis. -180 <= roll <= 180
    // a positive pitch angle is defined to be a clockwise rotation about the positive Y-axis
    //
    //                                 -x
    //      pitch = atan(-------------------------------)
    //                    y * sin(roll) + z * cos(roll)
    //
    // where:  x, y, z are returned value from accelerometer sensor

    if (y*sinf(roll)+z*cosf(roll) == 0) {

        pitch =  x > 0 ? (M_PI/2) : (-M_PI/2);

    }else{

        pitch = atan2(-x,(y*sinf(roll)+z*cosf(roll)));

    }

  // heading: Rotation around the Z-axis. -180 <= roll <= 180
  // a positive heading angle is defined to be a clockwise rotation about the positive Z-axis
  //
  //                                       z * sin(roll) - y * cos(roll)
  //   heading = atan2(--------------------------------------------------------------------------)
  //                    x * cos(pitch) + y * sin(pitch) * sin(roll) + z * sin(pitch) * cos(roll))
  //
  // where:  x, y, z are returned value from magnetometer sensor

    heading  = atan2f(hz*sinf(roll) - hy*cosf(roll), hx*cosf(pitch)+hy*sinf(pitch)*sinf(roll)+hz*sinf(pitch)*cos(roll));


    roll *= 180.0 / M_PI;
    pitch *= 180.0 / M_PI;
    heading *= 180.0 / M_PI;
}



//*****************************************************************************
//
//! \brief Float to ASCII
//!
//! Converts a floating point number to ASCII. Note that buf must be
//! large enough to hold
//!
//! \param f is the floating point number.
//! \param buf is the buffer in which the resulting string is placed.
//! \return None.
//!
//! \par Example:
//! ftoa(3.14) returns "3.14"
//!
//
//*****************************************************************************
void ftoa(float f,char * buf)
{
    int pos=0,ix,dp,num;
    if (f<0)
    {
        buf[pos++]='-';
        f = -f;
    }
    dp=0;
    while (f>=10.0)
    {
        f=f/10.0;
        dp++;
    }
    for (ix=1;ix<8;ix++)
    {
            num = (int)f;
            f=f-num;
            if (num>9)
                buf[pos++]='#';
            else
                buf[pos++]='0'+num;
            if (dp==0) buf[pos++]='.';
            f=f*10.0;
            dp--;
    }
}

void UARTstringf(char *car,uint8_t len){

	uint8_t inx;
	for(inx=0; inx<len ; inx++){
		ROM_UARTCharPut(UART0_BASE,car[inx]);
	}
}



void printValuesRAW(void){

	char buffer[7];


	readMag();
	readAccel();
	readGyro();
	readTemp();


	// print out accelleration data
	UARTprintf("Accel X: "); ftoa(calcAccel(ax),buffer);
	UARTstringf(buffer,7); UARTprintf(" ");

	UARTprintf("  \tY: "); ftoa(calcAccel(ay),buffer);
	UARTstringf(buffer,7); UARTprintf(" ");

	UARTprintf("  \tZ: "); ftoa(calcAccel(az),buffer);
	UARTstringf(buffer,7);
	UARTprintf("  \tg\n");

	// print out magnetometer data
	UARTprintf("Magn. X: "); ftoa(calcMag(mx),buffer);
	UARTstringf(buffer,7); UARTprintf(" ");

	UARTprintf("  \tY: "); ftoa(calcMag(my),buffer);
	UARTstringf(buffer,7); UARTprintf(" ");

	UARTprintf("  \tZ: "); ftoa(calcMag(mz),buffer);
	UARTstringf(buffer,7);  UARTprintf("  \tgauss\n");

	// print out gyroscopic data
	UARTprintf("Gyro  X: "); ftoa(calcGyro(gx),buffer);
	UARTstringf(buffer,7);  UARTprintf(" ");

	UARTprintf("  \tY: "); ftoa(calcGyro(gy),buffer);
	UARTstringf(buffer,7);  UARTprintf(" ");

	UARTprintf("  \tZ: "); ftoa(calcGyro(gz),buffer);
	UARTstringf(buffer,7); UARTprintf("  \tdps\n");

	// print out temperature data
	UARTprintf("Temp: ");
	UARTprintf("%d",temperature); UARTprintf(" *C\n");

	UARTprintf("**********************\n");

}

void printValuesCalib(void){
	char buffer[7];
	readMag();
	readAccel();
	readGyro();
	readTemp();

	axf = calcAccel(ax) - abias[0];
	ayf = calcAccel(ay) - abias[1];
	azf = calcAccel(az) - abias[2];

	gxf = calcGyro(gx) - gbias[0];
	gyf = calcGyro(gy) - gbias[1];
	gzf = calcGyro(gz) - gbias[2];


	// print out accelleration data
	UARTprintf("Accel X: "); ftoa(axf,buffer);
	UARTstringf(buffer,7); UARTprintf(" ");

	UARTprintf("  \tY: "); ftoa(ayf,buffer);
	UARTstringf(buffer,7); UARTprintf(" ");

	UARTprintf("  \tZ: "); ftoa(azf,buffer);
	UARTstringf(buffer,7);
	UARTprintf("  \tg\n");

	// print out magnetometer data
	UARTprintf("Magn. X: "); ftoa(calcMag(mx),buffer);
	UARTstringf(buffer,7); UARTprintf(" ");

	UARTprintf("  \tY: "); ftoa(calcMag(my),buffer);
	UARTstringf(buffer,7); UARTprintf(" ");

	UARTprintf("  \tZ: "); ftoa(calcMag(mz),buffer);
	UARTstringf(buffer,7);  UARTprintf("  \tgauss\n");

	// print out gyroscopic data
	UARTprintf("Gyro  X: "); ftoa(gxf,buffer);
	UARTstringf(buffer,7);  UARTprintf(" ");

	UARTprintf("  \tY: "); ftoa(gyf,buffer);
	UARTstringf(buffer,7);  UARTprintf(" ");

	UARTprintf("  \tZ: "); ftoa(gzf,buffer);
	UARTstringf(buffer,7); UARTprintf("  \tdps\n");

	// print out temperature data
	UARTprintf("Temp: ");
	UARTprintf("%d",temperature); UARTprintf(" *C\n");

	UARTprintf("**********************\n");


}

void printTestAdafruitAHRS(void){
		char buffer[7];
		readMag();
		readAccel();
		getOrientation(calcMag(mx),calcMag(my),calcMag(mz),calcAccel(ax),calcAccel(ay),calcAccel(az));

	    /* 'orientation' should have valid .roll and .pitch fields */
	    UARTprintf("Orientation: ");ftoa(roll,buffer);
	    UARTstringf(buffer,7);// print roll
	    UARTprintf(" ");
	    ftoa(pitch,buffer);
	    UARTstringf(buffer,7);// print pitch
	    UARTprintf(" ");
	    ftoa(heading,buffer);
	    UARTstringf(buffer,7);// print heading
	    UARTprintf(" \n");
}
