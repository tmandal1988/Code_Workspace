/*
 * PINS_Definations.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: Tanmay
 */

#include "PINS_Definations.h"
#include <basictypes.h>
#include <sim.h>
#include <pins.h>
#include <serial.h>

void initPINS()
{
	int fdDebug=0;

	J2[48].function(0);//led D2,GPIO
	J2[48]=0;

	J1[7].function(0);//User defined Switch

	J2[21].function(1);//SPI3 Input
	J2[22].function(1);//SPI3 Out
	J2[23].function(1);//SPI3 chip select 0
	J2[24].function(1);//SPI3 clock

	J2[39].function(2);//UART8 RX
	J2[42].function(2);//UART8 TX

	J2[43].function(3);//External interrupt 2 reading PPM
	sim2.eport.eppar |= 0x0020;

	SerialClose(0);
	fdDebug=OpenSerial(0,115200,1,8,eParityNone);


}


