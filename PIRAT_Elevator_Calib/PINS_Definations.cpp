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

	J2[48].function(0);//led D2,GPIO
	J2[48]=0;

/*
	J1[7].function(0);//User defined Switch

	J2[21].function(1);//SPI3 Input
	J2[22].function(1);//SPI3 Out
	J2[23].function(1);//SPI3 chip select 0
	J2[24].function(1);//SPI3 clock
*/

	J2[39].function(2);//UART8 RX
	J2[42].function(2);//UART8 TX

	J2[31].function(3);//UART2 RX

/*	J2[43].function(3);//External interrupt 2 reading PPM
	sim2.eport.eppar |= 0x0020;*/

	J2[25].function(1);//SPI1 SCK
	J2[27].function(1);//SPI1 DIN
	J2[28].function(1);//SPI1 DOUT
	J2[30].function(1);// SPI1 CS0

/*	J2[32].function(0);//OSD Reset

	J2[32]=0;*/

}


