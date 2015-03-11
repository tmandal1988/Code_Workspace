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

	J2[39].function(2);//UART8 RX
	J2[42].function(2);//UART8 TX


	J2[21].function(1);//SPI3 Input
	J2[22].function(1);//SPI3 Out
	J2[23].function(1);//SPI3 chip select 0
	J2[24].function(1);//SPI3 clock





	J2[41].function(2);//UART9 RX
	J2[44].function(2);//UART9 TX



	J2[31].function(3);//UART2 RX



}
