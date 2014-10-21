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

	J2[31].function(3);//UART2 RX
	//J2[22].function(3);//UART1 TX

	SerialClose(0);
	fdDebug=OpenSerial(0,115200,1,8,eParityNone);


}


