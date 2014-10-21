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

int initPINS()
{

	SerialClose(1);
	int fdDebug=OpenSerial( 1 , 115200, 1, 8, eParityNone );
	return fdDebug;

}


