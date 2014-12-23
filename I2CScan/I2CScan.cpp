/******************************************************************************
 * Copyright 2014-2015 NetBurner, Inc.  ALL RIGHTS RESERVED
 *   Permission is hereby granted to purchasers of NetBurner Hardware
 *   to use or modify this computer program for any use as long as the
 *   resultant program is only executed on NetBurner provided hardware.
 *
 *   No other rights to use this program or it's derivitives in part or
 *   in whole are granted.
 *
 *   It may be possible to license this or other NetBurner software for
 *        use on non NetBurner Hardware.
 *   Please contact Licensing@Netburner.com for more infomation.
 *
 *   NetBurner makes no representation or warranties
 *   with respect to the performance of this computer program, and
 *   specifically disclaims any responsibility for any damages,
 *   special or consequential, connected with the use of this program.
 *
 *   NetBurner, Inc.
 *   5405 Morehouse Dr
 *   San Diego Ca, 92121
 *   www.netburnre.com
 *****************************************************************************/
#include "predef.h"
#include <stdio.h>
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <smarttrap.h>
#include <taskmon.h>
#include <i2cmaster.h>
//#include <i2cmulti.h>
#include "pins.h"

extern "C" {
void UserMain(void * pd);
}


const char * AppName = "I2C Scan";

void UserMain(void * pd)
{
	InitializeStack(); /* Setup the TCP/IP stack buffers */
	GetDHCPAddressIfNecessary(); /* Get a DHCP address if needed */
	/*You may want to add a check for the return value from this function*/
	/*See the function definition in  \nburn\include\dhcpclient.h*/

	OSChangePrio(MAIN_PRIO); /* Change our priority from highest to something in the middle */
	EnableAutoUpdate(); /* Enable the ability to update code over the network */
	EnableSmartTraps(); /* Enable the smart reporting of traps and faults */
	EnableTaskMonitor(); /*Enable the Task scan utility */

	/* The I2C bus speed on the 5270 processor is set by a divider of the internal
	 * clock frequency of 147.5MHz / 2 = 73.75MHz. The maximum I2C bus speed is
	 * 100KHz. 73.75MHz/100KHz = 737.5. Referring to the I2C freq divider table
	 * in the Freescale manual the closest divider is 768 (register value = 0x39).
	 * 73.75MHz/768 = 95,703 bps.
	 */
	   J2[39].function( PINJ2_39_I2C0_SDA);  // Set Pins to I2C
	   J2[42].function( PINJ2_42_I2C0_SCL);

	//I2CInit(0x68,0x3C);  // I2C multi parameters are ( slave id, freq divider )
	I2CInit(0x3C); 			// Approx 100Khz.  I2C master parameters are ( freq divider )

	iprintf("Scanning I2C \r\n");

	for (uint8_t x = 0; x < 0x82; x++)
	{
		int result = I2CStart(x, I2C_START_WRITE, 1);
		if (result < I2C_TIMEOUT)
		{
			iprintf("Found device at  0x%X, Result: %d\r\n", x, result);
			I2CStop();
		}
		else
		{
			I2CStop();
			//I2CResetPeripheral();
		}
	}

	iprintf("Scan complete\r\n");

	while (1)
	{
		OSTimeDly(TICKS_PER_SECOND * 1);
	}
}
