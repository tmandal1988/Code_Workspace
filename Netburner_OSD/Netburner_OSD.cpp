/*
 * Netburner_OSD.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: Tanmay
 */


///included
#include "predef.h"
#include <stdio.h>
#include <stdlib.h>
#include <basictypes.h>
#include <ucos.h>
#include <ctype.h>
#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <taskmon.h>
#include <smarttrap.h>
#include <effs_fat/fat.h>
#include <effs_fat/multi_drive_mmc_mcf.h>
#include <effs_fat/effs_utils.h>
#include <sim.h>
#include <pins.h>
#include <ucosmcfc.h>
#include <pinconstant.h>
#include <HiResTimer.h>
#include <utils.h>
#include <constants.h>
#include <cfinter.h>
#include <math.h>
#include <serial.h>
#include <dspi.h> //needed for IMU communication
#include "PINS_Definations.h"
#include "OSD_Functions.h"

extern "C"{
	void UserMain( void * pd);
}

const char *AppName = "Netburner OSD v4";

/*****************************UserMain***************************************************/
void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	//BYTE OSD_DIN[8]={0};
	//BYTE OSD_DOUT[2]={0};
	//BYTE OSD_Disable[2]={0x00,0x00};

	iprintf( "%s application started\r\n", AppName );

	initPINS();
	OSTimeDly(5);
	DSPIInit(1,2000000,16,0x00,0x01,0,0,0,0,0);//Initializing the Hardware to talk to OSD

	Enable_OSD();
	OSTimeDly(5);
	initOSD();

	/*******************replacing 0x0C charcter************************/
	//ReplaceCharacter(0xC0);//Center Circle
	//ReplaceCharacter(0xC1);//Vertical Line
	//ReplaceCharacter(0xC2);//Horizontal Line

	/***********************************Initializing Initial Artificial Horizon*****************************************************************/

	uint16_t x=194;
	//OSD_Position_H(0x00);

	Display_Center(x);
	Display_Center_Line(x);
	Display_Top_Line(x);

	//OSD_Position_H(0x01);
	Display_Bottom_Line(x);

	OSTimeDly(2);

	while(1){

		for (int i=-6;i<7;i++){

			Replace_Center_Line(i);
			Replace_Top_Line(i);
			Replace_Bottom_Line(i);
		}
	}

}
