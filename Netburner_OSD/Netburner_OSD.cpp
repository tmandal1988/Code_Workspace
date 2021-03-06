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
#include "ReplaceCharacter.h"

extern "C"{
	void UserMain( void * pd);
}

const char *AppName = "Netburner OSD v6";

/*****************************UserMain***************************************************/
void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	iprintf( "%s application started\r\n", AppName );

	initPINS();//Initialize Hardware pins
	J2[32]=1;//Resets the OSD

	OSTimeDly(5);
	DSPIInit(1,2000000,16,0x00,0x01,0,0,0,0,0);//Initializing the Hardware to talk to OSD

	Enable_OSD();//Enables the MAX7456 to display video
	OSTimeDly(5);
	initOSD();//Initializes the OSD to proper black value and sets DMM to 0;

	/*******************replacing character************************/
	/**In this part call the function ReplaceCharacter with the character memory address to replace it with custom character
	 * You also have to go to the ReplaceCharacter.cpp file and replace the 54 bytes with the custom values of the black, gray and transperancy
	 * of each pixel
	 */

	//ReplaceCharacter(0xC0);//Center Circle
	//ReplaceCharacter(0xC1);//Vertical Line
	//ReplaceCharacter(0xC2);//Horizontal Line
	//Replace_Character(0x91);
	//Replace_Character(0x50);

	/***********************************Initializing Initial Artificial Horizon*****************************************************************/

	uint16_t x=2834;//Center Position of the Center Circle
	Display_Center(x);//Displaying the Center, argument center address
	Display_Data();//Initializing Pitch and Roll value Display


	int j=0;//roll variable
	while(1){
		for (int i=-60;i<61;i=i+5){//pitch variable
			j=-60;
		while (j<61){
				Display_Roll(j);//Displays roll value, argument roll
				Display_Pitch(i);//Display pitch values, argument pitch
				Replace_Center_Line(i,j);//deletes old center line and redraws it based on new roll and pitch value, first argument pitch, second argument roll

				//printf("%d,%d\n",i,j);
				OSTimeDly(1);//Time delay to slow down the display to see what is happening on the screen
				j=j+1;
			}
		}
	}


}
