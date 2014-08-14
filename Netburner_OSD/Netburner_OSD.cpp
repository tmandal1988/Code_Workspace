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
#include "ReplaceCharacter.h"

extern "C"{
	void UserMain( void * pd);
}

const char *AppName = "Netburner OSD v3";

/*****************************UserMain***************************************************/
void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	iprintf( "%s application started\r\n", AppName );
	initPINS();

	BYTE OSD_Enable[2]={0x00,0x08};
	BYTE OSD_DIN[8]={0};
	BYTE OSD_DOUT[2]={0};
	//BYTE OSD_Disable[2]={0x00,0x00};


	DSPIInit(1,2000000,16,0x00,0x01,0,0,0,0,0);//Initializing the Hardware to talk to OSD
	OSTimeDly(5);


	DSPIStart(1,OSD_Enable,NULL,2,NULL);//Enable display of OSD image
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSTimeDly(5);

	OSD_DOUT[0]=0xEC;OSD_DOUT[1]=0x00;
	DSPIStart(1,OSD_DOUT,OSD_DIN,2,NULL);//Get Image Black Level
	while(!DSPIdone(1)){};//wait for DSPI to finish

	//OSTimeDly(5);

	OSD_DOUT[0]=0x6C;OSD_DOUT[1] =OSD_DIN[1] & 0xEF;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Set OSD Black level to Image Black level
	while(!DSPIdone(1)){};//wait for DSPI to finish

	//OSTimeDly(5);

	OSD_DOUT[0]=0x04;OSD_DOUT[1] =0x00;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMM set to 0
	while(!DSPIdone(1)){};//wait for DSPI to finish

	/*******************replacing 0x0C charcter************************/
	//ReplaceCharacter(0xC0);//Center Circle
	//ReplaceCharacter(0xC1);//Vertical Line
	//ReplaceCharacter(0xC2);//Horizontal Line


	/*******************************************************************************************************/

	OSD_DOUT[0]=0x04;OSD_DOUT[1] =0x00;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMM set to 0
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x05;OSD_DOUT[1] =0x00;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAH
	while(!DSPIdone(1)){};//wait for DSPI to finish

	uint8_t x=194;

	OSD_DOUT[0]=0x06;OSD_DOUT[1] =x;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish


	OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC0;//o
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish*/


	/*OSD_DOUT[0]=0x06;OSD_DOUT[1] =x-30;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish


	OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC1;//o
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish*/

	OSD_DOUT[0]=0x06;OSD_DOUT[1] =x-1;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish


	OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC2;//o
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish*/

	OSD_DOUT[0]=0x06;OSD_DOUT[1] =x+1;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish


	OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC2;//o
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish*/

	DSPIStart(1,OSD_Enable,NULL,2,NULL);//Enable OSD display
	while(!DSPIdone(1)){};//wait for DSPI to finish
	OSTimeDly(2);

	while(1){
		for (int i=0;i<3;i++){
			x=194-i*30;


		OSD_DOUT[0]=0x06;OSD_DOUT[1] =x-3;
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish


		OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC2;//o
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish*/

		OSD_DOUT[0]=0x06;OSD_DOUT[1] =x-4;
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish


		OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC2;//o
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish*/

		OSD_DOUT[0]=0x06;OSD_DOUT[1] =x-5;
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish


		OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC2;//o
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish*/

		OSD_DOUT[0]=0x06;OSD_DOUT[1] =x+3;
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish


		OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC2;//o
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish*/

		OSD_DOUT[0]=0x06;OSD_DOUT[1] =x+4;
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish


		OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC2;//o
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish*/

		OSD_DOUT[0]=0x06;OSD_DOUT[1] =x+5;
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish


		OSD_DOUT[0]=0x07;OSD_DOUT[1] =0xC2;//o
		DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
		while(!DSPIdone(1)){};//wait for DSPI to finish*/
		}

	}
}
