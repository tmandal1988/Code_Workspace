/*
 * GenVI_Netburner.cpp
 *
 *  Created on: Sep 8, 2014
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
#include "FileSystemUtils.h"
#include "Card_Routines.h"


extern "C"{
	void UserMain( void * pd);
}

const char *AppName = "GenVI Avionics Netburner Code v1";


/*****************************UserMain***************************************************/
void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	iprintf( "%s Application started\r\n", AppName );

	initPINS();//Initialize Hardware pins

	f_enterFS();
	char File_name[6];
	int drv=OpenOnBoardFlash();
	int *card_status=initOnBoardSD(drv);
	sprintf(File_name,"LOG%d.txt",card_status[1]+1);
	//F_FILE* fp = f_open( File_name, "w+" );

	/*if ( fp )
	{
	BYTE  cp[5]={0x00,0x01,0x02,0x03,0x04};
    int n = f_write( &cp, 1, 5, fp );
    iprintf( "Wrote %d bytes: [%s]", n, cp);
	f_close(fp);
	}*/

	UnmountFlash(drv);

	iprintf( "Program complete. Reset to repeat\r\n");
	f_releaseFS();

	while ( 1 )
	{
	   OSTimeDly( TICKS_PER_SECOND );
	}




}
