/*
 * Serial_Test_ROS.cpp
 *
 *  Created on: Oct 9, 2014
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
#include <pitr_sem.h>//for PIT SEM


extern "C"{
	void UserMain( void * pd);
	void SetIntc( int intc, long func, int source, int level);
}


/*****************************UserMain***************************************************/
void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	int fdDebug=initPINS();

	OS_SEM PitSem1;//Time Sem
    OSSemInit( &PitSem1, 0 );
    // Init for timer 1, at 20ms second intervals
    InitPitOSSem( 1, &PitSem1, 50 );

    char send_buff[15]={0};
    send_buff[0]=0xAA;
    send_buff[1]=0xAB;
    send_buff[2]=0xBB;

	while(1){
		BYTE status = OSSemPend( &PitSem1, TICKS_PER_SECOND * 5 );
		if ( status == OS_NO_ERR ){
			send_buff[14]=send_buff[14]+1;
			for (int i=0;i<15;i++)
				write(fdDebug,&send_buff[i],1);
				//iprintf("Hi")
		}
	}

}
