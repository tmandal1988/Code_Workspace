/*
 * Read_Receiver_Serial.cpp
 *
 *  Created on: Oct 6, 2014
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
#include <pitr_sem.h>//for PIT SEM

extern "C"{
	void UserMain( void * pd);
	//void SetIntc( int intc, long func, int source, int level);
}

const char *AppName = "Read Blue Satellite Receiver v1";
F_FILE* fp;

void Read_Sat_Rec(void *){

	iprintf( "\n%s Application started running on Laptop\r\n", AppName );
	uint16_t throttle=0,aeliron=0,elevator=0,rudder=0;
	int fdReceiver=0;

	SerialClose(2);
	fdReceiver=OpenSerial(2,115200,1,8,eParityNone);

	char raw_data[25]={0};
	int i=0;

	f_enterFS();
	char File_name[6];
	int drv=OpenOnBoardFlash();
	int *card_status=initOnBoardSD(drv);
	sprintf(File_name,"RLOG%d.txt",card_status[1]+1);

	fp = f_open( File_name, "w+" );

	char SD_card[12]={0};
	SD_card[0]=0xAA;SD_card[1]=0xAB;SD_card[2]=0xBB;

	while(1){

		ReadWithTimeout(fdReceiver,&raw_data[0],1,1);
		if((unsigned char)raw_data[0]==0xFF){
			ReadWithTimeout(fdReceiver,&raw_data[1],1,1);
			if((unsigned char)raw_data[1]==0xFF){
				for(i=2;i<5;i++)
					ReadWithTimeout(fdReceiver,&raw_data[i],1,1);
				if((unsigned char)(raw_data[4] & 0x80) == 0x80){
					for(i=5;i<25;i++)
						ReadWithTimeout(fdReceiver,&raw_data[i],1,1);

					throttle= (uint16_t)((raw_data[4] & 0x07)*256)+(uint8_t)(raw_data[5]);
					elevator= (uint16_t)((raw_data[6] & 0x07)*256)+(uint8_t)(raw_data[7]);
					rudder= (uint16_t)((raw_data[8] & 0x07)*256)+(uint8_t)(raw_data[9]);

					if(raw_data[20]!=0)
						aeliron= (uint16_t)((raw_data[20] & 0x07)*256)+(uint8_t)(raw_data[21]);
					else
						aeliron= (uint16_t)((raw_data[21] & 0x07)*256)+(uint8_t)(raw_data[22]);

					SD_card[3]=(throttle & 0xFF00)>>8;
					SD_card[4]=(throttle & 0x00FF);

					SD_card[5]=(elevator & 0xFF00)>>8;
					SD_card[6]=(elevator & 0x00FF);

					SD_card[7]=(rudder & 0xFF00)>>8;
					SD_card[8]=(rudder & 0x00FF);

					SD_card[9]=(aeliron & 0xFF00)>>8;
					SD_card[10]=(aeliron & 0x00FF);

					SD_card[11]=SD_card[11]+1;

					if(fp)
					{
						f_write( &SD_card, 1, 12, fp );
						//iprintf("PIT Timer count: %d\n", n );
						//f_close(fp);
					}
				}
			}
		}
	}

	f_close(fp);
	UnmountFlash(drv);
	f_releaseFS();
}

/*****************************UserMain***************************************************/
void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	initPINS();//Initialize Hardware pins



	OSSimpleTaskCreate(Read_Sat_Rec,MAIN_PRIO+1);//Creating an Autopilot Task


	OSTimeDly(100);
	f_enterFS();

	while (J1[7])
	{
	   OSTimeDly( TICKS_PER_SECOND*30);
	   f_flush(fp);

	}

	f_releaseFS();
}

