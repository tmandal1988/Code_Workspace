/*
 * Blue_Vanes_Calibration.cpp
 *
 *  Created on: Oct 28, 2014
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
//#include "PINS_Definations.h"
#include "FileSystemUtils.h"
#include "Card_Routines.h"
#include <pitr_sem.h>//for PIT SEM
#include "SimpleAD.h"

#include "Accessory_Functions.h"

extern "C" {
void UserMain(void * pd);
}

F_FILE* fp;

void Read_ADC(void *) {

	//OS_SEM PitSem1;
	//Init_100Hz_Control(PitSem1);

	InitSingleEndAD();
	//int drv=Init_Logfile(&fp);

	char SD_card[30] = { 0 };
	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;

	double Vane_Values[8] = { 0 };
	uint16_t Vane_Values_Int[3] = { 0 };

	while (1) {
		//printf("Hi\n");
		//BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);
		//if (status == OS_NO_ERR) {

		StartAD();
		while (!ADDone())

		asm("nop");
		for (int i = 0; i < 4; i++)
			Vane_Values[i] = ((double) (GetADResult(i))) * 3.3 / 32768.0;



		printf("Vane1=%f, Vane2=%f, Vane3=%f\n", ((double) (GetADResult(1))) * 3.3 / 32768.0, ((double) (GetADResult(2))) * 3.3 / 32768.0,
				((double) (GetADResult(3))) * 3.3 / 32768.0);

		Vane_Values_Int[0] = Vane_Values[1] * 15000;
		Vane_Values_Int[1] = Vane_Values[2] * 15000;
		Vane_Values_Int[2] = Vane_Values[3] * 15000;

		//printf("Vane1=%0.2f, Vane2=%0.2f, Vane3=%0.2f\n",Vane_Values[1],Vane_Values[2],Vane_Values[3]);

		SD_card[3] = (Vane_Values_Int[0] & 0xFF00) >> 8;
		SD_card[4] = (Vane_Values_Int[0] & 0x00FF);

		SD_card[5] = (Vane_Values_Int[1] & 0xFF00) >> 8;
		SD_card[6] = (Vane_Values_Int[1] & 0x00FF);

		SD_card[7] = (Vane_Values_Int[2] & 0xFF00) >> 8;
		SD_card[8] = (Vane_Values_Int[2] & 0x00FF);

		SD_card[29] = SD_card[29] + 1;

		//if (fp) {
		//f_write(&SD_card, 1, 30, fp);
		//}

		//} //PIT SEM Status if

	} //Task While

	//f_close(fp);
	//UnmountFlash(drv);
	//f_releaseFS();
} //Read_ADC Task

/*****************************UserMain***************************************************/
void UserMain(void* pd) {
	/////Usual Routine
	InitializeStack();
	OSChangePrio (MAIN_PRIO);
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();
	OSSimpleTaskCreate(Read_ADC, MAIN_PRIO + 1);

	OSTimeDly(100);
	//f_enterFS();

	while (1) {
		OSTimeDly(TICKS_PER_SECOND * 30);

		//f_flush(fp);

	}
	//f_releaseFS();

}
