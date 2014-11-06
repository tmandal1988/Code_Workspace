/*
 * Accessory_Functions.cpp
 *
 *  Created on: Oct 28, 2014
 *      Author: Tanmay
 */
#include "Accessory_Functions.h"
#include "SimpleAD.h"
#include <stdio.h>
#include "FileSystemUtils.h"
#include "Card_Routines.h"
#include <ucos.h>
#include <pitr_sem.h>//for PIT SEM
#include <effs_fat/fat.h>
#include <effs_fat/multi_drive_mmc_mcf.h>
#include <effs_fat/effs_utils.h>

void Init_100Hz_Control(OS_SEM PitSem1) {
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 10ms second intervals
	InitPitOSSem(1, &PitSem1, 20);
}

int Init_Logfile(F_FILE** fp) {
	f_enterFS();
	char File_name[6];
	int drv = OpenOnBoardFlash();
	int* card_status = initOnBoardSD(drv);
	sprintf(File_name, "LOG%d.txt", card_status[1] + 1);
	*fp = f_open(File_name, "w+");
	return drv;
}

double* Get_Adc_Values() {
	static double ADC_Values[8]={0};
	StartAD();
	while (!ADDone())

		asm("nop");
	for (int i = 0; i < 4; i++)
		ADC_Values[i] = ((double) (GetADResult(i))) * 3.3 / 32768.0;

	return ADC_Values;


}

