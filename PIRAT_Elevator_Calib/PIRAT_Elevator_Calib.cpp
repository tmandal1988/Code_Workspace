/*
 * PIRAT_Elevator_Calib.cpp
 *
 *  Created on: Dec 11, 2014
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
#include "Accessory_Functions.h" //Housekeeping functions
extern "C" {
void UserMain(void * pd);
//void SetIntc(int intc, long func, int source, int level);
}

const char *AppName = "PIRAT Elevator Calibration Code v1";

void Read_Elev_Angle(void *) { //Reads Elevator Tilt Angle

	iprintf("\n%s Application started\r\n", AppName);

	////Initialize 100Hz Semaphore Timer to time the filter loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 10ms second intervals
	InitPitOSSem(1, &PitSem1, 50);

	//OSTimeDly(TICKS_PER_SECOND * 5);
	BYTE PWR_CTL[] = { 0x2D, 0x08};
	BYTE DATA_FORMAT[]={0x31, 0x0A};

	//BYTE IMU_CMD[] = { 0x36,0x37,0x36};
	//BYTE IMU_RAW[sizeof(IMU_CMD)] = { 0 };
	//short int IMU_data[1] = { 0 };
	BYTE IMU_CMD1[] = {0x32|0x80|0x40,0x33|0x80|0x40,0x34|0x80|0x40,0x35|0x80|0x40,0x36|0x80|0x40,0x37|0x80|0x40};
	BYTE IMU_RAW1[sizeof(IMU_CMD1)]={0};

	BYTE IMU_CMD2[] = {0x34|0x80|0x40,0x35|0x80|0x40,0x36|0x80|0x40,0x37|0x80|0x40,0x32|0x80|0x40,0x33|0x80|0x40};;
	BYTE IMU_RAW2[sizeof(IMU_CMD2)]={0};

	short int IMU_Data[3]={0};

	DSPIStart(1, DATA_FORMAT, NULL, sizeof(DATA_FORMAT), NULL); //Configure the sensor
	DSPIStart(1, PWR_CTL, NULL, sizeof(PWR_CTL), NULL); //Configure the sensor


	while (1) {
		J2[48] = 0;
		BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);

		if (status == OS_NO_ERR) {
			J2[48] = 1;
			DSPIStart(1, IMU_CMD1, IMU_RAW1,sizeof(IMU_CMD1),NULL);
			while (!DSPIdone(1)) {
			}; //wait for DSPI to finish

			DSPIStart(1, IMU_CMD2, IMU_RAW2,sizeof(IMU_CMD2),NULL);
			while (!DSPIdone(1)) {
			}; //wait for DSPI to finish

			IMU_Data[0]=((short int)IMU_RAW2[4]<<8)|(short int)IMU_RAW2[3];//Z axis
			IMU_Data[1]=((short int)IMU_RAW1[2]<<8)|(short int)IMU_RAW1[1];//X axis
			IMU_Data[2]=((short int)IMU_RAW1[4]<<8)|(short int)IMU_RAW1[3];//Y axis

			printf("Az=%0.2f, Ax=%0.2f, Az=%0.2f\n",(IMU_Data[0]*0.004-1),IMU_Data[1]*0.004,IMU_Data[2]*0.004);

		} //SEM if
	} //Task While
} //Task

/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	Usual_Routine();

	OSSimpleTaskCreate(Read_Elev_Angle, MAIN_PRIO + 1); //Creating Receiver Reading Task
	while (1) {
		OSTimeDly(TICKS_PER_SECOND * 30);
	} //Main While
} //UserMain
