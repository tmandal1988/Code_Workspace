/*
 * GenVI_v2.0.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Tanmay
 */


/***Note: This code is derived from GenVI_Netburner_Advanced except for the GPS reading part**********/

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

const char *AppName = "GenVI Avionics Netburner Firmware v2";


static uint16_t pilot_input[4]={0};//0.Throttle, 1.Aileron, 2.Elevator,3.Rudder
static uint8_t GPS_ms[6] = { 0 }; //Array to store GPS time (ms from the beginning of the GPS week)


/**********************Task for reading GPS data***********************************/
void Read_GPS_Data(void*) { //Reads Data from GPS
	int fdGPS = SerialRoutine(9);
	char GPS_packet[100] = { 0 };

	uint8_t header[] = { 0xAA, 0x44, 0x12 };

	while (1) {
		Read_Serial_Data(fdGPS, header, sizeof(header), GPS_packet, 28);
		GPS_ms[0] = GPS_packet[14];
		GPS_ms[1] = GPS_packet[15];
		GPS_ms[2] = GPS_packet[16];
		GPS_ms[3] = GPS_packet[17];
		GPS_ms[4] = GPS_packet[18];
		GPS_ms[5] = GPS_packet[19];

	}
}

/**********************Task for reading receiver data******************************/
void Read_Spektrum_Receiver(void *) { //Reads Data from Spektrum Receiver

	int fdSpektrum = SerialRoutine(2);
	char spektrum_packet_raw[30] = { 0 };

	while (1) {

		readSatRec(fdSpektrum,spektrum_packet_raw);
		pilot_input[0] = (uint16_t)((spektrum_packet_raw[4] & 0x07) * 256) + (uint8_t)(spektrum_packet_raw[5]);

		pilot_input[3] = (uint16_t)((spektrum_packet_raw[8] & 0x07) * 256) + (uint8_t)(spektrum_packet_raw[9]);

		if (spektrum_packet_raw[20] != 0){
			pilot_input[1] = (uint16_t)((spektrum_packet_raw[20] & 0x07) * 256) + (uint8_t)(spektrum_packet_raw[21]);
			pilot_input[2] = (uint16_t)((spektrum_packet_raw[24] & 0x07) * 256) + (uint8_t)(spektrum_packet_raw[25]);
		}
		else{
			pilot_input[1] = (uint16_t)((spektrum_packet_raw[21] & 0x07) * 256) + (uint8_t)(spektrum_packet_raw[22]);
			pilot_input[2] = (uint16_t)((spektrum_packet_raw[25] & 0x07) * 256) + (uint8_t)(spektrum_packet_raw[26]);
		}

		} //task while
} //task

void IMU_Filter_Loop(void *) { //runs Filter loop and saves data to the SD-Card

	iprintf("\n%s Application started\r\n", AppName);

	////Initialize 100Hz Semaphore Timer to time the filter loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 10ms second intervals
	InitPitOSSem(1, &PitSem1, 100);

	//IMU variables
	BYTE IMU_command[14] = { xahigh, 00, yahigh, 00, zahigh, 00, xghigh, 00,yghigh, 00, yghigh, 00, zghigh, 00 };
	double IMU_data[7] = { 0, 0, 0, 0, 0, 0 }; //0-Gyro Z,1-Accel X,2-Accel Y,3-Accel Z,4 Gyro X,5 Gyro Y;
	uint8_t IMU_raw[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ,0,0 };

	uint16_t i = 0;//index

	//Angle variables
	angle complimentary_filter={0,0};
	angle tilt_angle={0,0};

	//File Routines
	f_enterFS();
	FileRoutines OpenOnboardSD;
	OpenFileRoutine(&OpenOnboardSD);

	//Buffer to be saved on the SD card
	char SD_card[36] = { 0 };
	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;
	uint16_t flush_count=0;

	OSTimeDly(TICKS_PER_SECOND*5);

	for (i = 0; i < 5000; i++) { //Initializing the angle values

		GetIMUdata(IMU_command,14,IMU_data,IMU_raw);
		//Getting angle from accelerometer
		tilt_angle.roll =( GetTiltAngle_Roll(IMU_data) + i * tilt_angle.roll) / (i + 1);
		tilt_angle.pitch =( GetTiltAngle_Pitch(IMU_data)+ i * tilt_angle.pitch) / (i + 1);
	}

	complimentary_filter.roll = tilt_angle.roll * rad2deg;
	complimentary_filter.pitch = tilt_angle.pitch * rad2deg;

	//printf("Roll=%0.2f,Pitch=%0.2f\n",complimentary_filter.roll,complimentary_filter.pitch);

	int fdlogger=SerialRoutine(8);

	//Initialize the timer to measure time difference between adjacent data points
	HiResTimer* timer=InitTimer(0);
	GetDeltaT(timer);


	while (J1[7]) {

		BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);
		J2[48]=0;
		if (status == OS_NO_ERR) {

			GetIMUdata(IMU_command,14,IMU_data,IMU_raw);
			GetAttitude(IMU_data,&complimentary_filter);

			//Assign Data to SD card Array
			AssignIMUtoSD(SD_card,IMU_raw);
			AssignPilot_toSD(SD_card,pilot_input);
			AssignAttitudetoSD(complimentary_filter,SD_card);

			SD_card[27]=GPS_ms[0];
			SD_card[28]=GPS_ms[1];
			SD_card[29]=GPS_ms[2];
			SD_card[30]=GPS_ms[3];
			SD_card[31]=GPS_ms[4];
			SD_card[32]=GPS_ms[5];


			AssignDeltaTandCounter(SD_card,timer);


			if (OpenOnboardSD.fp) {
				f_write(&SD_card, 1, 36, OpenOnboardSD.fp);
			}

			i = 0;

			for (i = 1; i < 36; i++) {
				write(fdlogger, &SD_card[i], 1);
			}


			flush_count++;

			if(flush_count==2999){
				f_flush(OpenOnboardSD.fp);
				flush_count=0;
			}

			J2[48] =1;
		} //If for PIT Sem
	} //Task While Loop
	J2[48] = 1;
	f_close(OpenOnboardSD.fp);
	UnmountFlash(OpenOnboardSD.drv);
	f_releaseFS();

} //IMU_Filter_Loop Task

/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	Usual_Routine();

	//Task Routines
	OSSimpleTaskCreate(IMU_Filter_Loop,MAIN_PRIO+3);//Creating IMU Filter task
	OSSimpleTaskCreate(Read_Spektrum_Receiver, MAIN_PRIO + 2); //Creating Receiver Reading Task
	OSSimpleTaskCreate(Read_GPS_Data, MAIN_PRIO + 1); //Creating GPS reading Task

	while (J1[7]) {
		OSTimeDly(TICKS_PER_SECOND * 120);
	}//Main While
}//Main Task




