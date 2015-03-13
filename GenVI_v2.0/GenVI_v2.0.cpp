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
#include "OSD_functions.h" //Onscreen Display functions
#include "SimpleAD.h"
#include "lookup_sin_cos.h"



extern "C" {
void UserMain(void * pd);
//void SetIntc(int intc, long func, int source, int level);
}

const char *AppName = "GenVI Avionics Netburner Firmware v2.2";

static uint16_t pilot_input[4] = { 0 }; //0.Throttle, 1.Aileron, 2.Elevator,3.Rudder
static uint16_t controlSwitch=0;
static char GPS_packet[144] = { 0 };
static uint8_t GPSavailFlag = 0;
static uint8_t recavailFlag=0;
static int fdUart8;
double elevDoublet=0;


/**********************Task for reading GPS data***********************************/
void Read_GPS_Data(void*) { //Reads Data from GPS

	fdUart8 = SerialRoutine(8);
	uint8_t header[] = { 0xAA, 0x44, 0x12 };

	while (1) {

		Read_Serial_Data(fdUart8, header, sizeof(header), GPS_packet, 144);
		GPSavailFlag = 1;

	}
}

/**********************Task for reading receiver data******************************/
void Read_Spektrum_Receiver(void *) { //Reads Data from Spektrum Receiver

	int fdSpektrum = SerialRoutine(2);
	char spektrum_packet_raw[30] = { 0 };

	while (1) {
		readSatRec(fdSpektrum, spektrum_packet_raw);
		pilot_input[0] = (uint16_t)((spektrum_packet_raw[4] & 0x07) * 256)
				+ (uint8_t)(spektrum_packet_raw[5]);

		pilot_input[3] = (uint16_t)((spektrum_packet_raw[8] & 0x07) * 256)
				+ (uint8_t)(spektrum_packet_raw[9]);


		if (spektrum_packet_raw[20] != 0) {
			pilot_input[1] = (uint16_t)((spektrum_packet_raw[20] & 0x07) * 256)
					+ (uint8_t)(spektrum_packet_raw[21]);//Aileron
			pilot_input[2] = (uint16_t)((spektrum_packet_raw[24] & 0x07) * 256)
					+ (uint8_t)(spektrum_packet_raw[25]);//Elev

			controlSwitch=(uint16_t)((spektrum_packet_raw[26] & 0x07) * 256)
									+ (uint8_t)(spektrum_packet_raw[27]);
		} else {
			pilot_input[1] = (uint16_t)((spektrum_packet_raw[21] & 0x07) * 256)
					+ (uint8_t)(spektrum_packet_raw[22]);
			pilot_input[2] = (uint16_t)((spektrum_packet_raw[25] & 0x07) * 256)
					+ (uint8_t)(spektrum_packet_raw[26]);

			controlSwitch=(uint16_t)((spektrum_packet_raw[27] & 0x07) * 256)
												+ (uint8_t)(spektrum_packet_raw[28]);
		}

		recavailFlag=1;



	} //task while
} //task

void IMU_Filter_Loop(void *) { //runs Filter loop and saves data to the SD-Card



	////Initialize 100Hz Semaphore Timer to time the filter loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 10ms second intervals
	InitPitOSSem(1, &PitSem1, 100);

	configIMU();
	J2[32] = 1; //Resets the OSD
	Enable_OSD();//Enables the MAX7456 to display video
	OSTimeDly(2);
	initOSD();//Initializes the OSD to proper black value and sets DMM to 0;

	InitSingleEndAD();
	float adcVal[8] = {0};

	uint16_t flushCount=0;



	//IMU variables
	BYTE IMU_command[14] = { xahigh, 00, yahigh, 00, zahigh, 00, xghigh, 00,
			yghigh, 00, yghigh, 00, zghigh, 00 };
	double IMU_data[7] = { 0, 0, 0, 0, 0, 0 }; //0-Gyro Z,1-Accel X,2-Accel Y,3-Accel Z,4 Gyro X,5 Gyro Y;
	uint8_t IMU_raw[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	uint16_t i = 0; //index

	//Angle variables
	angle complimentary_filter = { 0, 0 };
	angle tilt_angle = { 0, 0 };

	//File Routines
	f_enterFS();
	FileRoutines OpenOnboardSD;
	OpenFileRoutine(&OpenOnboardSD);

	//Buffer to be saved on the SD card
	char SD_card[140] = { 0 };
	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;

	OSTimeDly(TICKS_PER_SECOND * 5);

	for (i = 0; i < 5000; i++) { //Initializing the angle values

		GetIMUdata(IMU_command, 14, IMU_data, IMU_raw);
		//Getting angle from accelerometer
		tilt_angle.roll = (GetTiltAngle_Roll(IMU_data) + i * tilt_angle.roll)
				/ (i + 1);
		tilt_angle.pitch = (GetTiltAngle_Pitch(IMU_data) + i * tilt_angle.pitch)
				/ (i + 1);
	}

	complimentary_filter.roll = tilt_angle.roll * rad2deg;
	complimentary_filter.pitch = tilt_angle.pitch * rad2deg;

	//Initialize the timer to measure time difference between adjacent data points
	HiResTimer* timer = InitTimer(0);
	GetDeltaT(timer);

	//PWM variables
	uint16_t pwm_maxlim;
	uint16_t pwm_freq = 200;


	pwm_maxlim = configPWM(10, pwm_freq);
	pwm_maxlim = configPWM(20, pwm_freq);

	pwm_maxlim = configPWM(11, pwm_freq);
	pwm_maxlim = configPWM(21, pwm_freq);

	double alignVal = 0.0013;
	setPWM(10, alignVal * pwm_freq * pwm_maxlim);
	setPWM(20, alignVal * pwm_freq * pwm_maxlim);

	setPWM(11, alignVal * pwm_freq * pwm_maxlim);
	setPWM(21, alignVal * pwm_freq * pwm_maxlim);

	uint8_t controlSwitchFlag=0;
	uint32_t controlSwitchTime=0;
	double elevDeflection=0;
	//double multisinePWM=0;




	iprintf("\n%s Application started\r\n", AppName);


	while (1) {

		BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);
		J2[48] = 0;
		//uint16_t pwmr = sim1.mcpwm.mcr;
		//sim1.mcpwm.mcr |= LDOK;
		if (status == OS_NO_ERR) {


			GetIMUdata(IMU_command, 14, IMU_data, IMU_raw);
			GetAttitude(IMU_data, &complimentary_filter);

			updateOSD(complimentary_filter.roll, complimentary_filter.pitch);

			//Assign Data to SD card Array
			AssignIMUtoSD(SD_card, IMU_raw);

			AssignAttitudetoSD(complimentary_filter, SD_card);

			if(GPSavailFlag==1){
				unsigned long GPScrc=CalculateBlockCRC32(sizeof(GPS_packet)-4,GPS_packet);
				if((unsigned char)(GPScrc & 0x000000FF)==(unsigned char)GPS_packet[140] && (unsigned char)((GPScrc & 0x0000FF00)>>8)==(unsigned char)GPS_packet[141] && (unsigned char)((GPScrc & 0x00FF0000)>>16)==(unsigned char)GPS_packet[142] && (unsigned char)((GPScrc & 0xFF000000)>>24)==(unsigned char)GPS_packet[143]){
					AssignGPSxtoSD(GPS_packet, SD_card);
					//iprintf("%x %02x %02x %02x %02x\n",GPScrc,(unsigned char)GPS_packet[140],(unsigned char)GPS_packet[141],(unsigned char)GPS_packet[142],(unsigned char)GPS_packet[143]);
				}
				GPSavailFlag=0;
			}

			if(recavailFlag==1){

				AssignPilot_toSD(SD_card, pilot_input,controlSwitch);

				if(controlSwitch>1000){
					J1[13]=0;//Autopilot off
					controlSwitchFlag=0;
					controlSwitchTime=0;

					//printf(" %ld %g\n",controlSwitchTime,(pilot_input[2] * 0.00057675 + 0.91439) * 0.001);

				}
				if(controlSwitch<=1000){

					if(controlSwitchFlag==0){
						controlSwitchFlag=1;

					}

					J1[13]=1;//Autopilot on

					if(controlSwitchTime<25)
						elevDeflection=15;
					else if(controlSwitchTime>=25 && controlSwitchTime<50)
						elevDeflection=-15;
					else
						elevDeflection=0;

					//elevDeflection=0.3*lookup_sin(10*controlSwitchTime*0.01)*rad2deg;// 0.0524*lookup_cos(2*PI*0.5*multisineTime  -1.121) - 0.0698*lookup_cos(2*PI*0.875*multisineTime - 1.242) +
									// 0.0611*lookup_cos(2*PI*1.250*multisineTime + 2.604) - 0.0611*(2*PI*1.625*multisineTime + 0.524) -
									// 0.0349*lookup_cos(2*PI*2*multisineTime + 2.442)*rad2deg;
					controlSwitchTime++;
					elevDoublet=0.000012915*elevDeflection + 0.0014448 + (pilot_input[2] * 0.00057675 + 0.91439) * 0.001-0.00145;


					//printf("%d %g\n",controlSwitch,elevDoublet);





				}



			/*	setPWM(10,(pilot_input[0] * 0.00057675 + 0.91439) * 0.001 * pwm_freq* pwm_maxlim);
				setPWM(20,(pilot_input[1] * 0.00057675 + 0.91439) * 0.001 * pwm_freq* pwm_maxlim);
				setPWM(11,(pilot_input[2] * 0.00057675 + 0.91439) * 0.001 * pwm_freq* pwm_maxlim);
				setPWM(21,(pilot_input[3] * 0.00057675 + 0.91439) * 0.001 * pwm_freq* pwm_maxlim);*/

				recavailFlag=0;
			}


			//if(controlSwitch<=1000)


			StartAD();
			while (!ADDone())

				asm("nop");
			adcVal[0] = ((double) (GetADResult(1))) * 3.3 / 32768.0;

			//printf("%g\n",adcVal[0]);

			AssignADCtoSD(adcVal,SD_card);

			AssignDeltaTandCounter(SD_card, timer);

			//for (i = 0; i < sizeof(SD_card); i++) {
				//write(fdUart8, &SD_card[i], 1);
			//}

			if(OpenOnboardSD.fp)
				f_write(&SD_card,1,sizeof(SD_card),OpenOnboardSD.fp);

			flushCount=flushCount+1;

			if(flushCount==3999){
				f_flush(OpenOnboardSD.fp);
				flushCount=0;
			}

			J2[48] = 1;
		} //If for PIT Sem

		setPWM(11,elevDoublet*pwm_freq*pwm_maxlim);
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
	OSSimpleTaskCreate(IMU_Filter_Loop, MAIN_PRIO + 3); //Creating IMU Filter task
	OSSimpleTaskCreate(Read_Spektrum_Receiver, MAIN_PRIO + 2); //Creating Receiver Reading Task
	OSSimpleTaskCreate(Read_GPS_Data, MAIN_PRIO + 1); //Creating GPS reading Task

	while (J1[7]) {
		OSTimeDly(TICKS_PER_SECOND * 120);
	} //Main While
} //Main Task

