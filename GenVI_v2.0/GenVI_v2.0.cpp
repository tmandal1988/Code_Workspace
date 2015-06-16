/*
 * GenVI_v2.0.cpp
 *
 *  Created on: Jan 22, 2015
 *      Author: Tanmay
 */

/***Note: This code is derived from GenVI_Netburner_Advanced except for the GPS reading part**********/

///included
#include "predef.h"
//#include "syslog.h"
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
void SetIntc(int intc, long func, int source, int level);
}

const char *AppName = "GenVI Avionics Netburner Firmware v2";

static uint16_t pilot_input[4] = { 0 }; //0.Throttle, 1.Aileron, 2.Elevator,3.Rudder
static uint16_t controlSwitch = 0;
static char GPS_packet[144] = { 0 };
static char GCSdata[2] = { 0 };
static uint8_t GPSavailFlag = 0;
static uint8_t recavailFlag = 0;
static int fdUart8;
static uint16_t delayVal = 0;

//Control Switch PWM interrupt variables
HiResTimer* controlSwitchReadTimer;
static double controlSwitchPWMval = 0;

FileRoutines OpenOnboardSD; //file descriptor

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
		pilot_input[0] = (uint16_t) ((spektrum_packet_raw[4] & 0x07) * 256)
				+ (uint8_t) (spektrum_packet_raw[5]);

		pilot_input[3] = (uint16_t) ((spektrum_packet_raw[8] & 0x07) * 256)
				+ (uint8_t) (spektrum_packet_raw[9]);

		if (spektrum_packet_raw[20] != 0) {
			pilot_input[1] = (uint16_t) ((spektrum_packet_raw[20] & 0x07) * 256)
					+ (uint8_t) (spektrum_packet_raw[21]); //Aileron
			pilot_input[2] = (uint16_t) ((spektrum_packet_raw[24] & 0x07) * 256)
					+ (uint8_t) (spektrum_packet_raw[25]); //Elev

			controlSwitch = (uint16_t) ((spektrum_packet_raw[26] & 0x07) * 256)
					+ (uint8_t) (spektrum_packet_raw[27]);
		} else {
			pilot_input[1] = (uint16_t) ((spektrum_packet_raw[21] & 0x07) * 256)
					+ (uint8_t) (spektrum_packet_raw[22]);
			pilot_input[2] = (uint16_t) ((spektrum_packet_raw[25] & 0x07) * 256)
					+ (uint8_t) (spektrum_packet_raw[26]);

			controlSwitch = (uint16_t) ((spektrum_packet_raw[27] & 0x07) * 256)
					+ (uint8_t) (spektrum_packet_raw[28]);
		}

		recavailFlag = 1;

	} //task while
} //task

void FlushSD(void*) { //flushes data to SD card

	f_enterFS();

	while (1) {
		OSTimeDly(30 * TICKS_PER_SECOND);
		f_flush(OpenOnboardSD.fp);
	}

	f_releaseFS();

}

void ReadGCS(void*) { //Read commands from ground station

	int fdUart0 = SerialRoutine(0);

	//iprintf("waiting for air data\n");
	while (1) {

		read(fdUart0, &GCSdata[0], 1);

		delayVal = GCSdata[0] * 10;

	}

}

void IMU_Filter_Loop(void *) { //runs Filter loop and saves data to the SD-Card

	////Initialize 100Hz Semaphore Timer to time the filter loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 10ms second intervals
	InitPitOSSem(1, &PitSem1, 100);

	configIMU();
	J2[32] = 1; //Resets the OSD
	Enable_OSD(); //Enables the MAX7456 to display video
	OSTimeDly(2);
	initOSD(); //Initializes the OSD to proper black value and sets DMM to 0;

	InitSingleEndAD();
	float adcVal[8] = { 0 };

	//IMU variables
	BYTE IMU_command[14] = { xahigh, 00, yahigh, 00, zahigh, 00, xghigh, 00,
			yghigh, 00, yghigh, 00, zghigh, 00 };
	double IMU_data[7] = { 0, 0, 0, 0, 0, 0 }; //0-Gyro Z,1-Accel X,2-Accel Y,3-Accel Z,4 Gyro X,5 Gyro Y;
	uint8_t IMU_raw[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

	uint16_t i = 0; //index

	//Angle variables
	angle complimentary_filter = { 0, 0, 0 };
	angle tilt_angle = { 0, 0 };

	//File Routines
	f_enterFS();

	OpenFileRoutine(&OpenOnboardSD);

	//Buffer to be saved on the SD card
	static char SD_card[140] = { 0 };
	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;

	OSTimeDly(TICKS_PER_SECOND * 1);

	float biasSumGx = 0;
	float biasSumGy = 0;
	float biasSumGz = 0;


	 for (i = 0; i < 250; i++) { //Initializing the angle values

	 OSTimeDly(2);
	 GetIMUdata(IMU_command, 14, IMU_data, IMU_raw);
	 biasSumGx += IMU_data[5];
	 biasSumGy += IMU_data[6];
	 biasSumGz += IMU_data[0];
	 //Getting angle from accelerometer
	 tilt_angle.roll = (GetTiltAngle_Roll(IMU_data) + i * tilt_angle.roll)
	 / (i + 1);
	 tilt_angle.pitch = (GetTiltAngle_Pitch(IMU_data) + i * tilt_angle.pitch)
	 / (i + 1);
	 }


	float biasGx = biasSumGx / 250;
	float biasGy = biasSumGy / 250;
	float biasGz = biasSumGz / 250;

	i = 0;

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

	uint8_t controlSwitchFlag = 0;
	uint32_t controlSwitchTime = 0;
	double elevDeflection = 0;

	double elevCommand = 0;

	double doubletTime = 0;

	uint8_t ElevAction = 27; //27 for MultiSine, 23 for doublet,29 for pilot delay

	float pilotCommandArray[100] = { 0.0015 };
	float pilotDelayedCommand = 0;
	uint8_t pilotDelayIndex = 0;
	float pilotms = 0.0015;

	//double multisinePWM=0;

	iprintf("\n%s Application started\r\n", AppName);

	while (1) {
		J2[48] = 0;

		BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);
		//uint16_t pwmr = sim1.mcpwm.mcr;
		//sim1.mcpwm.mcr |= LDOK;
		if (status == OS_NO_ERR) {

			GetIMUdata(IMU_command, 14, IMU_data, IMU_raw);
			GetAttitude(IMU_data, &complimentary_filter, biasGx, biasGy,
					biasGz);

			//updateOSD(complimentary_filter.roll, complimentary_filter.pitch);

			//Assign Data to SD card Array
			AssignIMUtoSD(SD_card, IMU_raw);

			AssignAttitudetoSD(complimentary_filter, SD_card);

			if (GPSavailFlag == 1) {
				unsigned long GPScrc = CalculateBlockCRC32(
						sizeof(GPS_packet) - 4, GPS_packet);
				if ((unsigned char) (GPScrc & 0x000000FF)
						== (unsigned char) GPS_packet[140]
						&& (unsigned char) ((GPScrc & 0x0000FF00) >> 8)
								== (unsigned char) GPS_packet[141]
						&& (unsigned char) ((GPScrc & 0x00FF0000) >> 16)
								== (unsigned char) GPS_packet[142]
						&& (unsigned char) ((GPScrc & 0xFF000000) >> 24)
								== (unsigned char) GPS_packet[143]) {
					AssignGPSxtoSD(GPS_packet, SD_card);
					//iprintf("%x %02x %02x %02x %02x\n",GPScrc,(unsigned char)GPS_packet[140],(unsigned char)GPS_packet[141],(unsigned char)GPS_packet[142],(unsigned char)GPS_packet[143]);
				}
				GPSavailFlag = 0;
			}

			if (recavailFlag == 1) { //new receiver data available

				AssignPilot_toSD(SD_card, pilot_input, controlSwitch);
				pilotms = (pilot_input[2] * 0.00057675 + 0.91439) * 0.001; //Converting 11-bit receiver data to PWM ms

				if (controlSwitch > 1000 && controlSwitchPWMval > 0.0015) { //Control Switch Inactive
					J1[13] = 0; //Autopilot off
					controlSwitchFlag = 0;
					controlSwitchTime = 0;
					doubletTime = 0;
					i = 0;
				}

				pilotCommandArray[pilotDelayIndex] = pilotms;

				delayVal = 120;

				int delayedIndex = pilotDelayIndex - delayVal / 20;

				//
				if (delayedIndex < 0)
					delayedIndex = 100 + delayedIndex;

				pilotDelayedCommand = pilotCommandArray[delayedIndex];

				//
				pilotDelayIndex++;
				//
				if (pilotDelayIndex == 100)
					pilotDelayIndex = 0;

				recavailFlag = 0;

				AssignModPilotDatatoSD(pilotDelayedCommand, SD_card);

			}

			if (controlSwitch < 1000 && controlSwitchPWMval < 0.0015) { //Control Switch Active

			//printf("fuck yeah %d\n", i);

				if (ElevAction == 23) { //Doublet part if active

					double CurrentTime = GetCurrentTime(timer);

					if (controlSwitchFlag == 0) {
						controlSwitchFlag = 1;
						doubletTime = CurrentTime;

					}

					J1[13] = 1; //Autopilot on

					if ((CurrentTime - doubletTime) < 1.0)
						elevDeflection = -15.00;
					else if ((CurrentTime - doubletTime) > 1.0
							&& (CurrentTime - doubletTime) < 2.0)
						elevDeflection = 15.00;
					else
						elevDeflection = 0.00;

					controlSwitchTime++;

					elevCommand = 0.000016529 * elevDeflection + 0.0014727
							+ pilotms - 0.00145;
				}

				if (ElevAction == 27) { //Mutlisine if active
					J1[13] = 1; //Autopilot on
					if (i < 801) {
						elevDeflection = -multiSineElev2p5A5[i] * rad2deg; //get elevator deflection in deg
						i++;

					}

					else {
						elevDeflection = 0;
					}

					elevCommand = 0.000016529 * elevDeflection + 0.0014727 //calculate elevator command in ms
							+ pilotms - 0.00146;
				}

				if (ElevAction == 29) { //Pilot Delay if active
					J1[13] = 1; //Autopilot on
					elevCommand = pilotDelayedCommand;
				}

			}

			StartAD();
			while (!ADDone())

				asm("nop");
			adcVal[0] = ((double) (GetADResult(1))) * 3.3 / 32768.0;

			//printf("%g\n",adcVal[0]);

			AssignADCtoSD(adcVal, SD_card);

			AssignDeltaTandCounter(SD_card, timer);

			uint16_t elevCommandInt = (elevCommand * 10000000.0); //assigning final actuator value to be saved
			SD_card[91] = (elevCommandInt & 0xFF00) >> 8;
			SD_card[92] = (elevCommandInt & 0x00FF);

			//saving biases.
			SD_card[93] = (int8_t) (biasGx * 1000.0);
			SD_card[94] = (int8_t) (biasGy * 1000.0);
			SD_card[95] = (int8_t) (biasGz * 1000.0);

			//saving GPS time
			SD_card[96] = GPS_packet[14];
			SD_card[97] = GPS_packet[15]; //GPS_week

			SD_card[98] = GPS_packet[16]; //GPS_ms
			SD_card[99] = GPS_packet[17];
			SD_card[100] = GPS_packet[18];
			SD_card[101] = GPS_packet[19];

			SD_card[102] = GCSdata[0];

			//printf("%g %d\n",controlSwitchPWMval,controlSwitch);
			//SysLog("%d  %g\n",(uint8_t)SD_card[2],pilotDelayedCommand);

			//for (i = 0; i < sizeof(SD_card); i++) {
			//write(fdUart8, &SD_card[i], 1);
			//}

			if (OpenOnboardSD.fp)
				f_write(&SD_card, 1, sizeof(SD_card), OpenOnboardSD.fp);

			J2[48] = 1;

			//printf("biasGx=%d\n",(uint8_t)SD_card[91]);
		} //If for PIT Sem

		setPWM(11, elevCommand * pwm_freq * pwm_maxlim);
	} //Task While Loop
	  //J2[48] = 1;
	f_close(OpenOnboardSD.fp);
	UnmountFlash(OpenOnboardSD.drv);
	f_releaseFS();

} //IMU_Filter_Loop Task

/**********************Interrupt for reading receiver data******************************/

INTERRUPT(controlSwitchInterrupt,0x2200) {

	sim2.eport.epfr = 0x04; //clearing interrupt flag
	static double previousTime = 0;

	double currentTime = controlSwitchReadTimer->readTime();
	double timeWidth = currentTime - previousTime;

	if (timeWidth < 0.002 && timeWidth > 0.0009)
		controlSwitchPWMval = timeWidth;

	previousTime = currentTime;

}

/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	Usual_Routine();

	//Task Routines
	OSSimpleTaskCreate(IMU_Filter_Loop, MAIN_PRIO + 5);
	//Creating IMU Filter task
	OSSimpleTaskCreate(Read_Spektrum_Receiver, MAIN_PRIO + 4);
	//Creating Receiver Reading Task
	OSSimpleTaskCreate(Read_GPS_Data, MAIN_PRIO + 3);
	//Creating GPS reading Task
	OSSimpleTaskCreate(FlushSD, MAIN_PRIO+1)

	OSSimpleTaskCreate(ReadGCS, MAIN_PRIO+2);

	OSTimeDly(2 * TICKS_PER_SECOND);

	//Setting up interrupt to have 2nd reading of control switch
	J2[43].function(3); //External interrupt 2 reading PPM
	sim2.eport.eppar |= 0x0030;

	controlSwitchReadTimer = HiResTimer::getHiResTimer(1);
	controlSwitchReadTimer->init();


	SetIntc(0, (long) &controlSwitchInterrupt, 2, 1); // Configuring external pins to interrupt and read receiver data
	sim2.eport.epier |= 0x04; // enabling the interrupt
	controlSwitchReadTimer->start();

	while (J1[7]) {
		OSTimeDly(TICKS_PER_SECOND * 1000);
	} //Main While
} //Main Task

