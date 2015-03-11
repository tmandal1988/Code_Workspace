/*
 * Accessory_Functions.cpp
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
#include "PINS_Definations.h"
#include "FileSystemUtils.h"
#include "Card_Routines.h"
#include <pitr_sem.h>//for PIT SEM
#include "Accessory_Functions.h" //Housekeeping functions
#include "OSD_functions.h" //Onscreen Display functions
#include "fast_atan2.h"

void Usual_Routine() {
	InitializeStack();
	OSChangePrio (MAIN_PRIO);
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();
	initPINS();

	J2[32]=1;//Resets the OSD

	OSTimeDly(2);

	DSPIInit(3, 2000000, 16, 0x00, 1, 1, 1, 0, 0, 0); //Initializing the Hardware to talk to IMU
	DSPIInit(1,2000000,16,0x00,0x01,0,0,0,0,0);//Initializing the Hardware to talk to OSD

	Enable_OSD();//Enables the MAX7456 to display video
	OSTimeDly(2);
	initOSD();//Initializes the OSD to proper black value and sets DMM to 0;

	/***********************************Initializing Initial Artificial Horizon*****************************************************************/

	uint16_t x=2834;//Center Position of the Center Circle
	Display_Center(x);//Displaying the Center, argument center address
	Display_Data();//Initializing Pitch and Roll value Display
}

void OpenFileRoutine(struct FileRoutines* OpenOnboardSD) {

	char File_name[20]={0};
	(*OpenOnboardSD).drv =OpenOnBoardFlash();
	int *card_status = initOnBoardSD((*OpenOnboardSD).drv);
	uint8_t n=card_status[1] + 1;
	siprintf(File_name, "LOG%d.txt", n);

	(*OpenOnboardSD).fp = f_open(File_name, "w+");

}

void GetIMUdata(BYTE IMU_command[],uint8_t Command_buffer_size,double IMU_data[],uint8_t IMU_raw[]){

	DSPIStart(3, IMU_command, IMU_raw, Command_buffer_size, NULL); //start talking to IMU
	while (!DSPIdone(3)) {
	}; //wait for DSPI to finish
	IMU_data[0] = 0.02 * (short int) (IMU_raw[0] * 256 + IMU_raw[1]); //Z Gyro
	IMU_data[1] = 0.00025 * (short int) (IMU_raw[2] * 256 + IMU_raw[3]); //Zero For some reason
	IMU_data[2] = 0.00025 * (short int) (IMU_raw[4] * 256 + IMU_raw[5]); //X Accel
	IMU_data[3] = 0.00025 * (short int) (IMU_raw[6] * 256 + IMU_raw[7]); //Y Accel
	IMU_data[4] = 0.00025 * (short int) (IMU_raw[8] * 256 + IMU_raw[9]); //Z Accel
	IMU_data[5] = 0.02 * (short int) (IMU_raw[10] * 256 + IMU_raw[11]); // X Gyro
	IMU_data[6] = 0.02 * (short int) (IMU_raw[12] * 256 + IMU_raw[13]); // Y Gyro
}

double GetTiltAngle_Pitch(double IMU_data[]){
	return(-fast_atan2(-IMU_data[2],sqrt(IMU_data[3] * IMU_data[3]+ IMU_data[4] * IMU_data[4]))*rad2deg);

}

double GetTiltAngle_Roll(double IMU_data[]){

	return(fast_atan2(IMU_data[3],sqrt(IMU_data[2] * IMU_data[2]+ IMU_data[4] * IMU_data[4]))*rad2deg);
}


void GetAttitude(double IMU_data[],struct angle* complimentary_filter){
	(*complimentary_filter).roll=0.98 * ((*complimentary_filter).roll + 0.01 * IMU_data[5]) + 0.02 * GetTiltAngle_Roll(IMU_data);
	(*complimentary_filter).pitch=0.98 * ((*complimentary_filter).pitch + 0.01 * IMU_data[6]) + 0.02 * GetTiltAngle_Pitch(IMU_data);
}

int SerialRoutine(int port_num){
	SerialClose(port_num);
	return(OpenSerial(port_num, 115200, 1, 8, eParityNone));
}

void AssignIMUtoSD(char SD_card[],uint8_t IMU_raw[]){
	uint8_t i = 3;
	for (i = 3; i < 15; i++) {
		if(i==5)
			SD_card[5] = IMU_raw[12];
		if(i==6)
			SD_card[6] = IMU_raw[13];
		if(i!=5 && i!=6)
			SD_card[i] = IMU_raw[i - 3];
	}

}

void AssignAttitudetoSD(struct angle complimentary_filter,char SD_card[]){
	int16_t int_roll_fil = complimentary_filter.roll * 250;
	int16_t int_pitch_fil = complimentary_filter.pitch * 250;

	SD_card[23] = (int_roll_fil & 0xFF00) >> 8;
	SD_card[24] = (int_roll_fil & 0x00FF);

	SD_card[25] = (int_pitch_fil & 0xFF00) >> 8;
	SD_card[26] = (int_pitch_fil & 0x00FF);
}

void AssignPilot_toSD(char SD_card[],uint16_t pilot_input[]){

	SD_card[15] = (pilot_input[0] & 0xFF00) >> 8;
	SD_card[16]= (pilot_input[0] & 0x00FF);

	SD_card[17] = (pilot_input[2] & 0xFF00) >> 8;
	SD_card[18]= (pilot_input[2] & 0x00FF);

	SD_card[19] = (pilot_input[1] & 0xFF00) >> 8;
	SD_card[20]= (pilot_input[1]  & 0x00FF);

	SD_card[21] = (pilot_input[3] & 0xFF00) >> 8;
	SD_card[22]= (pilot_input[3]  & 0x00FF);
}

void AssignGPSxtoSD(char GPS_packet[],char SD_card[]){

	SD_card[27]=GPS_packet[95];
	SD_card[28]=GPS_packet[94];
	SD_card[29]=GPS_packet[93];
	SD_card[30]=GPS_packet[92];
	SD_card[31]=GPS_packet[91];
	SD_card[32]=GPS_packet[90];
	SD_card[33]=GPS_packet[89];
	SD_card[34]=GPS_packet[88];

}

void readSatRec(int fdSpektrum,char spektrum_packet_raw[]){
	uint8_t i=2;
	read(fdSpektrum, &spektrum_packet_raw[0], 1);
		if ((unsigned char) spektrum_packet_raw[0] == 0xFF) {
			read(fdSpektrum, &spektrum_packet_raw[1], 1);
			if ((unsigned char) spektrum_packet_raw[1] == 0xFF) {
				for (i = 2; i < 5; i++)
					read(fdSpektrum, &spektrum_packet_raw[i], 1);
				if ((unsigned char) (spektrum_packet_raw[4] & 0x80)
						== 0x80) {
					for (i = 5; i < 30; i++)
						read(fdSpektrum, &spektrum_packet_raw[i], 1);

					}

				}
			}
}

HiResTimer* InitTimer(int clock_number){
	HiResTimer* timer;
	timer = HiResTimer::getHiResTimer(clock_number); //for keeping time
	timer->init();
	timer->start();
	return(timer);
}

uint8_t GetDeltaT(HiResTimer* timer){
	static double LastTime=0;
	double CurrentTime = timer->readTime();
	uint8_t DeltaTime=(CurrentTime-LastTime)*10000;
	LastTime=CurrentTime;
	return(DeltaTime);
}

void AssignDeltaTandCounter(char SD_card[],HiResTimer* timer){
	SD_card[137] = GetDeltaT(timer);
	SD_card[138] = 0;
	SD_card[139] = SD_card[139] + 1;
}

void AssignADCtoSD(float adcVal[],char SD_card[]){
	int8_t intval0=adcVal[0]*120;
	int8_t intval1=adcVal[1]*120;
	int8_t intval2=adcVal[2]*120;
	int8_t intval3=adcVal[3]*120;

	int8_t intval4=adcVal[4]*120;
	int8_t intval5=adcVal[5]*120;
	int8_t intval6=adcVal[6]*120;
	int8_t intval7=adcVal[7]*120;

	SD_card[35]=intval0;
	SD_card[36]=intval1;
	SD_card[37]=intval2;
	SD_card[38]=intval3;

	SD_card[39]=intval4;
	SD_card[40]=intval5;
	SD_card[41]=intval6;
	SD_card[42]=intval7;
}

void Read_Serial_Data(int fdGPS,uint8_t header[],int SizeofHeader,char GPS_packet[], uint8_t NumberofBytestoRead){
	int i=0;uint8_t header_flag=1;
	read(fdGPS,&GPS_packet[0],1);
	i=1;
	if((unsigned char)GPS_packet[0]==header[0]){
		for(i=1;i<SizeofHeader;i++){
			read(fdGPS,&GPS_packet[i],1);
			if((unsigned char)GPS_packet[i]==header[i])
				header_flag +=1;
		}
		if(header_flag==SizeofHeader){
			for(i=SizeofHeader;i<NumberofBytestoRead;i++)
			read(fdGPS,&GPS_packet[i],1);
		}


	}
}


void configIMU(){

	OSTimeDly(TICKS_PER_SECOND*2);

	BYTE IMU_config[16] = { 0x80, 0x03, 0x8C, 0x30, 0x8D, 0x00, 0x8E, 0x0A,
			0x8F, 0x07, 0x82, 0x01, 0x83, 0x00, 0x80, 0x00 }; //Configs IMU decimation rate value to 68 so Gyro output is 50 SPS (2460/(48+1))
	//configure IMU
	DSPIStart(3, IMU_config, NULL, sizeof(IMU_config), NULL);
	while (!DSPIdone(3)) {
	};
}

void updateOSD(double roll, double pitch){
	Display_Roll(roll);//Displays roll value, argument roll
	Display_Pitch(pitch);//Display pitch values, argument pitch
	Replace_Center_Line(roll,pitch);//deletes old center line and redraws it based on new roll and pitch value, first argument pitch, second argument roll
}
