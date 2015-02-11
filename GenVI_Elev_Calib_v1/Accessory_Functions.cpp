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
#include <i2cmaster.h>

void Usual_Routine() {
	InitializeStack();
	OSChangePrio (MAIN_PRIO);
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();
	initPINS();
	//DSPIInit(3, 2000000, 16, 0x00, 1, 1, 1, 0, 0, 0); //Initializing the Hardware to talk to IMU
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
	return(-atan(-IMU_data[2] / sqrt(IMU_data[3] * IMU_data[3]+ IMU_data[4] * IMU_data[4]))*rad2deg);

}

double GetTiltAngle_Roll(double IMU_data[]){

	return(atan(IMU_data[3] / IMU_data[4])*rad2deg);
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
		SD_card[i] = IMU_raw[i - 3];
	}

	SD_card[5] = IMU_raw[12];
	SD_card[6] = IMU_raw[13];
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
	SD_card[33] = GetDeltaT(timer);
	SD_card[34] = 0;
	SD_card[35] = SD_card[35] + 1;
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

void configMPU6050I2C(uint8_t DevAddr){

	I2CInit(0x39);

	BYTE PWR_MGMT_1[]={0x6B,0x01};
	I2CSendBuf(DevAddr,PWR_MGMT_1,sizeof(PWR_MGMT_1),true);//Set X Gyro as clock source and disable sleep mode

	BYTE GYRO_CONFIG[]={0x1B,0x08,0x08};
	I2CSendBuf(DevAddr,GYRO_CONFIG,sizeof(GYRO_CONFIG),true);//Configure the Range of Accel and Gyro

	BYTE WHO_AM_I=0x75;
	I2CSendBuf(DevAddr,&WHO_AM_I,1,false);//Read the device ID
	I2CRestart(DevAddr,true,1);
	BYTE RDevAddr=0;
	I2CReadBuf(DevAddr,&RDevAddr,1,true);

	iprintf("Device Address is=%d\n",RDevAddr);

	BYTE CONFIG[]={0x1A,0x01};
	I2CSendBuf(DevAddr,CONFIG,sizeof(CONFIG),true);//Set Config to set sensor output rate at 1 KHz and disable FSYNC

	BYTE SMPRT_DIV[]={0x19,0x01};
	I2CSendBuf(DevAddr,SMPRT_DIV,sizeof(SMPRT_DIV),true);//Set Sample rate to 500 Hz

	BYTE FIFO_EN[]={0x23,0x78};
	I2CSendBuf(DevAddr,FIFO_EN,sizeof(FIFO_EN),true);//Enable FIFO for Gyro and Accel
}
