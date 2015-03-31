/*
 * supportFunction.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: Tanmay
 */

#include <basictypes.h>
#include <pins.h>

#include <startnet.h>
#include <autoupdate.h>
#include <dhcpclient.h>
#include <taskmon.h>
#include <smarttrap.h>

#include <dspi.h> //needed for IMU communication

#include <HiResTimer.h>

#include <serial.h>

#include "supportFunction.h"
#include "pinDefinations.h"
#include "fast_atan2.h"
#include "fast_sqrt.h"
#include "FileSystemUtils.h"
#include "Card_Routines.h"



void Usual_Routine() {
	InitializeStack();
	OSChangePrio(MAIN_PRIO);
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();
	initPins();

	OSTimeDly(2);

	DSPIInit(3, 2000000, 16, 0x00, 1, 1, 1, 0, 0, 0); //Initializing the Hardware to talk to IMU
}

void OpenFileRoutine(struct FileRoutines* OpenOnboardSD) {

	char File_name[20] = { 0 };
	(*OpenOnboardSD).drv = OpenOnBoardFlash();
	int *card_status = initOnBoardSD((*OpenOnboardSD).drv);
	uint8_t n = card_status[1] + 1;
	siprintf(File_name, "LOG%d.txt", n);

	(*OpenOnboardSD).fp = f_open(File_name, "w+");

}

void AssignIMUtoSD(char SD_card[], uint8_t IMU_raw[]) {
	uint8_t i = 3;
	for (i = 3; i < 15; i++) {
		if (i == 5)
			SD_card[5] = IMU_raw[12];
		if (i == 6)
			SD_card[6] = IMU_raw[13];
		if (i != 5 && i != 6)
			SD_card[i] = IMU_raw[i - 3];
	}

}

void AssignAttitudetoSD(struct angle complimentary_filter, char SD_card[]) {
	int16_t int_roll_fil = complimentary_filter.roll * 250;
	int16_t int_pitch_fil = complimentary_filter.pitch * 250;

	SD_card[23] = (int_roll_fil & 0xFF00) >> 8;
	SD_card[24] = (int_roll_fil & 0x00FF);

	SD_card[25] = (int_pitch_fil & 0xFF00) >> 8;
	SD_card[26] = (int_pitch_fil & 0x00FF);
}

void AssignDeltaTandCounter(char SD_card[], HiResTimer* timer) {
	SD_card[137] = GetDeltaT(timer)*10000;
	SD_card[138] = 0;
	SD_card[139] = SD_card[139] + 1;
}

int SerialRoutine(int port_num) {
	SerialClose(port_num);
	return (OpenSerial(port_num, 115200, 1, 8, eParityNone));
}

void Read_Serial_Data(int fdGPS, uint8_t header[], int SizeofHeader,
		char GPS_packet[], uint8_t NumberofBytestoRead) {
	int i = 0;
	uint8_t header_flag = 1;
	read(fdGPS, &GPS_packet[0], 1);
	i = 1;
	if ((unsigned char) GPS_packet[0] == header[0]) {
		for (i = 1; i < SizeofHeader; i++) {
			read(fdGPS, &GPS_packet[i], 1);
			if ((unsigned char) GPS_packet[i] == header[i])
				header_flag += 1;
		}
		if (header_flag == SizeofHeader) {
			for (i = SizeofHeader; i < NumberofBytestoRead; i++)
				read(fdGPS, &GPS_packet[i], 1);
		}

	}
}

void GetIMUdata(BYTE IMU_command[], uint8_t Command_buffer_size,
		float IMU_data[], uint8_t IMU_raw[]) {

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

float GetTiltAngle_Pitch(float IMU_data[]) {
	return (-fast_atan2(-IMU_data[2],
			fast_sqrt(IMU_data[3] * IMU_data[3] + IMU_data[4] * IMU_data[4]))
			* rad2deg);

}

float GetTiltAngle_Roll(float IMU_data[]) {

	return (fast_atan2(IMU_data[3],
			fast_sqrt(IMU_data[2] * IMU_data[2] + IMU_data[4] * IMU_data[4]))
			* rad2deg);
}

float GetDeltaT(HiResTimer* timer) {
	static double LastTime = 0;
	double CurrentTime = timer->readTime();
	float deltaTime=CurrentTime - LastTime;
	LastTime = CurrentTime;
	return(deltaTime);

}

void GetAttitude(float IMU_data[], struct angle* complimentary_filter,HiResTimer* timer) {
	float dt=GetDeltaT(timer);

	(*complimentary_filter).roll = 0.98
			* ((*complimentary_filter).roll + dt * IMU_data[5])
			+ 0.02 * GetTiltAngle_Roll(IMU_data);
	(*complimentary_filter).pitch = 0.98
			* ((*complimentary_filter).pitch + dt * IMU_data[6])l
			+ 0.02 * GetTiltAngle_Pitch(IMU_data);
}

HiResTimer* InitTimer(int clock_number) {
	HiResTimer* timer;
	timer = HiResTimer::getHiResTimer(clock_number); //for keeping time
	timer->init();
	timer->start();
	return (timer);
}
