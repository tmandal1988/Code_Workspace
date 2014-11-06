/*
 * Accessory_Functions.h
 *
 *  Created on: Oct 28, 2014
 *      Author: Tanmay
 */


#ifndef ACCESSORY_FUNCTIONS_H_
#define ACCESSORY_FUNCTIONS_H_

#include "Accessory_Functions.h" //Housekeeping functions

struct angle{
	double roll;
	double pitch;
	double yaw;
};

void Usual_Routine();
void CreateTasks();
void OpenFileRoutine(struct FileRoutines* OpenOnboardSD);
void GetIMUdata(BYTE IMU_command[],uint8_t Command_buffer_size,double IMU_data[],uint8_t IMU_raw[]);
double GetTiltAngle_Pitch(double IMU_data[]);
double GetTiltAngle_Roll(double IMU_data[]);
void GetAttitude(double IMU_data[],struct angle* complimentary_filter);
int SerialRoutine(int port_num);
void AssignIMUtoSD(char SD_card[],uint8_t IMU_raw[]);
void AssignAttitudetoSD(struct angle complimentary_filter,char SD_card[]);
void AssignPilot_toSD(char SD_card[],uint16_t pilot_input[]);
void readSatRec(int fdSpektrum,char spektrum_packet_raw[]);
HiResTimer* InitTimer(int clock_number);
uint8_t GetDeltaT(HiResTimer* timer);
void AssignDeltaTandCounter(char SD_card[],HiResTimer* timer);



#endif /* ACCESSORY_FUNCTIONS_H_ */
