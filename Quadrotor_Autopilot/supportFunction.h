/*
 * supportFunction.h
 *
 *  Created on: Mar 23, 2015
 *      Author: Tanmay
 */

#ifndef SUPPORTFUNCTION_H_
#define SUPPORTFUNCTION_H_

struct angle{
	float roll;
	float pitch;
	float yaw;
};

struct coordinates{
	float x;
	float y;
	float z;
};



void Usual_Routine();
void GetIMUdata(BYTE IMU_command[],uint8_t Command_buffer_size,float IMU_data[],uint8_t IMU_raw[]);
float GetTiltAngle_Pitch(float IMU_data[]);
float GetTiltAngle_Roll(float IMU_data[]);
HiResTimer* InitTimer(int clock_number);
void GetAttitude(float IMU_data[],struct angle* complimentary_filter,HiResTimer* timer);
float GetDeltaT(HiResTimer* timer);
int SerialRoutine(int port_num);
void Read_Serial_Data(int fdGPS,uint8_t header[],int SizeofHeader,char GPS_packet[], uint8_t NumberofBytestoRead);

void OpenFileRoutine(struct FileRoutines* OpenOnboardSD);
void AssignIMUtoSD(char SD_card[],uint8_t IMU_raw[]);
void AssignAttitudetoSD(struct angle complimentary_filter,char SD_card[]);
void AssignDeltaTandCounter(char SD_card[],HiResTimer* timer);


#endif /* SUPPORTFUNCTION_H_ */
