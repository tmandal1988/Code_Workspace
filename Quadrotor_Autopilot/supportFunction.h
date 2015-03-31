/*
 * supportFunction.h
 *
 *  Created on: Mar 23, 2015
 *      Author: Tanmay
 */

#ifndef SUPPORTFUNCTION_H_
#define SUPPORTFUNCTION_H_

#include <startnet.h>

#include <dspi.h> //needed for IMU communication

#include "fast_atan2.h"
//#include "fast_sqrt.h"
#include "pinDefinations.h"



struct angle {
	float roll;
	float pitch;
	float yaw;
};

struct coordinates {
	float x;
	float y;
	float z;
};

void Usual_Routine();

inline float fast_sqrt(const float x)
{
  union
  {
    int i;
    float x;
  } u;

  u.x = x;
  u.i = (1<<29) + (u.i >> 1) - (1<<22);
  return u.x;
}

inline void GetIMUdata(BYTE IMU_command[], uint8_t Command_buffer_size,
		float IMU_data[], uint8_t IMU_raw[]) {

	DSPIStart(3, IMU_command, IMU_raw, Command_buffer_size, NULL); //start talking to IMU
	while (!DSPIdone(3)) {
	}; //wait for DSPI to finish


	////for NBEclipse IDE-2.6.2
	IMU_data[0]=0.02*(short int)(IMU_raw[0]*256+IMU_raw[1]);//Z Gyro
	IMU_data[1]=0.00025*(short int)(IMU_raw[2]*256+IMU_raw[3]);//X Accel
	IMU_data[2]=0.00025*(short int)(IMU_raw[4]*256+IMU_raw[5]);//Y Accel
	IMU_data[3]=0.00025*(short int)(IMU_raw[6]*256+IMU_raw[7]);//Z Accel
	IMU_data[4]=0.02*(short int)(IMU_raw[8]*256+IMU_raw[9]);// X Gyro
	IMU_data[5]=0.02*(short int)(IMU_raw[10]*256+IMU_raw[11]);// Y Gyro

	//for NBEclipse IDE-2.7 and above
/*

	IMU_data[0] = 0.02 * (short int) (IMU_raw[0] * 256 + IMU_raw[1]); //Z Gyro
	IMU_data[1] = 0.00025 * (short int) (IMU_raw[2] * 256 + IMU_raw[3]); //Zero For some reason
	IMU_data[2] = 0.00025 * (short int) (IMU_raw[4] * 256 + IMU_raw[5]); //X Accel
	IMU_data[3] = 0.00025 * (short int) (IMU_raw[6] * 256 + IMU_raw[7]); //Y Accel
	IMU_data[4] = 0.00025 * (short int) (IMU_raw[8] * 256 + IMU_raw[9]); //Z Accel
	IMU_data[5] = 0.02 * (short int) (IMU_raw[10] * 256 + IMU_raw[11]); // X Gyro
	IMU_data[6] = 0.02 * (short int) (IMU_raw[12] * 256 + IMU_raw[13]); // Y Gyro
*/

}

inline float GetTiltAngle_Pitch(float IMU_data[]) {
	return (-fast_atan2(-IMU_data[1],
			fast_sqrt(IMU_data[2] * IMU_data[2] + IMU_data[3] * IMU_data[3]+IMU_data[1]*IMU_data[1]))
			* rad2deg);

}

inline float GetTiltAngle_Roll(float IMU_data[]) {

	return (fast_atan2(IMU_data[2],
			fast_sqrt(IMU_data[2] * IMU_data[2] + IMU_data[3] * IMU_data[3]+IMU_data[1]*IMU_data[1]))
			* rad2deg);
}
HiResTimer* InitTimer(int clock_number);

inline float GetDeltaT(HiResTimer* timer) {
	static double LastTime = 0;
	double CurrentTime = timer->readTime();
	float deltaTime=CurrentTime - LastTime;
	LastTime = CurrentTime;
	return(deltaTime);

}

inline void GetAttitude(float IMU_data[], struct angle* complimentary_filter,HiResTimer* timer) {
	float dt=GetDeltaT(timer);

	(*complimentary_filter).roll = 0.98
			* ((*complimentary_filter).roll + dt * IMU_data[4])
			+ 0.02 * GetTiltAngle_Roll(IMU_data);
	(*complimentary_filter).pitch = 0.98
			* ((*complimentary_filter).pitch + dt * IMU_data[5])
			+ 0.02 * GetTiltAngle_Pitch(IMU_data);
}



int SerialRoutine(int port_num);

inline void Read_Serial_Data(int fdGPS, uint8_t header[], int SizeofHeader,
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

inline void Write_Serial_Data(int fd,char data[],uint8_t numBytes){
	for(uint8_t i=0;i<numBytes;i++)
		write(fd,&data[i],1);
}

inline void AssignIMUtoSD(char SD_card[], uint8_t IMU_raw[]) {
	uint8_t i = 3;
	for (i = 3; i < 15; i++) {
			SD_card[i] = IMU_raw[i - 3];
	}

}

void OpenFileRoutine(struct FileRoutines* OpenOnboardSD);


inline void AssignAttitudetoSD(struct angle complimentary_filter, char SD_card[]) {
	int16_t int_roll_fil = complimentary_filter.roll * 250;
	int16_t int_pitch_fil = complimentary_filter.pitch * 250;

	SD_card[25] = (int_roll_fil & 0xFF00) >> 8;
	SD_card[26] = (int_roll_fil & 0x00FF);

	SD_card[27] = (int_pitch_fil & 0xFF00) >> 8;
	SD_card[28] = (int_pitch_fil & 0x00FF);
}

inline void AssignDeltaTandCounter(char SD_card[], HiResTimer* timer) {
	SD_card[96] = GetDeltaT(timer)*10000;
	SD_card[97] = 0;
	SD_card[98] = SD_card[98] + 1;
}

#endif /* SUPPORTFUNCTION_H_ */
