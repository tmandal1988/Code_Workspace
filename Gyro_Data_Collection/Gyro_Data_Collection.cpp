/*
 * Gyro_Data_Collection.cpp
 *
 *  Created on: Feb 25, 2015
 *      Author: Tanmay
 *
 *      Collects Gyro Data and saves it on microSD card
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
#include "Accessory_Functions.h" //Housekeeping functions
#include <pitr_sem.h>//for PIT SEM
//Header files necessary for Micro SD card to work
#include "FileSystemUtils.h"
#include "Card_Routines.h"
#include <i2cmaster.h>
#include<fast_atan2.h>
#include<lookup_sin_cos.h>

extern "C" {
void UserMain(void * pd);
}

const char *AppName = "Collecting Gyro Data";

double localPx = 0, localPy = 0, localPz = 0, localVx = 0, localVy = 0,
		localVz = 0;

static char GPS_packet[140] = { 0 };
uint8_t GPSavailFlag = 0;
double Px = 0, Py = 0, Pz = 0, Vx = 0, Vy = 0, Vz = 0;

/**********************Task for reading GPS data***********************************/
void Read_GPS_Data(void*) { //Reads Data from GPS
	SerialClose(8);

	int fdGPS = OpenSerial(8, 115200, 1, 8, eParityNone);

	uint8_t header[] = { 0xAA, 0x44, 0x12 };
	//double Px = 0, Py = 0, Pz = 0, Vx = 0, Vy = 0, Vz = 0;
	Bytes2Double GPSarray;

	while (1) {

		Read_Serial_Data(fdGPS, header, sizeof(header), GPS_packet, 140);

		GPSarray.byteData[0] = GPS_packet[43];
		GPSarray.byteData[1] = GPS_packet[42];
		GPSarray.byteData[2] = GPS_packet[41];
		GPSarray.byteData[3] = GPS_packet[40];

		GPSarray.byteData[4] = GPS_packet[39];
		GPSarray.byteData[5] = GPS_packet[38];
		GPSarray.byteData[6] = GPS_packet[37];
		GPSarray.byteData[7] = GPS_packet[36];

		Px = GPSarray.doubleData;

		GPSarray.byteData[0] = GPS_packet[51];
		GPSarray.byteData[1] = GPS_packet[50];
		GPSarray.byteData[2] = GPS_packet[49];
		GPSarray.byteData[3] = GPS_packet[48];

		GPSarray.byteData[4] = GPS_packet[47];
		GPSarray.byteData[5] = GPS_packet[46];
		GPSarray.byteData[6] = GPS_packet[45];
		GPSarray.byteData[7] = GPS_packet[44];

		Py = GPSarray.doubleData;

		GPSarray.byteData[0] = GPS_packet[59];
		GPSarray.byteData[1] = GPS_packet[58];
		GPSarray.byteData[2] = GPS_packet[57];
		GPSarray.byteData[3] = GPS_packet[56];

		GPSarray.byteData[4] = GPS_packet[55];
		GPSarray.byteData[5] = GPS_packet[54];
		GPSarray.byteData[6] = GPS_packet[53];
		GPSarray.byteData[7] = GPS_packet[52];

		Pz = GPSarray.doubleData;

		GPSarray.byteData[0] = GPS_packet[87];
		GPSarray.byteData[1] = GPS_packet[86];
		GPSarray.byteData[2] = GPS_packet[85];
		GPSarray.byteData[3] = GPS_packet[84];

		GPSarray.byteData[4] = GPS_packet[83];
		GPSarray.byteData[5] = GPS_packet[82];
		GPSarray.byteData[6] = GPS_packet[81];
		GPSarray.byteData[7] = GPS_packet[80];

		Vx = GPSarray.doubleData;

		GPSarray.byteData[0] = GPS_packet[95];
		GPSarray.byteData[1] = GPS_packet[94];
		GPSarray.byteData[2] = GPS_packet[93];
		GPSarray.byteData[3] = GPS_packet[92];

		GPSarray.byteData[4] = GPS_packet[91];
		GPSarray.byteData[5] = GPS_packet[90];
		GPSarray.byteData[6] = GPS_packet[89];
		GPSarray.byteData[7] = GPS_packet[88];

		Vy = GPSarray.doubleData;

		GPSarray.byteData[0] = GPS_packet[103];
		GPSarray.byteData[1] = GPS_packet[102];
		GPSarray.byteData[2] = GPS_packet[101];
		GPSarray.byteData[3] = GPS_packet[100];

		GPSarray.byteData[4] = GPS_packet[99];
		GPSarray.byteData[5] = GPS_packet[98];
		GPSarray.byteData[6] = GPS_packet[97];
		GPSarray.byteData[7] = GPS_packet[96];

		Vz = GPSarray.doubleData;

		localPx = 0.1107 * Px - 0.6258 * Py - 0.7721 * Pz;
		localPy =-( 0.9847 * Px + 0.1741 * Py);
		localPz =-( 0.1344 * Px - 0.7603 * Py + 0.6355 * Pz - 6369948.151050708);

		localVx = 0.1107 * Vx - 0.6258 * Vy - 0.7721 * Vz;
		localVy =-( 0.9847 * Vx + 0.1741 * Vy);
		localVz =-( 0.1344 * Vx - 0.7603 * Vy + 0.6355 * Vz);

		GPSavailFlag = 1;

	}
}

void getGyroData(void *) { //reads Gyro Data and saves it on SD card
	iprintf("\n%s Application started\r\n", AppName);

	////Initialize 50Hz Semaphore Timer to time the filter loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 0.02s second intervals
	InitPitOSSem(1, &PitSem1, 50);

	//IMU variables
	BYTE xgHigh[2] = { xghigh, 0 };
	BYTE ygHigh[2] = { yghigh, 0 };
	BYTE zgHigh[2] = { zghigh, 0 };
	BYTE xgLow[2] = { xglow, 0 };
	BYTE ygLow[2] = { yglow, 0 };
	BYTE zgLow[2] = { zglow, 0 };

	BYTE xaHigh[2] = { xahigh, 0 };
	BYTE yaHigh[2] = { yahigh, 0 };
	BYTE zaHigh[2] = { zahigh, 0 };
	BYTE xaLow[2] = { xalow, 0 };
	BYTE yaLow[2] = { yalow, 0 };
	BYTE zaLow[2] = { zalow, 0 };

	uint8_t xgHighRaw[2] = { 0 };
	uint8_t ygHighRaw[2] = { 0 };
	uint8_t zgHighRaw[2] = { 0 };

	uint8_t xgLowRaw[2] = { 0 };
	uint8_t ygLowRaw[2] = { 0 };
	uint8_t zgLowRaw[2] = { 0 };

	uint8_t xaHighRaw[2] = { 0 };
	uint8_t yaHighRaw[2] = { 0 };
	uint8_t zaHighRaw[2] = { 0 };

	uint8_t xaLowRaw[2] = { 0 };
	uint8_t yaLowRaw[2] = { 0 };
	uint8_t zaLowRaw[2] = { 0 };

	//BYTE IMU_config[8] = { 0x80, 0x03, 0x8C, 0x30, 0x8D, 0x00, 0x80, 0x00 }; //Configs IMU decimation rate value to 48 so Gyro output is 50 SPS (2460/(48+1))
	//BYTE IMU_config[16] = { 0x80, 0x03, 0x8C, 0x3D, 0x8D, 0x00,0x8E,0x0A,0x8F,0x07,0x82,0x01, 0x83, 0x00,0x80,0x00 }; //Configs IMU decimation rate value to 61 so Gyro output is 39.68 SPS (2460/(61+1))
	BYTE IMU_config[16] = { 0x80, 0x03, 0x8C, 0x30, 0x8D, 0x00, 0x8E, 0x0A,
			0x8F, 0x07, 0x82, 0x01, 0x83, 0x00, 0x80, 0x00 }; //Configs IMU decimation rate value to 68 so Gyro output is 50 SPS (2460/(48+1))
	//configure IMU
	DSPIStart(3, IMU_config, NULL, sizeof(IMU_config), NULL);
	while (!DSPIdone(3)) {
	};

	//File Routines
	f_enterFS();
	FileRoutines OpenOnboardSD;
	OpenFileRoutine(&OpenOnboardSD);

	OSTimeDly(TICKS_PER_SECOND * 2);

	//Buffer to be saved on the SD card
	char SD_card[55] = { 0 };
	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;

	uint16_t flush_count = 0;
	uint32_t PITcount = 0;

	//Angle variables
	angle complimentary_filter = { 0, 0 };
	angle tilt_angle = { 0, 0 };

	double IMU_data[7] = { 0 };

	Bytes2Double GPSlocal;

	uint8_t i = 0,j=0,k=0;

	double EKFstates[6]={0,0,0,0,0,0}; //initialize EKF states phi, theta, psi, Vx,Vy,Vz
	double s_phi=0,c_phi=0,s_theta=0,c_theta=0,t_theta=0,sec_theta=0,s_psi=0,c_psi=0;

	double dt=0.02,g=9.80665,pi=3.1416;

	double A[6][6]={{0},{0}};
	double AT[6][6]={{0},{0}};
	double H[3][6]={{0},{0}};
	double P[6][6]={{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
	double PmultAT[6][6]={{0},{0}};
	//double AmultPmultAT[6][6]={{0},{0}};
	double Q[6][6]={{7.88e-4,0,0,0,0,0},{0,8.937e-4,0,0,0,0},{0,0,8.87e-4,0,0,0},{0,0,0,3.047e-6,0,0},{0,0,0,0,4.059e-6,0},{0,0,0,0,0,4.675e-6}};
	//double alpha=2;
	//double R[3][3]={{alpha*3.89e-4,0,0},{0,alpha*1.113e-4,0},{0,0,alpha*9.84e-4}};
	double R[3][3]={{1,0,0},{0,1,0},{0,0,1}};
	double HPHTplusR[3][3]={{0},{0}};
	double detHPHTplusR=1;
	double invHPHTplusR[3][3]={{1},{1}};
	double PHT[6][3]={{0},{0}};
	double K[6][3]={{1},{1}};
	double IminusKH[6][6]={{0},{0}};
	double updateP[6][6]={{0},{0}};

	H[0][3]=1;H[1][4]=1;H[2][5]=1;

	while (1) {

		BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);
		J2[48] = 0;
		if (status == OS_NO_ERR) {

			//precalculate euler angles trignometric Identities
			s_phi=lookup_sin(EKFstates[0]);
			c_phi=lookup_cos(EKFstates[0]);

			s_theta=lookup_sin(EKFstates[1]);
			c_theta=lookup_cos(EKFstates[1]);
			sec_theta=1/c_theta;
			t_theta=s_theta/c_theta;

			s_psi=lookup_sin(EKFstates[2]);
			c_psi=lookup_cos(EKFstates[2]);



			DSPIStart(3, xgHigh, zaLowRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, ygHigh, xgHighRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, zgHigh, ygHighRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, xgLow, zgHighRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, ygLow, xgLowRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, zgLow, ygLowRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, xaHigh, zgLowRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, yaHigh, xaHighRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, zaHigh, yaHighRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, xaLow, zaHighRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, yaLow, xaLowRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			DSPIStart(3, zaLow, yaLowRaw, 2, NULL); //start talking to IMU
			while (!DSPIdone(3)) {
			}; //wait for DSPI to finish

			PITcount = gPitCount[1];

			IMU_data[0] = ((int32_t) zgHighRaw[0] << 24
					| (int32_t) zgHighRaw[1] << 16 | (int32_t) zgLowRaw[0] << 8
					| (int32_t) zgLowRaw[1]) * 0.00000030518*deg2rad; //zGyro
			IMU_data[1] = 0;
			IMU_data[2] = ((int32_t) xaHighRaw[0] << 24
					| (int32_t) xaHighRaw[1] << 16 | (int32_t) xaLowRaw[0] << 8
					| (int32_t) xaLowRaw[1]) * 0.0000000038147*g; //xAccel
			IMU_data[3] = ((int32_t) yaHighRaw[0] << 24
					| (int32_t) yaHighRaw[1] << 16 | (int32_t) yaLowRaw[0] << 8
					| (int32_t) yaLowRaw[1]) * 0.0000000038147*g; //yAccel
			IMU_data[4] = ((int32_t) zaHighRaw[0] << 24
					| (int32_t) zaHighRaw[1] << 16 | (int32_t) zaLowRaw[0] << 8
					| (int32_t) zaLowRaw[1]) * 0.0000000038147*g; //zAccel
			IMU_data[5] = ((int32_t) xgHighRaw[0] << 24
					| (int32_t) xgHighRaw[1] << 16 | (int32_t) xgLowRaw[0] << 8
					| (int32_t) xgLowRaw[1]) * 0.00000030518*deg2rad; //xGyro
			IMU_data[6] = ((int32_t) ygHighRaw[0] << 24
					| (int32_t) ygHighRaw[1] << 16 | (int32_t) ygLowRaw[0] << 8
					| (int32_t) ygLowRaw[1]) * 0.00000030518*deg2rad; //yGyro

			tilt_angle.roll = GetTiltAngle_Roll(IMU_data);
			tilt_angle.pitch = GetTiltAngle_Pitch(IMU_data);

			GetAttitude(IMU_data, &complimentary_filter);

			//EKF prediction stage
			EKFstates[0]=EKFstates[0] + dt*(IMU_data[5]+(IMU_data[6]*s_phi + IMU_data[0]*c_phi)*t_theta);
			EKFstates[1]=EKFstates[1] + dt*(IMU_data[6]*c_phi - IMU_data[0]*s_phi);
			EKFstates[2]=EKFstates[2] + dt*(IMU_data[6]*s_phi + IMU_data[0]*c_phi)*sec_theta;

			EKFstates[3]=EKFstates[3] + dt*(IMU_data[2]*c_theta*c_psi + IMU_data[3]*(-c_phi*s_psi + s_phi*s_theta*c_psi) + IMU_data[4]*(s_phi*s_psi + c_phi*s_theta*c_psi));
			EKFstates[4]=EKFstates[4] + dt*(IMU_data[2]*c_theta*s_psi + IMU_data[3]*(c_phi*c_psi + s_phi*s_theta*s_psi) + IMU_data[4]*(-s_phi*c_psi + c_phi*s_theta*s_psi));
			EKFstates[5]=EKFstates[5] + dt*(-IMU_data[2]*s_theta + (IMU_data[3]*s_phi + IMU_data[4]*c_phi)*c_theta)+g*dt;

			//Calculating A Jacobian
			A[0][0]=1+dt*t_theta*(c_phi*IMU_data[6] - s_phi*IMU_data[0]);
			AT[0][0]=A[0][0];
			A[0][1]=dt*sec_theta*sec_theta*(s_phi*IMU_data[6] + c_phi*IMU_data[0]);
			AT[1][0]=A[0][1];

			A[1][0]=-dt*(s_phi*IMU_data[6] + c_phi*IMU_data[0]);
			AT[0][1]=A[1][0];
			A[1][1]=1;
			AT[1][1]=A[1][1];

			A[2][0]=dt*sec_theta*(c_phi*IMU_data[6] - s_phi*IMU_data[0]);
			AT[0][2]=A[2][0];
			A[2][1]=dt*sec_theta*t_theta*(s_phi*IMU_data[6] + c_phi*IMU_data[0]);
			AT[1][2]=A[2][1];
			A[2][2]=1;
			AT[2][2]=A[2][2];

			A[3][0]=dt*(IMU_data[3]*(s_phi*s_psi + c_phi*s_theta*c_psi) + IMU_data[4]*(c_phi*s_psi - s_phi*s_theta*c_psi));
			AT[0][3]=A[3][0];
			A[3][1]=dt*(IMU_data[2]*(-s_theta*c_psi) + IMU_data[3]*(c_theta*c_psi*s_phi) + IMU_data[4]*(c_phi*c_theta*c_psi));
			AT[1][3]=A[3][1];
			A[3][2]=dt*(IMU_data[2]*(-c_theta*s_psi)-IMU_data[3]*(c_phi*c_psi+s_phi*s_theta*s_psi)+IMU_data[4]*(s_phi*s_psi-s_phi*s_theta*s_psi));
			AT[2][3]=A[3][2];
			A[3][3]=1;
			AT[3][3]=A[3][3];

			A[4][0]=dt*(IMU_data[3]*(-s_phi*c_psi + c_phi*s_theta*s_psi) - IMU_data[4]*(c_phi*c_psi - s_phi*s_theta*s_psi));
			AT[0][4]=A[4][0];
			A[4][1]=dt*(IMU_data[2]*(-s_theta*s_psi) + IMU_data[3]*(c_theta*s_phi*s_psi) + IMU_data[4]*(c_phi*c_theta*s_psi));
			AT[1][4]=A[4][1];
			A[4][2]=dt*(IMU_data[2]*(c_theta*c_psi) + IMU_data[3]*(-c_phi*s_psi + s_phi*s_theta*c_psi) + IMU_data[4]*(s_phi*s_psi + c_phi*s_theta*c_psi));
			AT[2][4]=A[4][2];
			A[4][4]=1;
			AT[4][4]=A[4][4];

			A[5][0]=dt*(IMU_data[3]*(c_phi*c_theta) - IMU_data[4]*(s_phi*c_theta));
			AT[0][5]=A[5][0];
			A[5][1]=-dt*(IMU_data[2]*(c_theta) + IMU_data[3]*(s_theta*s_phi) + IMU_data[4]*(c_phi*s_theta));
			AT[1][5]=A[5][1];
			A[5][5]=1;
			AT[5][5]=A[5][5];

			//Calculating P*A'
			for(i=0;i<6;i++){

				for(j=0;j<6;j++){
					PmultAT[i][j]=0;
					for(k=0;k<6;k++){
						PmultAT[i][j] +=P[i][k]*AT[k][j];
					}

				}

			}

			//Calculating 	P=A*P*A'+Q
			for(i=0;i<6;i++){

				for(j=0;j<6;j++){
					P[i][j]=0;
					for(k=0;k<6;k++){
						P[i][j] +=A[i][k]*PmultAT[k][j];
					}

					P[i][j]+=Q[i][j];

				}

			}

			if (GPSavailFlag == 1) {

				//Assigning Values to HPHTplusR
				HPHTplusR[0][0]=P[3][3]+R[0][0];
				HPHTplusR[0][1]=P[3][4];
				HPHTplusR[0][2]=P[3][5];

				HPHTplusR[1][0]=P[4][3];
				HPHTplusR[1][1]=P[4][4]+R[1][1];
				HPHTplusR[1][2]=P[4][5];

				HPHTplusR[2][0]=P[5][3];
				HPHTplusR[2][1]=P[5][4];
				HPHTplusR[2][2]=P[5][5]+R[2][2];

				detHPHTplusR=HPHTplusR[0][0]*(HPHTplusR[1][1]*HPHTplusR[2][2] - HPHTplusR[2][1]*HPHTplusR[1][2])
					- HPHTplusR[0][1]*(HPHTplusR[1][0]*HPHTplusR[2][2] - HPHTplusR[2][0]*HPHTplusR[1][2])
					+HPHTplusR[0][2]*(HPHTplusR[1][0]*HPHTplusR[2][1] - HPHTplusR[2][0]*HPHTplusR[1][1]);

				invHPHTplusR[0][0]=(HPHTplusR[1][1]*HPHTplusR[2][2] - HPHTplusR[1][2]*HPHTplusR[2][1])/detHPHTplusR;
				invHPHTplusR[0][1]=(HPHTplusR[0][2]*HPHTplusR[2][1] - HPHTplusR[0][1]*HPHTplusR[2][2])/detHPHTplusR;
				invHPHTplusR[0][2]=(HPHTplusR[0][1]*HPHTplusR[1][2] - HPHTplusR[0][2]*HPHTplusR[1][1])/detHPHTplusR;

				invHPHTplusR[1][0]=(HPHTplusR[1][2]*HPHTplusR[2][1] - HPHTplusR[1][0]*HPHTplusR[2][2])/detHPHTplusR;
				invHPHTplusR[1][1]=(HPHTplusR[0][0]*HPHTplusR[2][2] - HPHTplusR[0][2]*HPHTplusR[2][0])/detHPHTplusR;
				invHPHTplusR[1][2]=(HPHTplusR[0][2]*HPHTplusR[1][0] - HPHTplusR[0][0]*HPHTplusR[1][2])/detHPHTplusR;

				invHPHTplusR[2][0]=(HPHTplusR[1][0]*HPHTplusR[2][1] - HPHTplusR[1][1]*HPHTplusR[2][0])/detHPHTplusR;
				invHPHTplusR[2][1]=(HPHTplusR[0][1]*HPHTplusR[2][0] - HPHTplusR[0][0]*HPHTplusR[2][1])/detHPHTplusR;
				invHPHTplusR[2][2]=(HPHTplusR[0][0]*HPHTplusR[1][1] - HPHTplusR[0][1]*HPHTplusR[1][0])/detHPHTplusR;

				//calculating PHT
				PHT[0][0]=P[0][3];
				PHT[0][1]=P[0][4];
				PHT[0][2]=P[0][5];

				PHT[1][0]=P[1][3];
				PHT[1][1]=P[1][4];
				PHT[1][2]=P[1][5];

				PHT[2][0]=P[2][3];
				PHT[2][1]=P[2][4];
				PHT[2][2]=P[2][5];

				PHT[3][0]=P[3][3];
				PHT[3][1]=P[3][4];
				PHT[3][2]=P[3][5];

				PHT[4][0]=P[4][3];
				PHT[4][1]=P[4][4];
				PHT[4][2]=P[4][5];

				PHT[5][0]=P[5][3];
				PHT[5][1]=P[5][4];
				PHT[5][2]=P[5][5];

				//calculating Kalman gain
				for(i=0;i<6;i++){
					for(j=0;j<3;j++){
						K[i][j]=0;
						for(k=0;k<3;k++){
							K[i][j]+=PHT[i][k]*invHPHTplusR[k][j];
						}
					}
				}

				EKFstates[0]=EKFstates[0]+K[0][0]*(localVx - EKFstates[3]) + K[0][1]*(localVy - EKFstates[4]) + K[0][2]*(localVz - EKFstates[5]);
				EKFstates[1]=EKFstates[1]+K[1][0]*(localVx - EKFstates[3]) + K[1][1]*(localVy - EKFstates[4]) + K[1][2]*(localVz - EKFstates[5]);
				EKFstates[2]=EKFstates[2]+K[2][0]*(localVx - EKFstates[3]) + K[2][1]*(localVy - EKFstates[4]) + K[2][2]*(localVz - EKFstates[5]);
				EKFstates[3]=EKFstates[3]+K[3][0]*(localVx - EKFstates[3]) + K[3][1]*(localVy - EKFstates[4]) + K[3][2]*(localVz - EKFstates[5]);
				EKFstates[4]=EKFstates[4]+K[4][0]*(localVx - EKFstates[3]) + K[4][1]*(localVy - EKFstates[4]) + K[4][2]*(localVz - EKFstates[5]);
				EKFstates[5]=EKFstates[5]+K[5][0]*(localVx - EKFstates[3]) + K[5][1]*(localVy - EKFstates[4]) + K[5][2]*(localVz - EKFstates[5]);


				//calculating (I-KH)
				IminusKH[0][0]=1;
				IminusKH[0][3]=-K[0][0];
				IminusKH[0][4]=-K[0][1];
				IminusKH[0][5]=-K[0][2];

				IminusKH[1][1]=1;
				IminusKH[1][3]=-K[1][0];
				IminusKH[1][4]=-K[1][1];
				IminusKH[1][5]=-K[1][2];

				IminusKH[2][2]=1;
				IminusKH[2][3]=-K[2][0];
				IminusKH[2][4]=-K[2][1];
				IminusKH[2][5]=-K[2][2];

				IminusKH[3][3]=1-K[3][0];
				IminusKH[3][4]=-K[3][1];
				IminusKH[3][5]=-K[3][2];

				IminusKH[4][3]=-K[4][0];
				IminusKH[4][4]=1-K[4][1];
				IminusKH[4][5]=-K[4][2];

				IminusKH[5][3]=-K[5][0];
				IminusKH[5][4]=-K[5][1];
				IminusKH[5][5]=1-K[5][2];

				//Calculating P=(I-KH)*P

				for(i=0;i<6;i++){
					for(j=0;j<6;j++){
						updateP[i][j]=0;
						for(k=0;k<6;k++){
							updateP[i][j]+=IminusKH[i][k]*P[k][j];
						}
					}
				}

				//assigning updateP to P;
				for(i=0;i<6;i++){
					for(j=0;j<6;j++){
						P[i][j]=updateP[i][j];
					}
				}



				GPSlocal.doubleData = EKFstates[0];
				for (i = 27; i < 35; i++)
					SD_card[i] = GPSlocal.byteData[i - 27];

				GPSlocal.doubleData = EKFstates[1];
				for (i = 35; i < 43; i++)
					SD_card[i] = GPSlocal.byteData[i - 35];

				GPSlocal.doubleData = EKFstates[2];
				for (i = 43; i < 51; i++)
					SD_card[i] = GPSlocal.byteData[i - 43];

				GPSavailFlag = 0;
			}


			if(EKFstates[0]>2*pi)
				EKFstates[0]=EKFstates[0]-2*pi;
			if(EKFstates[0]<-2*pi)
				EKFstates[0]=EKFstates[0]+2*pi;

			if(EKFstates[1]>2*pi)
				EKFstates[1]=EKFstates[1]-2*pi;
			if(EKFstates[1]<-2*pi)
				EKFstates[1]=EKFstates[1]+2*pi;

			if(EKFstates[2]>2*pi)
				EKFstates[2]=EKFstates[2]-2*pi;
			if(EKFstates[2]<-2*pi)
				EKFstates[2]=EKFstates[2]+2*pi;



			SD_card[3] = xgHighRaw[0];
			SD_card[4] = xgHighRaw[1];
			SD_card[5] = xgLowRaw[0];
			SD_card[6] = xgLowRaw[1];

			SD_card[7] = ygHighRaw[0];
			SD_card[8] = ygHighRaw[1];
			SD_card[9] = ygLowRaw[0];
			SD_card[10] = ygLowRaw[1];

			SD_card[11] = zgHighRaw[0];
			SD_card[12] = zgHighRaw[1];
			SD_card[13] = zgLowRaw[0];
			SD_card[14] = zgLowRaw[1];

			SD_card[15] = xaHighRaw[0];
			SD_card[16] = xaHighRaw[1];
			SD_card[17] = xaLowRaw[0];
			SD_card[18] = xaLowRaw[1];

			SD_card[19] = yaHighRaw[0];
			SD_card[20] = yaHighRaw[1];
			SD_card[21] = yaLowRaw[0];
			SD_card[22] = yaLowRaw[1];

			SD_card[23] = zaHighRaw[0];
			SD_card[24] = zaHighRaw[1];
			SD_card[25] = zaLowRaw[0];
			SD_card[26] = zaLowRaw[1];



			SD_card[51] = (PITcount & 0xFF000000) >> 24;
			SD_card[52] = (PITcount & 0x00FF0000) >> 16;
			SD_card[53] = (PITcount & 0x0000FF00) >> 8;
			SD_card[54] = (PITcount & 0x000000FF);

			if (OpenOnboardSD.fp) {
				f_write(&SD_card, 1, 55, OpenOnboardSD.fp);
			}

			flush_count++;
			if (flush_count == 4999) {

				f_flush(OpenOnboardSD.fp);
				flush_count = 0;
			}

			J2[48] = 1;
		} //PIT Semaphore if

	} //task while scope

	J2[48] = 1;
	f_close(OpenOnboardSD.fp);
	UnmountFlash(OpenOnboardSD.drv);
	f_releaseFS();
} //getGyroData scope

/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	Usual_Routine();

	OSSimpleTaskCreate(Read_GPS_Data, MAIN_PRIO - 2); //Creating GPS reading Task
	OSSimpleTaskCreate(getGyroData, MAIN_PRIO - 1); //Creating GyroReading Task reading Task

	while (1) {
		OSTimeDly(TICKS_PER_SECOND * 120);

	} //Main While Bracket

} //Main scope

