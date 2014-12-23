/*
 * MPU_6050_I2C.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: Tanmay
 */

#include <i2cmaster.h>
//#include <i2cmulti.h>

#include <autoupdate.h>
#include <dhcpclient.h>
#include <smarttrap.h>
#include <taskmon.h>
#include <startnet.h>

#include <pitr_sem.h>//for PIT SEM
#include <pins.h>


extern "C" {
void UserMain(void * pd);
//void SetIntc(int intc, long func, int source, int level);
}

const char *AppName = "Read MPU6050 v1";

void Read_MPU6050(void *) {//Reads data from sparkfun MPU6050 breakout board
	iprintf("\n%s Application started\r\n", AppName);

	////Initialize 100Hz Semaphore Timer to time the filter loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 20ms second intervals
	InitPitOSSem(1, &PitSem1, 50);

	//Initialize I2C
	I2CInit(0x39);
	uint8_t DevAddr=0x68;


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

	BYTE ACCEL_XOUT[]={0x3B,0x3C,0x3D,0x3E,0x3F,0x40,0x43,0x44,0x45,0x46,0x47,0x48};
	BYTE IMU_RAW[sizeof(ACCEL_XOUT)]={0};

	while (1) {
			J2[48] = 0;
			BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);

			if (status == OS_NO_ERR) {

				for(uint8_t i=0;i<sizeof(ACCEL_XOUT);i++){

					I2CSendBuf(DevAddr,&ACCEL_XOUT[i],1,false);//Send the registers value to be read
					I2CRestart(DevAddr,true,1);
					I2CReadBuf(DevAddr,&IMU_RAW[i],1,true);
				}

			short int Ax_int=((short int)IMU_RAW[0]<<8)|(short int)IMU_RAW[1];
			short int Ay_int=((short int)IMU_RAW[2]<<8)|(short int)IMU_RAW[3];
			short Az_int=((short int)IMU_RAW[4]<<8)|(short int)IMU_RAW[5];

			short Gx_int=((short int)IMU_RAW[6]<<8)|(short int)IMU_RAW[7];
			short Gy_int=((short int)IMU_RAW[8]<<8)|(short int)IMU_RAW[9];
			short Gz_int=((short int)IMU_RAW[10]<<8)|(short int)IMU_RAW[11];

			double Ax=(double)Ax_int/8192;
			double Ay=(double)Ay_int/8192;
			double Az=-(double)Az_int/8192;

			double Gx=(double)Gx_int/65.5;
			double Gy=(double)Gy_int/65.5;
			double Gz=(double)Gz_int/65.5;

			printf("Ax=%0.2f Ay=%0.2f Az=%0.2f Gx=%0.2f Gy=%0.2f Gz=%0.2f\n",Ax,Ay,Az,Gx,Gy,Gz);


			}//SEM IF
	}//Task While
}
/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	InitializeStack();
	OSChangePrio (MAIN_PRIO);
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	J2[48].function(0);//led D2,GPIO
	J2[48]=0;

	OSSimpleTaskCreate(Read_MPU6050, MAIN_PRIO + 1); //Creating Receiver Reading Task
	while (1) {
		OSTimeDly(TICKS_PER_SECOND * 30);
	} //Main While
} //UserMain
