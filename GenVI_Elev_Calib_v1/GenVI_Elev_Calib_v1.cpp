/*
 * GenVI_Elev_Calib_v1.cpp
 *
 *  Created on: Feb 10, 2015
 *      Author: Tanmay
 *
 *      Runs the elevator and stores IMU data for offline calibration
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

extern "C" {
void UserMain(void * pd);
}

const char *AppName = "Calibrate PIRAT Elevator";

void PWM_Calib(void *) {

	iprintf("\n%s Application started\r\n", AppName);

	//Initialize I2C
	uint8_t DevAddr = 0x68;

	//setup I2C and confi MPU_6050_I2C
	configMPU6050I2C(DevAddr);

	BYTE ACCEL_XOUT[] = { 0x3B, 0x3C, 0x3D, 0x3E, 0x3F, 0x40, 0x43, 0x44, 0x45,
			0x46, 0x47, 0x48 };
	BYTE IMU_RAW[sizeof(ACCEL_XOUT)] = { 0 };

	OSTimeDly(TICKS_PER_SECOND * 2);
	//File Routines
	f_enterFS();
	FileRoutines OpenOnboardSD;
	OpenFileRoutine(&OpenOnboardSD);

	uint16_t pwm_maxlim;
	uint16_t pwm_freq = 200;

	pwm_maxlim = configPWM(10, pwm_freq);
	OSTimeDly(TICKS_PER_SECOND * 2);
	/*configPWM(xy,Freq)
	 * 	x=1->PWMA
	 * 	x=2->PWMB
	 * 	y=Submodule number
	 * 	Freq=Frequency in Hz
	 * 	Starts PWM at 0.0009 ms duty cycle
	 * 	MAX count 19531//scale accordingly
	 * 	Has to be same frequency for same submodule
	 *
	 * 	Tested for freq between 50-
	 *
	 */

	double PWM_time = 0.0009;
	uint16_t setPWMval = PWM_time * pwm_freq * pwm_maxlim;
	double Av_Ax=0,Av_Ay=0,Av_Az=0;
	uint16_t Avg_count=1;

	 HiResTimer *timer;
	 timer = HiResTimer::getHiResTimer();
	 iprintf("Calibartion started\n");

	while (PWM_time<=0.002) {

		J2[48] = 0;
		PWM_time = PWM_time + 0.0001;
		setPWMval = PWM_time * pwm_freq * pwm_maxlim;



		setPWM(10, setPWMval);
		OSTimeDly(TICKS_PER_SECOND * 3);



		for(Avg_count=1;Avg_count<11;Avg_count++){

			//printf("I am here\n");

			for (uint8_t i = 0; i < sizeof(ACCEL_XOUT); i++) {

				I2CSendBuf(DevAddr, &ACCEL_XOUT[i], 1, false); //Send the registers value to be read
				I2CRestart(DevAddr, true, 1);
				I2CReadBuf(DevAddr, &IMU_RAW[i], 1, true);
			}

			short int Ax_int=((short int)IMU_RAW[0]<<8)|(short int)IMU_RAW[1];
			short int Ay_int=((short int)IMU_RAW[2]<<8)|(short int)IMU_RAW[3];
			short Az_int=((short int)IMU_RAW[4]<<8)|(short int)IMU_RAW[5];

/*
			short Gx_int=((short int)IMU_RAW[6]<<8)|(short int)IMU_RAW[7];
			short Gy_int=((short int)IMU_RAW[8]<<8)|(short int)IMU_RAW[9];
			short Gz_int=((short int)IMU_RAW[10]<<8)|(short int)IMU_RAW[11];
*/

			double Ax=(double)Ax_int/8192;
			double Ay=(double)Ay_int/8192;
			double Az=-(double)Az_int/8192;

/*			double Gx=(double)Gx_int/65.5;
			double Gy=(double)Gy_int/65.5;
			double Gz=(double)Gz_int/65.5;*/

			Av_Ax=(double)((Av_Ax*(Avg_count-1))+Ax)/Avg_count;
			Av_Ay=(double)((Av_Ay*(Avg_count-1))+Ay)/Avg_count;
			Av_Az=(double)((Av_Az*(Avg_count-1))+Az)/Avg_count;

			timer->delay(0.05);




		}//Avg For Loop

		//saving data to SD card

		f_fprintf(OpenOnboardSD.fp, "%d  %g  %g  %g\n", setPWMval,Av_Ax,Av_Ay,Av_Az);
		printf("%d  %g  %g  %g\n", setPWMval,Av_Ax,Av_Ay,Av_Az);

		J2[48] = 1;



	} //Task While Loop

	f_flush(OpenOnboardSD.fp);
	J2[48] = 1;
	f_close(OpenOnboardSD.fp);
	UnmountFlash(OpenOnboardSD.drv);
	f_releaseFS();

	printf("Calibration Finished\n");

} //PWM_Calib scope

/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	Usual_Routine();

	OSSimpleTaskCreate(PWM_Calib, MAIN_PRIO + 1); //Creating GPS reading Task

	while (1) {
		OSTimeDly(TICKS_PER_SECOND * 120);

	} //Main While Bracket

} //Main scope
