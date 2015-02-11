/*
 * Gen_VI_PWM_v1.cpp
 *
 *  Created on: Feb 9, 2015
 *      Author: Tanmay
 *
 *
 *      This code sets up a 250 Hz PWM for HS-7966HB Servo
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



extern "C" {
void UserMain(void * pd);

}

void PWM_Calib(void *) { //PWM Calibration code running at 100 Hz

	//Starting the Code
	iprintf(
			"***********************PWM calibration code v1************************\n");

	//File Routines
	f_enterFS();
	FileRoutines OpenOnboardSD;
	OpenFileRoutine(&OpenOnboardSD);

	OSTimeDly(TICKS_PER_SECOND * 5);

	uint16_t pwm_maxlim;
	uint16_t pwm_freq = 50;

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

	while (PWM_time < 0.002) {

		J2[48] = 0;
		PWM_time = PWM_time + 0.0001;
		setPWMval = PWM_time * pwm_freq * pwm_maxlim;

		setPWM(10, setPWMval);
		OSTimeDly(TICKS_PER_SECOND * 3);

		f_fprintf(OpenOnboardSD.fp, "%d\n", setPWMval);
		printf("%d\n", setPWMval);

		J2[48] = 1;

	} //Task While

	f_flush(OpenOnboardSD.fp);
	J2[48] = 1;
	f_close(OpenOnboardSD.fp);
	UnmountFlash(OpenOnboardSD.drv);
	f_releaseFS();

	printf("Code executed\n");

} //PWM_Calib

/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	Usual_Routine();

	OSSimpleTaskCreate(PWM_Calib, MAIN_PRIO + 1); //Creating GPS reading Task

	while (1) {
		OSTimeDly(TICKS_PER_SECOND * 120);

	} //Main While Bracket

} ///Main Bracket
