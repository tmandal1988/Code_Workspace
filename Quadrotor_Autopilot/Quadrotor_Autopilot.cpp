/*
 * Quadrotor_Autopilot.cpp
 *
 *  Created on: Mar 23, 2015
 *      Author: Tanmay
 *
 *      1-PWM macros to set up PWM
 *  	2-Different task to flush fp
 *
 * */
//#include <syslog.h>
#include <ucos.h>
#include <constants.h>

#include <sim.h>
#include <pins.h>

#include <HiResTimer.h>
#include <cfinter.h>

#include <pitr_sem.h>//for PIT SEM
#include "supportFunction.h"
#include "pinDefinations.h"

#include "FileSystemUtils.h"
#include "Card_Routines.h"

/**********HiResTimer************/
HiResTimer* ppm_timer;

/**********PPM Variables*********/
float pilot_channel[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

//Inner loop gains
static float const innerloop_gain_pitch = 20.0f;
static float const innerloop_gain_roll = 20.0f;
static float const innerloop_gain_yaw = 30.0f;

/****vicon control data****/
angle viconControl = { 0, 0, 0 };
coordinates viconData = { 0, 0, 0 };

float viconControlZ;
uint8_t viconAvailflag = 0;
int fdUart8;

char SD_card[140] = { 0 };
uint8_t dataReady = 0;

extern "C" {
void UserMain(void * pd);
void SetIntc(int intc, long func, int source, int level);
}

const char *AppName = "Quadrotor Autopilot v1";

/*****************************Inner Loop***************************************************/
void innerLoop(void *) { //runs inner control loop
	//IMU variables
	BYTE IMU_command[12] = { xahigh, 00, yahigh, 00, zahigh, 00, xghigh, 00,
			yghigh, 00, zghigh, 00, };
	float IMU_data[6] = { 0, 0, 0, 0, 0, 0 }; //0-Gyro Z,1-Accel X,2-Accel Y,3-Accel Z,4 Gyro X,5 Gyro Y;
	uint8_t IMU_raw[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	float noSpikeIMUdata[6] = { 0.00 };

/*	BYTE IMU_config[16] = { 0x80, 0x03, 0x8C, 0x01, 0x8D, 0x00, 0x8E, 0x0A,
			0x8F, 0x07, 0x82, 0x01, 0x83, 0x00, 0x80, 0x00 }; //Configs IMU decimation rate value to 2 so Gyro output is 1230 SPS (2460/(1+1))
	//configure IMU
	DSPIStart(3, IMU_config, NULL, sizeof(IMU_config), NULL);
	while (!DSPIdone(3)) {
	};*/

	/***************Initializing PWM*******************/
	//PWM variables
	uint16_t pwm_maxlim;
	uint16_t pwm_freq = 400;

	pwm_maxlim = configPWM(10, pwm_freq);
	configPWM(11, pwm_freq);
	configPWM(12, pwm_freq);
	configPWM(20, pwm_freq);

	uint32_t pwmFactor = pwm_maxlim * pwm_freq;
	uint16_t pwmMin = 0.0009 * pwmFactor, pwmMax = 0.0021 * pwmFactor;
	float throttleLimit = 0.0012 * pwmFactor;

	setPWM(10, pwmMin, false);
	setPWM(11, pwmMin, false);
	setPWM(12, pwmMin, false);
	setPWM(20, pwmMin, true);
	/******************PWM Initialization done**************/

	/*************Angle Initialization******************/
	//Angle variables
	angle complimentary_filter = { 0, 0, 0 };
	angle tilt_angle = { 0, 0, 0 };
	angle rateError = { 0, 0, 0 };
	angle attitudeControl = { 0, 0, 0 };
	angle filviconControl = { 0, 0, 0 };

	/*************IMU_prev*****************/
	float IMU_dataPrev[6] = { 0.0 };

	for (uint16_t i = 0; i < 5000; i++) { //Initializing the angle values

		GetIMUdata(IMU_command, 12, IMU_data, IMU_raw);
		//Getting angle from accelerometer
		tilt_angle.roll = (GetTiltAngle_Roll(IMU_data) + i * tilt_angle.roll)
				/ (i + 1);
		tilt_angle.pitch = (GetTiltAngle_Pitch(IMU_data) + i * tilt_angle.pitch)
				/ (i + 1);
	}

	complimentary_filter.roll = tilt_angle.roll * rad2deg;
	complimentary_filter.pitch = tilt_angle.pitch * rad2deg;
	/***************Angle Initialization done ******************/
	HiResTimer* timer = InitTimer(0);

	//SysLog("\n%s Application started\r\n", AppName);

	OSTimeDly(2);

	//Auto Takeoff variable
	uint8_t takeOff = 0;
	uint32_t loopCount = 0;
	float throttle = 0;

	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;

	float pilotThrottlePrev = 0.00089 * pwmFactor;
	float pilotRollPrev = 0.00001 * pwmFactor;
	float pilotPitchPrev = 0.00001 * pwmFactor;
	float pilotYawPrev = 0.00001 * pwmFactor;

	float filpilotThrottle = 7600;
	float filpilotRoll = 0.0;
	float filpilotPitch = 0.0;
	float filpilotYaw = 0.0;

	angle viconControlPrev = { 0, 0, 0 };
	coordinates viconDataPrev = { 0, 0, 0 };
	coordinates filviconData = { 0, 0, 0 };

	while (1) {
		//dataReady=0;
		J2[48] = 0;
		GetIMUdata(IMU_command, 12, IMU_data, IMU_raw);

		if (fast_abs(IMU_data[0]) > 200.00)
			noSpikeIMUdata[0] = IMU_dataPrev[0];
		else
			noSpikeIMUdata[0] = IMU_data[0];

		if (fast_abs(IMU_data[1]) > 3.00)
			noSpikeIMUdata[1] = IMU_dataPrev[1];
		else
			noSpikeIMUdata[1] = IMU_data[1];

		if (fast_abs(IMU_data[2]) > 3.00)
			noSpikeIMUdata[2] = IMU_dataPrev[2];
		else
			noSpikeIMUdata[2] = IMU_data[2];

		if (fast_abs(IMU_data[3]) > 3.00)
			noSpikeIMUdata[3] = IMU_dataPrev[3];
		else
			noSpikeIMUdata[3] = IMU_data[3];

		if (fast_abs(IMU_data[4]) > 200.00)
			noSpikeIMUdata[4] = IMU_dataPrev[4];
		else
			noSpikeIMUdata[4] = IMU_data[4];

		if (fast_abs(IMU_data[5]) > 200.00)
			noSpikeIMUdata[5] = IMU_dataPrev[5];
		else
			noSpikeIMUdata[5] = IMU_data[5];

		for (uint8_t i = 0; i < 6; i++)
			IMU_dataPrev[i] = IMU_data[i];

		GetAttitude(noSpikeIMUdata, &complimentary_filter, timer);

		if (pilot_channel[4] > 0.0016 && pilot_channel[5] > 0.0016
				&& takeOff == 0) {
			if (loopCount % 5 == 0 & viconControlZ < 1.00)
				throttle++;
			if (viconControlZ > 1.00) {
				takeOff = 1;
				throttle = throttle * 0.95;
			}
		}

		if (pilot_channel[5] < 0.0016 && takeOff == 0) {

			if (throttle > 0) {
				if (loopCount % 5 == 0)
					throttle--;
			}

			else
				throttle = 0;
		}

		if (pilot_channel[5] < 0.0016 && takeOff == 1) {

			if (throttle > 0) {
				if (loopCount % 5 == 0)
					throttle--;
			}

			else {
				throttle = 0;
				takeOff = 0;
			}
		}

		//Pilot command
		float pilotThrottle = pilot_channel[2];
		if (pilotThrottle < throttleLimit)
			pilotThrottle = pilotThrottle * pwmFactor;
		else
			pilotThrottle = 1.2 * pilotThrottle * pwmFactor;

		float pilotRoll = 0.8 * (0.0015 - pilot_channel[0]) * pwmFactor;
		float pilotYaw = (0.0015 - pilot_channel[3]) * pwmFactor;
		float pilotPitch = 0.8 * (0.0015 - pilot_channel[1]) * pwmFactor;

/*		if (fast_abs(pilotThrottle - pilotThrottlePrev) > 1200.00)
			filpilotThrottle = pilotThrottlePrev;
		else
			filpilotThrottle = pilotThrottle;

		if (fast_abs(pilotRoll - pilotRollPrev) > 1200.00)
			filpilotRoll = pilotRollPrev;
		else
			filpilotRoll = pilotRoll;

		if (fast_abs(pilotPitch - pilotPitchPrev) > 1200.00)
			filpilotPitch = pilotPitchPrev;
		else
			filpilotPitch = pilotPitch;

		if (fast_abs(pilotYaw - pilotYawPrev) > 1200.00)
			filpilotYaw = pilotYawPrev;
		else
			filpilotPitch = pilotPitch;*/

		//filpilotThrottle = 7000;

		filpilotRoll = 0;
		filpilotYaw = 0;
		filpilotPitch = 0;

		pilotYawPrev = pilotYaw;
		pilotThrottlePrev = pilotThrottle;
		pilotRollPrev = pilotRoll;
		pilotPitchPrev = pilotPitch;

		//attitude rate error
		rateError.pitch = -noSpikeIMUdata[5];
		rateError.roll = -noSpikeIMUdata[4];
		rateError.yaw = -noSpikeIMUdata[0];

		if (fast_abs(viconControl.roll) > 1000)
			filviconControl.roll = viconControlPrev.roll;
		else
			filviconControl.roll = viconControl.roll;

		if (fast_abs(viconControl.pitch) > 1000)
			filviconControl.pitch = viconControlPrev.pitch;
		else
			filviconControl.pitch = viconControl.pitch;

		if (fast_abs(viconControl.yaw) > 1000)
			filviconControl.yaw = viconControlPrev.yaw;
		else
			filviconControl.yaw = viconControl.yaw;

		if (fast_abs(viconData.z) > 1500)
			filviconData.z = viconDataPrev.z;
		else
			filviconData.z = viconData.z;

		viconControlPrev.roll = viconControl.roll;
		viconControlPrev.pitch = viconControl.pitch;
		viconControlPrev.yaw = viconControl.yaw;
		viconDataPrev.z = viconData.z;

		//attitude control
		attitudeControl.pitch = viconControl.pitch
				+ innerloop_gain_pitch * rateError.pitch; ///pitch control
		attitudeControl.roll = viconControl.roll
				+ innerloop_gain_roll * rateError.roll; //roll control
		attitudeControl.yaw = viconControl.yaw
				+ innerloop_gain_yaw * rateError.yaw; //yaw control

				//X configuration Pitch control
		float Auto1 = throttle + filpilotThrottle + attitudeControl.pitch
				+ filpilotPitch + attitudeControl.yaw - attitudeControl.roll
				+ filpilotRoll - filpilotYaw + filviconData.z;
		float Auto2 = throttle + filpilotThrottle + attitudeControl.pitch
				+ filpilotPitch - attitudeControl.yaw + attitudeControl.roll
				- filpilotRoll + filpilotYaw + filviconData.z;
		float Auto3 = throttle + filpilotThrottle - attitudeControl.pitch
				- filpilotPitch + attitudeControl.yaw + attitudeControl.roll
				- filpilotRoll - filpilotYaw + filviconData.z;
		float Auto4 = throttle + filpilotThrottle - attitudeControl.pitch
				- filpilotPitch - attitudeControl.yaw - attitudeControl.roll
				+ filpilotRoll + filpilotYaw + filviconData.z;

		if (pilot_channel[4] > 0.0016) //Gear Switch, Up kills the motor
				{
			if (Auto1 < pwmMin)
				setPWM(10, pwmMin, false);
			else if (Auto1 > pwmMax)
				setPWM(10, pwmMax, false);
			else
				setPWM(10, Auto1, false);

			if (Auto2 < pwmMin)
				setPWM(11, pwmMin, false);
			else if (Auto2 > pwmMax)
				setPWM(11, pwmMax, false);
			else
				setPWM(11, Auto2, false);

			if (Auto3 < pwmMin)
				setPWM(12, pwmMin, false);
			else if (Auto3 > pwmMax)
				setPWM(12, pwmMax, false);
			else
				setPWM(12, Auto3, false);

			if (Auto4 < pwmMin)
				setPWM(20, pwmMin, true);
			else if (Auto4 > pwmMax)
				setPWM(20, pwmMax, true);
			else
				setPWM(20, Auto4, true);

		}

		else {

			setPWM(10, pwmMin, false);
			setPWM(11, pwmMin, false);
			setPWM(12, pwmMin, false);
			setPWM(20, pwmMin, true);

		}

		//Assign Data to SD card Array

		if (dataReady == 0 && viconAvailflag == 1) {
			AssignIMUtoSD(SD_card, IMU_raw);
			AssignAttitudetoSD(complimentary_filter, SD_card);

			SD_card[15] = ((uint16_t) pilotThrottle & 0xFF00) >> 8;
			SD_card[16] = ((uint16_t) pilotThrottle & 0x00FF);

			SD_card[17] = ((int16_t) pilotRoll & 0xFF00) >> 8;
			SD_card[18] = ((int16_t) pilotRoll & 0x00FF);

			SD_card[19] = ((int16_t) pilotYaw & 0xFF00) >> 8;
			SD_card[20] = ((int16_t) pilotYaw & 0x00FF);

			SD_card[21] = ((int16_t) pilotPitch & 0xFF00) >> 8;
			SD_card[22] = ((int16_t) pilotPitch & 0x00FF);

			SD_card[23] = ((uint16_t) throttle & 0xFF00) >> 8;
			SD_card[24] = ((uint16_t) throttle & 0x00FF);

			SD_card[41] = ((uint16_t) Auto1 & 0xFF00) >> 8;
			SD_card[42] = ((uint16_t) Auto1 & 0x00FF);

			SD_card[43] = ((uint16_t) Auto2 & 0xFF00) >> 8;
			SD_card[44] = ((uint16_t) Auto2 & 0x00FF);

			SD_card[45] = ((uint16_t) Auto3 & 0xFF00) >> 8;
			SD_card[46] = ((uint16_t) Auto3 & 0x00FF);

			SD_card[47] = ((uint16_t) Auto4 & 0xFF00) >> 8;
			SD_card[48] = ((uint16_t) Auto4 & 0x00FF);

			AssignDeltaTandCounter(SD_card, timer);
			dataReady = 1;
			viconAvailflag = 0;
			//SysLog("%g \r\n",pilot_channel[4]);

		}


		loopCount++;

	} //inner loop task while

} //innerloop task

/*****************************Read Vicon Data***************************************************/
void viconRead(void *) { //Task to read data from vicon

	uint8_t header[] = { 0xAA, 0xAB, 0xBB };
	char viconInput[26] = { 0 };

	/*********initializing uart8***********/
	fdUart8 = SerialRoutine(8);

	while (1) {
		Read_Serial_Data(fdUart8, header, sizeof(header), viconInput, 26);

		//J2[48] = 1;

		if ((uint8_t)viconInput[25] == 0xCF) {

			viconControl.roll = (float) ((int16_t) ((uint16_t) viconInput[3]
					* 256) + (uint8_t) viconInput[4]) / 10;

			SD_card[29] = viconInput[3];
			SD_card[30] = viconInput[4];

			viconControl.pitch = (float) ((int16_t) ((uint16_t) viconInput[5]
					* 256) + (uint8_t) viconInput[6]) / 10;

			SD_card[31] = viconInput[5];
			SD_card[32] = viconInput[6];

			viconControl.yaw = (float) ((int16_t) ((uint16_t) viconInput[7]
					* 256) + (uint8_t) viconInput[8]) / 2;

			SD_card[33] = viconInput[7];
			SD_card[34] = viconInput[8];

			viconData.x = (float) ((int16_t) ((uint16_t) viconInput[9] * 256)
					+ (uint8_t) viconInput[10]);
			SD_card[35] = viconInput[9];
			SD_card[36] = viconInput[10];

			viconControlZ = (float) ((int16_t) ((uint16_t) viconInput[11] * 256)
					+ (uint8_t) viconInput[12]) / 100;
			SD_card[37] = viconInput[11];
			SD_card[38] = viconInput[12];

			viconData.z = (float) ((int16_t) ((uint16_t) viconInput[13] * 256)
					+ (uint8_t) viconInput[14]) / 10;
			SD_card[39] = viconInput[13];
			SD_card[40] = viconInput[14];

			SD_card[49] = viconInput[15]; //roll
			SD_card[50] = viconInput[16];

			SD_card[51] = viconInput[17]; //pitch
			SD_card[52] = viconInput[18];

			SD_card[53] = viconInput[19]; //yaw
			SD_card[54] = viconInput[20];

			SD_card[55] = viconInput[21]; //x
			SD_card[56] = viconInput[22];

			SD_card[57] = viconInput[23]; //y
			SD_card[58] = viconInput[24];

			SD_card[99]=0xCF;

			if (dataReady == 1) {
				//SysLog("%d %d\r\n",(unsigned char)SD_card[137],(unsigned char)SD_card[139]);
				Write_Serial_Data(fdUart8, SD_card, 100);
				dataReady = 0;

			}

			viconAvailflag = 1;


		}

	}

}

/*****************************send data to Serial****************************************************/
void send2Serial(void*) {

	////Initialize 100Hz Semaphore Timer to time the filter loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 10ms second intervals
	InitPitOSSem(1, &PitSem1, 25);

	while (1) {
		BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);
		if (status == OS_NO_ERR) {
			if (dataReady == 1) {
				J2[48] = 1;
				Write_Serial_Data(fdUart8, SD_card, 50);
				dataReady = 0;
			}
		}
	}
} //send2Serial scope

/*****************************save data to SD card***************************************************/
void save2SD(void *) { //Task to save Data to SD card
	////Initialize 50Hz Semaphore Timer to time the data saving loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 20ms second intervals
	InitPitOSSem(1, &PitSem1, 25);

	//File Routines
	f_enterFS();
	FileRoutines OpenOnboardSD;
	OpenFileRoutine(&OpenOnboardSD);

	//Buffer to be saved on the SD card

	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;

	uint16_t loopCount = 0;

	while (1) {
		J2[48] = 1;

		BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);
		if (status == OS_NO_ERR) {

			if (dataReady == 1) {

				if (OpenOnboardSD.fp)
					f_write(&SD_card, 1, sizeof(SD_card), OpenOnboardSD.fp);
				dataReady = 0;
			}

			if (loopCount == 100 && pilot_channel[4] < 0.0016) {
				if (OpenOnboardSD.fp)
					f_flush(OpenOnboardSD.fp);
				loopCount = 0;
			}

			loopCount++;

		} //if
	} //save2SD while
} //save2SD scope

/**********************Interrupt for reading receiver data******************************/
INTERRUPT(readPPM,0x2200) {
	sim2.eport.epfr = 0x04; //clearing interrupt flag

	double current_time = 0, time_width = 0;
	static double previous_time = 0;
	static uint8_t pulse_count = 0;
	static uint8_t ppmsync = 0;
	/*
	 * 0-Roll Left->Right 1 ms->1.9 ms
	 * 1-Throttle Bottom->Top
	 * 2-Pitch Bottom->Top
	 * 3-Yaw Left->Right
	 * 4-Gear Up=1.2 ms Down-1.5 ms
	 * 5-Throttle cut Down=1 ms Up=1.9 ms
	 * 6-Pitch Trim Minus sign=1.9 Plus sign=1 ms
	 * 7- Default value=1.5 ms not mapped to anything
	 */
	current_time = ppm_timer->readTime();
	time_width = current_time - previous_time;

	if ((time_width >= 0.006) && ppmsync == 0) {
		ppmsync = 1;
		pulse_count = 0;
	}
	if (ppmsync == 2) {
		if (time_width < 2.19e-3 && time_width > 0.89e-3)
			pilot_channel[pulse_count] = time_width;
		pulse_count = pulse_count + 1;
	}
	if (pulse_count == 8) {
		pulse_count = 0;
		ppmsync = 0;
	}
	previous_time = current_time;
	if (ppmsync == 1)
		ppmsync = 2;

	J2[48] = 1;

	//SysLog("Interrup\r\n");
}

/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	Usual_Routine();

	//Task Routines
	OSSimpleTaskCreate(innerLoop, MAIN_PRIO + 2);
	//Creating inner loop control task
	//OSSimpleTaskCreate(save2SD, MAIN_PRIO + 3); //Creating Task to save data to SD card
	//OSSimpleTaskCreate(send2Serial, MAIN_PRIO + 1);
	OSSimpleTaskCreate(viconRead, MAIN_PRIO + 1);
	//Creating Task to read data from Vicon

	//Configuring and starting the timer to read receiver signall
	ppm_timer = HiResTimer::getHiResTimer(1); //timer for ppm
	ppm_timer->init();
	ppm_timer->start();

	SetIntc(0, (long) &readPPM, 2, 1); // Configuring external pins to interrupt and read receiver data
	sim2.eport.epier |= 0x04; // enabling the interrupt

	while (1) {
		OSTimeDly(10);
	}

}

