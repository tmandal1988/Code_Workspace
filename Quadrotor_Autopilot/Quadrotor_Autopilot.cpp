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
float pilot_channel[8]={0,0,0,0,0,0,0,0};

//Inner loop gains
static float const innerloop_gain_pitch= 20.0f;
static float const innerloop_gain_roll= 20.0f;
static float const innerloop_gain_yaw = 30.0f;

/****vicon control data****/
angle viconControl={0, 0, 0};
coordinates viconData={0, 0, 0};

float viconControlZ;
uint8_t viconAvailflag=0;

char SD_card[140] = { 0 };
uint8_t dataReady=0;




extern "C" {
void UserMain(void * pd);
void SetIntc(int intc, long func, int source, int level);
}

const char *AppName = "Quadrotor Autopilot v1";

/*****************************Inner Loop***************************************************/
void innerLoop(void *) { //runs inner control loop
	//IMU variables
	BYTE IMU_command[14] = { xahigh, 00, yahigh, 00, zahigh, 00, xghigh, 00, yghigh, 00, yghigh, 00, zghigh, 00 };
	float IMU_data[7] = { 0, 0, 0, 0, 0, 0 }; //0-Gyro Z,1-Accel X,2-Accel Y,3-Accel Z,4 Gyro X,5 Gyro Y;
	uint8_t IMU_raw[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };


	/***************Initializing PWM*******************/
	//PWM variables
	uint16_t pwm_maxlim;
	uint16_t pwm_freq = 400;

	pwm_maxlim = configPWM(10, pwm_freq);
	configPWM(11, pwm_freq);
	configPWM(12, pwm_freq);
	configPWM(20, pwm_freq);

	uint32_t pwmFactor=pwm_maxlim*pwm_freq;
	uint16_t pwmMin=0.0009*pwmFactor,pwmMax=0.0021*pwmFactor;
	float throttleLimit=0.0012*pwmFactor;


	setPWM(10,pwmMin,false);
	setPWM(11,pwmMin,false);
	setPWM(12,pwmMin,false);
	setPWM(20,pwmMin,true);
	/******************PWM Initialization done**************/

	/*************Angle Initialization******************/
	//Angle variables
	angle complimentary_filter = { 0, 0 , 0};
	angle tilt_angle = { 0, 0, 0 };
	angle rateError = {0, 0, 0};
	angle attitudeControl={0, 0, 0};

	for (uint16_t i = 0; i < 5000; i++) { //Initializing the angle values

		GetIMUdata(IMU_command, 14, IMU_data, IMU_raw);
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

	iprintf("\n%s Application started\r\n", AppName);

	//Auto Takeoff variable
	uint8_t takeOff=0;
	uint32_t loopCount=0;
	uint32_t throttle=0;

	while(1){
		dataReady=0;
		J2[48]=0;
		GetIMUdata(IMU_command, 14, IMU_data, IMU_raw);
		GetAttitude(IMU_data, &complimentary_filter,timer);

		if(pilot_channel[4] > 0.0016 && pilot_channel[5]>0.0016 && takeOff==0){
			if(loopCount%10 == 0 & viconData.z < 1.00)
				throttle++;
			if(viconData.z>1.00){
				takeOff=1;
			}
		}

		if(pilot_channel[5]<0.0016 && takeOff==0){

			if(throttle>0){
				if(loopCount%5==0)
					throttle--;
			}

			else
				throttle=0;
		}

		if(pilot_channel[5] < 0.0016 && takeOff==1){

			if(throttle>0){
				if(loopCount%5==0)
					throttle--;
			}

			else{
				throttle=0;
				takeOff=0;
			}
		}


		//Pilot command
		float pilotThrottle=pilot_channel[1];
		if(pilotThrottle<throttleLimit)
			pilotThrottle=pilotThrottle*pwmFactor;
		else
			pilotThrottle=1.2*pilotThrottle*pwmFactor;

		float pilotRoll=0.8*(0.0015-pilot_channel[0])*pwmFactor;
		float pilotYaw=(0.0015-pilot_channel[3])*pwmFactor;
		float pilotPitch=0.8*(0.0015-pilot_channel[2])*pwmFactor;

		//attitude rate error
		rateError.pitch=-IMU_data[6];
		rateError.roll=-IMU_data[5];
		rateError.yaw=-IMU_data[0];

		//attitude control
		attitudeControl.pitch=viconControl.pitch+innerloop_gain_pitch*rateError.pitch;///pitch control
		attitudeControl.roll=viconControl.roll+innerloop_gain_roll*rateError.roll;//roll control
		attitudeControl.yaw=viconControl.yaw+innerloop_gain_yaw*rateError.yaw;//yaw control

		//X configuration Pitch control
		float Auto1=throttle+pilotThrottle + attitudeControl.pitch + pilotPitch+attitudeControl.yaw - attitudeControl.roll + pilotRoll-pilotYaw+viconControlZ;
		float Auto2=throttle+pilotThrottle + attitudeControl.pitch + pilotPitch-attitudeControl.yaw + attitudeControl.roll - pilotRoll+pilotYaw+viconControlZ;
		float Auto3=throttle+pilotThrottle - attitudeControl.pitch - pilotPitch+attitudeControl.yaw + attitudeControl.roll - pilotRoll-pilotYaw+viconControlZ;
		float Auto4=throttle+pilotThrottle - attitudeControl.pitch - pilotPitch-attitudeControl.yaw - attitudeControl.roll + pilotRoll+pilotYaw+viconControlZ;


		if (pilot_channel[4] > 0.0016) //Gear Switch, Up kills the motor
		{
			if(Auto1<pwmMin)
				setPWM(10,pwmMin,false);
			else if(Auto1>pwmMax)
				setPWM(10,pwmMax,false);
			else
				setPWM(10,Auto1,false);

			if(Auto2<pwmMin)
				setPWM(11,pwmMin,false);
			else if(Auto2>pwmMax)
				setPWM(11,pwmMax,false);
			else
				setPWM(11,Auto2,false);

			if(Auto3<pwmMin)
				setPWM(12,pwmMin,false);
			else if(Auto3>pwmMax)
				setPWM(12,pwmMax,false);
			else
				setPWM(12,Auto3,false);

			if(Auto4<pwmMin)
				setPWM(20,pwmMin,true);
			else if(Auto4>pwmMax)
				setPWM(20,pwmMax,true);
			else
				setPWM(20,Auto4,true);

		}

		else{

			setPWM(10,pwmMin,false);
			setPWM(11,pwmMin,false);
			setPWM(12,pwmMin,false);
			setPWM(20,pwmMin,true);

		}

		//Assign Data to SD card Array
		AssignIMUtoSD(SD_card, IMU_raw);
		AssignAttitudetoSD(complimentary_filter, SD_card);

		SD_card[15] = ((uint16_t)pilotThrottle & 0xFF00) >> 8;
		SD_card[16] = ((uint16_t)pilotThrottle & 0x00FF);

		SD_card[17] = ((uint16_t)pilotRoll & 0xFF00) >> 8;
		SD_card[18] = ((uint16_t)pilotRoll & 0x00FF);

		SD_card[19] = ((uint16_t)pilotYaw & 0xFF00) >> 8;
		SD_card[20] = ((uint16_t)pilotYaw & 0x00FF);

		SD_card[21] = ((uint16_t)pilotPitch & 0xFF00) >> 8;
		SD_card[22] = ((uint16_t)pilotPitch & 0x00FF);

		SD_card[23] = ((uint16_t)throttle & 0xFF00) >> 8;
		SD_card[24] = ((uint16_t)throttle & 0x00FF);



		AssignDeltaTandCounter(SD_card, timer);
		dataReady=1;

		loopCount++;


	}//inner loop task while

}//innerloop task while

/*****************************Read Vicon Data***************************************************/
void viconRead(void *) { //Task to read data from vicon
	int fdUart8 = SerialRoutine(8);
	uint8_t header[] = { 0xAA, 0xAB, 0xBB };
	char viconInput[15]={0};

	while (1) {

		Read_Serial_Data(fdUart8, header, sizeof(header), viconInput, 15);

		viconControl.roll=(float)((int16_t)((uint16_t)viconInput[3]*256)+(uint8_t)viconInput[4])/10;
		SD_card[27]=viconInput[3];
		SD_card[28]=viconInput[4];

		viconControl.pitch=(float)((int16_t)((uint16_t)viconInput[5]*256)+(uint8_t)viconInput[6])/10;
		SD_card[29]=viconInput[5];
		SD_card[30]=viconInput[6];

		viconControl.yaw=(float)((int16_t)((uint16_t)viconInput[7]*256)+(uint8_t)viconInput[8])/2;
		SD_card[31]=viconInput[7];
		SD_card[32]=viconInput[8];

		viconData.x=(float)((int16_t)((uint16_t)viconInput[9]*256)+(uint8_t)viconInput[10]);
		SD_card[33]=viconInput[9];
		SD_card[34]=viconInput[10];

		viconControlZ=(float)((int16_t)((uint16_t)viconInput[11]*256)+(uint8_t)viconInput[12])/100;
		SD_card[35]=viconInput[11];
		SD_card[36]=viconInput[12];

		viconData.z=(float)((int16_t)((uint16_t)viconInput[13]*256)+(uint8_t)viconInput[14])/10;
		SD_card[37]=viconInput[13];
		SD_card[38]=viconInput[14];

		viconAvailflag = 1;

		J2[48]=1;

	}

}

/*****************************save data to SD card***************************************************/
void save2SD(void *) { //Task to save Data to SD card
	////Initialize 100Hz Semaphore Timer to time the filter loop
	OS_SEM PitSem1; //Time Sem
	OSSemInit(&PitSem1, 0);
	// Init for timer 1, at 10ms second intervals
	InitPitOSSem(1, &PitSem1, 100);

	//File Routines
	f_enterFS();
	FileRoutines OpenOnboardSD;
	OpenFileRoutine(&OpenOnboardSD);

	//Buffer to be saved on the SD card

	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;

	uint16_t loopCount=0;

	while (1) {

		BYTE status = OSSemPend(&PitSem1, TICKS_PER_SECOND * 5);
		if (status == OS_NO_ERR) {

			if(dataReady==1){
				if(OpenOnboardSD.fp)
					f_write(&SD_card,1,sizeof(SD_card),OpenOnboardSD.fp);
				dataReady=0;
			}

			if(loopCount==100){
				if(OpenOnboardSD.fp)
					f_flush(OpenOnboardSD.fp);
				loopCount=0;
			}

		}//if
	}//save2SD while
}//save2SD scope


/**********************Interrupt for reading receiver data******************************/
INTERRUPT(readPPM,0x2200) {
	sim2.eport.epfr = 0x04; //clearing interrupt flag

	double current_time=0,time_width=0;
	static double previous_time=0;
	static uint8_t pulse_count=0;
	static uint8_t ppmsync=0;
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
	current_time=ppm_timer->readTime();
	time_width=current_time-previous_time;

	if((time_width>=0.006) && ppmsync==0)
	{
		ppmsync=1;
		pulse_count=0;
	}
	if(ppmsync==2)
	{
		pilot_channel[pulse_count]=time_width;
		pulse_count=pulse_count+1;
	}
	if(pulse_count==8)
	{
		pulse_count=0;
		ppmsync=0;
	}
	previous_time=current_time;
	if(ppmsync==1)
		ppmsync=2;
}

/*****************************UserMain***************************************************/
void UserMain(void* pd) {

	/////Usual Routine
	Usual_Routine();

	//Task Routines
	OSSimpleTaskCreate(innerLoop, MAIN_PRIO + 3); //Creating inner loop control task
	OSSimpleTaskCreate(save2SD, MAIN_PRIO + 2); //Creating Task to save data to SD card
	OSSimpleTaskCreate(viconRead, MAIN_PRIO + 1); //Creating Task to read data from Vicon

	//Configuring and starting the timer to read receiver signall
	ppm_timer = HiResTimer::getHiResTimer(1); //timer for ppm
	ppm_timer->init();
	ppm_timer->start();

	SetIntc(0, (long) &readPPM, 2, 1); // Configuring external pins to interrupt and read receiver data
	sim2.eport.epier |= 0x04; // enabling the interrupt

	while (1) {
		OSTimeDly(100);
	}

}

