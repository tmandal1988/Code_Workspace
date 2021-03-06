/*
 * Autopilot_new.cpp
 *
 * Created on: Jun 26, 2014
 * Author: Tanmay, Caleb
 *
 * Modified on: Oct 27 by Tanmay to save data on onboard MicroSD card
 *
 *
 * New Version of Quadrotor autopilot code. Restarting with everything
 * Modifications added Vicon input to control it autonomously
 * This code uses VICON+PILOT CONTROLS to fly. Incorporates Motor Kill switch command.
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

//Header files necessary for Micro SD card to work
#include "FileSystemUtils.h"
#include "Card_Routines.h"

//IMU Commands Send the hex value to send the corresponding IMU values
#define xghigh 0x12
#define xglow 0x10
#define yghigh 0x16
#define yglow 0x14
#define zghigh 0x1A
#define zglow 0x18
#define xahigh 0x1E
#define xalow 0x1C
#define yahigh 0x22
#define yalow 0x20
#define zahigh 0x26
#define zalow 0x24
#define xm 0x28
#define ym 0x2A
#define zm 0x2C
#define barhigh 0x30
#define barlow 0x2E
#define command_size 12
//Inner loop gains
#define innerloop_gain_pitch 20.0f
#define innerloop_gain_roll 20.0f
#define innerloop_gain_yaw 30.0f
//Vicon data headers
#define header1 0xAA
#define header2 0xAB
#define header3 0xBB
extern "C" {
void UserMain(void * pd);
void SetIntc(int intc, long func, int source, int level);
}
//Global Variables Warning: Think twice before defining any global variable, Do you need it absolutely?
/**********HiResTimer************/
HiResTimer* ppm_timer;
/**********PPM Variables*********/
uint8_t ppmsync = 0;
double pilot_channel[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
double current_time, previous_time, time_width;
uint8_t pulse_count = 0;
/******Structures********/
//Coordinate structure
typedef struct {
	double roll;
	double pitch;
	double yaw;
	double x;
	double alt;
	double z;
	/********VICON Structure********/
} COORDINATE;
COORDINATE vicon_control;

F_FILE* fp;

char SD_card[38] = { 0 };

/**************************************Task to read Vicon Data*********************************/
void Vicon_read(void *) {
	//printf("1");
	int fdvicon;
	SerialClose(8);
	fdvicon = OpenSerial(8, 115200, 1, 8, eParityNone);
	uint8_t i = 0;
	char vicon_input[100] = { 0 };
	while (1) {
		read(fdvicon, &vicon_input[0], 1);
		if ((uint8_t) vicon_input[0] == header1) {
			//printf("2");
			read(fdvicon, &vicon_input[1], 1);
			read(fdvicon, &vicon_input[2], 1);
			if ((uint8_t) vicon_input[1] == header2
					&& (uint8_t) vicon_input[2] == header3) {
				for (i = 3; i < 16; i++)
					read(fdvicon, &vicon_input[i], 1);
				vicon_control.roll = (double) ((int16_t)(
						(uint16_t) vicon_input[3] * 256)
						+ (uint8_t) vicon_input[4]) / 10;
				SD_card[15] = vicon_input[3];
				SD_card[16] = vicon_input[4];
				vicon_control.pitch = (double) ((int16_t)(
						(uint16_t) vicon_input[5] * 256)
						+ (uint8_t) vicon_input[6]) / 10;
				SD_card[17] = vicon_input[5];
				SD_card[18] = vicon_input[6];
				vicon_control.yaw = (double) ((int16_t)(
						(uint16_t) vicon_input[7] * 256)
						+ (uint8_t) vicon_input[8]) / 2;
				SD_card[19] = vicon_input[7];
				SD_card[20] = vicon_input[8];
				vicon_control.x = (double) ((int16_t)(
						(uint16_t) vicon_input[9] * 256)
						+ (uint8_t) vicon_input[10]);
				SD_card[21] = vicon_input[9];
				SD_card[22] = vicon_input[10];
				vicon_control.alt = (double) ((int16_t)(
						(uint16_t) vicon_input[11] * 256)
						+ (uint8_t) vicon_input[12]) / 100;
				SD_card[23] = vicon_input[11];
				SD_card[24] = vicon_input[12];
				vicon_control.z = (double) ((int16_t)(
						(uint16_t) vicon_input[13] * 256)
						+ (uint8_t) vicon_input[14]) / 10;
				SD_card[25] = vicon_input[13];
				SD_card[26] = vicon_input[14];

				//printf("%g %g %g %g\n", vicon_control.roll, vicon_control.pitch, vicon_control.yaw, vicon_control.alt);
			} //second receive if
		} //first receive if
	} //Vicon Task While Loop
} //Vicon_Task Bracket

/*************************************Autopilot_loop*******************************************/
void Autopilot_loop(void *) { //runs inner loop and controls motor comamnds according to the control algorithm

	f_enterFS(); //Necessary to call file functions
	char File_name[6];
	int drv = OpenOnBoardFlash(); //load on board microSD
	int *card_status = initOnBoardSD(drv);
	sprintf(File_name, "LOG%d.txt", card_status[1] + 1);

	fp = f_open(File_name, "w+"); //open the file to write

	SD_card[0] = 0xAA;
	SD_card[1] = 0xAB;
	SD_card[2] = 0xBB;

	//IMU variables
	BYTE IMU_command[12] = { xahigh, 00, yahigh, 00, zahigh, 00, xghigh, 00,
			yghigh, 00, zghigh, 00, };
	double IMU_data[6] = { 0, 0, 0, 0, 0, 0 }; //0-Gyro Z,1-Accel X,2-Accel Y,3-Accel Z,4 Gyro X,5 Gyro Y;
	uint8_t IMU_raw[12] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint16_t pilot_throttle = 0;
	int16_t pilot_roll = 0, pilot_yaw = 0, pilot_pitch = 0;
	COORDINATE rate_error, attitude_control;
	uint16_t Auto1, Auto2, Auto3, Auto4;
	uint16_t pwmr = 0;

	/*******AutoTakeoff*********/
	uint16_t throttle = 0;
	int take_off = 0;
	int short first = 1; //for auto landing
	uint32_t count = 0;

	//Loop counter
	int i;

	while (1) {

		if (pilot_channel[5] > 0.0015 && take_off == 0) //If Auto Takeoff Switch is ON initiate Takeoff
				{
			if (count % 10 == 0 && vicon_control.alt < 1.00) //Every ten iterations add to throttle
					{
				//if(throttle<P_END) throttle_set = P_END;
				throttle = throttle + 1; //0.5*(P_END/throttle_set);]

			}

			if (vicon_control.alt > 1.00) //vicon control during cruise
					{
				throttle = throttle * 0.95;
				take_off = 1; //take off complete
			}

		}

		if (pilot_channel[5] < 0.0015 && take_off == 1) //Initiate auto land sequence
				{
			if (count % 8 == 0 && take_off == 1) //every 8 iterations take away from throttle
					{

				if (throttle > 0) //P_END)
						{
					if (first == 1) {
						throttle = throttle * 0.95;
						first = 0;
					}
					throttle = throttle - 1;
				} else {
					throttle = 0; //P_END;
					take_off = 0; //when throttle drops below P_END set it to P_END
				}
			}
		}

		count = count + 1;

		DSPIStart(3, IMU_command, IMU_raw, 12, NULL); //start talking to IMU
		while (!DSPIdone(3)) {
		}; //wait for DSPI to finish
		IMU_data[0] = 0.02 * (short int) (IMU_raw[0] * 256 + IMU_raw[1]); //Z Gyro
		IMU_data[1] = 0.00025 * (short int) (IMU_raw[2] * 256 + IMU_raw[3]); //X Accel
		IMU_data[2] = 0.00025 * (short int) (IMU_raw[4] * 256 + IMU_raw[5]); //Y Accel
		IMU_data[3] = 0.00025 * (short int) (IMU_raw[6] * 256 + IMU_raw[7]); //Z Accel
		IMU_data[4] = 0.02 * (short int) (IMU_raw[8] * 256 + IMU_raw[9]); // X Gyro
		IMU_data[5] = 0.02 * (short int) (IMU_raw[10] * 256 + IMU_raw[11]); // Y Gyro

		//Parsing Pilot command
		pilot_throttle = pilot_channel[2] * P_FACTOR;
		if (pilot_throttle < T_LIMIT)
			pilot_throttle = T_LIMIT;
		if (pilot_throttle > T_LIMIT)
			pilot_throttle = 1.2 * pilot_throttle;

		pilot_roll = 0.8 * (0.0015 - pilot_channel[0]) * P_FACTOR - 650;
		pilot_yaw = 1 * (0.0015 - pilot_channel[3]) * P_FACTOR + 550;
		pilot_pitch = 0.8 * (0.0015 - pilot_channel[1]) * P_FACTOR - 550;

		//Attitude rate error
		rate_error.pitch = -IMU_data[5];
		rate_error.roll = -IMU_data[4];
		rate_error.yaw = -IMU_data[0];

		//attitude control
		attitude_control.pitch = vicon_control.pitch
				+ innerloop_gain_pitch * rate_error.pitch; ///pitch control
		attitude_control.roll = vicon_control.roll
				+ innerloop_gain_roll * rate_error.roll; //roll control
		attitude_control.yaw = vicon_control.yaw
				+ innerloop_gain_yaw * rate_error.yaw; //yaw control

				//X configuration Pitch control
		Auto1 = throttle + pilot_throttle + vicon_control.z
				+ attitude_control.pitch + pilot_pitch + attitude_control.yaw
				- attitude_control.roll + pilot_roll - pilot_yaw;
		Auto2 = throttle + pilot_throttle + vicon_control.z
				+ attitude_control.pitch + pilot_pitch - attitude_control.yaw
				+ attitude_control.roll - pilot_roll + pilot_yaw;
		Auto3 = throttle + pilot_throttle + vicon_control.z
				- attitude_control.pitch - pilot_pitch + attitude_control.yaw
				+ attitude_control.roll - pilot_roll - pilot_yaw;
		Auto4 = throttle + pilot_throttle + vicon_control.z
				- attitude_control.pitch - pilot_pitch - attitude_control.yaw
				- attitude_control.roll + pilot_roll + pilot_yaw;

		//printf("Throttle=%d\n",throttle);

		if (pilot_channel[4] > 0.0015) //Gear Switch, Up kills the motor
				{
			J2[48] = J2[48] ^ J2[48];

			if (Auto1 < P_END)
				sim1.mcpwm.sm[0].val[3] = P_END;
			else if (Auto1 > P_MAX2)
				sim1.mcpwm.sm[0].val[3] = P_MAX2;
			else
				//sim1.mcpwm.sm[0].val[3]=Auto1;
				sim1.mcpwm.sm[0].val[3] = Auto1;

			if (Auto2 < P_END)
				sim1.mcpwm.sm[1].val[3] = P_END;
			else if (Auto2 > P_MAX2)
				sim1.mcpwm.sm[1].val[3] = P_MAX2;
			else
				sim1.mcpwm.sm[1].val[3] = Auto2;

			if (Auto3 < P_END)
				sim1.mcpwm.sm[2].val[3] = P_END;
			else if (Auto3 > P_MAX2)
				sim1.mcpwm.sm[2].val[3] = P_MAX2;
			else
				sim1.mcpwm.sm[2].val[3] = Auto3;

			if (Auto4 < P_END)
				sim1.mcpwm.sm[0].val[5] = P_END;
			else if (Auto4 > P_MAX2)
				sim1.mcpwm.sm[0].val[5] = P_MAX2;
			else
				sim1.mcpwm.sm[0].val[5] = Auto4;
		} else {
			sim1.mcpwm.sm[0].val[3] = P_END;
			sim1.mcpwm.sm[1].val[3] = P_END;
			sim1.mcpwm.sm[2].val[3] = P_END;
			sim1.mcpwm.sm[0].val[5] = P_END;
		}
		pwmr = sim1.mcpwm.mcr;
		sim1.mcpwm.mcr |= LDOK;

		i = 3;

		for (int i = 3; i < 15; i++) {
			SD_card[i] = IMU_raw[i - 3];
		}

		SD_card[27] = (pilot_throttle & 0xFF00) >> 8;
		SD_card[28] = (pilot_throttle & 0x00FF);

		SD_card[29] = (pilot_roll & 0xFF00) >> 8;
		SD_card[30] = (pilot_roll & 0x00FF);

		SD_card[31] = (pilot_yaw & 0xFF00) >> 8;
		SD_card[32] = (pilot_yaw & 0x00FF);

		SD_card[33] = (pilot_pitch & 0xFF00) >> 8;
		SD_card[34] = (pilot_pitch & 0x00FF);

		SD_card[35] = (throttle & 0xFF00) >> 8;
		SD_card[36] = (throttle & 0x00FF);

		SD_card[37]=SD_card[37]+1;

		if (fp) {
			f_write(&SD_card, 1, 38, fp);
			//iprintf("PIT Timer count: %d\n", n );
			//f_close(fp);
		}

		// printf("%g %g %g %g %g %g %g\n", pilot_channel[0], pilot_channel[1], pilot_channel[2], pilot_channel[3], pilot_channel[4], pilot_channel[5], pilot_channel[6]);
		//printf("%i %i %i %i \n", Auto1, Auto2, Auto3, Auto4);
		//printf("zg=%f xg=%f yg=%f xa=%f ya=%f za=%f \n", IMU_data[0], IMU_data[4], IMU_data[5], IMU_data[1], IMU_data[2], IMU_data[3]);
		//printf("%i %i %i %i %i \n",throttle, sim1.mcpwm.sm[0].val[3],sim1.mcpwm.sm[1].val[3],sim1.mcpwm.sm[2].val[3],sim1.mcpwm.sm[0].val[5]);

	} //Autopilot_loop while loop

	f_close(fp);
	UnmountFlash(drv);
	f_releaseFS();
} //Autopilot_loop task main bracket

/**********************Interrupt for reading receiver data******************************/
INTERRUPT(PPM_read_interrupt,0x2200) {
	sim2.eport.epfr = 0x04; //clearing interrupt flag
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
} //PPM_read_interrupt bracket

/*****************************UserMain***************************************************/
void UserMain(void* pd) {
	/////Usual Routine
	InitializeStack();
	OSChangePrio (MAIN_PRIO);
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();
	initPINS();

	/**********************Start Serial Port**************/
	int fdDebug; //Serial fd variable
	SerialClose(0);
	fdDebug = OpenSerial(0, 115200, 1, 8, eParityNone); //opening
	ReplaceStdio(0, fdDebug); // stdin via UART 7
	ReplaceStdio(1, fdDebug); // stdout via UART 7
	ReplaceStdio(2, fdDebug); // sterr via UART 7

	//Starting the Code
	iprintf(
			"***********************Autopilot New Version 2.2 code************************\n");
	DSPIInit(3, 2000000, 16, 0x01, 1, 1, 1, 0, 0, 0); //Initializing the Hardware to talk to IMU
	OSSimpleTaskCreate(Autopilot_loop, MAIN_PRIO + 2); //Creating an Autopilot Task
	OSSimpleTaskCreate(Vicon_read, MAIN_PRIO + 1); //Creating a Task to read data from vicon
	//Configuring and starting the timer to read receiver signal
	ppm_timer = HiResTimer::getHiResTimer(1); //timer for ppm
	ppm_timer->init();
	ppm_timer->start();
	SetIntc(0, (long) &PPM_read_interrupt, 2, 1); // Configuring external pins to interrupt and read receiver data
	sim2.eport.epier |= 0x04; // enabling the interrupt

	f_enterFS();
	while (1) {
		OSTimeDly(TICKS_PER_SECOND * 30);
		f_flush(fp);
	} //Main While Bracket

	f_releaseFS();
} ///Main Bracket

