/*
 * GenVI_Netburner.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: Tanmay
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
#include "FileSystemUtils.h"
#include "Card_Routines.h"
#include <pitr_sem.h>//for PIT SEM

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

#define MAX_PILOT  0.0021f
#define rad2deg 57.296f
#define deg2rad 0.017f

extern "C"{
	void UserMain( void * pd);
	void SetIntc( int intc, long func, int source, int level);
}

const char *AppName = "GenVI Avionics Netburner Code v1";

/**********HiResTimer************/
HiResTimer* ppm_timer;
/**********PPM Variables*********/
uint8_t ppmsync=0;
double pilot_channel[8]={0,0,0,0,0,0,0,0};
double current_time,previous_time,time_width;
uint8_t pulse_count=0;

F_FILE* fp;

/**********************Interrupt for reading receiver data******************************/
INTERRUPT(PPM_read_interrupt,0x2200){
	sim2.eport.epfr = 0x04;//clearing interrupt flag
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
	J2[48]=J2[48]^J2[48];
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
}//PPM_read_interrupt bracket

void IMU_Filter_Loop(void *){ //runs Filter loop and saves data to the SD-Card

	iprintf( "\n%s Application started\r\n", AppName );

	OS_SEM PitSem1;//Time Sem
    OSSemInit( &PitSem1, 0 );
    // Init for timer 1, at 20ms second intervals
    InitPitOSSem( 1, &PitSem1, 50 );

    HiResTimer* timer;

    timer = HiResTimer::getHiResTimer(0);//for keeping time
    timer->init();

	//IMU variables
	BYTE IMU_command[12]={xahigh,00,yahigh,00,zahigh,00,xghigh,00,yghigh,00,yghigh,00};
	double IMU_data[7]={0,0,0,0,0,0};//0-Gyro Z,1-Accel X,2-Accel Y,3-Accel Z,4 Gyro X,5 Gyro Y;
	uint8_t IMU_raw[12]={0,0,0,0,0,0,0,0,0,0,0,0};

	uint8_t pilot_throttle=0,pilot_roll=0,pilot_yaw=0,pilot_pitch=0;

	uint16_t i=0;
	double Roll_Acc=0;
	double Pitch_Acc=0;

	f_enterFS();
	char File_name[6];
	int drv=OpenOnBoardFlash();
	int *card_status=initOnBoardSD(drv);
	sprintf(File_name,"LOG%d.txt",card_status[1]+1);

	fp = f_open( File_name, "w+" );

	char SD_card[24]={0};
	SD_card[0]=0xAA;SD_card[1]=0xAB;SD_card[2]=0xBB;

	for(i=0;i<5000;i++){//Initializing the angle values

		DSPIStart(3,IMU_command,IMU_raw,12,NULL);//start talking to IMU
		while(!DSPIdone(3)){};//wait for DSPI to finish
		IMU_data[0]=0.02*(short int)(IMU_raw[0]*256+IMU_raw[1]);//Y Gyro
		IMU_data[1]=0.00025*(short int)(IMU_raw[2]*256+IMU_raw[3]);//Zero For some reason
		IMU_data[2]=0.00025*(short int)(IMU_raw[4]*256+IMU_raw[5]);//X Accel
		IMU_data[3]=0.00025*(short int)(IMU_raw[6]*256+IMU_raw[7]);//Y Accel
		IMU_data[4]=0.00025*(short int)(IMU_raw[8]*256+IMU_raw[9]);//Z Accel
		IMU_data[5]=0.02*(short int)(IMU_raw[10]*256+IMU_raw[11]);// X Gyro

		//Getting angle from accelerometer
		Roll_Acc=(atan(IMU_data[3]/IMU_data[4])+i*Roll_Acc)/(i+1);
		Pitch_Acc=(-atan(-IMU_data[2]/sqrt(IMU_data[3]*IMU_data[3]+IMU_data[4]*IMU_data[4]))+i*Pitch_Acc)/(i+1);
	}

	double Roll_Fil=Roll_Acc*rad2deg;
	double Pitch_Fil=Pitch_Acc*rad2deg;

	//printf("%0.2f %0.2f\n",Roll_Fil,Pitch_Fil);



	timer->start();

	SerialClose(8);
	int fdlogger = OpenSerial( 8 , 115200, 1, 8, eParityNone );

	while(J1[7]){



		BYTE status = OSSemPend( &PitSem1, TICKS_PER_SECOND * 5 );


		if ( status == OS_NO_ERR ){

			DSPIStart(3,IMU_command,IMU_raw,12,NULL);//start talking to IMU
			while(!DSPIdone(3)){};//wait for DSPI to finish
				IMU_data[0]=0.02*(short int)(IMU_raw[0]*256+IMU_raw[1]);//Z Gyro
				IMU_data[1]=0.00025*(short int)(IMU_raw[2]*256+IMU_raw[3]);//Zero for some reason
				IMU_data[2]=0.00025*(short int)(IMU_raw[4]*256+IMU_raw[5]);//X Accel
				IMU_data[3]=0.00025*(short int)(IMU_raw[6]*256+IMU_raw[7]);//Y Accel
				IMU_data[4]=0.00025*(short int)(IMU_raw[8]*256+IMU_raw[9]);//Z Accel
				IMU_data[5]=0.02*(short int)(IMU_raw[10]*256+IMU_raw[11]);// X Gyro

				//Parsing Pilot Command
				pilot_throttle=(pilot_channel[1]*255-0.251)/0.001024;
				pilot_roll=(pilot_channel[0]*255-0.251)/0.001024;
				pilot_pitch=(pilot_channel[2]*255-0.251)/0.001024;
				pilot_yaw=(pilot_channel[3]*255-0.251)/0.001024;

				//Getting angle from Accelerometer
				Roll_Acc=atan(IMU_data[3]/IMU_data[4])*rad2deg;
				Pitch_Acc=-atan(-IMU_data[2]/sqrt(IMU_data[3]*IMU_data[3]+IMU_data[4]*IMU_data[4]))*rad2deg;


				//Getting angle from Rate Gyros
				double Roll_Gyro=Roll_Fil+0.02*IMU_data[5];
				double Pitch_Gyro=Pitch_Fil+0.02*IMU_data[0];



				//Applying Complementary Filter
				Roll_Fil=0.98*Roll_Gyro+0.02*Roll_Acc;
				Pitch_Fil=0.98*Pitch_Gyro+0.02*Pitch_Acc;



				i=3;
				for (i=3;i<15;i++){
					SD_card[i]=IMU_raw[i-3];
				}

				SD_card[15]=pilot_throttle;
				SD_card[16]=pilot_roll;
				SD_card[17]=pilot_pitch;
				SD_card[18]=pilot_yaw;

				//printf("%f\n",pilot_channel[3]);
				//iprintf("%d %d %d %d\n",(unsigned char)SD_card[15],(unsigned char)SD_card[16],(unsigned char)SD_card[17],(unsigned char)SD_card[18]);

				SD_card[19]=Roll_Fil*deg2rad*150;
				SD_card[20]=Pitch_Fil*deg2rad*150;

				double TotalTime=timer->readTime();
				uint16_t Int_Time=TotalTime*60;
				SD_card[21]=(Int_Time & 0xFF00)>>8;
				SD_card[22]=(Int_Time & 0x00FF);

				SD_card[23]=SD_card[23]+1;



				J2[48]=1;
				if(fp)
				{
					f_write( &SD_card, 1, 24, fp );
					//iprintf("PIT Timer count: %d\n", n );
					//f_close(fp);
				}

				i=0;

				for(i=1;i<24;i++)
					write(fdlogger,&SD_card[i],1);
				J2[48]=0;

				//

		}//If for PIT Sem
	}//Task While LoopaAAAA
	J2[48]=1;
	f_close(fp);
	UnmountFlash(drv);
	f_releaseFS();

}//IMU_Filter_Loop Task

/*****************************UserMain***************************************************/
void UserMain( void* pd ){

	/////Usual Routine
	InitializeStack();
	OSChangePrio( MAIN_PRIO );
	EnableAutoUpdate();
	EnableTaskMonitor();
	EnableSmartTraps();

	initPINS();//Initialize Hardware pins

	//Configuring and starting the timer to read receiver signal
	ppm_timer=HiResTimer::getHiResTimer(1);//timer for ppm
	ppm_timer->init();
	ppm_timer->start();

	DSPIInit(3,2000000,16,0x00,1,1,1,0,0,0);//Initializing the Hardware to talk to IMU
	SetIntc(0,(long)&PPM_read_interrupt,2,1);// Configuring external pins to interrupt and read receiver data
	sim2.eport.epier |= 0x04;// enabling the interrupt

	OSSimpleTaskCreate(IMU_Filter_Loop,MAIN_PRIO+1);//Creating an Autopilot Task

	OSTimeDly(100);
	f_enterFS();

	while (J1[7])
	{
	   OSTimeDly( TICKS_PER_SECOND*30);
	   f_flush(fp);

	}

	f_releaseFS();

}
