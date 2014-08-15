/*
 * OSD_Functions.cpp
 *
 *  Created on: Aug 15, 2014
 *      Author: Tanmay
 */


//included
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
#include "OSD_Functions.h"

void initOSD(){

	//BYTE OSD_Enable[2]={0x00,0x08};
	BYTE OSD_DIN[8]={0};
	BYTE OSD_DOUT[2]={0};
	//BYTE OSD_Disable[2]={0x00,0x00};

	OSD_DOUT[0]=0xEC;OSD_DOUT[1]=0x00;
	DSPIStart(1,OSD_DOUT,OSD_DIN,2,NULL);//Get Image Black Level
	while(!DSPIdone(1)){};//wait for DSPI to finish

	//OSTimeDly(5);

	OSD_DOUT[0]=0x6C;OSD_DOUT[1] =OSD_DIN[1] & 0xEF;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Set OSD Black level to Image Black level
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSTimeDly(5);

	OSD_DOUT[0]=0x04;OSD_DOUT[1] =0x00;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMM set to 0
	while(!DSPIdone(1)){};//wait for DSPI to finish

	/*******************replacing 0x0C charcter************************/
	//ReplaceCharacter(0xC0);//Center Circle
	//ReplaceCharacter(0xC1);//Vertical Line
	//ReplaceCharacter(0xC2);//Horizontal Line

}

void  DisplayCharacter(uint8_t L_Add, uint8_t Character ){

	//BYTE OSD_Enable[2]={0x00,0x08};
	//BYTE OSD_DIN[8]={0};
	BYTE OSD_DOUT[2]={0};
	//BYTE OSD_Disable[2]={0x00,0x00};

	OSD_DOUT[0]=0x06;OSD_DOUT[1] =L_Add;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish


	OSD_DOUT[0]=0x07;OSD_DOUT[1] =Character;//o
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish*/


}

void Enable_OSD(){

	BYTE OSD_Enable[2]={0x00,0x08};
	DSPIStart(1,OSD_Enable,NULL,2,NULL);//Enable display of OSD image
	while(!DSPIdone(1)){};//wait for DSPI to finish
}

void OSD_Position_H(uint8_t H_Add){

	BYTE OSD_DOUT[2]={0};

	OSD_DOUT[0]=0x05;OSD_DOUT[1] =H_Add; //Charcter position High Address
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAH
	while(!DSPIdone(1)){};//wait for DSPI to finish
}

void Display_Center(uint16_t x){

	if(x >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x,0xC0);

	if((x-1) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-1,0xC2);

	if((x+1) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+1,0xC2);
}

void Display_Center_Line(uint16_t x){

	if((x+7)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+7,0xC2);

	if((x+8)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+8,0xC2);

	if((x+9)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+9,0xC2);

	if((x+10)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+10,0xC2);

	if((x+11)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);

	DisplayCharacter(x+11,0xC2);


	if((x-7) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-7,0xC2);

	if((x-8) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-8,0xC2);

	if((x-9) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-9,0xC2);

	if((x-10) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-10,0xC2);

	if((x-11) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-11,0xC2);
}

void Display_Top_Line(uint16_t x){

	if((x+7-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+7-90,0xC2);

	if((x+8-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+8-90,0x00);

	if((x+9-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+9-90,0xC2);

	if((x+10-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+10-90,0x00);

	if((x+11-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+11-90,0xC2);

	if((x-7-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-7-90,0xC2);

	if((x-8-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-8-90,0x00);

	if((x-9-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-9-90,0xC2);

	if((x-10-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-10-90,0x00);

	if((x-11-90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-11-90,0xC2);
}

void Display_Bottom_Line(uint16_t x){

	if((x+7+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+7+90,0xC2);

	if((x+7+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+8+90,0x00);

	if((x+7+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+9+90,0xC2);

	if((x+7+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+10+90,0x00);

	if((x+7+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x+11+90,0xC2);

	if((x-7+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-7+90,0xC2);

	if((x-8+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-8+90,0x00);

	if((x-9+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-9+90,0xC2);

	if((x-10+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-10+90,0x00);

	if((x-11+90)>=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(x-11+90,0xC2);
}

void Replace_Center_Line(int8_t i){
	static int y=194;
	uint8_t x=194+i*30;

	//Remove_Center_Line(y);
	//Display_Center_Line(x);

	y=x;
}

void Replace_Top_Line(int8_t i){
	static int y=194;
}

void Replace_Bottom_Line(int8_t i){
	static int y=194;
}
