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

	BYTE OSD_DIN[8]={0};
	BYTE OSD_DOUT[2]={0};

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

}

void  DisplayCharacter(uint8_t L_Add, uint8_t Character ){

	BYTE OSD_DOUT[2]={0};

	OSD_DOUT[0]=0x06;OSD_DOUT[1] =L_Add;//Lower byte of the diplay address
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish


	OSD_DOUT[0]=0x07;OSD_DOUT[1] =Character;//Character to Display
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//
	while(!DSPIdone(1)){};//wait for DSPI to finish*/


}

void Enable_OSD(){

	BYTE OSD_Enable[2]={0x00,0x08};
	DSPIStart(1,OSD_Enable,NULL,2,NULL);//Enable display of OSD image
	while(!DSPIdone(1)){};//wait for DSPI to finish
}

void OSD_Position_H(uint8_t H_Add){

	BYTE OSD_DOUT[2]={0};

	OSD_DOUT[0]=0x05;OSD_DOUT[1] =H_Add; //high bit of OSD display address
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAH
	while(!DSPIdone(1)){};//wait for DSPI to finish
}

void Display_Center(uint16_t x){

	uint8_t pixel_row=0,column_no=0,bar_no=0,actual_row=0;;


	//computing actual address from Tanmay Coordinates
	pixel_row=x / 30;
	column_no=x % 30;
	actual_row=pixel_row / 17;
	bar_no= pixel_row % 17;

	uint16_t Disp_Add=30*actual_row+29+(column_no+1);//Actual address
	uint8_t Disp_Char=0xC2;

	if(Disp_Add >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(Disp_Add,0xC0);//Display the center O

	if((Disp_Add-1) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(Disp_Add-1,Disp_Char);//Display the left wing

	if((Disp_Add+1) >=255)
		OSD_Position_H(0x01);
	else
		OSD_Position_H(0x00);
	DisplayCharacter(Disp_Add+1,Disp_Char);//Display the right wing
}

void Display_Center_Line(uint16_t x,int8_t roll){//Display Center Line


	float froll=roll*0.1;
	int16_t pixel_row=0,column_no=0,bar_no=0,actual_row=0;
	uint16_t Disp_Add=0;
	uint8_t Disp_Char=0;


	for (int i=-9;i<10;i++){//for loop to Display 19 lines to draw center Line

		pixel_row=(x-i*30*froll) / 30;//Computing actuall address from Tanmay Address
		if(pixel_row>-1){//making sure we don't have negative display address
			column_no=14;
			actual_row=pixel_row / 17;
			bar_no= pixel_row % 17;

			Disp_Add=30*actual_row+29+(column_no+1);
			Disp_Char=128+bar_no;


			if(Disp_Add< 480){//making sure that we don't have any address above 480
				if ((Disp_Add+i)>255)
					OSD_Position_H(0x01);
				else
					OSD_Position_H(0x00);
				DisplayCharacter(Disp_Add+i,Disp_Char);//////////////////Center Line
			}


		}
	}

}

void Remove_Center_Line(uint16_t x,int8_t roll){//Remove Center Line //See Display Center Line for more info.


	float froll=roll*0.1;
	int16_t pixel_row=0,column_no=0,bar_no=0,actual_row=0;
	uint16_t Disp_Add=0;
	uint8_t Disp_Char=0;


	for (int i=-9;i<10;i++){

		pixel_row=(x-i*30*froll) / 30;
		if(pixel_row>-1){
			column_no=14;
			actual_row=pixel_row / 17;
			bar_no= pixel_row % 17;

			Disp_Add=30*actual_row+29+(column_no+1);
			Disp_Char=128+bar_no;


			if(Disp_Add< 480){
				if ((Disp_Add+i)>255)
					OSD_Position_H(0x01);
				else
					OSD_Position_H(0x00);
				DisplayCharacter(Disp_Add+i,0x00);//Blank Character

			}


		}
	}
}


void Replace_Center_Line(int8_t i, int8_t j){//Calls Remove and Display Center Line function
	static int y_pitch=2834;
	static int y_roll=0;

	uint16_t x=2834+i*30;

	Remove_Center_Line(y_pitch,y_roll);//removes the old line
	Display_Center_Line(x,j);//displays the new line
	Display_Center(2834);//just to be sure the center is always pressent

	y_pitch=x;
	y_roll=j;

}

void Display_Data(void){//Setting up text display

	OSD_Position_H(0x00);
	DisplayCharacter(25,0x1A);
	DisplayCharacter(26,0x44);

	DisplayCharacter(55,0x1C);
	DisplayCharacter(56,0x44);
}

void Display_Roll(int8_t roll){//Displaying the roll data

	int f_char=0;
	int s_char=0;

	if(roll<0)//Deciding which sign to display
		DisplayCharacter(57,0x49);
	if(roll>=0)
		DisplayCharacter(57,0x50);

	f_char=abs(roll / 10);//Showing just the integer value
	//f_char---->First Character
	if(f_char!=0)//check for 0
		DisplayCharacter(58,f_char);
	else
		DisplayCharacter(58,0x0A);

	s_char=abs(roll % 10);
	//s_char----->Second Character
	if(s_char!=0)//Check for 0
		DisplayCharacter(59,s_char);
	else
		DisplayCharacter(59,0x0A);

}
void Display_Pitch(int8_t pitch){//Same as Roll but with Pitch angle

	int f_char=0;
	int s_char=0;

	if(pitch<0)
		DisplayCharacter(27,0x49);
	if(pitch>=0)
		DisplayCharacter(27,0x50);

	f_char=abs(pitch / 10);
	if(f_char!=0)
		DisplayCharacter(28,f_char);
	else
		DisplayCharacter(28,0x0A);

	s_char=abs(pitch % 10);
	if(s_char!=0)
		DisplayCharacter(29,s_char);
	else
		DisplayCharacter(29,0x0A);
}



