/*
 * ReplaceCharacter.cpp
 *
 *  Created on: Aug 18, 2014
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
#include "ReplaceCharacter.h"


void Replace_Character(uint8_t Char_Add){

	BYTE OSD_Disable[2]={0x00,0x00};
	BYTE OSD_DOUT[2]={0};

	DSPIStart(1,OSD_Disable,NULL,2,NULL);//Disable display of OSD image
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x09;OSD_DOUT[1] =Char_Add;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//DMAL
	while(!DSPIdone(1)){};//wait for DSPI to finish

	//Send 54 Bytes////////////////
	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =0;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =1;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =2;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =3;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =4;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =5;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =6;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =7;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =8;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =9;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =10;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =11;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =12;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =13;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =14;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =15;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =16;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =17;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =18;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =19;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =20;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =21;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =22;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =23;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =24;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =234;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =25;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =170;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =26;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =175;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =27;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =28;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =29;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =30;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =31;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =32;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =33;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =34;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =35;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =36;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =37;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =38;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =39;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =40;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =239;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =41;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =42;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =43;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =44;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =45;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =46;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =47;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =48;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =49;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =50;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =51;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =52;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0A;OSD_DOUT[1] =53;//Character to replace
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSD_DOUT[0]=0x0B;OSD_DOUT[1] =255;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Send Data
	while(!DSPIdone(1)){};//wait for DSPI to finish

	/////////////////

	OSD_DOUT[0]=0x08;OSD_DOUT[1] =0xA0;
	DSPIStart(1,OSD_DOUT,NULL,2,NULL);//Copy the character to NON Volatile Memory
	while(!DSPIdone(1)){};//wait for DSPI to finish

	OSTimeDly(5);

	Enable_OSD();
}
