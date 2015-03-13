/*
 * PINS_Definations.cpp
 *
 *  Created on: Aug 13, 2014
 *      Author: Tanmay
 */

#include "PINS_Definations.h"
#include <basictypes.h>
#include <sim.h>
#include <pins.h>
#include <serial.h>



void initPINS()
{

	//For Blue LED
	J2[48].function(0);//led D2,GPIO
	J2[48]=0;

	//J1[7].function(0);//User defined Switch

	//For IMU
	J2[21].function(1);//SPI3 Input
	J2[22].function(1);//SPI3 Out
	J2[23].function(1);//SPI3 chip select 0
	J2[24].function(1);//SPI3 clock

	//For OSD
	J2[25].function(1);//SPI1 SCK
	J2[27].function(1);//SPI1 DIN
	J2[28].function(1);//SPI1 DOUT
	J2[30].function(1);// SPI1 CS0

	J2[32].function(0);//OSD Reset
	J2[32]=0;

	//for reading GPS
	J2[39].function(2);//UART8 RX
	J2[42].function(2);//UART8 TX

	//for reading spektrum receiver
	J2[31].function(3);//UART2 RX


	//For PWM
	//AND and Multiplexer pins
	J2[15].function(0);//CH7 Enable
	J2[17].function(0);//CH1 Enable
	J2[18].function(0);//CH2 Enable
	J2[37].function(0);//CH3 Enable
	//J2[43].function(0);//CH4 Enable
	J2[45].function(0);//CH5 Enable
	J2[47].function(0);//CH6 Enable
	J1[5].function(0);//CH8 Enable
	J1[13].function(0);//AP_OUT


	J2[15]=0;J2[17]=0;J2[18]=1;J2[37]=0;J1[5]=0;J1[13]=0;//J2[43]=1;///Running everything on Autopilot.
	J2[45]=0;J2[47]=0;

}

uint16_t configPWM(int module,int freq){
	int module_no=999,pwm_num=999;
	uint16_t pwmr=0;

	//PWM Constants
	static uint16_t CR2= 0xE000; //Debug enabled, Wait enabled, output A&B are independent,
	//#define CR1 0x0444 //400 Hz Full cycle reloaded enabled, PWN clock fclk/16, Load mode selected immediatly after LDOK being set.
	//#define CR1 0x0474 //50 Hz
	static uint16_t CR1=0x0404; //Vanilla CR1
	static uint16_t OCR=0x0000; //10-8: output of A&B&x not inverted, 5-4 3-2 1-0: output of A&B&x forced to 0 prior to output polarity control
	static uint16_t DISMAP=0x0000; //Fault disable mapping register. PWM fault pin has no effect on x B and A
	static uint16_t OUTEN=0x0000;  //Output enable register. 11-8 7-4 3-0: A B enabled, x not enabled.
  //Master control register. 15-12: ingored by independent mode; 3-0 Load OK,
	static uint16_t P_RUN=0x0000 ; //PWM generator enabled.


	if(module==10){
		module_no=0;
		pwm_num=3;
		J2[25].function(2);//PWMA0
		OUTEN=OUTEN | 0x0100;
		LDOK=LDOK | 0x0001;
		P_RUN=P_RUN | 0x0100;
	}

	if(module==20){
		module_no=0;
		pwm_num=5;
		J2[27].function(2);//PWMB0;
		OUTEN=OUTEN | 0x0010;
		LDOK=LDOK | 0x0001;
		P_RUN=P_RUN | 0x0100;
	}

	if(module==11){
		module_no=1;
		pwm_num=3;
		J2[30].function(2);//PWMA1
		OUTEN=OUTEN | 0x0200;
		LDOK=LDOK | 0x0002;
		P_RUN=P_RUN | 0x0200;
	}

	if(module==21){
		module_no=1;
		pwm_num=5;
		J2[40].function(2);//PWMB1
		OUTEN=OUTEN | 0x0020;
		LDOK=LDOK | 0x0002;
		P_RUN=P_RUN | 0x0200;
	}

	if(module==12){
		module_no=2;
		pwm_num=3;
		J2[35].function(2);//PWMA2
		OUTEN=OUTEN | 0x0400;
		LDOK=LDOK | 0x0004;
		P_RUN=P_RUN | 0x0400;
	}

	if(module==22){
		module_no=2;
		pwm_num=5;
		J2[28].function(2);//PWMB2
		OUTEN=OUTEN | 0x0040;
		LDOK=LDOK | 0x0004;
		P_RUN=P_RUN | 0x0400;
	}

	if(module==13){
		module_no=3;
		pwm_num=3;
		J2[31].function(2);//PWMA3
		OUTEN=OUTEN | 0x0800;
		LDOK=LDOK | 0x0008;
		P_RUN=P_RUN | 0x0800;
	}

	if(module==23){
		module_no=3;
		pwm_num=5;
		J2[19].function(2);//PWMB3
		OUTEN=OUTEN | 0x0080;
		LDOK=LDOK | 0x0008;
		P_RUN=P_RUN | 0x0800;
	}

/*	int freq_divider=125000000L/(P_MAX*freq);
	float fPWM_scalar=log(freq_divider)/log(2);
	int iPWM_scalar=(int)fPWM_scalar;

	if((fPWM_scalar-iPWM_scalar)>=0.5)
		iPWM_scalar=iPWM_scalar+1;*/

	int freq_divider=1;

	if(freq<=59){
		CR1=CR1|(7<<4);
		freq_divider=128;

	}
	else if(freq>59 && freq<=100){
		CR1=CR1|(6<<4);
		freq_divider=64;
	}
	else if(freq>100 && freq<=200){
		CR1=CR1|(5<<4);
		freq_divider=32;
	}
	else{
		CR1=CR1|(4<<4);
		freq_divider=16;
	}

	uint16_t PWM_lim=125000000L/(freq*freq_divider)+1;
	//printf("%d\n",PWM_lim);

	//PWM Registers
	sim1.mcpwm.sm[module_no].cr2=CR2;
	sim1.mcpwm.sm[module_no].cr1=CR1;
	sim1.mcpwm.sm[module_no].ocr=OCR;
	sim1.mcpwm.sm[module_no].dismap=DISMAP;
	//Edge Aligned PWM
	sim1.mcpwm.sm[module_no].init=P_INIT;
	sim1.mcpwm.sm[module_no].val[1]=PWM_lim;
	sim1.mcpwm.sm[module_no].val[pwm_num-1]=P_START;
	sim1.mcpwm.sm[module_no].val[pwm_num]=PWM_lim*freq*0.0009;

	//enabling PWM Outputs
	sim1.mcpwm.outen=OUTEN;
	//starting PWM
	pwmr=sim1.mcpwm.mcr;
	sim1.mcpwm.mcr=LDOK;
	sim1.mcpwm.mcr |=P_RUN;
	return PWM_lim;

}


void setPWM(int module,int PWMval){

	int module_no=999,pwm_num=999;
	uint16_t pwmr=0;

	if(module==10){
		module_no=0;
		pwm_num=3;

	}

	if(module==20){
		module_no=0;
		pwm_num=5;

	}

	if(module==11){
		module_no=1;
		pwm_num=3;

	}

	if(module==21){
		module_no=1;
		pwm_num=5;

	}

	if(module==12){
		module_no=2;
		pwm_num=3;

	}

	if(module==22){
		module_no=2;
		pwm_num=5;

	}

	if(module==13){
		module_no=3;
		pwm_num=3;

	}

	if(module==23){
		module_no=3;
		pwm_num=5;

	}

	sim1.mcpwm.sm[module_no].val[pwm_num]=PWMval;
	pwmr = sim1.mcpwm.mcr;
	sim1.mcpwm.mcr |= LDOK;

}



