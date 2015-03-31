/*
 * pinDefinations.h
 *
 *  Created on: Mar 23, 2015
 *      Author: Tanmay
 */

#ifndef PINDEFINATIONS_H_
#define PINDEFINATIONS_H_

#include <basictypes.h>

#define P_INIT 0
#define P_START 0

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

void initPins();
uint16_t configPWM(int module,int freq);
void setPWM(int module,int PWMval,bool upload);
void updatePWM();


#endif /* PINDEFINATIONS_H_ */
