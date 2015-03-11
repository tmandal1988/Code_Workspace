/*
 * PINS_Definations.h
 *
 *  Created on: Oct 27, 2014
 *      Author: Tanmay
 */

/*
 * PINS_Definations.h
 *
 *  Created on: Jun 26, 2014
 *      Author: Tanmay, Caleb
 */

#ifndef PINS_DEFINATIONS_H_
#define PINS_DEFINATIONS_H_

#include <basictypes.h>


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


#define rad2deg 57.296f
#define deg2rad 0.017f


#ifdef __cplusplus
extern "C"
{
#endif

void initPINS();



#ifdef __cplusplus
}
#endif



#endif /* PINS_DEFINATIONS_H_ */
