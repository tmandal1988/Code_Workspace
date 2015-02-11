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


#define P_END 7600 //400 Hz 7812/19531=0.3999 1000 usec
#define P_FACTOR 7812500 //125 MHz/16---->19531=400 Hz PWM  ?Purpose unknown
#define T_LIMIT 7800 //400 Hz  purpose unknown
#define P_INIT 0
#define P_MAX 19531//400 Hz
#define P_MAX2 15625 //400 Hz 80% 2000 usec
#define P_START 0

#define rad2deg 57.296f
#define deg2rad 0.017f


#ifdef __cplusplus
extern "C"
{
#endif

void initPINS();
uint16_t configPWM(int module,int freq);
void setPWM(int module,int PWMval);



#ifdef __cplusplus
}
#endif



#endif /* PINS_DEFINATIONS_H_ */
