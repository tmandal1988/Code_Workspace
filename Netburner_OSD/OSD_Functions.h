/*
 * OSD_Functions.h
 *
 *  Created on: Aug 15, 2014
 *      Author: Tanmay
 */

#include <basictypes.h>


#ifndef OSD_FUNCTIONS_H_
#define OSD_FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

void initOSD(void);

void DisplayCharacter(uint8_t L_Add, uint8_t Character );
void Enable_OSD(void);
void OSD_Position_H(uint8_t H_Add);
void Display_Center(uint16_t x);
void Display_Center_Line(uint16_t x,int8_t roll);
void Replace_Center_Line(int8_t i,int8_t roll);
void Remove_Center_Line(uint16_t x,int8_t roll);
void Replace_Character(uint8_t Char_Add);
void Display_Data(void);
void Display_Roll(int8_t roll);
void Display_Pitch(int8_t pitch);

#ifdef __cplusplus
}
#endif


#endif /* OSD_FUNCTIONS_H_ */
