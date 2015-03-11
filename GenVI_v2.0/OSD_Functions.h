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

/*************Tanmay Coordinates*********************
 * A=Tanmay Address
 * P=A / 30 (just the quotient)
 * C=A % 30
 * R= P / 17 (just the quotient)
 * b=P % 17 (this the the row of the pixel you want to display from datasheet each display pixel has 18 rows numbered from 0-17
 * so this b represents which row in a display pixel you want to show) this display pixels were custom created from 0x50 character memory address
 *
 * Then Actual Display Address Ad=30*R+29+(C+1)
 * Since in AH(Artificial Horizon) the column number is constant therefore C here is 14 but the formula above is for any general address
 */

void DisplayCharacter(uint8_t L_Add, uint8_t Character );//function to display a character, argument Lower Byte of the OSD Display address and the character memory address
void Enable_OSD(void);//Enables OSD Display
void OSD_Position_H(uint8_t H_Add);//Decides the high bit of OSD display address
void Display_Center(uint16_t x);//Displays the center circle, center address in Tanmay Coordinates
void Display_Center_Line(uint16_t x,int8_t roll);//Displays the Center line or Horizon,Address in Tanmay Coordinates and roll value
void Replace_Center_Line(int8_t i,int8_t roll);//Removes and redraws the center line, picth and roll input
void Remove_Center_Line(uint16_t x,int8_t roll);//Removes the center line, address in Tanmay Coordinates and roll angle
void Replace_Character(uint8_t Char_Add);//Replaces a character
void Display_Data(void);//Displays the Text Data
void Display_Roll(int8_t roll);//Displays Roll
void Display_Pitch(int8_t pitch);//Displays Pitch

#ifdef __cplusplus
}
#endif


#endif /* OSD_FUNCTIONS_H_ */
