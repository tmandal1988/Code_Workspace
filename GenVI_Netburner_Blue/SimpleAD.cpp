/******************************************************************************
 * Copyright 2012 NetBurner, Inc.  ALL RIGHTS RESERVED
 *   Permission is hereby granted to purchasers of NetBurner Hardware
 *   to use or modify this computer program for any use as long as the
 *   resultant program is only executed on NetBurner provided hardware.
 *
 *   No other rights to use this program or it's derivitives in part or
 *   in whole are granted.
 *
 *   It may be possible to license this or other NetBurner software for
 *   use on non NetBurner Hardware. Please contact Licensing@Netburner.com
 *   for more information.
 *
 *   NetBurner makes no representation or warranties with respect to the
 *   performance of this computer program, and specifically disclaims any
 *   responsibility for any damages, special or consequential, connected
 *   with the use of this program.
 *
 *   NetBurner, Inc
 *   5405 Morehouse Drive
 *   San Diego Ca, 92121
 *   http://www.netburner.com
 *
 *****************************************************************************/
#include <basictypes.h>
#include <sim.h>


void InitSingleEndAD()
{
    //See MCF5441X RM Chapter 29
    sim2.adc.cr1 = 0;      // Set up control register1
    sim2.adc.cr2 = 0;      // Set up control register2
    sim2.adc.zccr = 0;     // Disable ZC
    sim2.adc.lst1 = 0x3210;// Set samples 0-3 to ADC_IN0-3
    sim2.adc.lst2 = 0x7654;// Set samples 4-7 to ADC_IN4-7
    sim2.adc.sdis = 0;     // Enable all samples
    sim2.adc.sr = 0xFFFF;  // Clear Status Register
    for (int i = 0; i < 8; i++) { // Clear result + offset registers
        sim2.adc.rslt[i] = 0;
        sim2.adc.ofs[i] = 0;
    }
    sim2.adc.lsr = 0xFFFF; // Clear limit register
    sim2.adc.zcsr = 0xFFFF;// Clear ZC

    sim2.adc.pwr = 0;      // Everything is turned on
    sim2.adc.cal = 0;      // User internal calibration
    sim2.adc.pwr2 = 0x0005;// Default power converion
    sim2.adc.div = 0x505;  // Set default
    sim2.adc.asdiv = 0x13; // ASDivisor set
}

void StartAD()
{
   sim2.adc.sr = 0xffff;   // Clear status codes
   sim2.adc.cr1 |= 0x2000; // Starting A2D
}

bool ADDone()
{
   if (sim2.adc.sr & 0x0800)
      return true;
   else
      return false;
}

WORD GetADResult(int ch) //Get the AD Result
{
   return sim2.adc.rslt[ch];
}

