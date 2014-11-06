/*
 * Accessory_Functions.h
 *
 *  Created on: Oct 28, 2014
 *      Author: Tanmay
 */

#include <ucos.h>
#include "FileSystemUtils.h"
#include "Card_Routines.h"
#include <ucos.h>
#include <stdio.h>
#include <effs_fat/fat.h>
#include <effs_fat/multi_drive_mmc_mcf.h>
#include <effs_fat/effs_utils.h>

#ifndef ACCESSORY_FUNCTIONS_H_
#define ACCESSORY_FUNCTIONS_H_

void Init_100Hz_Control(OS_SEM PitSem1);
int Init_Logfile(F_FILE** fp);
double* Get_Adc_Values();



#endif /* ACCESSORY_FUNCTIONS_H_ */
