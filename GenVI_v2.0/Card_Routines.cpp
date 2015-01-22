/*
 * Card_Routines.cpp
 *
 *  Created on: Sep 8, 2014
 *      Author: Tanmay
 */

#include "predef.h"
#include <stdio.h>
#include <ctype.h>
#include <basictypes.h>
#include <utils.h>
#include <ucos.h>
#include "FileSystemUtils.h"
#include <effs_fat/fat.h>
#include <effs_fat/multi_drive_mmc_mcf.h>
#include <effs_fat/effs_utils.h>
#include "Card_Routines.h"

int *initOnBoardSD(int drv) {

	static int card_status[2] = { 0, 0 };
	f_enterFS();
	int rv = f_chdrive(drv);

	if (rv == F_NO_ERROR) {
		iprintf("drive change successful\r\n");
		//iprintf("No of Files Found = %d\n",DumpDir());
		card_status[0] = 1;
		card_status[1] = DumpDir();
		return card_status;

	}

	else {
		iprintf("drive change failed: ");
		DisplayEffsErrorCode(rv);
		card_status[0] = -1;
		card_status[1] = 0;
		return card_status;
	}
}
