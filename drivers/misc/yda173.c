 /*
 * D-4SS1 control module
 *
 * Copyright(C)   2010-2011 Yamaha Corporation
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.


 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
/****************************************************************************
 *
 *
 *		Copyright (c) 2011 Yamaha Corporation
 *
 *		Module		: D4SS1_Ctrl.c
 *
 *		Description	: D-4SS1 control module
 *
 *		Version	:	1.0.0	2011.11.24
 *
 ****************************************************************************/
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <mach/vreg.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/i2c/yda173.h>
#include <linux/i2c/yda173_def.h>
#include <linux/syscalls.h>
#include <linux/module.h>

#define MODULE_NAME "yda173"
#define AMPREG_DEBUG 0
#define MODE_NUM_MAX 30

static struct yda173_i2c_data g_data;

static struct i2c_client *pclient;
static struct snd_set_ampgain g_ampgain[MODE_NUM_MAX];
static struct snd_set_ampgain temp;
static int load_ampgain(void)
{
	struct yda173_i2c_data *yd ;
	int numofmodes;
	int index = 0;
	yd = &g_data;
	numofmodes = yd->num_modes;
	for (index = 0; index < numofmodes; index++) {
		memcpy(&g_ampgain[index], &yd->ampgain[index],
		sizeof(struct snd_set_ampgain));
#if AMPREG_DEBUG
		pr_info(MODULE_NAME ":[%d].in1_gain	=	%d\n",
		index, g_ampgain[index].in1_gain);
		pr_info(MODULE_NAME ":[%d].in2_gain	=	%d\n",
		index, g_ampgain[index].in2_gain);
		pr_info(MODULE_NAME ":[%d].hp_att	=	%d\n",
		index, g_ampgain[index].hp_att);
		pr_info(MODULE_NAME ":[%d].sp_att	=	%d\n",
		index, g_ampgain[index].sp_att);
#endif
	}
	memcpy(&temp,  &g_ampgain[0],  sizeof(struct snd_set_ampgain));
#if AMPREG_DEBUG
	pr_info(MODULE_NAME ":temp.in1_gain	=	%d\n", temp.in1_gain);
	pr_info(MODULE_NAME ":temp.in2_gain	=	%d\n", temp.in2_gain);
	pr_info(MODULE_NAME ":temp.hp_att	= %d\n", temp.hp_att);
	pr_info(MODULE_NAME ":temp.sp_att	= %d\n", temp.sp_att);
#endif

	pr_info(MODULE_NAME ":%s completed\n", __func__);
	return 0;
}

/* D-4SS1 register map */
UINT8 g_bD4SS1RegisterMap[9] = {
	0x84,/* 0x80 : SRST(7)=1,  CT_CAN(6-5)=0,  PPDOFF(4)=0,
	MODESEL(3)=0,  CPMODE(2)=1,  VLEVEL(0)=0 */
	0x1C,/* 0x81 : DRC_MODE(7-6)=0,  DATRT(5-4)=1,
	NG_ATRT(3-2)=3 */
	0x10,/* 0x82 : DPLT(7-5)=0,  HP_NG_RAT(4-2)=4,
	HP_NG_ATT(1-0)=0 */
	0x10,/* 0x83 : DPLT_EX(6)=0,  NCLIP(5)=0,
	SP_NG_RAT(4-2)=4,  SP_NG_ATT(1-0)=0 */
	0x22,/* 0x84 : VA(7-4)=2,  VB(3-0)=2 */
	0x04,/* 0x85 : DIFA(7)=0,  DIFB(6)=0,  HP_SVOFF(3)=0,
	HP_HIZ(2)=1,  SP_SVOFF(1)=0,  SP_HIZ(0)=0 */
	0x00,/* 0x86 : VOLMODE(7)=0,  SP_ATT(6-0)=0 */
	0x00,/* 0x87 : HP_ATT(6-0)=0 */
	0x00/* 0x88 : OCP_ERR(7)=0,  OTP_ERR(6)=0,  DC_ERR(5)=0,
	HP_MONO(4)=0,  HP_AMIX(3)=0,  HP_BMIX(2)=0,  SP_AMIX(1)=0,
	SP_BMIX(0)=0 */
};

/* AMP Attenuator */
UINT8 g_bHPAmpAtt;
UINT8 g_bSPAmpAtt;
#if 1
/*******************************************************************************
 *	d4Write
 *
 *	Function:
 *			write register parameter function
 *	Argument:
 *			UINT8 bWriteRA  : register address
 *			UINT8 bWritePrm : register parameter
 *
 *	Return:
 *			SINT32	>= 0 success
 *					<  0 error
 *
 ******************************************************************************/
SINT32 d4Write(UINT8 bWriteRA,  UINT8 bWritePrm)
{
	/* Write 1 byte data to the register for each system. */

	int rc = 0;
	if (pclient == NULL) {
		pr_err(MODULE_NAME ": i2c_client error\n");
		return -ENOMEM;
	}
	rc = i2c_smbus_write_byte_data(pclient,  bWriteRA,  bWritePrm);
	if (rc) {
		pr_err(MODULE_NAME ": i2c write error %d\n",  rc);
		return rc;
	}

	return 0;
}

/*******************************************************************************
 *	d4WriteN
 *
 *	Function:
 *			write register parameter function
 *	Argument:
 *			UINT8 bWriteRA    : register address
 *			UINT8 *pbWritePrm : register parameter
 *			UINT8 bWriteSize  : size of "*pbWritePrm"
 *
 *	Return:
 *			SINT32	>= 0 success
 *					<  0 error
 *
 ******************************************************************************/
SINT32 d4WriteN(UINT8 bWriteRA,  UINT8 *pbWritePrm,  UINT8 bWriteSize)
{
	/* Write N byte data to the register for each system. */

	int rc = 0;

	if (pclient == NULL) {
		pr_err(MODULE_NAME ": i2c_client error\n");
		return -ENOMEM;
	}

	rc = i2c_smbus_write_block_data(pclient,
	bWriteRA, bWriteSize, pbWritePrm);
	if (rc) {
		pr_err(MODULE_NAME ": i2c write error %d\n",  rc);
		return rc;
	}

	return 0;

}

/*******************************************************************************
 *	d4Read
 *
 *	Function:
 *			read register parameter function
 *	Argument:
 *			UINT8 bReadRA    : register address
 *			UINT8 *pbReadPrm : register parameter
 *
 *	Return:
 *			SINT32	>= 0 success
 *					<  0 error
 *
 ******************************************************************************/
SINT32 d4Read(UINT8 bReadRA,  UINT8 *pbReadPrm)
{
	/* Read byte data to the register for each system. */

	int buf;

	if (pclient == NULL) {
		pr_err(MODULE_NAME ": i2c_client error\n");
		return -ENOMEM;
	}

	buf  = i2c_smbus_read_byte_data(pclient,  bReadRA);
	if (buf < 0) {
		pr_err(MODULE_NAME ": i2c read error %d\n",  buf);
		return buf;
	}

	*pbReadPrm = (UINT8)buf;

	return 0;
}

/*******************************************************************************
 *	d4Wait
 *
 *	Function:
 *			wait function
 *	Argument:
 *			UINT32 dTime : wait time [ micro second ]
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void d4Wait(UINT32 dTime)
{
	/* Wait procedure for each system */
	udelay(dTime);
}

/*******************************************************************************
 *	d4Sleep
 *
 *	Function:
 *			sleep function
 *	Argument:
 *			UINT32 dTime : sleep time [ milli second ]
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void d4Sleep(UINT32 dTime)
{
	/* Sleep procedure for each system */

	msleep(dTime);
}

/*******************************************************************************
 *	d4ErrHandler
 *
 *	Function:
 *			error handler function
 *	Argument:
 *			SINT32 dError : error code
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void d4ErrHandler(SINT32 dError)
{
	/* Error procedure for each system */

	pr_err(MODULE_NAME ": %s error %ld\n",  __func__,  dError);
}
#endif
/*******************************************************************************
 *	D4SS1_UpdateRegisterMap
 *
 *	Function:
 *			update register map (g_bD4SS1RegisterMap[]) function
 *	Argument:
 *			SINT32	sdRetVal	: update flag
 *			UINT8	bRN	: register number
 *			UINT8	bPrm	: register parameter
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
static void D4SS1_UpdateRegisterMap(SINT32 sdRetVal,  UINT8 bRN,  UINT8 bPrm)
{
	if (sdRetVal < 0) {
		d4ErrHandler(D4SS1_ERROR);
	} else {
		/* update register map */
		g_bD4SS1RegisterMap[bRN] = bPrm;
	}
}

/*******************************************************************************
 *	D4SS1_UpdateRegisterMapN
 *
 *	Function:
 *			update register map (g_bD4SS1RegisterMap[]) function
 *	Argument:
 *			SINT32	sdRetVal	: update flag
 *			UINT8	bRN			: register number(0 - 8)
 *			UINT8	*pbPrm		: register parameter
 *			UINT8	bPrmSize	: size of " *pbPrm"
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
static void D4SS1_UpdateRegisterMapN(SINT32 sdRetVal, UINT8 bRN,
UINT8 *pbPrm, UINT8 bPrmSize)
{
	UINT8 bCnt = 0;

	if (sdRetVal < 0) {
		d4ErrHandler(D4SS1_ERROR);
	} else {
		/* update register map */
		for (bCnt = 0; bCnt < bPrmSize; bCnt++) {
			/*Update Register*/
			g_bD4SS1RegisterMap[bRN + bCnt] = pbPrm[bCnt];
		}
	}
}

/*******************************************************************************
 *	D4SS1_WriteRegisterBit
 *
 *	Function:
 *			write register "bit" function
 *	Argument:
 *			UINT32	dName	: register name
 *			UINT8	bPrm	: register parameter
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_WriteRegisterBit(UINT32 dName,  UINT8 bPrm)
{
	UINT8 bWritePrm;			/* I2C sending parameter */
	UINT8 bDummy;				/* setting parameter */
	UINT8 bRA,  bRN,  bMB,  bSB;

	/*
	dName
	bit 31 - 16 : register address
	bit 15 -  8	: mask bit
	bit  7 -  0	: shift bit
	*/
	bRA = (UINT8)((dName & 0xFF0000) >> 16);
	bRN = bRA - 0x80;
	bMB = (UINT8)((dName & 0x00FF00) >> 8);
	bSB = (UINT8)(dName & 0x0000FF);

	/* check arguments */
	if (bRA < D4SS1_MIN_REGISTERADDRESS) {
		if (D4SS1_MAX_WRITE_REGISTERADDRESS < bRA)
			d4ErrHandler(D4SS1_ERROR_ARGUMENT);
	}
	/* set register parameter */
	bPrm = (bPrm << bSB) & bMB;
	bDummy = bMB ^ 0xFF;
	bWritePrm = g_bD4SS1RegisterMap[bRN] & bDummy;
	bWritePrm = bWritePrm | bPrm;
	D4SS1_UpdateRegisterMap(d4Write(bRA,  bWritePrm),  bRN,  bWritePrm);
}

/*******************************************************************************
 *	D4SS1_WriteRegisterByte
 *
 *	Function:
 *			write register "byte" function
 *	Argument:
 *			UINT8 bAddress  : register address
 *			UINT8 bPrm : register parameter
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_WriteRegisterByte(UINT8 bAddress,  UINT8 bPrm)
{
	UINT8 bNumber;
	/* check arguments */
	if (bAddress < D4SS1_MIN_REGISTERADDRESS) {
		if (D4SS1_MAX_WRITE_REGISTERADDRESS < bAddress)
			d4ErrHandler(D4SS1_ERROR_ARGUMENT);
	}
	bNumber = bAddress - 0x80;
	D4SS1_UpdateRegisterMap(d4Write(bAddress,  bPrm),  bNumber,  bPrm);
}

/*******************************************************************************
 *	D4SS1_WriteRegisterByteN
 *
 *	Function:
 *			write register "n byte" function
 *	Argument:
 *			UINT8 bAddress	: register address
 *			UINT8 *pbPrm	: register parameter
 *			UINT8 bPrmSize	: size of "*pbPrm"
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_WriteRegisterByteN(UINT8 bAddress,  UINT8 *pbPrm,  UINT8 bPrmSize)
{
	UINT8 bNumber;
	UINT8 bNum_Reg;
	/* check arguments */
	bNum_Reg = D4SS1_MAX_WRITE_REGISTERADDRESS - D4SS1_MIN_REGISTERADDRESS;
	if (bAddress < D4SS1_MIN_REGISTERADDRESS) {
		if (D4SS1_MAX_WRITE_REGISTERADDRESS < bAddress)
			d4ErrHandler(D4SS1_ERROR_ARGUMENT);
	}
	if (bPrmSize > (bNum_Reg + 1))
		d4ErrHandler(D4SS1_ERROR_ARGUMENT);

	bNumber = bAddress - 0x80;
	D4SS1_UpdateRegisterMapN(d4WriteN(bAddress, pbPrm, bPrmSize),
	bNumber, pbPrm, bPrmSize);
}

/*******************************************************************************
 *	D4SS1_ReadRegisterBit
 *
 *	Function:
 *			read register "bit" function
 *	Argument:
 *			UINT32	dName	: register name
 *			UINT8	*pbPrm	: register parameter
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_ReadRegisterBit(UINT32 dName,  UINT8 *pbPrm)
{
	SINT32 sdRetVal = D4SS1_SUCCESS;
	UINT8 bRA,  bRN,  bMB,  bSB;

	/*
	dName
	bit 31 - 16	: register address
	bit 15 -  8	: mask bit
	bit  7 -  0	: shift bit
	*/
	bRA = (UINT8)((dName & 0xFF0000) >> 16);
	bRN = bRA - 0x80;
	bMB = (UINT8)((dName & 0x00FF00) >> 8);
	bSB = (UINT8)(dName & 0x0000FF);

	/* check arguments */
	if (bRA < D4SS1_MIN_REGISTERADDRESS) {
		if (D4SS1_MAX_READ_REGISTERADDRESS < bRA)
			d4ErrHandler(D4SS1_ERROR_ARGUMENT);
	}
	sdRetVal = d4Read(bRA,  pbPrm);
	D4SS1_UpdateRegisterMap(sdRetVal,  bRN,  *pbPrm);
	*pbPrm = ((g_bD4SS1RegisterMap[bRN] & bMB) >> bSB);
}

/*******************************************************************************
 *	D4SS1_ReadRegisterByte
 *
 *	Function:
 *			read register "byte" function
 *	Argument:
 *			UINT8	bAddress	: register address
 *			UINT8	*pbPrm	: register parameter
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_ReadRegisterByte(UINT8 bAddress,  UINT8 *pbPrm)
{
	SINT32 sdRetVal = D4SS1_SUCCESS;
	UINT8 bNumber;
	/* check arguments */
	if (bAddress < D4SS1_MIN_REGISTERADDRESS) {
		if (D4SS1_MAX_READ_REGISTERADDRESS < bAddress)
			d4ErrHandler(D4SS1_ERROR_ARGUMENT);
	}
	bNumber = bAddress - 0x80;
	sdRetVal = d4Read(bAddress,  pbPrm);
	D4SS1_UpdateRegisterMap(sdRetVal,  bNumber,  *pbPrm);
}

/*******************************************************************************
 *	D4SS1_CheckArgument_Mixer
 *
 *	Function:
 *			check D-4SS1 setting information for mixer
 *	Argument:
 *			UINT8 bHpFlag : "change HP amp mixer setting" flag
 *(0 : no check,  1 : check)
 *			UINT8 bSpFlag : "change SP amp mixer setting" flag
 *(0 : no check,  1 : check)
 *			D4SS1_SETTING_INFO *pstSettingInfo :
 *D-4SS1 setting information
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_CheckArgument_Mixer(UINT8 bHpFlag,  UINT8 bSpFlag,
D4SS1_SETTING_INFO *pstSettingInfo)
{
	UINT8 bCheckArgument = 0;

	/* HP */
	if (bHpFlag == 1) {
		if (pstSettingInfo->bHpCh > 1) {
			pstSettingInfo->bHpCh = 0;
			bCheckArgument++;
		}
		if (pstSettingInfo->bHpMixer_INA > 1) {
			pstSettingInfo->bHpMixer_INA = 0;
			bCheckArgument++;
		}
		if (pstSettingInfo->bHpMixer_INB > 1) {
			pstSettingInfo->bHpMixer_INB = 0;
			bCheckArgument++;
		}
	}

	/* SP */
	if (bSpFlag == 1) {
		if (pstSettingInfo->bSpMixer_INA > 1) {
			pstSettingInfo->bSpMixer_INA = 0;
			bCheckArgument++;
		}
		if (pstSettingInfo->bSpMixer_INB > 1) {
			pstSettingInfo->bSpMixer_INB = 0;
			bCheckArgument++;
		}
	}

	/* check argument */
	if (bCheckArgument > 0)
		d4ErrHandler(D4SS1_ERROR_ARGUMENT);
}

/*******************************************************************************
 *	D4SS1_CheckArgument
 *
 *	Function:
 *			check D-4SS1 setting information
 *	Argument:
 *			D4SS1_SETTING_INFO *pstSettingInfo
 *: D-4SS1 setting information
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_CheckArgument(D4SS1_SETTING_INFO *pstSettingInfo)
{
	UINT8 bCheckArgument = 0;

	/* IN */
	if (pstSettingInfo->bINAGain > 12) {
		pstSettingInfo->bINAGain = 2;
		bCheckArgument++;
	}
	if (pstSettingInfo->bINBGain > 12) {
		pstSettingInfo->bINBGain = 2;
		bCheckArgument++;
	}
	if (pstSettingInfo->bINABalance > 1) {
		pstSettingInfo->bINABalance = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bINBBalance > 1)	{
		pstSettingInfo->bINBBalance = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bVolMode > 1)	{
		pstSettingInfo->bVolMode = 0;
		bCheckArgument++;
	}

	/* HP */
	if (pstSettingInfo->bHpCTCan > 3) {
		pstSettingInfo->bHpCTCan = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpCpMode > 1) {
		pstSettingInfo->bHpCpMode = 1;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpDvddLev > 1) {
		pstSettingInfo->bHpDvddLev = 0;
		bCheckArgument++;
	}
	if ((pstSettingInfo->bHpAtt > 0 && pstSettingInfo->bHpAtt < 36)
		|| pstSettingInfo->bHpAtt > 127) {
		pstSettingInfo->bHpAtt = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpSvolOff > 1) {
		pstSettingInfo->bHpSvolOff = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpCh > 1) {
		pstSettingInfo->bHpCh = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpMixer_INA > 1) {
		pstSettingInfo->bHpMixer_INA = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpMixer_INB > 1) {
		pstSettingInfo->bHpMixer_INB = 0;
		bCheckArgument++;
	}

	/* SP */
	if ((pstSettingInfo->bSpAtt > 0 && pstSettingInfo->bSpAtt < 36)
		|| pstSettingInfo->bSpAtt > 127) {
		pstSettingInfo->bSpAtt = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bSpSvolOff > 1) {
		pstSettingInfo->bSpSvolOff = 0;
		bCheckArgument++;
		}
	if (pstSettingInfo->bSpPPDOff > 1)	{
		pstSettingInfo->bSpPPDOff = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bSpMixer_INA > 1) {
		pstSettingInfo->bSpMixer_INA = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bSpMixer_INB > 1) {
		pstSettingInfo->bSpMixer_INB = 0;
		bCheckArgument++;
	}

	/* Noise Gate */
	if (pstSettingInfo->bSpNg_DetectionLv > 7) {
		pstSettingInfo->bSpNg_DetectionLv = 4;
		bCheckArgument++;
	}
	if (pstSettingInfo->bSpNg_Att > 2) {
		pstSettingInfo->bSpNg_Att = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpNg_DetectionLv == 1
		|| pstSettingInfo->bHpNg_DetectionLv > 5) {
		pstSettingInfo->bHpNg_DetectionLv = 4;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpNg_Att > 3) {
		pstSettingInfo->bHpNg_Att = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpSpNg_ReleaseTime > 3) {
		pstSettingInfo->bHpSpNg_ReleaseTime = 3;
		bCheckArgument++;
	}
	if (pstSettingInfo->bHpSpNg_ModeSel > 1) {
		pstSettingInfo->bHpSpNg_ModeSel = 0;
		bCheckArgument++;
	}

	/* Non-Clip & PowerLimit & DRC */
	if (pstSettingInfo->bSpNcpl_Enable > 1) {
		pstSettingInfo->bSpNcpl_Enable = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bSpNcpl_DRCMode == 2
		|| pstSettingInfo->bSpNcpl_DRCMode > 3) {
		pstSettingInfo->bSpNcpl_DRCMode = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bSpNcpl_PowerLimit > 10) {
		pstSettingInfo->bSpNcpl_PowerLimit = 0;
		bCheckArgument++;
	}
	if (pstSettingInfo->bSpNcpl_AttackTime > 3) {
		pstSettingInfo->bSpNcpl_AttackTime = 1;
		bCheckArgument++;
	}

	/* check argument */
	if (bCheckArgument > 0)
		d4ErrHandler(D4SS1_ERROR_ARGUMENT);
}

/*******************************************************************************
 *	D4SS1_ControlMixer
 *
 *	Function:
 *			control HP amp mixer and SP amp mixer in D-4SS1
 *	Argument:
 *			UINT8 bHpFlag : "change HP amp mixer setting" flag
 *(0 : no change,  1 : change)
 *			UINT8 bSpFlag : "change SP amp mixer setting" flag
 *(0 : no change,  1 : change)
 *			D4SS1_SETTING_INFO *pstSetMixer :
 *D-4SS1 setting information
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_ControlMixer(UINT8 bHpFlag,  UINT8 bSpFlag,
D4SS1_SETTING_INFO *pstSetMixer)
{
	UINT8 bWriteRA,  bWritePrm;
	UINT8 bTempHpCh,  bTempHpMixer_INA,  bTempHpMixer_INB;
	UINT8 bTempSpMixer_INA,  bTempSpMixer_INB;

	/* check argument */
	if (bHpFlag > 1)
		bHpFlag = 0;
	if (bSpFlag > 1)
		bSpFlag = 0;
	/* check argument */
	D4SS1_CheckArgument_Mixer(bHpFlag,  bSpFlag,  pstSetMixer);

	/* change mixer sequence */
	/* SP */
	if (bSpFlag == 1) {
		bTempSpMixer_INA = 0;
		bTempSpMixer_INB = 0;
	} else {
		bTempSpMixer_INA =
		(g_bD4SS1RegisterMap[8] & 0x02) >> (D4SS1_SP_AMIX & 0xFF);
		bTempSpMixer_INB =
		(g_bD4SS1RegisterMap[8] & 0x01) >> (D4SS1_SP_BMIX & 0xFF);
	}

	/* HP */
	bTempHpCh = (g_bD4SS1RegisterMap[8] & 0x10) >> (D4SS1_HP_MONO & 0xFF);
	if (bHpFlag == 1) {
		bTempHpMixer_INA = 0;
		bTempHpMixer_INB = 0;
	} else {
		bTempHpMixer_INA =
		(g_bD4SS1RegisterMap[8] & 0x08) >> (D4SS1_HP_AMIX & 0xFF);
		bTempHpMixer_INB =
		(g_bD4SS1RegisterMap[8] & 0x04) >> (D4SS1_HP_BMIX & 0xFF);
	}

	/* write register #0x88 */
	bWriteRA = 0x88;
	bWritePrm = (bTempHpCh << 4)/* HP_MONO */
| (bTempHpMixer_INA << 3)
| (bTempHpMixer_INB << 2)/* HP_AMIX,  HP_BMIX */
| (bTempSpMixer_INA << 1)
| (bTempSpMixer_INB);/* SP_AMIX,  SP_BMIX */

	D4SS1_WriteRegisterByte(bWriteRA,  bWritePrm);

	/* set HP amp mixer,  SP amp mixer */
	if (bHpFlag == 1) {
		bTempHpCh = pstSetMixer->bHpCh;
		bTempHpMixer_INA = pstSetMixer->bHpMixer_INA;
		bTempHpMixer_INB = pstSetMixer->bHpMixer_INB;
	}
	if (bSpFlag == 1) {
		bTempSpMixer_INA = pstSetMixer->bSpMixer_INA;
		bTempSpMixer_INB = pstSetMixer->bSpMixer_INB;
	}

	/* write register #0x88 */
	if ((bHpFlag == 1) || (bSpFlag == 1)) {
		bWriteRA = 0x88;
		bWritePrm = (bTempHpCh << 4)/* HP_MONO */
	| (bTempHpMixer_INA << 3)
	| (bTempHpMixer_INB << 2)/* HP_AMIX,  HP_BMIX */
	| (bTempSpMixer_INA << 1)
	| (bTempSpMixer_INB);/* SP_AMIX,  SP_BMIX */

		D4SS1_WriteRegisterByte(bWriteRA,  bWritePrm);
	}
}

/*******************************************************************************
 *	D4SS1_ControlVolume
 *
 *	Function:
 *			control HP amp attenuator
 *			and SP amp attenuator in D-4SS1
 *	Argument:
 *			UINT8 bHpFlag : "change HP amp attenuator setting"
 *			flag(0 : no change,  1 : change)
 *			UINT8 bSpFlag : "change SP amp attenuator setting"
 *			flag(0 : no change,  1 : change)
 *			D4SS1_SETTING_INFO *pstSetVol
 *			: D-4SS1 setting information
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_ControlVolume(UINT8 bHpFlag,  UINT8 bSpFlag,
D4SS1_SETTING_INFO *pstSetVol)
{
	UINT8 bPwrOff;

	/* check argument */
	if (bHpFlag > 1)
		bHpFlag = 0;
	if (bSpFlag > 1)
		bSpFlag = 0;

	D4SS1_ReadRegisterBit(D4SS1_SRST,  &bPwrOff);

	/* HP */
	if (bHpFlag == 1) {
		/* check argument */
		if ((pstSetVol->bHpAtt > 0 && pstSetVol->bHpAtt < 36)
			|| pstSetVol->bHpAtt > 127) {
			pstSetVol->bHpAtt = 0;
			d4ErrHandler(D4SS1_ERROR_ARGUMENT);
		}
		g_bHPAmpAtt = pstSetVol->bHpAtt;
		/* PowerOn... */
		if (bPwrOff == 0)
			D4SS1_WriteRegisterBit(D4SS1_HP_ATT,  g_bHPAmpAtt);
	}

	/* SP */
	if (bSpFlag == 1) {
		/* check argument */
		if ((pstSetVol->bSpAtt > 0 && pstSetVol->bSpAtt < 36)
			|| pstSetVol->bSpAtt > 127) {
			pstSetVol->bSpAtt = 0;
			d4ErrHandler(D4SS1_ERROR_ARGUMENT);
		}

		g_bSPAmpAtt = pstSetVol->bSpAtt;

		/* PowerOn... */
		if (bPwrOff == 0)
			D4SS1_WriteRegisterBit(D4SS1_SP_ATT,  g_bSPAmpAtt);
	}
}

/*******************************************************************************
 *	D4SS1_PowerOn
 *
 *	Function:
 *			power on D-4SS1
 *	Argument:
 *			D4SS1_SETTING_INFO *pstSettingInfo :
 *			D-4SS1 setting information
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_PowerOn(D4SS1_SETTING_INFO *pstSettingInfo)
{
	UINT8 bWriteAddress;
	UINT8 abWritePrm[9];

	/* PowerLimit Setting */
	UINT8 bSpNcpl_DPLT[]    = { 0,  1,  2,  3,  1,  2,  3,  4,  5,  6,  7};
	UINT8 bSpNcpl_DPLT_EX[] = { 0,  1,  1,  1,  0,  0,  0,  0,  0,  0,  0};

	/* check argument */
	D4SS1_CheckArgument(pstSettingInfo);

	/* set parameter */
	bWriteAddress = 0x80;

	/* 0x80 SoftReset */
	D4SS1_WriteRegisterBit(D4SS1_SRST,  0);
	/* 0x80 */
	abWritePrm[0] =
(pstSettingInfo->bHpCTCan << (D4SS1_CTCAN & 0xFF))
| (pstSettingInfo->bSpPPDOff << (D4SS1_PPDOFF & 0xFF))
| (pstSettingInfo->bHpSpNg_ModeSel << (D4SS1_MODESEL & 0xFF))
| (pstSettingInfo->bHpCpMode << (D4SS1_CPMODE & 0xFF))
| (pstSettingInfo->bHpDvddLev << (D4SS1_VLEVEL & 0xFF));
	/* 0x81 */
	abWritePrm[1] =
(pstSettingInfo->bSpNcpl_DRCMode << (D4SS1_DRC_MODE & 0xFF))
| (pstSettingInfo->bSpNcpl_AttackTime << (D4SS1_DATRT & 0xFF))
| (pstSettingInfo->bHpSpNg_ReleaseTime << (D4SS1_NG_ATRT & 0xFF));

	/* 0x82 */
	abWritePrm[2] =
(bSpNcpl_DPLT[pstSettingInfo->bSpNcpl_PowerLimit] << (D4SS1_DPLT & 0xFF))
| (pstSettingInfo->bHpNg_DetectionLv << (D4SS1_HP_NG_RAT & 0xFF))
| (pstSettingInfo->bHpNg_Att << (D4SS1_HP_NG_ATT & 0xFF));
	/* 0x83 */
	abWritePrm[3] =
(bSpNcpl_DPLT_EX[pstSettingInfo->bSpNcpl_PowerLimit]<<(D4SS1_DPLT_EX & 0xFF))
| (pstSettingInfo->bSpNcpl_Enable << (D4SS1_NCLIP & 0xFF))
| (pstSettingInfo->bSpNg_DetectionLv << (D4SS1_SP_NG_RAT & 0xFF))
| (pstSettingInfo->bSpNg_Att << (D4SS1_SP_NG_ATT & 0xFF));

	/* 0x84 */
	abWritePrm[4] = (pstSettingInfo->bINAGain << (D4SS1_VA & 0xFF))
| (pstSettingInfo->bINBGain << (D4SS1_VB & 0xFF));
	/* 0x85 */
	abWritePrm[5] = (pstSettingInfo->bINABalance << (D4SS1_DIFA & 0xFF))
| (pstSettingInfo->bINBBalance << (D4SS1_DIFB & 0xFF))
| (pstSettingInfo->bHpSvolOff << (D4SS1_HP_SVOFF & 0xFF))
#ifdef HP_HIZ_ON
| (0x01 << (D4SS1_HP_HIZ & 0xFF))
#endif
#ifdef SP_HIZ_ON
| (0x01 << (D4SS1_SP_HIZ & 0xFF))
#endif
| (pstSettingInfo->bSpSvolOff << (D4SS1_SP_SVOFF & 0xFF));

	/* 0x86 */
	abWritePrm[6] = (pstSettingInfo->bVolMode << (D4SS1_VOLMODE & 0xFF))
| (pstSettingInfo->bSpAtt << (D4SS1_SP_ATT & 0xFF));
	/* 0x87 */
	abWritePrm[7] = (pstSettingInfo->bHpAtt << (D4SS1_HP_ATT & 0xFF));
	/* 0x88 */
	abWritePrm[8] = (pstSettingInfo->bHpCh << (D4SS1_HP_MONO & 0xFF))
| (pstSettingInfo->bHpMixer_INA << (D4SS1_HP_AMIX & 0xFF))
| (pstSettingInfo->bHpMixer_INB << (D4SS1_HP_BMIX & 0xFF))
| (pstSettingInfo->bSpMixer_INA << (D4SS1_SP_AMIX & 0xFF))
| (pstSettingInfo->bSpMixer_INB << (D4SS1_SP_BMIX & 0xFF));

	/* write 0x80 - 0x88 : power on */
	D4SS1_WriteRegisterByte(0x80,  abWritePrm[0]);
	D4SS1_WriteRegisterByte(0x81,  abWritePrm[1]);
	D4SS1_WriteRegisterByte(0x82,  abWritePrm[2]);
	D4SS1_WriteRegisterByte(0x83,  abWritePrm[3]);
	D4SS1_WriteRegisterByte(0x84,  abWritePrm[4]);
	D4SS1_WriteRegisterByte(0x85,  abWritePrm[5]);
	D4SS1_WriteRegisterByte(0x86,  abWritePrm[6]);
	D4SS1_WriteRegisterByte(0x87,  abWritePrm[7]);
	D4SS1_WriteRegisterByte(0x88,  abWritePrm[8]);
}

/*******************************************************************************
 *	D4SS1_PowerOff
 *
 *	Function:
 *			power off D-4SS1
 *	Argument:
 *			none
 *
 *	Return:
 *			none
 *
 ******************************************************************************/
void D4SS1_PowerOff(void)
{
	UINT8 bWriteAddress;
	UINT8 bWritePrm;
	UINT8 bHP_Ch;

	/* 0x88 : power off HP amp,  SP amp */
	bWriteAddress = 0x88;
	bWritePrm = 0x00;

	D4SS1_ReadRegisterBit(D4SS1_HP_MONO,  &bHP_Ch);
	bWritePrm = (bHP_Ch << 4);
/* HP_MONO,  HP_AMIX=0,  HP_BMIX=0,  SP_AMIX=0,  SP_BMIX=0 */
	D4SS1_WriteRegisterByte(bWriteAddress,  bWritePrm);

	d4Sleep(D4SS1_OFFSEQUENCE_WAITTIME);
}

void yda173_speaker_onoff(int onoff)	/* speaker path amp onoff */
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":speaker on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 0;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNg_DetectionLv = 0;
#else
	stInfo.bSpNg_DetectionLv = 0;
#endif
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNcpl_PowerLimit = 4;
#else
	stInfo.bSpNcpl_PowerLimit = 3;
#endif
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}

void yda173_speaker_onoff_incall(int onoff)	/* spk path onoff in a call */
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":speaker call on\n");
	stInfo.bINAGain = g_ampgain[1].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[1].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[1].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 0;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[1].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNg_DetectionLv = 0;
#else
	stInfo.bSpNg_DetectionLv = 0;
#endif
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNcpl_PowerLimit = 0;
#else
	stInfo.bSpNcpl_PowerLimit = 4;
#endif

	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}

void yda173_speaker_onoff_ext(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":external speaker call on\n");
	stInfo.bINAGain = g_ampgain[4].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[4].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[4].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 0;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[4].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNg_DetectionLv = 0;
#else
	stInfo.bSpNg_DetectionLv = 0;
#endif
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNcpl_PowerLimit = 0;
#else
	stInfo.bSpNcpl_PowerLimit = 4;
#endif

	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}
void yda173_speaker_onoff_inVoip1(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":speaker voip1 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 0;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNg_DetectionLv = 0;
#else
	stInfo.bSpNg_DetectionLv = 0;
#endif
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNcpl_PowerLimit = 5;
#else
	stInfo.bSpNcpl_PowerLimit = 5;
#endif

	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}

void yda173_speaker_onoff_inVoip2(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":speaker voip2 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 0;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNg_DetectionLv = 0;
#else
	stInfo.bSpNg_DetectionLv = 0;
#endif
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNcpl_PowerLimit = 5;
#else
	stInfo.bSpNcpl_PowerLimit = 5;
#endif

	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}

void yda173_speaker_onoff_inVoip3(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":speaker voip3 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 0;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNg_DetectionLv = 0;
#else
	stInfo.bSpNg_DetectionLv = 0;
#endif
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNcpl_PowerLimit = 5;
#else
	stInfo.bSpNcpl_PowerLimit = 5;
#endif

	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}

void yda173_speaker_onoff_inVoip4(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":speaker voip4 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 0;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNg_DetectionLv = 0;
#else
	stInfo.bSpNg_DetectionLv = 0;
#endif
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNcpl_PowerLimit = 5;
#else
	stInfo.bSpNcpl_PowerLimit = 5;
#endif

	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}

void yda173_speaker_onoff_inVoip5(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":speaker voip5 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 0;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNg_DetectionLv = 0;
#else
	stInfo.bSpNg_DetectionLv = 0;
#endif
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bSpNcpl_PowerLimit = 5;
#else
	stInfo.bSpNcpl_PowerLimit = 5;
#endif

	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}

void yda173_headset_onoff(int onoff)	/* headset path amp onoff */
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":headset on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#else
	stInfo.bHpNg_DetectionLv = 2;
	/* HP Noise Gate : detection level */
#endif
	stInfo.bHpNg_Att = 3;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}


void yda173_headset_onoff_inVoip1(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":headset voip1 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#else
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#endif
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}


void yda173_headset_onoff_inVoip2(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":headset voip2 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#else
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#endif
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}


void yda173_headset_onoff_inVoip3(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":headset voip3 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#else
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#endif
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}


void yda173_headset_onoff_inVoip4(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":headset voip4 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#else
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#endif
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}


void yda173_headset_onoff_inVoip5(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":headset voip5 on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#else
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#endif
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}


void yda173_headset_onoff_incall(int onoff)
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":headset incall on\n");
	stInfo.bINAGain = g_ampgain[1].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[1].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}

void yda173_headset_onoff_fac(int onoff)	/* headset path amp onoff */
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":headset factory on\n");
	stInfo.bINAGain = g_ampgain[2].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[2].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[2].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[2].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
#if defined(CONFIG_MACH_PREVAIL2)
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#else
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
#endif
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":headset factory off\n");
		D4SS1_PowerOff();
	}
}


void yda173_speaker_headset_onoff(int onoff)	/* spk+ear amp onoff */
{
	D4SS1_SETTING_INFO stInfo;

	if (onoff) {
		pr_info(MODULE_NAME ":speaker_headset on\n");
	stInfo.bINAGain = g_ampgain[3].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[3].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */
	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
					2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[3].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */
	/* SP */
	stInfo.bSpAtt = g_ampgain[3].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 1;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}
void yda173_tty_onoff(int onoff)	/* tty path amp onoff */
{
	D4SS1_SETTING_INFO stInfo;
	if (onoff) {
		pr_info(MODULE_NAME ":tty on\n");
	stInfo.bINAGain = g_ampgain[0].in1_gain;
	/* INA Gain Amp */
	stInfo.bINBGain = g_ampgain[0].in2_gain;
	/* INB Gain Amp */
	stInfo.bINABalance = 0;
	/* INA unBalance(0) or Balance(1) */
	stInfo.bINBBalance = 0;
	/* INB unBalance(0) or Balance(1) */
	stInfo.bVolMode = 0;
	/* Volmode */

	/* HP */
	stInfo.bHpCTCan = 0;
	/* HP Cross talk cancel */
	stInfo.bHpCpMode = 0;
	/* HP charge pump mode,  3-stage mode(0) or 2-stage mode(1) */
	stInfo.bHpDvddLev = 0;
	/* HP charge pump DVDD level,  1.60<=DVDD<2.40V(0) or
	2.40V<=DVDD<=2.80V(1) */
	stInfo.bHpAtt = g_ampgain[0].hp_att;
	/* HP attenuator */
	stInfo.bHpSvolOff = 0;
	/* HP soft volume off setting,  on(0) / off(1) */
	stInfo.bHpCh = 0;
	/* HP channel,  stereo(0)/mono(1) */
	stInfo.bHpMixer_INA = 0;
	/* HP mixer INA setting */
	stInfo.bHpMixer_INB = 1;
	/* HP mixer INB setting */

	/* SP */
	stInfo.bSpAtt = g_ampgain[0].sp_att;
	/* SP attenuator */
	stInfo.bSpSvolOff = 0;
	/* SP soft volume off setting,  on(0) / off(1) */
	stInfo.bSpPPDOff = 0;
	/* SP partial power down setting,  on(0) / off(1) */
	stInfo.bSpMixer_INA = 0;
	/* SP mixer INA setting */
	stInfo.bSpMixer_INB = 0;
	/* SP mixer INB setting */
	/* Noise Gate */
	stInfo.bSpNg_DetectionLv = 0;
	/* SP Noise Gate : detection level */
	stInfo.bSpNg_Att = 0;
	/* SP Noise Gate : attenuator */
	stInfo.bHpNg_DetectionLv = 0;
	/* HP Noise Gate : detection level */
	stInfo.bHpNg_Att = 0;
	/* HP Noise Gate : attenuator */
	stInfo.bHpSpNg_ReleaseTime = 0;
	/* HP / SP Noise Gate : release Time */
	stInfo.bHpSpNg_ModeSel = 0;
	/* HP / SP Noise Gate mode */
	/* Non-Clip & PowerLimit & DRC */
	stInfo.bSpNcpl_Enable = 0;
	/* SP Non-Clip power limit : Enable(1)/Disable(0) */
	stInfo.bSpNcpl_DRCMode = 0;
	/* SP Non-Clip power limit : DRC */
	stInfo.bSpNcpl_PowerLimit = 0;
	/* SP Non-Clip power limit : Power Limit */
	stInfo.bSpNcpl_AttackTime = 0;
	/* SP Non-Clip power limit : attack Time */

		D4SS1_PowerOn(&stInfo);

#if AMPREG_DEBUG
		D4SS1_ReadRegisterByte(0x80,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x81,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x82,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x83,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x84,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x85,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x86,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x87,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
		D4SS1_ReadRegisterByte(0x88,  &buf);
		pr_info(MODULE_NAME ":%d = %02x\n", __LINE__, buf);
#endif
	} else {
		pr_info(MODULE_NAME ":speaker off\n");
		D4SS1_PowerOff();
	}
}


static int amp_open(struct inode *inode,  struct file *file)
{
	return nonseekable_open(inode,  file);
}

static int amp_release(struct inode *inode,  struct file *file)
{
	return 0;
}

static long amp_ioctl(struct file *file,  unsigned int cmd, unsigned long arg)
{
	/* int mode;
	 * switch (cmd)
	 * {
		 * case SND_SET_AMPGAIN :
			 * if (copy_from_user(&mode,
			 (void __user *) arg,  sizeof(mode))) {
				 * pr_err(MODULE_NAME ": %s fail\n",  __func__);
				 * break;
			 * }
			 * if (mode >= 0 && mode < MODE_NUM_MAX)
				 * cur_mode = mode;

			 * break;
		 * default :
			 * break;
	 * } */
	return 0;
}

const struct file_operations amp_fops = {
	.owner = THIS_MODULE,
	.open = amp_open,
	.release = amp_release,
	.unlocked_ioctl = amp_ioctl,
};

static struct miscdevice amp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "amp",
	.fops = &amp_fops,
};

static int yda173_probe(struct i2c_client *client,
const struct i2c_device_id *dev_id)
{
	int err = 0;
	pr_info(MODULE_NAME ":%s\n", __func__);

	if (!i2c_check_functionality(client->adapter,  I2C_FUNC_I2C))
		goto exit_check_functionality_failed;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev,  "platform data is NULL. exiting.\n");
		err = -ENODEV;
		return err;
	}

	pclient = client;

	memcpy(&g_data,  client->dev.platform_data,
	sizeof(struct yda173_i2c_data));

	if (misc_register(&amp_device))
		pr_err(MODULE_NAME ": misc device register failed\n");
	load_ampgain();

	return 0;

exit_check_functionality_failed:
	return err;
}

static int yda173_remove(struct i2c_client *client)
{
	pclient = NULL;

	return 0;
}


static const struct i2c_device_id yda173_id[] = {
	{ "yda173", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, yda173_id);

static struct i2c_driver yda173_driver = {
.id_table = yda173_id,
.probe = yda173_probe,
.remove = yda173_remove,
/*
#ifndef CONFIG_ANDROID_POWER
.suspend = yda173_suspend,
.resume = yda173_resume,
#endif
.shutdown = yda173_shutdown,
*/
.driver = {
.name = "yda173",
	},
};

static int __init yda173_init(void)
{
	pr_info(MODULE_NAME ":%s\n", __func__);
	return i2c_add_driver(&yda173_driver);
}

static void __exit yda173_exit(void)
{
	i2c_del_driver(&yda173_driver);
}

module_init(yda173_init);
module_exit(yda173_exit);

MODULE_AUTHOR("Jongcheol Park");
MODULE_DESCRIPTION("YDA173 Driver");
MODULE_LICENSE("GPL");
