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
/*********
 *
 *
 *		Copyright (c) 2011 Yamaha Corporation
 *
 *		Module		: D4SS1_Ctrl.h
 *
 *		Description	: D-4SS1 control module define
 *
 *		Version	: 1.0.0	2011.11.24
 *
*********/
#ifndef	_D4SS1_CTRL_H_
#define	_D4SS1_CTRL_H_


/* user setting */
/*********/
#define HP_HIZ_ON
/* #define SP_HIZ_ON */

#define D4SS1_OFFSEQUENCE_WAITTIME	30
/*********/
/* structure */
/*********/
/* D-4SS1 setting information */
typedef struct {
	/* input */
	unsigned char	bINAGain;
	unsigned char	bINBGain;
	unsigned char	bINABalance;
	unsigned char	bINBBalance;
	unsigned char	bVolMode;
	/* HP */
	unsigned char	bHpCTCan;
	unsigned char	bHpCpMode;
	unsigned char	bHpDvddLev;
	unsigned char	bHpAtt;
	unsigned char	bHpSvolOff;
	unsigned char	bHpCh;
	unsigned char	bHpMixer_INA;
	unsigned char	bHpMixer_INB;
	/* SP */
	unsigned char	bSpAtt;
	unsigned char	bSpSvolOff;
	unsigned char	bSpPPDOff;
	unsigned char	bSpMixer_INA;
	unsigned char	bSpMixer_INB;
	/* Noise Gate */
	unsigned char	bSpNg_DetectionLv;
	unsigned char	bSpNg_Att;
	unsigned char	bHpNg_DetectionLv;
	unsigned char	bHpNg_Att;
	unsigned char	bHpSpNg_ReleaseTime;
	unsigned char	bHpSpNg_ModeSel;
	/* Non-Clip & PowerLimit & DRC */
	unsigned char	bSpNcpl_Enable;
	unsigned char	bSpNcpl_DRCMode;
	unsigned char	bSpNcpl_PowerLimit;
	unsigned char	bSpNcpl_AttackTime;
} D4SS1_SETTING_INFO;
/*********/

/* D-4SS1 Control module API */
/*********/
void D4SS1_PowerOn(D4SS1_SETTING_INFO *pstSettingInfo);
void D4SS1_PowerOff(void);
void D4SS1_ControlMixer(unsigned char bHpFlag,
unsigned char bSpFlag, D4SS1_SETTING_INFO *pstSetMixer);
void D4SS1_ControlVolume(unsigned char bHpFlag,
unsigned char bSpFlag, D4SS1_SETTING_INFO *pstSetVol);
void D4SS1_WriteRegisterBit(unsigned long bName, unsigned char bPrm);
void D4SS1_WriteRegisterByte(unsigned char bAddress, unsigned char bPrm);
void D4SS1_WriteRegisterByteN(unsigned char bAddress,
unsigned char *pbPrm, unsigned char bPrmSize);
void D4SS1_ReadRegisterBit(unsigned long bName, unsigned char *pbPrm);
void D4SS1_ReadRegisterByte(unsigned char bAddress, unsigned char *pbPrm);
/*********/

struct snd_set_ampgain {
	int in1_gain;
	int in2_gain;
	int hp_att;
	int sp_att;
};

struct yda173_i2c_data {
	struct snd_set_ampgain *ampgain;
	int num_modes;
};
void yda173_speaker_onoff(int onoff);
void yda173_speaker_onoff_incall(int onoff);
void yda173_speaker_onoff_ext(int onoff);
void yda173_speaker_onoff_inVoip1(int onoff);
void yda173_speaker_onoff_inVoip2(int onoff);
void yda173_speaker_onoff_inVoip3(int onoff);
void yda173_speaker_onoff_inVoip4(int onoff);
void yda173_speaker_onoff_inVoip5(int onoff);
void yda173_headset_onoff(int onoff);
void yda173_headset_onoff_inVoip1(int onoff);
void yda173_headset_onoff_inVoip2(int onoff);
void yda173_headset_onoff_inVoip3(int onoff);
void yda173_headset_onoff_inVoip4(int onoff);
void yda173_headset_onoff_inVoip5(int onoff);
void yda173_headset_onoff_incall(int onoff);
void yda173_speaker_headset_onoff(int onoff);
void yda173_headset_onoff_fac(int onoff);
void yda173_tty_onoff(int onoff);

#define SND_IOCTL_MAGIC 's'
#define SND_SET_AMPGAIN _IOW(SND_IOCTL_MAGIC, 2, int mode)
#endif	/* _D4SS1_CTRL_H_ */
