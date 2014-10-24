
/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name          : k3dh_misc.h
* Authors            : MH - C&I BU - Application Team
*		     : Carmine Iascone (carmine.iascone@st.com)
*		     : Matteo Dameno (matteo.dameno@st.com)
* Version            : V 1.0.5
* Date               : 26/08/2010
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
*
*******************************************************************************/
/*******************************************************************************
Version History.

Revision 1-0-0 05/11/2009
First Release
Revision 1-0-1 26/01/2010
Linux K&R Compliant Release
Revision 1-0-5 16/08/2010
Interrupt Management

*******************************************************************************/

#ifndef	__K3DH_H__
#define	__K3DH_H__

#include	<linux/ioctl.h>	/* For IOCTL macros */
#include	<linux/input.h>

#define SAD0L			0x00
#define SAD0H			0x01
#define K2DM_ACC_I2C_SADROOT	0x0C
#define K2DM_ACC_I2C_SAD_L	((K2DM_ACC_I2C_SADROOT<<1)|SAD0L)
#define K2DM_ACC_I2C_SAD_H	((K2DM_ACC_I2C_SADROOT<<1)|SAD0H)
#define	K2DM_ACC_DEV_NAME	"k2dm"
#define K2DM_ACC_DEV_VENDOR "STM"


/************************************************/
/*	Accelerometer defines section				*/
/************************************************/

/* Accelerometer Sensor Full Scale */
#define	K2DM_ACC_FS_MASK	0x30
#define K2DM_ACC_G_2G		0x00
#define K2DM_ACC_G_4G		0x10
#define K2DM_ACC_G_8G		0x20
#define K2DM_ACC_G_16G		0x30


/* Accelerometer Sensor Operating Mode */
#define K2DM_ACC_ENABLE		0x01
#define K2DM_ACC_DISABLE		0x00


#define K2DM_DEFAULT_DELAY         200
#define K2DM_MAX_DELAY             2000


#ifdef	__KERNEL__
struct k2dm_acc_platform_data {
	int (*vreg_en)(int);
};
#endif	/* __KERNEL__ */

#endif	/* __K3DH_H__ */
