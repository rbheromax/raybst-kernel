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
 *		Module		: D4SS1_Ctrl_def.h
 *
 *		Description	: D-4SS1 control define
 *
 *		Version		: 1.0.0	2011.11.24
 *
*********/
#ifndef	_D4SS1_CTRL_DEF_H_
#define	_D4SS1_CTRL_DEF_H_

/*********/
/*
	Register Name
	bit 31 - 16	: register address
	bit 15 -  8	: mask bit
	bit  7 -  0	: shift bit
*/

/* #0 */
#define D4SS1_SRST			0x808007
#define D4SS1_CTCAN			0x806005
#define D4SS1_PPDOFF		0x801004
#define D4SS1_MODESEL		0x800803
#define D4SS1_CPMODE		0x800402
#define D4SS1_VLEVEL		0x800100

/* #1 */
#define D4SS1_DRC_MODE		0x81C006
#define D4SS1_DATRT			0x813004
#define D4SS1_NG_ATRT		0x810C02

/* #2 */
#define D4SS1_DPLT			0x82E005
#define D4SS1_HP_NG_RAT		0x821C02
#define D4SS1_HP_NG_ATT		0x820300

/* #3 */
#define D4SS1_DPLT_EX		0x834006
#define D4SS1_NCLIP			0x832005
#define D4SS1_SP_NG_RAT		0x831C02
#define D4SS1_SP_NG_ATT		0x830300

/* #4 */
#define D4SS1_VA			0x84F004
#define D4SS1_VB			0x840F00

/* #5 */
#define D4SS1_DIFA			0x858007
#define D4SS1_DIFB			0x854006
#define D4SS1_HP_SVOFF		0x850803
#define D4SS1_HP_HIZ		0x850402
#define D4SS1_SP_SVOFF		0x850201
#define D4SS1_SP_HIZ		0x850100

/* #6 */
#define D4SS1_VOLMODE		0x868007
#define D4SS1_SP_ATT		0x867F00

/* #7 */
#define D4SS1_HP_ATT		0x877F00

/* #8 */
#define D4SS1_OCP_ERR		0x888007
#define D4SS1_OTP_ERR		0x884006
#define D4SS1_DC_ERR		0x882005
#define D4SS1_HP_MONO		0x881004
#define D4SS1_HP_AMIX		0x880803
#define D4SS1_HP_BMIX		0x880402
#define D4SS1_SP_AMIX		0x880201
#define D4SS1_SP_BMIX		0x880100

/*********/

/* return value */
#define D4SS1_SUCCESS			0
#define D4SS1_ERROR				-1
#define D4SS1_ERROR_ARGUMENT	-2

/* D-4SS1 value */
#define D4SS1_MIN_REGISTERADDRESS			0x80
#define D4SS1_MAX_WRITE_REGISTERADDRESS		0x88
#define D4SS1_MAX_READ_REGISTERADDRESS		0x88

/* type */
#define SINT32 signed long
#define UINT32 unsigned long
#define SINT8 signed char
#define UINT8 unsigned char
#endif	/* _D4SS1_CTRL_H_ */
