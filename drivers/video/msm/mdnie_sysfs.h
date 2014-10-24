/*
 * =====================================================================
 *
 *       Filename:  mdnie_sysfs.h
 *
 *    Description:  SYSFS Node control driver
 *
 *        Version:  1.0
 *        Created:  2012 05/32 20:18:45
 *       Revision:  none
 *       Compiler:  arm-linux-gcc
 *
 *         Author:  Jang Chang Jae (),
 *        Company:  Samsung Electronics
 *
 * =====================================================================

Copyright (C) 2012, Samsung Electronics. All rights reserved.

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *

*/

#define FALSE 0
#define TRUE  1

enum eCabc_Mode {
	CABC_OFF_MODE = 0,
	CABC_ON_MODE,
	MAX_CABC_MODE,
};

enum eNegative_Mode {
	NEGATIVE_OFF_MODE = 0,
	NEGATIVE_ON_MODE,
	MAX_NEGATIVE_MODE,
};
struct mdnie_state_type {
	enum eNegative_Mode negative;
	enum eCabc_Mode cabc_mode;
};

struct mdnie_ops {
	int (*apply_negative_value)(enum eNegative_Mode negative_mode);
	int (*apply_cabc_value)(enum eCabc_Mode negative_mode);
};

/* mdnie functions */
int mdnie_sysfs_init(struct mdnie_ops *);
