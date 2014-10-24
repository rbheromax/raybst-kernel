/* Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
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

/*
 * this needs to be before <linux/kernel.h> is loaded,
 * and <linux/sched.h> loads <linux/kernel.h>
 */

/* #define DEBUG 1 */

/* ***** Test Features ***** */

/* #define __BATT_TEST_DEVICE__ */
/* #define __AUTO_TEMP_TEST__ */
/* #define __FULL_CHARGE_TEST__ */

#define BATT_LED_CONTROL_DISABLE 1 /*No LED control in Battery*/

#include <linux/android_alarm.h>
#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/atomic.h>

#include <mach/msm_rpcrouter.h>
#include <mach/msm_battery.h>

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <linux/wakelock.h>

#include <linux/pm.h>

#include <mach/vreg.h>

#ifdef CONFIG_LEDS_PMIC8058
#include <linux/leds-pmic8058.h>
#include <linux/hrtimer.h>
#endif

static struct wake_lock vbus_wake_lock;
static struct wake_lock monitor_wake_lock;

#ifdef DEBUG
#undef pr_debug
#define pr_debug pr_info
#endif

#ifdef CONFIG_MAX17043_FUEL_GAUGE
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/time.h>
#include "fuelgauge_max17040.c"
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */

#include "samsung_battery.h"

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_4_1	0x00040001
#define BATTERY_RPC_VER_5_1	0x00050001

#define BATTERY_RPC_CB_PROG	(BATTERY_RPC_PROG | 0x01000000)

#define CHG_RPC_PROG	0x3000001a
#define CHG_RPC_VER_1_1	0x00010001
#define CHG_RPC_VER_1_3	0x00010003
#define CHG_RPC_VER_2_2	0x00020002
#define CHG_RPC_VER_3_1	0x00030001
#define CHG_RPC_VER_4_1	0x00040001

#define BATTERY_REGISTER_PROC			2
#define BATTERY_MODIFY_CLIENT_PROC		4
#define BATTERY_DEREGISTER_CLIENT_PROC		5
#define BATTERY_READ_MV_PROC			12
#define BATTERY_ENABLE_DISABLE_FILTER_PROC	14

#define VBATT_FILTER		2

#define BATTERY_CB_TYPE_PROC	1
#define BATTERY_CB_ID_ALL_ACTIV	1
#define BATTERY_CB_ID_LOW_VOL	2

#define BATTERY_LOW	3400	/* 2800 */
#if defined(CONFIG_MACH_ICON)
#define BATTERY_HIGH	4350	/* 4300 */
#else
#define BATTERY_HIGH	4200	/* 4300 */
#endif

#define ONCRPC_CHG_GET_GENERAL_STATUS_PROC	12
#define ONCRPC_CHARGER_API_VERSIONS_PROC	0xffffffff

#define BATT_RPC_TIMEOUT	5000	/* 5 sec */

#define INVALID_BATT_HANDLE	-1

#define RPC_TYPE_REQ				0
#define RPC_TYPE_REPLY				1
#define RPC_REQ_REPLY_COMMON_HEADER_SIZE	(3 * sizeof(uint32_t))

/*******************************/
/* Charging control settings */

typedef enum {
	STOP_CHARGING,
	START_CHARGING
} chg_enable_type;

const int temp_table[][2] = {
	/* ADC, Temperature (C) */
#if defined(CONFIG_MACH_ICON)
	{	21,	-250	},
	{	24,	-240	},
	{	27,	-230	},
	{	30,	-220	},
	{	33,	-210	},
	{	36,	-200	},
	{	39,	-190	},
	{	42,	-180	},
	{	45,	-170	},
	{	49,	-160	},
	{	53,	-150	},
	{	57,	-140	},
	{	61,	-130	},
	{	65,	-120	},
	{	69,	-110	},
	{	73,	-100	},
	{	77,	-90	},
	{	82,	-80	},
	{	87,	-70	},
	{	92,	-60	},
	{	97,	-50	},
	{	103,	-40	},
	{	107,	-30	},
	{	113,	-20	},
	{	118,	-10	},
	{	124,	0	},
	{	129,	10	},
	{	134,	20	},
	{	140,	30	},
	{	146,	40	},
	{	152,	50	},
	{	157,	60	},
	{	162,	70	},
	{	168,	80	},
	{	174,	90	},
	{	180,	100	},
	{	185,	110	},
	{	190,	120	},
	{	195,	130	},
	{	200,	140	},
	{	205,	150	},
	{	210,	160	},
	{	215,	170	},
	{	220,	180	},
	{	226,	190	},
	{	232,	200	},
	{	236,	210	},
	{	240,	220	},
	{	245,	230	},
	{	250,	240	},
	{	255,	250	},
	{	259,	260	},
	{	263,	270	},
	{	267,	280	},
	{	271,	290	},
	{	276,	300	},
	{	280,	310	},
	{	284,	320	},
	{	288,	330	},
	{	292,	340	},
	{	297,	350	},
	{	300,	360	},
	{	303,	370	},
	{	307,	380	},
	{	311,	390	},
	{	315,	400	},
	{	318,	410	},
	{	321,	420	},
	{	323,	430	},
	{	326,	440	},
	{	329,	450	},
	{	331,	460	},
	{	333,	470	},
	{	336,	480	},
	{	339,	490	},
	{	342,	500	},
	{	343,	510	},
	{	345,	520	},
	{	347,	530	},
	{	349,	540	},
	{	351,	550	},
	{	352,	560	},
	{	354,	570	},
	{	356,	580	},
	{	358,	590	},
	{	360,	600	},
	{	361,	610	},
	{	362,	620	},
	{	363,	630	},
	{	364,	640	},
	{	366,	650	},
	{	367,	660	},
	{	368,	670	},
	{	369,	680	},
	{	370,	690	},
	{	372,	700	},
	{	373,	710	},
	{	374,	720	},
	{	375,	730	},
	{	376,	740	},
	{	378,	750	},
	{	379,	760	},
	{	380,	770	},
	{	381,	780	},
	{	381,	790	},
	{	382,	800	},
#elif defined(CONFIG_MACH_PREVAIL2)
	{	85,	-50	},
	{	90,	-40	},
	{	95,	-30	},
	{	100,	-20	},
	{	105,	-10	},
	{	110,	0	},
	{	115,	10	},
	{	120,	20	},
	{	125,	30	},
	{	144,	40	},
	{	148,	50	},
	{	153,	60	},
	{	158,	70	},
	{	163,	80	},
	{	168,	90	},
	{	173,	100	},
	{	178,	110	},
	{	183,	120	},
	{	188,	130	},
	{	193,	140	},
	{	199,	150	},
	{	204,	160	},
	{	209,	170	},
	{	214,	180	},
	{	219,	190	},
	{	223,	200	},
	{	228,	210	},
	{	233,	220	},
	{	238,	230	},
	{	242,	240	},
	{	246,	250	},
	{	250,	260	},
	{	254,	270	},
	{	258,	280	},
	{	262,	290	},
	{	267,	300	},
	{	271,	310	},
	{	275,	320	},
	{	279,	330	},
	{	283,	340	},
	{	286,	350	},
	{	289,	360	},
	{	292,	370	},
	{	296,	380	},
	{	300,	390	},
	{	349,	400	},
	{	354,	410	},
	{	358,	420	},
	{	363,	430	},
	{	368,	440	},
	{	376,	450	},
	{	380,	460	},
	{	384,	470	},
	{	388,	480	},
	{	392,	490	},
	{	394,	500	},
	{	398,	510	},
	{	402,	520	},
	{	406,	530	},
	{	410,	540	},
	{	413,	550	},
	{	417,	560	},
	{	421,	570	},
	{	425,	580	},
	{	429,	590	},
	{	431,	600	},
#elif defined(CONFIG_MACH_RAY)
	{	14,	-250	},
	{	16,	-240	},
	{	18,	-230	},
	{	20,	-220	},
	{	23,	-210	},
	{	26,	-200	},
	{	29,	-190	},
	{	32,	-180	},
	{	35,	-170	},
	{	38,	-160	},
	{	42,	-150	},
	{	46,	-140	},
	{	51,	-130	},
	{	56,	-120	},
	{	61,	-110	},
	{	66,	-100	},
	{	69,	-90	},
	{	72,	-80	},
	{	75,	-70	},
	{	78,	-60	},
	{	82,	-50	},
	{	87,	-40	},
	{	92,	-30	},
	{	97,	-20	},
	{	102,	-10	},
	{	108,	0	},
	{	112,	10	},
	{	116,	20	},
	{	120,	30	},
	{	124,	40	},
	{	128,	50	},
	{	134,	60	},
	{	140,	70	},
	{	146,	80	},
	{	152,	90	},
	{	158,	100	},
	{	167,	110	},
	{	177,	120	},
	{	187,	130	},
	{	197,	140	},
	{	207,	150	},
	{	212,	160	},
	{	217,	170	},
	{	223,	180	},
	{	229,	190	},
	{	235,	200	},
	{	239,	210	},
	{	243,	220	},
	{	248,	230	},
	{	253,	240	},
	{	258,	250	},
	{	264,	260	},
	{	270,	270	},
	{	277,	280	},
	{	284,	290	},
	{	290,	300	},
	{	295,	310	},
	{	300,	320	},
	{	305,	330	},
	{	310,	340	},
	{	316,	350	},
	{	321,	360	},
	{	327,	370	},
	{	333,	380	},
	{	339,	390	},
	{	345,	400	},
	{	349,	410	},
	{	353,	420	},
	{	357,	430	},
	{	361,	440	},
	{	366,	450	},
	{	369,	460	},
	{	372,	470	},
	{	376,	480	},
	{	380,	490	},
	{	383,	500	},
	{	387,	510	},
	{	391,	520	},
	{	395,	530	},
	{	399,	540	},
	{	404,	550	},
	{	408,	560	},
	{	412,	570	},
	{	417,	580	},
	{	421,	590	},
	{	425,	600	},
	{	428,	610	},
	{	431,	620	},
	{	434,	630	},
	{	437,	640	},
	{	440,	650	},
#else
	{ 30,		-300	},
	{ 33,		-290	},
	{ 35,		-280	},
	{ 38,		-270	},
	{ 40,		-260	},
	{ 43,		-250	},
	{ 45,		-240	},
	{ 48,		-230	},
	{ 50,		-220	},
	{ 53,		-210	},
	{ 55,		-200	},
	{ 58,		-190	},
	{ 61,		-180	},
	{ 64,		-170	},
	{ 67,		-160	},
	{ 70,		-150	},
	{ 74,		-140	},
	{ 78,		-130	},
	{ 82,		-120	},
	{ 86,		-110	},
	{ 90,		-100	},
	{ 95,		-90 },
	{ 99,		-80 },
	{ 104,		-70	},
	{ 108,		-60	},
	{ 113,		-50	},
	{ 118,		-40	},
	{ 123,		-30	},
	{ 128,		-20	},
	{ 133,		-10	},
	{ 138,		0	},
	{ 143,		10	},
	{ 148,		20	},
	{ 153,		30	},
	{ 158,		40	},
	{ 163,		50	},
	{ 168,		60	},
	{ 174,		70	},
	{ 179,		80	},
	{ 185,		90	},
	{ 190,		100	},
	{ 196,		110	},
	{ 201,		120	},
	{ 207,		130	},
	{ 212,		140	},
	{ 218,		150	},
	{ 223,		160	},
	{ 229,		170	},
	{ 234,		180	},
	{ 240,		190	},
	{ 245,		200	},
	{ 250,		210	},
	{ 255,		220	},
	{ 260,		230	},
	{ 265,		240	},
	{ 270,		250	},
	{ 275,		260	},
	{ 279,		270	},
	{ 284,		280	},
	{ 288,		290	},
	{ 293,		300	},
	{ 297,		310	},
	{ 301,		320	},
	{ 305,		330	},
	{ 309,		340	},
	{ 313,		350	},
	{ 316,		360	},
	{ 320,		370	},
	{ 323,		380	},
	{ 327,		390	},
	{ 330,		400	},
	{ 333,		410	},
	{ 336,		420	},
	{ 339,		430	},
	{ 342,		440	},
	{ 345,		450	},
	{ 348,		460	},
	{ 351,		470	},
	{ 354,		480	},
	{ 357,		490	},
	{ 360,		500	},
	{ 362,		510	},
	{ 364,		520	},
	{ 366,		530	},
	{ 368,		540	},
	{ 370,		550	},
	{ 372,		560	},
	{ 374,		570	},
	{ 376,		580	},
	{ 378,		590	},
	{ 380,		600	},
	{ 382,		610	},
	{ 383,		620	},
	{ 385,		630	},
	{ 386,		640	},
	{ 388,		650	},
	{ 389,		660 },
	{ 391,		670 },
	{ 392,		680 },
	{ 394,		690 },
	{ 395,		700 },
#endif
};

#define AVERAGE_COUNT	10

#define TIME_UNIT_SECOND	1
#define TIME_UNIT_MINUTE	60
#define TIME_UNIT_HOUR	(60 * 60)

#ifdef __FULL_CHARGE_TEST__
#define TOTAL_CHARGING_TIME	(1 * TIME_UNIT_MINUTE)
#define TOTAL_RECHARGING_TIME	(1 * TIME_UNIT_MINUTE)
#else
#if defined(CONFIG_MACH_PREVAIL2) || defined(CONFIG_MACH_ICON)\
|| defined(CONFIG_MACH_RAY)
#define TOTAL_CHARGING_TIME	(6 * TIME_UNIT_HOUR)
#else/*vital2 refresh*/
#define TOTAL_CHARGING_TIME	(5 * TIME_UNIT_HOUR)
#endif
#if defined(CONFIG_MACH_RAY)
#define SAFETYTIMER   (5 * TIME_UNIT_HOUR)
#define TOTAL_RECHARGING_TIME	(1.5 * TIME_UNIT_HOUR)
#else
#define TOTAL_RECHARGING_TIME	(2 * TIME_UNIT_HOUR)
#endif
#endif
#define TOTAL_WATING_TIME	(20 * TIME_UNIT_SECOND)
/* wait for full-charging and recharging */

#define TEMP_TABLE_OFFSET		30

#if defined(CONFIG_MACH_PREVAIL2)

#define BATT_TEMP_EVENT_BLOCK		431
#define BATT_TEMP_HIGH_BLOCK		399
#define BATT_TEMP_HIGH_RECOVER		358
#define BATT_TEMP_LOW_BLOCK		85
#define BATT_TEMP_LOW_RECOVER		110

#define BATT_TEMP_HIGH_BLOCK_LPM	382
#define BATT_TEMP_HIGH_RECOVER_LPM	376
#define BATT_TEMP_LOW_BLOCK_LPM	89
#define BATT_TEMP_LOW_RECOVER_LPM	102

#elif defined(CONFIG_MACH_RAY)

#define BATT_TEMP_EVENT_BLOCK		422
#define BATT_TEMP_HIGH_BLOCK		390
#define BATT_TEMP_HIGH_RECOVER		352
#define BATT_TEMP_LOW_BLOCK		90
#define BATT_TEMP_LOW_RECOVER		114

#define BATT_TEMP_HIGH_BLOCK_LPM	377
#define BATT_TEMP_HIGH_RECOVER_LPM	376
#define BATT_TEMP_LOW_BLOCK_LPM	91
#define BATT_TEMP_LOW_RECOVER_LPM	93

#elif defined(CONFIG_MACH_ICON)

#define BATT_TEMP_EVENT_BLOCK		367
#define BATT_TEMP_HIGH_BLOCK		334
#define BATT_TEMP_HIGH_RECOVER		317
#define BATT_TEMP_LOW_BLOCK		90
#define BATT_TEMP_LOW_RECOVER		120

#define BATT_TEMP_HIGH_BLOCK_LPM	329
#define BATT_TEMP_HIGH_RECOVER_LPM	322
#define BATT_TEMP_LOW_BLOCK_LPM		107
#define BATT_TEMP_LOW_RECOVER_LPM	118

#else/*vital2 refresh*/

#define BATT_TEMP_EVENT_BLOCK		373
#define BATT_TEMP_HIGH_BLOCK		354
#define BATT_TEMP_HIGH_RECOVER		323
#define BATT_TEMP_LOW_BLOCK			110
#define BATT_TEMP_LOW_RECOVER		130

#define BATT_TEMP_HIGH_BLOCK_LPM	332
#define BATT_TEMP_HIGH_RECOVER_LPM	326
#define BATT_TEMP_LOW_BLOCK_LPM	113
#define BATT_TEMP_LOW_RECOVER_LPM	116

#endif

/* TEMP BLOCK EVENT */
#ifdef CONFIG_BATTERY_CTIA
#define USE_CALL			(0x1 << 0)
#define USE_VIDEO			(0x1 << 1)
#define USE_MUSIC			(0x1 << 2)
#define USE_BROWSER		(0x1 << 3)
#define USE_HOTSPOT			(0x1 << 4)
#define USE_CAMERA			(0x1 << 5)
#define USE_DATA_CALL		(0x1 << 6)
#define USE_GPS			(0x1 << 7)
#define USE_LTE			(0x1 << 8)
#define USE_WIFI			(0x1 << 9)
#endif /*CONFIG_BATTERY_CTIA*/

#define TOTAL_EVENT_TIME  (10 * 60)	/* 10 minites */
#define ALARM_INTERVAL		(5 * 60)
#define CHARGING_ALARM_INTERVAL	(40)

#if defined(CONFIG_MACH_ICON)
#define BATT_FULL_CHARGING_VOLTAGE	4150
#define BATT_FULL_CHARGING_CURRENT	710

#define BATT_RECHARGING_VOLTAGE_1	4300 /*4130*/
#define BATT_RECHARGING_VOLTAGE_2	4150
#elif defined(CONFIG_MACH_PREVAIL2)
#define BATT_FULL_CHARGING_VOLTAGE	4000
#define BATT_FULL_CHARGING_CURRENT	500

#define BATT_RECHARGING_VOLTAGE_1	4128
#define BATT_RECHARGING_VOLTAGE_2	4000
#elif defined(CONFIG_MACH_RAY)
#define BATT_FULL_CHARGING_VOLTAGE	4000
#define BATT_FULL_CHARGING_CURRENT	600

#define BATT_RECHARGING_VOLTAGE_1	4131
#define BATT_RECHARGING_VOLTAGE_2	4000
#else/*vital2 refresh*/
#define BATT_FULL_CHARGING_VOLTAGE	4000
#define BATT_FULL_CHARGING_CURRENT	250

#define BATT_RECHARGING_VOLTAGE_1	4130
#define BATT_RECHARGING_VOLTAGE_2	4000
#endif

#ifdef __BATT_TEST_DEVICE__
static int temp_test_adc;
#endif

static int chg_start_time_event;
static int is_msm_batt_updating;

static int ovp_en;
static int ovp_en_handled;
enum {
	BATTERY_REGISTRATION_SUCCESSFUL = 0,
	BATTERY_DEREGISTRATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_MODIFICATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_INTERROGATION_SUCCESSFUL = BATTERY_REGISTRATION_SUCCESSFUL,
	BATTERY_CLIENT_TABLE_FULL = 1,
	BATTERY_REG_PARAMS_WRONG = 2,
	BATTERY_DEREGISTRATION_FAILED = 4,
	BATTERY_MODIFICATION_FAILED = 8,
	BATTERY_INTERROGATION_FAILED = 16,
	/* Client's filter could not be set because perhaps it does not exist */
	BATTERY_SET_FILTER_FAILED = 32,
	/* Client's could not be found for enabling or disabling the individual
	 * client */
	BATTERY_ENABLE_DISABLE_INDIVIDUAL_CLIENT_FAILED = 64,
	BATTERY_LAST_ERROR = 128,
};

enum {
	BATTERY_VOLTAGE_UP = 0,
	BATTERY_VOLTAGE_DOWN,
	BATTERY_VOLTAGE_ABOVE_THIS_LEVEL,
	BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
	BATTERY_VOLTAGE_LEVEL,
	BATTERY_ALL_ACTIVITY,
	VBATT_CHG_EVENTS,
	BATTERY_VOLTAGE_UNKNOWN,
};

/*
 * This enum contains defintions of the charger hardware status
 */
enum chg_charger_status_type {
	/* The charger is good */
	CHARGER_STATUS_GOOD,
	/* The charger is bad */
	CHARGER_STATUS_BAD,
	/* The charger is weak */
	CHARGER_STATUS_WEAK,
	/* Invalid charger status. */
	CHARGER_STATUS_INVALID
};

/*
 *This enum contains defintions of the charger hardware type
 */
enum chg_charger_hardware_type {
	/* The charger is removed */
	CHARGER_TYPE_NONE,
	/* The charger is a regular wall charger */
	CHARGER_TYPE_WALL,
	/* The charger is a PC USB */
	CHARGER_TYPE_USB_PC,
	/* The charger is a wall USB charger */
	CHARGER_TYPE_USB_WALL,
	/* The charger is a USB carkit */
	CHARGER_TYPE_USB_CARKIT,
	/* Invalid charger hardware status. */
	CHARGER_TYPE_INVALID
};

/*
 *  This enum contains defintions of the battery status
 */
enum chg_battery_status_type {
	/* The battery is good */
	BATTERY_STATUS_GOOD,
	/* The battery is cold/hot */
	BATTERY_STATUS_BAD_TEMP,
	/* The battery is bad */
	BATTERY_STATUS_BAD,
	/* The battery is removed */
	BATTERY_STATUS_REMOVED,	/* on v2.2 only */
	BATTERY_STATUS_INVALID_v1 = BATTERY_STATUS_REMOVED,
	/* Invalid battery status. */
	BATTERY_STATUS_INVALID
};

/*
 *This enum contains defintions of the battery voltage level
 */
enum chg_battery_level_type {
	/* The battery voltage is dead/very low (less than 3.2V)  */
	BATTERY_LEVEL_DEAD,
	/* The battery voltage is weak/low (between 3.2V and 3.4V)  */
	BATTERY_LEVEL_WEAK,
	/* The battery voltage is good/normal(between 3.4V and 4.2V) */
	BATTERY_LEVEL_GOOD,
	/* The battery voltage is up to full (close to 4.2V) */
	BATTERY_LEVEL_FULL,
	/* Invalid battery voltage level. */
	BATTERY_LEVEL_INVALID
};

struct rpc_reply_batt_chg_v1 {
	struct rpc_reply_hdr hdr;
	u32 more_data;

	u32 charger_status;
	u32 charger_type;
	u32 battery_status;
	u32 battery_level;
	u32 battery_voltage;
	u32 battery_temp;
	u32 chg_current;
	u32 batt_id;
};

struct rpc_reply_batt_chg_v2 {
	struct rpc_reply_batt_chg_v1 v1;

	u32 is_charger_valid;
	u32 is_charging;
	u32 is_battery_valid;
	u32 ui_event;
};

union rpc_reply_batt_chg {
	struct rpc_reply_batt_chg_v1 v1;
	struct rpc_reply_batt_chg_v2 v2;
};

static union rpc_reply_batt_chg rep_batt_chg;

struct msm_battery_info {
	struct device *dev;

	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 chg_api_version;
	u32 batt_technology;
	u32 batt_api_version;

	u32 avail_chg_sources;
	u32 current_chg_source;	/* NC (charging_source) */

	u32 batt_status;
	u32 pre_batt_status;
	u32 batt_health;
	u32 charger_valid;	/* NC */
	u32 batt_valid;
	u32 batt_capacity;	/* in percentage */
	u32 batt_presoc;

	u32 charger_status;	/* NC */
	u32 charger_type;
	u32 battery_status;	/* NC */
	u32 battery_level;	/* NC (batt_capacity) */
	u32 battery_voltage;

	u32 fg_soc;	/* NC */
	u32 batt_vol;	/* NC (battery_voltage) */
	u32 batt_temp_check;
	u32 batt_full_check;
	u32 charging_source;

	/*u32 battery_temp;*/	/* in celsius */
	int battery_temp_degc;
	u32 battery_temp_adc;	/* ADC code from CP */
	u32 chg_current_adc;	/* ICHG ADC code (charging current) */
	u32 batt_recharging;

	u32 chargingblock_clear;

	 u32(*calculate_capacity) (u32 voltage);	/* NC */

	s32 batt_handle;

	struct msm_charger_data *pdata;	/*For FSA related callback*/

	enum cable_type_t cable_status;	/*For FSA related callback*/
	enum cable_type_t prev_cable;

	struct msm_battery_callback callback; /*For FSA related callback*/

	struct power_supply *msm_psy_ac;
	struct power_supply *msm_psy_usb;
	struct power_supply *msm_psy_batt;
	struct power_supply *current_ps;	/* NC */

	struct msm_rpc_client *batt_client;
	struct msm_rpc_endpoint *chg_ep;

	struct workqueue_struct *msm_batt_wq;
	struct delayed_work msm_batt_work;

	wait_queue_head_t wait_q;

	u32 vbatt_modify_reply_avail;	/* NC */

	struct early_suspend early_suspend;

	u32 batt_slate_mode;
	u32 off_backlight;

	unsigned int voice_call_state;

#if defined(CONFIG_BATTERY_CTIA)
	struct alarm	event_alarm;
	ktime_t	cur_time;
	unsigned int batt_use;
	unsigned int batt_use_wait;
#endif /*CONFIG_BATTERY_CTIA*/
	struct alarm	alarm;
	ktime_t	cur_monitor_time;

	unsigned long charging_start_time;
	unsigned long charging_passed_time;
	unsigned int abs_time;
	unsigned int normal_abs_time;
	unsigned int rechg_time;

	bool is_timeout_chgstop;
	int vbus_valid;

	unsigned int slow_polling;
	unsigned int check_interval;

	bool check_full_state;
	bool factory_mode;
	unsigned int check_full_state_cnt;
	unsigned int full_check_count;
	unsigned int full_charged_state;
	unsigned int full_check_type;
	unsigned int first_full_count;
	unsigned int recharging_count;
	bool check_cable_change;
};

static struct msm_battery_info msm_batt_info = {
	.batt_handle = INVALID_BATT_HANDLE,
	.charger_type = CHARGER_TYPE_NONE,
	.battery_voltage = BATTERY_HIGH,
	.batt_capacity = 100,
	.batt_status = POWER_SUPPLY_STATUS_DISCHARGING,
	.batt_health = POWER_SUPPLY_HEALTH_GOOD,
	.batt_valid = 1,
	/*.battery_temp = 230,*/	/* 23.0`C */
	.batt_slate_mode = 0,
	.off_backlight = 0,
};

static enum power_supply_property msm_power_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property msm_batt_power_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,	/* in celcius */
};

static char *msm_power_supplied_to[] = {
	"battery",
};

static int msm_batt_power_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val);

static int msm_power_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val);

static struct power_supply msm_psy_ac = {
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_usb = {
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,
	.supplied_to = msm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(msm_power_supplied_to),
	.properties = msm_power_props,
	.num_properties = ARRAY_SIZE(msm_power_props),
	.get_property = msm_power_get_property,
};

static struct power_supply msm_psy_batt = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = msm_batt_power_props,
	.num_properties = ARRAY_SIZE(msm_batt_power_props),
	.get_property = msm_batt_power_get_property,
};

static struct hrtimer LedTimer;

#define CHECK_DSG_INTERVAL	(30 * 1000)
#define CHECK_CHG_INTERVAL	(30 * 1000)

static int msm_batt_driver_init;
static int msm_batt_unhandled_interrupt;

#ifdef CONFIG_WIBRO_CMC
static struct timer_list use_wimax_timer;
#define USE_MODULE_TIMEOUT	(10*60*1000)
static void use_wimax_timer_func(unsigned long unused);
#endif

#ifdef CONFIG_MAX17043_FUEL_GAUGE
static u32 get_voltage_from_fuelgauge(void);
static u32 get_level_from_fuelgauge(void);
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */

static ssize_t msm_batt_show_property(struct device *dev,
				      struct device_attribute *attr, char *buf);
static ssize_t msm_batt_store_property(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count);

static int msm_batt_average_chg_current(int chg_current_adc);

static void msm_batt_check_event(struct work_struct *work);
static void msm_batt_cable_status_update(void);

/*static int msm_batt_get_charger_type(void);*/
static void msm_batt_update_psy_status(void);
static void msm_batt_preupdate_psy_status(void);

static int msm_batt_is_over_abs_time(void);

static void msm_batt_chg_en(chg_enable_type enable);

static void msm_batt_check_event(struct work_struct *work)
{
	msm_batt_update_psy_status();
}

#define MSM_BATTERY_ATTR(_name)\
{\
	.attr = { .name = #_name, .mode = 0664 },\
	.show = msm_batt_show_property,\
	.store = msm_batt_store_property,\
}

static struct device_attribute samsung_battery_attrs[] = {
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	MSM_BATTERY_ATTR(fg_soc),
	MSM_BATTERY_ATTR(batt_reset_soc),
	MSM_BATTERY_ATTR(batt_read_raw_soc),
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */
	MSM_BATTERY_ATTR(batt_vol),
	MSM_BATTERY_ATTR(batt_temp_check),
	MSM_BATTERY_ATTR(batt_full_check),
	MSM_BATTERY_ATTR(batt_charging_source),
	MSM_BATTERY_ATTR(auth),
	MSM_BATTERY_ATTR(batt_vf_adc),
	MSM_BATTERY_ATTR(chg_current_adc),
	/* ICHG ADC code (charging current) */
	MSM_BATTERY_ATTR(batt_temp_adc),
	/* use in BatteryStatus.java (*#0228# test screen) */
	MSM_BATTERY_ATTR(batt_type),
#ifdef __BATT_TEST_DEVICE__
	MSM_BATTERY_ATTR(batt_temp_test_adc),
#endif
	MSM_BATTERY_ATTR(chargingblock_clear),
	MSM_BATTERY_ATTR(batt_slate_mode),
	MSM_BATTERY_ATTR(batt_temp_degc),
	MSM_BATTERY_ATTR(off_backlight),
#if !defined(CONFIG_BATTERY_CTIA)
	MSM_BATTERY_ATTR(talk_wcdma),
	MSM_BATTERY_ATTR(talk_gsm),
	MSM_BATTERY_ATTR(data_call),
	MSM_BATTERY_ATTR(camera),
	MSM_BATTERY_ATTR(browser),
#else
	MSM_BATTERY_ATTR(talk_wcdma),
	MSM_BATTERY_ATTR(talk_gsm),
	MSM_BATTERY_ATTR(call),
	MSM_BATTERY_ATTR(video),
	MSM_BATTERY_ATTR(music),
	MSM_BATTERY_ATTR(browser),
	MSM_BATTERY_ATTR(hotspot),
	MSM_BATTERY_ATTR(camera),
	MSM_BATTERY_ATTR(data_call),
	MSM_BATTERY_ATTR(gps),
	MSM_BATTERY_ATTR(lte),
	MSM_BATTERY_ATTR(wifi),
	MSM_BATTERY_ATTR(batt_use),
	MSM_BATTERY_ATTR(factory_mode),
#endif /*CONFIG_BATTERY_CTIA*/
};

enum {
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	FG_SOC,
	BATT_RESET_SOC,
	BATT_READ_RAW_SOC,
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */
	BATT_VOL,
	BATT_TEMP_CHECK,
	BATT_FULL_CHECK,
	BATT_CHARGING_SOURCE,
	AUTH,
	BATT_VF_ADC,
	CHG_CURRENT_ADC,
	BATT_TEMP_ADC,
	BATT_TYPE,
#ifdef __BATT_TEST_DEVICE__
	BATT_TEMP_TEST_ADC,
#endif
	CHARGINGBLOCK_CLEAR,
	BATT_SLATE_MODE,
	BATT_TEMP_DEGC,
	OFF_BACKLIGHT,
#if !defined(CONFIG_BATTERY_CTIA)
	BATT_WCDMA_CALL,
	BATT_GSM_CALL,
	BATT_DATACALL,
	BATT_CAMERA,
	BATT_BROWSER,
#else
	BATT_WCDMA_CALL,
	BATT_GSM_CALL,
	BATT_CALL,
	BATT_VIDEO,
	BATT_MUSIC,
	BATT_BROWSER,
	BATT_HOTSPOT,
	BATT_CAMERA,
	BATT_DATA_CALL,
	BATT_GPS,
	BATT_LTE,
	BATT_WIFI,
	BATT_USE,
#endif /*CONFIG_BATTERY_CTIA*/
	BATT_FACTORY,
};

static void  sec_bat_monitoring_alarm(int sec)
{
	ktime_t low_interval = ktime_set(sec - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(msm_batt_info.cur_monitor_time, low_interval);
	alarm_start_range(&msm_batt_info.alarm, next, ktime_add(next, slack));

	pr_info("%s: called\n", __func__);
}

static void sec_bat_monitor_queue(struct alarm *alarm)
{
	struct msm_battery_info *info =
		container_of(alarm, struct msm_battery_info, alarm);

	queue_delayed_work(info->msm_batt_wq,
		&info->msm_batt_work, 0);
}

#if defined(CONFIG_BATTERY_CTIA)
static void  sec_bat_program_alarm(int seconds)
{
	ktime_t low_interval = ktime_set(seconds - 10, 0);
	ktime_t slack = ktime_set(20, 0);
	ktime_t next;

	next = ktime_add(msm_batt_info.cur_time, low_interval);
	alarm_start_range(&msm_batt_info.event_alarm, next,
		ktime_add(next, slack));
}

static void sec_bat_use_timer_func(struct alarm *alarm)
{
	msm_batt_info.batt_use &= (~msm_batt_info.batt_use_wait);
	pr_info("/BATT_USE/ timer expired (0x%x)\n", msm_batt_info.batt_use);
}

static void sec_bat_use_module(int module, int enable)
{
	struct timespec ts;

	/* ignore duplicated deactivation of same event  */
	if (!enable && (msm_batt_info.batt_use ==
		msm_batt_info.batt_use_wait)) {
		pr_info("/BATT_USE/ ignore duplicated same event\n");
		return;
	}

	/*del_timer_sync(&msm_batt_info.bat_use_timer);*/
	alarm_cancel(&msm_batt_info.event_alarm);
	msm_batt_info.batt_use &= (~msm_batt_info.batt_use_wait);

	if (enable) {
		msm_batt_info.batt_use_wait = 0;
		msm_batt_info.batt_use |= module;

		/* debug msg */
		if (module == USE_CALL)
			pr_info("/BATT_USE/ call 0x%x\n",
				msm_batt_info.batt_use);
		else if (module == USE_VIDEO)
			pr_info("/BATT_USE/ video 0x%x\n",
				msm_batt_info.batt_use);
		else if (module == USE_MUSIC)
			pr_info("/BATT_USE/ music 0x%x\n",
				msm_batt_info.batt_use);
		else if (module == USE_BROWSER)
			pr_info("/BATT_USE/ browser 0x%x\n",
				msm_batt_info.batt_use);
		else if (module == USE_HOTSPOT)
			pr_info("/BATT_USE/ hotspot 0x%x\n",
				msm_batt_info.batt_use);
		else if (module == USE_CAMERA)
			pr_info("/BATT_USE/ camera 0x%x\n",
				msm_batt_info.batt_use);
		else if (module == USE_DATA_CALL)
			pr_info("/BATT_USE/ datacall 0x%x\n",
				msm_batt_info.batt_use);
		else if (module == USE_LTE)
			pr_info("/BATT_USE/ lte 0x%x\n",
				msm_batt_info.batt_use);
		else if (module == USE_WIFI)
			pr_info("/BATT_USE/ wifi 0x%x\n",
				msm_batt_info.batt_use);
	} else {
		if (msm_batt_info.batt_use == 0) {
			pr_info("/BATT_USE/ nothing to clear\n");
			return;	/* nothing to clear */
		}
		msm_batt_info.batt_use_wait = module;
		msm_batt_info.cur_time = alarm_get_elapsed_realtime();
		ts = ktime_to_timespec(msm_batt_info.cur_time);

		sec_bat_program_alarm(TOTAL_EVENT_TIME);
		pr_info("/BATT_USE/ start timer (curr 0x%x, wait 0x%x)\n",
			msm_batt_info.batt_use, msm_batt_info.batt_use_wait);
	}
}
#endif /*CONFIG_BATTERY_CTIA*/

static int msm_batt_create_attrs(struct device *dev)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(samsung_battery_attrs); i++) {
		rc = device_create_file(dev, &samsung_battery_attrs[i]);
		if (rc)
			goto failed;
	}
	goto succeed;

 failed:
	while (i--)
		device_remove_file(dev, &samsung_battery_attrs[i]);

 succeed:
	return rc;
}

static void msm_batt_remove_attrs(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(samsung_battery_attrs); i++)
		device_remove_file(dev, &samsung_battery_attrs[i]);
}

static ssize_t msm_batt_show_property(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i = 0;
	const ptrdiff_t offset = attr - samsung_battery_attrs;

	switch (offset) {
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	case FG_SOC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			get_level_from_fuelgauge());
		break;
	case BATT_READ_RAW_SOC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			fg_read_raw_soc());
		break;
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */
	case BATT_VOL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.battery_voltage);
		break;
	case BATT_TEMP_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.batt_temp_check);
		break;
	case BATT_FULL_CHECK:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.batt_full_check);
		break;
	case BATT_CHARGING_SOURCE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.charging_source);
		break;
	case AUTH:	/* vzw battery auth. */
		if (msm_batt_info.batt_health ==
			POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 0);
		else
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", 1);
		break;
	case BATT_VF_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			rep_batt_chg.v1.batt_id);
		break;
	case CHG_CURRENT_ADC: /* ICHG ADC code (charging current) */
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.chg_current_adc);
		break;
	case BATT_TEMP_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.battery_temp_adc);
		break;
	case BATT_TYPE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n", "SDI_SDI");
		break;
#ifdef __BATT_TEST_DEVICE__
	case BATT_TEMP_TEST_ADC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", temp_test_adc);
		break;
#endif
	case CHARGINGBLOCK_CLEAR:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.chargingblock_clear);
		break;
	case BATT_SLATE_MODE:
		/*FOR SLATE TEST Don't charge by USB */
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n"
		, msm_batt_info.batt_slate_mode);
		break;
	case BATT_TEMP_DEGC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.battery_temp_degc);
		break;
	case OFF_BACKLIGHT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.off_backlight);
		break;
#if defined(CONFIG_BATTERY_CTIA)
	case BATT_WCDMA_CALL:
	case BATT_GSM_CALL:
	case BATT_CALL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_CALL) ? 1 : 0);
		break;
	case BATT_VIDEO:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_VIDEO) ? 1 : 0);
		break;
	case BATT_MUSIC:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_MUSIC) ? 1 : 0);
		break;
	case BATT_BROWSER:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_BROWSER) ? 1 : 0);
		break;
	case BATT_HOTSPOT:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_HOTSPOT) ? 1 : 0);
		break;
	case BATT_CAMERA:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_CAMERA) ? 1 : 0);
		break;
	case BATT_DATA_CALL:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_DATA_CALL) ? 1 : 0);
		break;
	case BATT_GPS:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_GPS) ? 1 : 0);
		break;
	case BATT_LTE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & USE_GPS) ? 1 : 0);
		break;
	case BATT_WIFI:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			(msm_batt_info.batt_use & BATT_WIFI) ? 1 : 0);
		break;
	case BATT_USE:
		i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n",
			msm_batt_info.batt_use);
		break;
#endif
	default:
		i = -EINVAL;
	}

	return i;
}

static ssize_t msm_batt_store_property(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int x = 0;
	int ret = 0;
	const ptrdiff_t offset = attr - samsung_battery_attrs;

	switch (offset) {
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	case BATT_RESET_SOC:
		if (sscanf(buf, "%d\n", &x) == 1) {
			fg_reset_soc();
			if (!is_msm_batt_updating) {
				/*updating status after batt_reset_soc*/
				pr_info("%s: fg_reset_soc update", __func__);
				msm_batt_update_psy_status();
			}
			ret = count;
			pr_info("%s: BATT_RESET_SOC case", __func__);
		}
		break;
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */
#ifdef __BATT_TEST_DEVICE__
	case BATT_TEMP_TEST_ADC:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x == 0)
				temp_test_adc = 0;
			else
				temp_test_adc = x;
			ret = count;
		}
		break;
#endif
	case CHARGINGBLOCK_CLEAR:
		if (sscanf(buf, "%d\n", &x) == 1) {
			pr_debug("\n[BATT] %s: chargingblock_clear -> write 0x%x\n\n",
				__func__, x);
			msm_batt_info.chargingblock_clear = x;
			ret = count;
		}
		break;

	case BATT_SLATE_MODE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			msm_batt_info.batt_slate_mode = x;
			msm_batt_cable_status_update();
			msm_batt_update_psy_status();
			ret = count;
		}
		break;

	case OFF_BACKLIGHT:
		if (sscanf(buf, "%d\n", &x) == 1) {
			msm_batt_info.off_backlight = x;
			ret = count;
		}
		break;

#if !defined(CONFIG_BATTERY_CTIA)
	case BATT_WCDMA_CALL:
	case BATT_GSM_CALL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			msm_batt_info.voice_call_state = x;
			pr_info("%s : voice call = %d, %d\n", __func__,
				x, msm_batt_info.voice_call_state);

			if ((msm_batt_info.batt_status ==
			     POWER_SUPPLY_STATUS_CHARGING)
				|| (msm_batt_info.batt_recharging == 1)) {
				chg_start_time_event = 1;
				msm_batt_chg_en(START_CHARGING);
				chg_start_time_event = 0;
			}
		}
		break;
	case BATT_DATACALL:
		if (sscanf(buf, "%d\n", &x) == 1)
			dev_dbg(msm_batt_info.dev, "%s : data call = %d\n",
				__func__, x);
		break;
	case BATT_CAMERA:
		if (sscanf(buf, "%d\n", &x) == 1)
			ret = count;
		break;
	case BATT_BROWSER:
		if (sscanf(buf, "%d\n", &x) == 1)
			ret = count;
		break;
#else /*CONFIG_BATTERY_CTIA*/
	case BATT_WCDMA_CALL:
	case BATT_GSM_CALL:
	case BATT_CALL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_CALL, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: call = %d\n", __func__, x);
		break;
	case BATT_VIDEO:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_VIDEO, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: video = %d\n", __func__, x);
		break;
	case BATT_MUSIC:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_MUSIC, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: music = %d\n", __func__, x);
		break;
	case BATT_BROWSER:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_BROWSER, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: browser = %d\n", __func__, x);
		break;
	case BATT_HOTSPOT:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_HOTSPOT, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: hotspot = %d\n", __func__, x);
		break;
	case BATT_CAMERA:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_CAMERA, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: camera = %d\n", __func__, x);
		break;
	case BATT_DATA_CALL:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_DATA_CALL, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: data call = %d\n", __func__, x);
		break;
	case BATT_GPS:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_GPS, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: gps = %d\n", __func__, x);
		break;
	case BATT_LTE:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_LTE, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: LTE = %d\n", __func__, x);
		break;
	case BATT_WIFI:
		if (sscanf(buf, "%d\n", &x) == 1) {
			sec_bat_use_module(USE_WIFI, x);
			ret = count;
		}
		pr_debug("[BAT]:%s: WIFI = %d\n", __func__, x);
		break;
#endif

	case BATT_FACTORY:
		if (sscanf(buf, "%d\n", &x) == 1) {
			if (x)
				msm_batt_info.factory_mode = true;
			else
				msm_batt_info.factory_mode = false;
			ret = count;
		}
		pr_info("[BATT]:%s: factory mode = %d\n", __func__, x);
		break;

	default:
		return -EINVAL;
	}	/* end of switch */

	return ret;
}

static int msm_power_get_property(struct power_supply *psy,
					  enum power_supply_property psp,
					  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			val->intval = msm_batt_info.charging_source & AC_CHG
				? 1 : 0;
		}
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			if (msm_batt_info.batt_slate_mode)
				msm_batt_info.charging_source = NO_CHG;
			val->intval = msm_batt_info.charging_source & USB_CHG
				? 1 : 0;
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_MAX17043_FUEL_GAUGE
static u32 get_voltage_from_fuelgauge(void)
{
	if (is_attached)
		return fg_read_vcell();
	return 3700;	/* default */
}

static u32 get_level_from_fuelgauge(void)
{
	if (is_attached)
		return fg_read_soc();
	return 100;	/* default */
}
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */

static int msm_batt_power_get_property(struct power_supply *psy,
					       enum power_supply_property psp,
					       union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = msm_batt_info.batt_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = msm_batt_info.batt_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = msm_batt_info.batt_valid;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = msm_batt_info.batt_technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = msm_batt_info.voltage_max_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = msm_batt_info.voltage_min_design;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: /* *1000 for #0228# */
		val->intval = msm_batt_info.battery_voltage * 1000;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = msm_batt_info.batt_capacity;
		break;
	case POWER_SUPPLY_PROP_TEMP:	/* in celcius */
		val->intval = msm_batt_info.battery_temp_degc;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

#ifdef CONFIG_MAX17043_FUEL_GAUGE

static int fg_set_alert(int value)
{
	if (value) {
		if (msm_batt_info.charging_source == NO_CHG) {
			pr_info("[BATT] %s: low battery, power off...\n",
				__func__);
			is_alert = 1;
		} else {
			is_alert = 0;
		}
	} else {
		/* clear alert flag */
		is_alert = 0;
	}

	return is_alert;
}

#endif	/* CONFIG_MAX17043_FUEL_GAUGE */

static void msm_batt_chg_en(chg_enable_type enable)
{
	ktime_t current_time;
	struct timespec ts;

	current_time = alarm_get_elapsed_realtime();
	ts = ktime_to_timespec(current_time);

	msm_batt_info.check_interval = CHECK_DSG_INTERVAL;
	if (enable == START_CHARGING) {
		if (msm_batt_info.charging_source == NO_CHG) {
			/* *Note: DO NOT USE "&" operation for NO_CHG (0x0),
			 * * it returns FALSE always. */
			pr_err("[BATT] %s: charging_source not defined!\n",
				__func__);
			return;
		}
		/* Set charging current (ICHG; mA) */
		if (msm_batt_info.charging_source & AC_CHG) {
			pr_info("[BATT] %s: Start charging! (charging_source = AC)\n",
				__func__);
			msm_batt_info.check_interval =
				CHECK_CHG_INTERVAL;

		#if !defined(CONFIG_CHARGER_SMB328A) /*vital2 refresh*/
			hsusb_chg_connected(USB_CHG_TYPE__WALLCHARGER);
			hsusb_chg_vbus_draw(600);
		#else
		#if defined(CONFIG_MACH_ICON)
			smb328a_charger_control(900, 150, 1, 0);
		#elif defined(CONFIG_MACH_RAY)
			smb328a_charger_control(1000, 75, 1, 0);
		#else /*PREVAIL2, RAY*/
			smb328a_charger_control(1000, 150, 1, 0);
		#endif
		#endif
		} else {	/* USB_CHG */
			pr_info("[BATT] %s: Start charging! (charging_source = USB)\n",
				__func__);

			msm_batt_info.check_interval =
				CHECK_CHG_INTERVAL;

			if (msm_batt_info.cable_status == CABLE_TYPE_CDP) {
			#if !defined(CONFIG_CHARGER_SMB328A)
				hsusb_chg_connected
					(USB_CHG_TYPE__WALLCHARGER);
				hsusb_chg_vbus_draw(600);
			#else
				#if defined(CONFIG_MACH_ICON)
				smb328a_charger_control(900, 150, 1, 1);
				#elif defined(CONFIG_MACH_RAY)
				smb328a_charger_control(1000, 75, 1, 1);
				#else /*PREVAIL2, RAY*/
				smb328a_charger_control(1000, 150, 1, 1);
				#endif
			#endif
			} else {
			#if !defined(CONFIG_CHARGER_SMB328A)
				hsusb_chg_connected
					(USB_CHG_TYPE__SDP);
				hsusb_chg_vbus_draw(450);
				/* USB charging   (400mA) */
			#else
				#if defined(CONFIG_MACH_ICON)
				smb328a_charger_control(500, 150, 1, 0);
				#elif defined(CONFIG_MACH_RAY)
				smb328a_charger_control(500, 75, 1, 0);
				#else
				smb328a_charger_control(500, 150, 1, 0);
				#endif
			#endif
			}
		}
	#ifdef CONFIG_MAX17043_FUEL_GAUGE
		#if defined(CONFIG_MACH_ICON)
		max17040_rcomp_update(msm_batt_info.battery_temp_degc);
		#else
		fuel_gauge_rcomp(START_CHARGING);
		#endif
		if ((chg_start_time_event == 0) &&
			(msm_batt_info.charging_start_time == 0)) {
			msm_batt_info.charging_start_time = ts.tv_sec;
		}
	#else
		msm_batt_info.charging_start_time = ts.tv_sec;
	#endif

	#ifdef CONFIG_MAX17043_FUEL_GAUGE
		fg_set_alert(0);
	#endif	/* CONFIG_MAX17043_FUEL_GAUGE */

	} else {	/* STOP_CHARGING */
		msm_batt_info.charging_passed_time = 0;
		msm_batt_info.charging_start_time = 0;

	#if !defined(CONFIG_CHARGER_SMB328A) /*vital2 refresh*/
		if (msm_batt_info.charging_source == NO_CHG)/* not charging */
			hsusb_chg_connected(USB_CHG_TYPE__INVALID);
		else
			hsusb_chg_vbus_draw(0);	/* discharging */
	#else
		#if defined(CONFIG_MACH_ICON)
		smb328a_charger_control(0, 150, 0, 0);
		#elif defined(CONFIG_MACH_RAY)
		smb328a_charger_control(0, 75, 0, 0);
		#else
		smb328a_charger_control(0, 150, 0, 0);
		#endif
	#endif
		msm_batt_average_chg_current(-1);
		/* Initialize all current data sampling */

		pr_info("[BATT] %s: Stop charging! "\
			"(charging_source = 0x%x, full_check = %d)\n",\
			__func__, msm_batt_info.charging_source,
			     msm_batt_info.batt_full_check);

#ifdef CONFIG_MAX17043_FUEL_GAUGE
	#if !defined(CONFIG_MACH_ICON)
		fuel_gauge_rcomp(STOP_CHARGING);
	#endif
#endif
	}
}

static int msm_batt_average_chg_current(int chg_current_adc)
{
	static int history[AVERAGE_COUNT] = { 0 };
	static int count;
	static int index;
	int i, j, sum, ret;
	int max[2], min[2];
	int min_index[2], max_index[2];

	if (chg_current_adc == 0)
		return 0;

	if (chg_current_adc < 0) {	/* initialize all data */
		count = 0;
		index = 0;
		for (i = 0; i < AVERAGE_COUNT; i++)
			history[i] = 0;

		return 0;
	}

	if (index >= count)
		count++;

	/*max = min = history[0];*/
	max[0] = min[0] = 0;
	max[1] = min[1] = 0;
	sum = 0;

	if (count < AVERAGE_COUNT) {
		for (i = 0; i < count; i++) {
			if (i == index)
				history[i] = chg_current_adc;

			sum += history[i];
		}

		ret = (sum / count);
	} else {
		for (i = 0; i < AVERAGE_COUNT; i++) {
			if (i == index)
				history[i] = chg_current_adc;

			sum += history[i];
		}

		for (j = 0; j < 2; j++) {
			min[j] = 1000000;
			max[j] = 0;

			for (i = 0; i < AVERAGE_COUNT; i++) {
				if (j > 0) { /* Pass first max & min */
					if ((i == min_index[0]) ||
						(i == max_index[0]))
						continue;
				}

				if (max[j] < history[i]) {
					max[j] = history[i];
					max_index[j] = i;
				}
				if (min[j] > history[i]) {
					min[j] = history[i];
					min_index[j] = i;
				}
			}
			sum = sum-max[j]-min[j];
		}
		ret = (sum / (AVERAGE_COUNT-4));
	}

	index++;
	if (index == AVERAGE_COUNT)
		index = 0;

	pr_debug("[BATT] %s: adc=%d, sum=%d, ret=%d\n",
		__func__, chg_current_adc, sum, ret);
	pr_debug("%s: max[0]=%d, max[1]=%d, min[0]=%d, min[1]=%d\n",
		__func__, max[0], max[1], min[0], min[1]);

	return ret;
}

static int msm_batt_check_full_charging(int chg_current_adc)
{
	unsigned long charging_time;
	ktime_t	current_time;
	struct timespec ts;
	bool full_current_condtion;
	int f_cond_soc;

	f_cond_soc = 95;

	current_time = alarm_get_elapsed_realtime();
	ts = ktime_to_timespec(current_time);

#if defined(CONFIG_MACH_ICON) || defined(CONFIG_MACH_PREVAIL2)\
|| defined(CONFIG_MACH_RAY)
	full_current_condtion = smb328a_check_bat_full();
	pr_info("%s: full_current_condtion = %d\n",
		__func__, full_current_condtion);
#endif

	if ((chg_current_adc == 0) ||
		(msm_batt_info.charging_source == NO_CHG)) {
		msm_batt_info.first_full_count = 0;
		return 0;	/* not charging */
	}

	if (ts.tv_sec >= msm_batt_info.charging_start_time) {
		charging_time = ts.tv_sec - msm_batt_info.charging_start_time;
	} else {
		charging_time = 0xFFFFFFFF - msm_batt_info.charging_start_time +
					ts.tv_sec;
	}

	msm_batt_info.charging_passed_time = charging_time;
	pr_info("%s: start_time = %lu, chg_passed_time = %lu\n", __func__,
		msm_batt_info.charging_start_time,
		msm_batt_info.charging_passed_time);

	/* check charging absolute time */
	if (msm_batt_is_over_abs_time()) {
		pr_info("[BATT] %s: Fully charged, over abs time! (recharging=%d)\n",
			__func__, msm_batt_info.batt_recharging);
		msm_batt_info.batt_full_check = 1;
		msm_batt_info.batt_recharging = 0;
		msm_batt_info.check_full_state = true;
		msm_batt_info.is_timeout_chgstop = true;
		msm_batt_info.full_check_count = 0;
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_FULL;
		msm_batt_info.full_charged_state = STATUS_ABS_FULL;
		msm_batt_chg_en(STOP_CHARGING);
		return 1;
	}

#if defined(CONFIG_MACH_ICON) || defined(CONFIG_MACH_PREVAIL2)\
|| defined(CONFIG_MACH_RAY)
	if ((msm_batt_info.battery_voltage >= BATT_FULL_CHARGING_VOLTAGE)
		&& (msm_batt_info.batt_capacity >= f_cond_soc)) {
	#if defined(USE_2STEP_TERM)
		if (msm_batt_info.full_charged_state == STATUS_NOT_FULL) {
			switch (msm_batt_info.full_check_type) {
			case FULLCHARGED_ADC:
				if (msm_batt_info.chg_current_adc <=
					BATT_FULL_CHARGING_CURRENT) {
					msm_batt_info.first_full_count++;
					if (msm_batt_info.first_full_count
						>= 3) {
						pr_info("%s: 1st full(%d<=%d)\n",
						__func__,
						msm_batt_info.chg_current_adc,
						BATT_FULL_CHARGING_CURRENT);
						msm_batt_info.full_charged_state
							= STATUS_1ST_FULL;
						msm_batt_info.first_full_count
							= 0;
					} else {
						pr_debug("%s: full count(%d)\n",
						__func__,
						msm_batt_info.first_full_count);
					}
				} else {
					msm_batt_info.first_full_count = 0;
					pr_debug("%s: high chg_current_adc(%d)\n",
						__func__,
						msm_batt_info.chg_current_adc);
				}
				break;
			case FULLCHARGED_SOC:/*max 100%*/
				if (msm_batt_info.batt_capacity >= 99) {
					msm_batt_info.first_full_count++;
					if (msm_batt_info.first_full_count
						>= 3) {
						pr_info("%s: 1st full by soc (%d)\n",
						__func__,
						msm_batt_info.batt_capacity);
						msm_batt_info.full_charged_state
							= STATUS_1ST_FULL;
						msm_batt_info.first_full_count
							= 0;
					} else {
						pr_debug("%s: full count(%d)\n",
						__func__,
						msm_batt_info.first_full_count);
					}
				} else {
					msm_batt_info.first_full_count = 0;
				}
				break;
			default:
				pr_err("%s: Invalid Full Check\n", __func__);
				break;
			}

			if (msm_batt_info.full_charged_state ==
				STATUS_1ST_FULL) {
				msm_batt_info.batt_full_check = 1;
				msm_batt_info.batt_recharging = 0;
				msm_batt_info.check_full_state = true;
				msm_batt_info.is_timeout_chgstop = false;
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_FULL;
				msm_batt_info.full_check_count = 0;
				return 1;
			}
			return 0;
		} else if (msm_batt_info.full_charged_state ==
			STATUS_1ST_FULL) {
			if (full_current_condtion == true) {
				pr_info("[BATT] %s: 2nd full charged, "\
					"cut off charging current! "\
					"(voltage=%d, ICHG=%d)\n",
					__func__, msm_batt_info.battery_voltage,
					chg_current_adc);
				msm_batt_info.full_charged_state =
					STATUS_2ND_FULL;
				msm_batt_info.is_timeout_chgstop = false;
				msm_batt_chg_en(STOP_CHARGING);
			} else {
				pr_debug("%s: 1st full but not 2nd full\n",
					__func__);
			}
			return 0;
		}

		/* Recharging after abs/real full */
		if (full_current_condtion == true) {
			pr_info("%s: fully charged w/o 1st full"\
				"(f_state: %d, V: %d, I=%d, soc=%d), r_c(%d)\n",
				__func__, msm_batt_info.full_charged_state,
				msm_batt_info.battery_voltage,
				chg_current_adc, msm_batt_info.batt_capacity,
				msm_batt_info.batt_recharging);
			msm_batt_info.batt_full_check = 1;
			msm_batt_info.batt_recharging = 0;
			msm_batt_info.check_full_state = true;
			msm_batt_info.is_timeout_chgstop = false;
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_FULL;
			msm_batt_info.full_check_count = 0;
			msm_batt_info.full_charged_state =
					STATUS_2ND_FULL;
			msm_batt_info.first_full_count = 0;
			msm_batt_chg_en(STOP_CHARGING);
			return 1;
		}
	#else
		if (full_current_condtion == true) {
			/* fully charged !*/
			pr_info("[BATT] %s: Fully charged, "\
				"cut off charging current! "\
				"(voltage=%d, ICHG=%d)\n",
				__func__, msm_batt_info.battery_voltage,
				chg_current_adc);
			msm_batt_info.batt_full_check = 1;
			msm_batt_info.batt_recharging = 0;
			msm_batt_info.check_full_state = true;
			msm_batt_info.is_timeout_chgstop = false;
			msm_batt_info.batt_status =
				    POWER_SUPPLY_STATUS_FULL;
			msm_batt_info.full_check_count = 0;
			msm_batt_chg_en(STOP_CHARGING);
			return 1;
		} else {
			pr_debug("%s: over current condition(V=%d, I=%d)\n",
			__func__, msm_batt_info.battery_voltage,
			chg_current_adc);
		}
	#endif
	} else {
		pr_debug("[BATT] %s: voltage = %d(< %d)\n",
			__func__, msm_batt_info.battery_voltage,
			BATT_FULL_CHARGING_VOLTAGE);
	}
#else
	if (msm_batt_info.battery_voltage >= BATT_FULL_CHARGING_VOLTAGE) {
		/* check charging current threshold */
		if (chg_current_adc < BATT_FULL_CHARGING_CURRENT) {
			msm_batt_info.full_check_count++;

			if (msm_batt_info.full_check_count >= 3) {
					/* fully charged !*/
					pr_info("[BATT] %s: Fully charged, "\
					"cut off charging current! "\
					"(voltage=%d, ICHG=%d)\n",
					__func__, msm_batt_info.battery_voltage,
					chg_current_adc);
					msm_batt_info.batt_full_check = 1;
					msm_batt_info.batt_recharging = 0;
					msm_batt_info.check_full_state = true;
					msm_batt_info.is_timeout_chgstop
						= false;
					msm_batt_info.batt_status =
						    POWER_SUPPLY_STATUS_FULL;
					msm_batt_info.full_check_count = 0;
					msm_batt_chg_en(STOP_CHARGING);
					return 1;
			}
		} else {
			msm_batt_info.full_check_count = 0;
		}
	} else {
		msm_batt_info.full_check_count = 0;
	}
#endif

	return 0;
}

static int msm_batt_check_recharging(void)
{
	if ((msm_batt_info.batt_full_check == 0) ||
		(msm_batt_info.batt_recharging == 1) ||
		(msm_batt_info.batt_health != POWER_SUPPLY_HEALTH_GOOD)) {
		msm_batt_info.recharging_count = 0;
		return 0;
	}

	#if defined(USE_2STEP_TERM)
	if (msm_batt_info.full_charged_state < STATUS_2ND_FULL) {
		msm_batt_info.recharging_count = 0;
		return 0;
	}
	#endif

	/* check 1st voltage */
	if (msm_batt_info.battery_voltage <= BATT_RECHARGING_VOLTAGE_1) {
		pr_info("[BATT] %s: Recharging ! (voltage1 = %d)\n",
			__func__, msm_batt_info.battery_voltage);
		msm_batt_info.recharging_count++;
		if (msm_batt_info.recharging_count >= 3) {
			msm_batt_info.batt_recharging = 1;
			msm_batt_chg_en(START_CHARGING);
			return 1;
		}
		return 0;
	} else {
		msm_batt_info.recharging_count = 0;
		return 0;
	}

	/* check 2nd voltage */
	if (msm_batt_info.battery_voltage <= BATT_RECHARGING_VOLTAGE_2) {
		pr_info("[BATT] %s: Recharging ! (voltage2 = %d)\n",
			__func__, msm_batt_info.battery_voltage);

		msm_batt_info.batt_recharging = 1;
		msm_batt_chg_en(START_CHARGING);
		return 1;
	}

	return 0;
}

static int msm_batt_check_level(int battery_level)
{
	int or_battery_level;

	or_battery_level = battery_level;

	if (msm_batt_info.batt_full_check &&
		!msm_batt_info.is_timeout_chgstop) {
		battery_level = 100;
	} else if ((battery_level == 0)
#ifdef CONFIG_MAX17043_FUEL_GAUGE
		   && (is_alert == 0)
#endif		/* CONFIG_MAX17043_FUEL_GAUGE */
	    ) {
		battery_level = 1;
		/* not yet alerted low battery (do not power off yet) */
	}

/*
 *	if (msm_batt_info.battery_voltage< msm_batt_info.voltage_min_design)
 *	{
 *		battery_level = 0;
 *	}
*/

	/* Additional power off conditoin for low battery */
	if ((msm_batt_info.battery_voltage <= BATTERY_LOW) &&
		(or_battery_level == 0) && (msm_batt_info.batt_status !=
		POWER_SUPPLY_STATUS_CHARGING)) {
			pr_info("%s: PWR off(batt_v = %d(<=%d), or_soc = %d, "\
				"batt_status = %d)\n",
				__func__, msm_batt_info.battery_voltage,
				BATTERY_LOW, or_battery_level,
				msm_batt_info.batt_status);
			battery_level = 0;
	}

	if (msm_batt_info.check_full_state == true &&
		msm_batt_info.batt_status == POWER_SUPPLY_STATUS_DISCHARGING
		&& !msm_batt_info.is_timeout_chgstop) {
		if (msm_batt_info.check_full_state_cnt <= 1) {
			battery_level = 100;
			msm_batt_info.check_full_state_cnt++;
		} else {
			if (msm_batt_info.batt_presoc > or_battery_level) {
				battery_level = msm_batt_info.batt_presoc - 1;
			} else {
				msm_batt_info.check_full_state = false;
				msm_batt_info.check_full_state_cnt = 0;
			}
		}
	}

	if (msm_batt_info.batt_capacity != battery_level) {
		pr_info("[BATT] %s: Battery level changed ! (%d -> %d)\n",
			__func__, msm_batt_info.batt_capacity, battery_level);
		msm_batt_info.batt_capacity = battery_level;
		return 1;
	}

	if (msm_batt_info.battery_voltage <= BATTERY_LOW)
		return 1;

#ifdef CONFIG_MAX17043_FUEL_GAUGE
	if (is_alert)
		return 1;	/* force update to power off ! */
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */

	return 0;
}

static int msm_batt_average_temperature(int temp_adc)
{
	static int history[AVERAGE_COUNT] = { 0 };
	static int count;
	static int index;

	int i, sum, max, min, ret;

	if (temp_adc == 0)
		return 0;

#ifdef __BATT_TEST_DEVICE__
	if (temp_test_adc)
		return temp_test_adc;
#endif

	/*if (temp_adc < 0)	// initialize all data
	{
		count = 0;
		index = 0;
		for (i=0; i<AVERAGE_COUNT; i++)
			history[i] = 0;

		return 0;
	}*/

	if (index >= count)
		count++;

	max = min = 0;
	sum = 0;

	if (count < AVERAGE_COUNT) {
		for (i = 0; i < count; i++)	{
			if (i == index)
				history[i] = temp_adc;

			sum += history[i];
		}

		ret = (sum / count);
	} else {
		max = min = history[AVERAGE_COUNT/2];

		for (i = 0; i < AVERAGE_COUNT; i++)	{
			if (i == index)
				history[i] = temp_adc;

			if (max < history[i])
				max = history[i];
			if (min > history[i])
				min = history[i];

			sum += history[i];
		}

		ret = ((sum-max-min) / (AVERAGE_COUNT-2));
	}

	index++;
	if (index == AVERAGE_COUNT)
		index = 0;

	pr_debug("[BATT] %s: adc=%d, sum=%d, max=%d, min=%d, ret=%d\n",
		__func__, temp_adc, sum, max, min, ret);

	return ret;
}

#ifdef CONFIG_LEDS_PMIC8058

#define LED_NOT_CHARGING		0x01
#define LED_IN_CHARGING		0x02
#define LED_TEMPERATURE_BLOCK	0x03
#define LED_FULL_CHARGING	0x04
#define LED_ON			6
#define LED_OFF			0

static int led_toggle = 1;
static int new_led_status;
static int pre_led_status;

#ifndef BATT_LED_CONTROL_DISABLE
static void msm_batt_led_control(void)
{
	int rc1 = 0, rc2 = 0;

#if defined(CONFIG_LEDS_TRIGGER_NOTIFICATION)
	/* If we are using notifications, anroid platform should have
	 * full control over the LED device. But since we are not using
	 * the android platform for off charging mode, let we can have
	 * full access to the LED device. */
	if (!charging_boot)
		return;
#endif
	if (msm_batt_info.charger_type != CHARGER_TYPE_NONE) {
		if ((msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_OVERHEAT)
			|| (msm_batt_info.batt_health ==
			POWER_SUPPLY_HEALTH_COLD)) {
			/* blocked temperature */
			new_led_status = LED_TEMPERATURE_BLOCK;
		} else if ((msm_batt_info.batt_full_check == 1)
			   || (msm_batt_info.batt_recharging == 1)) {
			/* full charging */
			new_led_status = LED_FULL_CHARGING;
		} else if (msm_batt_info.batt_health ==
			POWER_SUPPLY_HEALTH_GOOD) {
			/* charging */
			new_led_status = LED_IN_CHARGING;
		} else if (msm_batt_info.batt_health ==
			   POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) {
			/* V_F Block */
			new_led_status = LED_NOT_CHARGING;
		}
	} else {
		/*not connected */
		new_led_status = LED_NOT_CHARGING;
	}

	if (pre_led_status != new_led_status) {
		pr_info("[BATT] Charging LED Status : %d\n", new_led_status);
		hrtimer_try_to_cancel(&LedTimer);
		switch (new_led_status) {
		case LED_NOT_CHARGING:
			rc1 = pm8058_set_led_current(PMIC8058_ID_LED_1,
				LED_OFF);
			rc2 = pm8058_set_led_current(PMIC8058_ID_LED_2,
				LED_OFF);
			break;

		case LED_IN_CHARGING:
			rc1 = pm8058_set_led_current(PMIC8058_ID_LED_1,
				LED_OFF);
			rc2 = pm8058_set_led_current(PMIC8058_ID_LED_2, LED_ON);
			break;

		case LED_TEMPERATURE_BLOCK:
			hrtimer_start(&LedTimer, ktime_set(1, 0),
				HRTIMER_MODE_REL);
			break;

		case LED_FULL_CHARGING:
			rc1 = pm8058_set_led_current(PMIC8058_ID_LED_1, LED_ON);
			rc2 = pm8058_set_led_current(PMIC8058_ID_LED_2,
				LED_OFF);
			break;

		default:
			break;
		}
		pre_led_status = new_led_status;
		if (rc1 < 0 || rc2 < 0)
			pr_err("%s: pm8058_set_led_current FAIL, rc1 = %d, rc2 = %d\n",
			__func__, rc1, rc2);
	}
}
#endif

static enum hrtimer_restart led_timer_func(struct hrtimer *timer)
{
	if (led_toggle) {
		pm8058_set_led_current(PMIC8058_ID_LED_1, LED_ON);
		pm8058_set_led_current(PMIC8058_ID_LED_2, LED_ON);
		led_toggle = 0;
	} else {
		pm8058_set_led_current(PMIC8058_ID_LED_1, LED_OFF);
		pm8058_set_led_current(PMIC8058_ID_LED_2, LED_OFF);
		led_toggle = 1;
	}
	hrtimer_start(timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

#endif

int get_voice_call_status(void)
{
	return msm_batt_info.voice_call_state;
}
EXPORT_SYMBOL(get_voice_call_status);

static int msm_batt_control_temperature(u32 temp_adc)
{
	int prev_health = msm_batt_info.batt_health;
	int new_health = prev_health;
	int array_size = 0;
	int i;
	int degree = 0;

	int high_block;
	int high_recover;
	int low_block;
	int low_recover;

	static char *health_text[] = {
		"Unknown", "Good", "Overheat", "Dead", "Over voltage",
		"Unspecified failure", "Cold",
	};

#define HIGH_BLOCK	0
#define LOW_BLOCK	1
#define HIGH_RECOVERY	2
#define LOW_RECOVERY	3

	if (charging_boot == 1) {/*lpm mode*/
		high_block = BATT_TEMP_HIGH_BLOCK_LPM;
		high_recover = BATT_TEMP_HIGH_RECOVER_LPM;
		low_block = BATT_TEMP_LOW_BLOCK_LPM;
		low_recover = BATT_TEMP_LOW_RECOVER_LPM;
	} else {
		high_block = BATT_TEMP_HIGH_BLOCK;
		high_recover = BATT_TEMP_HIGH_RECOVER;
		low_block = BATT_TEMP_LOW_BLOCK;
		low_recover = BATT_TEMP_LOW_RECOVER;
	}

	if (temp_adc == 0)
		return 0;

#ifdef __AUTO_TEMP_TEST__
	static unsigned int auto_test_start_time;
	static unsigned int auto_test_interval = (2 * TIME_UNIT_MINUTE);
	static int auto_test_mode;
	/* 0: normal (recover cold), 1: force overheat,
	 * 2: normal (recover overheat), 3: force cold */

	ktime_t	current_time;
	struct timespec ts;

	current_time = alarm_get_elapsed_realtime();
	ts = ktime_to_timespec(current_time);

	if (msm_batt_info.charging_source != NO_CHG) {	/* charging */
		if (auto_test_start_time == 0)
			auto_test_start_time = ts.tv_sec;

		if (ts.tv_sec > auto_test_start_time + auto_test_interval) {
			auto_test_mode++;
			if (auto_test_mode > 3)
				auto_test_mode = 0;
			auto_test_start_time = ts.tv_sec;
		}
		pr_debug("[BATT] auto test mode = %d (0:normal,1:overheat,2:normal,3:cold)\n",
			auto_test_mode);

		if (auto_test_mode == 1) {
			temp_adc = high_block + 10;
			msm_batt_info.battery_temp_adc = temp_adc;
		} else if (auto_test_mode == 3) {
			temp_adc = low_block - 10;
			msm_batt_info.battery_temp_adc = temp_adc;
		}
	} else {	/* not chargin*/
		auto_test_start_time = 0;
		auto_test_mode = 0;
	}
#endif
	/* map in celcius degree */
	array_size = ARRAY_SIZE(temp_table);
	for (i = 0; i < (array_size - 1); i++) {
		if (i == 0) {
			if (temp_adc <= temp_table[0][0]) {
				degree = temp_table[0][1];
				break;
			} else if (temp_adc >= temp_table[array_size - 1][0]) {
				degree = temp_table[array_size - 1][1];
				break;
			}
		}

		if (temp_table[i][0] < temp_adc &&
			temp_table[i + 1][0] >= temp_adc)
			degree = temp_table[i + 1][1];
	}
	/* pr_info("[BATT] degree %d\n",degree ); */

	msm_batt_info.battery_temp_degc = degree;	/* celcius degree */
	msm_batt_info.battery_temp_adc = temp_adc;	/*code */

	/* TODO:  check application */

	if (prev_health == POWER_SUPPLY_HEALTH_UNSPEC_FAILURE)
		return 0;
		/* do not check temperature... (charging is already blocked!) */

	if (msm_batt_info.charging_source != NO_CHG) {
		if (temp_adc >= high_block) {
		#if defined(CONFIG_BATTERY_CTIA)
			if (msm_batt_info.batt_use && !charging_boot) {
				if (temp_adc >= BATT_TEMP_EVENT_BLOCK) {
					if (prev_health !=
						POWER_SUPPLY_HEALTH_OVERHEAT)
						new_health =
						POWER_SUPPLY_HEALTH_OVERHEAT;
					msm_batt_info.batt_full_check = 0;
					msm_batt_info.batt_recharging = 0;
					msm_batt_info.full_charged_state =
						STATUS_NOT_FULL;
				} else {
					if (prev_health !=
						POWER_SUPPLY_HEALTH_OVERHEAT)
						new_health =
						    POWER_SUPPLY_HEALTH_GOOD;
				}
			} else {
				/* over high block */
				if (prev_health != POWER_SUPPLY_HEALTH_OVERHEAT)
					new_health =
						POWER_SUPPLY_HEALTH_OVERHEAT;
				msm_batt_info.batt_full_check = 0;
				msm_batt_info.batt_recharging = 0;
				msm_batt_info.full_charged_state =
					STATUS_NOT_FULL;
				}
		#else
			/* over high block */
			if (prev_health != POWER_SUPPLY_HEALTH_OVERHEAT)
				new_health = POWER_SUPPLY_HEALTH_OVERHEAT;
			msm_batt_info.batt_full_check = 0;
			msm_batt_info.batt_recharging = 0;
			msm_batt_info.full_charged_state = STATUS_NOT_FULL;
		#endif
		} else if ((temp_adc <= high_recover)
			&& (temp_adc >= low_recover)) {
				/* low recover ~ high recover (normal) */
			if ((prev_health ==
					POWER_SUPPLY_HEALTH_OVERHEAT)
				|| (prev_health ==
					POWER_SUPPLY_HEALTH_COLD))
					new_health =
						POWER_SUPPLY_HEALTH_GOOD;
		} else if (temp_adc <=	low_block) {
			/* under low block */
			if (prev_health != POWER_SUPPLY_HEALTH_COLD)
				new_health = POWER_SUPPLY_HEALTH_COLD;
			msm_batt_info.batt_full_check = 0;
			msm_batt_info.batt_recharging = 0;
			msm_batt_info.full_charged_state = STATUS_NOT_FULL;
		}

		if (msm_batt_info.charging_source == NO_CHG) {
			if ((low_block < temp_adc)
				&& (temp_adc < high_block)) {
				if ((prev_health ==
					POWER_SUPPLY_HEALTH_OVERHEAT)
					|| (prev_health ==
					POWER_SUPPLY_HEALTH_COLD))
					new_health = POWER_SUPPLY_HEALTH_GOOD;
			}
		}

		if (prev_health != new_health) {
			if (msm_batt_info.charging_source == NO_CHG) {
				/* not charging */
				pr_info("[BATT] %s: "\
					"Health changed by temperature!"\
					"(ADC = %d, %s-> %s)\n", __func__,
					temp_adc, health_text[prev_health],
					health_text[new_health]);
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_DISCHARGING;
			} else {	/* in charging */
				if (new_health != POWER_SUPPLY_HEALTH_GOOD) {
					/* block! */
					pr_info("[BATT] %s: Block charging!"\
						"(ADC = %d, %s-> %s)\n",
						__func__, temp_adc,
						health_text[prev_health],
						health_text[new_health]);
					msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
					msm_batt_chg_en(STOP_CHARGING);
				} else if (msm_batt_info.batt_full_check == 0) {
					/* recover!*/
					pr_info("[BATT] %s: Recover charging!"\
						"(ADC = %d, %s-> %s)\n",
						__func__, temp_adc,
						health_text[prev_health],
						health_text[new_health]);
					msm_batt_info.batt_status =
						POWER_SUPPLY_STATUS_CHARGING;
					msm_batt_chg_en(START_CHARGING);
				}
			}

			msm_batt_info.batt_health = new_health;
			return 1;
		}
	}

	return 0;	/* nothing is changed */
}

#define	be32_to_cpu_self(v)	(v = be32_to_cpu(v))
#define	be16_to_cpu_self(v)	(v = be16_to_cpu(v))

static int msm_batt_get_batt_chg_status(void)
{
	int rc;
	struct rpc_req_batt_chg {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_batt_chg;
	struct rpc_reply_batt_chg_v1 *v1p;

	req_batt_chg.more_data = cpu_to_be32(1);
	memset(&rep_batt_chg, 0, sizeof(rep_batt_chg));

	v1p = &rep_batt_chg.v1;
	rc = msm_rpc_call_reply(msm_batt_info.chg_ep,
		ONCRPC_CHG_GET_GENERAL_STATUS_PROC,
		&req_batt_chg, sizeof(req_batt_chg), &rep_batt_chg,
		sizeof(rep_batt_chg), msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: ERROR. msm_rpc_call_reply failed! proc=%d rc=%d\n",
			__func__, ONCRPC_CHG_GET_GENERAL_STATUS_PROC, rc);
		return rc;
	} else if (be32_to_cpu(v1p->more_data)) {
		be32_to_cpu_self(v1p->charger_status);
		be32_to_cpu_self(v1p->charger_type);
		be32_to_cpu_self(v1p->battery_status);
		be32_to_cpu_self(v1p->battery_level);
		be32_to_cpu_self(v1p->battery_voltage);
		be32_to_cpu_self(v1p->battery_temp);
		be32_to_cpu_self(v1p->chg_current);
		be32_to_cpu_self(v1p->batt_id);
	} else {
		pr_err("%s: No battery/charger data in RPC reply\n", __func__);
		return -EIO;
	}

		return 0;
}

static unsigned int msm_batt_check_vfopen(u32 vf) /*open: 2400 , close: 620 */
{
	unsigned int rc = 0;
	static int batt_id_check;

	if (vf > 2000)
		batt_id_check++;
	else {
		if (batt_id_check >= 2) {
			pr_info("vf recovery!!!    vf = %d\n", vf);
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
			msm_batt_unhandled_interrupt = 1;
		}
		batt_id_check = 0;
	}

	if (batt_id_check >= 2) {
		if (msm_batt_info.charging_source != NO_CHG) {
			pr_info("unspec vf!!!    vf = %d\n", vf);
			if (!msm_batt_info.factory_mode) {
				msm_batt_info.batt_health =
				    POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
			} else {
				msm_batt_info.batt_health =
				    POWER_SUPPLY_HEALTH_GOOD;
				pr_info("%s: factory mode case\n", __func__);
			}
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_DISCHARGING;
			msm_batt_chg_en(STOP_CHARGING);
			batt_id_check = 2;
			rc = 1;	/* open */
		}
	}

	return rc;
}

static struct vreg *accel_vreg_io;

static void msm_batt_update_psy_status(void)
{
	u32 charger_status;
	u32 charger_type;
	u32 battery_status;
	u32 battery_level;
	u32 battery_voltage;
	u32 battery_temp_adc;
	u32 chg_current_adc;
	u32 status_changed = 0;
	u32 batt_id = 0;

	static int flag;

	struct timespec ts;

	wake_lock(&monitor_wake_lock);

	if (is_reset_soc == 1) /*while fg_reset_soc, don't update status*/
		goto monitoring_skip;

	 /*avoid periodical update and update for fg_reset_soc happen together*/
	is_msm_batt_updating = 1;

	if (!flag) {	/* (+)Open Check */
		msm_batt_chg_en(STOP_CHARGING);
		usleep_range(3000, 4000);
		msm_batt_chg_en(START_CHARGING);
		flag = 1;

		if (charging_boot) {
			if (accel_vreg_io == NULL) {
				accel_vreg_io = vreg_get(NULL, "ruim");

				if (accel_vreg_io == NULL) {
					pr_err("%s: accel_vreg_io null!\n",
						__func__);
					is_msm_batt_updating = 0;
					goto monitoring_skip;
				}
				if (vreg_set_level(accel_vreg_io, 1800) < 0) {
					pr_err("%s: accel_vreg_io set level failed!\n",
						__func__);
					is_msm_batt_updating = 0;
					goto monitoring_skip;
				}

				vreg_disable(accel_vreg_io);
			}
		}
	}

	/* Get general status from CP by RPC */
	if (msm_batt_get_batt_chg_status()) {
		is_msm_batt_updating = 0;
		goto monitoring_skip;
	}

	charger_status = rep_batt_chg.v1.charger_status;
	charger_type = rep_batt_chg.v1.charger_type;
	battery_status = rep_batt_chg.v1.battery_status;
	battery_temp_adc = rep_batt_chg.v1.battery_temp;
	chg_current_adc = rep_batt_chg.v1.chg_current;
	batt_id = rep_batt_chg.v1.batt_id;

	pr_info("[BATT] %s: chg_current = %d, Temperature = %d, V_F = %d,  (from CP)\n",
		__func__, chg_current_adc, battery_temp_adc, batt_id);

	if (!(msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING)
		&& !(msm_batt_info.batt_recharging == 1)) {
	#if defined(USE_2STEP_TERM)
		if (msm_batt_info.full_charged_state != STATUS_1ST_FULL)
			chg_current_adc = 0;	/* not charging */
	#else
		chg_current_adc = 0;	/* not charging */
	#endif
	}

	msm_batt_info.batt_presoc = msm_batt_info.batt_capacity;
#ifdef CONFIG_MAX17043_FUEL_GAUGE
	battery_level = get_level_from_fuelgauge();
	fg_wakeunlock(battery_level);
	battery_voltage = get_voltage_from_fuelgauge();
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */

	msm_batt_info.battery_voltage = battery_voltage;

	/**************************/
	/* Check what is changed */
	status_changed += msm_batt_check_vfopen(batt_id);

	/* check temperature */
	msm_batt_info.battery_temp_adc =
	    msm_batt_average_temperature(battery_temp_adc);

	if (msm_batt_info.check_cable_change == false) {
		status_changed += msm_batt_control_temperature
					(msm_batt_info.battery_temp_adc);
	} else {
		msm_batt_info.check_cable_change = false;
		pr_info("%s: Skip temp check at cable attach/detach\n",
			__func__);
	}

	/* RCOMP update depending on temperature */
#ifdef CONFIG_MACH_ICON
	max17040_rcomp_update(msm_batt_info.battery_temp_degc);
#endif

	/* check full charging */
	msm_batt_info.chg_current_adc =
		msm_batt_average_chg_current(chg_current_adc);
	status_changed += msm_batt_check_full_charging
				(msm_batt_info.chg_current_adc);

	/* check recharging */
	status_changed += msm_batt_check_recharging();

	/* battery level, capacity (%) */
	status_changed += msm_batt_check_level(battery_level);

	/* temperature health for power off charging */
	if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_GOOD)
		msm_batt_info.batt_temp_check = 1;
	else
		msm_batt_info.batt_temp_check = 0;

	pr_info("[BATT] %s: Voltage=%d, Soc = %d, temperature = %d, "\
		"current = %d, Source = %d, Health = %d, status = %d, "\
		"rechg = %d", __func__,
		msm_batt_info.battery_voltage,
		msm_batt_info.batt_capacity,
		msm_batt_info.battery_temp_adc,
		msm_batt_info.chg_current_adc,
		msm_batt_info.charging_source,
		msm_batt_info.batt_health,
		msm_batt_info.batt_status,
		msm_batt_info.batt_recharging);

#if defined(USE_2STEP_TERM)
	pr_cont(", f_s(%d)\n", msm_batt_info.full_charged_state);
#else
	pr_cont("\n");
#endif

	pr_info("%s: power_supply_changed\n", __func__);
	power_supply_changed(&msm_psy_batt);

#ifndef BATT_LED_CONTROL_DISABLE  /*LED is not controlled by battery driver */
	pr_info("[BATT] %s: battery led control enabled!\n", __func__);
#ifdef CONFIG_LEDS_PMIC8058
	msm_batt_led_control();
#endif
#endif
	/*if (msm_batt_unhandled_interrupt) {
		msm_batt_cable_status_update();
		msm_batt_unhandled_interrupt = 0;
	}*/

	is_msm_batt_updating = 0;

monitoring_skip:
	msm_batt_info.cur_monitor_time = alarm_get_elapsed_realtime();
	ts = ktime_to_timespec(msm_batt_info.cur_monitor_time);

	sec_bat_monitoring_alarm(CHARGING_ALARM_INTERVAL);

	if (likely(status_changed == 0))
		wake_unlock(&monitor_wake_lock);
	else
		wake_lock_timeout(&monitor_wake_lock, (3 * HZ));

	return;
}

static void msm_batt_preupdate_psy_status(void)
{
	u32	battery_temp_adc;

	if (msm_batt_get_batt_chg_status())
		return;

	battery_temp_adc = rep_batt_chg.v1.battery_temp;

	/* check temperature */
	msm_batt_info.battery_temp_adc =
		msm_batt_average_temperature(battery_temp_adc);
	msm_batt_control_temperature(msm_batt_info.battery_temp_adc);

	/* temperature health for power off charging */
	if (msm_batt_info.batt_health == POWER_SUPPLY_HEALTH_GOOD)
		msm_batt_info.batt_temp_check = 1;
	else
		msm_batt_info.batt_temp_check = 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
struct batt_modify_client_req {
	u32 client_handle;

	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
};

struct batt_modify_client_rep {
	u32 result;
};

static int msm_batt_modify_client_arg_func(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct batt_modify_client_req *batt_modify_client_req =
		(struct batt_modify_client_req *)data;
	u32 *req = (u32 *) buf;
	int size = 0;

	*req = cpu_to_be32(batt_modify_client_req->client_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->desired_batt_voltage);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->voltage_direction);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->batt_cb_id);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(batt_modify_client_req->cb_data);
	size += sizeof(u32);

	return size;
}

static int msm_batt_modify_client_ret_func(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct batt_modify_client_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_modify_client_rep *)data;
	buf_ptr = (struct batt_modify_client_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);

	return 0;
}

static int msm_batt_modify_client(u32 client_handle, u32 desired_batt_voltage,
					  u32 voltage_direction, u32 batt_cb_id,
					  u32 cb_data)
{
	int rc;

	struct batt_modify_client_req req;
	struct batt_modify_client_rep rep;

	req.client_handle = client_handle;
	req.desired_batt_voltage = desired_batt_voltage;
	req.voltage_direction = voltage_direction;
	req.batt_cb_id = batt_cb_id;
	req.cb_data = cb_data;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
		BATTERY_MODIFY_CLIENT_PROC, msm_batt_modify_client_arg_func,
		&req, msm_batt_modify_client_ret_func, &rep,
		msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: ERROR. failed to modify  Vbatt client\n", __func__);
			return rc;
	}

	if (rep.result != BATTERY_MODIFICATION_SUCCESSFUL) {
		pr_err("%s: ERROR. modify client failed. result = %u\n",
		       __func__, rep.result);
		return -EIO;
	}

	return 0;
}

void msm_batt_early_suspend(struct early_suspend *h)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
			BATTERY_LOW, BATTERY_VOLTAGE_BELOW_THIS_LEVEL,
			BATTERY_CB_ID_LOW_VOL, BATTERY_LOW);

		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client. rc=%d\n",
				__func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	pr_debug("%s: exit\n", __func__);
}

void msm_batt_late_resume(struct early_suspend *h)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_modify_client(msm_batt_info.batt_handle,
			BATTERY_LOW, BATTERY_ALL_ACTIVITY,
			BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
		if (rc < 0) {
			pr_err("%s: msm_batt_modify_client FAIL rc=%d\n",
				__func__, rc);
			return;
		}
	} else {
		pr_err("%s: ERROR. invalid batt_handle\n", __func__);
		return;
	}

	pr_debug("%s: exit\n", __func__);
}
#endif

struct msm_batt_vbatt_filter_req {
	u32 batt_handle;
	u32 enable_filter;
	u32 vbatt_filter;
};

struct msm_batt_vbatt_filter_rep {
	u32 result;
};

static int msm_batt_filter_arg_func(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct msm_batt_vbatt_filter_req *vbatt_filter_req =
		(struct msm_batt_vbatt_filter_req *)data;
	u32 *req = (u32 *) buf;
	int size = 0;

	*req = cpu_to_be32(vbatt_filter_req->batt_handle);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->enable_filter);
	size += sizeof(u32);
	req++;

	*req = cpu_to_be32(vbatt_filter_req->vbatt_filter);
	size += sizeof(u32);
	return size;
}

static int msm_batt_filter_ret_func(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct msm_batt_vbatt_filter_rep *data_ptr, *buf_ptr;

	data_ptr = (struct msm_batt_vbatt_filter_rep *)data;
	buf_ptr = (struct msm_batt_vbatt_filter_rep *)buf;

	data_ptr->result = be32_to_cpu(buf_ptr->result);
	return 0;
}

static int msm_batt_enable_filter(u32 vbatt_filter)
{
	int rc;
	struct msm_batt_vbatt_filter_req vbatt_filter_req;
	struct msm_batt_vbatt_filter_rep vbatt_filter_rep;

	vbatt_filter_req.batt_handle = msm_batt_info.batt_handle;
	vbatt_filter_req.enable_filter = 1;
	vbatt_filter_req.vbatt_filter = vbatt_filter;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
				BATTERY_ENABLE_DISABLE_FILTER_PROC,
				msm_batt_filter_arg_func, &vbatt_filter_req,
				msm_batt_filter_ret_func, &vbatt_filter_rep,
				msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: enable vbatt filter. rc=%d\n", __func__, rc);
		return rc;
	}

	if (vbatt_filter_rep.result != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: FAIL: enable vbatt filter: result=%d\n",
			__func__, vbatt_filter_rep.result);
		return -EIO;
	}

	pr_debug("%s: enable vbatt filter: OK\n", __func__);
	return rc;
}

struct batt_client_registration_req {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 more_data;
	u32 batt_error;
};

struct batt_client_registration_req_4_1 {
	/* The voltage at which callback (CB) should be called. */
	u32 desired_batt_voltage;

	/* The direction when the CB should be called. */
	u32 voltage_direction;

	/* The registered callback to be called when voltage and
	 * direction specs are met. */
	u32 batt_cb_id;

	/* The call back data */
	u32 cb_data;
	u32 batt_error;
};

struct batt_client_registration_rep {
	u32 batt_handle;
};

struct batt_client_registration_rep_4_1 {
	u32 batt_handle;
	u32 more_data;
	u32 err;
};

static int msm_batt_register_arg_func(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct batt_client_registration_req *batt_reg_req =
		(struct batt_client_registration_req *)data;

	u32 *req = (u32 *) buf;
	int size = 0;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->voltage_direction);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_cb_id);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->cb_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_error);
		size += sizeof(u32);

		return size;
	} else {
		*req = cpu_to_be32(batt_reg_req->desired_batt_voltage);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->voltage_direction);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_cb_id);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->cb_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->more_data);
		size += sizeof(u32);
		req++;

		*req = cpu_to_be32(batt_reg_req->batt_error);
		size += sizeof(u32);

		return size;
	}
}

static int msm_batt_register_ret_func(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct batt_client_registration_rep *data_ptr, *buf_ptr;
	struct batt_client_registration_rep_4_1 *data_ptr_4_1,
	    *buf_ptr_4_1;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		data_ptr_4_1 = (struct batt_client_registration_rep_4_1 *)data;
		buf_ptr_4_1 = (struct batt_client_registration_rep_4_1 *)buf;

		data_ptr_4_1->batt_handle =
			be32_to_cpu(buf_ptr_4_1->batt_handle);
		data_ptr_4_1->more_data = be32_to_cpu(buf_ptr_4_1->more_data);
		data_ptr_4_1->err = be32_to_cpu(buf_ptr_4_1->err);
		return 0;
	} else {
		data_ptr = (struct batt_client_registration_rep *)data;
		buf_ptr = (struct batt_client_registration_rep *)buf;

		data_ptr->batt_handle = be32_to_cpu(buf_ptr->batt_handle);
		return 0;
	}
}

static int msm_batt_register(u32 desired_batt_voltage, u32 voltage_direction,
	u32 batt_cb_id, u32 cb_data)
{
	struct batt_client_registration_req batt_reg_req;
	struct batt_client_registration_req_4_1 batt_reg_req_4_1;
	struct batt_client_registration_rep batt_reg_rep;
	struct batt_client_registration_rep_4_1 batt_reg_rep_4_1;
	void *request;
	void *reply;
	int rc;

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		batt_reg_req_4_1.desired_batt_voltage = desired_batt_voltage;
		batt_reg_req_4_1.voltage_direction = voltage_direction;
		batt_reg_req_4_1.batt_cb_id = batt_cb_id;
		batt_reg_req_4_1.cb_data = cb_data;
		batt_reg_req_4_1.batt_error = 1;
		request = &batt_reg_req_4_1;
	} else {
		batt_reg_req.desired_batt_voltage = desired_batt_voltage;
		batt_reg_req.voltage_direction = voltage_direction;
		batt_reg_req.batt_cb_id = batt_cb_id;
		batt_reg_req.cb_data = cb_data;
		batt_reg_req.more_data = 1;
		batt_reg_req.batt_error = 0;
		request = &batt_reg_req;
	}

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1)
		reply = &batt_reg_rep_4_1;
	else
		reply = &batt_reg_rep;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
		BATTERY_REGISTER_PROC, msm_batt_register_arg_func,
		request, msm_batt_register_ret_func, reply,
		msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt register. rc=%d\n", __func__, rc);
		return rc;
	}

	if (msm_batt_info.batt_api_version == BATTERY_RPC_VER_4_1) {
		if (batt_reg_rep_4_1.more_data != 0 && batt_reg_rep_4_1.err
		    != BATTERY_REGISTRATION_SUCCESSFUL) {
			pr_err("%s: vBatt Registration Failed proc_num=%d\n",
				__func__, BATTERY_REGISTER_PROC);
			return -EIO;
		}
		msm_batt_info.batt_handle = batt_reg_rep_4_1.batt_handle;
	} else {
		msm_batt_info.batt_handle = batt_reg_rep.batt_handle;
	}

	return 0;
}

struct batt_client_deregister_req {
	u32 batt_handle;
};

struct batt_client_deregister_rep {
	u32 batt_error;
};

static int msm_batt_deregister_arg_func(struct msm_rpc_client *batt_client,
	void *buf, void *data)
{
	struct batt_client_deregister_req *deregister_req =
		(struct batt_client_deregister_req *)data;
	u32 *req = (u32 *) buf;
	int size = 0;

	*req = cpu_to_be32(deregister_req->batt_handle);
	size += sizeof(u32);

	return size;
}

static int msm_batt_deregister_ret_func(struct msm_rpc_client *batt_client,
	void *buf,	void *data)
{
	struct batt_client_deregister_rep *data_ptr, *buf_ptr;

	data_ptr = (struct batt_client_deregister_rep *)data;
	buf_ptr = (struct batt_client_deregister_rep *)buf;

	data_ptr->batt_error = be32_to_cpu(buf_ptr->batt_error);

	return 0;
}

static int msm_batt_deregister(u32 batt_handle)
{
	int rc;
	struct batt_client_deregister_req req;
	struct batt_client_deregister_rep rep;

	req.batt_handle = batt_handle;

	rc = msm_rpc_client_req(msm_batt_info.batt_client,
				BATTERY_DEREGISTER_CLIENT_PROC,
				msm_batt_deregister_arg_func, &req,
				msm_batt_deregister_ret_func, &rep,
				msecs_to_jiffies(BATT_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: vbatt deregister. rc=%d\n", __func__, rc);
		return rc;
	}

	if (rep.batt_error != BATTERY_DEREGISTRATION_SUCCESSFUL) {
		pr_err("%s: vbatt deregistration FAIL. error=%d, handle=%d\n",
			__func__, rep.batt_error, batt_handle);
		return -EIO;
	}

	return 0;
}

/*For FSA related callback*/
void msm_set_cable(struct msm_battery_callback *ptr,
			enum cable_type_t status)
{
	struct msm_battery_info *chg =
		container_of(ptr, struct msm_battery_info, callback);

	if (msm_batt_info.cable_status == status) {
		pr_info("%s: cable status has not changed!!(%d=%d)\n",
			__func__, msm_batt_info.cable_status, status);
		return;
	}

	chg->cable_status = status;
	msm_batt_info.cable_status = status;

	msm_batt_cable_status_update();
}

/*For FSA related callback*/
static void msm_batt_cable_status_update(void)
{
	struct msm_battery_info *chg = &msm_batt_info;
	u32 prev_batt_health;

	static char *health_text[] = {
		"Unknown", "Good", "Overheat", "Dead",
		"Over voltage", "Unspecified failure", "Cold",
	};

	wake_lock(&vbus_wake_lock);

	msm_batt_info.batt_full_check = 0;
	msm_batt_info.batt_recharging = 0;
	msm_batt_info.full_charged_state = STATUS_NOT_FULL;

	pr_info("%s : [BATT] cable_status(%d)\n",
		__func__, chg->cable_status);

	msm_batt_info.pre_batt_status = msm_batt_info.batt_status;

	if ((chg->cable_status == CABLE_TYPE_NONE) ||
		(chg->cable_status == CABLE_TYPE_UNKNOWN) ||
		(chg->cable_status == CABLE_TYPE_JIG)) {
		msm_batt_info.charging_source = NO_CHG;
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		msm_batt_info.charger_type = CHARGER_TYPE_NONE;
		msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
		msm_batt_chg_en(STOP_CHARGING);
	} else  if ((chg->cable_status == CABLE_TYPE_CAR_DOCK)
			|| (chg->cable_status == CABLE_TYPE_DESK_DOCK)
			|| (chg->cable_status == CABLE_TYPE_UART)) {
		pr_debug("%s: ovp_en=%d\n", __func__, ovp_en);
		if (fsa9485_vbus_valid()) {
			if ((msm_batt_info.batt_health !=
				POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) &&
				(msm_batt_info.batt_health !=
				POWER_SUPPLY_HEALTH_OVERHEAT) &&
				(msm_batt_info.batt_health !=
				POWER_SUPPLY_HEALTH_COLD)) {
				prev_batt_health = msm_batt_info.batt_health;
				msm_batt_info.batt_health =
					POWER_SUPPLY_HEALTH_GOOD;
				pr_info("[BATT] %s: Battery health changed(%s->%s)\n",
				__func__, health_text[prev_batt_health],
				health_text[msm_batt_info.batt_health]);
			}

			msm_batt_info.charging_source = AC_CHG;
			msm_batt_info.charger_type = CHARGER_TYPE_WALL;

			if (msm_batt_info.batt_health !=
				POWER_SUPPLY_HEALTH_GOOD) {
				pr_info("[BATT] %s: Battery health is %s, stop charging!\n",
				__func__,
				health_text[msm_batt_info.batt_health]);
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_NOT_CHARGING;
				msm_batt_chg_en(STOP_CHARGING);
			} else {
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_CHARGING;
				msm_batt_chg_en(START_CHARGING);
			}
		} else {
			msm_batt_info.charging_source = NO_CHG;
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_DISCHARGING;
			msm_batt_info.charger_type = CHARGER_TYPE_NONE;
			msm_batt_chg_en(STOP_CHARGING);
		}
	} else {
		if ((msm_batt_info.batt_health !=
			POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) &&
			(msm_batt_info.batt_health !=
			POWER_SUPPLY_HEALTH_OVERHEAT) &&
			(msm_batt_info.batt_health !=
			POWER_SUPPLY_HEALTH_COLD)) {
			prev_batt_health = msm_batt_info.batt_health;
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
			pr_info("[BATT] %s: Battery health changed(%s->%s)\n",
			__func__, health_text[prev_batt_health],
			health_text[msm_batt_info.batt_health]);
		}

		if (chg->cable_status == CABLE_TYPE_USB ||
			chg->cable_status == CABLE_TYPE_CDP) {
			if (msm_batt_info.batt_slate_mode == 1) {
				msm_batt_info.charging_source = NO_CHG;
				msm_batt_info.charger_type = CHARGER_TYPE_NONE;
			} else {
				msm_batt_info.charging_source = USB_CHG;
				msm_batt_info.charger_type =
					CHARGER_TYPE_USB_PC;
			}
		} else if (chg->cable_status == CABLE_TYPE_TA) {
			msm_batt_info.charging_source = AC_CHG;
			msm_batt_info.charger_type = CHARGER_TYPE_WALL;
		}

		if (msm_batt_info.batt_health != POWER_SUPPLY_HEALTH_GOOD) {
			pr_info("[BATT] %s: Battery health is %s, stop charging!\n",
				__func__,
				health_text[msm_batt_info.batt_health]);
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			msm_batt_chg_en(STOP_CHARGING);
		} else {
			if (msm_batt_info.batt_slate_mode == 1) {
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_DISCHARGING;
				msm_batt_chg_en(STOP_CHARGING);
			} else {
				msm_batt_info.batt_status =
					POWER_SUPPLY_STATUS_CHARGING;
				msm_batt_chg_en(START_CHARGING);
			}
		}
	}

	if (ovp_en == 1 && ovp_en_handled == 0) {
		ovp_en_handled = 1;
		if (msm_batt_info.batt_status ==
			POWER_SUPPLY_STATUS_CHARGING) {
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_DISCHARGING;
			msm_batt_info.batt_health =
				POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			msm_batt_chg_en(STOP_CHARGING);
			pr_info("[BATT] %s: OVP case!!\n", __func__);
		}
	}

	/*pr_info("[BATT] %s: power_supply_changed !\n", __func__);
	power_supply_changed(&msm_psy_batt);
	power_supply_changed(&msm_psy_usb);
	power_supply_changed(&msm_psy_ac); */

	msm_batt_info.check_cable_change = true;
	queue_delayed_work(msm_batt_info.msm_batt_wq,
		&msm_batt_info.msm_batt_work, 0);
	pr_info("%s: battery work queue called\n", __func__);

	msm_batt_info.prev_cable = msm_batt_info.cable_status;

	if (likely(!(charging_boot &&
		msm_batt_info.batt_status !=
			POWER_SUPPLY_STATUS_CHARGING)))
		wake_lock_timeout(&vbus_wake_lock, (3 * HZ));
	else
		wake_lock_timeout(&vbus_wake_lock, (10 * HZ));
}

/*For SMB328A related functionality*/
void msm_batt_cable_status_update_ext(int vbus_in)
{
	struct msm_battery_info *chg = &msm_batt_info;
	int pre_vbus = msm_batt_info.vbus_valid;
	u32 prev_batt_health;

	static char *health_text[] = {
		"Unknown", "Good", "Overheat", "Dead",
		"Over voltage", "Unspecified failure", "Cold",
	};

	if (pre_vbus == vbus_in) {
		pr_info("%s: vbus not changed!!(%d=%d)\n",
			__func__, pre_vbus, vbus_in);
		return;
	} else {
		msm_batt_info.vbus_valid = vbus_in;
		pr_info("%s: vbus changed!!(%d->%d)\n",
			__func__, pre_vbus, vbus_in);
	}

	wake_lock(&vbus_wake_lock);

	msm_batt_info.batt_full_check = 0;
	msm_batt_info.batt_recharging = 0;
	msm_batt_info.full_charged_state = STATUS_NOT_FULL;

	pr_info("%s : [BATT] cable_status(%d)\n",
		__func__, chg->cable_status);

	msm_batt_info.pre_batt_status = msm_batt_info.batt_status;

	if ((chg->cable_status == CABLE_TYPE_USB) ||
		(chg->cable_status == CABLE_TYPE_TA) ||
		(chg->cable_status == CABLE_TYPE_CDP))
		goto cable_update_skip;

	pr_debug("%s: ovp_en=%d\n", __func__, ovp_en);
	if (vbus_in) {
		if ((msm_batt_info.batt_health !=
			POWER_SUPPLY_HEALTH_UNSPEC_FAILURE) &&
			(msm_batt_info.batt_health !=
			POWER_SUPPLY_HEALTH_OVERHEAT) &&
			(msm_batt_info.batt_health !=
			POWER_SUPPLY_HEALTH_COLD)) {
			prev_batt_health = msm_batt_info.batt_health;
			msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
			pr_info("[BATT] %s: Battery health changed(%s->%s)\n",
			__func__, health_text[prev_batt_health],
			health_text[msm_batt_info.batt_health]);
		}

		msm_batt_info.charging_source = AC_CHG;
		msm_batt_info.charger_type = CHARGER_TYPE_WALL;

		if (msm_batt_info.batt_health !=
			POWER_SUPPLY_HEALTH_GOOD) {
			pr_info("[BATT] %s: Battery health is %s, stop charging!\n",
			__func__, health_text[msm_batt_info.batt_health]);
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_NOT_CHARGING;
			msm_batt_chg_en(STOP_CHARGING);
		} else {
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_CHARGING;
			msm_batt_chg_en(START_CHARGING);
		}
	} else {
		msm_batt_info.charging_source = NO_CHG;
		msm_batt_info.batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
		msm_batt_info.charger_type = CHARGER_TYPE_NONE;
		msm_batt_info.batt_health = POWER_SUPPLY_HEALTH_GOOD;
		msm_batt_chg_en(STOP_CHARGING);
	}

	if (ovp_en == 1 && ovp_en_handled == 0) {
		ovp_en_handled = 1;
		if (msm_batt_info.batt_status ==
			POWER_SUPPLY_STATUS_CHARGING) {
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_DISCHARGING;
			msm_batt_info.batt_health =
				POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			msm_batt_chg_en(STOP_CHARGING);
			pr_info("[BATT] %s: OVP case!!\n", __func__);
		}
	}

	/*pr_info("[BATT] %s: power_supply_changed !\n", __func__);
	power_supply_changed(&msm_psy_batt);
	power_supply_changed(&msm_psy_usb);
	power_supply_changed(&msm_psy_ac);*/

	pr_debug("%s: pre_batt_status = %d, batt_status = %d, chg_source=%d\n",
		__func__, msm_batt_info.pre_batt_status,
		msm_batt_info.batt_status, msm_batt_info.charging_source);

	msm_batt_info.check_cable_change = true;
	queue_delayed_work(msm_batt_info.msm_batt_wq,
		&msm_batt_info.msm_batt_work, 0);
	pr_info("%s: battery work queue called\n", __func__);

cable_update_skip:
	if (likely(!(charging_boot &&
		msm_batt_info.batt_status !=
			POWER_SUPPLY_STATUS_CHARGING)))
		wake_lock_timeout(&vbus_wake_lock, (3 * HZ));
	else
		wake_lock_timeout(&vbus_wake_lock, (10 * HZ));
}
EXPORT_SYMBOL(msm_batt_cable_status_update_ext);

int msm_batt_is_ovp(int enable)
{
	wake_lock(&vbus_wake_lock);

	if (enable == 1) {
		if (ovp_en == 1) {
			pr_debug("[BATT] %s: duplicated ovp_en!\n", __func__);
			wake_unlock(&vbus_wake_lock);
			return 0;
		}
		msm_batt_info.batt_status =
			POWER_SUPPLY_STATUS_DISCHARGING;
		msm_batt_info.batt_health =
				POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		ovp_en = 1;
		ovp_en_handled = 0;
		msm_batt_chg_en(STOP_CHARGING);
		pr_info("[BATT] %s: OVP enabled !\n", __func__);
	} else {
		if (ovp_en == 0) {
			pr_debug("[BATT] %s: duplicated ovp_dis!\n", __func__);
			wake_unlock(&vbus_wake_lock);
			return 0;
		}
		ovp_en = 0;
		ovp_en_handled = 1;
		msm_batt_info.batt_health =
				POWER_SUPPLY_HEALTH_GOOD;
		if (fsa9485_vbus_valid() &&
			(msm_batt_info.charging_source != NO_CHG)) {
			msm_batt_info.batt_status =
				POWER_SUPPLY_STATUS_CHARGING;
			msm_batt_chg_en(START_CHARGING);
		}
		pr_info("[BATT] %s: OVP disabled !\n", __func__);
	}

	pr_info("[BATT] %s: power_supply_changed !\n", __func__);
	power_supply_changed(&msm_psy_batt);
	/*power_supply_changed(&msm_psy_usb);
	power_supply_changed(&msm_psy_ac);*/

	wake_lock_timeout(&vbus_wake_lock, (3 * HZ));
}
EXPORT_SYMBOL(msm_batt_is_ovp);

static int msm_batt_suspend(struct device *dev)
{
	pr_info("[BATT] %s\n", __func__);

	cancel_delayed_work(&msm_batt_info.msm_batt_work);

	msm_batt_info.cur_monitor_time = alarm_get_elapsed_realtime();
	/*pr_info("%s: cur_monitor_time = %u\n",
		__func__, msm_batt_info.cur_monitor_time);*/

	if (msm_batt_info.charging_source == NO_CHG) {
		pr_info("%s: cable_type_none\n", __func__);
		sec_bat_monitoring_alarm(ALARM_INTERVAL);
		msm_batt_info.slow_polling = 1;
	} else {
		pr_info("%s: valid cable_type\n", __func__);
		sec_bat_monitoring_alarm(CHARGING_ALARM_INTERVAL);
	}

	return 0;
}

static void msm_batt_resume(struct device *dev)
{
	pr_debug("[BATT] %s\n", __func__);
	if (msm_batt_info.slow_polling)
		msm_batt_info.slow_polling = 0;

	if (msm_batt_info.slow_polling)
		msm_batt_info.slow_polling = 0;

	queue_delayed_work(msm_batt_info.msm_batt_wq,
		&msm_batt_info.msm_batt_work, 0);
}

int batt_restart(void)
{
	if (msm_batt_driver_init) {
		/*msm_batt_cable_status_update();*/

		queue_delayed_work(msm_batt_info.msm_batt_wq,
			&msm_batt_info.msm_batt_work, 0);
	} else {
		pr_err("[BATT] %s: Battery driver is not ready !!\n", __func__);
		msm_batt_unhandled_interrupt = 1;
	}

	return 0;
}
EXPORT_SYMBOL(batt_restart);

static int msm_batt_cleanup(void)
{
	int rc = 0;

	pr_info("[BATT] %s\n", __func__);

	/*For FSA related callback*/
	if (msm_batt_info.pdata->register_callbacks)
		msm_batt_info.pdata->register_callbacks(NULL);

	msm_batt_remove_attrs(msm_psy_batt.dev);
	i2c_del_driver(&fg_i2c_driver);

	if (msm_batt_info.batt_handle != INVALID_BATT_HANDLE) {
		rc = msm_batt_deregister(msm_batt_info.batt_handle);
		if (rc < 0)
			pr_err("%s: FAIL: msm_batt_deregister. rc=%d\n",
			__func__, rc);
	}

	msm_batt_info.batt_handle = INVALID_BATT_HANDLE;

	if (msm_batt_info.batt_client)
		msm_rpc_unregister_client(msm_batt_info.batt_client);

	if (msm_batt_info.msm_psy_ac)
		power_supply_unregister(msm_batt_info.msm_psy_ac);

	if (msm_batt_info.msm_psy_usb)
		power_supply_unregister(msm_batt_info.msm_psy_usb);
	if (msm_batt_info.msm_psy_batt)
		power_supply_unregister(msm_batt_info.msm_psy_batt);

	if (msm_batt_info.chg_ep) {
		rc = msm_rpc_close(msm_batt_info.chg_ep);
		if (rc < 0) {
			pr_err("%s: FAIL. msm_rpc_close(chg_ep). rc=%d\n",
				__func__, rc);
		}
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	if (msm_batt_info.early_suspend.suspend == msm_batt_early_suspend)
		unregister_early_suspend(&msm_batt_info.early_suspend);
#endif
	return rc;
}

int msm_batt_get_charger_api_version(void)
{
	int rc;
	struct rpc_reply_hdr *reply;

	struct rpc_req_chg_api_ver {
		struct rpc_request_hdr hdr;
		u32 more_data;
	} req_chg_api_ver;

	struct rpc_rep_chg_api_ver {
		struct rpc_reply_hdr hdr;
		u32 num_of_chg_api_versions;
		u32 *chg_api_versions;
	};

	u32 num_of_versions;

	struct rpc_rep_chg_api_ver *rep_chg_api_ver;

	req_chg_api_ver.more_data = cpu_to_be32(1);

	msm_rpc_setup_req(&req_chg_api_ver.hdr, CHG_RPC_PROG, CHG_RPC_VER_1_1,
			  ONCRPC_CHARGER_API_VERSIONS_PROC);

	rc = msm_rpc_write(msm_batt_info.chg_ep, &req_chg_api_ver,
		sizeof(req_chg_api_ver));
	if (rc < 0) {
		pr_err("%s: FAIL: msm_rpc_write. proc=0x%08x, rc=%d\n",
			__func__, ONCRPC_CHARGER_API_VERSIONS_PROC, rc);
		return rc;
	}

	for (;;) {
		rc = msm_rpc_read(msm_batt_info.chg_ep, (void *)&reply, -1,
			BATT_RPC_TIMEOUT);
		if (rc < 0)
			return rc;
		if (rc < RPC_REQ_REPLY_COMMON_HEADER_SIZE) {
			pr_err("%s: LENGTH ERR: msm_rpc_read. rc=%d (<%d)\n",
				__func__, rc, RPC_REQ_REPLY_COMMON_HEADER_SIZE);

			rc = -EIO;
			break;
		}
		/* we should not get RPC REQ or call packets -- ignore them */
		if (reply->type == RPC_TYPE_REQ) {
			pr_err("%s: TYPE ERR: type=%d (!=%d)\n",
			       __func__, reply->type, RPC_TYPE_REQ);
			kfree(reply);
			continue;
		}

		/* If an earlier call timed out, we could get the (no
		 * longer wanted) reply for it.  Ignore replies that
		 * we don't expect
		 */
		if (reply->xid != req_chg_api_ver.hdr.xid) {
			pr_err("%s: XID ERR: xid=%d (!=%d)\n", __func__,
			       reply->xid, req_chg_api_ver.hdr.xid);
			kfree(reply);
			continue;
		}
		if (reply->reply_stat != RPCMSG_REPLYSTAT_ACCEPTED) {
			rc = -EPERM;
			break;
		}
		if (reply->data.acc_hdr.accept_stat != RPC_ACCEPTSTAT_SUCCESS) {
			rc = -EINVAL;
			break;
		}

		rep_chg_api_ver = (struct rpc_rep_chg_api_ver *)reply;

		num_of_versions =  be32_to_cpu(rep_chg_api_ver->
			num_of_chg_api_versions);

		rep_chg_api_ver->chg_api_versions = (u32 *)((u8 *) reply
			+ sizeof(struct rpc_reply_hdr)
			+ sizeof(rep_chg_api_ver->num_of_chg_api_versions));

		rc = be32_to_cpu(rep_chg_api_ver->
			chg_api_versions[num_of_versions - 1]);

		pr_debug("%s: num_of_chg_api_versions = %u. The chg api version = 0x%08x\n",
			__func__, num_of_versions, rc);
		break;
	}
	kfree(reply);
	return rc;
}

static int msm_batt_cb_func(struct msm_rpc_client *client,
	void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case BATTERY_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
			__func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(msm_batt_info.batt_client,
		be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(msm_batt_info.batt_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	if (accept_status == RPC_ACCEPTSTAT_SUCCESS) {
		queue_delayed_work(msm_batt_info.msm_batt_wq,
			&msm_batt_info.msm_batt_work, 0);
	}

	return rc;
}

static int msm_batt_is_over_abs_time(void)
{
	unsigned int total_time;
#if defined(CONFIG_MACH_RAY)
	static int flag_watchdog;
#endif

	if (msm_batt_info.charging_start_time == 0) {
#if defined(CONFIG_MACH_RAY)
		flag_watchdog = 0;
#endif
		return 0;	/* not charging*/
	}

	if (msm_batt_info.batt_full_check == 1) {
		/* already fully charged... (recharging) */
		total_time = TOTAL_RECHARGING_TIME;
		#if defined(USE_2STEP_TERM)
		if (msm_batt_info.full_charged_state == STATUS_1ST_FULL)
			total_time = TOTAL_CHARGING_TIME;
		#endif
	} else
		total_time = TOTAL_CHARGING_TIME;

#if defined(CONFIG_MACH_RAY)
	if ((msm_batt_info.charging_passed_time > SAFETYTIMER)
		&& (flag_watchdog == 0)) {
		smb328a_watchdog_enable(1);
		msleep(100);
		smb328a_watchdog_enable(0);

		flag_watchdog = 1;
	}
#endif

	if (msm_batt_info.charging_passed_time > total_time) {
		pr_debug("[BATT] %s: abs time is over !!\n", __func__);
#if defined(CONFIG_MACH_RAY)
		flag_watchdog = 0;
#endif
		return 1;
	} else {
		return 0;
	}
}

/*#ifdef CONFIG_MAX17043_FUEL_GAUGE*/
/* Quick start condition check. */
/* static struct fuelgauge_linear_data {
	u32 min_vcell;
	u32 slope;
	u32 y_interception;
} qstrt_table[2][12] = {
	{	// w/o charger
		{450000000, 0, 0},
		{407900000, 2171993, 193731125},
		{389790000, 847027, 324374902},
		{378060000, 602617, 343245771},
		{372020000, 293109, 361124348},
		{368220000, 209554, 364231282},
		{362530000, 596997, 356856383},
		{359070000, 604297, 356792124},
		{354500000, 2679480, 348980813},
		{344820000, 6365513, 341425848},
		{339970000, 9109696, 339974670},
		{100000000, 0, 0}
	},
	{	// w/charger
		{450000000, 0, 0},
		{419270000, 12835, 406438276},
		{418480000, 73645, 349402654},
		{404650000, 45824, 370277069},
		{392800000, 20460, 382744637},
		{387510000, 51008, 375639409},
		{377390000, 298446, 367071455},
		{373320000, 630069, 360004053},
		{363720000, 1231165, 356301531},
		{100000000, 0, 0},
		{100000000, 0, 0},
		{100000000, 0, 0}
	},
}; */

/*#define FG_SOC_TOLERANCE	20*/	/* 15 */

/* static int check_quick_start(void)
{
	unsigned int vcell_raw = 0;
	int soc_raw = 0, soc_cal = 0;
	int i, curr_idx = 0;
	int status = 0;
	int array_size = 0;

	if (msm_batt_get_charger_type() == CHARGER_TYPE_NONE) {
		status = 0;
		array_size = 12;
		pr_debug("[BATT] %s: No charger !\n", __func__);
	} else {
		status = 1;
		array_size = 10;
		pr_debug("[BATT] %s: charger detected !\n", __func__);
	}

	// get vcell.
	vcell_raw = fg_read_raw_vcell();

	// get soc.
	soc_raw = fg_read_raw_soc();

	// find range
	for (i = 0; i < array_size - 1; i++) {
		if (vcell_raw < qstrt_table[status][i].min_vcell &&
		    vcell_raw >= qstrt_table[status][i + 1].min_vcell) {
			curr_idx = i + 1;
			break;
		}
	}

	pr_debug("[BATT] %s: curr_idx = %d (vol=%d)\n", __func__, curr_idx,
		qstrt_table[status][curr_idx].min_vcell);

	//calculate assumed soc and compare
	if ((status == 0 && curr_idx > 0 && curr_idx < 11) ||
		(status == 1 && curr_idx > 0 && curr_idx < 9)) {
		int limit_min, limit_max;
		soc_cal = (int)((vcell_raw
			- qstrt_table[status][curr_idx].y_interception)
			/ qstrt_table[status][curr_idx].slope);

		pr_debug("[BATT] %s: soc_cal = %d\n", __func__, soc_cal);

		limit_min = soc_cal - FG_SOC_TOLERANCE;
		limit_max = soc_cal + FG_SOC_TOLERANCE;
		if (limit_min < 0)
			limit_min = 0;

		if (soc_raw > limit_max || soc_raw < limit_min) {
			//  hsusb_chg_vbus_draw(0);
			fg_reset_soc();
			pr_info("\n[BATT] %s: QUICK START (reset_soc)!!!"
				"\n\n", __func__);
		}
	}

	return 0;
} */
/*#endif*/	/* CONFIG_MAX17043_FUEL_GAUGE */

static int __devinit msm_batt_probe(struct platform_device *pdev)
{
	int rc;
	struct msm_psy_batt_pdata *pdata = pdev->dev.platform_data;
	msm_batt_info.pdata = pdata->charger;

	pr_info("[BATT] %s begins\n", __func__);/*check probe works*/

	if (pdev->id != -1) {
		dev_err(&pdev->dev, "%s: MSM chipsets Can only support "\
			"one battery ", __func__);
		return -EINVAL;
	}

	msm_batt_info.batt_slate_mode = 0;

	if (pdata->avail_chg_sources & AC_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_ac);
		if (rc < 0) {
			dev_err(&pdev->dev, "%s: power_supply_register failed "\
				"rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_ac = &msm_psy_ac;
		msm_batt_info.avail_chg_sources |= AC_CHG;
	}

	if (pdata->avail_chg_sources & USB_CHG) {
		rc = power_supply_register(&pdev->dev, &msm_psy_usb);
		if (rc < 0) {
			dev_err(&pdev->dev, "%s: power_supply_register failed "\
				"rc = %d\n", __func__, rc);
			msm_batt_cleanup();
			return rc;
		}
		msm_batt_info.msm_psy_usb = &msm_psy_usb;
		msm_batt_info.avail_chg_sources |= USB_CHG;
	}

	if (!msm_batt_info.msm_psy_ac && !msm_batt_info.msm_psy_usb) {
		dev_err(&pdev->dev, "%s: No external Power supply(AC or USB) "\
			"is avilable\n", __func__);
		msm_batt_cleanup();
		return -ENODEV;
	}

	msm_batt_info.voltage_max_design = pdata->voltage_max_design;
	msm_batt_info.voltage_min_design = pdata->voltage_min_design;
	msm_batt_info.batt_technology = pdata->batt_technology;
	msm_batt_info.full_charged_state = STATUS_NOT_FULL;
	msm_batt_info.full_check_type = FULLCHARGED_SOC;

	if (!msm_batt_info.voltage_min_design)
		msm_batt_info.voltage_min_design = BATTERY_LOW;
	if (!msm_batt_info.voltage_max_design)
		msm_batt_info.voltage_max_design = BATTERY_HIGH;
	if (msm_batt_info.batt_technology == POWER_SUPPLY_TECHNOLOGY_UNKNOWN)
		msm_batt_info.batt_technology = POWER_SUPPLY_TECHNOLOGY_LION;

	rc = power_supply_register(&pdev->dev, &msm_psy_batt);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: power_supply_register failed rc=%d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}
	msm_batt_info.msm_psy_batt = &msm_psy_batt;

	rc = msm_batt_register(BATTERY_LOW, BATTERY_ALL_ACTIVITY,
		BATTERY_CB_ID_ALL_ACTIV, BATTERY_ALL_ACTIVITY);
	if (rc < 0) {
		dev_err(&pdev->dev, "%s: msm_batt_register failed rc = %d\n",
			__func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	rc = msm_batt_enable_filter(VBATT_FILTER);

	if (rc < 0) {
		dev_err(&pdev->dev, "%s: msm_batt_enable_filter failed "\
			"rc = %d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	msm_batt_create_attrs(msm_psy_batt.dev);

#if defined(CONFIG_BATTERY_CTIA)
	alarm_init(&msm_batt_info.event_alarm,
			ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			sec_bat_use_timer_func);
#endif /*CONFIG_BATTERY_CTIA*/
	alarm_init(&msm_batt_info.alarm,
			ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
			sec_bat_monitor_queue);

#ifdef CONFIG_HAS_EARLYSUSPEND
	 msm_batt_info.early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	msm_batt_info.early_suspend.suspend = msm_batt_early_suspend;
	msm_batt_info.early_suspend.resume = msm_batt_late_resume;
	register_early_suspend(&msm_batt_info.early_suspend);
#endif

	msm_batt_preupdate_psy_status(); /*status update*/

#ifdef CONFIG_WIBRO_CMC
	setup_timer(&use_wimax_timer, use_wimax_timer_func, 0);
#endif
	wake_lock_init(&vbus_wake_lock, WAKE_LOCK_SUSPEND, "vbus_wake_lock");
	wake_lock_init(&monitor_wake_lock, WAKE_LOCK_SUSPEND,
		       "sec-battery-monitor");

	if (msm_batt_info.batt_status == POWER_SUPPLY_STATUS_CHARGING) {
		msm_batt_info.cable_status = CABLE_TYPE_UNKNOWN;
		msm_batt_info.prev_cable = CABLE_TYPE_NONE;
		msm_batt_info.check_interval = CHECK_CHG_INTERVAL;
	} else {
		msm_batt_info.batt_status =
			POWER_SUPPLY_STATUS_DISCHARGING;
		msm_batt_info.cable_status = CABLE_TYPE_NONE;
		msm_batt_info.prev_cable = CABLE_TYPE_NONE;
		msm_batt_info.check_interval = CHECK_DSG_INTERVAL;
	}

	msm_batt_info.check_full_state = false;
	msm_batt_info.is_timeout_chgstop = false;
	msm_batt_info.check_full_state_cnt = 0;
	msm_batt_info.vbus_valid = 0;
	msm_batt_info.full_check_count = 0;
	msm_batt_info.factory_mode = false;
	msm_batt_info.check_cable_change = false;

#ifdef CONFIG_MAX17043_FUEL_GAUGE
	if (i2c_add_driver(&fg_i2c_driver))
		pr_err("%s : Can't add max17043 fuel gauge i2c drv"\
		"\n", __func__);

	/* check_quick_start(); */

	if (is_attached) {
		msm_batt_info.batt_capacity = get_level_from_fuelgauge();
		msm_batt_info.battery_voltage = get_voltage_from_fuelgauge();
	}
#endif	/* CONFIG_MAX17043_FUEL_GAUGE */

	msm_batt_driver_init = 1;

	/*For FSA related callback*/
	msm_batt_info.callback.set_cable = msm_set_cable;

	INIT_DELAYED_WORK_DEFERRABLE(&msm_batt_info.msm_batt_work,
		msm_batt_check_event);

	if (msm_batt_info.pdata->register_callbacks)
		msm_batt_info.pdata->register_callbacks
		(&msm_batt_info.callback);

	/* Need to check init connect check!*/
	/*msm_batt_cable_status_update();*/

#ifndef BATT_LED_CONTROL_DISABLE  /*LED is not controlled by battery driver */
	pr_info("[BATT] %s: battery led control enabled!\n", __func__);
	/* Led Toggle Timer init */
	hrtimer_init(&LedTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	LedTimer.function = led_timer_func;
#endif

	queue_delayed_work(msm_batt_info.msm_batt_wq,
		&msm_batt_info.msm_batt_work, 0);

	pr_debug("[BATT] %s : success!\n", __func__);

	return 0;
}

static int __devexit msm_batt_remove(struct platform_device *pdev)
{
	int rc;
	pr_info("%s\n", __func__);
	rc = msm_batt_cleanup();

	flush_workqueue(msm_batt_info.msm_batt_wq);
	destroy_workqueue(msm_batt_info.msm_batt_wq);

	cancel_delayed_work(&msm_batt_info.msm_batt_work);

	wake_lock_destroy(&vbus_wake_lock);
	wake_lock_destroy(&monitor_wake_lock);

#if defined(CONFIG_BATTERY_CTIA)
	alarm_cancel(&msm_batt_info.event_alarm);
#endif /*CONFIG_BATTERY_CTIA*/
	alarm_cancel(&msm_batt_info.alarm);

	if (rc < 0) {
		dev_err(&pdev->dev, "%s: msm_batt_cleanup  failed rc=%d\n",
			__func__, rc);
		return rc;
	}

	return 0;
}

/* static void msm_batt_shutdown(struct platform_device *pdev)
{
	int rc;
	pr_info("%s\n", __func__);
	rc = msm_batt_cleanup();

	if (rc < 0)
		dev_err(&pdev->dev, "%s: msm_batt_cleanup  failed rc=%d\n",
			__func__, rc);
} */

static const struct dev_pm_ops msm_bat_pm_ops = {
	.prepare = msm_batt_suspend,
	.complete = msm_batt_resume,
};

static struct platform_driver msm_batt_driver = {
	.probe = msm_batt_probe,
	.remove = __devexit_p(msm_batt_remove),
	/*.shutdown = msm_batt_shutdown,*/
	.driver = {
		   .name = "samsung-battery",
		   .owner = THIS_MODULE,
		   .pm = &msm_bat_pm_ops,
	},
};

static int __devinit msm_batt_init_rpc(void)
{
	int rc;

	msm_batt_info.msm_batt_wq =
		create_freezable_workqueue("msm_battery");

	if (!msm_batt_info.msm_batt_wq) {
		printk(KERN_ERR "%s: create workque failed\n", __func__);
		return -ENOMEM;
	}

	msm_batt_info.chg_ep = msm_rpc_connect_compatible(CHG_RPC_PROG,
		CHG_RPC_VER_4_1, 0);
	msm_batt_info.chg_api_version = CHG_RPC_VER_4_1;
	if (msm_batt_info.chg_ep == NULL) {
		pr_err("%s: rpc connect CHG_RPC_PROG = NULL\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(CHG_RPC_PROG,
			CHG_RPC_VER_3_1, 0);
		msm_batt_info.chg_api_version = CHG_RPC_VER_3_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(CHG_RPC_PROG,
			CHG_RPC_VER_1_1, 0);
		msm_batt_info.chg_api_version = CHG_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(CHG_RPC_PROG,
					       CHG_RPC_VER_1_3, 0);
		msm_batt_info.chg_api_version = CHG_RPC_VER_1_3;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		msm_batt_info.chg_ep = msm_rpc_connect_compatible(CHG_RPC_PROG,
			CHG_RPC_VER_2_2, 0);
		msm_batt_info.chg_api_version = CHG_RPC_VER_2_2;
	}
	if (IS_ERR(msm_batt_info.chg_ep)) {
		rc = PTR_ERR(msm_batt_info.chg_ep);
		pr_err("%s: FAIL: rpc connect for CHG_RPC_PROG. rc=%d\n",
			__func__, rc);
		msm_batt_info.chg_ep = NULL;
		return rc;
	}

	/* Get the real 1.x version */
	if (msm_batt_info.chg_api_version == CHG_RPC_VER_1_1)
		msm_batt_info.chg_api_version =
		msm_batt_get_charger_api_version();

	/* Fall back to 1.1 for default */
	if (msm_batt_info.chg_api_version < 0)
		msm_batt_info.chg_api_version = CHG_RPC_VER_1_1;
	msm_batt_info.batt_api_version = BATTERY_RPC_VER_4_1;

	msm_batt_info.batt_client = msm_rpc_register_client("battery",
		BATTERY_RPC_PROG, BATTERY_RPC_VER_4_1, 1, msm_batt_cb_func);

	if (msm_batt_info.batt_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. batt_client=NULL\n",
			__func__);
		return -ENODEV;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client = msm_rpc_register_client("battery",
			BATTERY_RPC_PROG, BATTERY_RPC_VER_1_1, 1,
			msm_batt_cb_func);
		msm_batt_info.batt_api_version = BATTERY_RPC_VER_1_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client = msm_rpc_register_client("battery",
			BATTERY_RPC_PROG, BATTERY_RPC_VER_2_1, 1,
			msm_batt_cb_func);
		msm_batt_info.batt_api_version = BATTERY_RPC_VER_2_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		msm_batt_info.batt_client = msm_rpc_register_client("battery",
			BATTERY_RPC_PROG, BATTERY_RPC_VER_5_1, 1,
			msm_batt_cb_func);
		msm_batt_info.batt_api_version = BATTERY_RPC_VER_5_1;
	}
	if (IS_ERR(msm_batt_info.batt_client)) {
		rc = PTR_ERR(msm_batt_info.batt_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n",
			__func__, rc);
		msm_batt_info.batt_client = NULL;
		return rc;
	}

	rc = platform_driver_register(&msm_batt_driver);

	if (rc < 0)
		pr_err("%s: FAIL: platform_driver_register. rc = %d\n",
		__func__, rc);

	return rc;
	}

static int __init msm_batt_init(void)
{
	int rc;

	pr_debug("%s: enter\n", __func__);

	rc = msm_batt_init_rpc();

	if (rc < 0) {
		pr_err("%s: FAIL: msm_batt_init_rpc.  rc=%d\n", __func__, rc);
		msm_batt_cleanup();
		return rc;
	}

	pr_info("%s: Charger/Battery = 0x%08x/0x%08x (RPC version)\n", __func__,
		msm_batt_info.chg_api_version, msm_batt_info.batt_api_version);

	return 0;
}

static void __exit msm_batt_exit(void)
{
	platform_driver_unregister(&msm_batt_driver);
}

#ifdef CONFIG_WIBRO_CMC
/*In case wimax is used, need to reconfigured*/
static void use_wimax_timer_func(unsigned long unused)
{
	/*msm_batt_info.chg_temp_event_check
		= msm_batt_info.chg_temp_event_check & (~USE_WIMAX);
	pr_info("%s: OFF (0x%x)\n", __func__,
		msm_batt_info.chg_temp_event_check);*/
}

int s3c_bat_use_wimax(int onoff)
{
	/*if (onoff) {
		del_timer_sync(&use_wimax_timer);
		msm_batt_info.chg_temp_event_check =
			msm_batt_info.chg_temp_event_check | USE_WIMAX;
		pr_info("%s: ON (0x%x)\n", __func__,
			msm_batt_info.chg_temp_event_check);
	} else {
		mod_timer(&use_wimax_timer,
			jiffies + msecs_to_jiffies(USE_MODULE_TIMEOUT));
		pr_info("%s: OFF111 (0x%x)\n", __func__,
			msm_batt_info.chg_temp_event_check);
	}

	return msm_batt_info.chg_temp_event_check;*/
	return 0;
}
EXPORT_SYMBOL(s3c_bat_use_wimax);
#endif

module_init(msm_batt_init);
module_exit(msm_batt_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Kiran Kandi, Qualcomm Innovation Center, Inc.");
MODULE_DESCRIPTION("Battery driver for Qualcomm MSM chipsets.");
MODULE_VERSION("1.0");
MODULE_ALIAS("platform:samsung_battery");
