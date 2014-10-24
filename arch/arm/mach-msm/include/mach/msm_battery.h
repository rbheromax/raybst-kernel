/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
 */


#ifndef __MSM_BATTERY_H__
#define __MSM_BATTERY_H__

#define NO_CHG     0x00000000
#define AC_CHG     0x00000001
#define USB_CHG    0x00000002

/*For FSA related callback*/
enum cable_type_t {
	CABLE_TYPE_NONE = 0,
	CABLE_TYPE_UNKNOWN,
	CABLE_TYPE_USB,
	CABLE_TYPE_TA,
	CABLE_TYPE_USB_OTG,
	CABLE_TYPE_CAR_DOCK,
	CABLE_TYPE_DESK_DOCK,
	CABLE_TYPE_JIG,
	CABLE_TYPE_UART,
	CABLE_TYPE_CDP,
};

/*For FSA related callback*/
struct msm_battery_callback {
	void (*set_cable)(struct msm_battery_callback *ptr,
		enum cable_type_t status);
};

/*For FSA related callback*/
struct msm_charger_data {
	void (*register_callbacks)(struct msm_battery_callback *ptr);
};

struct msm_psy_batt_pdata {
	struct msm_charger_data *charger;/*For FSA*/
	u32 voltage_max_design;
	u32 voltage_min_design;
	u32 voltage_fail_safe;
	u32 avail_chg_sources;
	u32 batt_technology;
	u32 (*calculate_capacity)(u32 voltage);
};

#endif
