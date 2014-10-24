/*
 * ====================================================
 *  Name : samsung_battery.h
 *  Description :  Extern functions used for samsung_battery.c
 * ====================================================
 */

extern int charging_boot;

#if defined(CONFIG_CHARGER_SMB328A)
extern int smb328a_charger_control(int fast_charging_current,
				   int termination_current, int on,
				   int usb_cdp);
extern bool smb328a_check_bat_full(void);
#if defined(CONFIG_MACH_RAY)
extern void smb328a_watchdog_enable(int on_off);
#endif
#endif

/* Temp for USB OTG charging problem */
enum chg_type {
	USB_CHG_TYPE__SDP,
	USB_CHG_TYPE__CARKIT,
	USB_CHG_TYPE__WALLCHARGER,
	USB_CHG_TYPE__INVALID
};

/* use 2step charge termination */
#if defined(CONFIG_MACH_RAY)
#define USE_2STEP_TERM
#else
#undef USE_2STEP_TERM
#endif

enum select_full_check_type {
	FULLCHARGED_ADC		= 0,
	FULLCHARGED_SOC,
};

enum status_full_type {
	STATUS_NOT_FULL		= 0,
	STATUS_1ST_FULL,
	STATUS_2ND_FULL,
	STATUS_ABS_FULL,
};

extern void hsusb_chg_connected(enum chg_type chgtype);
extern void hsusb_chg_vbus_draw(unsigned mA);

extern int is_reset_soc;

/* ------------------------------ */

int batt_restart(void);

/* ------------------------------ */

#ifdef CONFIG_MAX17043_FUEL_GAUGE
extern void fuel_gauge_rcomp(int state);
extern void fg_wakeunlock(int soc);
#endif

extern int fsa9485_vbus_valid(void);
