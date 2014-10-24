/*
 *  SMB328A-charger.c
 *  SMB328A charger interface driver
 *
 *  Copyright (C) 2011 Samsung Electronics
 *
 *  <jongmyeong.ko@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/regulator/machine.h>
#include "smb328a_charger.h"

/* TODO: @disys temp */
struct smb328a_platform_data {
	void (*hw_init) (void);
};

/* Register define */
#define SMB328A_INPUT_AND_CHARGE_CURRENTS	0x00
#define SMB328A_CURRENT_TERMINATION		0x01
#define SMB328A_FLOAT_VOLTAGE			0x02
#define SMB328A_FUNCTION_CONTROL_A1		0x03
#define SMB328A_FUNCTION_CONTROL_A2		0x04
#define SMB328A_FUNCTION_CONTROL_B		0x05
#define SMB328A_OTG_PWR_AND_LDO_CONTROL		0x06
#define SMB328A_VARIOUS_CONTROL_FUNCTION_A	0x07
#define SMB328A_CELL_TEMPERATURE_MONITOR	0x08
#define SMB328A_INTERRUPT_SIGNAL_SELECTION	0x09
#define SMB328A_I2C_BUS_SLAVE_ADDRESS		0x0A

#define SMB328A_CLEAR_IRQ				0x30
#define SMB328A_COMMAND				0x31
#define SMB328A_INTERRUPT_STATUS_A			0x32
#define SMB328A_BATTERY_CHARGING_STATUS_A	0x33
#define SMB328A_INTERRUPT_STATUS_B			0x34
#define SMB328A_BATTERY_CHARGING_STATUS_B	0x35
#define SMB328A_BATTERY_CHARGING_STATUS_C	0x36
#define SMB328A_INTERRUPT_STATUS_C			0x37
#define SMB328A_BATTERY_CHARGING_STATUS_D	0x38
#define SMB328A_AUTOMATIC_INPUT_CURRENT_LIMMIT_STATUS	0x39

enum {
	BAT_NOT_DETECTED,
	BAT_DETECTED
};

enum {
	CHG_MODE_NONE,
	CHG_MODE_AC,
	CHG_MODE_USB,
	CHG_MODE_MISC,
	CHG_MODE_UNKNOWN
};

struct smb328a_chip {
	struct i2c_client *client;
	struct delayed_work work;
	struct power_supply psy_bat;
	struct smb328a_platform_data *pdata;
	struct mutex	mutex;

	int chg_mode;
	int lpm_chg_mode;
};
static struct smb328a_chip *g_chip;

/* static enum power_supply_property smb328a_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
}; */

static int smb328a_write_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0) {
		printk(KERN_ERR "[SMB328a][%s] err! ret = %d try again!\n",
			__func__, ret);
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	}

	return ret;
}

static int smb328a_read_reg(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0) {
		printk(KERN_ERR "[SMB328a][%s] err! ret = %d try again!\n",
			__func__, ret);
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	}

	return ret;
}

static void smb328a_print_reg(struct i2c_client *client, int reg)
{
	int data = 0;

	data = i2c_smbus_read_byte_data(client, reg);

	if (data < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, data);
	else
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n",
			__func__, reg, data);
}

static void smb328a_print_all_regs(struct i2c_client *client)
{
	printk(KERN_ERR "[SMB328a][%s] ----- START -----\n", __func__);

	smb328a_print_reg(client, 0x31);
	smb328a_print_reg(client, 0x32);
	smb328a_print_reg(client, 0x33);
	smb328a_print_reg(client, 0x34);
	smb328a_print_reg(client, 0x35);
	smb328a_print_reg(client, 0x36);
	smb328a_print_reg(client, 0x37);
	smb328a_print_reg(client, 0x38);
	smb328a_print_reg(client, 0x39);
	smb328a_print_reg(client, 0x00);
	smb328a_print_reg(client, 0x01);
	smb328a_print_reg(client, 0x02);
	smb328a_print_reg(client, 0x03);
	smb328a_print_reg(client, 0x04);
	smb328a_print_reg(client, 0x05);
	smb328a_print_reg(client, 0x06);
	smb328a_print_reg(client, 0x07);
	smb328a_print_reg(client, 0x08);
	smb328a_print_reg(client, 0x09);
	smb328a_print_reg(client, 0x0a);

	printk(KERN_ERR "[SMB328a][%s] ----- END -----\n", __func__);
}

static void smb328a_allow_volatile_writes(struct i2c_client *client)
{
	int val;
	u8 data;

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if ((val >= 0) && !(val & 0x80)) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
			SMB328A_COMMAND, data);
		data |= (0x1 << 7);
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0)
			printk(KERN_ERR "%s : error!\n", __func__);
		val = smb328a_read_reg(client, SMB328A_COMMAND);
		if (val >= 0) {
			data = (u8) data;
			printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
				__func__, SMB328A_COMMAND, data);
		}
	}
}

static void smb328a_set_command_reg(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);
	int val, reg;
	u8 data;

	reg = SMB328A_COMMAND;
	val = smb328a_read_reg(client, reg);
	if (val >= 0) {
		data = (u8)val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n",
			__func__, reg, data);
		if (chip->chg_mode == CHG_MODE_AC ||\
			chip->chg_mode == CHG_MODE_MISC ||\
			chip->chg_mode == CHG_MODE_UNKNOWN)
			data = 0xad; /*0xbc*/
		else
			data = 0xa9; /* 0xb8 usb */
		if (smb328a_write_reg(client, reg, data) < 0)
			pr_err("%s : error!\n", __func__);
		val = smb328a_read_reg(client, reg);
		if (val >= 0) {
			data = (u8)data;
			pr_info("%s : => reg (0x%x) = 0x%x\n",
				__func__, reg, data);
		}
	}
}

static void smb328a_clear_irqs(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);
	int reg;

	pr_info("%s\n", __func__);
	reg = SMB328A_CLEAR_IRQ;
	if (smb328a_write_reg(chip->client, reg, 0xff) < 0)
		pr_err("%s : irq clear error!\n", __func__);
}

static void smb328a_changer_function_conrol(struct i2c_client *client,\
	int chg_current)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);
	int val;
	u8 data, set_data;

	smb328a_allow_volatile_writes(client);

	val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
			SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
		if (chip->chg_mode == CHG_MODE_AC) {
			set_data = 0xff; /* fast MAX 1200mA, pre 125mA */
		} else if (chip->chg_mode == CHG_MODE_MISC) {
			set_data = 0x5f; /* fast 700mA, pre 125mA */
		} else { /*CHG_MODE_USB*/
			set_data = 0x1f; /* fast 500mA, pre-fast 125mA */
		}

		if (data != set_data) {
			data = set_data;
			if (smb328a_write_reg(client,
				SMB328A_INPUT_AND_CHARGE_CURRENTS, data) < 0)
				pr_err("%s : error!\n", __func__);
			val = smb328a_read_reg(client,
				SMB328A_INPUT_AND_CHARGE_CURRENTS);
			if (val >= 0) {
				data = (u8)val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_INPUT_AND_CHARGE_CURRENTS,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_CURRENT_TERMINATION);
	if (val >= 0) {
		data = (u8)val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
			SMB328A_CURRENT_TERMINATION, data);

		if (chip->chg_mode == CHG_MODE_AC) {
			/* 101x xxxx : AC Input Current Limit = 1000mA
			xxx1 xxxx : Pre-Bias Current = Enabled
			xxxx x0xx : Automatic Input Current Limit = Enabled
			xxxx xx00 : Automatic Input Current Limit Threshold
			    = 4.25V */
			if (chg_current == 1000)
				set_data = 0xb0; /* input 1A */
			else if (chg_current == 900)
				set_data = 0x90; /* input 900mA */
			else
				set_data = 0x70; /* input 800mA */
		} else if (chip->chg_mode == CHG_MODE_MISC) {
			set_data = 0x50; /* input 700mA */
		} else {
			set_data = 0x10; /* input 450mA */
		}

		if (data != set_data) { /* AICL enable */
			data = set_data;
			if (smb328a_write_reg(client,\
				SMB328A_CURRENT_TERMINATION, data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client,\
				SMB328A_CURRENT_TERMINATION);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_ERR "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_CURRENT_TERMINATION,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FLOAT_VOLTAGE);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_FLOAT_VOLTAGE, data);
		#if defined(CONFIG_MACH_ICON)
		if (data != 0xda) {
			data = 0xda; /* 4.36V float voltage */
		#else
		if (data != 0xca) {
			data = 0xca;	/* 4.2V float voltage */
		#endif
			if (smb328a_write_reg
			    (client, SMB328A_FLOAT_VOLTAGE, data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client, SMB328A_FLOAT_VOLTAGE);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_FLOAT_VOLTAGE,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A1);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_FUNCTION_CONTROL_A1, data);
		if (data != 0xd2) {
			/* 1xxx xxxx : Automatic Recharge = Diabled
			* x1xx xxxx : Current Termination
			*	       = Not Allowed to end a charge cycle
			* xx01 1xxx : Pre-Charge to Fast-charge
			*	       Voltage Threshold = 2.5V
			* xxxx x0xx : LDO input Under-voltage Level = 3.50V
			* xxxx xx10 : Not Used (Default) */
			data = 0xd2;
			if (smb328a_write_reg
			    (client, SMB328A_FUNCTION_CONTROL_A1, data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client,
				SMB328A_FUNCTION_CONTROL_A1);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_FUNCTION_CONTROL_A1,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_A2);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_FUNCTION_CONTROL_A2, data);
		if (data != 0x4d) {
			/* 0xxx xxxx : STATOutput = Indicates Charging State
			  * x1xx xxxx : Battery OV = Battery OV does cause
			  *	       charge cycle to end
			  * xx0x xxxx : Reload Functionality = Reload
			  *	       non-volatile values on
			  *	       valid input power presence
			  * xxx0 xxxx : Pre-to-fast charge Functionality
			  *	       = Enabled
			  * xxxx 1xxx : Charge Safety Timers = disabled
			  * xxxx x1xx : Not Used
			  * xxxx xx0x : Watchdog Timer = disabled
			  * xxxx xxx1 : Interrupt(IRQ) Output = Enabled */
			data = 0x4d;
			if (smb328a_write_reg
			    (client, SMB328A_FUNCTION_CONTROL_A2, data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client,
					     SMB328A_FUNCTION_CONTROL_A2);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_FUNCTION_CONTROL_A2,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_FUNCTION_CONTROL_B, data);
		if (data != 0x0) {
			/* 0xxx xxxx : Battery missing Detection Method=BMD pin
			  * xxxx 00xx : Enable(EN) Control = "0" in 0x31[4]
			  *	       turns on (enables) charger
			  * xxxx xx0x : Fast Charge Current Control Method
			  *	       = Config. Reg.
			  * xxxx xxx0 : Input Current Limit Control Method
			  *	       = Command Reg. */
			data = 0x0;
			if (smb328a_write_reg
			    (client, SMB328A_FUNCTION_CONTROL_B, data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client,
					     SMB328A_FUNCTION_CONTROL_B);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_FUNCTION_CONTROL_B,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_OTG_PWR_AND_LDO_CONTROL);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_OTG_PWR_AND_LDO_CONTROL, data);
#if defined(CONFIG_TARGET_LOCALE_USA) || defined(CONFIG_MACH_PREVAIL2)\
	|| defined(CONFIG_MACH_ICON) || defined(CONFIG_MACH_RAY)
		if (data != 0x45) {
			data = 0x45;
#else
		/*  0xxx xxxx : Battery Missing Detection = Disable
		  *  x1xx xxxx : Automatic Recharge Threshold = 105mV
		  *  xx1x xxxx : LDO Control = Disable
		  *  xxx0 0xxx : OTG Current Limit = 950mA
		  *  xxxx x101 : OTG Mode UVLO Threshold = 3.30V  */
		if (data != 0x65 /* 0xc5 */) {
			data = 0x65;	/* 0xc5 SUMMIT_REQ */
#endif
			if (smb328a_write_reg(client,
				SMB328A_OTG_PWR_AND_LDO_CONTROL, data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client,
					     SMB328A_OTG_PWR_AND_LDO_CONTROL);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_OTG_PWR_AND_LDO_CONTROL,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
		if (data != 0xf6) {
			/* this can be changed with top-off setting
			 * 111x xxxx : STAT Assertion Termination Current=200mA
			 * xxx1 0xxx : Float Volatge Compensation Level = 180mV
			 * xxxx x11x : Thermistor Current = 0uA(Disabled)
			 * xxxx xxx0 : Float Voltage Compensation = Disabled */
			data = 0xf6;
			if (smb328a_write_reg
			    (client, SMB328A_VARIOUS_CONTROL_FUNCTION_A,
			     data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client,
				SMB328A_VARIOUS_CONTROL_FUNCTION_A);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_VARIOUS_CONTROL_FUNCTION_A,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_CELL_TEMPERATURE_MONITOR);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_CELL_TEMPERATURE_MONITOR, data);
		if (data != 0x0) {
			data = 0x0;
			if (smb328a_write_reg
			    (client, SMB328A_CELL_TEMPERATURE_MONITOR,
			     data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client,
				SMB328A_CELL_TEMPERATURE_MONITOR);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_CELL_TEMPERATURE_MONITOR,
					data);
			}
		}
	}

	val = smb328a_read_reg(client, SMB328A_INTERRUPT_SIGNAL_SELECTION);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_INTERRUPT_SIGNAL_SELECTION, data);
		if (data != 0x1) {
			/* Does not Trigger interrupt Signal(IRQ) */
			data = 0x1;
			if (smb328a_write_reg
			    (client, SMB328A_INTERRUPT_SIGNAL_SELECTION,
			     data) < 0)
				printk(KERN_ERR "%s : error!\n", __func__);
			val = smb328a_read_reg(client,
				    SMB328A_INTERRUPT_SIGNAL_SELECTION);
			if (val >= 0) {
				data = (u8) val;
				printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
					__func__,
					SMB328A_INTERRUPT_SIGNAL_SELECTION,
					data);
			}
		}
	}

	/* command register setting */
/*#if 0
	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
			SMB328A_COMMAND, data);
		data |= (0x1 << 3 | 0x1 << 5);
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0)
			printk(KERN_ERR "%s : error!\n", __func__);
		val = smb328a_read_reg(client, SMB328A_COMMAND);
		if (val >= 0) {
			data = (u8) data;
			printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
				__func__, SMB328A_COMMAND, data);
		}
	}
#endif */
}

/* static int smb328a_check_charging_status(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	int ret = -1;

	// printk("%s :\n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_BATTERY_CHARGING_STATUS_C, data);

		ret = (data & (0x3 << 1)) >> 1;
		printk(KERN_INFO "%s : status = 0x%x\n", __func__, data);
	}

	return ret;
} */

/* static bool smb328a_check_is_charging(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

	printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_BATTERY_CHARGING_STATUS_C, data);

		if (data & 0x1)
			ret = true;	// charger enabled
	}

	return ret;
} */

bool smb328a_check_bat_full(void)
{
	int val;
	u8 data = 0;
	bool ret = false;

	pr_info("[SMB328a][%s]\n", __func__);

	val = smb328a_read_reg(g_chip->client,
		SMB328A_BATTERY_CHARGING_STATUS_C);
	if (val >= 0) {
		data = (u8) val;
		pr_info("%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_BATTERY_CHARGING_STATUS_C, data);

		if (data & (0x1 << 6))
			ret = true;/* full*/
	}

	return ret;
}
EXPORT_SYMBOL(smb328a_check_bat_full);

void smb328a_watchdog_enable(int on_off)
{
	int val;
	u8 data = 0;
	bool ret = false;

	smb328a_allow_volatile_writes(g_chip->client);
	if (on_off == 0) {
		val = smb328a_read_reg(g_chip->client,
			SMB328A_FUNCTION_CONTROL_A2);
		if (val >= 0) {
			data = (u8) val;
			printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
				SMB328A_FUNCTION_CONTROL_A2, data);
			if (data != 0x4d) {
				/* 0xxx xxxx : STATOutput
				  * = Indicates Charging State
				  * x1xx xxxx : Battery OV
				  * = Battery OV does cause
				  *        charge cycle to end
				  * xx0x xxxx : Reload Functionality = Reload
				  *        non-volatile values on
				  *        valid input power presence
				  * xxx0 xxxx : Pre-to-fast charge Functionality
				  *        = Enabled
				  * xxxx 1xxx : Charge Safety Timers = disabled
				  * xxxx x1xx : Not Used
				  * xxxx xx0x : Watchdog Timer
				  * = disabled
				  * xxxx xxx1 : Interrupt(IRQ) Output
				  * = Enabled */
				data = 0x4d;
				if (smb328a_write_reg(g_chip->client,
					SMB328A_FUNCTION_CONTROL_A2, data) < 0)
					printk(KERN_ERR "%s : error!\n",
						__func__);
					val = smb328a_read_reg(g_chip->client,
						SMB328A_FUNCTION_CONTROL_A2);
					if (val >= 0) {
						data = (u8) val;
						printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
						__func__,
						SMB328A_FUNCTION_CONTROL_A2,
					      data);
				}
			}
		}
		return ret;
	 } else {
		val = smb328a_read_reg(g_chip->client,
			SMB328A_FUNCTION_CONTROL_A2);
		if (val >= 0) {
			data = (u8) val;
			printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
				SMB328A_FUNCTION_CONTROL_A2, data);
			if (data != 0x4f) {
				/* 0xxx xxxx : STATOutput
				  * = Indicates Charging State
				  * x1xx xxxx : Battery OV
				  * = Battery OV does cause
				  *        charge cycle to end
				  * xx0x xxxx : Reload Functionality = Reload
				  *        non-volatile values on
				  *        valid input power presence
				  * xxx0 xxxx : Pre-to-fast charge Functionality
				  *        = Enabled
				  * xxxx 1xxx : Charge Safety Timers = disabled
				  * xxxx x1xx : Not Used
				  * xxxx xx1x : Watchdog Timer = enabled
				  * xxxx xxx1 : Interrupt(IRQ) Output
				  * = Enabled */
				data = 0x4f;
				if (smb328a_write_reg(g_chip->client,
					SMB328A_FUNCTION_CONTROL_A2, data) < 0)
					printk(KERN_ERR "%s : error!\n",
						__func__);
					val = smb328a_read_reg(g_chip->client,
						SMB328A_FUNCTION_CONTROL_A2);
				if (val >= 0) {
					data = (u8) val;
					printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
						__func__,
						SMB328A_FUNCTION_CONTROL_A2,
						data);
				}
			}
		}
		return ret;
	}
}
EXPORT_SYMBOL(smb328a_watchdog_enable);

/* vf check */
/* static bool smb328a_check_bat_missing(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

	printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_B);
	if (val >= 0) {
		data = (u8) val;
		// printk("%s : reg (0x%x) = 0x%x\n", __func__,
		// SMB328A_BATTERY_CHARGING_STATUS_B, data);

		if (data & (0x1 << 1)) {
			printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
			       SMB328A_BATTERY_CHARGING_STATUS_B, data);
			ret = true;	// missing battery
		}
	}

	return ret;
} */

/* whether valid dcin or not */
static bool smb328a_check_vdcin(struct i2c_client *client)
{
	int val;
	u8 data = 0;
	bool ret = false;

	printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	val = smb328a_read_reg(client, SMB328A_BATTERY_CHARGING_STATUS_A);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_BATTERY_CHARGING_STATUS_A, data);

		if (data & (0x1 << 1))
			ret = true;
	}

	return ret;
}

/* static int smb328a_chg_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct smb328a_chip *chip =
		container_of(psy, struct smb328a_chip, psy_bat);

	printk(KERN_ERR "[SMB328a][%s] ### psp[%d]\n",
		__func__, psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (smb328a_check_vdcin(chip->client))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			// val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (smb328a_check_bat_missing(chip->client))
			val->intval = BAT_NOT_DETECTED;
		else
			val->intval = BAT_DETECTED;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		// battery is always online
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		if (smb328a_check_bat_full(chip->client))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		switch (smb328a_check_charging_status(chip->client)) {
		case 0:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		case 1:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
			break;
		case 2:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case 3:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		default:
			printk(KERN_ERR "get charge type error!\n");
			return -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (smb328a_check_is_charging(chip->client))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
} */

static int smb328a_set_top_off(struct i2c_client *client, int top_off)
{
	int val, set_val = 0;
	u8 data;

	/*smb328a_allow_volatile_writes(client);*/

	set_val = top_off / 25;
	set_val -= 1;

	if (set_val < 0 || set_val > 7) {
		printk(KERN_ERR "%s: invalid topoff set value(%d)\n",
			__func__, set_val);
		return -EINVAL;
	}

	val = smb328a_read_reg(client, SMB328A_INPUT_AND_CHARGE_CURRENTS);
	if (val >= 0) {
		data = (u8) val & ~(0x07 << 0);
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
		data |= (set_val << 0);
		if (smb328a_write_reg
		    (client, SMB328A_INPUT_AND_CHARGE_CURRENTS, data) < 0) {
			printk(KERN_ERR "%s : error!\n", __func__);
			return -EIO;
		}
		data =
		    smb328a_read_reg(client,
			SMB328A_INPUT_AND_CHARGE_CURRENTS);
		printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_INPUT_AND_CHARGE_CURRENTS, data);
	}

	val = smb328a_read_reg(client, SMB328A_VARIOUS_CONTROL_FUNCTION_A);
	if (val >= 0) {
		data = (u8) val & ~(0x07 << 5);
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
		data |= (set_val << 5);
		if (smb328a_write_reg
		    (client, SMB328A_VARIOUS_CONTROL_FUNCTION_A, data) < 0) {
			printk(KERN_ERR "%s : error!\n", __func__);
			return -EIO;
		}
		data = smb328a_read_reg(client,
				     SMB328A_VARIOUS_CONTROL_FUNCTION_A);
		printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_VARIOUS_CONTROL_FUNCTION_A, data);
	}

	return 0;
}

static int smb328a_set_charging_current(struct i2c_client *client,
					int chg_current)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	if (chg_current < 100 || chg_current > 1050)
		return -EINVAL;

	if (chg_current == 500) {
		chip->chg_mode = CHG_MODE_USB;
	} else if ((chg_current == 1000) || (chg_current == 900)
		|| (chg_current == 800)) {
		chip->chg_mode = CHG_MODE_AC;
	} else if (chg_current == 700) {
		chip->chg_mode = CHG_MODE_MISC;
	} else if (chg_current == 450) {
		chip->chg_mode = CHG_MODE_UNKNOWN;
	} else {
		printk(KERN_ERR "%s: can't read register(0x%x)\n",
		       __func__, SMB328A_CURRENT_TERMINATION);
		return -EIO;
	}
	return 0;
}

/* static int smb328a_enable_otg(struct i2c_client *client)
{
	int val;
	u8 data;

	printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8) val;
		if (data != 0x9a) {
			printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
			       SMB328A_COMMAND, data);
			data = 0x9a;
			if (smb328a_write_reg(client, SMB328A_COMMAND, data) <
			    0) {
				printk(KERN_ERR "%s : error!\n", __func__);
				return -EIO;
			}
			msleep(100);

			data = smb328a_read_reg(client, SMB328A_COMMAND);
			printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n",
				__func__, SMB328A_COMMAND, data);
		}
	}
	return 0;
} */

/* static int smb328a_disable_otg(struct i2c_client *client)
{
	int val;
	u8 data;

	printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	val = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_FUNCTION_CONTROL_B, data);
		data = 0x0c;
		if (smb328a_write_reg(client, SMB328A_FUNCTION_CONTROL_B, data)
		    < 0) {
			printk(KERN_ERR "%s : error!\n", __func__);
			return -EIO;
		}
		msleep(100);
		data = smb328a_read_reg(client, SMB328A_FUNCTION_CONTROL_B);
		printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n", __func__,
		       SMB328A_FUNCTION_CONTROL_B, data);

	}

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8) val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
			SMB328A_COMMAND, data);
		data = 0x98;
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			printk(KERN_ERR "%s : error!\n", __func__);
			return -EIO;
		}
		msleep(100);
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n", __func__,
			SMB328A_COMMAND, data);
#if 0	// TODO: @disys compile error, re-check
		fsa9480_otg_detach();
#endif
	}
	return 0;
}*/

static void smb328a_ldo_disable(struct i2c_client *client)
{
	int val, reg;
	u8 data;

	dev_info(&client->dev, "%s :\n", __func__);

	smb328a_allow_volatile_writes(client);

	reg = SMB328A_OTG_PWR_AND_LDO_CONTROL;
	val = smb328a_read_reg(client, reg);
	if (val >= 0) {
		data = (u8)val;
		dev_info(&client->dev, "%s : reg (0x%x) = 0x%x\n",
			__func__, reg, data);

		data |= (0x1 << 5);
		if (smb328a_write_reg(client, reg, data) < 0)
			pr_err("%s : error!\n", __func__);
		val = smb328a_read_reg(client, reg);
		if (val >= 0) {
			data = (u8)val;
			dev_info(&client->dev, "%s : => reg (0x%x) = 0x%x\n",
				__func__, reg, data);
		}
	}
}

static int smb328a_enable_charging(struct i2c_client *client)
{
	int val;
	u8 data;
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		printk(KERN_INFO "%s : reg (0x%x) = 0x%x\n", __func__,
			SMB328A_COMMAND, data);
		/* data &= ~(0x1 << 4);  */ /* "0" turn off the charger */
		if (chip->chg_mode == CHG_MODE_AC ||\
			chip->chg_mode == CHG_MODE_MISC ||\
			chip->chg_mode == CHG_MODE_UNKNOWN)
			data = 0xad;/*data = 0x8c;*/
		else if (chip->chg_mode == CHG_MODE_USB)
			data = 0xa9;/*data = 0x88;*/
		else /*no charge*/
			data = 0xb9;/*data = 0x98;*/
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			printk(KERN_ERR "%s : error!\n", __func__);
			return -EIO;
		}
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		printk(KERN_INFO "%s : => reg (0x%x) = 0x%x\n", __func__,
			SMB328A_COMMAND, data);
	}

	return 0;
}

static int smb328a_disable_charging(struct i2c_client *client)
{
	int val;
	u8 data;
	/* struct smb328a_chip *chip = i2c_get_clientdata(client); */

	val = smb328a_read_reg(client, SMB328A_COMMAND);
	if (val >= 0) {
		data = (u8)val;
		pr_info("%s : reg (0x%x) = 0x%x\n", __func__, SMB328A_COMMAND,
		       data);
		/* data |= (0x1 << 4); */ /* "1" turn off the charger */
		data = 0xb9;/*0xD8  0x98 */
		if (smb328a_write_reg(client, SMB328A_COMMAND, data) < 0) {
			printk(KERN_ERR "%s : error!\n", __func__);
			return -EIO;
		}
		data = smb328a_read_reg(client, SMB328A_COMMAND);
		pr_info("%s : => reg (0x%x) = 0x%x\n", __func__,
			SMB328A_COMMAND, data);
	}

	return 0;
}

int smb328a_charger_control(int fast_charging_current, int termination_current,
			    int on, int usb_cdp)
{
	int ret;

	if (!g_chip) {
		pr_err("[SMB328a] %s: device not found !!!\n", __func__);
		return -ENODEV;
	}

	if (on) {
		msleep(200); /*wait until charger is completely attached*/
		/* step1) Set charging current */
		ret = smb328a_set_charging_current(g_chip->client,
			fast_charging_current);
		smb328a_set_command_reg(g_chip->client);
		smb328a_changer_function_conrol(g_chip->client,
			fast_charging_current);
		pr_info("[BATT] %s: smb328a_set_charging_current() ret = %d\n",
			__func__, ret);

		/* step2) Set top-off current */
		if (termination_current < 25 || termination_current > 200) {
			dev_err(&g_chip->client->dev,
				"%s: invalid topoff current(%d)\n", __func__,
				termination_current);
			return -EINVAL;
		}
		ret = smb328a_set_top_off(g_chip->client, termination_current);
		pr_info("[BATT] %s: smb328a_set_top_off() ret = %d\n",
			__func__, ret);

		/* step3) Enable/Disable charging */
		if (((g_chip->chg_mode != CHG_MODE_USB) && !usb_cdp)
			|| g_chip->lpm_chg_mode)
				smb328a_ldo_disable(g_chip->client);

		#if defined(CONFIG_MACH_ICON)
		ret = smb328a_disable_charging(g_chip->client);
		pr_info("[BATT] %s: smb328a_disable_charging() ret = %d\n",
			__func__, ret);
		#endif

		ret = smb328a_enable_charging(g_chip->client);
		pr_info("[BATT] %s: smb328a_enable_charging() ret = %d\n",
			__func__, ret);
	} else {
		ret = smb328a_disable_charging(g_chip->client);
		pr_info("[BATT] %s: smb328a_disable_charging() ret = %d\n",
			__func__, ret);
	}

	/*smb328a_print_all_regs(g_chip->client);*/
	return 0 ;
}
EXPORT_SYMBOL(smb328a_charger_control);

/* static int smb328a_chg_set_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct smb328a_chip *chip = container_of(psy, struct smb328a_chip,
		psy_bat);
	int ret;

	printk(KERN_ERR "[SMB328a][%s]\n", __func__);

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW: // step1) Set charging current
		smb328a_changer_function_conrol(chip->client);
		// smb328a_print_all_regs(chip->client);
		ret = smb328a_set_charging_current(chip->client, val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL: // step2) Set top-off current
		if (val->intval < 25 || val->intval > 200) {
			dev_err(&chip->client->dev,
				"%s: invalid topoff current(%d)\n", __func__,
				val->intval);
			return -EINVAL;
		}
		ret = smb328a_set_top_off(chip->client, val->intval);
		break;
	case POWER_SUPPLY_PROP_STATUS:	// step3) Enable/Disable charging
		if (val->intval == POWER_SUPPLY_STATUS_CHARGING)
			ret = smb328a_enable_charging(chip->client);
		else
			ret = smb328a_disable_charging(chip->client);
		// smb328a_print_all_regs(chip->client);
		break;
#if 0	// TODO: @disys compile error, re-check
	case POWER_SUPPLY_PROP_OTG:
		if (val->intval == POWER_SUPPLY_CAPACITY_OTG_ENABLE) {
			smb328a_changer_function_conrol(chip->client);
			ret = smb328a_enable_otg(chip->client);
		} else
			ret = smb328a_disable_otg(chip->client);
		break;
#endif
	default:
		return -EINVAL;
	}
	return ret;
} */

static irqreturn_t smb328a_int_work_func(int irq, void *smb_chip)
{
	struct smb328a_chip *chip = smb_chip;
	/*int val, reg;
	u8 intr_a = 0;
	u8 intr_b = 0;
	u8 intr_c = 0;
	u8 chg_status = 0;*/
	static int vbus_in;

	pr_info("%s\n", __func__);

	/*reg = SMB328A_INTERRUPT_STATUS_A;
	val = smb328a_read_reg(chip->client, reg);
	if (val >= 0) {
		intr_a = (u8)val;
		pr_info("%s : reg (0x%x) = 0x%x\n", __func__, reg, intr_a);
	}

	reg = SMB328A_INTERRUPT_STATUS_B;
	val = smb328a_read_reg(chip->client, reg);
	if (val >= 0) {
		intr_b = (u8)val;
		pr_info("%s : reg (0x%x) = 0x%x\n", __func__, reg, intr_b);
	}

	reg = SMB328A_INTERRUPT_STATUS_C;
	val = smb328a_read_reg(chip->client, reg);
	if (val >= 0) {
		intr_c = (u8)val;
		pr_info("%s : reg (0x%x) = 0x%x\n", __func__, reg, intr_c);
	}

	reg = SMB328A_BATTERY_CHARGING_STATUS_C;
	val = smb328a_read_reg(chip->client, reg);
	if (val >= 0) {
		chg_status = (u8)val;
		pr_info("%s : reg (0x%x) = 0x%x\n", __func__, reg, chg_status);
	}*/

	vbus_in = smb328a_check_vdcin(chip->client);
	pr_info("%s : VBUS IN = %d\n", __func__, vbus_in);

	msm_batt_cable_status_update_ext(vbus_in);

	/* clear IRQ */
	smb328a_clear_irqs(chip->client);

	return IRQ_HANDLED;
}

static int __devinit smb328a_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct smb328a_chip *chip;
	int ret;

	pr_info(KERN_ERR "[SMB328a][%s]\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	pr_info("%s: SMB328A driver Loading!\n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	pr_info("%s: after kzalloc() : chip = %X\n",
		__func__, (unsigned int)chip);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	/* chip->pdata = client->dev.platform_data; */

	i2c_set_clientdata(client, chip);

	/* chip->pdata->hw_init(); */ /* important */
	chip->chg_mode = CHG_MODE_NONE;
	chip->lpm_chg_mode = 0;

	if (charging_boot == 1) {
		chip->lpm_chg_mode = 1;
		printk(KERN_INFO "%s : is lpm charging mode (%d)\n",
			__func__, chip->lpm_chg_mode);
	}

	smb328a_clear_irqs(chip->client);

	ret = request_threaded_irq(chip->client->irq, NULL,
		smb328a_int_work_func, IRQF_TRIGGER_FALLING,
		"smb328a", chip);

	if (ret) {
		pr_err("%s : Failed to request smb328a charger irq\n",
			__func__);
		goto err_request_irq;
	}

	ret = enable_irq_wake(chip->client->irq);
	if (ret) {
		pr_err("%s : Failed to enable smb328a charger irq wake\n",
			__func__);
		goto err_irq_wake;
	}

	g_chip = chip;
	return 0;

err_irq_wake:
	free_irq(chip->client->irq, NULL);
err_request_irq:
	power_supply_unregister(&chip->psy_bat);
	kfree(chip);

	return ret;
}

static int __devexit smb328a_remove(struct i2c_client *client)
{
	struct smb328a_chip *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->psy_bat);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM
static int smb328a_suspend(struct i2c_client *client, pm_message_t state)
{
	/* struct smb328a_chip *chip = i2c_get_clientdata(client); */

	return 0;
}

static int smb328a_resume(struct i2c_client *client)
{
	/* struct smb328a_chip *chip = i2c_get_clientdata(client); */

	return 0;
}
#else
#define smb328a_suspend NULL
#define smb328a_resume NULL
#endif				/* CONFIG_PM */

static const struct i2c_device_id smb328a_id[] = {
	{"smb328a", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, smb328a_id);

static struct i2c_driver smb328a_i2c_driver = {
	.driver = {
		   .name = "smb328a",
		   },
	.probe = smb328a_probe,
	.remove = __devexit_p(smb328a_remove),
	.suspend = smb328a_suspend,
	.resume = smb328a_resume,
	.id_table = smb328a_id,
};

static int __init smb328a_init(void)
{
	printk(KERN_ERR "[SMB328a][%s]\n", __func__);
	return i2c_add_driver(&smb328a_i2c_driver);
}

module_init(smb328a_init);

static void __exit smb328a_exit(void)
{
	printk(KERN_ERR "[SMB328a][%s]\n", __func__);
	i2c_del_driver(&smb328a_i2c_driver);
}

module_exit(smb328a_exit);

MODULE_DESCRIPTION("SMB328A charger control driver");
MODULE_AUTHOR("<jongmyeong.ko@samsung.com>");
MODULE_LICENSE("GPL");
