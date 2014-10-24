/*
 *
 * Copyright (C) 2009 SAMSUNG ELECTRONICS.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/slab.h>
#include <linux/i2c.h>

#define ADJUSTED_SOC

/* Register address */
#define VCELL0_REG			0x02
#define VCELL1_REG			0x03
#define SOC0_REG			0x04
#define SOC1_REG			0x05
#define MODE0_REG			0x06
#define MODE1_REG			0x07
#define RCOMP0_REG			0x0C
#define RCOMP1_REG			0x0D
#define CMD0_REG			0xFE
#define CMD1_REG			0xFF

/* MAX17048 register */
#define HIBRT0_REG	0x0A
#define HIBRT1_REG	0x0B
#define VART0_REG	0x14
#define VART1_REG	0x15
#define VRESET0_REG	0x18
#define VRESET1_REG	0x19

/* MAX17048 using 4.35V battery */
#define RCOMP0_TEMP	20 /* 'C */

/*Low Battery Alert Value*/
#define ALERT_THRESHOLD_VALUE_01 0x1E	/* 1% alert */
#define ALERT_THRESHOLD_VALUE_05 0x1A	/* 5% alert */
#define ALERT_THRESHOLD_VALUE_15 0x10	/* 15% alert */

static struct i2c_driver fg_i2c_driver;
struct fg_i2c_chip {
	struct i2c_client *client;
	struct wake_lock	lowbat_wake_lock;
	bool is_wakelock_active;
	/* current rcomp */
	u16 rcomp;
	/* new rcomp */
	u16 new_rcomp;
};
static struct fg_i2c_chip *fg_max17043;
int is_reset_soc;
static int is_attached;
static int is_alert;	/* ALARM_INT */
static int is_max17048;

static void check_using_max17048(void)
{
#if defined(CONFIG_MACH_ICON) /*max17048, batt=4.35v*/
	is_max17048 = 2;
#elif defined(CONFIG_MACH_PREVAIL2) || defined(CONFIG_MACH_RAY)
	is_max17048 = 1; /*max17048, batt=4.2v*/
#else /*max17043, vital2 refresh*/
	is_max17048 = 0;
#endif
}
/*static int fg_i2c_read(struct i2c_client *client, u8 reg, u8 * data)
{
	int ret;
	u8 buf[1];
	struct i2c_msg msg[2];
	buf[0] = reg;
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = buf;
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret != 2)
		return -EIO;
	*data = buf[0];
	return 0;
}*/

static int fg_i2c_write(struct i2c_client *client, u8 reg, u8 *data)
{
	int ret;
	u8 buf[3];
	struct i2c_msg msg[1];
	buf[0] = reg;
	buf[1] = *data;
	buf[2] = *(data + 1);
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 3;
	msg[0].buf = buf;
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret != 1)
		return -EIO;
	return 0;
}

static int fg_read_word(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_word_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

#if defined(CONFIG_MACH_ICON) /*Fuelgauge 17048 with 4.35V batt*/
static void fg_set_rcomp(struct i2c_client *client, u16 new_rcomp)
{
	int ret;
	/*
	pr_info("%s : new rcomp = 0x%x(%d)\n", __func__,
				new_rcomp, new_rcomp>>8);
	*/

	ret = i2c_smbus_write_word_data(client, RCOMP0_REG, swab16(new_rcomp));
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
}
#endif

unsigned int fg_read_vcell(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2];
	u32 vcell = 0;
	int temp;

	if (!client)
		return -ENOMEM;

	temp = fg_read_word(client, VCELL0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;

	vcell = ((((data[0] << 4) & 0xFF0) | ((data[1] >> 4) & 0xF)) * 125)/100;

	pr_debug("%s: VCELL=%d\n", __func__, vcell);
	return vcell;
}

unsigned int fg_read_soc(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2];
	int temp;

	unsigned int soc;
	int temp_soc;
	/*static int fg_zcount;*/
	u64 psoc64 = 0;
	u64 data64[2] = {0, 0};
	u32 divisor = 1000000000;

	if (!client)
		return -ENOMEM;

	temp = fg_read_word(client, SOC0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;

	if (is_max17048 == 2) { /* 4.35V battery, ICON*/
		data64[0] = data[0];
		data64[1] = data[1];
		pr_debug("soc[0] = %lld, soc[1] = %lld\n",
			data64[0], data64[1]);

		/* [ TempSOC = ((SOC1 * 256) + SOC2) * 0.001953125 ] */
		psoc64 = ((data64[0]*256 + data64[1]) * 1953125);
		psoc64 = div_u64(psoc64, divisor);
		temp_soc = psoc64 & 0xffff;

		/* [ AdjSOC = ((TempSOC - 0.8) * 100) /(99.3-0.8) ] */
		soc = (((temp_soc * 100) - 80) * 10) / 985;
		pr_debug("[battery] soc = %d, temp_soc = %d (0.8)\n",
			soc, temp_soc);
	} else if (is_max17048 == 1) { /* 4.2V battery, prevail2, ray */
		soc = data[0] * 10 + (data[1] >> 7) * 5;
		if (soc <= 5)
			soc = 0;
		else {
		#if defined(CONFIG_MACH_RAY)
			soc = ((soc - 8) * 100) / (959 - 8);
		#else /* Prevail2 */
			soc = ((soc - 5) * 100) / (960 - 5);
		#endif
		}
	} else {
	#ifdef ADJUSTED_SOC /*vital2 refresh*/
		soc = data[0] * 10 + (data[1] >> 7) * 5;
		if (soc <= 10)
			soc = 0;
		else
			soc = ((soc - 10) * 100) / (940 - 10);
	#else
		soc = data[0];
	#endif
	}

	if (soc > 100)
		soc = 100;

	pr_debug("%s: SOC [0]=%d [1]=%d, adj_soc=%d\n",
		__func__, data[0], data[1], soc);

	if (is_reset_soc) {
		pr_info("%s: Reseting SOC\n", __func__);
		/*return -EINVAL;*/
	}

	/*if (soc == 0) {
		fg_zcount++;
		if (fg_zcount >= 3) {
			pr_info("[fg] real 0%%\n");
			soc = 0;
			fg_zcount = 0;
		} else
			soc = chip->soc;
	} else
		fg_zcount = 0;

	chip->soc = soc;*/
	return soc;
}

void fg_register_dump(void)	/*print register information*/
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2];
	int temp;

	temp = fg_read_word(client, VCELL0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;
	pr_debug("%s: VCELL0_REG(0x%x) = 0x%x\n",
		__func__, VCELL0_REG, data[0]);
	pr_debug("%s: VCELL1_REG(0x%x) = 0x%x\n",
		__func__, VCELL1_REG, data[1]);

	temp = fg_read_word(client, SOC0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;
	pr_debug("%s: SOC0_REG(0x%x) = 0x%x\n",
		__func__, SOC0_REG, data[0]);
	pr_debug("%s: SOC1_REG(0x%x) = 0x%x\n",
		__func__, SOC1_REG, data[1]);

	temp = fg_read_word(client, MODE0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;
	pr_debug("%s: MODE0_REG(0x%x) = 0x%x\n",
		__func__, MODE0_REG, data[0]);
	pr_debug("%s: MODE1_REG(0x%x) = 0x%x\n",
		__func__, MODE1_REG, data[1]);

	temp = fg_read_word(client, RCOMP0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;
	pr_debug("%s: RCOMP0_REG(0x%x) = 0x%x\n",
		__func__, RCOMP0_REG, data[0]);
	pr_debug("%s: RCOMP1_REG(0x%x) = 0x%x\n",
		__func__, RCOMP1_REG, data[1]);

	temp = fg_read_word(client, CMD0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;
	pr_debug("%s: CMD0_REG(0x%x) = 0x%x\n",
		__func__, CMD0_REG, data[0]);
	pr_debug("%s: CMD1_REG(0x%x) = 0x%x\n",
		__func__, CMD1_REG, data[1]);

	temp = fg_read_word(client, HIBRT0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;
	pr_debug("%s: HIBRT0_REG(0x%x) = 0x%x\n",
		__func__, HIBRT0_REG, data[0]);
	pr_debug("%s: HIBRT1_REG(0x%x) = 0x%x\n",
		__func__, HIBRT1_REG, data[1]);

	temp = fg_read_word(client, VART0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;
	pr_debug("%s: VART0_REG(0x%x) = 0x%x\n",
		__func__, VART0_REG, data[0]);
	pr_debug("%s: VART1_REG(0x%x) = 0x%x\n",
		__func__, VART1_REG, data[1]);

	temp = fg_read_word(client, VRESET0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;
	pr_debug("%s: VRESET0_REG(0x%x) = 0x%x\n",
		__func__, VRESET0_REG, data[0]);
	pr_debug("%s: VRESET1_REG(0x%x) = 0x%x\n",
		__func__, VRESET1_REG, data[1]);
}


unsigned int fg_read_raw_vcell(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2];
	u32 vcell = 0;
	int temp;
	if (!client)
		return -ENOMEM;

	temp = fg_read_word(client, VCELL0_REG);
	data[0] = temp & 0xff;
	data[1] = (temp >> 8) & 0xff;

	vcell = data[0] << 8 | data[1];
	vcell = (vcell >> 4) * 125 * 1000;
	pr_debug("%s: VCELL=%d\n", __func__, vcell);
	return vcell;
}

/* soc should be 0.01% unit */
unsigned int fg_read_raw_soc(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 data[2] = {0, 0};
	int temp, soc;
	u64 psoc64 = 0;
	u64 temp64;
	u32 divisor = 10000000;

	if (!client)
		return -ENOMEM;

	if (is_max17048 == 1 || is_max17048 == 0) {
		temp = fg_read_word(client, SOC0_REG);
		data[0] = temp & 0xff;
		data[1] = (temp >> 8) & 0xff;
		soc = ((data[0] * 100) + (data[1] * 100 / 256));
	} else if (is_max17048 == 2) {
		/* [ TempSOC = ((SOC1 * 256) + SOC2) * 0.001953125 ] */
		temp = fg_read_word(client, SOC0_REG);
		temp64 = swab16(temp);
		psoc64 = temp64 * 1953125;
		psoc64 = div_u64(psoc64, divisor);
		soc = psoc64 & 0xffff;
	}

	pr_debug("%s: SOC [0]=%d [1]=%d => SOC = %d\n",
		__func__, data[0], data[1], soc);
	if (is_reset_soc) {
		pr_info("%s: Reseting SOC\n", __func__);
		return -EINVAL;
	} else {
		return soc;
	}
}

unsigned int fg_reset_soc(void)
{
	struct i2c_client *client = fg_max17043->client;
	u8 rst_cmd[2];
	s32 ret = 0;

	if (!client)
		return -ENOMEM;

	is_reset_soc = 1;

	/* Quick-start */
	rst_cmd[0] = 0x40;
	rst_cmd[1] = 0x00;

	ret = fg_i2c_write(client, MODE0_REG, rst_cmd);
	if (ret)
		pr_err("[BATT] %s: failed reset SOC(%d)\n", __func__, ret);

	msleep(300);
	is_reset_soc = 0;
	return ret;
}

void fuel_gauge_rcomp(int state)
{
	struct i2c_client *client = fg_max17043->client;
	u8 rst_cmd[2];
	s32 ret = 0;

	if (!client)
		return;

	if (state) {
	#if defined(CONFIG_MACH_ICON)
		rst_cmd[0] = 0x57;
	#elif defined(CONFIG_MACH_PREVAIL2) || defined(CONFIG_MACH_RAY)
		rst_cmd[0] = 0xB0;
	#else
		rst_cmd[0] = 0xE0;/*0xC0;*/
	#endif
		rst_cmd[1] = 0x1f;
	} else {
	#if defined(CONFIG_MACH_ICON)
		rst_cmd[0] = 0x57;
	#elif defined(CONFIG_MACH_PREVAIL2)
		rst_cmd[0] = 0xB0;
	#elif defined(CONFIG_MACH_RAY)
		rst_cmd[0] = 0xC0;
	#else
		rst_cmd[0] = 0xB0;/*0xC0;*/
	#endif
		rst_cmd[1] = 0x1f;
	}

	ret = fg_i2c_write(client, RCOMP0_REG, rst_cmd);
	if (ret)
		pr_err("[BATT] %s: failed fuel_gauge_rcomp(%d)\n",
			__func__, ret);

	/* msleep(500); */
}

#if defined(CONFIG_MACH_ICON) /*Fuelgauge 17048 with 4.35V batt*/
static void max17040_rcomp_update(int temp)
{
	struct i2c_client *client = fg_max17043->client;
	int starting_rcomp = 0;
	int temp_cohot;
	int temp_cocold;
	int new_rcomp = 0;
	int temperature = 0;

	/* ICON */
	temp_cohot = -300;	/* Cohot (-0.3) */
	temp_cocold = -6075;	/* Cocold (-6.075) */
	starting_rcomp = 0x57; /* 0x57 = 87 */

	temperature = temp/10;

	if (temperature > RCOMP0_TEMP)
		new_rcomp = starting_rcomp +
			((temperature-RCOMP0_TEMP) * temp_cohot/1000);
	else if (temperature < RCOMP0_TEMP)
		new_rcomp = starting_rcomp +
			((temperature-RCOMP0_TEMP) * temp_cocold/1000);
	else
		new_rcomp = starting_rcomp;

	if (new_rcomp > 255)
		new_rcomp = 255;
	else if (new_rcomp < 0)
		new_rcomp = 0;

	fg_max17043->new_rcomp =
		((u8)new_rcomp << 8) | (fg_max17043->rcomp&0xFF);

	if (fg_max17043->rcomp != fg_max17043->new_rcomp) {
		pr_debug("%s : temp(%d), rcomp 0x%x -> 0x%x (%d)\n",
			__func__, temperature, fg_max17043->rcomp,
			fg_max17043->new_rcomp,
			fg_max17043->new_rcomp>>8);
		fg_max17043->rcomp = fg_max17043->new_rcomp;

		fg_set_rcomp(client, fg_max17043->new_rcomp);
	}
}
#endif

void fg_wakeunlock(int soc)
{
	/*Value chagnes on fg alter threshold*/
	if ((soc >= 3) && fg_max17043->is_wakelock_active) {
		wake_unlock(&fg_max17043->lowbat_wake_lock);
		fg_max17043->is_wakelock_active = 0;
		pr_info("%s: unlock lowbat_wake_lock\n", __func__);
	}
}

static int fg_set_alert(int is_alert);
/* function body is in samsung_battery.c */

static irqreturn_t fg_interrupt_handler(int irq, void *data) /* ALARM_INT */
{
	struct i2c_client *client = fg_max17043->client;
	u8 rst_cmd[2];
	int temp;
	int soc;

	pr_info("%s\n", __func__);
#define ALERT 0x20
	if (!client)
		return IRQ_HANDLED;
	rst_cmd[0] = 0x00;
	rst_cmd[1] = 0x00;

	temp = fg_read_word(client, RCOMP0_REG);
	rst_cmd[0] = temp & 0xff;
	rst_cmd[1] = (temp >> 8) & 0xff;

	if (temp < 0)
		return IRQ_HANDLED;

	soc = fg_read_soc();

#ifdef DEBUG
	pr_info("\n-----------------------------------------------------\n");
	pr_info(" << %s (vcell:%d, soc:%d, rcomp:0x%x,0x%x) >>\n", __func__,
		fg_read_vcell(), soc, rst_cmd[0], rst_cmd[1]);
	pr_info("-----------------------------------------------------\n\n");
#endif

	wake_lock(&fg_max17043->lowbat_wake_lock);
	fg_max17043->is_wakelock_active = 1;

	if (soc >= 3) {/*Value chagnes on fg alter threshold*/
		wake_unlock(&fg_max17043->lowbat_wake_lock);
		fg_max17043->is_wakelock_active = 0;
		pr_info("%s: unlock lowbat_wake_lock, wrong alert\n", __func__);
	}

	if (fg_set_alert(1)) {
		/* alert flag is set */
		pr_info("[BATT]: %s: low battery alert, "\
			"ready to power down (0x%x, 0x%x)\n",
			__func__, rst_cmd[0], rst_cmd[1]);
	} else {
		/* ignore alert */
		pr_info("[BATT] %s: Ignore low battery alert "\
			"during charging...\n", __func__);
	}

	/* Clear ALRT bit to prevent another low battery interrupt... */
	rst_cmd[1] = rst_cmd[1] & 0xDF;

#ifdef DEBUG
	pr_info("[FG] %s: clear the bit = 0x%x, 0x%x\n",
		__func__, rst_cmd[0], rst_cmd[1]);
#endif
	if (fg_i2c_write(client, RCOMP0_REG, rst_cmd))
		pr_err("[BATT] %s: failed write rcomp\n", __func__);
	return IRQ_HANDLED;
}

static int __devinit fg_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	if (fg_max17043 == NULL) {
		fg_max17043 = kzalloc(sizeof(struct fg_i2c_chip), GFP_KERNEL);
		if (fg_max17043 == NULL)
			return -ENOMEM;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	fg_max17043->client = client;

	i2c_set_clientdata(client, fg_max17043);

	check_using_max17048();

	if (is_max17048 == 2) { /* ICON */
		fg_max17043->rcomp = 0x571D;
		fg_max17043->new_rcomp = 0x571D;
	}

	fuel_gauge_rcomp(0);
	wake_lock_init(&fg_max17043->lowbat_wake_lock,
		WAKE_LOCK_SUSPEND, "fuelgague-lowbat");
	if (request_threaded_irq(client->irq, NULL, fg_interrupt_handler,
	       IRQF_ONESHOT | IRQF_TRIGGER_FALLING, "ALARM_INT", client)) {
		free_irq(client->irq, NULL);
		wake_lock_destroy(&fg_max17043->lowbat_wake_lock);
		pr_err("[BATT] fg_interrupt_handler can't register the handler"\
			"! and passing....\n");
	}
	is_attached = 1;

	pr_debug("[BATT] %s : success!\n", __func__);
	return 0;
}

static int __devexit fg_i2c_remove(struct i2c_client *client)
{
	struct max17043_chip *chip = i2c_get_clientdata(client);
	pr_info("[BATT] %s\n", __func__);

	wake_lock_destroy(&fg_max17043->lowbat_wake_lock);
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	fg_max17043->client = NULL;
	return 0;
}

#define fg_i2c_suspend NULL
#define fg_i2c_resume NULL
static const struct i2c_device_id fg_i2c_id[] = {
	{ "fuelgauge_max17043", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, max17043_id);
static struct i2c_driver fg_i2c_driver = {
	.driver = {
		.name = "fuelgauge_max17043",
	},
	.probe = fg_i2c_probe,
	.remove = __devexit_p(fg_i2c_remove),
	.suspend = fg_i2c_suspend,
	.resume = fg_i2c_resume,
	.id_table = fg_i2c_id,
};
