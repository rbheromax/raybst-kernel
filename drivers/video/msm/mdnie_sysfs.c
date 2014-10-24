/*
 * =====================================================================
 *
 *       Filename:  mdnie_sysfs.c
 *
 *    Description:  SYSFS Node control driver
 *
 *        Version:  1.0
 *        Created:  2012 05/22 20:18:45
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/lcd.h>
#include <linux/notifier.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include "mdnie_sysfs.h"

static struct class *mdnie_class;
struct device *mdnie_dev;

int (*apply_negative_value)(enum eNegative_Mode negative_mode);
int (*apply_cabc_value)(enum eCabc_Mode negative_mode);

struct mdnie_state_type mdnie_state = {
	.negative = NEGATIVE_OFF_MODE,
	.cabc_mode = CABC_OFF_MODE,

};

/* ##########################################################
 * #
 * # MDNIE CABC Sysfs node
 * #
 * ##########################################################*/
static ssize_t cabc_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t maxsize = 30;

	return snprintf(buf, maxsize, "Current cabc Value : %s\n",
		(mdnie_state.cabc_mode == 0) ? "Disabled" : "Enabled");
}

static ssize_t cabc_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int value;

	sscanf(buf, "%d", &value);
	pr_debug("[MDNIE:INFO] set cabc : %d\n", value);

	if (value < CABC_OFF_MODE || value >= MAX_CABC_MODE) {
		pr_debug("[MDNIE:ERROR] : wrong cabc mode value : %d\n",
			value);
		return size;
	}

	if (value == mdnie_state.cabc_mode) {
		pr_debug("cabc mode already changed\n");
		return size;
	}

	ret = apply_cabc_value(value);
	if (ret != 0)
		pr_debug("[MDNIE:ERROR] ERROR : set cabc value faild\n");
	mdnie_state.cabc_mode = value;

	return size;
}

static DEVICE_ATTR(cabc, 0664, cabc_show, cabc_store);

int apply_cabc_value_default(enum eCabc_Mode cabc_mode)
{
	return 0;
}

/* ##########################################################
 * #
 * # MDNIE NEGATIVE Sysfs node
 * #
 * ##########################################################*/
static ssize_t negative_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	size_t maxsize = 30;

	return snprintf(buf, maxsize, "Current negative Value : %s\n",
		(mdnie_state.negative == 0) ? "Disabled" : "Enabled");
}

static ssize_t negative_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;
	int value;

	sscanf(buf, "%d", &value);
	pr_debug("[MDNIE:INFO] set negative : %d\n", value);

	if (value < NEGATIVE_OFF_MODE || value >= MAX_NEGATIVE_MODE) {
		pr_debug("[MDNIE:ERROR] : wrong negative mode value : %d\n",
			value);
		return size;
	}

	if (value == mdnie_state.negative) {
		pr_debug("negative mode already changed\n");
		return size;
	}

	ret = apply_negative_value(value);
	if (ret != 0)
		pr_debug("[MDNIE:ERROR] ERROR : set negative value faild\n");
	mdnie_state.negative = value;

	return size;
}

static DEVICE_ATTR(negative, 0664, negative_show, negative_store);

int apply_negative_value_default(enum eNegative_Mode negative_mode)
{
	return 0;
}

int mdnie_sysfs_init(struct mdnie_ops *mop)
{

	int ret = 0;

	/*  1. CLASS Create
	 *  2. Device Create
	 *  3. node create
	 *   - negative colors on/off node */

	/* 1. CLASS Create */
	mdnie_class = class_create(THIS_MODULE, "mdnie");
	if (IS_ERR(mdnie_class)) {
		pr_debug("Failed to create class(mdnie_class)!!\n");
		ret = -1;
	}

	/* 2. Device Create */
	mdnie_dev = device_create(mdnie_class, NULL, 0, NULL, "mdnie");
	if (IS_ERR(mdnie_dev)) {
		pr_debug("Failed to create device(mdnie_dev)!!");
		ret = -1;
	}

	/* 3. node function setting */
	if (mop->apply_negative_value != NULL)
		apply_negative_value = mop->apply_negative_value;
	else
		apply_negative_value = apply_negative_value_default;

	if (mop->apply_cabc_value != NULL)
		apply_cabc_value = mop->apply_cabc_value;
	else
		apply_cabc_value = apply_cabc_value_default;

	/* 4. node create */
	if (device_create_file(mdnie_dev, &dev_attr_negative) < 0) {
		pr_debug("[MDNIE:ERROR] device_create_file(%s)\n",\
			dev_attr_negative.attr.name);
		ret = -1;
	}

	if (device_create_file(mdnie_dev, &dev_attr_cabc) < 0) {
		pr_debug("[MDNIE:ERROR] device_create_file(%s)\n",\
			dev_attr_cabc.attr.name);
		ret = -1;
	}
	return 0;
}
