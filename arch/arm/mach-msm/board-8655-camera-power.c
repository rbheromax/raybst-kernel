
/* Copyright (c) 2011-2012, Code Aurora Forum. All rights reserved.
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

#include <asm/mach-types.h>
#include <linux/gpio.h>
#include <mach/board.h>
#include <mach/msm_bus_board.h>
#include <mach/gpiomux.h>
#ifndef CONFIG_MACH_RAY
#include <mach/msm8930-gpio.h>
#include "devices.h"
#include "board-8930.h"
#endif
#include <mach/vreg.h>
#include <mach/ray-cam-gpio.h>

#undef CAM_DEBUG
#if defined(DEBUG_LEVEL_HIGH)
#define CAM_DEBUG(fmt, arg...)	\
	do {					\
		printk(KERN_DEBUG "[%s:%d] " fmt,	\
			__func__, __LINE__, ##arg);	\
	}						\
	while (0)
#else
#define CAM_DEBUG(fmt, arg...)
#endif

#undef cam_err
#define cam_err(fmt, arg...)			\
	do {					\
		printk(KERN_ERR "[%s:%d] " fmt,		\
			__func__, __LINE__, ##arg);	\
	}						\
	while (0)

#if defined(CONFIG_ISX012) || defined(CONFIG_SR130PC10)
void msm_camera_gpio_install(void)
{
	CAM_DEBUG("Camera GPIO install!!\n");

#ifndef CONFIG_MACH_RAY		/* marco.zhang 20120912 */
	/*Flash En : PM GPIO 28*/
	gpio_tlmm_config(GPIO_CFG(GPIO_MSM_FLASH_CNTL_EN, 0,
	     GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);
	/*Flash Set : PM GPIO 27*/
	gpio_tlmm_config(GPIO_CFG(GPIO_MSM_FLASH_NOW, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA),
		GPIO_CFG_ENABLE);
	/*CAM_MCLK1  : GPIO 4*/
	gpio_tlmm_config(GPIO_CFG(GPIO_SUB_CAM_MCLK, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
#endif
	/*CAM_MCLK0  : GPIO 15*/
	gpio_tlmm_config(GPIO_CFG(GPIO_MAIN_CAM_MCLK, 1, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	/*GPIO_CAM_CORE_EN  : GPIO 132*/
	gpio_tlmm_config(GPIO_CFG(GPIO_CAM_CORE_EN, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	/*Front cam stby : GPIO 2*/
	gpio_tlmm_config(GPIO_CFG(GPIO_VT_STBY, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	/*GPIO_CAM_A_EN : GPIO 3*/
	gpio_tlmm_config(GPIO_CFG(GPIO_CAM_A_EN, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	/*GPIO_CAM_IO_EN : GPIO 143*/
	gpio_tlmm_config(GPIO_CFG(GPIO_CAM_IO_EN, 0,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG_ENABLE);
	/*Main cam stby : GPIO 131*/
	gpio_tlmm_config(GPIO_CFG(GPIO_MAIN_STBY, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	/*Front cam reset : GPIO 175*/
	gpio_tlmm_config(GPIO_CFG(GPIO_CAM2_RST_N, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	/*Main cam reset  : GPIO 130*/
	gpio_tlmm_config(GPIO_CFG(GPIO_CAM1_RST_N, 0, GPIO_CFG_OUTPUT,
		GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), GPIO_CFG_ENABLE);

	CAM_DEBUG("Finish!!\n");
}
/*EXPORT_SYMBOL(msm_camera_gpio_install);*/

/* CAM power
	CAM_SENSOR_A_2.8		:  GPIO_CAM_A_EN(GPIO 38)
	CAM_SENSOR_IO_1.8		: GPIO_CAM_IO_EN(GPIO 41)
	CAM_AF_2.8				: VREG_L8		: l8
	CAM_ISP_CORE_1.2		: GPIO_CAM_CORE_EN(GPIO 6)

	CAM_DVDD_1.5		: VREG_L12		: l12
*/

/*static struct regulator *l8, *l12;*/
#define MAX_CAM_POWER_PIN 5
int error_gpio_buf[MAX_CAM_POWER_PIN];
int error_gpio_cnt;

char cam_power_pin[MAX_CAM_POWER_PIN][32] = {
	"GPIO_CAM_CORE_EN",
	"GPIO_CAM_IO_EN",
	"GPIO_CAM_A_EN",
	"8038_l12",
	"8038_l8",
};

void search_error_pin(char *name)
{
	int i = 0;

	cam_err("search_error_pin(%s)", name);
	for (i = 0; i < MAX_CAM_POWER_PIN; i++) {
		if (!strncmp(cam_power_pin[i], name, strnlen(name, 50))) {
			error_gpio_buf[error_gpio_cnt++] = i;
			break;
		}
	}
	if (i == MAX_CAM_POWER_PIN)
		cam_err("Can't find error pin : %s", name);
}
void cam_ldo_power_on(int mode)
{
	int ret = 0;
	int power_on_cnt = 0;
	int i = 0;

	struct vreg *vreg_L8;
	struct vreg *vreg_L16;

/*	cam_err("%s CAMERA POWER ON!!\n",
	       mode ? "FRONT" : "REAR");*/

	error_gpio_cnt = 0;
	for (i = 0; i < MAX_CAM_POWER_PIN; i++)
		error_gpio_buf[i] = 0;

	vreg_L8 = vreg_get(NULL, "gp7");
	vreg_set_level(vreg_L8, 1800);
	udelay(1000);

	if (mode == 0) {
		/* CAM_AF_3.0V Set High*/
		vreg_L16 = vreg_get(NULL, "gp10");
		vreg_set_level(vreg_L16, 3000);
		ret = vreg_enable(vreg_L16);
		if (ret)
			cam_err("error setting voltage CAM_AF_3.0V\n");

		udelay(1000);
	}

	/*5M Core 1.2V - CAM_ISP_CORE_1P2*/
	gpio_set_value(GPIO_CAM_CORE_EN, 1);
	ret = gpio_get_value(GPIO_CAM_CORE_EN);
	if (ret) {
		CAM_DEBUG("check CAM_CORE_EN : %d\n", ret);
		power_on_cnt++;
	} else
		search_error_pin("GPIO_CAM_CORE_EN");

	udelay(200);

	/*Sensor IO 1.8V -CAM_SENSOR_IO_1P8  */
	gpio_set_value(GPIO_CAM_IO_EN, 1);
	ret = gpio_get_value(GPIO_CAM_IO_EN);
	if (ret) {
		CAM_DEBUG("check GPIO_CAM_IO_EN : %d\n", ret);
		power_on_cnt++;
	} else
		search_error_pin("GPIO_CAM_IO_EN");

	/*Sensor AVDD 2.8V - CAM_SENSOR_A2P8 */
	gpio_set_value(GPIO_CAM_A_EN, 1);
	ret = gpio_get_value(GPIO_CAM_A_EN);
	if (ret) {
		CAM_DEBUG("check GPIO_CAM_A_EN : %d\n", ret);
		power_on_cnt++;
	} else
		search_error_pin("GPIO_CAM_A_EN");

	/* VCAM_1.8VDV Set High*/
	ret = vreg_enable(vreg_L8);
	if (ret)
		cam_err("error setting voltage VCAM_1.8VDV\n");

/*VT core 1.2V - CAM_DVDD_1P5V*/
/*
	l12 = regulator_get(NULL, "8038_l12");
	ret = regulator_set_voltage(l12, 1500000, 1500000);
	if (ret)
		cam_err("error setting voltage\n");
	ret = regulator_enable(l12);
	if (ret) {
		cam_err("error enabling regulator\n");
		search_error_pin("8038_l12");
	} else
		power_on_cnt++;
*/
	mdelay(1);

/*Sensor AF 2.8V -CAM_AF_2P8  */
/*
	if (!mode) {
		l8 = regulator_get(NULL, "8038_l8");
		ret = regulator_set_voltage(l8, 3000000, 3000000);
		if (ret)
			cam_err("error setting voltage\n");
		ret = regulator_enable(l8);
		if (ret) {
			cam_err("error enabling regulator\n");
			search_error_pin("8038_l8");
		} else
			power_on_cnt++;
	} else
	    power_on_cnt++;

	if (power_on_cnt == MAX_CAM_POWER_PIN)
		CAM_DEBUG("Cam power success!!\n");
	else {
		cam_err("Cam power failed : [%d / %d]\n",
			error_gpio_cnt, MAX_CAM_POWER_PIN);
		for (i = 0; i < error_gpio_cnt; i++)
			cam_err("Failed power pin[%d] : %s\n",
				error_gpio_buf[i],
				cam_power_pin[error_gpio_buf[i]]);
	}
*/
/*	cam_err("%s End!!\n",
	       mode ? "FRONT" : "REAR");*/
}
/*EXPORT_SYMBOL(cam_ldo_power_on);*/

void cam_ldo_power_off(int mode)
{
	int ret = 0;
	struct vreg *vreg_L8;

/*	cam_err("%s CAMERA POWER OFF!!\n",
	       mode ? "FRONT" : "REAR");*/

/*Sensor AF 2.8V -CAM_AF_2P8  */
/*
	if (!mode) {
		if (l8) {
			ret = regulator_disable(l8);
			if (ret)
				cam_err("error disabling regulator\n");
		}
	}

VT core 1.2 - CAM_DVDD_1P5V*/
/*
	if (l12) {
		ret = regulator_disable(l12);
		if (ret)
			cam_err("error disabling regulator\n");
	}
	 VCAM_1.8VDV Set Low*/
	vreg_L8 = vreg_get(NULL, "gp7");
	ret = vreg_disable(vreg_L8);
	if (ret)
		cam_err("error disabling regulator VCAM_1.8VDV\n");

	mdelay(1);

	/*Sensor AVDD 2.8V - CAM_SENSOR_A2P8 */
	gpio_set_value(GPIO_CAM_A_EN, 0);
	mdelay(1);

	/*Sensor IO 1.8V -CAM_SENSOR_IO_1P8  */
	gpio_set_value(GPIO_CAM_IO_EN, 0);
	udelay(15);

	/*5M Core 1.2V - CAM_ISP_CORE_1P2*/
	gpio_set_value(GPIO_CAM_CORE_EN, 0);

/*	CAM_DEBUG("%s End!!\n",
	       mode ? "FRONT" : "REAR");*/
}
/*EXPORT_SYMBOL(cam_ldo_power_off);*/

#else /*#if defined(CONFIG_MACH_BOOST)*/
void msm_camera_gpio_install(void)
{
	cam_err("Camera GPIO install!!\n");
	cam_err("Invalid function call.\n");
}
void cam_ldo_power_on(int mode)
{
	cam_err("%s CAMERA POWER ON!!\n",
	       mode ? "FRONT" : "REAR");

	cam_err("Invalid function call.\n");
}
void cam_ldo_power_off(int mode)
{
	cam_err("%s CAMERA POWER OFF!!\n",
	       mode ? "FRONT" : "REAR");
	cam_err("Invalid function call.\n");
}

#endif
