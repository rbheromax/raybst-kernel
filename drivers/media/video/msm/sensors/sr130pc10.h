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

#ifndef __SR130PC10_H__
#define __SR130PC10_H__

#include "msm_sensor.h"
#include <linux/types.h>
#include <mach/board.h>

#undef CONFIG_LOAD_FILE
/*#define CONFIG_LOAD_FILE*/

#define VCAM_I2C_SCL 177
#define VCAM_I2C_SDA 174

#undef DEBUG_LEVEL_HIGH
#undef DEBUG_LEVEL_MID
#define DEBUG_LEVEL_HIGH
/*#define DEBUG_LEVEL_MID */

#if defined(DEBUG_LEVEL_HIGH)
#define CAM_DEBUG(fmt, arg...)	\
	do {					\
		printk(KERN_DEBUG "[%s:%d] " fmt,	\
			__func__, __LINE__, ##arg);	\
	}						\
	while (0)

#define cam_info(fmt, arg...)			\
	do {					\
		printk(KERN_INFO "[%s:%d] " fmt,	\
			__func__, __LINE__, ##arg);	\
	}						\
	while (0)
#elif defined(DEBUG_LEVEL_MID)
#define CAM_DEBUG(fmt, arg...)
#define cam_info(fmt, arg...)			\
	do {					\
		printk(KERN_INFO "[%s:%d] " fmt,	\
			__func__, __LINE__, ##arg);	\
	}						\
	while (0)
#else
#define CAM_DEBUG(fmt, arg...)
#define cam_info(fmt, arg...)
#endif

#undef DEBUG_CAM_I2C
#define DEBUG_CAM_I2C

#if defined(DEBUG_CAM_I2C)
#define cam_i2c_dbg(fmt, arg...)		\
	do {					\
		printk(KERN_DEBUG "[%s:%d] " fmt,	\
			__func__, __LINE__, ##arg);	\
	}						\
	while (0)
#else
#define cam_i2c_dbg(fmt, arg...)
#endif


#define cam_err(fmt, arg...)	\
	do {					\
		printk(KERN_ERR "[%s:%d] " fmt,		\
			__func__, __LINE__, ##arg);	\
	}						\
	while (0)


DEFINE_MUTEX(sr130pc10_mut);

struct sr130pc10_work {
	struct work_struct work;
};

struct sr130pc10_exif_data {
	unsigned short shutterspeed;
};

static struct sr130pc10_exif_data *sr130pc10_exif;
static struct msm_sensor_ctrl_t sr130pc10_s_ctrl;
static struct device sr130pc10_dev;

struct sr130pc10_userset {
	unsigned int	focus_mode;
	unsigned int	focus_status;
	unsigned int	continuous_af;

	unsigned int	metering;
	unsigned int	exposure;
	unsigned int	wb;
	unsigned int	iso;
	int		contrast;
	int		saturation;
	int		sharpness;
	int		brightness;
	int		ev;
	int		scene;
	unsigned int	zoom;
	unsigned int	effect;		/* Color FX (AKA Color tone) */
	unsigned int	scenemode;
	unsigned int	detectmode;
	unsigned int	antishake;
	unsigned int	fps;
	unsigned int	flash_mode;
	unsigned int	flash_state;
	unsigned int	stabilize;	/* IS */
	unsigned int	strobe;
	unsigned int	jpeg_quality;
	/*unsigned int preview_size;*/
	unsigned int	preview_size_idx;
	unsigned int	capture_size;
	unsigned int	thumbnail_size;
};
struct sr130pc10_ctrl {
	const struct msm_camera_sensor_info *sensordata;
	struct sr130pc10_userset settings;
	struct msm_camera_i2c_client *sensor_i2c_client;
	struct v4l2_subdev *sensor_dev;
	struct v4l2_subdev sensor_v4l2_subdev;
	struct v4l2_subdev_info *sensor_v4l2_subdev_info;
	uint8_t sensor_v4l2_subdev_info_size;
	struct v4l2_subdev_ops *sensor_v4l2_subdev_ops;

	int op_mode;
	int dtp_mode;
	int cam_mode;
	int vtcall_mode;
	int started;
	int dtpTest;
	int isCapture;
};

/*static unsigned int config_csi2;*/
static struct sr130pc10_ctrl *sr130pc10_ctrl;

struct sr130pc10_format {
	enum v4l2_mbus_pixelcode code;
	enum v4l2_colorspace colorspace;
	u16 fmt;
	u16 order;
};

extern struct i2c_client *sr130pc10_client;

struct sr130pc10_reg {
unsigned int addr;
unsigned int val;
};

struct sr130pc10_regset_type {
unsigned int *regset;
int len;
};

enum msm_sensor_mode {
	SENSOR_CAMERA,
	SENSOR_MOVIE,
};

/*
 * Macro
 */
#define REGSET_LENGTH(x)(sizeof(x)/sizeof(sr130pc10_reg))

/*
 * User defined commands
 */
/* S/W defined features for tune */
#define REG_DELAY	0xFF00/* in ms */
#define REG_CMD		0xFFFF/* Followed by command */

extern struct class *camera_class;

/* Following order should not be changed */
enum image_size_sr130pc10 {
/* This SoC supports upto SXGA (1280*1024) */
/*
QQVGA,//160*120
QCIF,// 176*144
QVGA,// 320*240
CIF,// 352*288
VGA,// 640*480
*/
SVGA,/* 800*600 */
/*
HD720P,//1280*720
SXGA,// 1280*1024
UXGA,// 1600*1200
*/
};

/*
 * Following values describe controls of camera
 * in user aspect and must be match with index of sr130pc10_regset[]
 * These values indicates each controls and should be used
 * to control each control
 */
enum sr130pc10_control {
sr130pc10_INIT,
sr130pc10_EV,
sr130pc10_AWB,
sr130pc10_MWB,
sr130pc10_EFFECT,
sr130pc10_CONTRAST,
sr130pc10_SATURATION,
sr130pc10_SHARPNESS,
};

#define SR130PC10_REGSET(x) {\
.regset = x,\
.len = sizeof(x)/sizeof(sr130pc10_reg),}

#ifdef CONFIG_LOAD_FILE
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
static char *sr130pc10_regs_table;
static int sr130pc10_regs_table_size;
static int sr130pc10_regs_table_write(char *name);
#endif

/*static int16_t sr130pc10_effect = CAMERA_EFFECT_OFF; */
/*static int rotation_status;*/
static int factory_test;
static int Flipmode;

#endif/* __SR130PC10_H__ */

