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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <asm/mach-types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <media/v4l2-subdev.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include <linux/types.h>
#include <mach/vreg.h>

#include "msm_ispif.h"
#include "msm_sensor.h"
#include "msm.h"

#include "sr130pc10.h"
#include "sr130pc10_regs.h"


static int sr130pc10_sensor_read(unsigned short subaddr, unsigned short *data)
{
	/*printk(KERN_DEBUG "<=ASWOOGI=> sr130pc10_sensor_read\n");*/

	int ret;
	unsigned char buf[1] = { 0 };
	struct i2c_msg msg = { sr130pc10_client->addr, 0, 1, buf };

	buf[0] = subaddr;
/*      buf[1] = 0x0; */

	ret = i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO)
		goto error;

	msg.flags = I2C_M_RD;

	ret = i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1 ? 0 : -EIO;
	if (ret == -EIO)
		goto error;

/*      *data = ((buf[0] << 8) | buf[1]); */
	*data = buf[0];

 error:
	/*printk(KERN_DEBUG "[ASWOOGI] on read func
		sr130pc10_client->addr : %x\n",  sr130pc10_client->addr); */
	/*printk(KERN_DEBUG "[ASWOOGI] on read func
		subaddr : %x\n", subaddr); */
	/*printk(KERN_DEBUG "[ASWOOGI] on read func
		data : %x\n", data); */

	return ret;
}

static int sr130pc10_sensor_write(unsigned short subaddr, unsigned short val)
{
	unsigned char buf[2] = { 0 };
	struct i2c_msg msg = { sr130pc10_client->addr, 0, 2, buf };
/*      printk(KERN_DEBUG "[SR130PC10] on write func
	sr130pc10_client->addr : %x\n", subaddr); */
/*      printk(KERN_DEBUG "[SR130PC10] on write func
	sr130pc10_client->adapter->nr : %x\n", val); */
	buf[0] = subaddr;
	buf[1] = val;

	if (i2c_transfer(sr130pc10_client->adapter, &msg, 1) == 1) {
		return 0;
	} else {
		printk(KERN_DEBUG "[sr130pc10] sr130pc10_sensor_write fail\n");
		return -EIO;
	}
}

static int sr130pc10_check_dataline(unsigned short dtp)
{
	int rc = 0;

	if (dtp == 1) {
		pr_info("DATALINE_CHECK ON, dtp:%d\n", dtp);
		sr130pc10_sensor_write(0x03, 0x00);
		sr130pc10_sensor_write(0x50, 0x05);
	} else if (dtp == 0) {
		pr_info("DATALINE_CHECK OFF, dtp:%d\n", dtp);
		sr130pc10_sensor_write(0x03, 0x00);
		sr130pc10_sensor_write(0x50, 0x00);
	} else
		pr_info("[%s] value_1: %d", __func__, dtp);

	return rc;
}

static int
sr130pc10_sensor_write_list(struct sr130pc10_short_t *list,
							int size, char *name)
{
	int ret = 0;
#ifdef CONFIG_LOAD_FILE
	ret = sr130pc10_regs_table_write(name);
#else
	int i;

	pr_info("[%s] %s", __func__, name);
	for (i = 0; i < size; i++) {
		if (list[i].subaddr == 0xff) {
			printk(KERN_DEBUG "now SLEEP!!!!\n");
			msleep(list[i].value * 8);
		} else {
			if (sr130pc10_sensor_write
			    (list[i].subaddr, list[i].value) < 0) {
				printk(KERN_DEBUG
				    "sensor_write_list fail..\n");
				return -EINVAL;
			}
		}
	}
#endif
	return ret;
}

static int sr130pc10_set_movie_mode(int mode)
{
	printk(KERN_DEBUG "[%s:%d][E]\n", __func__, __LINE__);

	if (mode == SENSOR_MOVIE) {
		printk(KERN_DEBUG
			"[%s:%d] Camcorder_Mode_ON\n", __func__, __LINE__);

		sr130pc10_sensor_write_list(sr130pc10_fps_15,
					    sizeof(sr130pc10_fps_15) /
					    sizeof(sr130pc10_fps_15[0]),
					    "sr130pc10_fps_15");
	} else

	if ((mode != SENSOR_CAMERA) && (mode != SENSOR_MOVIE))
		return -EINVAL;

	return 0;
}

static long sr130pc10_set_exposure_value(int mode, int exposure)
{
	long rc = 0;

	printk(KERN_DEBUG "mode : %d, exposure value  : %d\n", mode, exposure);

	switch (exposure) {

	case CAMERA_EV_M4:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_-4\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_m4,
					    sizeof(sr130pc10_ev_m4) /
					    sizeof(sr130pc10_ev_m4[0]),
					    "sr130pc10_ev_m4");
		break;

	case CAMERA_EV_M3:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_-3\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_m3,
					    sizeof(sr130pc10_ev_m3) /
					    sizeof(sr130pc10_ev_m3[0]),
					    "sr130pc10_ev_m3");
		break;

	case CAMERA_EV_M2:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_-2\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_m2,
					    sizeof(sr130pc10_ev_m2) /
					    sizeof(sr130pc10_ev_m2[0]),
					    "sr130pc10_ev_m2");

		break;

	case CAMERA_EV_M1:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_-1\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_m1,
					    sizeof(sr130pc10_ev_m1) /
					    sizeof(sr130pc10_ev_m1[0]),
					    "sr130pc10_ev_m1");

		break;

	case CAMERA_EV_DEFAULT:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_0\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_default,
					    sizeof(sr130pc10_ev_default) /
					    sizeof(sr130pc10_ev_default[0]),
					    "sr130pc10_ev_default");

		break;

	case CAMERA_EV_P1:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_1\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_p1,
					    sizeof(sr130pc10_ev_p1) /
					    sizeof(sr130pc10_ev_p1[0]),
					    "sr130pc10_ev_p1");

		break;

	case CAMERA_EV_P2:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_2\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_p2,
					    sizeof(sr130pc10_ev_p2) /
					    sizeof(sr130pc10_ev_p2[0]),
					    "sr130pc10_ev_p2");
		break;

	case CAMERA_EV_P3:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_3\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_p3,
					    sizeof(sr130pc10_ev_p3) /
					    sizeof(sr130pc10_ev_p3[0]),
					    "sr130pc10_ev_p3");
		break;

	case CAMERA_EV_P4:
		printk(KERN_DEBUG "CAMERA_EXPOSURE_VALUE_4\n");
		sr130pc10_sensor_write_list(sr130pc10_ev_p4,
					    sizeof(sr130pc10_ev_p4) /
					    sizeof(sr130pc10_ev_p4[0]),
					    "sr130pc10_ev_p4");
		break;
	default:
		printk(KERN_DEBUG "unexpected Exposure Value %s/%d\n",
			 __func__, __LINE__);
/*                      return -EINVAL; */
		return 0;
	}

	sr130pc10_ctrl->settings.exposure = exposure;
	return rc;
}

static long sr130pc10_set_sensor_mode(int mode)
{
	int cnt, vsync_value;
	printk(KERN_DEBUG "[CAM-SENSOR] =Sensor Mode\n ");

	switch (mode) {
	case SENSOR_PREVIEW_MODE:
	case SENSOR_VIDEO_MODE:
		printk(KERN_DEBUG "[SR130PC10]-> Preview\n");
		if (sr130pc10_ctrl->op_mode == 0) {
			sr130pc10_sensor_write_list(sr130pc10_reg_init,
					sizeof(sr130pc10_reg_init) /
					sizeof(sr130pc10_reg_init[0]),
					"sr130pc10_reg_init");
			sr130pc10_ctrl->op_mode = 1;
			if (sr130pc10_ctrl->dtp_mode == 1)
				sr130pc10_check_dataline(1);
		}

		factory_test = 0;
		for (cnt = 0; cnt < 200; cnt++) {
			vsync_value = __gpio_get_value(14);
			if (vsync_value) {
				/*printk(KERN_DEBUG " on preview,
			start cnt:%d vsync_value:%d\n", cnt, vsync_value); */
				break;
			} else {
				/*printk(KERN_DEBUG
				    " on preview,  "
					"wait cnt:%d vsync_value:%d\n",
				     cnt, vsync_value);*/
				/*msleep(1);	changed for coding rule*/
				udelay(1000);
			}
		}
		printk(KERN_DEBUG
		  " on preview,  "
		  "wait cnt:%d vsync_value:%d\n",
		  cnt, vsync_value);

		sr130pc10_sensor_write_list(sr130pc10_preview_reg,
			sizeof(sr130pc10_preview_reg) /
			sizeof(sr130pc10_preview_reg[0]),
			"sr130pc10_preview_reg");	/* preview start */
		if (sr130pc10_ctrl->cam_mode == SENSOR_MOVIE)
			sr130pc10_set_movie_mode(SENSOR_MOVIE);
		sr130pc10_set_exposure_value(0, sr130pc10_ctrl->settings.exposure);

		break;

	case SENSOR_SNAPSHOT_MODE:
	case SENSOR_RAW_SNAPSHOT_MODE:
		printk(KERN_DEBUG "[PGH}-> Capture\n");
		if (Flipmode) {
			sr130pc10_sensor_write_list(
				sr130pc10_capture_reg_X_Flip,
				sizeof(sr130pc10_capture_reg_X_Flip) /
				sizeof(sr130pc10_capture_reg_X_Flip[0]),
				"sr130pc10_capture_reg_X_Flip");
			/* preview start */
		} else {
		sr130pc10_sensor_write_list(sr130pc10_capture_reg,
			sizeof(sr130pc10_capture_reg) /
			sizeof(sr130pc10_capture_reg[0]),
			"sr130pc10_capture_reg");	/* preview start */
		}
		   /*SecFeature : for Android CCD preview mirror
			/ snapshot non-mirror
		   if(factory_test == 0) {
			   if(rotation_status == 90 || rotation_status
				== 270) {
				   sr130pc10_sensor_write(0x03, 0x00);
				   sr130pc10_sensor_write(0x11, 0x93);
			   } else {
				   sr130pc10_sensor_write(0x03, 0x00);
				   sr130pc10_sensor_write(0x11, 0x90);
			   }
		   }
		 */
		break;
	default:
		return 0;
	}

	return 0;
}

static int sr130pc10_exif_iso(void)
{
	uint32_t iso_value = 0;

	unsigned short gain;
	int calc_gain;

	sr130pc10_sensor_write(0x03, 0x20);
	sr130pc10_sensor_read(0xb0, &gain);
	calc_gain = (gain/16) * 10000;

	if (calc_gain < 8750)
		iso_value = 50;
	else if (calc_gain < 17500)
		iso_value = 100;
	else if (calc_gain < 46250)
		iso_value = 200;
	else if (calc_gain < 69375)
		iso_value = 400;
	else if (calc_gain < 138750)
		iso_value = 800;
	else
		iso_value = 1600;

	pr_info("[%s]iso_value(%d)\n", __func__, iso_value);
	return (int)iso_value ;
}

static int sr130pc10_exif_shutter_speed(void)
{
	uint32_t shutter_speed = 0;

	unsigned short val1, val2, val3;

	sr130pc10_sensor_write(0x03, 0x20);
	sr130pc10_sensor_read(0x80, &val1);
	sr130pc10_sensor_read(0x81, &val2);
	sr130pc10_sensor_read(0x82, &val3);

	shutter_speed = 24000000 / ((val1<<19) + (val2<<11)+(val3<<3));

	pr_info("[%s]shutter_speed(%d)\n", __func__, shutter_speed);
	return (int)shutter_speed;
}

static int
sr130pc10_set_flipmode(int val)
{
	printk(KERN_DEBUG "Enter [value = %d]\n", val);
	Flipmode = val;
	return 0;
}

static int sr130pc10_get_exif(unsigned short exif_cmd, unsigned short val)
{
	unsigned short retVal = 0;

	switch (exif_cmd) {
	case EXIF_SHUTTERSPEED:
		retVal = sr130pc10_exif_shutter_speed();
		break;

	case EXIF_ISO:
		retVal = sr130pc10_exif_iso();
		break;

	default:
		printk(KERN_DEBUG
			"[%s:%d] invalid(%d)\n",
			__func__, __LINE__, exif_cmd);
		break;
	}

	return retVal;
}
/*
static long sr130pc10_set_effect(int mode, int effect)
{
	long rc = 0;

	pr_info("[%s]effect(%d)\n", __func__, effect);

	switch (effect) {
	case CAMERA_EFFECT_OFF:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_OFF\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_none,
					    sizeof(sr130pc10_effect_none) /
					    sizeof(sr130pc10_effect_none[0]),
					    "sr130pc10_effect_none");
		break;

	case CAMERA_EFFECT_MONO:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_MONO\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_gray,
					    sizeof(sr130pc10_effect_gray) /
					    sizeof(sr130pc10_effect_gray[0]),
					    "sr130pc10_effect_gray");
		break;

	case CAMERA_EFFECT_NEGATIVE:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_NEGATIVE\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_negative,
					    sizeof(sr130pc10_effect_negative) /
					    sizeof(sr130pc10_effect_negative
						   [0]),
					    "sr130pc10_effect_negative");
		break;

	case CAMERA_EFFECT_SEPIA:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_SEPIA\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_sepia,
					    sizeof(sr130pc10_effect_sepia) /
					    sizeof(sr130pc10_effect_sepia[0]),
					    "sr130pc10_effect_sepia");
		break;

	case CAMERA_EFFECT_AQUA:
		printk(KERN_DEBUG "[SR130PC10] CAMERA_EFFECT_AQUA\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_aqua,
					    sizeof(sr130pc10_effect_aqua) /
					    sizeof(sr130pc10_effect_aqua[0]),
					    "sr130pc10_effect_aqua");
		break;

	default:
		printk(KERN_DEBUG "[SR130PC10] default .dsfsdf\n");
		sr130pc10_sensor_write_list(sr130pc10_effect_none,
					    sizeof(sr130pc10_effect_none) /
					    sizeof(sr130pc10_effect_none[0]),
					    "sr130pc10_effect_none");
		return 0;
	}
	return rc;
}
*/
/*
static long sr130pc10_set_whitebalance(int mode, int wb)
{
	long rc = 0;

	printk(KERN_DEBUG "mode : %d,   whitebalance : %d\n", mode, wb);

	switch (wb) {
	case CAMERA_WB_AUTO:
		printk(KERN_DEBUG "CAMERA_WB_AUTO\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_auto,
				    sizeof(sr130pc10_wb_auto) /
				    sizeof(sr130pc10_wb_auto[0]),
				    "sr130pc10_wb_auto");
		break;

	case CAMERA_WB_INCANDESCENT:
		printk(KERN_DEBUG "CAMERA_WB_INCANDESCENT\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_tungsten,
				    sizeof(sr130pc10_wb_tungsten) /
				    sizeof(sr130pc10_wb_tungsten[0]),
				    "sr130pc10_wb_tungsten");
		break;

	case CAMERA_WB_FLUORESCENT:
		printk(KERN_DEBUG "CAMERA_WB_FLUORESCENT\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_fluorescent,
				    sizeof(sr130pc10_wb_fluorescent) /
				    sizeof(sr130pc10_wb_fluorescent[0]),
				    "sr130pc10_wb_fluorescent");
		break;

	case CAMERA_WB_DAYLIGHT:
		printk(KERN_DEBUG "CAMERA_WB_DAYLIGHT\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_sunny,
					    sizeof(sr130pc10_wb_sunny) /
					    sizeof(sr130pc10_wb_sunny[0]),
					    "sr130pc10_wb_sunny");
		break;

	case CAMERA_WB_CLOUDY_DAYLIGHT:
		printk(KERN_DEBUG "CAMERA_WB_CLOUDY_DAYLIGHT\n");
		sr130pc10_sensor_write_list(sr130pc10_wb_cloudy,
					    sizeof(sr130pc10_wb_cloudy) /
					    sizeof(sr130pc10_wb_cloudy[0]),
					    "sr130pc10_wb_cloudy");
		break;

	default:
		printk(KERN_DEBUG "unexpected WB mode %s/%d\n",
			__func__, __LINE__);
		return 0;
	}
	return rc;
}
*/
static struct msm_cam_clk_info cam_clk_info[] = {
	{"cam_clk", MSM_SENSOR_MCLK_24HZ},
};

static int sr130pc10_sensor_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;

	pr_info(" Nothing");

	return rc;
		}

#ifdef CONFIG_LOAD_FILE
int sr130pc10_regs_table_init(void)
{
	struct file *filp;
	char *dp;
	long l;
	loff_t pos;
	int ret;
	mm_segment_t fs = get_fs();

	printk(KERN_DEBUG "%s %d\n", __func__, __LINE__);

	set_fs(get_ds());
	filp = filp_open("/mnt/sdcard/sr130pc10_regs.h", O_RDONLY, 0);

	if (IS_ERR(filp)) {
		printk(KERN_DEBUG "file open error %d\n", PTR_ERR(filp));
		return -EINVAL;
	}
	l = filp->f_path.dentry->d_inode->i_size;
	printk(KERN_DEBUG "l = %ld\n", l);
	dp = kmalloc(l, GFP_KERNEL);
	if (dp == NULL) {
		printk(KERN_DEBUG "Out of Memory\n");
		filp_close(filp, current->files);
	}
	pos = 0;
	memset(dp, 0, l);
	ret = vfs_read(filp, (char __user *)dp, l, &pos);
	if (ret != l) {
		printk(KERN_DEBUG "Failed to read file ret = %d\n", ret);
		kfree(dp);
		filp_close(filp, current->files);
		return -EINVAL;
	}

	filp_close(filp, current->files);

	set_fs(fs);

	sr130pc10_regs_table = dp;

	sr130pc10_regs_table_size = l;

	*((sr130pc10_regs_table + sr130pc10_regs_table_size) - 1) = '\0';

/*      printk(KERN_DEBUG "sr130pc10_regs_table 0x%04x, %ld\n", dp, l); */
	return 0;
}

void sr130pc10_regs_table_exit(void)
{
	printk(KERN_DEBUG "%s %d\n", __func__, __LINE__);
/*
	if (sr130pc10_regs_table) {
		kfree(sr130pc10_regs_table);
		sr130pc10_regs_table = NULL;
	}
*/
	kfree(sr130pc10_regs_table);
	sr130pc10_regs_table = NULL;
}

static int sr130pc10_regs_table_write(char *name)
{
	char *start, *end, *reg;
	unsigned short addr, value;
	char reg_buf[5], data_buf[5];

	*(reg_buf + 4) = '\0';
	*(data_buf + 4) = '\0';

	start = strnstr(sr130pc10_regs_table, name, sr130pc10_regs_table_size);
	end = strnstr(start, "};", sr130pc10_regs_table_size);

	while (1) {
		/* Find Address */
		reg = strnstr(start, "{0x", sr130pc10_regs_table_size);
		if (reg)
			start = (reg + 11);
		if ((reg == NULL) || (reg > end))
			break;
		/* Write Value to Address */
		if (reg != NULL) {
			memcpy(reg_buf, (reg + 1), 4);
			memcpy(data_buf, (reg + 7), 4);
			kstrtol(reg_buf, 16, &addr);
			kstrtol(data_buf, 16, &value);

			if (addr == 0xdd) {
				/* msleep(value); */
				/* printk(KERN_DEBUG "delay 0x%04x,
					value 0x%04x\n", addr, value); */
			} else if (addr == 0xff) {
				msleep(value * 8);
				printk(KERN_DEBUG
					"delay 0x%04x, value 0x%04x\n", addr,
				       value);
			} else
				sr130pc10_sensor_write(addr, value);
		}
	}
	return 0;
}
#endif

static int sr130pc10_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{

	int err = 0;
	int rc = 0;
	int temp = 0;

	struct msm_camera_sensor_info *data = s_ctrl->sensordata;

#ifdef CONFIG_LOAD_FILE
	if (0 > sr130pc10_regs_table_init()) {
		CDBG("%s file open failed!\n", __func__);
		rc = -1;
		goto FAIL_END;
	}
#endif
	sr130pc10_ctrl->op_mode = 0;
	sr130pc10_ctrl->dtp_mode = 0;
	sr130pc10_ctrl->cam_mode = SENSOR_CAMERA;
	sr130pc10_ctrl->settings.exposure = 0;
	pr_info("=== Start ===");

	rc = msm_camera_request_gpio_table(data, 1);
	if (rc < 0)
		pr_info(" request gpio failed");

	gpio_set_value_cansleep(data->sensor_platform_info->vt_sensor_stby, 0);
	temp = __gpio_get_value(data->sensor_platform_info->vt_sensor_stby);
	pr_info("check VT standby : %d", temp);

	gpio_set_value_cansleep(data->sensor_platform_info->vt_sensor_reset, 0);
	temp = __gpio_get_value(data->sensor_platform_info->vt_sensor_reset);
	pr_info("check VT reset : %d", temp);

	gpio_set_value_cansleep(data->sensor_platform_info->sensor_reset, 0);
	temp = __gpio_get_value(data->sensor_platform_info->sensor_reset);
	pr_info("CAM_3M_RST : %d", temp);

	gpio_set_value_cansleep(data->sensor_platform_info->sensor_stby, 0);
	temp = __gpio_get_value(data->sensor_platform_info->sensor_stby);
	pr_info("CAM_3M_ISP_INIT : %d", temp);

	/*Power on the LDOs */
	data->sensor_platform_info->sensor_power_on(1);
	udelay(10);

	/*standy VT */
	gpio_set_value_cansleep(data->sensor_platform_info->vt_sensor_stby, 1);
	temp = __gpio_get_value(data->sensor_platform_info->vt_sensor_stby);
	pr_info("check VT standby : %d", temp);

	udelay(10);

	/*Set Main clock */
	gpio_tlmm_config(GPIO_CFG(data->sensor_platform_info->mclk, 1,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG_ENABLE);

	if (s_ctrl->clk_rate != 0)
		cam_clk_info->clk_rate = s_ctrl->clk_rate;

	rc = msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
		cam_clk_info, &s_ctrl->cam_clk, ARRAY_SIZE(cam_clk_info), 1);
	if (rc < 0) {
		gpio_tlmm_config(GPIO_CFG(data->sensor_platform_info->mclk, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);

		pr_info(" Mclk enable failed");
	}


	if (rc != 0)
		goto FAIL_END;

	mdelay(12);

	/*reset VT */
	gpio_set_value_cansleep(data->sensor_platform_info->vt_sensor_reset, 1);
	temp = __gpio_get_value(data->sensor_platform_info->vt_sensor_reset);
	pr_info("check VT reset : %d", temp);
	udelay(1500);

	err = sr130pc10_sensor_write_list(sr130pc10_i2c_check,
					    sizeof(sr130pc10_i2c_check) /
					    sizeof(sr130pc10_i2c_check[0]),
					    "sr130pc10_i2c_check");
	if (err == -EINVAL) {
		cam_err("[sr130pc20] start1 fail!\n");
		msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
				cam_clk_info, &s_ctrl->cam_clk, \
				ARRAY_SIZE(cam_clk_info), 0);

		gpio_tlmm_config(GPIO_CFG(data->sensor_platform_info->mclk, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);

		udelay(10);
		msm_camera_request_gpio_table(data, 0);

		return -EIO;
	}

FAIL_END:
	if (rc) {
		pr_info("Power up Failed!!");
		msm_camera_request_gpio_table(data, 0);
	} else {
		pr_info("power up x");
	}

	return rc;

}

int sr130pc10_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	int rc = 0;
	int temp = 0;
	struct msm_camera_sensor_info *data = s_ctrl->sensordata;

	pr_info("=== POWER DOWN Start ===");

	sr130pc10_sensor_write(0x03, 0x02);
	sr130pc10_sensor_write(0x55, 0x10);
	mdelay(1);
	udelay(50);

	gpio_set_value_cansleep(data->sensor_platform_info->sensor_reset, 0);
	temp = __gpio_get_value(data->sensor_platform_info->sensor_reset);
	pr_info("[%s]check sensor_reset : %d\n", __func__, temp);

	gpio_set_value_cansleep(data->sensor_platform_info->sensor_stby, 0);
	temp = __gpio_get_value(data->sensor_platform_info->sensor_stby);
	pr_info("[%s]check sensor_stby : %d\n", __func__, temp);

	gpio_set_value_cansleep(data->sensor_platform_info->vt_sensor_reset, 0);
	temp = __gpio_get_value(data->sensor_platform_info->vt_sensor_reset);
	pr_info("[%s]check vt_sensor_reset : %d\n", __func__, temp);

	mdelay(2);

	/*CAM_MCLK0*/
	msm_cam_clk_enable(&s_ctrl->sensor_i2c_client->client->dev,
			cam_clk_info, &s_ctrl->cam_clk, \
			ARRAY_SIZE(cam_clk_info), 0);

	gpio_tlmm_config(GPIO_CFG(data->sensor_platform_info->mclk, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, \
				GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);

	udelay(10);

	gpio_set_value_cansleep(data->sensor_platform_info->vt_sensor_stby, 0);
	temp = __gpio_get_value(data->sensor_platform_info->vt_sensor_stby);
	pr_info("[%s] check VT standby : %d", __func__, temp);

	udelay(10);

	data->sensor_platform_info->sensor_power_off(1);

	msm_camera_request_gpio_table(data, 0);

	sr130pc10_ctrl->op_mode = 0;
	sr130pc10_ctrl->dtp_mode = 0;
	sr130pc10_ctrl->settings.exposure = 0;
#ifdef CONFIG_LOAD_FILE
	sr130pc10_regs_table_exit();
#endif
	return rc;
}

void sensor_native_control_front(void __user *arg)
{
	int err = 0;
	struct ioctl_native_cmd ctrl_info;
	struct msm_camera_v4l2_ioctl_t *ioctl_ptr = arg;

	if (copy_from_user(&ctrl_info,
		(void __user *)ioctl_ptr->ioctl_ptr,
		sizeof(ctrl_info))) {
		pr_info("fail copy_from_user!");
	}
/*	pr_info("mode : %d", ctrl_info.mode);*/

	switch (ctrl_info.mode) {
	case EXT_CAM_EV:
		err = sr130pc10_set_exposure_value(0, ctrl_info.value_1);
		break;

	case EXT_CAM_MOVIE_MODE:
		sr130pc10_ctrl->cam_mode = ctrl_info.value_1;
		/*sr130pc10_set_movie_mode(ctrl_info.value_1);*/
		break;

	case EXT_CAM_EXIF:
		ctrl_info.value_1 = sr130pc10_get_exif(ctrl_info.address,
			ctrl_info.value_2);
		break;
	case EXT_CAM_SET_FLIP:
		sr130pc10_set_flipmode(ctrl_info.value_1);
		break;

	case EXT_CAM_DTP_TEST:
		sr130pc10_check_dataline(ctrl_info.value_1);
		sr130pc10_ctrl->dtp_mode = ctrl_info.value_1;
		break;

	default:
		break;
	}

	if (copy_to_user((void __user *)ioctl_ptr->ioctl_ptr,
		(const void *)&ctrl_info,
			sizeof(ctrl_info)))
		pr_info("fail copy_to_user!");

	return ;
}

int sr130pc10_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
		void __user *argp)
{
	struct sensor_cfg_data cfg_data;
	long rc = 0;

	if (copy_from_user(&cfg_data,
			   (void *)argp, sizeof(struct sensor_cfg_data)))
		return -EFAULT;

	printk(KERN_DEBUG "sr130pc10_ioctl, cfgtype = %d, mode = %d\n",
	       cfg_data.cfgtype, cfg_data.mode);

	switch (cfg_data.cfgtype) {
	case CFG_SET_MODE:
		rc = sr130pc10_set_sensor_mode(cfg_data.mode);
		break;

	default:
		rc = 0;
		break;
	}

	return rc;
}

long sr130pc10_sensor_subdev_ioctl(struct v4l2_subdev *sd,
		unsigned int cmd, void *arg)
{
	void __user *argp = (void __user *)arg;
	struct msm_sensor_ctrl_t *sr130pc10_s_ctrl = get_sctrl(sd);

	pr_info("sr130pc10_sensor_subdev_ioctl : E\n");
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_CFG:
		return sr130pc10_sensor_config(sr130pc10_s_ctrl, argp);

	case VIDIOC_MSM_SENSOR_CSID_INFO:
		{
			struct msm_sensor_csi_info *csi_info =
				(struct msm_sensor_csi_info *)arg;
			pr_info("is_csic = %d", csi_info->is_csic);
			sr130pc10_s_ctrl->is_csic = csi_info->is_csic;
			return 0;
		}
	default:
		pr_info("[%s] default(cmd:%d)\n", __func__, cmd);
		return -ENOIOCTLCMD;
	}
}

static struct v4l2_subdev_info sr130pc10_subdev_info[] = {
	{
	.code   = V4L2_MBUS_FMT_YUYV8_2X8,
	.colorspace = V4L2_COLORSPACE_JPEG,
	.fmt    = 1,
	.order    = 0,
	},
	/* more can be supported, to be added later */
};

static int sr130pc10_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code) {
	pr_info("Index is %d", index);
	if ((unsigned int)index >= ARRAY_SIZE(sr130pc10_subdev_info))
		return -EINVAL;

	*code = sr130pc10_subdev_info[index].code;
	return 0;
}

static struct v4l2_subdev_core_ops sr130pc10_subdev_core_ops = {
	.s_ctrl = msm_sensor_v4l2_s_ctrl,
	.queryctrl = msm_sensor_v4l2_query_ctrl,
	.ioctl = sr130pc10_sensor_subdev_ioctl,
	.s_power = msm_sensor_power,
};

static struct v4l2_subdev_video_ops sr130pc10_subdev_video_ops = {
	.enum_mbus_fmt = sr130pc10_enum_fmt,
};

static struct v4l2_subdev_ops sr130pc10_subdev_ops = {
	.core = &sr130pc10_subdev_core_ops,
	.video  = &sr130pc10_subdev_video_ops,
};

static int sr130pc10_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;

	pr_info("%s_i2c_probe called", client->name);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_info("i2c_check_functionality failed\n");
		rc = -ENOTSUPP;
		goto probe_failure;
	}

	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (s_ctrl->sensor_i2c_client != NULL) {
		s_ctrl->sensor_i2c_client->client = client;
		if (s_ctrl->sensor_i2c_addr != 0)
			s_ctrl->sensor_i2c_client->client->addr =
				s_ctrl->sensor_i2c_addr;
	} else {
		pr_info("s_ctrl->sensor_i2c_client is NULL\n");
		rc = -EFAULT;
		return rc;
	}

	s_ctrl->sensordata = client->dev.platform_data;
	if (s_ctrl->sensordata == NULL) {
		pr_err("%s: NULL sensor data\n", __func__);
		return -EFAULT;
	}

	sr130pc10_client = client;
	sr130pc10_dev = s_ctrl->sensor_i2c_client->client->dev;

	sr130pc10_ctrl = kzalloc(sizeof(struct sr130pc10_ctrl), GFP_KERNEL);
	if (!sr130pc10_ctrl) {
		pr_info("sr130pc10_ctrl alloc failed!\n");
		return -ENOMEM;
	}

	sr130pc10_exif = kzalloc(sizeof(struct sr130pc10_exif_data),
		GFP_KERNEL);
	if (!sr130pc10_exif) {
		pr_info("Cannot allocate memory fo EXIF structure!");
		kfree(sr130pc10_exif);
		rc = -ENOMEM;
	}

	snprintf(s_ctrl->sensor_v4l2_subdev.name,
		sizeof(s_ctrl->sensor_v4l2_subdev.name), "%s", id->name);

	v4l2_i2c_subdev_init(&s_ctrl->sensor_v4l2_subdev, client,
		&sr130pc10_subdev_ops);

	sr130pc10_ctrl->sensor_dev = &s_ctrl->sensor_v4l2_subdev;
	sr130pc10_ctrl->sensordata = client->dev.platform_data;

	msm_sensor_register(&s_ctrl->sensor_v4l2_subdev);

	pr_info("sr130pc10_probe succeeded!");
	return 0;

probe_failure:
	pr_info("sr130pc10_probe failed!");
	return rc;
}

static const struct i2c_device_id sr130pc10_i2c_id[] = {
	{"sr130pc10", (kernel_ulong_t)&sr130pc10_s_ctrl},
	{},
};

static struct msm_camera_i2c_client sr130pc10_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static struct i2c_driver sr130pc10_i2c_driver = {
	.id_table = sr130pc10_i2c_id,
	.probe  = sr130pc10_i2c_probe,
	.driver = {
		.name = "sr130pc10",
	},
};

static int __init sr130pc10_init(void)
{
	return i2c_add_driver(&sr130pc10_i2c_driver);
}

static struct msm_sensor_fn_t sr130pc10_func_tbl = {
	.sensor_config = sr130pc10_sensor_config,
	.sensor_power_up = sr130pc10_sensor_power_up,
	.sensor_power_down = sr130pc10_sensor_power_down,
	.sensor_match_id = sr130pc10_sensor_match_id,
};


static struct msm_sensor_reg_t sr130pc10_regs = {
	.default_data_type = MSM_CAMERA_I2C_BYTE_DATA,
};

static struct msm_sensor_ctrl_t sr130pc10_s_ctrl = {
	.msm_sensor_reg = &sr130pc10_regs,
	.sensor_i2c_client = &sr130pc10_sensor_i2c_client,
	.sensor_i2c_addr = 0x20,
	.cam_mode = MSM_SENSOR_MODE_INVALID,
	.msm_sensor_mutex = &sr130pc10_mut,
	.sensor_i2c_driver = &sr130pc10_i2c_driver,
	.sensor_v4l2_subdev_info = sr130pc10_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(sr130pc10_subdev_info),
	.sensor_v4l2_subdev_ops = &sr130pc10_subdev_ops,
	.func_tbl = &sr130pc10_func_tbl,
	.clk_rate = MSM_SENSOR_MCLK_24HZ,
};

module_init(sr130pc10_init);
