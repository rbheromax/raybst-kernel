/* drivers/input/touchscreen/tma140_lucas.c
 *
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/regulator/consumer.h>
#include <linux/irq.h>


#include <linux/firmware.h>
#include <linux/uaccess.h>
#include <linux/rtc.h>
/* firmware - update */

#include "tma140.h"

/* kwon added 2013-02-22 */
#define TOUCH_BOOSTER
#if defined(TOUCH_BOOSTER)
#include <linux/mfd/dbx500-prcmu.h>
#endif

#define MAX_X	480
#define MAX_Y	800
#define TSP_SDA 87
#define TSP_SCL 88
#define TSP_INT 89

#define TOUCH_EN 79

#define MAX_KEYS	2
#define MAX_USING_FINGER_NUM 2

/* kwon added 2013-02-22 */
#define MAX_TOUCH_ID 2

#define SEC_TSP_FACTORY_TEST

#define GLOBAL_IDAC_NUM	18
#define NODE_NUM	144	/* 16X9 */
#define NODE_X_NUM 9
#define NODE_Y_NUM 16

#define LOCAL_IDAC_START	6
#define GLOBAL_IDAC_START	7

/*force release previous press event,
when the next touch is pressed very quickly*/
#define __SEND_VIRTUAL_RELEASED__

#define TSP_EDS_RECOVERY

unsigned int raw_count[NODE_NUM];
unsigned int difference[NODE_NUM];
unsigned int local_idac[NODE_NUM];
unsigned int global_idac[GLOBAL_IDAC_NUM];


#define __TOUCHKEY__

#ifdef __TOUCHKEY__
static const int touchkey_keycodes[] = {
			KEY_MENU,
			KEY_BACK,
};
#endif

#ifdef SEC_TSP_FACTORY_TEST
#define TSP_BUF_SIZE 1024

#define TSP_CMD_STR_LEN 32
#define TSP_CMD_RESULT_STR_LEN 512
#define TSP_CMD_PARAM_NUM 8

#define TSP_SPEC_RAW_COUNT_MIN 50
#define TSP_SPEC_RAW_COUNT_MAX 150

#define TSP_SPEC_DIFFERENCE_MIN 70
#define TSP_SPEC_DIFFERENCE_MAX 130

#define TSP_SPEC_LOCAL_IDAC_MIN 1
#define TSP_SPEC_LOCAL_IDAC_MAX 30

#define TSP_SPEC_GLOBAL_IDAC_MIN 145
#define TSP_SPEC_GLOBAL_IDAC_MAX 295

#define MAX_CMD_NAME_STR_LEN 256

#endif /* SEC_TSP_FACTORY_TEST */


#define TOUCH_ON 1
#define TOUCH_OFF 0

#define TRUE    1
#define FALSE    0

#define I2C_RETRY_CNT	2

/*#define __TOUCH_DEBUG__*/
#define __TOUCH_KMSG__

#ifdef __TOUCH_KMSG__
#define	tma_kmsg(fmt, args...)	printk(KERN_INFO "[TSP][%-18s:%5d]"\
	fmt, __func__, __LINE__, ## args)
#else
#define	tma_kmsg(fmt, args...)	do {} while (0)
#endif


/*#define __TSP_FORCE_UPDATE__*/
#define USE_THREADED_IRQ	1

/*#define __TOUCH_KEYLED__*/

static struct regulator *touch_regulator;
#if defined(__TOUCH_KEYLED__)
static struct regulator *touchkeyled_regulator;
#endif


#if defined(TSP_EDS_RECOVERY)
static struct workqueue_struct *check_ic_wq;
#endif

#ifdef __TOUCHKEY__
static int touchkey_status[MAX_KEYS];

#define TK_STATUS_PRESS			1
#define TK_STATUS_RELEASE		0
#endif

int tsp_irq;
int st_old;

struct report_finger_info_t {
	int8_t id;	/*!< (id>>8) + size */
	int8_t status;/*IC*/
	int8_t z;	/*!< dn>0, up=0, none=-1 */
	int16_t x;			/*!< X */
	int16_t y;			/*!< Y */
};

/* kwon added 2013-02-22 */
#if defined(__SEND_VIRTUAL_RELEASED__)
static struct report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM+2] = {{0},};
#else
static struct report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM] = {0,};
#endif

enum {
	BUILT_IN = 0,
	UMS,
	REQ_FW,
};

struct touch_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;				/*IC*/
#if defined(TSP_EDS_RECOVERY)
	struct work_struct  esd_recovery_func;		/*IC*/
#endif
	unsigned char fw_ic_ver;

	bool				enabled;

#if defined(SEC_TSP_FACTORY_TEST)
	struct list_head			cmd_list_head;
	unsigned char cmd_state;
	char			cmd[TSP_CMD_STR_LEN];
	int			cmd_param[TSP_CMD_PARAM_NUM];
	char			cmd_result[TSP_CMD_RESULT_STR_LEN];
	struct mutex			cmd_lock;
	bool			cmd_is_running;

	bool ft_flag;
#endif				/* SEC_TSP_FACTORY_TEST */

	struct early_suspend early_suspend;
};


struct touch_data *ts_global;

/* firmware - update */
static int firmware_ret_val = -1;


unsigned char esd_conter;
EXPORT_SYMBOL(esd_conter);

unsigned char now_tspfw_update;
EXPORT_SYMBOL(now_tspfw_update);

unsigned char tsp_special_update;
EXPORT_SYMBOL(tsp_special_update);

static char IsfwUpdate[20] = {0};

/* touch information*/
unsigned char touch_vendor_id;
EXPORT_SYMBOL(touch_vendor_id);

unsigned char touch_hw_ver;
EXPORT_SYMBOL(touch_hw_ver);

unsigned char touch_sw_ver;
EXPORT_SYMBOL(touch_sw_ver);

/*
#define TSP_HW_REV03			0x03
#define TSP_HW_REV04			0x04

#define TSP_SW_VER_FOR_HW03		0x07
#define TSP_SW_VER_FOR_HW04		0x01
*/
#define TSP_VENDER_ID	0xF0

#define TSP_HW_VER1		0x01
#define TSP_SW_VER1_L		0x02
#define TSP_SW_VER1_H		0x00

#define TSP_HW_VER3		0x03
#define TSP_SW_VER3_L		0x0e
#define TSP_SW_VER3_H		0x01


int tsp_irq_num;
EXPORT_SYMBOL(tsp_irq_num);

int tsp_workqueue_num;
EXPORT_SYMBOL(tsp_workqueue_num);

int tsp_threadedirq_num;
EXPORT_SYMBOL(tsp_threadedirq_num);

int g_touch_info_x;
EXPORT_SYMBOL(g_touch_info_x);

int g_touch_info_y;
EXPORT_SYMBOL(g_touch_info_y);

int g_touch_info_press;
EXPORT_SYMBOL(g_touch_info_press);

int prev_pressed_num;
EXPORT_SYMBOL(prev_pressed_num);

static int pre_ta_stat;
int tsp_status;
EXPORT_SYMBOL(tsp_status);

int reset_check;/* flag to check in the reset sequecne : 1, or not : 0*/
EXPORT_SYMBOL(reset_check);


/*
static uint8_t raw_min=0;
static uint8_t raw_max=0;
static uint8_t IDAC_min=0;
static uint8_t IDAC_max=0;
*/
/*
extern int cypress_update(int);

int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);

void set_tsp_for_ta_detect(int);
*/
struct touch_trace_data {
	uint32_t time;
	int16_t fingernum;
	int8_t status;
	int8_t id;
	uint16_t x;
	uint16_t y;

};

#if defined(__TOUCH_DEBUG__)
#define MAX_TOUCH_TRACE_NUMBER	10000
static int touch_trace_index;
static struct touch_trace_data touch_trace_info[MAX_TOUCH_TRACE_NUMBER];
#endif

struct timespec ts;
struct rtc_time tm;

#ifdef __TOUCHKEY__
int touchkey_scan_mode;
EXPORT_SYMBOL(touchkey_scan_mode);

#define TOUCHKEY_SPEC_THRESHOLD 25
int touchkey_threshold_value = TOUCHKEY_SPEC_THRESHOLD;
#endif

/* sysfs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);

/*
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);
*/
struct device *sec_touchscreen;
EXPORT_SYMBOL(sec_touchscreen);
#ifdef __TOUCHKEY__
struct device *sec_touchkey;
EXPORT_SYMBOL(sec_touchkey);
#endif /* __TOUCHKEY__*/


static ssize_t read_threshold(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t firmware_In_Binary(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t firmware_In_TSP(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t firm_update(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t firmware_update_status(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t tsp_panel_rev(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t force_calibration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);


static DEVICE_ATTR(tsp_firm_version_phone, S_IRUGO /*0444*/,
	firmware_In_Binary, NULL);
static DEVICE_ATTR(tsp_firm_version_panel, S_IRUGO /*0444*/,
	firmware_In_TSP, NULL);
static DEVICE_ATTR(tsp_firm_update, S_IRUGO /*0444*/,
	firm_update, NULL);
static DEVICE_ATTR(tsp_firm_update_status, S_IRUGO /*0444*/,
	firmware_update_status, NULL);
static DEVICE_ATTR(tsp_threshold, S_IRUGO /*0444*/,
	read_threshold, NULL);
static DEVICE_ATTR(get_panel_rev, S_IRUGO /*0444*/,
	tsp_panel_rev, NULL);
static DEVICE_ATTR(tsp_force_calibration, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
	force_calibration_store);

#ifdef __TOUCHKEY__
static ssize_t menu_sensitivity_show(struct device *dev,
struct device_attribute *attr, char *buf);
static ssize_t back_sensitivity_show(struct device *dev,
	struct device_attribute *attr, char *buf);
static ssize_t touchkey_sensitivity_power_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size);
static ssize_t read_touchkey_threshold(struct device *dev,
	struct device_attribute *attr, char *buf);

static DEVICE_ATTR(touchkey_menu, S_IRUGO, menu_sensitivity_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, back_sensitivity_show, NULL);
static DEVICE_ATTR(touch_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
	touchkey_sensitivity_power_store);
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, read_touchkey_threshold, NULL);
#endif
/* sys fs */

static int firm_update_callfc(void);

#if defined(SEC_TSP_FACTORY_TEST)
#define TSP_CMD(name, func) .cmd_name = name, .cmd_func = func

struct tsp_cmd {
	struct list_head	list;
	const char	*cmd_name;
	void	(*cmd_func)(void *device_data);
};

static void fw_update(void *device_data);
static void get_fw_ver_bin(void *device_data);
static void get_fw_ver_ic(void *device_data);
static void get_config_ver(void *device_data);
static void get_threshold(void *device_data);
static void module_off_master(void *device_data);
static void module_on_master(void *device_data);
static void module_off_slave(void *device_data);
static void module_on_slave(void *device_data);
static void get_chip_vendor(void *device_data);
static void get_chip_name(void *device_data);
static void get_reference(void *device_data);
static void get_raw_count(void *device_data);
static void get_difference(void *device_data);
static void get_intensity(void *device_data);
static void get_local_idac(void *device_data);
static void get_global_idac(void *device_data);
static void get_x_num(void *device_data);
static void get_y_num(void *device_data);
static void run_reference_read(void *device_data);
static void run_raw_count_read(void *device_data);
static void run_difference_read(void *device_data);
static void run_intensity_read(void *device_data);
static void not_support_cmd(void *device_data);
static void run_raw_node_read(void *device_data);
static void run_global_idac_read(void *device_data);
static void run_local_idac_read(void *device_data);

struct tsp_cmd tsp_cmds[] = {
	{TSP_CMD("fw_update", fw_update),},
	{TSP_CMD("get_fw_ver_bin", get_fw_ver_bin),},
	{TSP_CMD("get_fw_ver_ic", get_fw_ver_ic),},
	{TSP_CMD("get_config_ver", get_config_ver),},
	{TSP_CMD("get_threshold", get_threshold),},
	{TSP_CMD("module_off_master", module_off_master),},
	{TSP_CMD("module_on_master", module_on_master),},
	{TSP_CMD("module_off_slave", module_off_slave),},
	{TSP_CMD("module_on_slave", module_on_slave),},
	{TSP_CMD("get_chip_vendor", get_chip_vendor),},
	{TSP_CMD("get_chip_name", get_chip_name),},
	{TSP_CMD("run_raw_node_read", run_raw_node_read),},
	{TSP_CMD("get_x_num", get_x_num),},
	{TSP_CMD("get_y_num", get_y_num),},
	{TSP_CMD("get_reference", get_reference),},
	{TSP_CMD("get_raw_count", get_raw_count),},
	{TSP_CMD("get_difference", get_difference),},
	{TSP_CMD("get_intensity", get_intensity),},
	{TSP_CMD("get_local_idac", get_local_idac),},
	{TSP_CMD("get_global_idac", get_global_idac),},
	{TSP_CMD("run_reference_read", run_reference_read),},
	{TSP_CMD("run_raw_count_read", run_raw_count_read),},
	{TSP_CMD("run_difference_read", run_difference_read),},
	{TSP_CMD("run_intensity_read", run_intensity_read),},
	{TSP_CMD("run_global_idac_read", run_global_idac_read),},
	{TSP_CMD("run_local_idac_read", run_local_idac_read),},
	{TSP_CMD("not_support_cmd", not_support_cmd),},
};
#endif

static int tsp_testmode;
static int prev_wdog_val = -1;
static int tsp_irq_operation;

/* kwon added 2013-02-22 */
static int tsp_releasing;

#ifdef TSP_EDS_RECOVERY
static unsigned int touch_present;
#endif

#define FW_DOWNLOADING "Downloading"
#define FW_DOWNLOAD_COMPLETE "Complete"
#define FW_DOWNLOAD_FAIL "FAIL"
#define FWUP_NOW -1

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ts_early_suspend(struct early_suspend *h);
static void ts_late_resume(struct early_suspend *h);
#endif

/*
extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
extern int set_irq_type(unsigned int irq, unsigned int type);
*/

/*extern int tsp_charger_type_status;*/

void touch_ctrl_regulator(int on_off)
{
 /*
	gpio_tlmm_config(GPIO_CFG(TOUCH_EN, 0,
	GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 0);
*/
	if (on_off == TOUCH_ON) {
		gpio_request(TOUCH_EN, "Touch_en");
		gpio_direction_output(TOUCH_EN, 1);
		gpio_set_value(TOUCH_EN, 1);
		gpio_free(TOUCH_EN);


#if defined(__TOUCH_KEYLED__)
/* Must change
	regulator_set_voltage(touchkeyled_regulator, 3300000, 3300000);
	regulator_enable(touchkeyled_regulator);
*/
#endif

	} else {
		gpio_request(TOUCH_EN, "Touch_en");
		gpio_direction_output(TOUCH_EN, 0);
		gpio_set_value(TOUCH_EN, 0);
		gpio_free(TOUCH_EN);

#if defined(__TOUCH_KEYLED__)
		regulator_disable(touchkeyled_regulator);
#endif
	}
}

/*EXPORT_SYMBOL(touch_ctrl_regulator);*/

int tsp_reset(void)
{
	int ret = 1;

#if defined(__TOUCH_DEBUG__)
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
#endif

	if (reset_check == 0) {
		reset_check = 1;

		touch_ctrl_regulator(0);

		gpio_direction_output(TSP_SCL , 0);
		gpio_direction_output(TSP_SDA , 0);
		/*gpio_direction_output(TSP_INT , 0);*/

		msleep(500);

		gpio_direction_output(TSP_SCL , 1);
		gpio_direction_output(TSP_SDA , 1);
		/*gpio_direction_output(TSP_INT , 1);*/

		touch_ctrl_regulator(1);

		/*msleep(10);*/
		usleep_range(10000, 20000);

		reset_check = 0;
	}

	return ret;
}

#ifdef __TOUCHKEY__
static void process_key_event(uint8_t tsk_msg)
{
	int i;
	int keycode = 0;
	int st_new;

	printk(KERN_INFO "[TSP] process_key_event : %d\n", tsk_msg);

	if (tsk_msg == 0) {
		input_report_key(ts_global->input_dev, st_old, 0);
		for (i = 0; i < MAX_KEYS; i++)
			touchkey_status[i] = TK_STATUS_RELEASE;
#if defined(__TOUCH_DEBUG__)
		printk(KERN_INFO
			"[TSP] release keycode: %4d, keypress: %4d\n",
			st_old, 0);
#endif
		printk(KERN_INFO "[TSP] keyrelease\n");
	} else {
		/*check each key status*/
		for (i = 0; i < MAX_KEYS; i++) {
			st_new = (tsk_msg>>(i)) & 0x1;
			if (st_new == 1) {
				keycode = touchkey_keycodes[i];
				input_report_key(ts_global->input_dev,
					keycode, 1);
				touchkey_status[i] = TK_STATUS_PRESS;
#if defined(__TOUCH_DEBUG__)
				printk(KERN_INFO
				"[TSP] press keycode: %4d, keypress: %4d\n",
				keycode, 1);
#endif
				printk(KERN_INFO "[TSP] keypress\n");
			}

			st_old = keycode;
		}
	}
}

static void force_release_key(void)
{
	int i;
	/*int keycode= 0;*/

	printk(KERN_INFO "[TSP] force_release_key ++\n");

	tsp_releasing = 1;
	for (i = 0; i < MAX_KEYS; i++) {
		if (touchkey_status[i] == TK_STATUS_PRESS) {
			input_report_key(ts_global->input_dev,
				touchkey_keycodes[i], 0);
			touchkey_status[i] = TK_STATUS_RELEASE;
#if defined(__TOUCH_DEBUG__)
			printk(KERN_INFO
				"[TSP] release keycode: %4d, keypress: %4d\n",
				touchkey_keycodes[i], 0);
#endif
			printk(KERN_INFO "[TSP] keyrelease\n");
		}
	}
	tsp_releasing = 0;
	printk(KERN_INFO "[TSP] force_release_key --\n");
}
#endif

#if defined(__TOUCH_DEBUG__)
void tsp_log(struct report_finger_info_t *fingerinfo, int i)
{
#if defined(__TOUCH_DEBUG__)
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	printk(KERN_INFO
		"[TSP][%02d:%02d:%02d.%03lu] ",
		tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
#endif

	touch_trace_info[touch_trace_index].time = jiffies;
	touch_trace_info[touch_trace_index].status = fingerInfo[i].status;
	touch_trace_info[touch_trace_index].id = fingerInfo[i].id;
	touch_trace_info[touch_trace_index].x = fingerInfo[i].x;
	touch_trace_info[touch_trace_index].y = fingerInfo[i].y;
	touch_trace_info[touch_trace_index].fingernum = i;

	printk(KERN_INFO "[TSP] i[%d] id[%d] xy[%d, %d] status[%d]\n", i,
		fingerInfo[i].id, fingerInfo[i].x,
		fingerInfo[i].y, fingerInfo[i].status);
/*
	if(fingerInfo[i].status==1)
		printk("[TSP] finger down\n");
	else
		printk("[TSP] finger up\n");
*/
	touch_trace_index++;

	if (touch_trace_index >= MAX_TOUCH_TRACE_NUMBER)
		touch_trace_index = 0;
	/* -- due to touch trace -- */

}
#endif
/* kwon added 2013-02-22 */
/*
static void force_release_all_fingers(void)
{
	int i;

	printk(KERN_DEBUG "[TSP] %s\n", __func__);

	for (i = 0; i < MAX_TOUCH_ID; i++) {
		fingerInfo[i].status = 0;

		input_mt_slot(ts_global->input_dev, i);
		input_mt_report_slot_state(ts_global->input_dev,
					MT_TOOL_FINGER, 0);
	}
	input_sync(ts_global->input_dev);
}
*/

static void force_release_all_fingers(void)
{
	int i;
	int released = 0;

	printk(KERN_DEBUG "[TSP] force_release_touch ++\n");

	tsp_releasing = 1;
	for (i = 0; i < MAX_USING_FINGER_NUM; i++) {
		if (1 == fingerInfo[i].status) {

			fingerInfo[i].status = 0;

			input_mt_slot(ts_global->input_dev, i);
			input_mt_report_slot_state(ts_global->input_dev,
						MT_TOOL_FINGER, 0);
			fingerInfo[i+2].status = 0;

			#if defined(__TOUCH_DEBUG__)
			printk(KERN_DEBUG
				"[TSP] release touch finger num : %d\n", i);

			tsp_log(fingerInfo, i);
			#endif
			printk(KERN_INFO "[TSP] %d release\n", i);

			released = 1;
		}
	}

	if (released)
		input_sync(ts_global->input_dev);

	tsp_releasing = 0;

	printk(KERN_DEBUG "[TSP] force_release_touch --\n");
}

#define ABS(a, b) (a > b ? (a-b) : (b-a))
static irqreturn_t ts_work_func(int irq, void *dev_id)
{
	int ret = 0;
	uint8_t buf[27]; /* 01h ~ 1Fh*/
	uint8_t i2c_addr = 0x01;
	int i = 0, j = 0;
	int pressed_num;
	#ifdef __TOUCHKEY__
	uint8_t buf_key[1];
	/*int button_check = 0;*/
	int touch_check = 0;
	#endif
	struct touch_data *ts = dev_id;

	/* kwon added 2013-02-22 */
	uint8_t SID_in1stbuf = 0; /* 1st buffer slot ID*/
	uint8_t FID_in1stbuf = 0; /* 1st buffer finger ID*/
	uint8_t SID_in2ndbuf = 0; /* 2nd buffer slot ID*/
	uint8_t FID_in2ndbuf = 0; /* 2nd buffer finger ID*/

	/* kwon added 2013-02-22 */
	if (tsp_testmode || tsp_releasing)
		return IRQ_HANDLED;

	tsp_irq_operation = 1;

	ret = tsp_i2c_read(i2c_addr, buf, sizeof(buf));

	if (ret <= 0) {
		printk(KERN_INFO "[TSP] i2c failed : ret=%d, ln=%d\n",
			ret, __LINE__);
		goto work_func_out;
	}

	if ((0x20&buf[0]) == 0x20) {
		printk(KERN_INFO "[TSP] buf[0]=%d 5 bit is 1 retry i2c,\n",
			buf[0]);
		ret = tsp_i2c_read(i2c_addr, buf, sizeof(buf));
		printk(KERN_INFO "[TSP] buf[0]=%d\n", buf[0]);

		/* kwon added 2013-02-22 */
		goto work_func_out;
	}

	pressed_num = 0x0f&buf[1];

	/* kwon added 2013-02-22 */
	if (pressed_num == 0) {
		SID_in1stbuf = 0;
		FID_in1stbuf = 0;
		SID_in2ndbuf = 1;
		FID_in2ndbuf = 0;
	} else if (pressed_num == 1) {
		/* pressed finger in 1st buffer*/
		FID_in1stbuf = buf[7] >> 4;
		SID_in1stbuf = (FID_in1stbuf - 1) & 0x3;/*max size:4*/

		if (FID_in1stbuf == 1) { /* finger ID=1*/

			FID_in2ndbuf = 0; /* released finger ID*/
			SID_in2ndbuf = 1; /* released slot ID*/
		} else {/* finger ID=2*/

			FID_in2ndbuf = 0;
			SID_in2ndbuf = 0;
		}
	} else {
		/* pressed finger in 1st buffer*/
		FID_in1stbuf = buf[7] >> 4;
		SID_in1stbuf = (FID_in1stbuf - 1) & 0x3;/*max size:4*/
		/* pressed finger in 2nd buffer*/
		FID_in2ndbuf = buf[7] & 0xf;
		SID_in2ndbuf = (FID_in2ndbuf - 1) & 0x3; /*max size:4*/
	}

	fingerInfo[SID_in1stbuf].x = (buf[2] << 8) | buf[3];
	fingerInfo[SID_in1stbuf].y = (buf[4] << 8) | buf[5];
	fingerInfo[SID_in1stbuf].z = buf[6];
	fingerInfo[SID_in1stbuf].id = FID_in1stbuf;
	fingerInfo[SID_in2ndbuf].x = (buf[8] << 8) | buf[9];
	fingerInfo[SID_in2ndbuf].y = (buf[10] << 8) | buf[11];
	fingerInfo[SID_in2ndbuf].z = buf[12];
	fingerInfo[SID_in2ndbuf].id = FID_in2ndbuf;

#ifdef __TOUCHKEY__

	buf_key[0] = buf[26] & 0x03; /*information of touch key*/

	/* kwon added 2013-02-22 */
	/*button_check = buf[0] & 0x40;*/

	for (i = 0; i < MAX_TOUCH_ID; i++) {
		if (fingerInfo[i].id >= 1 || fingerInfo[i].status == 1) {
			touch_check = 1;
			break;
		}
	}

	if (touch_check == 0 && buf_key[0] != 0) {
		process_key_event(buf_key[0]);
		goto key_process_ok;
	}

#endif

#if defined(__TOUCH_DEBUG__)
	printk(KERN_INFO "[TSP] pressed finger num [%d]\n", pressed_num);
#endif

#if defined(__SEND_VIRTUAL_RELEASED__)
	/* check touch event */
	for (i = 0; i < MAX_USING_FINGER_NUM; i++) {
		if (fingerInfo[i].id >= 1) {   /*press interrupt*/

/* kwon added 2013-02-22 */
#if defined(TOUCH_BOOSTER)
			if (pressed_num >= 1 && prev_pressed_num == 0) {
				prcmu_qos_update_requirement(
					PRCMU_QOS_APE_OPP,
					(char *)ts_global->client->name,
					PRCMU_QOS_APE_OPP_MAX);
				prcmu_qos_update_requirement(
					PRCMU_QOS_DDR_OPP,
					(char *)ts_global->client->name,
					PRCMU_QOS_DDR_OPP_MAX);
				prcmu_qos_update_requirement(
					PRCMU_QOS_ARM_KHZ,
					(char *)ts_global->client->name,
					800000);
			}
#endif
			fingerInfo[i].status = 1;

			input_mt_slot(ts->input_dev, i);
			input_mt_report_slot_state(ts->input_dev,
				MT_TOOL_FINGER, 1);

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
				fingerInfo[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
				fingerInfo[i].y);

			printk(KERN_INFO"[TSP] %d press\n", i);
#if defined(__TOUCH_DEBUG__)
			tsp_log(fingerInfo, i);
#endif
		} else if (fingerInfo[i].id == 0) {
			/*release interrupt (only first finger)*/
			if (fingerInfo[i].status == 1) {
				/* prev status is press*/
				fingerInfo[i].status = 0;

				input_mt_slot(ts->input_dev, i);
				input_mt_report_slot_state(ts->input_dev,
					MT_TOOL_FINGER, 0);
				printk(KERN_INFO"[TSP] %d release\n", i);
#if defined(__TOUCH_DEBUG__)
				tsp_log(fingerInfo, i);
#endif
			} else {  /*touch key release*/
				for (j = 0; j < MAX_KEYS; j++) {
					if (
					touchkey_status[j] == TK_STATUS_PRESS
					&& touchkey_keycodes[j] == st_old) {
						input_report_key(
						ts_global->input_dev,
						st_old, 0);
					}

					if (
					touchkey_status[j] == TK_STATUS_PRESS
					) {
						touchkey_status[j] = \
							TK_STATUS_RELEASE;
#if defined(__TOUCH_DEBUG__)
						printk(KERN_INFO
						"[TSP] release keycode: %4d,"\
						"keypress: %4d\n", st_old, 0);
#endif
						printk(KERN_INFO
							"[TSP] keyrelease\n");
					}
				}
			}

/* kwon added 2013-02-22 */
#if defined(TOUCH_BOOSTER)
			if (pressed_num == 0) {
				prcmu_qos_update_requirement(
					PRCMU_QOS_APE_OPP,
					(char *)ts_global->client->name,
					PRCMU_QOS_DEFAULT_VALUE);
				prcmu_qos_update_requirement(
					PRCMU_QOS_DDR_OPP,
					(char *)ts_global->client->name,
					PRCMU_QOS_DEFAULT_VALUE);
				prcmu_qos_update_requirement(
					PRCMU_QOS_ARM_KHZ,
					(char *)ts_global->client->name,
					PRCMU_QOS_DEFAULT_VALUE);
			}
#endif
		}

	}
#else
	/* check touch event */
	for (i = 0; i < MAX_USING_FINGER_NUM; i++) {
		/*IC*/
		if (fingerInfo[i].id >= 1) { /* press interrupt*/
			fingerInfo[i].status = 1;
		} else if (fingerInfo[i].id == 0) {
			/* release interrupt (only first finger)*/
			if (fingerInfo[i].status == 1)  /*prev is press*/
				fingerInfo[i].status = 0;
			else if (fingerInfo[i].status == 0) /*force release*/
				fingerInfo[i].status = -1;
		}

		if (fingerInfo[i].status < 0)
			continue;
		/*IC*/

		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
			fingerInfo[i].status);

		if (fingerInfo[i].status) {
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
				fingerInfo[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
				fingerInfo[i].y);
		}
#if defined(__TOUCH_DEBUG__)
		printk(KERN_INFO
			"[TSP] i[%d] id[%d] xyz[%d, %d, %d] status[%d]\n",
			i, fingerInfo[i].id, fingerInfo[i].x, fingerInfo[i].y,
			fingerInfo[i].z, fingerInfo[i].status);
#endif
	}
#endif

#if defined(__SEND_VIRTUAL_RELEASED__)
	fingerInfo[2].x = fingerInfo[0].x;
	fingerInfo[2].y = fingerInfo[0].y;
	fingerInfo[2].z = fingerInfo[0].z;
	fingerInfo[2].id = fingerInfo[0].id;

	fingerInfo[3].x = fingerInfo[1].x;
	fingerInfo[3].y = fingerInfo[1].y;
	fingerInfo[3].z = fingerInfo[1].z;
	fingerInfo[3].id = fingerInfo[1].id;

	prev_pressed_num = pressed_num;

#endif

key_process_ok:
	input_sync(ts->input_dev);

work_func_out:

	tsp_irq_operation = 0;

	return IRQ_HANDLED;
}


int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size)
{
	int i, ret = -1;
	struct i2c_msg rmsg;
	uint8_t start_reg;

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		rmsg.addr = ts_global->client->addr;
		rmsg.flags = 0; /*I2C_M_WR;*/
		rmsg.len = 1;
		rmsg.buf = &start_reg;
		start_reg = reg;

		ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

		if (ret >= 0) {
			rmsg.flags = I2C_M_RD;
			rmsg.len = buf_size;
			rmsg.buf = rbuf;
			ret = i2c_transfer(ts_global->client->adapter,
				&rmsg, 1);

			if (ret >= 0)
				break; /* i2c success*/
		}

		if (i == (I2C_RETRY_CNT - 1))
			printk(KERN_ERR
			"[TSP] Error code : %d, %d\n", __LINE__, ret);
	}

	return ret;
}



void set_tsp_for_ta_detect(int state)
{

	int i, ret = 0;
	uint8_t buf1[2] = {0,};
	uint8_t temp;
#if defined(__TOUCH_DEBUG__)
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
#endif
	if (tsp_status == 0 &&
	((tsp_testmode == 0)
	&& (tsp_irq_operation == 0 && (now_tspfw_update == 0)))) {
		if (state) {
			printk(KERN_INFO
			"[TSP] [1] set_tsp_for_ta_detect!!! state=1\n");

			for (i = 0; i < I2C_RETRY_CNT; i++) {
				buf1[0] = 0x01; /*address*/
				ret = i2c_master_send(ts_global->client,
					buf1, 1);

				if (ret >= 0)
					ret = i2c_master_recv(
						ts_global->client, buf1, 1);

				if (ret >= 0) {
					temp = buf1[0] | 0x04;/*0b0000 0100*/

					buf1[0] = 0x01;/*address*/
					buf1[1] = temp;/*data*/
					ret = i2c_master_send(
						ts_global->client, buf1, 2);

					if (ret >= 0) {
						printk(KERN_INFO
							"[TSP] 01h = 0x%x\n",
							temp);
						break; /* i2c success*/
					}
				}

				printk(KERN_INFO "[TSP] %s, %d, fail\n",
					__func__, __LINE__);
			}

			pre_ta_stat = 1;
		} else {
			printk(KERN_INFO
			"[TSP] [2] set_tsp_for_ta_detect!!! state=0\n");

			for (i = 0; i < I2C_RETRY_CNT; i++) {
				buf1[0] = 0x01; /*address*/
				ret = i2c_master_send(ts_global->client,
					buf1, 1);

				if (ret >= 0)
					ret = i2c_master_recv(
						ts_global->client, buf1, 1);

				if (ret >= 0) {
					temp = buf1[0] & 0xFB;/*0b1111 1011*/

					buf1[0] = 0x01;/*address*/
					buf1[1] = temp;/*data*/
					ret = i2c_master_send(
						ts_global->client, buf1, 2);

					if (ret >= 0) {
						printk(KERN_INFO
							"[TSP] 01h = 0x%x\n",
							temp);
						break; /* i2c success*/
					}

				}

				printk(KERN_INFO "[TSP] %s, %d, fail\n",
					__func__, __LINE__);
			}

			pre_ta_stat = 0;
		}
	}
}

#if defined(TSP_EDS_RECOVERY)
static void check_ic_work_func(struct work_struct *esd_recovery_func)
{
	int ret = 0;
	uint8_t buf_esd[1];
	uint8_t wdog_val[1];

	struct touch_data *ts = container_of(esd_recovery_func,
		struct touch_data, esd_recovery_func);

	buf_esd[0] = 0x1F;
	wdog_val[0] = 1;

	if ((tsp_testmode == 0)
	&& (tsp_irq_operation == 0 && (now_tspfw_update == 0))) {
		ret = i2c_master_send(ts->client, buf_esd, 1);
		if (ret >= 0) {
			ret = i2c_master_recv(ts->client, wdog_val, 1);

			if (((wdog_val[0] & 0xFC) >> 2)
			== (uint8_t)prev_wdog_val) {
				printk(KERN_INFO
				"[TSP] %s tsp_reset counter = %x, prev = %x\n",
				__func__,
				((wdog_val[0] & 0xFC) >> 2),
				(uint8_t)prev_wdog_val);

				disable_irq(ts_global->client->irq);
				force_release_all_fingers();
				tsp_reset();
				enable_irq(ts_global->client->irq);
				prev_wdog_val = -1;
			} else {
				#if defined(__TOUCH_DEBUG__)
				printk(KERN_INFO
					"[TSP] %s, No problem of tsp driver\n",
					__func__);
				#endif

				prev_wdog_val = (wdog_val[0] & 0xFC) >> 2;
			}
			esd_conter = 0;
		} else { /*if(ret < 0) */
			if (esd_conter == 1) {
				disable_irq(ts_global->client->irq);
				force_release_all_fingers();
				tsp_reset();
				enable_irq(ts_global->client->irq);
				printk(KERN_INFO
					"[TSP]  %s : tsp_reset() done!\n",
					__func__);
				esd_conter = 0;
			} else {
				esd_conter++;
#if defined(__TOUCH_DEBUG__)
				printk(KERN_INFO
					"[TSP]  %s : esd_conter [%d]\n",
					__func__, esd_conter);
#endif
			}
		}

		if (pre_ta_stat != tsp_charger_type_status)
			set_tsp_for_ta_detect(tsp_charger_type_status);

	} else {
#if defined(__TOUCH_DEBUG__)
		printk(KERN_INFO "[TSP] %s cannot check ESD\n");
#endif
	}
}

static enum hrtimer_restart watchdog_timer_func(struct hrtimer *timer)
{
	/*printk("[TSP] %s, %d\n", __func__, __LINE__ );*/

	queue_work(check_ic_wq, &ts_global->esd_recovery_func);
	hrtimer_start(&ts_global->timer, ktime_set(3, 0), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

#endif


int ts_check(void)
{
	int ret, i;
	uint8_t buf_tmp[3] = {0, 0, 0};
	int retry = 3;


	ret = tsp_i2c_read(0x1B, buf_tmp, sizeof(buf_tmp));

	/* i2c read retry*/
	if (ret <= 0) {
		for (i = 0; i < retry; i++) {
			ret = tsp_i2c_read(0x1B, buf_tmp, sizeof(buf_tmp));

			if (ret > 0)
				break;
		}
	}

	if (ret <= 0) {
		printk(KERN_INFO "[TSP][%s] %s\n", __func__, "Failed i2c");
		ret = 0;
	} else {
		printk(KERN_INFO "[TSP][%s] %s\n", __func__, "Passed i2c");

		printk(KERN_INFO
			"[TSP][%s][SlaveAddress : 0x%x][VendorID : 0x%x]"\
			"[HW : 0x%x] [SW : 0x%x]\n",
			__func__, ts_global->client->addr,
			buf_tmp[0], buf_tmp[1], buf_tmp[2]);

		/*(ts->hw_rev == 0) && (ts->fw_ver == 2))*/
		if (buf_tmp[0] == 0xf0) {
			ret = 1;
			printk(KERN_INFO "[TSP][%s] %s\n", __func__,
				"Passed ts_check");
		} else {
			ret = 0;
			printk(KERN_INFO "[TSP][%s] %s\n", __func__,
				"Failed ts_check");
		}

	}

	return ret;
}


#ifdef SEC_TSP_FACTORY_TEST
static void set_cmd_result(struct touch_data *info, char *buff, int len)
{
	/*strncat(info->cmd_result, buff, len);*/
	strlcat(info->cmd_result, buff, sizeof(info->cmd_result) - 1);
}

/*
static int get_fw_version(struct touch_data *info)
{
	uint8_t i2c_addr = 0x1B;
	uint8_t buf_tmp[3] = {0};

	printk(KERN_INFO "[TSP] %s\n", __func__);

	tsp_i2c_read(i2c_addr, buf_tmp, sizeof(buf_tmp));

	touch_vendor_id = buf_tmp[0] & 0xF0;
	touch_hw_ver = buf_tmp[1];
	touch_sw_ver = buf_tmp[2];
	printk(KERN_INFO "[TSP] %s:%d, ver tsp=%x, HW=%x, SW=%x\n",
		__func__, __LINE__,
		touch_vendor_id, touch_hw_ver, touch_sw_ver);

	return touch_sw_ver;

}
*/
static int get_hw_version(struct touch_data *info)
{
	uint8_t i2c_addr = 0x1B;
	uint8_t buf_tmp[3] = {0};

	printk(KERN_INFO "[TSP] %s\n", __func__);

	tsp_i2c_read(i2c_addr, buf_tmp, sizeof(buf_tmp));

	touch_vendor_id = buf_tmp[0] & 0xF0;
	touch_hw_ver = buf_tmp[1];
	touch_sw_ver = buf_tmp[2];
	printk(KERN_INFO "[TSP] %s:%d, ver tsp=%x, HW=%x, SW=%x\n",
		__func__, __LINE__,
		touch_vendor_id, touch_hw_ver, touch_sw_ver);

	return touch_hw_ver;
}


static ssize_t show_close_tsp_test(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return snprintf(buf, TSP_BUF_SIZE, "%u\n", 0);
}

static void set_default_result(struct touch_data *info)
{
	/*char delim = ':';*/
	char *delim = ":";

	memset(info->cmd_result, 0x00, ARRAY_SIZE(info->cmd_result));
	memcpy(info->cmd_result, info->cmd, strnlen(info->cmd,
			sizeof(info->cmd)));
	/*strncat(info->cmd_result, &delim, 1);*/
	strlcat(info->cmd_result, delim, sizeof(info->cmd_result));
}

static void not_support_cmd(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;
	char buff[16] = {0};

	set_default_result(info);
	snprintf(buff, sizeof(buff), "%s", "NA");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 4;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));
	return;
}

static void fw_update(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;
	int ret = 0;
	char buff[21] = {0};

	tsp_testmode = 1;
	set_default_result(info);

	/*tsp_special_update:
	    0: normal firmware update from phone-binary
	    1: special firmware update from t-flash*/
	tsp_special_update = info->cmd_param[0];

	ret = firm_update_callfc();

	snprintf(buff, sizeof(buff), "%s", IsfwUpdate);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: \"%s(%d)\"\n", __func__,
				buff, strnlen(buff, sizeof(buff)));

	tsp_testmode = 0;
	/*not_support_cmd(info);*/
}

static void run_raw_node_read(void *device_data) /*item 1*/
{


	struct touch_data *info = (struct touch_data *)device_data;

	int tma140_col_num = NODE_Y_NUM; /*0 ~ 7*/
	int tma140_row_num = NODE_X_NUM;/*0 ~ 9*/


	uint8_t buf1[2] = {0,};
	uint8_t buf2[NODE_NUM] = {0,};

	uint16_t ref1[NODE_NUM] = {0,};
	uint16_t ref2[NODE_NUM] = {0,};
	char buff[TSP_CMD_STR_LEN] = {0};
	uint8_t max_value = 0, min_value = 0;

	int i, ret;

	uint8_t i2c_addr;

	uint16_t RAWDATA_MIN = 70;
	uint16_t RAWDATA_MAX = 130;
	uint16_t LIDAC_MIN = 1;
	uint16_t LIDAC_MAX = 30;

	uint8_t test_result = 1;

	tsp_testmode = 1;

	set_default_result(info);


	/* Raw Value */
	/* Enter Raw Data Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x40;/*value*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	/*msleep(10);*/
	usleep_range(10000, 20000);

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0xC0;/*value*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}
	msleep(50);

	/* Read Raw Data */
	i2c_addr = 0x07;
	tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));

	printk(KERN_INFO "[TSP] Raw Value : ");
	for (i = 0 ; i < (tma140_col_num * tma140_row_num); i++) {
		if (i == 0) {
			min_value = max_value = buf2[i];

		} else {
			max_value = max(max_value, buf2[i]);
			min_value = min(min_value, buf2[i]);
		}

		printk(KERN_INFO " [%d]%3d", i, buf2[i]);
	}
	printk(KERN_INFO "\n");


	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/* Exit Inspection Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		/*exit Inspection Mode*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	/*mdelay(100);*/
	msleep(100);

	tsp_testmode = 0;


	/* Check Result */
	for (i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++) {
		if (ref1[i] < RAWDATA_MIN && ref1[i] > RAWDATA_MAX) {
			test_result = 0;
			break;
		}

		if (ref2[i] < LIDAC_MIN && ref2[i] > LIDAC_MAX) {
			test_result = 0;
			break;
		}
	}

	printk(KERN_INFO "[TSP] test_result = %d", test_result);



}


static void get_fw_ver_bin(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	int hw_rev;

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);


	set_default_result(info);
	hw_rev = get_hw_version(info);
	if (hw_rev == TSP_HW_VER1)
		snprintf(buff, sizeof(buff), "CY%02x%02x%02x",
			TSP_HW_VER1, TSP_SW_VER1_H, TSP_SW_VER1_L);
	else
		snprintf(buff, sizeof(buff), "CY%02x%02x%02x",
			TSP_HW_VER3, TSP_SW_VER3_H, TSP_SW_VER3_L);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_fw_ver_ic(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	uint8_t buf1[2] = {0,};
	uint8_t buf2[7] = {0,};
	uint8_t i2c_addr;
	uint8_t ic_vendorh = 0;
	uint8_t ic_vendorl = 0;
	uint8_t module_vendorh = 0;
	uint8_t module_vendorl = 0;
	uint8_t hw_id = 0;
	uint8_t fw_versionh = 0;
	uint8_t fw_versionl = 0;
	int i, ret;

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	tsp_testmode = 1;


	/* Enter System Information Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x20;/*value*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}
	msleep(50);

	i2c_addr = 0x03;
	tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));

	ic_vendorh = buf2[0];
	ic_vendorl = buf2[1];
	module_vendorh = buf2[2];
	module_vendorl = buf2[3];
	hw_id = buf2[4];
	fw_versionh = buf2[5];
	fw_versionl = buf2[6];

	printk(KERN_INFO "[TSP]  %s:%d, IC:%c%c, MODULE:%c%c,"\
		"HW ID:%02x, FW:%02x%02x\n",
		__func__, __LINE__,
		ic_vendorh, ic_vendorl,
		module_vendorh, module_vendorl,
		hw_id, fw_versionh, fw_versionl);

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%c%c%02x%02x%02x", ic_vendorh,
		ic_vendorl, hw_id, fw_versionh, fw_versionl);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		/*exit Inspection Mode*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	msleep(50);

	tsp_testmode = 0;
}

static void get_threshold(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	uint8_t buf1[2] = {0,};
	uint8_t buf2[1] = {0,};


	char buff[TSP_CMD_STR_LEN] = {0};
	int i, ret;

	uint8_t i2c_addr;

	tsp_testmode = 1;


	/* Enter System Information Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x10;/*value*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}
	msleep(50);

	/*  Read Threshold Value */
	i2c_addr = 0x30;
	tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));

	printk(KERN_INFO " [TSP] %d", buf2[0]);

	snprintf(buff, sizeof(buff), "%d", buf2[0]);

	set_default_result(info);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/* Exit System Information Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		/*exit Inspection Mode*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	msleep(100);

	tsp_testmode = 0;

}

static void run_global_idac_read(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	uint8_t buf1[2] = {0,};
	uint8_t buf2[6+NODE_X_NUM*NODE_Y_NUM] = {0,}; /* 0 ~ 6 , 7 ~ 86*/
	uint8_t odd_even_detect;
	uint8_t max_value = 0, min_value = 0;
	char buff[TSP_CMD_STR_LEN] = {0};

	int i, j;
	int ret;

	uint8_t i2c_addr;


	tsp_testmode = 1;
	set_default_result(info);

	/* Global IDAC Value */
	/* Enter Local IDAC Data Mode */

	for (i = 0; i < 10; i++) {
		for (j = 0; j < I2C_RETRY_CNT; j++) {
			buf1[0] = 0x00;/*address*/
			buf1[1] = 0x60;/*value*/
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0)
				break; /* i2c success*/
		}
		msleep(400);

	/* Read Global IDAC Data */

		i2c_addr = 0x01;

		tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));
		odd_even_detect = buf2[0]>>6;

#if defined(__TOUCH_DEBUG__)
		for (j = 0; j < 6+NODE_X_NUM*NODE_Y_NUM; j++)
			printk(KERN_INFO "%d ", buf2[j]);

		printk(KERN_INFO "\n");

		printk(KERN_INFO
			"Global IDAC odd_even_detect=%d , buf2[0]=%d\n",
			odd_even_detect, buf2[0]);
	#endif
		if (odd_even_detect%2)
			break;

		msleep(100);
	}

	printk(KERN_INFO "[TSP] Global IDAC Value : ");
	for (i = GLOBAL_IDAC_START ;
	i < GLOBAL_IDAC_START+GLOBAL_IDAC_NUM ; i++) {

		if (i == GLOBAL_IDAC_START) {
			min_value = max_value = buf2[i];

		} else {
			max_value = max(max_value, buf2[i]);
			min_value = min(min_value, buf2[i]);
		}

		j = i - GLOBAL_IDAC_START;
		global_idac[j] = buf2[i];
		printk(KERN_INFO " %d", global_idac[j]);
	}
	printk(KERN_INFO "\n");

	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/* Exit Inspection Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		/*exit Inspection Mode*/
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	/*mdelay(100);*/
	msleep(100);

	tsp_testmode = 0;

	/*info->cmd_state = 2;*/

}

static void run_local_idac_read(void *device_data)
{

	struct touch_data *info = (struct touch_data *)device_data;

	int tma140_col_num = NODE_Y_NUM;
	int tma140_row_num = NODE_X_NUM;

	uint8_t buf1[2] = {0,};
	uint8_t buf2[86] = {0,}; /* 0 ~ 5 , 6 ~ 85*/

	uint8_t max_value = 0, min_value = 0;
	char buff[TSP_CMD_STR_LEN] = {0};

	int i, j, ret;
	uint8_t odd_even_detect;

	uint8_t i2c_addr;
	tsp_testmode = 1;
	set_default_result(info);
	/* Local IDAC Value */
	/* Enter Local IDAC Data Mode */


	for (i = 0; i < 10; i++) {
		for (j = 0; j < I2C_RETRY_CNT; j++) {
			buf1[0] = 0x00;/*address*/
			buf1[1] = 0x60;/*value*/
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0)
				break; /* i2c success*/
		}
		msleep(400);


		/* Read Local IDAC Data */

		i2c_addr = 0x01;

		tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));
		odd_even_detect = buf2[0]>>6;

    #if defined(__TOUCH_DEBUG__)

		for (j = 0; j < 6+NODE_X_NUM*NODE_Y_NUM; j++)
			printk(KERN_INFO "%d ", buf2[j]);

		printk(KERN_INFO "\n");

		printk(KERN_INFO
			"Local IDAC, odd_even_detect=%d , buf2[0]=%d\n ",
			odd_even_detect, buf2[0]);
	#endif

		if (odd_even_detect%2 == 0)
			break;

		msleep(100);
	}

	if (i == 3 && odd_even_detect%2)
		printk(KERN_INFO " Error get Local IDAC");


	printk(KERN_INFO "[TSP] Local IDAC Value : ");
	for (i = LOCAL_IDAC_START;
	i < LOCAL_IDAC_START + (tma140_col_num * tma140_row_num); i++) {

		if (i == GLOBAL_IDAC_START) {
			min_value = max_value = buf2[i];

		} else {
			max_value = max(max_value, buf2[i]);
			min_value = min(min_value, buf2[i]);
		}

		j = i-LOCAL_IDAC_START;
		local_idac[j] = buf2[i];
		printk(KERN_INFO " %d", local_idac[j]);
	}
	printk(KERN_INFO "\n");

	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/* Exit Inspection Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		/*exit Inspection Mode*/
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	/*mdelay(100);*/
	msleep(100);

	tsp_testmode = 0;

	/*info->cmd_state = 2;*/

}

static void module_off_master(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[3] = {0};

	tsp_status = 1;

	if (info->use_irq && info->enabled) {
		disable_irq(info->client->irq);
		info->enabled = false;
	}

	gpio_direction_output(TSP_INT , 0);
	gpio_direction_output(TSP_SCL , 0);
	gpio_direction_output(TSP_SDA , 0);

	msleep(20);

	touch_ctrl_regulator(TOUCH_OFF);

	snprintf(buff, sizeof(buff), "%s", "OK");
	/*snprintf(buff, sizeof(buff), "%s", "NG");*/

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	info->cmd_state = 2;
	/*info->cmd_state = 3;*/

	dev_info(&info->client->dev, "%s: %s\n", __func__, buff);

	/*not_support_cmd(info);*/
}

static void module_on_master(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;
	char buff[3] = {0};

	gpio_direction_output(TSP_SCL , 1);
	gpio_direction_output(TSP_SDA , 1);

	gpio_direction_input(TSP_INT);

	touch_ctrl_regulator(TOUCH_ON);

	msleep(100);

	if (info->use_irq && !info->enabled) {
		enable_irq(info->client->irq);
		info->enabled = true;
	}

	tsp_status = 0;

	snprintf(buff, sizeof(buff), "%s", "OK");
	/*snprintf(buff, sizeof(buff), "%s", "NG");*/

	set_default_result(info);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));

	info->cmd_state = 2;
	/*info->cmd_state = 3;*/

	dev_info(&info->client->dev, "%s: %s\n", __func__, buff);

	/*not_support_cmd(info);*/
}

static void module_off_slave(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void module_on_slave(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void get_chip_vendor(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "CYPRESS");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_chip_name(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "TMA140");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static int check_rx_tx_num(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[TSP_CMD_STR_LEN] = {0};
	int node;

	if (info->cmd_param[0] < 0 ||
		info->cmd_param[0] >= NODE_X_NUM  ||
		info->cmd_param[1] < 0 ||
		info->cmd_param[1] >= NODE_Y_NUM) {
		snprintf(buff, sizeof(buff) , "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;

		dev_info(&info->client->dev, "%s: parameter error: %u,%u\n",
				__func__, info->cmd_param[0],
				info->cmd_param[1]);
		node = -1;
		return node;
	}
	/*node = info->cmd_param[1] * NODE_Y_NUM + info->cmd_param[0];*/
	node = info->cmd_param[0] * NODE_Y_NUM + info->cmd_param[1];
	dev_info(&info->client->dev, "%s: node = %d\n", __func__,
			node);
	return node;

}


static int global_value_check(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[TSP_CMD_STR_LEN] = {0};
	int value = -1;

	if (info->cmd_param[0] < 0 ||
		info->cmd_param[0] >= NODE_X_NUM  ||
		info->cmd_param[1] < 0 ||
		info->cmd_param[1] >= NODE_Y_NUM) {
		snprintf(buff, sizeof(buff) , "%s", "NG");
		set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
		info->cmd_state = 3;

		dev_info(&info->client->dev, "%s: parameter error: %u,%u\n",
				__func__, info->cmd_param[0],
				info->cmd_param[1]);

		return value;
	}

	value = info->cmd_param[0];
	dev_info(&info->client->dev, "%s: global value = %d\n", __func__,
			value);
	return value;

}



static void get_reference(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static void get_raw_count(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = raw_count[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void get_difference(void *device_data) /* item2*/
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = difference[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));
}

static void get_intensity(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}


static void get_local_idac(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	unsigned int val;
	int node;

	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = local_idac[node];
	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));

}


static void get_global_idac(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};
	unsigned int val;

	int x_channel_value = 0;

/*
	int node=0;
	set_default_result(info);
	node = check_rx_tx_num(info);

	if (node < 0)
		return;

	val = global_idac[node];
*/
	set_default_result(info);

	x_channel_value = global_value_check(info);

	val = global_idac[x_channel_value];

	printk(KERN_INFO "global_idac[%d]=%d\n", x_channel_value, val);

	snprintf(buff, sizeof(buff), "%u", val);
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__, buff,
			strnlen(buff, sizeof(buff)));


}

static void get_x_num(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};

	printk(KERN_INFO "[TSP] %s, x channel=%d\n", __func__ , NODE_X_NUM);

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%d", NODE_X_NUM);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_y_num(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[16] = {0};

	printk(KERN_INFO "[TSP] %s, y channel=%d\n", __func__, NODE_Y_NUM);

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%d", NODE_Y_NUM);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));
}

static void get_config_ver(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	char buff[20] = {0};

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	set_default_result(info);

	snprintf(buff, sizeof(buff), "%s", "CYPRESS_TMA140");
	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;

	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

}


static void run_reference_read(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);

/*	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__); */
}

static void run_raw_count_read(void *device_data) /*item 1*/
{
	struct touch_data *info = (struct touch_data *)device_data;

	int tma140_col_num = NODE_Y_NUM; /*0 ~ 7*/
	int tma140_row_num = NODE_X_NUM;/*0 ~ 9*/

	uint8_t buf1[2] = {0,};
	uint8_t buf2[NODE_NUM] = {0,};

	uint16_t ref1[NODE_NUM] = {0,};
	uint16_t ref2[NODE_NUM] = {0,};
	char buff[TSP_CMD_STR_LEN] = {0};
	uint8_t max_value = 0, min_value = 0;

	int i, ret;

	uint8_t i2c_addr;

	uint16_t RAWDATA_MIN = 70;
	uint16_t RAWDATA_MAX = 130;
	uint16_t LIDAC_MIN = 1;
	uint16_t LIDAC_MAX = 30;

	uint8_t test_result = 1;

	tsp_testmode = 1;

	set_default_result(info);


	/* Raw Value */
	/* Enter Raw Data Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x40;/*value*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	/*msleep(10);*/
	usleep_range(10000, 20000);

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0xC0;/*value*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}
	msleep(50);

	/* Read Raw Data */
	i2c_addr = 0x07;
	tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));

	printk(KERN_INFO "[TSP] Raw Value : ");
	for (i = 0 ; i < (tma140_col_num * tma140_row_num); i++) {
		if (i == 0) {
			min_value = max_value = buf2[i];

		} else {
			max_value = max(max_value, buf2[i]);
			min_value = min(min_value, buf2[i]);
		}

		raw_count[i] = buf2[i];
		printk(KERN_INFO " [%d]%3d", i, buf2[i]);
	}
	printk(KERN_INFO "\n");


	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/* Exit Inspection Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		/*exit Inspection Mode*/
		ret = i2c_master_send(info->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	/*mdelay(100);*/
	msleep(50);

	tsp_testmode = 0;


	/* Check Result */
	for (i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++) {
		if (ref1[i] < RAWDATA_MIN && ref1[i] > RAWDATA_MAX) {
			test_result = 0;
			break;
		}

		if (ref2[i] < LIDAC_MIN && ref2[i] > LIDAC_MAX) {
			test_result = 0;
			break;
		}
	}

	printk(KERN_INFO "[TSP] test_result = %d", test_result);

	info->cmd_state = 2;

}

static void run_difference_read(void *device_data) /*item2*/
{
	struct touch_data *info = (struct touch_data *)device_data;
	int tma140_col_num = NODE_Y_NUM; /*0 ~ 7*/
	int tma140_row_num = NODE_X_NUM;/*0 ~ 9*/

	uint8_t buf1[2] = {0,};
	uint8_t buf2[NODE_NUM] = {0,};
	uint8_t max_value = 0, min_value = 0;
	char buff[TSP_CMD_STR_LEN] = {0};

	int i, ret;
	uint8_t i2c_addr;
	tsp_testmode = 1;

	set_default_result(info);

	/* Difference Value */
	/* Enter Difference Data Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x50;/*value*/
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	/*msleep(10);*/
	usleep_range(10000, 20000);

	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0xD0;/*value*/
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}
	msleep(50);

	/* Read Difference Data */
	i2c_addr = 0x07;
	tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));
/*
	for (i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
		difference[i] = buf2[i];
*/
	printk(KERN_INFO "[TSP] difference Value : ");
	for (i = 0 ; i < (tma140_col_num * tma140_row_num); i++) {
		if (i == 0) {
			min_value = max_value = buf2[i];

		} else {
			max_value = max(max_value, buf2[i]);
			min_value = min(min_value, buf2[i]);
		}

		difference[i] = buf2[i];
		printk(KERN_INFO " [%d]%3d", i, buf2[i]);
	}
	printk(KERN_INFO "\n");

	snprintf(buff, sizeof(buff), "%d,%d", min_value, max_value);

	set_cmd_result(info, buff, strnlen(buff, sizeof(buff)));
	info->cmd_state = 2;
	dev_info(&info->client->dev, "%s: %s(%d)\n", __func__,
			buff, strnlen(buff, sizeof(buff)));

	/* Exit Inspection Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		/*exit Inspection Mode*/
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	/*mdelay(100);*/
	msleep(100);

	tsp_testmode = 0;

	/*info->cmd_state = 2;*/
}

static void run_intensity_read(void *device_data)
{
	struct touch_data *info = (struct touch_data *)device_data;

	not_support_cmd(info);
}

static ssize_t store_cmd(struct device *dev, struct device_attribute
				  *devattr, const char *buf, size_t count)
{
	struct touch_data *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->client;

	char *cur, *start, *end;
	char buff[TSP_CMD_STR_LEN] = {0};
	int len, i;
	struct tsp_cmd *tsp_cmd_ptr = NULL;
	char delim = ',';
	bool cmd_found = false;
	int param_cnt = 0;
	int ret;

	if (info->cmd_is_running == true) {
		dev_err(&info->client->dev,
			"tsp_cmd: other cmd is running.\n");
		goto err_out;
	}


	/* check lock  */
	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = true;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 1;

	for (i = 0; i < ARRAY_SIZE(info->cmd_param); i++)
		info->cmd_param[i] = 0;

	len = (int)count;
	if (*(buf + len - 1) == '\n')
		len--;
	memset(info->cmd, 0x00, ARRAY_SIZE(info->cmd));
	memcpy(info->cmd, buf, len);

	/*cur = strchr(buf, (int)delim);*/
	cur = strnchr(buf, TSP_BUF_SIZE, (int)delim);
	if (cur)
		memcpy(buff, buf, cur - buf);
	else
		memcpy(buff, buf, len);

	/* find command */
	list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
		if (!strncmp(buff, tsp_cmd_ptr->cmd_name,
		max(MAX_CMD_NAME_STR_LEN, (int)sizeof(buff) - 1))) {
			cmd_found = true;
			break;
		}
	}

	/* set not_support_cmd */
	if (!cmd_found) {
		list_for_each_entry(tsp_cmd_ptr, &info->cmd_list_head, list) {
			if (!strncmp("not_support_cmd", tsp_cmd_ptr->cmd_name,
				MAX_CMD_NAME_STR_LEN))
				break;
		}
	}

	/* parsing parameters */
	if (cur && cmd_found) {
		cur++;
		start = cur;
		memset(buff, 0x00, ARRAY_SIZE(buff));
		do {
			if (*cur == delim || cur - buf == len) {
				end = cur;
				memcpy(buff, start, end - start);
				*(buff + strnlen(buff, sizeof(buff))) = '\0';
				ret = kstrtoint(buff, 10,\
						info->cmd_param + param_cnt);
				start = cur + 1;
				memset(buff, 0x00, ARRAY_SIZE(buff));
				param_cnt++;
			}
			cur++;
		} while (cur - buf <= len);
	}

	dev_info(&client->dev, "cmd = %s\n", tsp_cmd_ptr->cmd_name);
	for (i = 0; i < param_cnt; i++)
		dev_info(&client->dev, "cmd param %d= %d\n", i,
			info->cmd_param[i]);

	tsp_cmd_ptr->cmd_func(info);


err_out:
	return count;
}

static ssize_t show_cmd_status(struct device *dev,
		struct device_attribute *devattr, char *buf)
{
	struct touch_data *info = dev_get_drvdata(dev);
	char buff[16] = {0};

	dev_info(&info->client->dev, "tsp cmd: status:%d\n",
			info->cmd_state);

	if (info->cmd_state == 0)
		snprintf(buff, sizeof(buff), "WAITING");

	else if (info->cmd_state == 1)
		snprintf(buff, sizeof(buff), "RUNNING");

	else if (info->cmd_state == 2)
		snprintf(buff, sizeof(buff), "OK");

	else if (info->cmd_state == 3)
		snprintf(buff, sizeof(buff), "FAIL");

	else if (info->cmd_state == 4)
		snprintf(buff, sizeof(buff), "NOT_APPLICABLE");

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", buff);
}

static ssize_t show_cmd_result(struct device *dev, struct device_attribute
				    *devattr, char *buf)
{
	struct touch_data *info = dev_get_drvdata(dev);

	dev_info(&info->client->dev, "tsp cmd: result: %s\n",
		info->cmd_result);

	mutex_lock(&info->cmd_lock);
	info->cmd_is_running = false;
	mutex_unlock(&info->cmd_lock);

	info->cmd_state = 0;

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", info->cmd_result);
}


static DEVICE_ATTR(close_tsp_test, S_IRUGO, show_close_tsp_test, NULL);
static DEVICE_ATTR(cmd, S_IWUSR | S_IWGRP, NULL, store_cmd);
static DEVICE_ATTR(cmd_status, S_IRUGO, show_cmd_status, NULL);
static DEVICE_ATTR(cmd_result, S_IRUGO, show_cmd_result, NULL);


static struct attribute *sec_touch_facotry_attributes[] = {
		&dev_attr_close_tsp_test.attr,
		&dev_attr_cmd.attr,
		&dev_attr_cmd_status.attr,
		&dev_attr_cmd_result.attr,
	NULL,
};

static struct attribute_group sec_touch_factory_attr_group = {
	.attrs = sec_touch_facotry_attributes,
};
#endif /* SEC_TSP_FACTORY_TEST */


/*extern struct class *sec_class;*/

static int ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct touch_data *ts;

	uint8_t buf_tmp[3] = {0, 0, 0};
	uint8_t addr[1];
	int i;
	int ret = 0, key = 0;

#ifdef SEC_TSP_FACTORY_TEST
	struct device *fac_dev_ts;
#endif


	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);

	/*touch_ctrl_regulator(TOUCH_ON);*/
	/*msleep(100);*/
	touch_ctrl_regulator(TOUCH_OFF);
	msleep(200);
	touch_ctrl_regulator(TOUCH_ON);
	msleep(100);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

#if defined(TSP_EDS_RECOVERY)
	INIT_WORK(&ts->esd_recovery_func, check_ic_work_func);
#endif

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

	tsp_irq = client->irq;

	printk(KERN_INFO "[TSP] tsp_irq = %d\n", tsp_irq);


	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "sec_touchscreen";

	ts->input_dev->keybit[BIT_WORD(KEY_POWER)] |= BIT_MASK(KEY_POWER);

	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	/*set_bit(BTN_TOUCH, ts->input_dev->keybit);*/

	/* for B type multi touch protocol*/
	set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	/*must be added for ICS*/
	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	set_bit(EV_LED, ts->input_dev->evbit);
	set_bit(LED_MISC, ts->input_dev->ledbit);

	/*for B type multi touch protocol*/
	input_mt_init_slots(ts->input_dev, MAX_TOUCH_ID);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

#ifdef __TOUCHKEY__
	for (key = 0; key < MAX_KEYS ; key++)
		input_set_capability(ts->input_dev, EV_KEY,
					touchkey_keycodes[key]);

	/* for TSK*/
	for (key = 0; key < MAX_KEYS ; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;

	for (i = 0; i < MAX_TOUCH_ID; i++) {
		fingerInfo[i].status = 0;
		fingerInfo[i].id = 0;
	}
#endif

	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR
			"ts_probe: Unable to register %s input device\n",
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

	printk(KERN_INFO "[TSP] %s, irq=%d\n", __func__, client->irq);

	if (client->irq) {
		ret = request_threaded_irq(client->irq, NULL, ts_work_func,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts);
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	ts->enabled = true;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = ts_early_suspend;
	ts->early_suspend.resume = ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	printk(KERN_INFO "ts_probe: Start touchscreen %s in %s mode\n",
		ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	/* sys fs */

	sec_touchscreen = device_create(sec_class, NULL, 0,
					ts, "sec_touchscreen");
	if (IS_ERR(sec_touchscreen)) {
		dev_err(&client->dev,
			"Failed to create device for the sysfs1\n");
		ret = -ENODEV;
	}

	if (device_create_file(sec_touchscreen,
		&dev_attr_tsp_firm_version_phone) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_tsp_firm_version_phone.attr.name);
	if (device_create_file(sec_touchscreen,
		&dev_attr_tsp_firm_version_panel) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_tsp_firm_version_panel.attr.name);
	if (device_create_file(sec_touchscreen,
		&dev_attr_tsp_firm_update) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_tsp_firm_update.attr.name);
	if (device_create_file(sec_touchscreen,
		&dev_attr_tsp_firm_update_status) < 0)
		pr_err("[TSP] Failed to create device file(%s)!\n",
			dev_attr_tsp_firm_update_status.attr.name);
	if (device_create_file(sec_touchscreen,
		&dev_attr_tsp_threshold) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_tsp_threshold.attr.name);
	if (device_create_file(sec_touchscreen,
		&dev_attr_get_panel_rev) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_get_panel_rev.attr.name);

	if (device_create_file(sec_touchscreen,
		&dev_attr_tsp_force_calibration) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_tsp_force_calibration.attr.name);

#ifdef __TOUCHKEY__
	sec_touchkey = device_create(sec_class, NULL, 0, ts, "sec_touchkey");
	if (IS_ERR(sec_touchkey)) {
		dev_err(&client->dev,
			"Failed to create device for the sysfs1\n");
		ret = -ENODEV;
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_menu) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_menu.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_back) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_back.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touch_sensitivity) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touch_sensitivity.attr.name);
	if (device_create_file(sec_touchkey, &dev_attr_touchkey_threshold) < 0)
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_threshold.attr.name);
#endif

#ifdef SEC_TSP_FACTORY_TEST
	INIT_LIST_HEAD(&ts->cmd_list_head);
	for (i = 0; i < ARRAY_SIZE(tsp_cmds); i++)
		list_add_tail(&tsp_cmds[i].list, &ts->cmd_list_head);

	mutex_init(&ts->cmd_lock);
	ts->cmd_is_running = false;

	fac_dev_ts = device_create(sec_class,
		NULL, 0, ts, "tsp");
	if (IS_ERR(fac_dev_ts))
		dev_err(&client->dev,
			"Failed to create device for the sysfs\n");

	ret = sysfs_create_group(&fac_dev_ts->kobj,
				&sec_touch_factory_attr_group);
	if (ret)
		dev_err(&client->dev, "Failed to create sysfs group\n");
#endif

	/* Check point - i2c check - start */
	for (i = 0; i < 2; i++) {
		printk(KERN_INFO "[TSP] %s, %d, send\n", __func__, __LINE__);
		addr[0] = 0x1B; /*address*/
		ret = i2c_master_send(ts_global->client, addr, 1);

		if (ret >= 0) {
			printk(KERN_INFO "[TSP] %s, %d, receive\n",
				__func__, __LINE__);
			ret = i2c_master_recv(ts_global->client, buf_tmp, 3);
			if (ret >= 0)
				break; /* i2c success*/
		}

		printk(KERN_ERR "[TSP] %s, %d, fail\n", __func__, __LINE__);
	}

	if (ret >= 0) {
		touch_vendor_id = buf_tmp[0]&0xF0;
		touch_hw_ver = buf_tmp[1];
		touch_sw_ver = buf_tmp[2];
		printk(KERN_INFO "[TSP] %s:%d, ver tsp=%x, HW=%x, SW=%x\n",
			__func__, __LINE__,
			touch_vendor_id, touch_hw_ver, touch_sw_ver);

		ts->fw_ic_ver = touch_sw_ver;

		if (ts->fw_ic_ver < TSP_SW_VER3_L) {
			tsp_testmode = 1;
			/*tsp_special_update:
			0: normal firmware update from phone-binary*/
			tsp_special_update = 0;
			ret = firm_update_callfc();
			tsp_testmode = 0;
		}

#if defined(TSP_EDS_RECOVERY)
		touch_present = 1;
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = watchdog_timer_func;
		hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif
	} else {		/*if(ret < 0)*/
		printk(KERN_ERR "i2c_transfer failed\n");
		printk(KERN_ERR
			"[TSP] %s, ln:%d, Failed to register TSP!!!\n\t"\
			"check the i2c line!!!, ret=%d\n",
			__func__, __LINE__, ret);
		/*goto err_check_functionality_failed;*/
	}
	/* Check point - i2c check - end */

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	kfree(ts);
err_alloc_data_failed:
	return ret;
}

static int ts_remove(struct i2c_client *client)
{
	struct touch_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct touch_data *ts = i2c_get_clientdata(client);

	printk(KERN_INFO "[TSP] %s+\n", __func__);

	tsp_status = 1;

	if (ts->use_irq && ts->enabled) {
		disable_irq(client->irq);
		ts->enabled = false;
	}

#if defined(TSP_EDS_RECOVERY)
	if (touch_present == 1)
		hrtimer_cancel(&ts->timer);
#endif

	gpio_direction_output(TSP_INT , 0);
	gpio_direction_output(TSP_SCL , 0);
	gpio_direction_output(TSP_SDA , 0);

	msleep(20);

	touch_ctrl_regulator(TOUCH_OFF);

/* kwon added 2013-02-22 */
	force_release_all_fingers();

	printk(KERN_INFO "[TSP] %s-\n", __func__);

	return 0;
}

static int ts_resume(struct i2c_client *client)
{
	int ret;
	struct touch_data *ts = i2c_get_clientdata(client);
	uint8_t i2c_addr = 0x1B;
	uint8_t buf[3];

	gpio_direction_output(TSP_SCL , 1);
	gpio_direction_output(TSP_SDA , 1);

	gpio_direction_input(TSP_INT);

	/* kwon added 2013-02-22 */
	force_release_all_fingers();
	touch_ctrl_regulator(TOUCH_ON);

	msleep(100);

	ret = tsp_i2c_read(i2c_addr, buf, sizeof(buf));

	buf[0] = buf[0]&0xF0;

	touch_vendor_id = buf[0];
	touch_hw_ver = buf[1];
	touch_sw_ver = buf[2];

	printk(KERN_INFO "[TSP] %s: ver tsp=%x, HW=%x, SW=%x\n", __func__,
		buf[0], buf[1], buf[2]);
	printk(KERN_INFO "[TSP] %s: ver tsp=%x, HW=%x, SW=%x\n", __func__,
		touch_vendor_id, touch_hw_ver, touch_sw_ver);

	/*enable_irq(client->irq);*/
	if (ts->use_irq && !ts->enabled) {
		enable_irq(client->irq);
		ts->enabled = true;
	}

	tsp_status = 0;

	if (tsp_charger_type_status == 1)
		set_tsp_for_ta_detect(tsp_charger_type_status);

#if defined(TSP_EDS_RECOVERY)
	prev_wdog_val = -1;

	if (touch_present == 1)
		hrtimer_start(&ts->timer, ktime_set(3, 0), HRTIMER_MODE_REL);
#endif

	printk(KERN_INFO "[TSP] %s-\n", __func__);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ts_early_suspend(struct early_suspend *h)
{
	struct touch_data *ts;
	ts = container_of(h, struct touch_data, early_suspend);
	ts_suspend(ts->client, PMSG_SUSPEND);
}

static void ts_late_resume(struct early_suspend *h)
{
	struct touch_data *ts;
	ts = container_of(h, struct touch_data, early_suspend);
	ts_resume(ts->client);
}
#endif

static const struct i2c_device_id ts_id[] = {
	{ "cypress-tma140", 0 },
	{ }
};

static struct i2c_driver ts_driver = {
	.probe		= ts_probe,
	.remove		= ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= ts_suspend,
	.resume		= ts_resume,
#endif
	.id_table	= ts_id,
	.driver = {
		.name	= "cypress-tma140",
	},
};

static int __devinit tsp_driver_init(void)
{

	printk(KERN_INFO "[TSP] %s\n", __func__);

	gpio_request(TSP_INT, "ts_irq");
	gpio_direction_input(TSP_INT);

	/*irq_set_irq_type(gpio_to_irq(TSP_INT), IRQ_TYPE_EDGE_FALLING);*/

	gpio_direction_output(TSP_SCL , 1);
	gpio_direction_output(TSP_SDA , 1);

#if defined(TSP_EDS_RECOVERY)
	check_ic_wq = create_singlethread_workqueue("check_ic_wq");

	if (!check_ic_wq)
		return -ENOMEM;
#endif

#if defined(__TOUCH_KEYLED__)
	touchkeyled_regulator = regulator_get(NULL, "touch_keyled");
#endif
	return i2c_add_driver(&ts_driver);
}

static void __exit tsp_driver_exit(void)
{
	if (touch_regulator) {
		regulator_put(touch_regulator);
		touch_regulator = NULL;
	}
#if defined(__TOUCH_KEYLED__)
	if (touchkeyled_regulator) {
		regulator_put(touchkeyled_regulator);
		touchkeyled_regulator = NULL;
	}
#endif
	i2c_del_driver(&ts_driver);

#if defined(TSP_EDS_RECOVERY)
	if (check_ic_wq)
		destroy_workqueue(check_ic_wq);
#endif

}

static ssize_t read_threshold(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t buf1[2] = {0,};
	uint8_t buf2[1] = {0,};
	int i, ret;

	uint8_t i2c_addr;

	tsp_testmode = 1;

	/* Enter System Information Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x10;/*value*/
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}
	msleep(50);

	/*  Read Threshold Value */
	i2c_addr = 0x30;
	tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));

	printk(KERN_INFO " [TSP] %d", buf2[0]);

	/* Exit System Information Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {
		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		/*exit Inspection Mode*/
		ret = i2c_master_send(ts_global->client, buf1, 2);

		if (ret >= 0)
			break; /* i2c success*/
	}

	msleep(100);

	tsp_testmode = 0;

	return snprintf(buf, TSP_BUF_SIZE, "%d\n",  buf2[0]);
}


static ssize_t firmware_In_Binary(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	int phone_ver = 0;

	return snprintf(buf, TSP_BUF_SIZE,  "%d\n", phone_ver);
}


static ssize_t firmware_In_TSP(struct device *dev,
	struct device_attribute *attr, char *buf)
{

	uint8_t i2c_addr = 0x1B;
	uint8_t buf_tmp[3] = {0};

	printk(KERN_INFO "[TSP] %s\n", __func__);

	tsp_i2c_read(i2c_addr, buf_tmp, sizeof(buf_tmp));

	touch_vendor_id = buf_tmp[0] & 0xF0;
	touch_hw_ver = buf_tmp[1];
	touch_sw_ver = buf_tmp[2];
	printk(KERN_INFO "[TSP] %s:%d, ver tsp=%x, HW=%x, SW=%x\n",
		__func__, __LINE__,
		touch_vendor_id, touch_hw_ver, touch_sw_ver);

	return snprintf(buf, TSP_BUF_SIZE,  "%d\n", touch_sw_ver);
}


static ssize_t tsp_panel_rev(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	uint8_t i2c_addr = 0x1B;
	uint8_t buf_tmp[3] = {0};

	printk(KERN_INFO "[TSP] %s\n", __func__);

	tsp_i2c_read(i2c_addr, buf_tmp, sizeof(buf_tmp));

	touch_vendor_id = buf_tmp[0] & 0xF0;
	touch_hw_ver = buf_tmp[1];
	touch_sw_ver = buf_tmp[2];
	printk(KERN_INFO "[TSP] %s:%d, ver tsp=%x, HW=%x, SW=%x\n",
		__func__, __LINE__,
		touch_vendor_id, touch_hw_ver, touch_sw_ver);

	return snprintf(buf, TSP_BUF_SIZE, "%d\n", touch_hw_ver);
}

#ifdef __TOUCHKEY__
static ssize_t touchkey_sensitivity_power_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int ret = 0;
	uint8_t buf1[2] = {0,};
	int i = 0;

	if (touchkey_scan_mode == 0) {
		printk(KERN_INFO
			"[TSP] %s, touchkey_sensitivity_power ON !!!\n",
			__func__);

		tsp_testmode = 1;

		/* Enter System Information Mode */
		for (i = 0; i < I2C_RETRY_CNT; i++) {
			buf1[0] = 0x00;/*address*/
			buf1[1] = 0x10;/*value*/
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0)
				break; /* i2c success*/
		}
		msleep(200);

		/*  set CMD */
		for (i = 0; i < I2C_RETRY_CNT; i++) {
			buf1[0] = 0x01;/*address*/
			buf1[1] = 0x1F;/*value*/
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0)
				break; /*i2c success*/
		}
		msleep(200);

		/* Exit System Information Mode */
		for (i = 0; i < I2C_RETRY_CNT; i++) {
			buf1[0] = 0x00;/*address*/
			buf1[1] = 0x00;/*value*/

			/*exit Inspection Mode*/
			ret = i2c_master_send(ts_global->client, buf1, 2);

			if (ret >= 0)
				break; /* i2c success*/
		}

		msleep(1125);

		touchkey_scan_mode = 1;
		tsp_testmode = 0;
	} else if (touchkey_scan_mode == 1) {
		printk(KERN_INFO
			"[TSP] %s, touchkey_sensitivity_power OFF !!!\n",
			__func__);

		disable_irq(ts_global->client->irq);
		force_release_key();
		force_release_all_fingers();
		tsp_reset();
		enable_irq(ts_global->client->irq);

		touchkey_scan_mode = 0;
		tsp_testmode = 0;
	}

	/*return snprintf(buf, TSP_BUF_SIZE,  "%d\n",  touchkey_scan_mode);*/
	return size;
}

static ssize_t read_touchkey_threshold(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return snprintf(buf, TSP_BUF_SIZE, "%d\n",  touchkey_threshold_value);
}


static ssize_t menu_sensitivity_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u16 local_menu_sensitivity;
	uint8_t i2c_addr;
	uint8_t buf2[1] = {0,};

	tsp_testmode = 1;
	printk(KERN_INFO "[TSP] %s, menu_sensitivity start !!!\n" , __func__);

	i2c_addr = 0x17;
	tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));

	printk(KERN_INFO "%s, menu_sensitivity : %d %x!!!\n",
		__func__, buf2[0], buf2[0]);
	local_menu_sensitivity = buf2[0];

	tsp_testmode = 0;
	return snprintf(buf, TSP_BUF_SIZE, "%d\n",  local_menu_sensitivity);
}

static ssize_t back_sensitivity_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	u16 local_back_sensitivity;
	uint8_t i2c_addr;
	uint8_t buf2[1] = {0,};

	tsp_testmode = 1;
	printk(KERN_INFO "[TSP] %s, back_sensitivity start !!!\n", __func__);

	i2c_addr = 0x18;
	tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));

	printk(KERN_INFO "%s, back_sensitivity : %d %x!!!\n",
		__func__, buf2[0], buf2[0]);
	local_back_sensitivity = buf2[0];

	tsp_testmode = 0;
	return snprintf(buf, TSP_BUF_SIZE, "%d\n",  local_back_sensitivity);
}
#endif


#define TMA140_RET_SUCCESS 0x00
int sv_tch_firmware_update;
EXPORT_SYMBOL(sv_tch_firmware_update);


static int firm_update_callfc(void)
{
	uint8_t update_num;

	disable_irq(tsp_irq);
	local_irq_disable();
	printk(KERN_INFO "[TSP] disable_irq : %d\n", __LINE__);

	for (update_num = 1; update_num <= 5 ; update_num++) {
		sv_tch_firmware_update = cypress_update(touch_hw_ver);

		if (sv_tch_firmware_update == TMA140_RET_SUCCESS) {
			firmware_ret_val = 1; /*SUCCESS */
			snprintf(IsfwUpdate, sizeof(IsfwUpdate), "%s",
					FW_DOWNLOAD_COMPLETE);
			printk(KERN_INFO
				"[TSP] %s, %d : firmware update SUCCESS !!\n",
				__func__, __LINE__);
			break;
		} else {
			printk(KERN_INFO
				"[TSP] %s, %d : firmware update RETRY !!\n",
				__func__, __LINE__);
			if (update_num == 5) {
				firmware_ret_val = -1; /*FAIL*/
			snprintf(IsfwUpdate, sizeof(IsfwUpdate), "%s",
						FW_DOWNLOAD_FAIL);
			printk(KERN_ERR
				"[TSP] %s, %d : firmware update FAIL !!\n",
				__func__, __LINE__);
			}
		}
	}

	printk(KERN_INFO "[TSP] enable_irq : %d\n", __LINE__);
	local_irq_enable();
	enable_irq(tsp_irq);

	return firmware_ret_val;
}


static ssize_t firm_update(struct device *dev, struct device_attribute *attr,
			char *buf)
{

	printk(KERN_INFO "[TSP] %s!\n", __func__);

	snprintf(IsfwUpdate, sizeof(IsfwUpdate), "%s", FW_DOWNLOADING);

	now_tspfw_update = 1;

	now_tspfw_update = 0;

	return snprintf(buf, TSP_BUF_SIZE, "%d", firmware_ret_val);
}


static ssize_t firmware_update_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[TSP] %s\n", __func__);

	return snprintf(buf, TSP_BUF_SIZE, "%s\n", IsfwUpdate);
}

/*static void force_calibration(void)*/
static ssize_t force_calibration_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	uint8_t buf1[2] = {0,};
	uint8_t buf2[1] = {0,};
	int i = 0;
	int ret = 0;
	uint8_t i2c_addr;

	tsp_testmode = 1;
	printk(KERN_INFO "[TSP] %s\n", __func__);

	/* Sensitivity Value */

	/* Enter System Information Mode */

	for (i = 0; i < I2C_RETRY_CNT; i++) {

		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x10;/*value*/

		ret = i2c_master_send(ts_global->client, buf1, 2);
		if (ret >= 0)
			break; /* i2c success*/
	}

	msleep(50);

	/* force calibration command */
	for (i = 0; i < I2C_RETRY_CNT; i++) {

		buf1[0] = 0x01;/*address*/
		buf1[1] = 0x20;/*value*/

		ret = i2c_master_send(ts_global->client, buf1, 2);
		if (ret >= 0)
			break; /* i2c success*/
	}

	msleep(480);

	/*check the calibration command*/
	/*calibration time :75*20ms again, max time: 1500ms*/
	for (i = 0; i < 75; i++) {

		i2c_addr = 0x01;
		tsp_i2c_read(i2c_addr, buf2, sizeof(buf2));
		printk(KERN_INFO "[TSP]%s, ACK%d:%d", __func__, i, buf2[0]);

		if (0x01&buf2[0])
			break;

		msleep(20);
	}

	/* Enter Normal Mode */
	for (i = 0; i < I2C_RETRY_CNT; i++) {

		buf1[0] = 0x00;/*address*/
		buf1[1] = 0x00;/*value*/

		ret = i2c_master_send(ts_global->client, buf1, 2);
		if (ret >= 0)
			break; /* i2c success*/
	}

	msleep(50);
	tsp_testmode = 0;

	return size;
}
module_init(tsp_driver_init);
module_exit(tsp_driver_exit);

MODULE_AUTHOR("Cypress");
MODULE_DESCRIPTION("TMA140 Touchscreen Driver");
MODULE_LICENSE("GPL");
