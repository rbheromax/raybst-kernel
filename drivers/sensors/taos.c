#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <mach/gpio.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/wakelock.h>
#include <linux/i2c/taos.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <mach/vreg.h>
#include <linux/sensors_core.h>
#define OFFSET_FILE_PATH	"/efs/prox_cal"
#define OFFSET_ARRAY_LENGTH		10
#define TAOS_CHIP_DEV_NAME		"TMD27723"
#define TAOS_CHIP_DEV_VENDOR	"TAOS"
#define TAOS_CHIP_DEV_ID		0x39

/* Triton register offsets */
#define CNTRL					0x00
#define ALS_TIME				0X01
#define PRX_TIME				0x02
#define WAIT_TIME				0x03
#define ALS_MINTHRESHLO			0X04
#define ALS_MINTHRESHHI			0X05
#define ALS_MAXTHRESHLO			0X06
#define ALS_MAXTHRESHHI			0X07
#define PRX_MINTHRESHLO			0X08
#define PRX_MINTHRESHHI			0X09
#define PRX_MAXTHRESHLO			0X0A
#define PRX_MAXTHRESHHI			0X0B
#define INTERRUPT			0x0C
#define PRX_CFG				0x0D
#define PRX_COUNT			0x0E
#define GAIN				0x0F
#define REVID				0x11
#define CHIPID				0x12
#define STATUS				0x13
#define ALS_CHAN0LO			0x14
#define ALS_CHAN0HI			0x15
#define ALS_CHAN1LO			0x16
#define ALS_CHAN1HI			0x17
#define PRX_LO				0x18
#define PRX_HI				0x19
#define PRX_OFFSET			0x1E
#define TEST_STATUS			0x1F

/* Triton cmd reg masks */
#define CMD_REG				0X80
#define CMD_BYTE_RW			0x00
#define CMD_WORD_BLK_RW		0x20
#define CMD_SPL_FN			0x60
#define CMD_PROX_INTCLR		0X05
#define CMD_ALS_INTCLR		0X06
#define CMD_PROXALS_INTCLR	0X07
#define CMD_TST_REG			0X08
#define CMD_USER_REG		0X09

/* Triton cntrl reg masks */
#define CNTL_REG_CLEAR			0x00
#define CNTL_PROX_INT_ENBL		0X20
#define CNTL_ALS_INT_ENBL		0X10
#define CNTL_WAIT_TMR_ENBL		0X08
#define CNTL_PROX_DET_ENBL		0X04
#define CNTL_ADC_ENBL			0x02
#define CNTL_PWRON				0x01
#define CNTL_ALSPON_ENBL		0x03
#define CNTL_INTALSPON_ENBL		0x13
#define CNTL_PROXPON_ENBL		0x0F
#define CNTL_INTPROXPON_ENBL	0x2F

/* Triton status reg masks */
#define STA_ADCVALID			0x01
#define STA_PRXVALID			0x02
#define STA_ADC_PRX_VALID		0x03
#define STA_ADCINTR				0x10
#define STA_PRXINTR				0x20

/* Lux constants */
#define SCALE_MILLILUX			3
#define FILTER_DEPTH			3
#define	GAINFACTOR				9

/* Thresholds */
#define ALS_THRESHOLD_LO_LIMIT		0x0000
#define ALS_THRESHOLD_HI_LIMIT		0xFFFF
#define PROX_THRESHOLD_LO_LIMIT		0x0000
#define PROX_THRESHOLD_HI_LIMIT		0xFFFF

/* Device default configuration */
#define CALIB_TGT_PARAM			300000
#define SCALE_FACTOR_PARAM		1
#define GAIN_TRIM_PARAM			512
#define GAIN_PARAM				1
#define ALS_THRSH_HI_PARAM		0xFFFF
#define ALS_THRSH_LO_PARAM		0
#define PRX_ADC_TIME_PARAM		0xFF	/* [HSS]Original value:0XEE */
#define PRX_WAIT_TIME_PARAM		0xFF	/* 2.73ms */
#define PRX_CONFIG_PARAM		0x00
#define PRX_GAIN_PARAM			0x28	/* 21 */

static struct device *prox_sys_device;
static struct device *light_sys_device;
static void set_prox_offset(struct taos_data *taos, u8 offset);
static int proximity_open_offset(struct taos_data *data);
/*  i2c write routine for taos */
static int opt_i2c_write(struct taos_data *taos, u8 reg, u8 * val)
{
	int err;
	struct i2c_msg msg[1];
	unsigned char data[2];
	if (taos->opt_i2c_client == NULL)
		return -ENODEV;
	data[0] = reg;
	data[1] = *val;
	msg->addr = taos->opt_i2c_client->addr;
	msg->flags = I2C_M_WR;
	msg->len = 2;
	msg->buf = data;
	err = i2c_transfer(taos->opt_i2c_client->adapter, msg, 1);
	if (err >= 0)
		return 0;
	printk(KERN_INFO
		"[TAOS] %s %d i2c transfer error : reg = [%X], err is %d\n",
		 __func__, __LINE__, reg, err);
	return err;
}



/*************************************************************************/
/*		TAOS sysfs					         */
/*************************************************************************/
static void taos_light_enable(struct taos_data *taos)
{
	hrtimer_start(&taos->timer, taos->light_polling_time,
		       HRTIMER_MODE_REL);
}

static void
taos_light_disable(struct taos_data *taos)
{
	hrtimer_cancel(&taos->timer);
	cancel_work_sync(&taos->work_light);
}

static ssize_t
light_delay_show(struct device *dev, struct device_attribute *attr,
		 char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%lld\n",
		ktime_to_ns(taos->light_polling_time));
}

static ssize_t
light_delay_store(struct device *dev, struct device_attribute *attr,
		  const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int64_t new_delay;
	int err;

	err = strict_strtoll(buf, 10, &new_delay);
	if (err < 0)
		return err;
	mutex_lock(&taos->power_lock);
	if (new_delay != ktime_to_ns(taos->light_polling_time)) {
		taos->light_polling_time = ns_to_ktime(new_delay);
		if (taos->light_enable) {
			taos_light_disable(taos);
			taos_light_enable(taos);
		}
	}
	mutex_unlock(&taos->power_lock);
	return size;
}

static ssize_t
light_enable_show(struct device *dev, struct device_attribute *attr,
		  char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n", (taos->light_enable) ? 1 : 0);
}
static ssize_t
light_raw_data_show(struct device *dev, struct device_attribute *attr,
		  char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d, %d\n",
		taos->cleardata, taos->irdata);
}
static ssize_t
proximity_enable_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d\n",
		(taos->proximity_enable) ? 1 : 0);
}

static ssize_t
light_enable_store(struct device *dev, struct device_attribute *attr,
		   const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int value = 0, err = 0;
	err = kstrtoint(buf, 10, &value);
	if (err < 0) {
		pr_err("%s, kstrtoint failed.", __func__);
	} else {
		pr_info("[TAOS] %s, %d\n", __func__, value);
		if (value == 1 && taos->light_enable == OFF) {
			if (taos->pdata->power_en)
				taos->pdata->power_en(1);
			taos_on(taos, TAOS_LIGHT);
			taos->light_enable = ON;
		} else if (value == 0 && taos->light_enable == ON) {
			taos_off(taos, TAOS_LIGHT);
			if (taos->pdata->power_en)
				taos->pdata->power_en(0);
			taos->light_enable = OFF;
		}
	}

	return size;
}

static ssize_t
proximity_enable_store(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int value = 0, ret = 0;
	ret = kstrtoint(buf, 10, &value);
	if (ret < 0) {
		pr_err("%s, kstrtoint failed.", __func__);
	} else {
		if (value == 1 && taos->proximity_enable == OFF) {
			if (taos->pdata->power_en)
				taos->pdata->power_en(1);
			if (taos->pdata->power_led)
				taos->pdata->power_led(1);
			/* reset Interrupt pin */
			/* to active Interrupt, TMD2771x Interuupt pin
			 * shoud be reset. */
			ret = proximity_open_offset(taos);
			if (ret < 0 && ret != -ENOENT)
				pr_err("%s: proximity_open_offset() failed\n",
					__func__);
			if (taos->offset_value != taos->initial_offset
					&& taos->cal_result == 1) {
				taos->threshold_high =
					taos->pdata->prx_thrsh_hi_calparam;
				taos->threshold_low =
					taos->pdata->prx_thrsh_lo_calparam;
			}

			pr_err("%s: hi= %d, low= %d, off= %d\n", __func__,
				taos->threshold_high, taos->threshold_low,
				taos->offset_value);
			i2c_smbus_write_byte(taos->opt_i2c_client,
						 (CMD_REG | CMD_SPL_FN |
						  CMD_PROXALS_INTCLR));
			taos_on(taos, TAOS_PROXIMITY);
			taos->proximity_enable = ON;
			input_report_abs(taos->prox_input_dev, ABS_DISTANCE,
					  !(taos->proximity_value));
			input_sync(taos->prox_input_dev);
			printk(KERN_INFO
				"[TAOS_PROXIMITY] Temporary : Power ON, chip ID = %X\n",
				taos->chipID);
		} else if (value == 0 && taos->proximity_enable == ON) {
			taos_off(taos, TAOS_PROXIMITY);
			if (taos->pdata->power_en)
				taos->pdata->power_en(0);
			if (taos->pdata->power_led)
				taos->pdata->power_led(0);
			taos->proximity_enable = OFF;
			printk(KERN_INFO
				"[TAOS_PROXIMITY] Temporary : Power OFF\n");
		}
	}
	return size;
}

static int taos_get_initial_offset(struct taos_data *taos)
{
	int ret = 0;

	ret = i2c_smbus_read_word_data(taos->opt_i2c_client,
		(CMD_REG | PRX_OFFSET));
	pr_err("%s: initial offset = %d\n", __func__, ret);

	return ret;
}

static void set_prox_offset(struct taos_data *taos, u8 offset)
{
	int ret = 0;

	ret = opt_i2c_write(taos, (CMD_REG|PRX_OFFSET), &offset);
	if (ret < 0)
		pr_info("%s: opt_i2c_write to prx offset reg failed\n"
			, __func__);
}

static void taos_thresh_set(struct taos_data *taos)
{
	int i = 0;
	int ret = 0;
	u8 prox_int_thresh[4];

	/* Setting for proximity interrupt */
	if (taos->proximity_value == 1) {
		prox_int_thresh[0] = (taos->threshold_low) & 0xFF;
		prox_int_thresh[1] = (taos->threshold_low >> 8) & 0xFF;
		prox_int_thresh[2] = (0xFFFF) & 0xFF;
		prox_int_thresh[3] = (0xFFFF >> 8) & 0xFF;
	} else if (taos->proximity_value == 0) {
		prox_int_thresh[0] = (0x0000) & 0xFF;
		prox_int_thresh[1] = (0x0000 >> 8) & 0xFF;
		prox_int_thresh[2] = (taos->threshold_high) & 0xff;
		prox_int_thresh[3] = (taos->threshold_high >> 8) & 0xff;
	}

	for (i = 0; i < 4; i++) {
		ret = opt_i2c_write(taos,
			(CMD_REG|(PRX_MINTHRESHLO + i)),
			&prox_int_thresh[i]);
		if (ret < 0)
			pr_info("%s: opt_i2c_write failed, err = %d\n"
				, __func__, ret);
	}
}

static int proximity_adc_read(struct taos_data *taos)
{
	int sum[OFFSET_ARRAY_LENGTH];
	int i = 0;
	int avg = 0;
	int min = 0;
	int max = 0;
	int total = 0;

	mutex_lock(&taos->prox_mutex);
	for (i = 0; i < OFFSET_ARRAY_LENGTH; i++) {
		usleep_range(11000, 11000);
		sum[i] = i2c_smbus_read_word_data(taos->opt_i2c_client,
			CMD_REG | PRX_LO);
		if (i == 0) {
			min = sum[i];
			max = sum[i];
		} else {
			if (sum[i] < min)
				min = sum[i];
			else if (sum[i] > max)
				max = sum[i];
		}
		total += sum[i];
	}
	mutex_unlock(&taos->prox_mutex);
	total -= (min + max);
	avg = (int)(total / (OFFSET_ARRAY_LENGTH - 2));

	return avg;
}


static int proximity_open_offset(struct taos_data *data)
{
	struct file *offset_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	offset_filp = filp_open(OFFSET_FILE_PATH, O_CREAT | O_RDONLY | O_SYNC, 0666);
	if (IS_ERR(offset_filp)) {
		pr_err("%s: no offset file\n", __func__);
		err = PTR_ERR(offset_filp);
		if (err != -ENOENT)
			pr_err("%s: Can't open cancelation file\n", __func__);
		set_fs(old_fs);
#ifdef CONFIG_MACH_RAY
		set_prox_offset(data, data->offset_value);
#endif
		return err;
	}

	err = offset_filp->f_op->read(offset_filp,
		(char *)&data->offset_value, sizeof(u8), &offset_filp->f_pos);
	if (err != sizeof(u8)) {
		pr_err("%s: Can't read the cancel data from file\n", __func__);
		err = -EIO;
	}
	if (data->offset_value < 121 &&
		data->offset_value != data->initial_offset) {
		data->cal_result = 1;
	}
	set_prox_offset(data, data->offset_value);

	filp_close(offset_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int proximity_store_offset(struct device *dev, bool do_calib)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	struct file *offset_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;
	int adc = 0;
	int target_xtalk = 150;
	int offset_change = 0x20;
	u8 reg_cntrl = 0x2D;

	pr_err("%s: return %d\n", __func__, err);
	if (do_calib) {
		/* tap offset button */
		pr_err("%s offset\n", __func__);

		err = opt_i2c_write(taos, (CMD_REG|CNTRL), &reg_cntrl);
		if (err < 0)
			pr_info("%s: opt_i2c_write to ctrl reg failed\n",
				__func__);
		usleep_range(12000, 15000);
		adc = proximity_adc_read(taos);
		if (adc > 250) {
			taos->offset_value = 0x3F;
			do {
				set_prox_offset(taos, taos->offset_value);
				adc = proximity_adc_read(taos);
				pr_err("%s: adc = %d, P_OFFSET = %d, change = %d\n",
					__func__, adc,
					taos->offset_value, offset_change);
				if (adc > target_xtalk)
					taos->offset_value += offset_change;
				else if (adc < target_xtalk)
					taos->offset_value -= offset_change;
				else
					break;
				offset_change /= 2;
			} while (offset_change > 0);

			if (taos->offset_value >= 121 &&
				taos->offset_value < 128) {
				taos->cal_result = 0;
				pr_err("%s: cal fail - return\n", __func__);
			} else {
				if (taos->offset_value == taos->initial_offset)
					taos->cal_result = 2;
				else
					taos->cal_result = 1;
			}
		} else {
			taos->cal_result = 2;
		}

		pr_err("%s: P_OFFSET = %d\n", __func__, taos->offset_value);
	} else {
		pr_err("%s reset\n", __func__);
		taos->offset_value = taos->initial_offset;
		taos->cal_result = 2;
	}
	pr_err("%s: prox_offset = %d\n", __func__, taos->offset_value);
	if (taos->offset_value == taos->initial_offset
			|| taos->cal_result == 0) {
		taos->threshold_high = taos->pdata->prx_thrsh_hi_param;
		taos->threshold_low = taos->pdata->prx_thrsh_lo_param;
	} else {
		taos->threshold_high = taos->pdata->prx_thrsh_hi_calparam;
		taos->threshold_low = taos->pdata->prx_thrsh_lo_calparam;
	}
	taos_thresh_set(taos);
	set_prox_offset(taos, taos->offset_value);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	offset_filp = filp_open(OFFSET_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY | O_SYNC, 0666);
	if (IS_ERR(offset_filp)) {
		pr_err("%s: Can't open prox_offset file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(offset_filp);
		return err;
	}

	err = offset_filp->f_op->write(offset_filp,
		(char *)&taos->offset_value, sizeof(u8), &offset_filp->f_pos);
	if (err != sizeof(u8)) {
		pr_err("%s: Can't write the offset data to file\n", __func__);
		err = -EIO;
	}

	filp_close(offset_filp, current->files);
	set_fs(old_fs);
	pr_err("%s: return %d\n", __func__, err);
	return err;
}

static ssize_t proximity_cal_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t size)
{
	bool do_calib;
	int err;
	if (sysfs_streq(buf, "1")) { /* calibrate cancelation value */
		do_calib = true;
	} else if (sysfs_streq(buf, "0")) { /* reset cancelation value */
		do_calib = false;
	} else {
		pr_err("%s: invalid value %d\n", __func__, *buf);
		return -EINVAL;
	}

	err = proximity_store_offset(dev, do_calib);
	if (err < 0) {
		pr_err("%s: proximity_store_offset() failed\n", __func__);
		return err;
	}

	return size;
}

static ssize_t prox_offset_pass_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", taos->cal_result);
}

static ssize_t proximity_cal_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int p_offset = 0;

	msleep(20);
	p_offset = i2c_smbus_read_byte_data(taos->opt_i2c_client,
		CMD_REG | PRX_OFFSET);

	return sprintf(buf, "%d,%d\n", p_offset,
		taos->threshold_high);
}

static ssize_t proximity_thresh_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int thresh_hi = 0;

	msleep(20);
	thresh_hi = i2c_smbus_read_word_data(taos->opt_i2c_client,
		(CMD_REG | PRX_MAXTHRESHLO));

	pr_err("%s: THRESHOLD = %d\n", __func__, thresh_hi);

	return sprintf(buf, "prox_threshold = %d\n", thresh_hi);
}

static ssize_t proximity_thresh_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int thresh_value = 0;
	int err = 0;

	err = kstrtoint(buf, 10, &thresh_value);
	if (err < 0) {
		pr_err("%s, kstrtoint failed.", __func__);
	} else {
		taos->threshold_high = thresh_value;
		taos_thresh_set(taos);
		msleep(20);
	}
	return size;
}

static ssize_t
proximity_adc_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	u16 proximity_value = 0;
	proximity_value =
	    i2c_smbus_read_word_data(taos->opt_i2c_client, CMD_REG | PRX_LO);
	if (proximity_value > TAOS_PROX_MAX)
		proximity_value = TAOS_PROX_MAX;

	return snprintf(buf, PAGE_SIZE, "%d\n", proximity_value);
}
static ssize_t
proximity_state_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	u16 proximity_value = 0;
	proximity_value =
	    i2c_smbus_read_word_data(taos->opt_i2c_client, CMD_REG | PRX_LO);
	if (proximity_value > TAOS_PROX_MAX)
		proximity_value = TAOS_PROX_MAX;

	return snprintf(buf, PAGE_SIZE, "%d\n", proximity_value);
}
static ssize_t
proximity_avg_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", taos->avg[0],
		taos->avg[1], taos->avg[2]);
}

static ssize_t
proximity_avg_store(struct device *dev, struct device_attribute *attr,
		    const char *buf, size_t size)
{
	struct taos_data *taos = dev_get_drvdata(dev);
	int enable = 0;
	int err = 0;

	err = kstrtoint(buf, 10, &enable);
	if (err < 0) {
		pr_err("%s, kstrtoint failed.", __func__);
	} else {
		pr_info("[TAOS] %s, %d\n", __func__, enable);
		if (enable)
			hrtimer_start(&taos->ptimer, taos->prox_polling_time,
		       HRTIMER_MODE_REL);
		else
			hrtimer_cancel(&taos->ptimer);
	}
	return size;
}
static ssize_t
prox_name_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", TAOS_CHIP_DEV_NAME);
}
static ssize_t
prox_vendor_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", TAOS_CHIP_DEV_VENDOR);
}
static ssize_t
light_name_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", TAOS_CHIP_DEV_NAME);
}
static ssize_t
light_vendor_show(struct device *dev, struct device_attribute *attr,
		   char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", TAOS_CHIP_DEV_VENDOR);
}
static DEVICE_ATTR(prox_cal, S_IRUGO | S_IWUSR, proximity_cal_show,
	proximity_cal_store);
static DEVICE_ATTR(name, 0440, prox_name_show, NULL);
static DEVICE_ATTR(vendor, 0440, prox_vendor_show, NULL);
static DEVICE_ATTR(delay, S_IRUGO | S_IWUSR | S_IWGRP, light_delay_show,
	light_delay_store);
static DEVICE_ATTR(state, 0644, proximity_state_show, NULL);
static DEVICE_ATTR(raw_data, 0644, proximity_adc_show, NULL);
static DEVICE_ATTR(prox_avg, 0644, proximity_avg_show,
	proximity_avg_store);
static DEVICE_ATTR(prox_offset_pass, S_IRUGO|S_IWUSR,
	prox_offset_pass_show, NULL);
static DEVICE_ATTR(prox_thresh, 0644, proximity_thresh_show,
	proximity_thresh_store);
static struct device_attribute dev_attr_proximity_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	proximity_enable_show, proximity_enable_store);
static struct device_attribute dev_attr_light_enable =
	__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	light_enable_show, light_enable_store);
static struct device_attribute dev_attr_light_raw_data =
	__ATTR(raw_data, S_IRUGO | S_IWUSR | S_IWGRP,
	light_raw_data_show, NULL);
static struct device_attribute dev_attr_light_name =
	__ATTR(name, S_IRUGO | S_IWUSR | S_IWGRP,
	light_name_show, NULL);
static struct device_attribute dev_attr_light_vendor =
	__ATTR(vendor, S_IRUGO | S_IWUSR | S_IWGRP,
	light_vendor_show, NULL);

static struct attribute *light_sysfs_attrs[] = {
	&dev_attr_light_enable.attr,
	&dev_attr_delay.attr,
	NULL
};

static struct attribute_group light_attribute_group = { .attrs =
	    light_sysfs_attrs,
};

static struct attribute *proximity_sysfs_attrs[] = {
	&dev_attr_proximity_enable.attr,
	NULL
};

static struct attribute_group proximity_attribute_group = { .attrs =
	    proximity_sysfs_attrs,
};

static struct device_attribute *prox_attrs[] = {
	&dev_attr_raw_data,
	&dev_attr_state,
	&dev_attr_vendor,
	&dev_attr_name,
	&dev_attr_prox_avg,
	&dev_attr_prox_cal,
	&dev_attr_prox_offset_pass,
	&dev_attr_prox_thresh,
	NULL,
};
static struct device_attribute *light_attrs[] = {
	&dev_attr_light_raw_data,
	&dev_attr_light_vendor,
	&dev_attr_light_name,
	NULL,
};
/*****************************************************************************
 *
 *  function    : taos_work_func_prox
 *  description : This function is for proximity sensor (using B-1 Mode ).
 *                when INT signal is occured , it gets value from VO register.
 *
 *
 */
static void taos_work_func_prox(struct work_struct *work)
{
	struct taos_data *taos =
	    container_of(work, struct taos_data, work_prox);
	u16 adc_data;
	u16 threshold_high;
	u16 threshold_low;

	/* change Threshold */
	adc_data =
	    i2c_smbus_read_word_data(taos->opt_i2c_client, CMD_REG | 0x18);
	threshold_high =
	    i2c_smbus_read_word_data(taos->opt_i2c_client,
				     (CMD_REG | PRX_MAXTHRESHLO));
	threshold_low =
	    i2c_smbus_read_word_data(taos->opt_i2c_client,
				     (CMD_REG | PRX_MINTHRESHLO));
	printk(KERN_INFO
		"[PROXIMITY] %s %d, high(%x), low(%x)\n", __func__,
		adc_data, threshold_high, threshold_low);

	if (taos_get_lux(taos) >= 1500) {
		/* this is protection code for saturation */
		taos->proximity_value = 0;
		/* end of protection code for saturation*/
	} else if ((threshold_high == (taos->threshold_high))
		 && (adc_data >= (taos->threshold_high))) {
		taos->proximity_value = 1;
	} else if ((threshold_high == (0xFFFF))
		 && (adc_data <= (taos->threshold_low))) {
		taos->proximity_value = 0;
	} else {
		taos->proximity_value = 0;
		printk(KERN_INFO
			"[PROXIMITY] [%s] Error! Not Common Case!adc_data=[%X],"
			"threshold_high=[%X],  threshold_min=[%X]\n",
		     __func__, adc_data, threshold_high, threshold_low);
	}
	taos_thresh_set(taos);
	input_report_abs(taos->prox_input_dev, ABS_DISTANCE,
			   !taos->proximity_value);
	input_sync(taos->prox_input_dev);
	msleep(20);

	/* reset Interrupt pin */
	/* to active Interrupt, TMD2771x Interuupt pin shoud be reset. */
	i2c_smbus_write_byte(taos->opt_i2c_client,
			 (CMD_REG | CMD_SPL_FN | CMD_PROXALS_INTCLR));

	/* enable INT */
	enable_irq(taos->irq);
}

static irqreturn_t taos_irq_handler(int irq, void *dev_id)
{
	struct taos_data *taos = dev_id;
	if (taos->irq != -1) {
		pr_info("[TAOS] %s\n", __func__);
		wake_lock_timeout(&taos->prx_wake_lock, 3 * HZ);
		disable_irq_nosync(taos->irq);
		queue_work(taos->taos_wq, &taos->work_prox);
	}
	return IRQ_HANDLED;
}

static void taos_work_func_light(struct work_struct *work)
{
	struct taos_data *taos =
	    container_of(work, struct taos_data, work_light);
	int lux = 0;

	lux = taos_get_lux(taos);

	input_report_rel(taos->light_input_dev, REL_MISC, lux + 1);
	input_sync(taos->light_input_dev);
}

static enum hrtimer_restart taos_timer_func(struct hrtimer *timer)
{
	struct taos_data *taos = container_of(timer, struct taos_data, timer);
	queue_work(taos->taos_wq, &taos->work_light);
	taos->light_polling_time = ktime_set(0, 0);
	taos->light_polling_time =
	    ktime_add_us(taos->light_polling_time, 200000);
	hrtimer_start(&taos->timer, taos->light_polling_time,
		       HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static void taos_work_func_ptime(struct work_struct *work)
{
	struct taos_data *taos =
	    container_of(work, struct taos_data, work_ptime);
	u16 proximity_value = 0;
	int min = 0, max = 0, avg = 0;
	int i = 0;
	for (i = 0; i < PROX_READ_NUM; i++) {
		proximity_value =
		    i2c_smbus_read_word_data(taos->opt_i2c_client,
					     CMD_REG | PRX_LO);
		if (proximity_value > TAOS_PROX_MAX)
			proximity_value = TAOS_PROX_MAX;
		if (proximity_value > TAOS_PROX_MIN) {
			avg += proximity_value;
			if (!i)
				min = proximity_value;
			else if (proximity_value < min)
				min = proximity_value;
			if (proximity_value > max)
				max = proximity_value;
		} else {
			proximity_value = TAOS_PROX_MIN;
		}
		msleep(40);
	}
	avg /= i;
	taos->avg[0] = min;
	taos->avg[1] = avg;
	taos->avg[2] = max;
}

static enum hrtimer_restart taos_ptimer_func(struct hrtimer *ptimer)
{
	struct taos_data *taos =
	    container_of(ptimer, struct taos_data, ptimer);
	queue_work(taos->taos_test_wq, &taos->work_ptime);
	hrtimer_forward_now(&taos->ptimer, taos->prox_polling_time);
	return HRTIMER_RESTART;
}

int taos_get_lux(struct taos_data *taos)
{
	int integration_time = 50;
	int als_gain = 1;
	int CPL = 0;
	int Lux1 = 0, Lux2 = 0;
	int irdata = 0;
	int cleardata = 0;
	int calculated_lux = 0;

	cleardata = i2c_smbus_read_word_data(taos->client,
		(CMD_REG | ALS_CHAN0LO));
	irdata = i2c_smbus_read_word_data(taos->client,
		(CMD_REG | ALS_CHAN1LO));
	taos->cleardata = cleardata;
	taos->irdata = irdata;

	CPL = (integration_time * als_gain * 1000) / taos->pdata->ga;
	Lux1 = (int)((taos->pdata->coef_a * cleardata
		- taos->pdata->coef_b * irdata) / CPL);
	Lux2 = (int)((taos->pdata->coef_c * cleardata
		- taos->pdata->coef_d * irdata) / CPL);

	if (Lux1 > Lux2)
		calculated_lux = Lux1;
	else
		calculated_lux = Lux2;

	if (calculated_lux < 0)
		calculated_lux = 0;

	/* protection code for strong sunlight */
	if (cleardata >= 18000 || irdata >= 18000) {
		calculated_lux = MAX_LUX;
		if (taos->proximity_value == 1) {
			taos->proximity_value = 0;
			taos_thresh_set(taos);
			input_report_abs(taos->prox_input_dev, ABS_DISTANCE,
					   !taos->proximity_value);
			input_sync(taos->prox_input_dev);
			msleep(20);
		}
	}

	/* printk("[TAOS] %s: lux = %d, ch[0] = %d, ch[1] = %d\n",
			__func__, calculated_lux, cleardata, irdata); */

	return calculated_lux;
}

void taos_chip_on(struct taos_data *taos)
{
	u8 value;
	int err = 0;
	int fail_num = 0;
	int num_try_init = 0;

	for (num_try_init = 0; num_try_init < 3 ; num_try_init++) {
		fail_num = 0;
		printk(KERN_INFO
				"[TAOS] %s\n", __func__);
		value = CNTL_REG_CLEAR;

		err = opt_i2c_write(taos, (CMD_REG | CNTRL), &value);
		if (err < 0) {
			printk(KERN_INFO
				"[diony] i2c_smbus_write_byte_data to clr ctrl"
				"reg failed in ioctl prox_on\n");
			fail_num++;
		}
		value = taos->pdata->als_time_param;

		err = opt_i2c_write(taos, (CMD_REG | ALS_TIME), &value);
		if (err < 0) {
			printk(KERN_INFO
				 "[diony] i2c_smbus_write_byte_data to als time"
				 "eg failed in ioctl prox_on\n");
			fail_num++;
		}
		value = PRX_ADC_TIME_PARAM;

		err = opt_i2c_write(taos, (CMD_REG | PRX_TIME), &value);
		if (err < 0) {
			printk(KERN_INFO
				 "[diony] i2c_smbus_write_byte_data to prox time"
				 "reg failed in ioctl prox_on\n");
			fail_num++;
		}
		value = PRX_WAIT_TIME_PARAM;

		err = opt_i2c_write(taos, (CMD_REG | WAIT_TIME), &value);
		if (err < 0) {
			printk(KERN_INFO
				 "[diony] i2c_smbus_write_byte_data to wait time"
				 "reg failed in ioctl prox_on\n");
			fail_num++;
		}
		value = taos->pdata->intr_filter_param;

		err = opt_i2c_write(taos, (CMD_REG | INTERRUPT), &value);
		if (err < 0) {
			printk(KERN_INFO
				"[diony] i2c_smbus_write_byte_data to interrupt"
				"reg failed in ioctl prox_on\n");
			fail_num++;
		}
		value = PRX_CONFIG_PARAM;

		err = opt_i2c_write(taos, (CMD_REG | PRX_CFG), &value);
		if (err < 0) {
			printk(KERN_INFO
				 "[diony] i2c_smbus_write_byte_data to prox"
				 "cfg reg failed in ioctl prox_on\n");
			fail_num++;
		}
		value = taos->pdata->prx_pulse_cnt_param;

		err = opt_i2c_write(taos, (CMD_REG | PRX_COUNT), &value);
		if (err < 0) {
			printk(KERN_INFO
				 "[diony] i2c_smbus_write_byte_data to prox"
				 "cnt reg failed in ioctl prox_on\n");
			fail_num++;
		}
		if (taos->chipID == 0x39)
			value = PRX_GAIN_PARAM;	/* 100mA, ch1, pgain 4x, again 1x */
		else if (taos->chipID == 0x29)
			value = 0x20;	/* 100mA, ch1, pgain 1x, again 1x */
		else
			value = PRX_GAIN_PARAM;

		err = opt_i2c_write(taos, (CMD_REG | GAIN), &value);
		if (err < 0) {
			printk(KERN_INFO
				"[diony] i2c_smbus_write_byte_data to prox"
				"gain reg failed in ioctl prox_on\n");
			fail_num++;
		}

		taos->proximity_value = 0;
		taos_thresh_set(taos);
		value = CNTL_INTPROXPON_ENBL;
		err = opt_i2c_write(taos, (CMD_REG | CNTRL), &value);
		if (err < 0) {
			printk(KERN_INFO
				"[diony]i2c_smbus_write_byte_data to ctrl reg"
				"failed in ioctl prox_on\n");
			fail_num++;
		}
		msleep(20);
		if (fail_num == 0) {
			taos->taos_chip_status = TAOS_CHIP_WORKING;
			break;
		} else {
			printk(KERN_INFO
				"I2C failed in taos_chip_on, # of fail I2C=[%d]\n",
				fail_num);
		}
	}
}

static int taos_chip_off(struct taos_data *taos)
{
	int ret = 0;
	u8 reg_cntrl;

	printk(KERN_INFO
		"[TAOS] %s\n", __func__);

	reg_cntrl = CNTL_REG_CLEAR;

	ret = opt_i2c_write(taos, (CMD_REG | CNTRL), &reg_cntrl);
	if (ret < 0) {
		printk(KERN_INFO
			"opt_i2c_write to ctrl reg failed in taos_chip_off\n");
		return ret;
	}
	taos->taos_chip_status = TAOS_CHIP_SLEEP;
	return ret;
}


/***********************************************************************
 *
 *  function    : taos_on
 *  description : This function is power-on function for optical sensor.
 *
 *  int type    : Sensor type. Two values is available (PROXIMITY,LIGHT).
 *                it support power-on function separately.
 *
 *
 */
void taos_on(struct taos_data *taos, int type)
{
	int err = 0;
	taos_chip_on(taos);

	if (type == TAOS_PROXIMITY || type == TAOS_ALL) {
		enable_irq(taos->irq);
		err = enable_irq_wake(taos->irq);
		if (err) {
			printk(KERN_INFO
				"[TAOS] register wakeup source failed\n");
		}
		taos->taos_prx_status = TAOS_PRX_OPENED;
		printk(KERN_INFO
			"[TAOS_PROXIMITY] %s: timer start for prox sensor\n",
			 __func__);
	}
	if (type == TAOS_LIGHT || type == TAOS_ALL) {
		printk(KERN_INFO
			"[TAOS_LIGHT] %s: timer start for light sensor\n",
			__func__);
		taos->light_polling_time = ktime_set(0, 0);
		taos->light_polling_time =
		    ktime_add_us(taos->light_polling_time, 200000);
		taos_light_enable(taos);
		msleep(50);
		taos->taos_als_status = TAOS_ALS_OPENED;
	}
}


/****************************************************************************
 *
 *  function    : taos_off
 *  description : This function is power-off function for optical sensor.
 *
 *  int type    : Sensor type. Three values is available (PROXIMITY,LIGHT,ALL).
 *                it support power-on function separately.
 *
 *
 */
void taos_off(struct taos_data *taos, int type)
{
	if (type == TAOS_PROXIMITY || type == TAOS_ALL) {
		printk(KERN_INFO
			"[TAOS] %s: disable irq for proximity\n", __func__);
		disable_irq(taos->irq);
		disable_irq_wake(taos->irq);
		taos->taos_prx_status = TAOS_PRX_CLOSED;
		/* wonjun initialize proximity_value*/
		taos->proximity_value = 0;
	}
	if (type == TAOS_LIGHT || type == TAOS_ALL) {
		printk(KERN_INFO
			"[TAOS] %s: timer cancel for light sensor\n", __func__);
		taos_light_disable(taos);
		taos->taos_als_status = TAOS_ALS_CLOSED;
	}
	if (taos->taos_prx_status == TAOS_PRX_CLOSED
	      && taos->taos_als_status == TAOS_ALS_CLOSED
	      && taos->taos_chip_status == TAOS_CHIP_WORKING) {
		taos_chip_off(taos);
	}
}

static int
taos_opt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	struct input_dev *light_input_dev;
	struct input_dev *prox_input_dev;
	struct taos_data *taos;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_INFO
			"[TAOS] i2c_check_functionality error\n");
		err = -ENODEV;
		goto exit;
	}
	if (!i2c_check_functionality
	     (client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_INFO
			"[TAOS] byte op is not permited.\n");
		err = -ENODEV;
		goto exit;
	}

	/* OK. For now, we presume we have a valid client.
	 *	We now create the client structure,
	 *	even though we cannot fill it completely yet. */
	taos =  kzalloc(sizeof(struct taos_data), GFP_KERNEL);
	if (taos == NULL) {
		err = -ENOMEM;
		goto exit_kfree;
	}
	memset(taos, 0, sizeof(struct taos_data));
	taos->client = client;
	i2c_set_clientdata(client, taos);
	taos->opt_i2c_client = client;
	printk(KERN_INFO
		"[%s] slave addr = %x\n", __func__, taos->opt_i2c_client->addr);
	msleep(20);
	taos->pdata = client->dev.platform_data;
	taos->threshold_high = taos->pdata->prx_thrsh_hi_param;
	taos->threshold_low = taos->pdata->prx_thrsh_lo_param;

	/* wake lock init */
	wake_lock_init(&taos->prx_wake_lock, WAKE_LOCK_SUSPEND,
			   "prx_wake_lock");
	mutex_init(&taos->power_lock);
	mutex_init(&taos->prox_mutex);
	/* allocate proximity input_device */
	prox_input_dev = input_allocate_device();
	if (prox_input_dev == NULL) {
		pr_err("Failed to allocate input device\n");
		err = -ENOMEM;
		goto err_input_allocate_device_proximity;
	}
	taos->prox_input_dev = prox_input_dev;
	input_set_drvdata(prox_input_dev, taos);
	prox_input_dev->name = "proximity_sensor";
	set_bit(EV_ABS, prox_input_dev->evbit);
	input_set_capability(prox_input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(prox_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	err = input_register_device(prox_input_dev);
	if (err) {
		pr_err("Unable to register %s input device\n",
			prox_input_dev->name);
		input_free_device(taos->prox_input_dev);
		goto err_input_register_device_proximity;
	}
	err =
	    sysfs_create_group(&prox_input_dev->dev.kobj,
			       &proximity_attribute_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		err = -ENOMEM;
		goto err_sysfs_create_group_proximity;
	}

	/* WORK QUEUE Settings */
	taos->taos_wq = create_singlethread_workqueue("taos_wq");
	if (!taos->taos_wq) {
		err = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_taos_wq;
	}

	taos->taos_test_wq = create_singlethread_workqueue("taos_test_wq");
	if (!taos->taos_test_wq) {
		err = -ENOMEM;
		pr_err("%s: could not create workqueue\n", __func__);
		goto err_create_taos_test_wq;
	}
	INIT_WORK(&taos->work_ptime, taos_work_func_ptime);
	INIT_WORK(&taos->work_prox, taos_work_func_prox);
	INIT_WORK(&taos->work_light, taos_work_func_light);
	/* hrtimer settings.  we poll for light values using a timer. */
	hrtimer_init(&taos->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	taos->timer.function = taos_timer_func;
	hrtimer_init(&taos->ptimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	taos->prox_polling_time = ns_to_ktime(2000 * NSEC_PER_MSEC);
	taos->ptimer.function = taos_ptimer_func;
	taos->cal_result = 0;
#ifdef CONFIG_MACH_RAY
	taos->initial_offset = 0x2D;
	taos->offset_value = taos->initial_offset;
	set_prox_offset(taos, taos->offset_value);
#else
	taos->initial_offset = (u8)taos_get_initial_offset(taos);
	taos->offset_value = taos->initial_offset;
#endif
	/* allocate lightsensor-level input_device */
	light_input_dev = input_allocate_device();
	if (!light_input_dev) {
		pr_err("%s: could not allocate input device\n", __func__);
		err = -ENOMEM;
		goto err_input_allocate_device_light;
	}
	input_set_drvdata(light_input_dev, taos);
	light_input_dev->name = "light_sensor";
	set_bit(EV_REL, light_input_dev->evbit);
	input_set_capability(light_input_dev, EV_REL, REL_MISC);
	err = input_register_device(light_input_dev);
	if (err) {
		pr_err("Unable to register %s input device\n",
			light_input_dev->name);
		input_free_device(taos->prox_input_dev);
		input_free_device(taos->light_input_dev);
		goto err_input_register_device_light;
	}
	taos->light_input_dev = light_input_dev;
	err =
	    sysfs_create_group(&light_input_dev->dev.kobj,
				&light_attribute_group);
	if (err) {
		pr_err("%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	/* INT Settings */
	taos->irq = taos->pdata->als_int;

	err = request_threaded_irq(taos->irq, NULL, taos_irq_handler,
				 IRQF_DISABLED | IRQ_TYPE_EDGE_FALLING,
				 "taos_int", taos);
	if (err) {
		printk(KERN_INFO
			"[TAOS] request_irq failed for taos\n");
		goto err_request_irq;
	}

	enable_irq_wake(taos->irq);
	taos->chipID =
	    i2c_smbus_read_byte_data(taos->opt_i2c_client, CMD_REG | CHIPID);
	printk(KERN_INFO
		"[TAOS] %s: chipID[%X]\n", __func__, taos->chipID);

	if (taos->chipID != TAOS_CHIP_DEV_ID) {
		printk(KERN_INFO "[TAOS] Chip ID Error\n");
		err = -ENOMEM;
		goto err_chipID_read_err;
	}
	err = sensors_register(&prox_sys_device,
		taos, prox_attrs,
		"proximity_sensor");
	if (err) {
		printk(KERN_INFO "[TAOS] sensors_register proximity Error\n");
		goto err_sensors_register_prox;
	}
	err = sensors_register(&light_sys_device,
		taos, light_attrs,
		"light_sensor");
	if (err) {
		printk(KERN_INFO "[TAOS] sensors_register light Error\n");
		goto err_sensors_register_light;
	}
	/* maintain power-down mode before using sensor */
	taos_off(taos, TAOS_ALL);
	taos->proximity_enable = OFF;
	taos->light_enable = OFF;
	if (taos->pdata->power_en)
		taos->pdata->power_en(0);
	else
		printk(KERN_INFO
			"[TAOS] %s disable power_en\n", __func__);
	if (taos->pdata->power_led)
		taos->pdata->power_led(0);
	else
		printk(KERN_INFO
			"[TAOS] %s disable power_led\n", __func__);

	goto exit;
err_sensors_register_light:
	sensors_unregister(light_sys_device, light_attrs);
err_sensors_register_prox:
	sensors_unregister(prox_sys_device, prox_attrs);
err_chipID_read_err:
	disable_irq_wake(taos->irq);
err_request_irq:
	sysfs_remove_group(&light_input_dev->dev.kobj,
		&light_attribute_group);
err_sysfs_create_group_light:
	input_unregister_device(taos->light_input_dev);
err_input_register_device_light:
err_input_allocate_device_light:
	destroy_workqueue(taos->taos_test_wq);
err_create_taos_test_wq:
	destroy_workqueue(taos->taos_wq);
err_create_taos_wq:
	sysfs_remove_group(&taos->prox_input_dev->dev.kobj,
			    &proximity_attribute_group);
err_sysfs_create_group_proximity:
	input_unregister_device(taos->prox_input_dev);
err_input_register_device_proximity:
err_input_allocate_device_proximity:
	mutex_destroy(&taos->power_lock);
	mutex_destroy(&taos->prox_mutex);
	wake_lock_destroy(&(taos->prx_wake_lock));
	kfree(taos);
exit_kfree:
exit:
	return err;
}

static void taos_opt_shutdown(struct i2c_client *client)
{
	struct taos_data *taos = i2c_get_clientdata(client);
	pr_info("[TAOS] %s, %d\n", __func__, __LINE__);
	if (taos != NULL) {
		if (taos->light_enable == ON)
			taos_off(taos, TAOS_LIGHT);
		if (taos->proximity_enable == ON)
			taos_off(taos, TAOS_PROXIMITY);

		if (taos->light_input_dev != NULL)
			sysfs_remove_group(&taos->light_input_dev->dev.kobj,
				      &light_attribute_group);
		if (taos->prox_input_dev != NULL)
			sysfs_remove_group(&taos->prox_input_dev->dev.kobj,
				    &proximity_attribute_group);

		input_unregister_device(taos->light_input_dev);
		input_unregister_device(taos->prox_input_dev);

		if (taos->taos_wq)
			destroy_workqueue(taos->taos_wq);
		if (taos->taos_test_wq)
			destroy_workqueue(taos->taos_test_wq);

		mutex_destroy(&taos->power_lock);
		mutex_destroy(&taos->prox_mutex);
		wake_lock_destroy(&(taos->prx_wake_lock));
		kfree(taos);
	}
}

static int taos_opt_suspend(struct device *dev)
{

	struct i2c_client *client = to_i2c_client(dev);
	struct taos_data *taos = i2c_get_clientdata(client);
	pr_info("[taos] %s, %d, %d\n",
		__func__,
		taos->light_enable,
		taos->proximity_enable);
	if (taos->light_enable == ON) {
		taos_off(taos, TAOS_LIGHT);
		taos->pdata->power_en(0);
	}
	return 0;
}

static int taos_opt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct taos_data *taos = i2c_get_clientdata(client);
	pr_info("[taos] %s, %d, %d\n",
		__func__,
		taos->light_enable,
		taos->proximity_enable);
	if (taos->light_enable == ON) {
		taos->pdata->power_en(1);
		taos_on(taos, TAOS_LIGHT);
	}
	return 0;
}

static const struct i2c_device_id taos_id[] = { {"taos", 0}, {}
};

MODULE_DEVICE_TABLE(i2c, taos_id);
static const struct dev_pm_ops taos_pm_ops = { .suspend =
	    taos_opt_suspend, .resume = taos_opt_resume,
};

static struct i2c_driver taos_opt_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "taos",
		.pm = &taos_pm_ops,
	},
	.probe = taos_opt_probe,
	.shutdown = taos_opt_shutdown,
	.id_table = taos_id,
};

static int __init taos_opt_init(void)
{
	return i2c_add_driver(&taos_opt_driver);
}

static void __exit taos_opt_exit(void)
{
	i2c_del_driver(&taos_opt_driver);
} module_init(taos_opt_init);

module_exit(taos_opt_exit);
MODULE_AUTHOR("SAMSUNG");
MODULE_DESCRIPTION("Optical Sensor driver for taosp002s00f");
MODULE_LICENSE("GPL");
