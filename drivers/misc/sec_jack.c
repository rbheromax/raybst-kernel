/*
 *  H2W device detection driver.
 *
 * Copyright (C) 2008 SAMSUNG Corporation.
 * Copyright (C) 2008 Google, Inc.
 *
 * Authors:
 *  Laurence Chen <Laurence_Chen@htc.com>
 *  Nick Pelly <npelly@google.com>
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

/*  For detecting SAMSUNG 2 Wire devices, such as wired headset.

    Logically, the H2W driver is always present, and H2W state (hi->state)
    indicates what is currently plugged into the H2W interface.

    When the headset is plugged in, CABLE_IN1 is pulled low. When the headset
    button is pressed, CABLE_IN2 is pulled low. These two lines are shared with
    the TX and RX (respectively) of UART3 - used for serial debugging.

    This headset driver keeps the CPLD configured as UART3 for as long as
    possible, so that we can do serial FIQ debugging even when the kernel is
    locked and this driver no longer runs. So it only configures the CPLD to
    GPIO while the headset is plugged in, and for 10ms during detection work.

    Unfortunately we can't leave the CPLD as UART3 while a headset is plugged
    in, UART3 is pullup on TX but the headset is pull-down, causing a 55 mA
    drain on bigfoot.

    The headset detection work involves setting CPLD to GPIO, and then pulling
    CABLE_IN1 high with a stronger pullup than usual. A H2W headset will still
    pull this line low, whereas other attachments such as a serial console
    would get pulled up by this stronger pullup.

    Headset insertion/removal causes UEvent's to be sent, and
    /sys/class/switch/h2w/state to be updated.

    Button presses are interpreted as input event (KEY_MEDIA). Button presses
    are ignored if the headset is plugged in, so the buttons on 11 pin -> 3.5mm
    jack adapters do not work until a headset is plugged into the adapter. This
    is to avoid serial RX traffic causing spurious button press events.

    We tend to check the status of CABLE_IN1 a few more times than strictly
    necessary during headset detection, to avoid spurious headset insertion
    events caused by serial debugger TX traffic.
*/

#include <linux/module.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/atomic.h>
#include <mach/board.h>
#include <mach/vreg.h>
#include <mach/pmic.h>
#include <linux/mfd/pmic8058.h>
#include <linux/sec_jack.h>
#include <mach/gpio.h>
#include <linux/slab.h>

#include <linux/gpio.h>

#include <mach/hardware.h>
#ifdef CONFIG_MACH_RAY
#include <linux/pm_qos.h>
#endif
#define FEATURE_ADC_READ

#ifdef FEATURE_ADC_READ
#include <mach/msm_rpcrouter.h>
#include <linux/atomic.h>
#include <linux/err.h>
#endif

#define ACCIDENT_EAR_BTN

#ifdef ACCIDENT_EAR_BTN
#include <linux/time.h>

#define ACCIDENT_BTN_DELAY_MAX  1000
#define ACCIDENT_BTN_TIME_MAX  10
#endif

enum {
	JACK_ACTIVE_LOW	= 0,
	JACK_ACTIVE_HIGH = 1,
};

int detect_type;

#define CONFIG_DEBUG_H2W

#define SUPPORT_PBA

static bool recheck_jack;

#ifdef ACCIDENT_EAR_BTN
static int accident_btn_count;
#endif

#ifdef FEATURE_ADC_READ
#if defined(CONFIG_MACH_ICON)
struct adc_range {
	int start;
	int end;
} adc_range_rev9[] = {
	{0, 620}, {621, 1700}, {1701, 9999}, {0, 106}, {107, 222}, {223, 450}
};

struct adc_range *adc_range;
#elif defined(CONFIG_MACH_RAY)
struct adc_range {
	int start;
	int end;
} adc_range_rev9[] = {
	{0, 830}, {831, 1700}, {1851, 9999}, {0, 113}, {114, 239}, {240, 470}, {1701, 1850}
};

struct adc_range *adc_range;
#else
struct adc_range {
	int start;
	int end;
} adc_range_rev9[] = {
	{0, 620}, {621, 1700}, {1701, 9999}, {50, 140}, {141, 215}, {216, 460}
};

struct adc_range *adc_range;
#endif
#endif

#ifdef CONFIG_MACH_RAY
static struct pm_qos_request jack_qos_req;
static int sec_jack_pm_qos;
#endif

static struct workqueue_struct *g_earbutton_work_queue;
static void earbutton_work(struct work_struct *work);
static DECLARE_WORK(g_earbutton_work, earbutton_work);
static unsigned int earbutton_pressed;

#define BIT_HEADSET			(1 << 0)
#define BIT_HEADSET_NO_MIC	(1 << 1)

enum {
	H2W_NO_DEVICE	= 0,
	H2W_SEC_HEADSET	= 1,
	H2W_NORMAL_HEADSET = 2,
};

struct h2w_info {
	struct switch_dev sdev;
	struct switch_dev button_sdev;

	struct input_dev *input;
	struct mutex mutex_lock;

	atomic_t btn_state;
	int ignore_btn;

	int irq;
	int irq_btn;
	int btn_11pin_35mm_flag;

	struct hrtimer timer;
	ktime_t debounce_time;
	int headset_state;

	struct hrtimer btn_timer;
	ktime_t btn_debounce_time;
	int button_state;

	struct hrtimer irq_delay_timer;
	ktime_t irq_delay_time;
	int btn_irq_available;
	int pressed_btn;
	unsigned int use_irq:1;
};
static struct h2w_info *hi;

#ifdef FEATURE_ADC_READ

#define EAR_RPC_PROG  0x30000089
#define EAR_RPC_TIMEOUT 5000	/* 5 sec */
#define EAR_CB_TYPE_PROC	1

#define EAR_READ_PROC 17

#define BATTERY_RPC_PROG	0x30000089
#define BATTERY_RPC_VER_1_1	0x00010001
#define BATTERY_RPC_VER_2_1	0x00020001
#define BATTERY_RPC_VER_5_1	0x00050001

struct msm_ear_get_adc_ret_data {
	u32 ear_adc;
};


struct msm_rpc_client *ear_client;

u32 ear_batt_api_version;

static int msm_ear_get_adc_ret_func(struct msm_rpc_client *client,
				       void *buf, void *data)
{
	struct msm_ear_get_adc_ret_data *data_ptr, *buf_ptr;

	data_ptr = (struct msm_ear_get_adc_ret_data *)data;
	buf_ptr = (struct msm_ear_get_adc_ret_data *)buf;

	data_ptr->ear_adc = be32_to_cpu(buf_ptr->ear_adc);

	return 0;
}

u32 ear_get_adc(void)
{
	int rc;

	struct msm_ear_get_adc_ret_data rep;

	rc = msm_rpc_client_req(ear_client,
			EAR_READ_PROC,
			NULL, NULL,
			msm_ear_get_adc_ret_func, &rep,
			msecs_to_jiffies(EAR_RPC_TIMEOUT));

	if (rc < 0) {
		pr_err("%s: FAIL: mpp4 get adc. rc=%d\n", __func__, rc);
		return 0;
	}

	return rep.ear_adc;
}
EXPORT_SYMBOL(ear_get_adc);


static int msm_ear_cb_func(struct msm_rpc_client *client,
			    void *buffer, int in_size)
{
	int rc = 0;
	struct rpc_request_hdr *req;
	u32 procedure;
	u32 accept_status;

	req = (struct rpc_request_hdr *)buffer;
	procedure = be32_to_cpu(req->procedure);

	switch (procedure) {
	case EAR_CB_TYPE_PROC:
		accept_status = RPC_ACCEPTSTAT_SUCCESS;
		break;

	default:
		accept_status = RPC_ACCEPTSTAT_PROC_UNAVAIL;
		pr_err("%s: ERROR. procedure (%d) not supported\n",
		       __func__, procedure);
		break;
	}

	msm_rpc_start_accepted_reply(ear_client,
			be32_to_cpu(req->xid), accept_status);

	rc = msm_rpc_send_accepted_reply(ear_client, 0);
	if (rc)
		pr_err("%s: FAIL: sending reply. rc=%d\n", __func__, rc);

	return rc;
}


void msm_ear_cleanup(void)
{
	if (ear_client)
		msm_rpc_unregister_client(ear_client);
}
EXPORT_SYMBOL(msm_ear_cleanup);

int msm_ear_init_rpc(void)
{
	int rc = 0;

	ear_client =
		msm_rpc_register_client("sec_jack", EAR_RPC_PROG,
					BATTERY_RPC_VER_2_1,
					1, msm_ear_cb_func);

	if (ear_client == NULL) {
		pr_err("%s: FAIL: rpc_register_client. ear_client=NULL\n",
		       __func__);
		return -ENODEV;
	} else if (IS_ERR(ear_client)) {
		ear_client =
			msm_rpc_register_client("sec_jack", EAR_RPC_PROG,
						BATTERY_RPC_VER_1_1,
						1, msm_ear_cb_func);
		ear_batt_api_version =  BATTERY_RPC_VER_1_1;
	} else
		ear_batt_api_version =  BATTERY_RPC_VER_2_1;

	if (IS_ERR(ear_client)) {
		ear_client =
			msm_rpc_register_client("sec_jack", EAR_RPC_PROG,
						BATTERY_RPC_VER_5_1,
						1, msm_ear_cb_func);
		ear_batt_api_version =  BATTERY_RPC_VER_5_1;
	}

	if (IS_ERR(ear_client)) {
		rc = PTR_ERR(ear_client);
		pr_err("%s: ERROR: rpc_register_client: rc = %d\n ",
		       __func__, rc);
		ear_client = NULL;
		return rc;
	}

	return rc;
}
EXPORT_SYMBOL(msm_ear_init_rpc);

void ear_rpc_init(void)
{
	/* RPC initial sequence */
	int err = 1;

	err = msm_ear_init_rpc();

	if (err < 0) {
		pr_err("%s: FAIL: msm_ear_init_rpc.  err=%d\n", __func__, err);
		msm_ear_cleanup();
	}
}
#endif

static ssize_t h2w_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hi->sdev)) {
	case H2W_NO_DEVICE:
		return sprintf(buf, "No Device\n");
	case H2W_SEC_HEADSET:
		return sprintf(buf, "H2W_SEC_HEADSET\n");
	case H2W_NORMAL_HEADSET:
		return sprintf(buf, "H2W_NORMAL_HEADSET\n");
	}
	return -EINVAL;
}

static ssize_t button_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hi->button_sdev)) {
	case 0:
		return sprintf(buf, "No Input\n");
	case 1:
		return sprintf(buf, "Button Pressed\n");
	}
	return -EINVAL;
}


void headset_button_event(int is_press)
{
	if (!is_press) {
		if (atomic_read(&hi->btn_state))
			earbutton_pressed = 0;
			/*button_released();*/
	} else {
		if (!atomic_read(&hi->btn_state))
			earbutton_pressed = 1;
	}
	queue_work(g_earbutton_work_queue, &g_earbutton_work);
}

static void remove_headset(void)
{
	unsigned long irq_flags;

	pr_info("remove_headset\n");

	gpio_set_value(MSM_GPIO_EAR_MICBIAS_EN, 0);

	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, switch_get_state(&hi->sdev)
	& ~(BIT_HEADSET | BIT_HEADSET_NO_MIC));
	mutex_unlock(&hi->mutex_lock);

	if (hi->btn_11pin_35mm_flag) {
		irq_set_irq_wake(hi->irq_btn, 0);
		local_irq_save(irq_flags);
		disable_irq(hi->irq_btn);
		local_irq_restore(irq_flags);
		hi->btn_11pin_35mm_flag = 0;
		if (atomic_read(&hi->btn_state))
			headset_button_event(0);
	}
	hi->debounce_time = ktime_set(0, 200000000);  /* 100 ms */
	hrtimer_cancel(&hi->btn_timer);
	hrtimer_cancel(&hi->irq_delay_timer);
	hi->btn_irq_available = 0;
#ifdef CONFIG_MACH_RAY
	if (sec_jack_pm_qos) {
		pm_qos_update_request(&jack_qos_req, PM_QOS_DEFAULT_VALUE);
		sec_jack_pm_qos = 0;
	}
#endif
}

static void insert_headset(void)
{
	unsigned long irq_flags;
	int state, adc;

	pr_info("insert_headset\n");
	gpio_set_value(MSM_GPIO_EAR_MICBIAS_EN, 1);

	state = BIT_HEADSET | BIT_HEADSET_NO_MIC;

	state = switch_get_state(&hi->sdev);
	state &= ~(BIT_HEADSET_NO_MIC | BIT_HEADSET);

#ifdef ACCIDENT_EAR_BTN
	accident_btn_count = 0;
#endif

	/* Wait pin be stable */
	msleep(160);
#ifdef FEATURE_ADC_READ
	adc = ear_get_adc();
	printk(KERN_INFO"adc : %d\n", adc);
	if ((adc >= adc_range[1].start && adc <= adc_range[1].end)
		|| (adc >= adc_range[6].start && adc <= adc_range[6].end)) {
		state |= BIT_HEADSET;
#ifdef CONFIG_MACH_RAY
		if (!sec_jack_pm_qos) {
			pm_qos_update_request(&jack_qos_req, 499);
			sec_jack_pm_qos = 1;
		}
#endif
	} else if (adc >= adc_range[2].start && adc <= adc_range[2].end) {
		if (recheck_jack == true) {
			remove_headset();
			recheck_jack = false;
			return;
		}
		state |= BIT_HEADSET;
#ifdef CONFIG_MACH_RAY
		if (!sec_jack_pm_qos) {
			pm_qos_update_request(&jack_qos_req, 499);
			sec_jack_pm_qos = 1;
		}
#endif
	} else if (adc >= adc_range[0].start && adc <= adc_range[0].end) {
		state |= BIT_HEADSET_NO_MIC;
	gpio_set_value(MSM_GPIO_EAR_MICBIAS_EN, 0);
#ifdef CONFIG_MACH_RAY
	if (sec_jack_pm_qos) {
		pm_qos_update_request(&jack_qos_req, PM_QOS_DEFAULT_VALUE);
		sec_jack_pm_qos = 0;
	}
#endif
	}
#else
	state |= BIT_HEADSET;
#endif
	recheck_jack = false;
	/* Enable button irq */
	if ((!hi->btn_11pin_35mm_flag) && (state == BIT_HEADSET)) {
		irq_set_irq_type(hi->irq_btn, IRQF_TRIGGER_RISING);
		irq_set_irq_wake(hi->irq_btn, 1);

		local_irq_save(irq_flags);
		enable_irq(hi->irq_btn);
		local_irq_restore(irq_flags);

		hi->btn_11pin_35mm_flag = 1;
	}

	hi->debounce_time = ktime_set(0, 20000000);  /* 500 -> 20 ms */

	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, state);
	mutex_unlock(&hi->mutex_lock);
	hrtimer_start(&hi->irq_delay_timer,
	hi->irq_delay_time, HRTIMER_MODE_REL);
}

static void earbutton_work(struct work_struct *work)
{
	int adc = ear_get_adc();
	pr_info("earbutton_work\n");
	pr_info("adc : %d, earbutton_pressed : %d", adc , earbutton_pressed);
	atomic_set(&hi->btn_state, earbutton_pressed);
	if (earbutton_pressed == 1) {
		if (adc >= adc_range[5].start && adc <= adc_range[5].end)
			hi->pressed_btn = KEY_VOLUMEDOWN;
		else if (adc >= adc_range[4].start && adc <= adc_range[4].end)
			hi->pressed_btn = KEY_VOLUMEUP;
		else if (adc >= adc_range[3].start && adc <= adc_range[3].end)
			hi->pressed_btn = KEY_MEDIA;
		else
			return;
		input_report_key(hi->input, hi->pressed_btn, 1);
		input_sync(hi->input);
	} else {
		input_report_key(hi->input, hi->pressed_btn, 0);
		input_sync(hi->input);
	}
	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->button_sdev, earbutton_pressed);
	mutex_unlock(&hi->mutex_lock);
}

static enum hrtimer_restart button_event_timer_func(struct hrtimer *data)
{
	if ((gpio_get_value(MSM_GPIO_EAR_DET) != detect_type))
		return HRTIMER_NORESTART;

	/* check again*/
	if (hi->button_state != gpio_get_value(MSM_GPIO_EAR_SEND_END)) {
		pr_info("ERROR - button value : %d -> %d\n",
		hi->button_state,
		gpio_get_value(MSM_GPIO_EAR_SEND_END));
		return HRTIMER_NORESTART;
	}

	/* butten active high*/
	if (gpio_get_value(MSM_GPIO_EAR_SEND_END)) {
		headset_button_event(1);
		/* 10 ms */
		hi->btn_debounce_time = ktime_set(0, 10000000);
	} else {
		headset_button_event(0);
		/* 150 ms */
		hi->btn_debounce_time = ktime_set(0, 300000000);
	}
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart irq_delay_timer_func(struct hrtimer *data)
{
	hi->btn_irq_available = 1;
	return HRTIMER_NORESTART;
}


static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value_det = 0;
	int retry_limit = 10;
	printk(KERN_INFO"%s\n", __func__);
	irq_set_irq_type(hi->irq_btn, IRQF_TRIGGER_RISING);
	while (retry_limit-- > 0) {
		value_det = gpio_get_value(MSM_GPIO_EAR_DET);
		if (value_det != detect_type) {
			if ((switch_get_state(&hi->sdev) == H2W_NO_DEVICE)
				^ (value_det ^ detect_type))
				remove_headset();
			return IRQ_HANDLED;
		}
		usleep(10000);
	}

	if ((switch_get_state(&hi->sdev) == H2W_NO_DEVICE)
		^ (value_det ^ detect_type)) {
		msleep(100);
		insert_headset();
	}
	return IRQ_HANDLED;
}

static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

#ifdef ACCIDENT_EAR_BTN
	static struct timeval accident_btn_last;
	static struct timeval accident_btn;
#endif

	if ((switch_get_state(&hi->sdev) == H2W_NO_DEVICE)
		|| (hi->btn_irq_available == 0)) {
			pr_err("%s: retrun_irq\n",
		__func__);

#ifdef ACCIDENT_EAR_BTN

		do_gettimeofday(&accident_btn);

		if(accident_btn.tv_usec - accident_btn_last.tv_usec < ACCIDENT_BTN_DELAY_MAX)
		{
			accident_btn_count++;
			pr_info("[%s] accident_btn_count=%d\n", __func__, accident_btn_count);
		}
		if(accident_btn_count > ACCIDENT_BTN_TIME_MAX)
		{
			accident_btn_count = 0;
			disable_irq_nosync(hi->irq_btn);
			pr_info("[%s] disable_irq(hi->irq_btn)\n", __func__);
		}
			
		accident_btn_last.tv_sec = accident_btn.tv_sec;
		accident_btn_last.tv_usec = accident_btn.tv_usec;
#endif
		return IRQ_HANDLED;
	}

	do {
		value1 = gpio_get_value(MSM_GPIO_EAR_SEND_END);
		irq_set_irq_type(hi->irq_btn, value1 ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		value2 = gpio_get_value(MSM_GPIO_EAR_SEND_END);
	} while (value1 != value2 && retry_limit-- > 0);

	if (value1 == value2 && retry_limit > 0) {
		hi->btn_debounce_time = ktime_set(0, 60000000);
		hi->button_state = value1;
		hrtimer_start(&hi->btn_timer, hi->btn_debounce_time,
		HRTIMER_MODE_REL);
	}
	return IRQ_HANDLED;
}

#if defined(CONFIG_DEBUG_FS)
static int h2w_debug_set(void *data, u64 val)
{
	mutex_lock(&hi->mutex_lock);
	switch_set_state(&hi->sdev, (int)val);
	mutex_unlock(&hi->mutex_lock);
	return 0;
}

static int h2w_debug_get(void *data, u64 *val)
{
	return 0;
}


DEFINE_SIMPLE_ATTRIBUTE(h2w_debug_fops, h2w_debug_get, h2w_debug_set, "%llu\n");
static int __init h2w_debug_init(void)
{
	struct dentry *dent;

	dent = debugfs_create_dir("h2w", 0);
	if (IS_ERR(dent))
		return PTR_ERR(dent);

	debugfs_create_file("state", 0644, dent, NULL, &h2w_debug_fops);

	return 0;
}

device_initcall(h2w_debug_init);
#endif

#ifdef SUPPORT_PBA
static ssize_t  key_state_onoff_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int value = 0;

	msleep(20);

	if (earbutton_pressed != true)
		value = 0;
	else
		value = 1;
	return sprintf(buf, "%d\n", value);
}

static DEVICE_ATTR(key_state, 0664 , key_state_onoff_show,
		NULL);

static ssize_t  earjack_state_onoff_show(struct device *dev,
		struct device_attribute *attr, char *buf)

{
	int value = 0;
	int det_value = 0;
	det_value = gpio_get_value(MSM_GPIO_EAR_DET);

	if (det_value)
		value = 0;
	else
		value = 1;

	return sprintf(buf, "%d\n", value);
}

static DEVICE_ATTR(state, 0664 , earjack_state_onoff_show,
		NULL);

static ssize_t select_jack_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_info("%s : operate nothing\n", __func__);
	return 0;
}

static ssize_t select_jack_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	sscanf(buf, "%d", &value);
	pr_err("%s: User  selection : 0X%x", __func__, value);
	if (value == BIT_HEADSET) {
		gpio_set_value(MSM_GPIO_EAR_MICBIAS_EN, 1);
		msleep(100);
	}

	insert_headset();

	return size;
}

static DEVICE_ATTR(select_jack, 0664, select_jack_show,
		select_jack_store);

static ssize_t reselect_jack_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	pr_info("%s : operate nothing\n", __func__);
	return 0;
}

static ssize_t reselect_jack_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int value = 0;

	sscanf(buf, "%d", &value);
	pr_err("%s: User  selection : 0X%x", __func__, value);

	if (value == 1) {
		recheck_jack = true;
		insert_headset();
	}

	return size;
}

static DEVICE_ATTR(reselect_jack, 0664, reselect_jack_show,
		reselect_jack_store);

#endif


static int h2w_probe(struct platform_device *pdev)
{
	int ret;
	struct class *audio;
	struct device *earjack;

	pr_info("[H2W] EAR_DET : %d, MICBIAS_EN : %d\n",
	MSM_GPIO_EAR_DET, MSM_GPIO_MICBIAS_EN);
	hi = kzalloc(sizeof(struct h2w_info), GFP_KERNEL);
	if (!hi) {
		pr_err("kzalloc couldn't allocate memory\n");
		return -ENOMEM;
	}
	detect_type	= JACK_ACTIVE_LOW;

#ifdef FEATURE_ADC_READ
	adc_range = adc_range_rev9;
#endif


	atomic_set(&hi->btn_state, 0);
	hi->button_state = 0;

	hi->debounce_time = ktime_set(0, 250000000);  /* 100 ms -> 200 ms */
	hi->btn_debounce_time = ktime_set(0, 1000000); /* 1 ms */

	hi->btn_11pin_35mm_flag = 0;

	mutex_init(&hi->mutex_lock);

	hi->sdev.name = "h2w";
	hi->sdev.print_name = h2w_print_name;

	ret = switch_dev_register(&hi->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	hi->button_sdev.name = "sec_earbutton";
	hi->button_sdev.print_name = button_print_name;

	ret = switch_dev_register(&hi->button_sdev);
	if (ret < 0)
		goto err_switch_dev_register;
	switch_set_state(&hi->button_sdev, 0);

	g_earbutton_work_queue = create_workqueue("earbutton");
	if (g_earbutton_work_queue == NULL) {
		ret = -ENOMEM;
		goto err_create_work_queue;
	}

#ifdef SUPPORT_PBA
	audio = class_create(THIS_MODULE, "audio");
	if (IS_ERR(audio))
		pr_err("Failed to create class(audio)!\n");

	earjack = device_create(audio, NULL, 0, NULL, "earjack");
	if (IS_ERR(earjack))
		pr_err("Failed to create device(earjack)!\n");

	ret = device_create_file(earjack, &dev_attr_key_state);
	if (ret)
		pr_err("Failed to create device file in sysfs entries(%s)!\n",
				dev_attr_key_state.attr.name);

	ret = device_create_file(earjack, &dev_attr_state);
	if (ret)
		pr_err("Failed to create device file in sysfs entries(%s)!\n",
				dev_attr_state.attr.name);

	ret = device_create_file(earjack, &dev_attr_select_jack);
	if (ret)
		pr_err("Failed to create device file in sysfs entries(%s)!\n",
				dev_attr_select_jack.attr.name);

	ret = device_create_file(earjack, &dev_attr_reselect_jack);
	if (ret)
		pr_err("Failed to create device file in sysfs entries(%s)!\n",
				dev_attr_reselect_jack.attr.name);
#endif

#ifdef CONFIG_MACH_RAY
	pm_qos_add_request(&jack_qos_req,
		PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	sec_jack_pm_qos = 0;
#endif

/*****************************************************************/
/* JACK_S_35 GPIO setting*/
/*****************************************************************/
ret = gpio_tlmm_config(GPIO_CFG(MSM_GPIO_EAR_DET, 0,
GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA),
GPIO_CFG_ENABLE);
	if (ret < 0)
		goto err_request_detect_gpio;
	ret = gpio_request(MSM_GPIO_EAR_DET, "h2w_detect");
	if (ret < 0)
		goto err_request_detect_gpio;

	ret = gpio_direction_input(MSM_GPIO_EAR_DET);
	if (ret < 0)
		goto err_set_detect_gpio;

	hi->irq = gpio_to_irq(MSM_GPIO_EAR_DET);
	if (hi->irq < 0) {
		ret = hi->irq;
		goto err_get_h2w_detect_irq_num_failed;
	}
/*****************************************************************/
/* SEND_END GPIO setting*/
/*****************************************************************/
	 ret = gpio_tlmm_config(GPIO_CFG(MSM_GPIO_EAR_SEND_END, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	 if (ret < 0)
		goto err_request_button_gpio;
	ret = gpio_request(MSM_GPIO_EAR_SEND_END, "h2w_button");
	if (ret < 0)
		goto err_request_button_gpio;

	ret = gpio_direction_input(MSM_GPIO_EAR_SEND_END);
	if (ret < 0)
		goto err_set_button_gpio;

	hi->irq_btn = MSM_GPIO_TO_INT(MSM_GPIO_EAR_SEND_END);
	if (hi->irq_btn < 0) {
		ret = hi->irq_btn;
		goto err_get_button_irq_num_failed;
	}

	hrtimer_init(&hi->btn_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hi->btn_timer.function = button_event_timer_func;

/*****************************************************************/
/* JACK_S_35 irq setting*/
/*****************************************************************/
	ret = request_threaded_irq(hi->irq, NULL, detect_irq_handler,
			  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
			  | IRQF_ONESHOT,
			  "h2w_detect", NULL);

	if (ret < 0)
		goto err_request_detect_irq;

	hi->use_irq = ret == 0;

	pr_info("Headset Driver: Start gpio inputs for %s in %s mode\n",
	hi->sdev.name, hi->use_irq ? "interrupt" : "polling");

	ret = irq_set_irq_wake(hi->irq, 1);
	if (ret < 0)
		goto err_request_input_dev;

/*****************************************************************/
/* SEND_END irq setting*/
/*****************************************************************/
			/* Disable button until plugged in */
			set_irq_flags(hi->irq_btn, IRQF_VALID | IRQF_NOAUTOEN);

			ret = request_irq(hi->irq_btn, button_irq_handler,
			IRQF_TRIGGER_RISING, "h2w_button", NULL);

			if (ret < 0)
				goto err_request_button_irq;

	hi->input = input_allocate_device();
	if (!hi->input) {
		ret = -ENOMEM;
		goto err_request_input_dev;
	}

	hi->input->name = "sec_jack";	/* Behold - SGH-T939*/
	set_bit(EV_KEY, hi->input->evbit);
	set_bit(KEY_MEDIA, hi->input->keybit);
	set_bit(KEY_VOLUMEDOWN, hi->input->keybit);
	set_bit(KEY_VOLUMEUP, hi->input->keybit);

	ret = input_register_device(hi->input);
	if (ret < 0)
		goto err_register_input_dev;

	hrtimer_init(&hi->irq_delay_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hi->irq_delay_timer.function = irq_delay_timer_func;
	hi->irq_delay_time = ktime_set(0, 1000000000);  /* 1sec*/

#ifdef FEATURE_ADC_READ
	ear_rpc_init();
#endif

	return 0;


err_register_input_dev:
	input_free_device(hi->input);
err_request_input_dev:
	free_irq(hi->irq, 0);
	free_irq(hi->irq_btn, 0);
err_request_button_irq:
	gpio_free(MSM_GPIO_EAR_SEND_END);
err_request_detect_irq:
	gpio_free(MSM_GPIO_EAR_DET);
err_get_button_irq_num_failed:
	gpio_free(MSM_GPIO_EAR_DET);
err_request_button_gpio:
	gpio_free(MSM_GPIO_EAR_SEND_END);
err_set_button_gpio:
	gpio_free(MSM_GPIO_EAR_DET);
err_get_h2w_detect_irq_num_failed:
	gpio_free(MSM_GPIO_EAR_DET);
err_set_detect_gpio:
	gpio_free(MSM_GPIO_EAR_DET);
err_request_detect_gpio:
	destroy_workqueue(g_earbutton_work_queue);
err_create_work_queue:
	switch_dev_unregister(&hi->sdev);
err_switch_dev_register:
	kzfree(hi);
	pr_err("H2W: Failed to register driver\n");

	return ret;
}

static int h2w_remove(struct platform_device *pdev)
{
	pr_info("h2w_remove\n");
	if (switch_get_state(&hi->sdev))
		remove_headset();
	input_unregister_device(hi->input);
	gpio_free(MSM_GPIO_EAR_SEND_END);
	gpio_free(MSM_GPIO_EAR_DET);
	free_irq(hi->irq_btn, 0);
	free_irq(hi->irq, 0);
	destroy_workqueue(g_earbutton_work_queue);
	switch_dev_unregister(&hi->sdev);

	return 0;
}


static struct platform_driver h2w_driver = {
	.probe		= h2w_probe,
	.remove		= h2w_remove,
	.driver		= {
		.name		= "sec_jack",
		.owner		= THIS_MODULE,
	},
};

int __init h2w_init(void)
{
	int ret;
	pr_info("JACK_S_35(%d), SEND_END(%d)",
	MSM_GPIO_EAR_DET, MSM_GPIO_EAR_SEND_END);
	ret = platform_driver_register(&h2w_driver);
	if (ret)
		return ret;
	else
		return 0;
}

static void __exit h2w_exit(void)
{
	platform_driver_unregister(&h2w_driver);
}

module_init(h2w_init);
module_exit(h2w_exit);

MODULE_AUTHOR("anonymous <anonymous@samsung.com>");
MODULE_DESCRIPTION("SAMSUNG Headset driver");
MODULE_LICENSE("GPL");


