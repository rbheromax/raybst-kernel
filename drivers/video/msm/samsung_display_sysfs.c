/*
* This probram is working for lcd on/off and read lcd type from sysfs.
* If the func "samsung_display_sysfs_create()" called from lcd driver,
* the sysfs(lcd_power, lcd_type) created under "/sys/class/lcd/panel".
*/

#include "samsung_display_sysfs.h"

#define LCDC_DEBUG

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_INFO "LCD(sysfs) " x)
#else
#define DPRINT(x...)
#endif

static char lcd_type[MAX_PANEL_NAME];
static struct msm_fb_data_type *mfd;

static void set_panel_name(char *panel_name, unsigned char size)
{
	/*snprintf(lcd_type, MAX_PANEL_NAME , panel_name);*/
	snprintf(lcd_type, MAX_PANEL_NAME , "%s\n", panel_name);
}

static char *get_panel_name(void)
{
	return lcd_type;
}

static ssize_t show_lcdtype(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	char *panel_name;
	/*unsigned len = 0;*/

	panel_name = get_panel_name();
	/*strncat(panel_name, "\n", 1);*/
	/*use  strlact instead, refer to checkpatch.pl.2012.0912*/
	/*strlcat(panel_name, "\n", 1);*/
	/*len = strnlen(panel_name, MAX_PANEL_NAME);*/

	DPRINT("%s : %s", __func__, panel_name);

	/*strncat(buf, panel_name, len);*/
	/*use  strlact instead, refer to checkpatch.pl.2012.0912*/
	/*strlcat(buf, panel_name, len);*/

	/*return len;*/
	return snprintf(buf, MAX_PANEL_NAME, panel_name);
}

static DEVICE_ATTR(lcd_type, S_IRUGO, show_lcdtype, NULL);

static void get_platform_data(struct platform_device *msm_fb_dev)
{
	mfd = platform_get_drvdata(msm_fb_dev);
}

static int default_disp_set_power(struct lcd_device *dev, int power)
{
	return 0;
}

static int default_disp_get_power(struct lcd_device *dev)
{
	return 0;
}

static int disp_set_power(struct lcd_device *dev, int power)
{
	struct msm_fb_panel_data *pdata = NULL;
	static __u32 bl_level;
	int on_flag = !!power;
	static int abuse_count;
	static int use_flag;

	if (!mfd || !(mfd->pdev) || !(mfd->pdev->dev.platform_data))
		return -ENODEV;

	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if (on_flag == mfd->panel_power_on)
		return 0;
	else if (!mfd->panel_power_on) {
		if (!use_flag && on_flag) {
			abuse_count++;
			DPRINT("LCD on is not working during LCD off status\n");
			return -EPERM;
		}
	} else if (abuse_count) {
		DPRINT("LCD off is not working because of abuse count is %d\n",
						abuse_count);
		abuse_count = 0;
		DPRINT("Abuse count reset as %d\n", abuse_count);
		return -EPERM;
	}

	if (on_flag) {
		use_flag = 0;
		mfd->fbi->fbops->fb_blank(FB_BLANK_UNBLANK, mfd->fbi);
		mfd->fbi->fbops->fb_pan_display(&mfd->fbi->var, mfd->fbi);
		mfd->bl_level = bl_level;
		if ((pdata) && (pdata->set_backlight))
			pdata->set_backlight(mfd);
		else {
			DPRINT("set_backlight is not working\n");
			return -EPERM;
		}
	} else {
		use_flag = 1;
		bl_level = mfd->bl_level;
		mfd->bl_level = 0x0;

		if ((pdata) && (pdata->set_backlight))
			pdata->set_backlight(mfd);
		else {
			DPRINT("set_backlight is not working\n");
			return -EPERM;
		}

		mfd->fbi->fbops->fb_blank(FB_BLANK_POWERDOWN, mfd->fbi);
	}
	DPRINT("disp_set_power : %d\n", on_flag);

	return 0;
}

static int disp_get_power(struct lcd_device *dev)
{
	DPRINT("disp_get_power\n");

	return mfd->panel_power_on;
}

static struct lcd_ops display_props = {
	.get_power = disp_get_power,
	.set_power = disp_set_power,
};
/*set, get power end*/

void sysfs_set_default_cbfunc(int flag)
{
	if (flag) {
		DPRINT("Change the sysfs callback function as default");
		display_props.set_power = default_disp_set_power;
		display_props.get_power = default_disp_get_power;
	}
}

int samsung_display_sysfs_create(
					struct platform_device *pdev,
					struct platform_device *msm_fb_dev,
					char *panel_name)
{
	int ret;
	struct lcd_device *lcd_device;

	if ((!pdev) || (!msm_fb_dev))
		return -ENODEV;
	if (!panel_name)
		return -EFAULT;

	set_panel_name(panel_name, strnlen(panel_name, MAX_PANEL_NAME));
	get_platform_data(msm_fb_dev);

	lcd_device = lcd_device_register("panel", &pdev->dev,
					NULL, &display_props);

	if (IS_ERR(lcd_device)) {
		ret = PTR_ERR(lcd_device);
		printk(KERN_ERR "lcd : failed to register device\n");
		return ret;
	}

	ret = sysfs_create_file(&lcd_device->dev.kobj,
					&dev_attr_lcd_type.attr);
	if (ret)
		printk(KERN_ERR "sysfs create fail - %s\n",
		dev_attr_lcd_type.attr.name);

	return 0;
}
