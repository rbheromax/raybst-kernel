#ifndef SAMSUNG_DISPLAY_SYSFS_H
#define SAMSUNG_DISPLAY_SYSFS_H

#include <linux/lcd.h>
#include "msm_fb.h"

#define MAX_PANEL_NAME 20
int samsung_display_sysfs_create(
					struct platform_device *pdev,
					struct platform_device *msm_fb_dev,
					char *panel_name);
void sysfs_set_default_cbfunc(int flag);
int  check_recovery_mode(char *recovery_mode);
#endif
