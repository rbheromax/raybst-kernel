/*
 * ray-cam-gpio.h
 *
 * header file supporting gpio functions for Samsung device
 *
 * COPYRIGHT(C) Samsung Electronics Co., Ltd. 2006-2011 All Right Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */
#include <linux/mfd/pmic8058.h>

/* MSM8655 Ray Camera GPIO */
#define PMIC_GPIO_CAM_FLASH_EN  PM8058_GPIO(28)
#define PMIC_GPIO_CAM_FLASH_SET PM8058_GPIO(27)
#define GPIO_MSM_FLASH_CNTL_EN	PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_EN)
#define GPIO_MSM_FLASH_CNTL_SET	PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_CAM_FLASH_SET)
#define GPIO_SUB_CAM_MCLK		15
#define GPIO_MAIN_CAM_MCLK		15	/* CAM_MCLK */
#define GPIO_VT_STBY			2	/* CAM_VT_nSTBY */
#define GPIO_I2C_DATA_CAM		17	/* CAM_I2C_SDA */
#define GPIO_I2C_CLK_CAM		30	/* CAM_I2C_SCL */
#define GPIO_CAM_A_EN			3	/* CAM_EN */
#define GPIO_CAM_IO_EN			143	/* CAM_EN_1 */
#define GPIO_CAM_CORE_EN		132	/* CAM_EN_2 */
#define GPIO_MAIN_STBY			131	/* CAM_nSTBY */
#define GPIO_CAM2_RST_N			175	/* CAM_VT_RST */
#define GPIO_CAM1_RST_N			130 /* CAM_nRST */

#define CAM_EN			3
#define CAM_EN_1		143
#define CAM_EN_2		132
#define CAM_nRST		130
#define CAM_nSTBY		131
#define CAM_I2C_SCL		30
#define CAM_I2C_SDA		17
#define CAM_VT_nSTBY		2
#define CAM_VT_RST		175
#define CAM_MCLK		15

void msm_camera_gpio_install(void);
void cam_ldo_power_on(int mode);
void cam_ldo_power_off(int mode);
extern int rear_sensor_check_vendorID_before(unsigned char *vendorID);
extern int rear_sensor_check_vendorID_done();

