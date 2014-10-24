/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <linux/lcd.h>
#include <linux/mfd/pmic8058.h>
/*yinbo.xie@ 20120822 #include <mach/gpio.h>*/
#include <linux/gpio.h>
#include <linux/semaphore.h>
#include "msm_fb.h"
/*#include "../../../arch/arm/mach-msm/smd_private.h"*/

#include "lcdc_backlight_ic.h"
#include "mdnie_sysfs.h"
#ifdef CONFIG_SAMSUNG_DISPLAY_SYSFS
#include "samsung_display_sysfs.h"
#endif

#define LCDC_DEBUG
/*#define USE_PREVAIL2*/

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_INFO "LCD(r69329) " x)
#else
#define DPRINT(x...)
#endif

static int spi_cs;
static int spi_sclk;
static int spi_sdi;
/*static int spi_dac;*/
static int lcd_reset;
static int spi_sdo;

/*#define FEATURE_LCD_ESD_DET*/


/*#define ESD_RECOVERY*/
#ifdef ESD_RECOVERY
static irqreturn_t trebon_disp_breakdown_det(int irq, void *handle);
static void lcdc_dsip_reset_work(struct work_struct *work_ptr);
#define GPIO_ESD_DET	39
static unsigned int lcd_det_irq;
static struct delayed_work lcd_reset_work;
static boolean irq_disabled = FALSE;
static boolean wa_first_irq = FALSE;
#endif


struct r69329_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
	boolean disp_cabc;
	boolean	force_backlight_on;
};

static struct r69329_state_type r69329_state = { 0 };
static struct msm_panel_common_pdata *lcdc_r69329_pdata;

static int lcd_prf;

static DEFINE_SEMAPHORE(backlight_sem);
static DEFINE_MUTEX(spi_mutex);

#define PMIC_GPIO_LCD_BL_CTRL	PM8058_GPIO_PM_TO_SYS(PM8058_GPIO(29))
#define LCD_BL_CTRL(value)	gpio_set_value(144, value)

/*
 * Serial Interface
 */

#define DEFAULT_USLEEP	1

struct spi_cmd_desc {
	int dlen;
	char *payload;
	int wait;
};

static char sleep_out_seq[1] = { 0x11 };
static char disp_on_seq[1] = { 0x29 };
/*static char disp_off_seq[1] = { 0x28 };*/
static char sleep_in_seq[1] = { 0x10 };
/*static char sw_reset_seq[1] = { 0x01 };*/

static char set_negative_on[1] = { 0x21 };
static char set_negative_off[1] = { 0x20 };

static char set_cabc_on[6] = {
	0xB8,
	0x01, 0x1A, 0x18, 0x02, 0x40
};

static char set_cabc_off[2] = {
	0xB8,
	0x00
};

static struct spi_cmd_desc set_negative_on_cmd[] = {
	{sizeof(set_negative_on), set_negative_on, 0},
};

static struct spi_cmd_desc set_negative_off_cmd[] = {
	{sizeof(set_negative_off), set_negative_off, 0},
};

static struct spi_cmd_desc set_cabc_on_cmd[] = {
	{sizeof(set_cabc_on), set_cabc_on, 0},
};

static struct spi_cmd_desc set_cabc_off_cmd[] = {
	{sizeof(set_cabc_off), set_cabc_off, 0},
};


#ifdef PWM_CABC
static char initial_setting_seq1[2] = {
	0xB0,
	0x04
};

static char initial_setting_seq2[6] = {
	0xC1,
	0x00, 0x03, 0x22, 0x11, 0x03
};

static char initial_setting_seq3[5] = {
	0xFD,
	0x20, 0x30, 0x20, 0x10
};

static char initial_setting_seq4[2] = {
	0x36,
	0x8A/*88*/
};

static char initial_setting_seq5[7] = {
	0xCE,
	0x05, 0x84, 0x00, 0xFF, 0x0A,
	0x01
};

static char initial_setting_seq6[10] = {
	0xBD,
	0xC1, 0x05, 0x00, 0x05, 0x00,
	0x02, 0x00, 0x05, 0x03
};

static char initial_setting_seq7[4] = {
	0xBE,
	0xF0, 0x04, 0x14
};

static char initial_setting_seq8[10] = {
	0xC0,
	0x00, 0x1F, 0x03, 0x6E, 0x01,
	0x02, 0x0A, 0x0A, 0x02,
};

static char initial_setting_seq9[5] = {
	0xC2,
	0x0F, 0x04, 0x0A, 0x0A,
};

static char initial_setting_seq10[6] = {
	0xC1,
	0x00, 0x83, 0x22, 0x11, 0x03
};

static char initial_setting_seq11[7] = {
	0xD0,
	0x54, 0x19, 0xAA, 0xC7, 0x8B,
	0x8E
};

static char initial_setting_seq12[15] = {
	0xD1,
	0x0D, 0x11, 0x02, 0x22, 0x32,
	0x03, 0x22, 0x43, 0x06, 0x77,
	0x87, 0x06, 0x77, 0x97
};

static char initial_setting_seq13[3] = {
	0xD4,
	0x0F, 0x0E
};

static char initial_setting_seq14[3] = {
	0xD5,
	0x34, 0x34
};
/*
* CABC Setting
* ON : 0x01, OFF : 0x00
static char initial_setting_seq15[2] = {
	0xB8,
	0x00
};
*/
static char initial_setting_seq16[2] = {
	0x53,
	0x2C
};

static char red28_gamma_setting_seq[27] = {
	0xC8,
	0x11, 0x12, 0x1E, 0x2A, 0x39,
	0x51, 0x61, 0x33, 0x24, 0x15,
	0x13, 0x12, 0x02, 0x11, 0x12,
	0x1E, 0x2A, 0x39, 0x51, 0x61,
	0x33, 0x24, 0x15, 0x13, 0x12,
	0x02
};

static char green28_gamma_setting_seq[27] = {
	0xC9,
	0x00, 0x18, 0x21, 0x2C, 0x3A,
	0x52, 0x61, 0x32, 0x24, 0x14,
	0x13, 0x12, 0x02, 0x00, 0x18,
	0x21, 0x2C, 0x3A, 0x52, 0x61,
	0x32, 0x24, 0x14, 0x13, 0x12,
	0x02
};

static char blue28_gamma_setting_seq[27] = {
	0xCA,
	0x00, 0x24, 0x2A, 0x31, 0x3D,
	0x53, 0x62, 0x32, 0x24, 0x12,
	0x11, 0x10, 0x02, 0x00, 0x24,
	0x2A, 0x31, 0x3D, 0x53, 0x62,
	0x32, 0x24, 0x12, 0x11, 0x10,
	0x02
};
/*
static char sleep_mode_setting_seq[2] = {
	0xB0,
	0x04
};

static char deep_standby_mode_setting_seq[2] = {
	0xB1,
	0x01
};
*/

static char disp_on_seq1[5] = {
	0xFD,
	0x00, 0x00, 0x00, 0x00
};

static char disp_on_seq2[6] = {
	0xC1,
	0x00, 0x83, 0x22, 0x11, 0x03
};

static char set_negative[1] = {
	0x20
};

static char set_cabc[6] = {
	0xB8,
	0x00, 0x1A, 0x18, 0x02, 0x40
};

static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(sleep_out_seq), sleep_out_seq, 40},

	{sizeof(initial_setting_seq1), initial_setting_seq1, 0},
	{sizeof(initial_setting_seq2), initial_setting_seq2, 0},
	{sizeof(initial_setting_seq3), initial_setting_seq3, 60},
	{sizeof(initial_setting_seq4), initial_setting_seq4, 0},
	{sizeof(initial_setting_seq5), initial_setting_seq5, 0},
	{sizeof(initial_setting_seq6), initial_setting_seq6, 0},
	{sizeof(initial_setting_seq7), initial_setting_seq7, 0},
	{sizeof(initial_setting_seq8), initial_setting_seq8, 0},
	{sizeof(initial_setting_seq9), initial_setting_seq9, 0},
	{sizeof(initial_setting_seq10), initial_setting_seq10, 0},

	{sizeof(red28_gamma_setting_seq), red28_gamma_setting_seq, 0},
	{sizeof(green28_gamma_setting_seq), green28_gamma_setting_seq, 0},
	{sizeof(blue28_gamma_setting_seq), blue28_gamma_setting_seq, 0},

	{sizeof(initial_setting_seq11), initial_setting_seq11, 0},
	{sizeof(initial_setting_seq12), initial_setting_seq12, 0},
	{sizeof(initial_setting_seq13), initial_setting_seq13, 0},
	{sizeof(initial_setting_seq14), initial_setting_seq14, 0},
	/*{sizeof(initial_setting_seq15), initial_setting_seq15, 0}, CABC*/
	{sizeof(set_cabc), set_cabc, 0},
	{sizeof(initial_setting_seq16), initial_setting_seq16, 0},
	{sizeof(set_negative), set_negative, 0},

	{sizeof(disp_on_seq), disp_on_seq, 0},
	{sizeof(disp_on_seq1), disp_on_seq1, 0},
	{sizeof(disp_on_seq2), disp_on_seq2, 0},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(sleep_in_seq), sleep_in_seq, 80},
};

static char set_brightness_max[3] = {
	0x51,
	0x0D, 0x34
};

static char set_brightness_min[3] = {
	0x51,
	0x00, 0x00
};

static char backlight_level[3] = {
	0x51,
	0x00, 0x00
};

static struct spi_cmd_desc backlight_on_cmd[] = {
	{sizeof(set_brightness_max), set_brightness_max, 1},
};

static struct spi_cmd_desc backlight_off_cmd[] = {
	{sizeof(set_brightness_min), set_brightness_min, 1},
};

static struct spi_cmd_desc set_backlight_cmd[] = {
	{sizeof(backlight_level), backlight_level, 1},
};
#else

/*
* Seq for ictl panel
*/
static char initial_setting_seq1[2] = {
	0xB0,
	0x04
};

static char initial_setting_seq2[2] = {
	0x36,
	0x48/*0x08*/
};

static char initial_setting_seq3[10] = {
	0xBD,
	0xC1, 0x05, 0x00, 0x05, 0x00,
	0x02, 0x00, 0x05, 0x00
};

static char initial_setting_seq4[4] = {
	0xBE,
	0xF0, 0x04, 0x14
};

static char initial_setting_seq5[10] = {
	0xC0,
	0x05, 0x1F, 0x03, 0x06, 0x00,
	0x02, 0x0A, 0x0A, 0x02,
};

static char initial_setting_seq6[5] = {
	0xC2,
	0x0F, 0x01, 0x0A, 0x0A,
};

static char initial_setting_seq7[6] = {
	0xC1,
	0x01, 0x00, 0x00, 0x00, 0x03
};

static char initial_setting_seq8[27] = {
	0xC8,
	0x15, 0x19, 0x1F, 0x24, 0x36,
	0x4C, 0x5C, 0x39, 0x2A, 0x16,
	0x0D, 0x07, 0x02, 0x15, 0x19,
	0x1F, 0x24, 0x36, 0x4C, 0x5C,
	0x39, 0x2A, 0x16, 0x0D, 0x07,
	0x02
};

static char initial_setting_seq9[7] = {
	0xD0,
	0x14, 0x19, 0x55, 0x4B, 0x88,
	0x8B
};

static char initial_setting_seq10[15] = {
	0xD1,
	0x0D, 0x11, 0x06, 0x77, 0x77,
	0x06, 0x77, 0x77, 0x06, 0x31,
	0x75, 0x06, 0x77, 0x77
};

static char initial_setting_seq11[3] = {
	0xD4,
	0x0F, 0x07
};

static char initial_setting_seq12[3] = {
	0xD5,
	0x29, 0x29
};

static char initial_setting_seq13[4] = {
	0xDE,
	0x03, 0x6E, 0x65
};

static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(sleep_out_seq), sleep_out_seq, 200},

	{sizeof(initial_setting_seq1), initial_setting_seq1, 0},
	{sizeof(initial_setting_seq2), initial_setting_seq2, 0},
	{sizeof(initial_setting_seq3), initial_setting_seq3, 0},
	{sizeof(initial_setting_seq4), initial_setting_seq4, 0},
	{sizeof(initial_setting_seq5), initial_setting_seq5, 0},
	{sizeof(initial_setting_seq6), initial_setting_seq6, 0},
	{sizeof(initial_setting_seq7), initial_setting_seq7, 0},
	{sizeof(initial_setting_seq8), initial_setting_seq8, 0},
	{sizeof(initial_setting_seq9), initial_setting_seq9, 0},
	{sizeof(initial_setting_seq10), initial_setting_seq10, 0},
	{sizeof(initial_setting_seq11), initial_setting_seq11, 0},
	{sizeof(initial_setting_seq12), initial_setting_seq12, 0},
	{sizeof(initial_setting_seq13), initial_setting_seq13, 0},

	{sizeof(disp_on_seq), disp_on_seq, 200},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 1},
	{sizeof(sleep_in_seq), sleep_in_seq, 100},
};
/*
static struct spi_cmd_desc display_standby_in_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 150},
	{sizeof(sleep_in_seq), sleep_in_seq, 150},
};

static struct spi_cmd_desc display_standby_out_cmds[] = {
	{sizeof(sleep_out_seq), sleep_out_seq, 0},
	{sizeof(disp_on_seq), disp_on_seq, 0},
};*/
#endif

static int lcdc_r69329_panel_off(struct platform_device *pdev);

static void spi_cmds_tx(struct spi_cmd_desc *desc, int cnt)
{
	long i, j, p;
	/*unsigned long irqflags;*/

	mutex_lock(&spi_mutex);
	for (p = 0; p < cnt; p++) {
		gpio_set_value(spi_cs, 1);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sclk, 1);
		udelay(DEFAULT_USLEEP);

		/* Write Command */
		gpio_set_value(spi_cs, 0);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sclk, 0);
		udelay(DEFAULT_USLEEP);
		gpio_set_value(spi_sdi, 0);
		udelay(DEFAULT_USLEEP);

		gpio_set_value(spi_sclk, 1);
		udelay(DEFAULT_USLEEP);

		for (i = 7; i >= 0; i--) {
			gpio_set_value(spi_sclk, 0);
			udelay(DEFAULT_USLEEP);
			if (((char)*(desc+p)->payload >> i) & 0x1)
				gpio_set_value(spi_sdi, 1);
			else
				gpio_set_value(spi_sdi, 0);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sclk, 1);
			udelay(DEFAULT_USLEEP);
		}

		gpio_set_value(spi_cs, 1);
		udelay(DEFAULT_USLEEP);

		/* Write Parameter */
		if ((desc+p)->dlen < 2)
			goto tx_done;

		for (j = 1; j < (desc+p)->dlen; j++) {
			gpio_set_value(spi_cs, 0);
			udelay(DEFAULT_USLEEP);

			gpio_set_value(spi_sclk, 0);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sdi, 1);
			udelay(DEFAULT_USLEEP);
			gpio_set_value(spi_sclk, 1);
			udelay(DEFAULT_USLEEP);

			for (i = 7; i >= 0; i--) {
				gpio_set_value(spi_sclk, 0);
				udelay(DEFAULT_USLEEP);
				if (((char)*((desc+p)->payload+j) >> i) & 0x1)
					gpio_set_value(spi_sdi, 1);
				else
					gpio_set_value(spi_sdi, 0);
				udelay(DEFAULT_USLEEP);
				gpio_set_value(spi_sclk, 1);
				udelay(DEFAULT_USLEEP);
			}

			gpio_set_value(spi_cs, 1);
			udelay(DEFAULT_USLEEP);
		}
tx_done:
		if ((desc+p)->wait)
			msleep((desc+p)->wait);
	}
	mutex_unlock(&spi_mutex);
}

static void spi_init(void)
{
	/* Setting the Default GPIO's */
	spi_sclk = *(lcdc_r69329_pdata->gpio_num);
	spi_cs   = *(lcdc_r69329_pdata->gpio_num + 1);
	spi_sdi  = *(lcdc_r69329_pdata->gpio_num + 2);
	lcd_reset  = *(lcdc_r69329_pdata->gpio_num + 3);
	spi_sdo  = *(lcdc_r69329_pdata->gpio_num + 4);


	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, 0);

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 0);

	gpio_set_value(spi_sdo, 0);
}

static void lcdc_panel_reset(int on)
{
	DPRINT("start %s, lcd_reset flag : %d\n", __func__, on);
	if (on) {
		gpio_set_value(lcd_reset, 0);
		usleep(2000);
		gpio_set_value(lcd_reset, 1);
		usleep(10000);
	} else {
		gpio_set_value(lcd_reset, 0);
	}
}

static void r69329_disp_powerup(void)
{
	DPRINT("start %s, lcd_reset:gpio %d\n", __func__, lcd_reset);

	if (!r69329_state.disp_powered_up && !r69329_state.display_on) {
		/* Reset the hardware first */

		lcdc_panel_reset(1);
		/* Include DAC power up implementation here */

	    r69329_state.disp_powered_up = TRUE;
	}
}

static void r69329_disp_powerdown(void)
{
	DPRINT("start %s, lcd_reset:gpio %d\n", __func__, lcd_reset);

	/* turn off LDO */
	/*TODO: turn off LDO*/

	r69329_state.disp_powered_up = FALSE;
}

void r69329_disp_on(void)
{
	/*int i;*/
	pr_info("@LCDINIT@:DISPON sequence\n");
	/*DPRINT("start %s\n", __func__);*/

	if (r69329_state.disp_powered_up && !r69329_state.display_on) {
		/*mdelay(20);*/

		/* r69329 setting */
		spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));

		r69329_state.display_on = TRUE;
	}
}

void r69329_sleep_off(void)
{
	/*int i = 0;*/
	DPRINT("start %s\n", __func__);

	spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));
}

void r69329_sleep_in(void)
{
	/*int i = 0;*/
	DPRINT("start %s\n", __func__);

	spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));

	lcd_prf = 0;
}

static void r69329_set_backlight(int level)
{
	int flag = !!level;

	if (flag)
		spi_cmds_tx(backlight_on_cmd, ARRAY_SIZE(backlight_on_cmd));
	else
		spi_cmds_tx(backlight_off_cmd, ARRAY_SIZE(backlight_off_cmd));
}

static int lcdc_r69329_panel_on(struct platform_device *pdev)
{
	static int bring_up_condition = 1;
	/*unsigned size;*/
	msleep(20);

	DPRINT("start %s, bring_up %d, disp_initialized %d\n",
		__func__, bring_up_condition, r69329_state.disp_initialized);

	if (!bring_up_condition) {
		/* trick initalization for timing issue */
		bring_up_condition = 1;

		/* Configure reset GPIO that drives DAC */
		spi_init();	/* LCD needs SPI */
		lcdc_r69329_pdata->panel_config_gpio(1);
		r69329_state.disp_powered_up = TRUE;
		r69329_state.display_on = TRUE;
		r69329_state.disp_initialized = TRUE;

		/* Recover the TSP_VSYNC */
		r69329_set_backlight(0x0);
		lcdc_r69329_panel_off(pdev);
		lcdc_r69329_panel_on(pdev);
		msleep(70);
		r69329_set_backlight(0xFF);
	} else {
		if (!r69329_state.disp_initialized) {
			/* Configure reset GPIO that drives DAC */
			spi_init();	/* LCD needs SPI */
			r69329_disp_powerup();
			lcdc_r69329_pdata->panel_config_gpio(1);
			r69329_disp_on();
			r69329_state.disp_initialized = TRUE;

			/*
			* This part working for recovery mode only
			*/
			if (r69329_state.force_backlight_on) {
				r69329_state.force_backlight_on = FALSE;
				DPRINT("%s : Panel on without backlight on\n",
							__func__);
				msleep(70);
				r69329_set_backlight(0xFF);
			}
#ifdef ESD_RECOVERY
			if (irq_disabled) {
				enable_irq(lcd_det_irq);
				irq_disabled = FALSE;
			}
#endif
		}
	}

	return 0;
}

static int lcdc_r69329_panel_off(struct platform_device *pdev)
{
	/*int i;*/

	DPRINT("start %s\n", __func__);


	if (r69329_state.disp_powered_up && r69329_state.display_on) {
#ifdef ESD_RECOVERY
		disable_irq_nosync(lcd_det_irq);
		irq_disabled = TRUE;
#endif
		/*
		* This part working for recovery mode only
		*/
		if (!lcd_prf) {
			DPRINT("%s : Panel off without backlight off\n",
						__func__);
			r69329_set_backlight(0xFF);
			r69329_state.force_backlight_on = TRUE;
		}

		spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));


		lcdc_r69329_pdata->panel_config_gpio(0);
		r69329_state.display_on = FALSE;
		r69329_state.disp_initialized = FALSE;
		r69329_disp_powerdown();
		lcd_prf = 0;
	}

	return 0;
}

int apply_negative_value_r69329(enum eNegative_Mode negative_mode)
{
	if (negative_mode == NEGATIVE_ON_MODE) {
		spi_cmds_tx(set_negative_on_cmd,
					ARRAY_SIZE(set_negative_on_cmd));
		set_negative[0] = set_negative_on[0];
	} else {
		spi_cmds_tx(set_negative_off_cmd,
					ARRAY_SIZE(set_negative_off_cmd));
		set_negative[0] = set_negative_off[0];
		if (r69329_state.disp_cabc == TRUE)
			spi_cmds_tx(set_cabc_on_cmd,
					ARRAY_SIZE(set_cabc_on_cmd));
		else
			spi_cmds_tx(set_cabc_off_cmd,
					ARRAY_SIZE(set_cabc_off_cmd));
	}
	return 0;
}

int apply_cabc_value_r69329(enum eCabc_Mode cabc_mode)
{
	if (cabc_mode == CABC_ON_MODE) {
		spi_cmds_tx(set_cabc_on_cmd, ARRAY_SIZE(set_cabc_on_cmd));
		set_cabc[1] = set_cabc_on[1];
		r69329_state.disp_cabc = TRUE;
	} else {
		spi_cmds_tx(set_cabc_off_cmd, ARRAY_SIZE(set_cabc_off_cmd));
		set_cabc[1] = set_cabc_off[1];
		r69329_state.disp_cabc = FALSE;
	}
	return 0;
}


static void lcdc_r69329_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_value = mfd->bl_level;
	static int lockup_count;
	static int pwm_level;

	up(&backlight_sem);
	DPRINT("[BACKLIGHT] : %d\n", bl_value);
	if (!bl_value) {
		/*  Turn off Backlight, don't check disp_initialized value */
		lcd_prf = 1;

	} else {
		if (lcd_prf)
			return;

		while (!r69329_state.disp_initialized) {
			msleep(100);
			lockup_count++;

			if (lockup_count > 50) {
				printk(KERN_ERR "Prevent infinite loop(wait for 5s)\n");
				printk(KERN_ERR "LCD can't initialize with in %d ms\n"
					, lockup_count*100);
				lockup_count = 0;

				down(&backlight_sem);
				return;
			}
		}
	}
	pwm_level = backlight_ic_set_brightness(bl_value);

	backlight_ic_set_brightness(bl_value);

	backlight_level[1] = (pwm_level & 0xF00) >> 8;
	backlight_level[2] = (pwm_level & 0xFF);
	spi_cmds_tx(set_backlight_cmd, ARRAY_SIZE(set_backlight_cmd));

	down(&backlight_sem);
}

static int __devinit r69329_probe(struct platform_device *pdev)
{
	/*int err;*/
	int ret = 0;
	struct mdnie_ops p_mdnie_ops;

#ifdef CONFIG_SAMSUNG_DISPLAY_SYSFS
	static struct platform_device *msm_fb_added_dev;
#endif
	/*unsigned size;*/
	DPRINT("start %s\n", __func__);

	r69329_state.disp_initialized = FALSE; /*signal_timing*/
	r69329_state.disp_powered_up = FALSE;
	r69329_state.display_on = FALSE;
	r69329_state.force_backlight_on = TRUE;

	if (pdev->id == 0) {
		lcdc_r69329_pdata = pdev->dev.platform_data;
		if (!lcdc_r69329_pdata)
			return -EINVAL;

#ifdef ESD_RECOVERY
		gpio_tlmm_config(GPIO_CFG(GPIO_ESD_DET,  0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		lcd_det_irq = MSM_GPIO_TO_INT(GPIO_ESD_DET);
		if (!lcd_det_irq)
			DPRINT("LCD_DETECT_IRQ is NULL!\n");

		INIT_DELAYED_WORK(&lcd_reset_work, lcdc_dsip_reset_work);
		ret = request_irq(lcd_det_irq, trebon_disp_breakdown_det,
		IRQF_TRIGGER_RISING, "lcd_esd_det", NULL);

		if (ret) {
			pr_err("Request_irq failed for TLMM_MSM_SUMMARY_IRQ - %d\n",
				ret);
			return ret;
		}
#endif
		return ret;
	}
#ifdef CONFIG_SAMSUNG_DISPLAY_SYSFS
	msm_fb_added_dev = msm_fb_add_device(pdev);
	samsung_display_sysfs_create(pdev, msm_fb_added_dev, "ICTL");
#else
	msm_fb_add_device(pdev);
#endif

	p_mdnie_ops.apply_negative_value = apply_negative_value_r69329;
	p_mdnie_ops.apply_cabc_value = apply_cabc_value_r69329;
	mdnie_sysfs_init(&p_mdnie_ops);

	return ret;
}

static void r69329_shutdown(struct platform_device *pdev)
{
	DPRINT("start %s\n", __func__);

	lcdc_r69329_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = r69329_probe,
	.shutdown	= r69329_shutdown,
	.driver = {
		.name   = "lcdc_r69329_wvga",
	},
};

static struct msm_fb_panel_data r69329_panel_data = {
	.on = lcdc_r69329_panel_on,
	.off = lcdc_r69329_panel_off,
	.set_backlight = lcdc_r69329_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_r69329_wvga",
	.id	= 1,
	.dev	= {
		.platform_data = &r69329_panel_data,
	}
};
#ifdef PWM_CABC
#define LCDC_FB_XRES	480
#define LCDC_FB_YRES	800
#define LCDC_HBP		20
#define LCDC_HPW		4
#define LCDC_HFP		100
#define LCDC_VBP		116
#define LCDC_VPW		250
#define LCDC_VFP		250
#else
#define LCDC_FB_XRES	480
#define LCDC_FB_YRES	800
#define LCDC_HBP		20
#define LCDC_HPW		4
#define LCDC_HFP		100
#define LCDC_VBP		4
#define LCDC_VPW		2
#define LCDC_VFP		350
#endif
static int __init lcdc_r69329_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_LCDC_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_r69329_wvga")) {
		printk(KERN_ERR "%s: detect another lcd driver!\n", __func__);
		return 0;
	}
#endif
	DPRINT("start %s\n", __func__);

	update_backlight_table(BACKLIGHT_R69329);

	ret = platform_driver_register(&this_driver);
	if (ret) {
		printk(KERN_ERR "%s: platform_driver_register failed! ret=%d\n",
						__func__, ret);
		return ret;
	}

	pinfo = &r69329_panel_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 45100000;/*48000000;*/
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;

	pinfo->lcdc.h_back_porch = LCDC_HBP;
	pinfo->lcdc.h_front_porch = LCDC_HFP;
	pinfo->lcdc.h_pulse_width = LCDC_HPW;
	pinfo->lcdc.v_back_porch = LCDC_VBP;
	pinfo->lcdc.v_front_porch = LCDC_VFP;
	pinfo->lcdc.v_pulse_width = LCDC_VPW;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret) {
		printk(KERN_ERR "%s: platform_device_register failed! ret=%d\n",
						__func__, ret);
		platform_driver_unregister(&this_driver);
	}

	return ret;
}

#ifdef ESD_RECOVERY
static irqreturn_t trebon_disp_breakdown_det(int irq, void *handle)
{
	if (nt35510_state.disp_initialized)
		schedule_delayed_work(&lcd_reset_work, 0);

	return IRQ_HANDLED;
}

static void lcdc_dsip_reset_work(struct work_struct *work_ptr)
{
	if (!wa_first_irq) {
		DPRINT("skip lcd reset\n");
		wa_first_irq = TRUE;
		return;
	}

	DPRINT("lcd reset\n");

	lcdc_r69329_panel_off(NULL);

	lcdc_r69329_panel_on(NULL);
}
#endif

module_init(lcdc_r69329_panel_init);
