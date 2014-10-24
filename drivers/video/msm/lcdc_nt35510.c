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
#ifdef CONFIG_SAMSUNG_MDNIE_SYSFS
#include "mdnie_sysfs.h"
#endif
#ifdef CONFIG_SAMSUNG_DISPLAY_SYSFS
#include "samsung_display_sysfs.h"
#endif

#define LCDC_DEBUG
#define LCDC_NOVATEC_LDI

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk(KERN_INFO "LCD(NT35510) " x)
#else
#define DPRINT(x...)
#endif

static int spi_cs;
static int spi_sclk;
static int spi_sdi;
/*static int spi_dac;*/
static int lcd_reset;
static int spi_sdo;

/*
* #define FEATURE_LCD_ESD_DET*
* #define ESD_RECOVERY
*/
#ifdef ESD_RECOVERY
static irqreturn_t samsung_disp_breakdown_det(int irq, void *handle);
static void lcdc_dsip_reset_work(struct work_struct *work_ptr);
#define GPIO_ESD_DET	39
static unsigned int lcd_det_irq;
static struct delayed_work lcd_reset_work;
static boolean irq_disabled = FALSE;
static boolean wa_first_irq = FALSE;
#endif

struct lcdc_state_type {
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
	boolean disp_cabc;
	boolean	force_backlight_on;
	boolean recovery_mode;
};

static struct lcdc_state_type lcdc_state = { 0 };
static struct msm_panel_common_pdata *lcdc_pdata;
static int lcd_prf;

static DEFINE_SEMAPHORE(backlight_sem);
static DEFINE_MUTEX(spi_mutex);

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
static char disp_off_seq[1] = { 0x28 };
static char sleep_in_seq[1] = { 0x10 };
/* static char sw_reset_seq[1] = { 0x01 }; */
static char set_negative_on[1] = { 0x21 };
static char set_negative_off[1] = { 0x20 };

static char set_cabc_on[2] = {
	0x55,
	0x03
};

static char set_cabc_off[2] = {
	0x55,
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

static char power_seq1[6] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08, 0x01
};

static char power_seq2[4] = {
	0xB0,
	0x09, 0x09, 0x09
};

static char power_seq3[4] = {
	0xB1,
	0x09, 0x09, 0x09
};

static char power_seq4[4] = {
	0xB3,
	0x05, 0x05, 0x05
};

static char power_seq5[4] = {
	0xB5,
	0x0B, 0x0B, 0x0B
};

static char power_seq6[4] = {
	0xB6,
	0x34, 0x34, 0x34
};

static char power_seq7[4] = {
	0xB7,
	0x24, 0x24, 0x24
};

static char power_seq8[4] = {
	0xB9,
	0x24, 0x24, 0x24
};

static char power_seq9[4] = {
	0xBA,
	0x24, 0x24, 0x24
};

static char power_seq10[4] = {
	0xBC,
	0x00, 0xA3, 0x00
};

static char power_seq11[4] = {
	0xBD,
	0x00, 0xA3, 0x00
};

static char power_seq12[2] = {
	0xBF,
	0x01
};

static char init_seq1[6] = {
	0xF0,
	0x55, 0xAA, 0x52, 0x08, 0x00
};

static char init_seq2[6] = {
	0xB0,
	0x08, 0x05, 0x05, 0x12, 0x05
};

static char init_seq3[3] = {
	0xB1,
	0x00, 0x06
};
/*
static char init_seq4[2] = {
	0xB3,
	0x00
};
*/
static char init_seq5[2] = {
	0xB6,
	0x0A
};

static char init_seq6[3] = {
	0xB7,
	0x00, 0x00
};

static char init_seq7[5] = {
	0xB8,
	0x01, 0x05, 0x05, 0x05
};

static char init_seq8[2] = {
	0xBA,
	0x01
};

static char init_seq9[4] = {
	0xBC,
	0x00, 0x00, 0x00
};

static char init_seq10[6] = {
	0xBD,
	0x01, 0x84, 0x07, 0x31, 0x00
};

static char init_seq11[4] = {
	0xCC,
	0x03, 0x00, 0x00
};

static char init_seq12[2] = {
	0x36,
	0x00
};

static char init_seq13[2] = {
	0x3A,
	0x77
};
static char init_seq14[2] = {
	0x51,
	0x00
};

static char init_seq15[2] = {
	0x53,
	0x24
};

static char init_seq16[11] = {
	0xE4,
	0xA1, 0xA1, 0xA1, 0x99, 0x95, 
	0x92, 0x8F, 0x8F, 0x8F, 0x8F	
};


/*Gamma Setting 2.2 */
static char gamma_seq1[53] = {
	/* Red+ gamma */
	0xD1,
	0x00, 0x27, 0x00, 0x47, 0x00,
	0x62, 0x00, 0x92, 0x00, 0xA2,
	0x00, 0xCE, 0x00, 0xEF, 0x01,
	0x23, 0x01, 0x4B, 0x01, 0x8B,
	0x01, 0xBB, 0x02, 0x09, 0x02,
	0x49, 0x02, 0x4B, 0x02, 0x82,
	0x02, 0xBC, 0x02, 0xDF, 0x03,
	0x12, 0x03, 0x3A, 0x03, 0x5A,
	0x03, 0x80, 0x03, 0x98, 0x03,
	0xB3, 0x03, 0xC4, 0x03, 0xC9,
	0x03, 0xEF
};

static char gamma_seq2[53] = {
	/* Red- gamma */
	0xD4,
	0x00, 0x27, 0x00, 0x47, 0x00,
	0x62, 0x00, 0x92, 0x00, 0xA2,
	0x00, 0xCE, 0x00, 0xEF, 0x01,
	0x23, 0x01, 0x4B, 0x01, 0x8B,
	0x01, 0xBB, 0x02, 0x09, 0x02,
	0x49, 0x02, 0x4B, 0x02, 0x82,
	0x02, 0xBC, 0x02, 0xDF, 0x03,
	0x12, 0x03, 0x3A, 0x03, 0x5A,
	0x03, 0x80, 0x03, 0x98, 0x03,
	0xB3, 0x03, 0xC4, 0x03, 0xC9,
	0x03, 0xEF
};

/*Gamma Setting 2.2 */
static char gamma_seq3[53] = {
	/* Green+ gamma */
	0xD2,
	0x00, 0x27, 0x00, 0x47, 0x00,
	0x62, 0x00, 0x92, 0x00, 0xA2,
	0x00, 0xCE, 0x00, 0xEF, 0x01,
	0x23, 0x01, 0x4B, 0x01, 0x8B,
	0x01, 0xBB, 0x02, 0x09, 0x02,
	0x49, 0x02, 0x4B, 0x02, 0x82,
	0x02, 0xBC, 0x02, 0xDF, 0x03,
	0x12, 0x03, 0x3A, 0x03, 0x5A,
	0x03, 0x80, 0x03, 0x98, 0x03,
	0xB3, 0x03, 0xC4, 0x03, 0xC9,
	0x03, 0xEF
};

static char gamma_seq4[53] = {
	/* Green- gamma */
	0xD5,
	0x00, 0x27, 0x00, 0x47, 0x00,
	0x62, 0x00, 0x92, 0x00, 0xA2,
	0x00, 0xCE, 0x00, 0xEF, 0x01,
	0x23, 0x01, 0x4B, 0x01, 0x8B,
	0x01, 0xBB, 0x02, 0x09, 0x02,
	0x49, 0x02, 0x4B, 0x02, 0x82,
	0x02, 0xBC, 0x02, 0xDF, 0x03,
	0x12, 0x03, 0x3A, 0x03, 0x5A,
	0x03, 0x80, 0x03, 0x98, 0x03,
	0xB3, 0x03, 0xC4, 0x03, 0xC9,
	0x03, 0xEF
};

/*Gamma Setting 2.2 */
static char gamma_seq5[53] = {
	/* Blue+ gamma */
	0xD3,
	0x00, 0x27, 0x00, 0x47, 0x00,
	0x62, 0x00, 0x92, 0x00, 0xA2,
	0x00, 0xCE, 0x00, 0xEF, 0x01,
	0x23, 0x01, 0x4B, 0x01, 0x8B,
	0x01, 0xBB, 0x02, 0x09, 0x02,
	0x49, 0x02, 0x4B, 0x02, 0x82,
	0x02, 0xBC, 0x02, 0xDF, 0x03,
	0x12, 0x03, 0x3A, 0x03, 0x5A,
	0x03, 0x80, 0x03, 0x98, 0x03,
	0xB3, 0x03, 0xC4, 0x03, 0xC9,
	0x03, 0xEF
};

static char gamma_seq6[53] = {
	/* Blue- gamma */
	0xD6,
	0x00, 0x27, 0x00, 0x47, 0x00,
	0x62, 0x00, 0x92, 0x00, 0xA2,
	0x00, 0xCE, 0x00, 0xEF, 0x01,
	0x23, 0x01, 0x4B, 0x01, 0x8B,
	0x01, 0xBB, 0x02, 0x09, 0x02,
	0x49, 0x02, 0x4B, 0x02, 0x82,
	0x02, 0xBC, 0x02, 0xDF, 0x03,
	0x12, 0x03, 0x3A, 0x03, 0x5A,
	0x03, 0x80, 0x03, 0x98, 0x03,
	0xB3, 0x03, 0xC4, 0x03, 0xC9,
	0x03, 0xEF
};

static char set_cabc[2] = {
	0x55,
	0x00
};

static char set_negative[1] = {
	0x20
};

static struct spi_cmd_desc display_on_cmds[] = {
	{sizeof(power_seq1), power_seq1, 0},
	{sizeof(power_seq2), power_seq2, 0},
	{sizeof(power_seq3), power_seq3, 0},
	{sizeof(power_seq4), power_seq4, 0},
	{sizeof(power_seq5), power_seq5, 0},
	{sizeof(power_seq6), power_seq6, 0},
	{sizeof(power_seq7), power_seq7, 0},
	{sizeof(power_seq8), power_seq8, 0},
	{sizeof(power_seq9), power_seq9, 0},
	{sizeof(power_seq10), power_seq10, 0},
	{sizeof(power_seq11), power_seq11, 0},
	{sizeof(power_seq12), power_seq12, 50},

	{sizeof(gamma_seq1), gamma_seq1, 0},
	{sizeof(gamma_seq2), gamma_seq2, 0},
	{sizeof(gamma_seq3), gamma_seq3, 0},
	{sizeof(gamma_seq4), gamma_seq4, 0},
	{sizeof(gamma_seq5), gamma_seq5, 0},
	{sizeof(gamma_seq6), gamma_seq6, 0},

	{sizeof(init_seq1), init_seq1, 0},
	{sizeof(init_seq2), init_seq2, 0},
	{sizeof(init_seq3), init_seq3, 0},
	/*{sizeof(init_seq4), init_seq4, 0},*/
	{sizeof(init_seq5), init_seq5, 0},
	{sizeof(init_seq6), init_seq6, 0},
	{sizeof(init_seq7), init_seq7, 0},
	{sizeof(init_seq8), init_seq8, 0},
	{sizeof(init_seq9), init_seq9, 0},
	{sizeof(init_seq10), init_seq10, 0},
	{sizeof(init_seq11), init_seq11, 0},
	{sizeof(init_seq12), init_seq12, 0},
	{sizeof(init_seq13), init_seq13, 0},
	{sizeof(init_seq14), init_seq14, 0},
	{sizeof(init_seq15), init_seq15, 0},
	{sizeof(init_seq16), init_seq16, 0},
	{sizeof(set_negative), set_negative, 0},
	{sizeof(set_cabc), set_cabc, 0},
/*
	{sizeof(gamma_seq1), gamma_seq1, 0},
	{sizeof(gamma_seq2), gamma_seq2, 0},
	{sizeof(gamma_seq3), gamma_seq3, 0},
	{sizeof(gamma_seq4), gamma_seq4, 0},
	{sizeof(gamma_seq5), gamma_seq5, 0},
	{sizeof(gamma_seq6), gamma_seq6, 0},
*/
	{sizeof(sleep_out_seq), sleep_out_seq, 120},
	{sizeof(disp_on_seq), disp_on_seq, 200},
};

static struct spi_cmd_desc display_off_cmds[] = {
	{sizeof(disp_off_seq), disp_off_seq, 150},
	{sizeof(sleep_in_seq), sleep_in_seq, 150},
};

static char set_brightness_max[2] = {
	0x51,
	0xC3
};

static char set_brightness_min[2] = {
	0x51,
	0x00
};

static char backlight_level[2] = {
	0x51,
	0x00
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

/*
static char set_negative[1] = {
	0x20
};
*/

static int lcdc_samsung_panel_off(struct platform_device *pdev);

#ifdef LCDC_NOVATEC_LDI
/*
* This function used to send the commands and only used
* for novatek ldi on RGB interface. Because that have different
* register and parameter address if compare with another
* vender of ldi.
*/
static void spi_cmds_tx(struct spi_cmd_desc *desc, int cnt)
{
	long i, j, p;
	char count = 0;

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

		/* Write Parameter */
		if ((desc+p)->dlen < 2) {
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

			for (i = 7; i >= 0; i--) {
				gpio_set_value(spi_sclk, 0);
				udelay(DEFAULT_USLEEP);
				gpio_set_value(spi_sdi, 0);
				udelay(DEFAULT_USLEEP);
				gpio_set_value(spi_sclk, 1);
				udelay(DEFAULT_USLEEP);
			}

			gpio_set_value(spi_cs, 1);
			udelay(DEFAULT_USLEEP);
			goto tx_done;
		}

		for (j = 1; j < (desc+p)->dlen; j++) {

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

			for (i = 7; i >= 0; i--) {
				gpio_set_value(spi_sclk, 0);
				udelay(DEFAULT_USLEEP);
				if ((count >> i) & 0x1)
					gpio_set_value(spi_sdi, 1);
				else
					gpio_set_value(spi_sdi, 0);
				udelay(DEFAULT_USLEEP);
				gpio_set_value(spi_sclk, 1);
				udelay(DEFAULT_USLEEP);
			}
			count++;

			gpio_set_value(spi_cs, 1);
			udelay(DEFAULT_USLEEP);

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
#else
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
#endif

static void spi_init(void)
{
	/* Setting the Default GPIO's */
	spi_sclk = *(lcdc_pdata->gpio_num);
	spi_cs   = *(lcdc_pdata->gpio_num + 1);
	spi_sdi  = *(lcdc_pdata->gpio_num + 2);
	lcd_reset  = *(lcdc_pdata->gpio_num + 3);
	spi_sdo  = *(lcdc_pdata->gpio_num + 4);

	/* Set the output so that we dont disturb the slave device */
	gpio_set_value(spi_sclk, 0);
	gpio_set_value(spi_sdi, 0);

	/* Set the Chip Select De-asserted */
	gpio_set_value(spi_cs, 0);
	gpio_set_value(spi_sdo, 0);
}

static void lcdc_samsung_disp_reset(int on)
{
	DPRINT("%s : lcd_reset flag : %d\n", __func__, on);
	if (on) {
		msleep(50);
		gpio_set_value(lcd_reset, 0);
		usleep(2000);
		gpio_set_value(lcd_reset, 1);
		msleep(50);
	} else {
		gpio_set_value(lcd_reset, 0);
	}
}

static void lcdc_samsung_disp_powerup(void)
{
	DPRINT("%s : lcd_reset:gpio %d\n", __func__, lcd_reset);

	if (!lcdc_state.disp_powered_up && !lcdc_state.display_on) {
		/* Reset the hardware first */
		lcdc_samsung_disp_reset(1);

		/* Include DAC power up implementation here */

	    lcdc_state.disp_powered_up = TRUE;
	}
}

static void lcdc_samsung_disp_powerdown(void)
{
	DPRINT("%s : lcd_reset:gpio %d\n", __func__, lcd_reset);

	/* turn off LDO */
	/*TODO: turn off LDO*/

	lcdc_state.disp_powered_up = FALSE;
}

static void lcdc_samsung_disp_on(void)
{
	DPRINT("%s\n", __func__);

	if (lcdc_state.disp_powered_up && !lcdc_state.display_on) {

		/* lcdc setting */
		spi_cmds_tx(display_on_cmds, ARRAY_SIZE(display_on_cmds));

		lcdc_state.display_on = TRUE;
	}
}

static void samsung_set_backlight(int level)
{
	int flag = !!level;

	if (flag)
		spi_cmds_tx(backlight_on_cmd, ARRAY_SIZE(backlight_on_cmd));
	else
		spi_cmds_tx(backlight_off_cmd, ARRAY_SIZE(backlight_off_cmd));
}

static int lcdc_samsung_panel_on(struct platform_device *pdev)
{
	static int bring_up_condition;

	DPRINT("%s : bring_up %d, disp_initialized %d\n",
		__func__, bring_up_condition, lcdc_state.disp_initialized);

	if (!bring_up_condition) {
		/* trick initalization for timing issue */
		bring_up_condition = 1;

		/* Configure reset GPIO that drives DAC */
		spi_init();	/* LCD needs SPI */
		lcdc_samsung_disp_powerup();
		lcdc_pdata->panel_config_gpio(1);
		lcdc_samsung_disp_on();
		samsung_set_backlight(0xFF);

		lcdc_state.disp_initialized = TRUE;
	} else {
		if (!lcdc_state.disp_initialized) {
			/* Configure reset GPIO that drives DAC */
			spi_init();	/* LCD needs SPI */
			lcdc_samsung_disp_powerup();
			lcdc_pdata->panel_config_gpio(1);
			lcdc_samsung_disp_on();
			lcdc_state.disp_initialized = TRUE;

			/*
			* This part working for recovery mode only
			*/
			if (unlikely(lcdc_state.recovery_mode && \
					lcdc_state.force_backlight_on)) {
				lcdc_state.force_backlight_on = FALSE;
				DPRINT("%s : Display on without backlight on\n",
							__func__);
				msleep(70);
				samsung_set_backlight(0xFF);
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

static int lcdc_samsung_panel_off(struct platform_device *pdev)
{
	DPRINT("%s\n", __func__);

	if (lcdc_state.disp_powered_up && lcdc_state.display_on) {
#ifdef ESD_RECOVERY
		disable_irq_nosync(lcd_det_irq);
		irq_disabled = TRUE;
#endif
		/*
		* This part working for recovery mode only
		*/
		if (!lcd_prf) {
			DPRINT("%s : Display off without backlight off\n",
						__func__);
			samsung_set_backlight(0x00);
			lcdc_state.force_backlight_on = TRUE;
		}

		spi_cmds_tx(display_off_cmds, ARRAY_SIZE(display_off_cmds));

		lcdc_pdata->panel_config_gpio(0);
		lcdc_state.display_on = FALSE;
		lcdc_state.disp_initialized = FALSE;
		lcdc_samsung_disp_powerdown();
		lcd_prf = 0;
	}

	return 0;
}

#ifdef CONFIG_SAMSUNG_MDNIE_SYSFS
int apply_negative_value_nt35510(enum eNegative_Mode negative_mode)
{
	if (negative_mode == NEGATIVE_ON_MODE) {
		spi_cmds_tx(set_negative_on_cmd,
					ARRAY_SIZE(set_negative_on_cmd));
		set_negative[0] = set_negative_on[0];
	} else {
		spi_cmds_tx(set_negative_off_cmd,
					ARRAY_SIZE(set_negative_off_cmd));
		set_negative[0] = set_negative_off[0];
		if (lcdc_state.disp_cabc == TRUE)
			spi_cmds_tx(set_cabc_on_cmd,
					ARRAY_SIZE(set_cabc_on_cmd));
		else
			spi_cmds_tx(set_cabc_off_cmd,
					ARRAY_SIZE(set_cabc_off_cmd));
	}
	return 0;
}

int apply_cabc_value_nt35510(enum eCabc_Mode cabc_mode)
{
	if (cabc_mode == CABC_ON_MODE) {
		spi_cmds_tx(set_cabc_on_cmd, ARRAY_SIZE(set_cabc_on_cmd));
		set_cabc[1] = set_cabc_on[1];
		lcdc_state.disp_cabc = TRUE;
	} else {
		spi_cmds_tx(set_cabc_off_cmd, ARRAY_SIZE(set_cabc_off_cmd));
		set_cabc[1] = set_cabc_off[1];
		lcdc_state.disp_cabc = FALSE;
	}
	return 0;
}
#endif

static void lcdc_samsung_set_backlight(struct msm_fb_data_type *mfd)
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
		if (!lcdc_state.display_on)
			return;

		while (!lcdc_state.disp_initialized) {
			msleep(100);
			lockup_count++;

			if (lockup_count > 50) {
				DPRINT("Prevent infinite loop(wait for 5s)\n");
				DPRINT("LCD can't initialize with in %d ms\n",
						lockup_count*100);
				lockup_count = 0;

				down(&backlight_sem);
				return;
			}
		}
	}
	pwm_level = backlight_ic_set_brightness(bl_value);

	backlight_level[1] = (pwm_level & 0xFF);
	spi_cmds_tx(set_backlight_cmd, ARRAY_SIZE(set_backlight_cmd));

	down(&backlight_sem);
}

static int __devinit lcdc_samsung_probe(struct platform_device *pdev)
{
	int ret = 0;
#ifdef CONFIG_SAMSUNG_MDNIE_SYSFS
	struct mdnie_ops p_mdnie_ops;
#endif
#ifdef CONFIG_SAMSUNG_DISPLAY_SYSFS
	static struct platform_device *msm_fb_added_dev;
#endif
	/*unsigned size;*/
	DPRINT("%s\n", __func__);

	lcdc_state.disp_initialized = FALSE; /*signal_timing*/
	lcdc_state.disp_powered_up = FALSE;
	lcdc_state.display_on = FALSE;
	lcdc_state.force_backlight_on = TRUE;
	lcdc_state.recovery_mode = check_recovery_mode("GET") ? TRUE : FALSE;

	if (pdev->id == 0) {
		lcdc_pdata = pdev->dev.platform_data;
		if (!lcdc_pdata)
			return -EINVAL;

#ifdef ESD_RECOVERY
		gpio_tlmm_config(GPIO_CFG(GPIO_ESD_DET,  0, GPIO_CFG_INPUT,
			GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		lcd_det_irq = MSM_GPIO_TO_INT(GPIO_ESD_DET);
		if (!lcd_det_irq)
			DPRINT("%s : LCD_DETECT_IRQ is NULL!\n", __func__);

		INIT_DELAYED_WORK(&lcd_reset_work, lcdc_dsip_reset_work);
		ret = request_irq(lcd_det_irq, samsung_disp_breakdown_det,
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
	samsung_display_sysfs_create(pdev, msm_fb_added_dev, "BOE");
	sysfs_set_default_cbfunc(lcdc_state.recovery_mode);
#else
	msm_fb_add_device(pdev);
#endif

#ifdef CONFIG_SAMSUNG_MDNIE_SYSFS
	p_mdnie_ops.apply_negative_value = apply_negative_value_nt35510;
	p_mdnie_ops.apply_cabc_value = apply_cabc_value_nt35510;
	mdnie_sysfs_init(&p_mdnie_ops);
#endif

	return ret;
}

static void lcdc_samsung_shutdown(struct platform_device *pdev)
{
	DPRINT("%s\n", __func__);

	lcdc_samsung_panel_off(pdev);
}

static struct platform_driver this_driver = {
	.probe  = lcdc_samsung_probe,
	.shutdown	= lcdc_samsung_shutdown,
	.driver = {
		.name   = "lcdc_samsung_wvga",
	},
};

static struct msm_fb_panel_data lcdc_disp_data = {
	.on = lcdc_samsung_panel_on,
	.off = lcdc_samsung_panel_off,
	.set_backlight = lcdc_samsung_set_backlight,
};

static struct platform_device this_device = {
	.name   = "lcdc_samsung_wvga",
	.id	= 1,
	.dev	= {
		.platform_data = &lcdc_disp_data,
	}
};

#define LCDC_FB_XRES	480
#define LCDC_FB_YRES	800
#define LCDC_HBP		18
#define LCDC_HPW		2
#define LCDC_HFP		5
#define LCDC_VBP		5
#define LCDC_VPW		2
#define LCDC_VFP		5

static int __init lcdc_samsung_disp_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

#ifdef CONFIG_FB_MSM_LCDC_AUTO_DETECT
	if (msm_fb_detect_client("lcdc_nt35510_wvga")) {
		DPRINT("%s : detect another lcd driver!\n", __func__);
		return 0;
	}
#endif
	DPRINT("%s\n", __func__);

	update_backlight_table(BACKLIGHT_NT35510);

	ret = platform_driver_register(&this_driver);
	if (ret) {
		printk(KERN_ERR "%s: platform_driver_register failed! ret=%d\n",
						__func__, ret);
		return ret;
	}

	pinfo = &lcdc_disp_data.panel_info;
	pinfo->xres = LCDC_FB_XRES;
	pinfo->yres = LCDC_FB_YRES;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 24;
	pinfo->fb_num = 2;
	pinfo->clk_rate = 24576000;
	pinfo->bl_max = 255;
	pinfo->bl_min = 1;
#ifdef CONFIG_FB_MSM_DPI_ENABLE
	pinfo->width = 56;
	pinfo->height = 96;
#endif

	pinfo->lcdc.h_back_porch = LCDC_HBP;
	pinfo->lcdc.h_front_porch = LCDC_HFP;
	pinfo->lcdc.h_pulse_width = LCDC_HPW;
	pinfo->lcdc.v_back_porch = LCDC_VBP;
	pinfo->lcdc.v_front_porch = LCDC_VFP;
	pinfo->lcdc.v_pulse_width = LCDC_VPW;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xFF;       /* blue */
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
static irqreturn_t samsung_disp_breakdown_det(int irq, void *handle)
{
	if (lcdc_state.disp_initialized)
		schedule_delayed_work(&lcd_reset_work, 0);

	return IRQ_HANDLED;
}

static void lcdc_dsip_reset_work(struct work_struct *work_ptr)
{
	if (!wa_first_irq) {
		DPRINT("%s : skip lcd reset\n", __func__);
		wa_first_irq = TRUE;
		return;
	}

	DPRINT("%s : lcd reset\n", __func__);

	lcdc_samsung_panel_off(NULL);
	lcdc_samsung_panel_on(NULL);
}
#endif

module_init(lcdc_samsung_disp_init);
