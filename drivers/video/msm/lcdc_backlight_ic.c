/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 and only
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS
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
/*yinbo.xie@ 20120822 #include <mach/gpio.h>*/
#include <linux/gpio.h>
#include "lcdc_backlight_ic.h"

static int lcd_brightness = -1;
static u8 use_pwm;

#ifdef PWM_CABC
static struct brt_value brt_table_r69329[] = {
		{ 255,	0xD34 }, /* Max */
		{ 242,	0xC6A },
		{ 229,	0xBA0 },
		{ 216,	0xAD6 },
		{ 203,	0xA0C },
		{ 190,	0x942 },
		{ 177,	0x878 },
		{ 164,	0x7AE },
		{ 151,	0x6E4 },
		{ 138,	0x61A }, /* default */
		{ 125,	0x582 },
		{ 113,	0x4EA },
		{ 101,	0x442 },
		{ 89,	0x3BA },
		{ 77,	0x322 },
		{ 65,	0x28A },
		{ 53,	0x1F2 },
		{ 41,	0x12C },
		{ 29,	0x6E }, /* Min */
		{ 20,	0x46 }, /* Dimming */
		{ 0,	0x0 }, /* Off */
};
#else
static struct brt_value brt_table_r69329[] = {
		{ 255,	7 }, /* Max */
		{ 235,	9 },
		{ 216,	10 },
		{ 197,	11 },
		{ 178,	12 },
		{ 159,	13 },
		{ 140,	14 },
		{ 121,	16 },
		{ 102,	18 }, /* default */
		{ 98,	20 },
		{ 94,	22 },
		{ 86,	23 },
		{ 78,	24 },
		{ 70,	25 },
		{ 62,	26 },
		{ 54,	27 },
		{ 46,	28 },
		{ 38,	29 },
		{ 30,	31 }, /* Min */
		{ 20,	31 }, /* Dimming */
		{ 0,	32 }, /* Off */
};
#endif

static struct brt_value brt_table_nt35510[] = {
		{ 255,	0xC3 }, /* Max */
		{ 242,	0xA9 },
		{ 229,	0xA1 },
		{ 216,	0x98 },
		{ 203,	0x8E },
		{ 190,	0x86 },
		{ 177,	0x7F },
		{ 164,	0x75 },
		{ 151,	0x6C },
		{ 138,	0x61 }, /* Default */
		{ 125,	0x58 },
		{ 113,	0x4F },
		{ 101,	0x47 },
		{ 89,	0x3E },
		{ 77,	0x38 },
		{ 65,	0x2B },
		{ 53,	0x1D },
		{ 41,	0x10 },
		{ 29,	0x04 }, /* Min */
		{ 20,	0x04 }, /* Dimming */
		{ 0,	0x0 }, /* Off */
};

static struct brt_value brt_table_s6d05a1[] = {
		{ 255,	11 }, /* Max */
		{ 240,	12 },
		{ 230,	13 },
		{ 215,	14 },
		{ 205,	15 },
		{ 195,	16 },
		{ 180,	17 },
		{ 168,	18 },
		{ 155,	20 },
		{ 145,	21 },
		{ 130,	22 }, /* default */
		{ 115,	23 },
		{ 102,	24 },
		{ 89,	25 },
		{ 76,	26 },
		{ 63,	27 },
		{ 50,	28 },
		{ 43,	29 },
		{ 35,	30 },
		{ 20,	31 }, /* Min, Dimming */
		{ 0,	32 }, /* Off */
};

static struct brt_value brt_table_db7420b[] = {
		{ 255,	7 }, /* Max */
		{ 235,	9 },
		{ 216,	10 },
		{ 197,	11 },
		{ 178,	12 },
		{ 159,	13 },
		{ 140,	14 },
		{ 121,	16 },
		{ 102,	18 }, /* default */
		{ 98,	20 },
		{ 94,	22 },
		{ 86,	23 },
		{ 78,	24 },
		{ 70,	25 },
		{ 62,	26 },
		{ 54,	27 },
		{ 46,	28 },
		{ 38,	29 },
		{ 30,	31 }, /* Min */
		{ 20,	31 }, /* Dimming */
		{ 0,	32 }, /* Off */

};

static struct brt_value *brt_table_aat = brt_table_s6d05a1;

#define MAX_BRT_STAGE_AAT (int)(sizeof(brt_table_s6d05a1)\
				/sizeof(struct brt_value))

static DEFINE_SPINLOCK(bl_ctrl_lock);

void update_backlight_table(int table_num)
{
	if (table_num == BACKLIGHT_NT35510) {
		brt_table_aat = brt_table_nt35510;
		use_pwm = 1;
	} else if (table_num == BACKLIGHT_DB7420B) {
		printk(KERN_INFO "LCD(vital2refresh_db7420b_backlight)");
		brt_table_aat = brt_table_db7420b;
	} else if (table_num == BACKLIGHT_S6D05A1) {
		brt_table_aat = brt_table_s6d05a1;
	} else if (table_num == BACKLIGHT_R69329) {
		brt_table_aat = brt_table_r69329;
		use_pwm = 1;
	} else {
		brt_table_aat = brt_table_s6d05a1;
	}
}

void aat1401_set_brightness(int level)
{
	int tune_level = 0;
	int i;

	spin_lock(&bl_ctrl_lock);
	if (level > 0) {
		if (level < MIN_BRIGHTNESS_VALUE) {
			tune_level = AAT_DIMMING_VALUE; /* DIMMING */
		} else {
			for (i = 0; i < MAX_BRT_STAGE_AAT - 1; i++) {
				if (level <= brt_table_aat[i].level
					&& level > brt_table_aat[i+1].level) {
					tune_level =
						brt_table_aat[i].tune_level;
					break;
				}
			}
		}
	} /*  BACKLIGHT is KTD model */

	if (!tune_level) {
		gpio_set_value(GPIO_BL_CTRL, 0);
		/*mdelay(3);*/
		udelay(2000);
		udelay(1000);
	} else {
		for (; tune_level > 0; tune_level--) {
			gpio_set_value(GPIO_BL_CTRL, 0);
			udelay(3);
			gpio_set_value(GPIO_BL_CTRL, 1);
			udelay(3);
		}
	}
	/*mdelay(1);*/
	udelay(1000);
	spin_unlock(&bl_ctrl_lock);
}

int ktd253_set_brightness(int level)
{
	int pulse;
	int tune_level = 0;
	int i;

	spin_lock(&bl_ctrl_lock);
	if (level > 0) {
		if ((level < MIN_BRIGHTNESS_VALUE) && !use_pwm) {
			tune_level = AAT_DIMMING_VALUE; /* DIMMING */
		} else {
			for (i = 0; i < MAX_BRT_STAGE_AAT - 1; i++) {
				if (level <= brt_table_aat[i].level
					&& level > brt_table_aat[i+1].level) {
					tune_level =
						brt_table_aat[i].tune_level;
					break;
				}
			}
		}
	} /*  BACKLIGHT is KTD model */

	if (use_pwm) {
		if (!tune_level)
			lcd_brightness = tune_level;
		else
			lcd_brightness = tune_level;
		udelay(1000);

	} else {
		if (!tune_level) {
			gpio_set_value(GPIO_BL_CTRL, 0);
			/*mdelay(3);*/
			udelay(2000);
			udelay(1000);
			lcd_brightness = tune_level;
		} else {
			if (unlikely(lcd_brightness < 0)) {
				int val = gpio_get_value(GPIO_BL_CTRL);
				if (val) {
					lcd_brightness = 0;
				gpio_set_value(GPIO_BL_CTRL, 0);
				/*mdelay(3);*/
				udelay(2000);
				udelay(1000);
				printk(KERN_INFO
				"LCD Baklight init in boot time on kernel\n");
				}
			}
			if (!lcd_brightness) {
				gpio_set_value(GPIO_BL_CTRL, 1);
				udelay(3);
				lcd_brightness = MAX_BRIGHTNESS_IN_BLU;
			}

			pulse = (tune_level - lcd_brightness +
				MAX_BRIGHTNESS_IN_BLU) % MAX_BRIGHTNESS_IN_BLU;

			for (; pulse > 0; pulse--) {
				gpio_set_value(GPIO_BL_CTRL, 0);
				udelay(3);
				/*ndelay(200);*/
				gpio_set_value(GPIO_BL_CTRL, 1);
				/*ndelay(200);*/
				udelay(3);
			}

			lcd_brightness = tune_level;
		}
		/*mdelay(1);*/
		udelay(1000);
	}
	spin_unlock(&bl_ctrl_lock);
	return tune_level;
}

int backlight_ic_set_brightness(int level)
{
	int ret = 0;
	ret = ktd253_set_brightness(level);
	return ret;
}
