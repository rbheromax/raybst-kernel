/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description:
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
** =========================================================================
*/
#include <linux/delay.h>
#include <linux/gpio.h>
#include "tspdrv.h"

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS 1
#define PWM_DUTY_MAX    213 /* 13MHz / (579 + 1) = 22.4kHz */
#define PWM_DEVICE	1

static bool g_bampenabled;

static int32_t vibe_set_pwm_freq(int percent)
{
	/* Put the MND counter in reset mode for programming */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK, 0);

	/* P: 0 => Freq/1, 1 => Freq/2, 4 => Freq/4 */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_PRE_DIV_SEL_BMSK,
		g_nlra_gp_clk_PreDiv<<HWIO_GP_NS_REG_PRE_DIV_SEL_SHFT);

	/* S : 0 => TXCO(19.2MHz), 1 => Sleep XTAL(32kHz) */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_SRC_SEL_BMSK, 0 <<
		HWIO_GP_NS_REG_SRC_SEL_SHFT);

	/* Dual-edge mode */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_MODE_BMSK, 2 <<
		HWIO_GP_NS_REG_MNCNTR_MODE_SHFT);

	/* Set : M value */
	HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_M_VAL_BMSK,
		g_nlra_gp_clk_m << HWIO_GP_MD_REG_M_VAL_SHFT);

	g_nlra_gp_clk_d = percent * g_nlra_gp_clk_n / 100;

	/* Set : M value */
	HWIO_OUTM(GP_MD_REG, HWIO_GP_MD_REG_D_VAL_BMSK,
		(~((int16_t)g_nlra_gp_clk_d << 1))
			<< HWIO_GP_MD_REG_D_VAL_SHFT);

	/* Set : N value */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_GP_N_VAL_BMSK,
		~(g_nlra_gp_clk_n - g_nlra_gp_clk_m)
			 << HWIO_GP_NS_REG_GP_N_VAL_SHFT);

	/* Enable M/N counter */
	HWIO_OUTM(GP_NS_REG, HWIO_GP_NS_REG_MNCNTR_EN_BMSK,
		1 << HWIO_GP_NS_REG_MNCNTR_EN_SHFT);

	printk(KERN_DEBUG "[VIB] %d, %d, %d(%d%%)\n",
		g_nlra_gp_clk_m, g_nlra_gp_clk_n, g_nlra_gp_clk_d, percent);

	return VIBE_S_SUCCESS;
}

#ifndef CONFIG_MACH_RAY
static int32_t vibe_pwm_onoff(u8 onoff)
{
	if (onoff) {
		HWIO_OUTM(GP_NS_REG,
				  HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_BMSK,
				  1<<HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT);
		HWIO_OUTM(GP_NS_REG,
				  HWIO_GP_NS_REG_GP_ROOT_ENA_BMSK,
				  1<<HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT);
	} else {
		HWIO_OUTM(GP_NS_REG,
				  HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_BMSK,
				  0<<HWIO_GP_NS_REG_GP_CLK_BRANCH_ENA_SHFT);
		HWIO_OUTM(GP_NS_REG,
				  HWIO_GP_NS_REG_GP_ROOT_ENA_BMSK,
				  0<<HWIO_GP_NS_REG_GP_ROOT_ENA_SHFT);
	}
	return VIBE_S_SUCCESS;
}
#endif

/*
** Called to disable amp (disable output force)
*/
static int32_t ImmVibeSPI_ForceOut_AmpDisable(u_int8_t nActuatorIndex)
{
	if (g_bampenabled) {
		printk(KERN_DEBUG "VIBRATOR OFF :::::\n ");

		g_bampenabled = false;
		clk_disable(vib_clk);

		gpio_tlmm_config(GPIO_CFG(vibrator_drvdata.vib_pwm_gpio, \
			0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, \
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);

		gpio_set_value(vibrator_drvdata.vib_pwm_gpio, \
			    VIBRATION_OFF);

		gpio_direction_output(vibrator_drvdata.vib_en_gpio,\
			    VIBRATION_OFF);
	}
	return VIBE_S_SUCCESS;
}

/*
** Called to enable amp (enable output force)
*/
static int32_t ImmVibeSPI_ForceOut_AmpEnable(u_int8_t nActuatorIndex)
{
	if (!g_bampenabled) {
		printk(KERN_DEBUG "VIBRATOR ON :::::::\n");

		g_bampenabled = true;
		clk_enable(vib_clk);

		gpio_tlmm_config(GPIO_CFG(vibrator_drvdata.vib_pwm_gpio, \
			 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, \
			GPIO_CFG_2MA), GPIO_CFG_ENABLE);

		gpio_direction_output(vibrator_drvdata.vib_en_gpio,\
				VIBRATION_ON);
	}
	return VIBE_S_SUCCESS;
}

/*
** Called at initialization time to set PWM freq,
** disable amp, etc...
*/
static int32_t ImmVibeSPI_ForceOut_Initialize(void)
{
	int ret;
	DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize.\n"));
	/* printk("\n Inside Vibrator INITIALIZE FUNCTION ::: AMIT JAIN\n"); */
	/* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
	g_bampenabled = true;

	/*
	** Disable amp.
	** If multiple actuators are supported, please make sure to call
	** ImmVibeSPI_ForceOut_AmpDisable for each actuator
	** (provide the actuator index as input argument).
	*/

	/* set gpio config	*/
	ret = gpio_request(vibrator_drvdata.vib_pwm_gpio,\
			"Vibrator PWM enable");
	if (ret < 0) {
		printk(KERN_ERR "Vibrator PWM enable gpio_request is failed\n");
		goto err0;
	}
	gpio_tlmm_config(GPIO_CFG(vibrator_drvdata.vib_pwm_gpio,  3,
		GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
		GPIO_CFG_ENABLE);
	if (ret < 0) {
		printk(KERN_ERR "[VIB] FAIL gpio_tlmm_config(%d)\n",
			vibrator_drvdata.vib_pwm_gpio);
		goto err1;
	}
	ret = gpio_request(vibrator_drvdata.vib_en_gpio, \
			"vib enable");
	if (ret < 0) {
		printk(KERN_ERR "vib enable gpio_request is failed\n");
		goto err1;
	}
	gpio_tlmm_config(GPIO_CFG(vibrator_drvdata.vib_en_gpio,  0,
		GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			GPIO_CFG_ENABLE);
	if (ret < 0) {
		printk(KERN_ERR "[VIB] FAIL gpio_tlmm_config(%d)\n",
			vibrator_drvdata.vib_en_gpio);
		goto err2;
	}

	ImmVibeSPI_ForceOut_AmpDisable(0);
	return VIBE_S_SUCCESS;

err2:
	gpio_free(vibrator_drvdata.vib_en_gpio);
err1:
	gpio_free(vibrator_drvdata.vib_pwm_gpio);
err0:
	;
	return VIBE_E_FAIL;
}

/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
static int32_t ImmVibeSPI_ForceOut_Terminate(void)
{
	DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate.\n"));
	printk(KERN_DEBUG "\n INSIDE VIBRATOR TERMINATE\n");
	/*
	** Disable amp.
	** If multiple actuators are supported, please make sure to call
	** ImmVibeSPI_ForceOut_AmpDisable for each actuator
	** (provide the actuator index as input argument).
	*/
	ImmVibeSPI_ForceOut_AmpDisable(0);
	gpio_free(vibrator_drvdata.vib_en_gpio);
	gpio_free(vibrator_drvdata.vib_pwm_gpio);

	return VIBE_S_SUCCESS;
}

/*
** Called by the real-time loop to set PWM duty cycle
*/
static int32_t ImmVibeSPI_ForceOut_SetSamples(u_int8_t nActuatorIndex,
						u_int16_t nOutputSignalBitDepth,
						u_int16_t nBufferSizeInBytes,
						int8_t *pForceOutputBuffer)
{
	int8_t nforce;
	int percent;
	static int8_t pre_nforce;
	switch (nOutputSignalBitDepth) {
	case 8:
		/* pForceOutputBuffer is expected to contain 1 byte */
		if (nBufferSizeInBytes != 1) {
			DbgOut(KERN_ERR
			"[ImmVibeSPI] ImmVibeSPI_ForceOut_SetSamples nBufferSizeInBytes =  %d\n",
			nBufferSizeInBytes);
			return VIBE_E_FAIL;
		}
		nforce = pForceOutputBuffer[0];
		break;
	case 16:
		/* pForceOutputBuffer is expected to contain 2 byte */
		if (nBufferSizeInBytes != 2)
			return VIBE_E_FAIL;

		/* Map 16-bit value to 8-bit */
		nforce = ((int16_t *)pForceOutputBuffer)[0] >> 8;
		break;
	default:
		/* Unexpected bit depth */
		return VIBE_E_FAIL;
	}

	/* printk(KERN_DEBUG "SetSamples : bit(%d), force(%d)\n",
			nOutputSignalBitDepth, nforce); */

	if (nforce == 0) {
		/* Set 50% duty cycle or disable amp */
		ImmVibeSPI_ForceOut_AmpDisable(0);
		/*vibe_pwm_onoff(0);*/
		nforce = 0;
		pre_nforce = 0;
	} else {
		/* Map force from [-127, 127] to [0, PWM_DUTY_MAX] */
		percent = 50+(5000/127*nforce/100);

		/* printk(KERN_DEBUG "[tspdrv]nForce===%d\n", nforce); */
		if (pre_nforce != nforce) {
			/*vibe_pwm_onoff(1);*/
			vibe_set_pwm_freq(percent);
			ImmVibeSPI_ForceOut_AmpEnable(0);
			pre_nforce = nforce;
		}
	}
	return VIBE_S_SUCCESS;
}

/*
** Called to get the device name (device name must be returned as ANSI char)
*/
static int32_t ImmVibeSPI_Device_GetName(
		u_int8_t nActuatorIndex, char *szDevName, int nSize)
{
	return VIBE_S_SUCCESS;
}
