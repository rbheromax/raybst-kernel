/* linux/arch/arm/mach-xxxx/board-ray-modems.c
 * Copyright (C) 2010 Samsung Electronics. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <mach/hardware.h>
#include <mach/msm_smsm.h>
#include <mach/msm_iomap.h>
#include <linux/platform_data/modem.h>

#define DPRAM_START_ADDRESS	0x0
#define DPRAM_SIZE              0x8000
#define DPRAM_END_ADDRESS	(DPRAM_START_ADDRESS + DPRAM_SIZE - 1)

static struct modem_io_t cdma_io_devices[] = {
	[0] = {
		.name = "cdma_ipc0",
		.id = 0x1,
		.format = IPC_FMT,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
	[1] = {
		.name = "multipdp",
		.id = 0x1,
		.format = IPC_MULTI_RAW,
		.io_type = IODEV_DUMMY,
		.link = LINKDEV_DPRAM,
	},
	[2] = {
		.name = "rmnet0",
		.id = 0x2A,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[3] = {
		.name = "rmnet1",
		.id = 0x2B,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[4] = {
		.name = "rmnet2",
		.id = 0x2C,
		.format = IPC_RAW,
		.io_type = IODEV_NET,
		.link = LINKDEV_DPRAM,
	},
	[5] = {
		.name = "cdma_cplog",
		.id = 0x1D,
		.format = IPC_RAW,
		.io_type = IODEV_MISC,
		.link = LINKDEV_DPRAM,
	},
};

/* cdma target platform data */
static struct modem_data cdma_modem_data = {
	.name = "msm8x55",

	.modem_type = QC_MSM8x55,
	.link_type = LINKDEV_DPRAM,
	.modem_net = CDMA_NETWORK,

	.num_iodevs = ARRAY_SIZE(cdma_io_devices),
	.iodevs = cdma_io_devices,
	.use_handover = false,
};

static struct resource cdma_modem_res[] = {
	[0] = {
		.name = "smd_info",
		.start = SMEM_ID_VENDOR0, /* ID */
		.end = DPRAM_SIZE, /* dpram size */
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "smd_irq_info",
		.start = INT_A9_M2A_4, /* irq number */
		.end = IRQ_TYPE_EDGE_RISING, /* INT type */
		.flags = IORESOURCE_IRQ,
	},
	[3] = {
		.name = "smd_irq_arg",
		.start = 4,
		/* .end = (MSM_GCC_BASE + 0x8), */
		.end = (MSM_APCS_GCC_BASE + 0x8),
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device cdma_modem = {
	.name = "modem_if",
	.id = 1,
	.num_resources = ARRAY_SIZE(cdma_modem_res),
	.resource = cdma_modem_res,
	.dev = {
		.platform_data = &cdma_modem_data,
	},
};

static int __init init_modem(void)
{
	int ret;
	pr_info("[MIF] <%s> init_modem\n", __func__);

	ret = platform_device_register(&cdma_modem);
	if (ret < 0)
		pr_err("[MIF] <%s> init_modem failed!!\n", __func__);

	return ret;
}
late_initcall(init_modem);
