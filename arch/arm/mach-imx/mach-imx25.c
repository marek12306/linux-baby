// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright 2012 Sascha Hauer, Pengutronix
 */

#include <linux/of.h>
#include <linux/string.h>
#include <asm/mach/arch.h>
#include <asm/system_info.h>
#include "common.h"
#include "hardware.h"

static void __init imx25_init_early(void)
{
	struct device_node *chosen;
	const void *uuid;
	u32 revision;
	int len;

	mxc_set_cpu_type(MXC_CPU_MX25);

	chosen = of_find_node_by_path("/chosen");
	if (chosen) {
		if (!of_property_read_u32(chosen, "logitech,system-revision",
					  &revision))
			system_rev = revision;

		uuid = of_get_property(chosen, "logitech,system-uuid", &len);
		if (uuid && len == sizeof(system_uuid)) {
			memcpy(system_uuid, uuid, sizeof(system_uuid));
			system_uuid_valid = true;
		}
		of_node_put(chosen);
	}
}

static void __init imx25_dt_init(void)
{
	imx_aips_allow_unprivileged_access("fsl,imx25-aips");
}

static const char * const imx25_dt_board_compat[] __initconst = {
	"fsl,imx25",
	NULL
};

DT_MACHINE_START(IMX25_DT, "Freescale i.MX25 (Device Tree Support)")
	.init_early	= imx25_init_early,
	.init_machine	= imx25_dt_init,
	.init_late      = imx25_pm_init,
	.dt_compat	= imx25_dt_board_compat,
MACHINE_END
