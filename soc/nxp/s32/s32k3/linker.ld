/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Emit the IVT section only in an image that carries the IVT (CONFIG_SOC_S32K3_IVT) */
#if defined(CONFIG_SOC_S32K3_IVT)

MEMORY
{
	IVT_HEADER (r) : ORIGIN = (CONFIG_FLASH_BASE_ADDRESS + CONFIG_IVT_HEADER_OFFSET),
			 LENGTH = CONFIG_IVT_HEADER_SIZE
}

/* Symbols for IVT */
PROVIDE(__ivt_region_start = ORIGIN(IVT_HEADER));
PROVIDE(__ivt_region_end   = ORIGIN(IVT_HEADER) + LENGTH(IVT_HEADER));

#endif

/* Include standard Zephyr ARM script */
#include <zephyr/arch/arm/cortex_m/scripts/linker.ld>
