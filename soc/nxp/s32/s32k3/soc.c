/*
 * Copyright 2023 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/cache.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>

#include <cmsis_core.h>
#include <OsIf.h>

/*
 * Emit IVT only for standalone XIP or MCUboot itself, on the boot core.
 * But not for the cases where Zephyr Image is loaded by MCUboot
 */
#ifdef CONFIG_SOC_S32K3_IVT
/* Image Vector Table structure definition for S32K3XX */
struct ivt {
	uint32_t header;
	uint32_t boot_configure;
	const uint32_t reserved_1;
	const uint32_t *cm7_0_start_address;
	const uint32_t reserved_2;
	const uint32_t *cm7_1_start_address;
	const uint32_t reserved_3;
	const uint32_t *cm7_2_start_address;
	const uint32_t reserved_4;
	const uint32_t *lc_configure;
	uint8_t reserved7[216];
};

#define IVT_MAGIC_MARKER 0x5AA55AA5

extern char _vector_start[];

/*
 * Attribute 'used' forces the compiler to emit ivt_header
 * even if nothing references it.
 * IVT for S32K3 SoCs, the minimal boot configuration is:
 * - Watchdog (SWT0) is disabled (default value).
 * - Non-Secure Boot is used (default value).
 * - Ungate clock for Cortex-M7_0 after boot.
 * - Application start address of Cortex-M7_0 is application's vector table.
 */
const struct ivt ivt_header __attribute__((section(".ivt_header"), used)) = {
	.header = IVT_MAGIC_MARKER,
	.boot_configure = 1,
	.cm7_0_start_address = (const void *)_vector_start,
	.cm7_1_start_address = NULL,
	.cm7_2_start_address = NULL,
	.lc_configure = NULL,
};
#endif /* CONFIG_SOC_S32K3_IVT */

#ifdef CONFIG_SOC_S32K3_START_CM7_2
/* MC_ME: Cortex-M7_2 is core 4 of partition 0 */
#define MC_ME_CTL_KEY            0x0U
#define MC_ME_CTL_KEY_KEY        0x5AF0U
#define MC_ME_CTL_KEY_INVERTED   0xA50FU
#define MC_ME_PRTN0_CORE4_PCONF  0x1C0U
#define MC_ME_PRTN0_CORE4_PUPD   0x1C4U
#define MC_ME_PRTN0_CORE4_STAT   0x1C8U
#define MC_ME_PRTN0_CORE4_ADDR   0x1CCU
#define MC_ME_PRTN_CORE_PCONF_CCE  BIT(0)
#define MC_ME_PRTN_CORE_PUPD_CCUPD BIT(0)
#define MC_ME_PRTN_CORE_STAT_CCS   BIT(0)

#define CM7_2_VECTOR_TABLE DT_REG_ADDR(DT_NODELABEL(cm7_2_code_partition))

/* VTOR alignment for 16 system exceptions and up to 240 interrupts */
BUILD_ASSERT((CM7_2_VECTOR_TABLE % 1024U) == 0U,
	     "Cortex-M7_2 vector table must be 1024-byte aligned");

#define SRAM_BASE  DT_REG_ADDR(DT_NODELABEL(sram0_2))
#define SRAM_END   (SRAM_BASE + DT_REG_SIZE(DT_NODELABEL(sram0_2)))
#define IMAGE_BASE DT_REG_ADDR(DT_CHOSEN(zephyr_sram))
#define IMAGE_END  (IMAGE_BASE + DT_REG_SIZE(DT_CHOSEN(zephyr_sram)))
#define NODE_BASE(node) DT_REG_ADDR(node)
#define NODE_END(node)  (DT_REG_ADDR(node) + DT_REG_SIZE(node))

/* SRAM shared with Cortex-M7_2 */
#define IS_SHARED_SRAM(node)                                                                       \
	((NODE_BASE(node) >= SRAM_BASE) && (NODE_END(node) <= SRAM_END) &&                         \
	 ((NODE_BASE(node) >= IMAGE_END) || (NODE_END(node) <= IMAGE_BASE)))

#define SHARED_SRAM_INIT(node)                                                                     \
	if (IS_SHARED_SRAM(node)) {                                                                \
		shared_sram_init(NODE_BASE(node), DT_REG_SIZE(node));                              \
	}

/* Initialize SRAM ECC with 64-bit writes */
static void shared_sram_init(uintptr_t addr, size_t size)
{
	for (uintptr_t p = addr; p < (addr + size); p += sizeof(uint64_t)) {
		*(volatile uint64_t *)p = 0ULL;
	}
	sys_cache_data_flush_range((void *)addr, size);
}

/* Start Cortex-M7_2 after the clocks are set up and shared SRAM is initialized */
static int soc_start_cm7_2(void)
{
	const mem_addr_t mc_me = DT_REG_ADDR(DT_NODELABEL(mc_me));

	if ((sys_read32(mc_me + MC_ME_PRTN0_CORE4_STAT) & MC_ME_PRTN_CORE_STAT_CCS) != 0U) {
		/* Already running, e.g. started by a debugger */
		return -EALREADY;
	}

	if (sys_read32(CM7_2_VECTOR_TABLE) == 0xFFFFFFFFU) {
		/* Erased: no image for Cortex-M7_2 */
		return -ENOENT;
	}

	DT_FOREACH_STATUS_OKAY(zephyr_memory_region, SHARED_SRAM_INIT)
	barrier_dsync_fence_full();

	sys_write32(CM7_2_VECTOR_TABLE, mc_me + MC_ME_PRTN0_CORE4_ADDR);
	sys_write32(MC_ME_PRTN_CORE_PCONF_CCE, mc_me + MC_ME_PRTN0_CORE4_PCONF);
	sys_write32(MC_ME_PRTN_CORE_PUPD_CCUPD, mc_me + MC_ME_PRTN0_CORE4_PUPD);
	sys_write32(MC_ME_CTL_KEY_KEY, mc_me + MC_ME_CTL_KEY);
	sys_write32(MC_ME_CTL_KEY_INVERTED, mc_me + MC_ME_CTL_KEY);

	if (!WAIT_FOR((sys_read32(mc_me + MC_ME_PRTN0_CORE4_STAT) & MC_ME_PRTN_CORE_STAT_CCS) != 0U,
		      1000, NULL)) {
		return -ETIMEDOUT;
	}

	return 0;
}

SYS_INIT(soc_start_cm7_2, PRE_KERNEL_2, 0);
#endif /* CONFIG_SOC_S32K3_START_CM7_2 */

void soc_early_init_hook(void)
{
	sys_cache_instr_enable();
	sys_cache_data_enable();

	OsIf_Init(NULL);
}
