/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_s32_qspi_hyperram

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#include <Qspi_Ip.h>
#include <Qspi_Ip_Controller.h>
#include <cmsis_core.h>

#include "memc_nxp_s32_qspi.h"

LOG_MODULE_REGISTER(nxp_s32_qspi_hyperram, CONFIG_MEMC_LOG_LEVEL);

BUILD_ASSERT(CONFIG_MEMC_NXP_S32_QSPI_HYPERRAM_INIT_PRIORITY > CONFIG_MEMC_INIT_PRIORITY,
	     "HyperRAM must initialize after its QSPI controller");

/* LUT sequences: AHB read as the HAL's QSPI_IP_AHB_LUT, IP commands use sequence 0 */
#define HYPERRAM_SEQ_READ     1U
#define HYPERRAM_SEQ_WRITE    2U
#define HYPERRAM_SEQ_READ_REG 3U

/* First command/address byte: CA47 read, CA46 register space, CA45 linear burst */
#define HYPERBUS_READ_MEM  0xA0U
#define HYPERBUS_WRITE_MEM 0x20U
#define HYPERBUS_READ_REG  0xE0U

/* Not described by the HAL register header */
#define QSPI_AWRCR 0x50U

#define HYPERBUS_OP(inst, op) QSPI_LUT_OP(inst, QSPI_IP_LUT_PADS_8, op)

/* AHB window of each QSPI instance */
static const uint32_t qspi_amba_base[] = FEATURE_QSPI_AMBA_BASE;

struct memc_nxp_s32_qspi_hyperram_config {
	const struct device *controller;
	QuadSPI_Type *base;
	uint8_t read_dummy;
	uint8_t write_dummy;
};

static void hyperram_lut_seq(uint8_t instance, uint8_t seq, uint8_t command, uint8_t dummy,
			     Qspi_Ip_InstrOpType data)
{
	const uint32_t lut[FEATURE_QSPI_LUT_SEQUENCE_SIZE] = {
		QSPI_IP_PACK_LUT_REG(HYPERBUS_OP(QSPI_IP_LUT_INSTR_CMD_DDR, command),
				     HYPERBUS_OP(QSPI_IP_LUT_INSTR_ADDR_DDR, 24U)),
		QSPI_IP_PACK_LUT_REG(HYPERBUS_OP(QSPI_IP_LUT_INSTR_CADDR_DDR, 16U),
				     HYPERBUS_OP(QSPI_IP_LUT_INSTR_DUMMY, dummy)),
		QSPI_IP_PACK_LUT_REG(data, QSPI_IP_LUT_SEQ_END),
	};

	Qspi_Ip_WriteLuts(instance, seq * FEATURE_QSPI_LUT_SEQUENCE_SIZE, lut,
			  FEATURE_QSPI_LUT_SEQUENCE_SIZE);
}

static int memc_nxp_s32_qspi_hyperram_init(const struct device *dev)
{
	const struct memc_nxp_s32_qspi_hyperram_config *cfg = dev->config;
	uint8_t instance;
	uint8_t id[2];

	if (!device_is_ready(cfg->controller)) {
		LOG_ERR("QSPI controller not ready");
		return -ENODEV;
	}
	instance = memc_nxp_s32_qspi_get_instance(cfg->controller);

	/* Data size comes from the IP or AHB transfer */
	hyperram_lut_seq(instance, HYPERRAM_SEQ_READ, HYPERBUS_READ_MEM, cfg->read_dummy,
			 HYPERBUS_OP(QSPI_IP_LUT_INSTR_READ_DDR, 0x80U));
	hyperram_lut_seq(instance, HYPERRAM_SEQ_WRITE, HYPERBUS_WRITE_MEM, cfg->write_dummy,
			 HYPERBUS_OP(QSPI_IP_LUT_INSTR_WRITE_DDR, 0x80U));
	hyperram_lut_seq(instance, HYPERRAM_SEQ_READ_REG, HYPERBUS_READ_REG, cfg->read_dummy,
			 HYPERBUS_OP(QSPI_IP_LUT_INSTR_READ_DDR, 0x02U));

	/* Identification register 0 */
	if (Qspi_Ip_IpRead(instance, HYPERRAM_SEQ_READ_REG, qspi_amba_base[instance], id, NULL,
			   sizeof(id)) != STATUS_QSPI_IP_SUCCESS) {
		LOG_ERR("Identification register read failed");
		return -EIO;
	}
	if (((id[0] == 0x00U) && (id[1] == 0x00U)) || ((id[0] == 0xFFU) && (id[1] == 0xFFU))) {
		LOG_ERR("No HyperRAM (ID0 %02x%02x)", id[0], id[1]);
		return -ENODEV;
	}
	LOG_INF("HyperRAM ID0 %02x%02x", id[0], id[1]);

	/* AHB: TX trigger level 0, RWDS as write mask, no prefetch (stale reads) */
	cfg->base->BFGENCR = QuadSPI_BFGENCR_SEQID(HYPERRAM_SEQ_READ) |
			     QuadSPI_BFGENCR_SEQID_WR_EN(1U) |
			     QuadSPI_BFGENCR_SEQID_WR(HYPERRAM_SEQ_WRITE);
	sys_write32(0U, (mem_addr_t)cfg->base + QSPI_AWRCR);
	cfg->base->MCR |= QuadSPI_MCR_DQS_OUT_EN_MASK;
	cfg->base->SPTRCLR = cfg->base->SPTRCLR | QuadSPI_SPTRCLR_PREFETCH_DIS_MASK |
			     QuadSPI_SPTRCLR_BFPTRC_MASK | QuadSPI_SPTRCLR_IPPTRC_MASK;

	if (IS_ENABLED(CONFIG_MEMC_NXP_S32_QSPI_HYPERRAM_NO_STREAMING_WRITES)) {
		/* See CONFIG_MEMC_NXP_S32_QSPI_HYPERRAM_NO_STREAMING_WRITES */
		SCnSCB->ACTLR |= SCnSCB_ACTLR_DISRAMODE_Msk;
		barrier_dsync_fence_full();
		barrier_isync_fence_full();
	}

	return 0;
}

/* Dummy clocks: 2x latency for reads, one less for writes */
#define HYPERRAM_READ_DUMMY(n)  (2U * DT_INST_PROP(n, read_latency_cycles))
#define HYPERRAM_WRITE_DUMMY(n) (HYPERRAM_READ_DUMMY(n) - 1U)

#define MEMC_NXP_S32_QSPI_HYPERRAM_INIT(n)                                                         \
	BUILD_ASSERT(DT_INST_REG_ADDR(n) == 0, "Only port A1 is supported");                      \
	BUILD_ASSERT(DT_PROP(DT_INST_BUS(n), sample_delay_half_cycle),                            \
		     "HyperRAM needs sample-delay-half-cycle on the QSPI controller");            \
	static const struct memc_nxp_s32_qspi_hyperram_config                                      \
		memc_nxp_s32_qspi_hyperram_config_##n = {                                          \
			.controller = DEVICE_DT_GET(DT_INST_BUS(n)),                               \
			.base = (QuadSPI_Type *)DT_REG_ADDR(DT_INST_BUS(n)),                       \
			.read_dummy = HYPERRAM_READ_DUMMY(n),                                      \
			.write_dummy = HYPERRAM_WRITE_DUMMY(n),                                    \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, memc_nxp_s32_qspi_hyperram_init, NULL, NULL,                      \
			      &memc_nxp_s32_qspi_hyperram_config_##n, POST_KERNEL,                 \
			      CONFIG_MEMC_NXP_S32_QSPI_HYPERRAM_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_NXP_S32_QSPI_HYPERRAM_INIT)
