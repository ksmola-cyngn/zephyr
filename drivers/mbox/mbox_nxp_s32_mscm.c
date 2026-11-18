/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_s32_mscm_mbox

#include <zephyr/drivers/mbox.h>
#include <zephyr/irq.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/sys/util.h>

#define LOG_LEVEL CONFIG_MBOX_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(nxp_s32_mscm_mbox);

#define MBOX_MSCM_CPXNUM          0x004U
#define MBOX_MSCM_CPXNUM_CPN_MASK 0x3U

/* IRCPnISRx at 0x200 + 0x20 * n + 0x8 * x, IRCPnIGRx 4 bytes after it */
#define MBOX_MSCM_IRCP_ISR(cp, ch) (0x200U + (0x20U * (cp)) + (0x8U * (ch)))
#define MBOX_MSCM_IRCP_IGR(cp, ch) (MBOX_MSCM_IRCP_ISR(cp, ch) + 0x4U)
#define MBOX_MSCM_IRCP_IGR_INT_EN  BIT(0)

#define MBOX_MSCM_CHANNELS 4U
#define MBOX_MSCM_MAX_CPN  3U

struct nxp_s32_mscm_mbox_config {
	mem_addr_t base;
	uint8_t remote_cpu;
	void (*irq_config)(void);
	void (*irq_set)(uint32_t channel, bool enable);
};

struct nxp_s32_mscm_mbox_data {
	uint8_t cpu;
	mbox_callback_t cb[MBOX_MSCM_CHANNELS];
	void *user_data[MBOX_MSCM_CHANNELS];
};

static int nxp_s32_mscm_mbox_send(const struct device *dev, uint32_t channel,
				  const struct mbox_msg *msg)
{
	const struct nxp_s32_mscm_mbox_config *cfg = dev->config;

	if (channel >= MBOX_MSCM_CHANNELS) {
		return -EINVAL;
	}

	/* Signalling only: core-to-core interrupts carry no data */
	if ((msg != NULL) && (msg->size > 0U)) {
		return -EMSGSIZE;
	}

	sys_write32(MBOX_MSCM_IRCP_IGR_INT_EN,
		    cfg->base + MBOX_MSCM_IRCP_IGR(cfg->remote_cpu, channel));

	return 0;
}

static int nxp_s32_mscm_mbox_register_callback(const struct device *dev, uint32_t channel,
					       mbox_callback_t cb, void *user_data)
{
	struct nxp_s32_mscm_mbox_data *data = dev->data;
	unsigned int key;

	if (channel >= MBOX_MSCM_CHANNELS) {
		return -EINVAL;
	}

	key = irq_lock();
	data->cb[channel] = cb;
	data->user_data[channel] = user_data;
	irq_unlock(key);

	return 0;
}

static int nxp_s32_mscm_mbox_mtu_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return 0;
}

static uint32_t nxp_s32_mscm_mbox_max_channels_get(const struct device *dev)
{
	ARG_UNUSED(dev);

	return MBOX_MSCM_CHANNELS;
}

static int nxp_s32_mscm_mbox_set_enabled(const struct device *dev, uint32_t channel, bool enable)
{
	const struct nxp_s32_mscm_mbox_config *cfg = dev->config;
	struct nxp_s32_mscm_mbox_data *data = dev->data;

	if (channel >= MBOX_MSCM_CHANNELS) {
		return -EINVAL;
	}

	if (enable && (data->cb[channel] == NULL)) {
		LOG_WRN("Enabling channel %u without a callback", channel);
	}

	cfg->irq_set(channel, enable);

	return 0;
}

static void nxp_s32_mscm_mbox_isr(const struct device *dev, uint32_t channel)
{
	const struct nxp_s32_mscm_mbox_config *cfg = dev->config;
	struct nxp_s32_mscm_mbox_data *data = dev->data;
	mem_addr_t isr = cfg->base + MBOX_MSCM_IRCP_ISR(data->cpu, channel);
	uint32_t pending = sys_read32(isr);

	/* One bit per initiating core, write 1 to clear */
	sys_write32(pending, isr);

	if ((pending != 0U) && (data->cb[channel] != NULL)) {
		data->cb[channel](dev, channel, data->user_data[channel], NULL);
	}
}

static int nxp_s32_mscm_mbox_init(const struct device *dev)
{
	const struct nxp_s32_mscm_mbox_config *cfg = dev->config;
	struct nxp_s32_mscm_mbox_data *data = dev->data;

	data->cpu =
		(uint8_t)(sys_read32(cfg->base + MBOX_MSCM_CPXNUM) & MBOX_MSCM_CPXNUM_CPN_MASK);

	if (data->cpu == cfg->remote_cpu) {
		LOG_ERR("remote-cpu %u is the local core", cfg->remote_cpu);
		return -EINVAL;
	}

	/* Drop interrupts left pending from before a reset of this core */
	for (uint32_t ch = 0U; ch < MBOX_MSCM_CHANNELS; ch++) {
		mem_addr_t isr = cfg->base + MBOX_MSCM_IRCP_ISR(data->cpu, ch);

		sys_write32(sys_read32(isr), isr);
	}

	cfg->irq_config();

	return 0;
}

static DEVICE_API(mbox, nxp_s32_mscm_mbox_driver_api) = {
	.send = nxp_s32_mscm_mbox_send,
	.register_callback = nxp_s32_mscm_mbox_register_callback,
	.mtu_get = nxp_s32_mscm_mbox_mtu_get,
	.max_channels_get = nxp_s32_mscm_mbox_max_channels_get,
	.set_enabled = nxp_s32_mscm_mbox_set_enabled,
};

#define MSCM_MBOX_ISR_DEFINE(idx, n)                                                               \
	static void nxp_s32_mscm_mbox_isr_##n##_##idx(const void *arg)                             \
	{                                                                                          \
		nxp_s32_mscm_mbox_isr((const struct device *)arg, idx);                            \
	}

#define MSCM_MBOX_IRQ_CONNECT(idx, n)                                                              \
	IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, idx, irq), DT_INST_IRQ_BY_IDX(n, idx, priority),         \
		    nxp_s32_mscm_mbox_isr_##n##_##idx, DEVICE_DT_INST_GET(n), 0);

#define MSCM_MBOX_IRQ_ENABLE_CASE(idx, n)                                                          \
	case idx:                                                                                  \
		if (enable) {                                                                      \
			irq_enable(DT_INST_IRQ_BY_IDX(n, idx, irq));                               \
		} else {                                                                           \
			irq_disable(DT_INST_IRQ_BY_IDX(n, idx, irq));                              \
		}                                                                                  \
		break;

#define NXP_S32_MSCM_MBOX_INIT(n)                                                                  \
	BUILD_ASSERT(DT_NUM_IRQS(DT_DRV_INST(n)) == MBOX_MSCM_CHANNELS,                            \
		     "One interrupt per core-to-core interrupt channel");                          \
	BUILD_ASSERT(DT_INST_PROP(n, remote_cpu) <= MBOX_MSCM_MAX_CPN, "Invalid remote-cpu");    \
                                                                                                   \
	LISTIFY(4, MSCM_MBOX_ISR_DEFINE, (), n)                                                    \
                                                                                                   \
	static void nxp_s32_mscm_mbox_irq_config_##n(void)                                         \
	{                                                                                          \
		LISTIFY(4, MSCM_MBOX_IRQ_CONNECT, (), n)                                           \
	}                                                                                          \
                                                                                                   \
	static void nxp_s32_mscm_mbox_irq_set_##n(uint32_t channel, bool enable)                \
	{                                                                                          \
		switch (channel) {                                                                 \
			LISTIFY(4, MSCM_MBOX_IRQ_ENABLE_CASE, (), n)                               \
		default:                                                                           \
			break;                                                                     \
		}                                                                                  \
	}                                                                                          \
                                                                                                   \
	static const struct nxp_s32_mscm_mbox_config nxp_s32_mscm_mbox_config_##n = {             \
		.base = DT_INST_REG_ADDR(n),                                                       \
		.remote_cpu = DT_INST_PROP(n, remote_cpu),                                         \
		.irq_config = nxp_s32_mscm_mbox_irq_config_##n,                                    \
		.irq_set = nxp_s32_mscm_mbox_irq_set_##n,                                    \
	};                                                                                         \
                                                                                                   \
	static struct nxp_s32_mscm_mbox_data nxp_s32_mscm_mbox_data_##n;                           \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, nxp_s32_mscm_mbox_init, NULL, &nxp_s32_mscm_mbox_data_##n,        \
			      &nxp_s32_mscm_mbox_config_##n, PRE_KERNEL_1,                         \
			      CONFIG_MBOX_INIT_PRIORITY, &nxp_s32_mscm_mbox_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NXP_S32_MSCM_MBOX_INIT)
