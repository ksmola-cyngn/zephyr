/*
 * Copyright 2022-2025 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_NXP_SIUL2_COMMON_H_
#define ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_NXP_SIUL2_COMMON_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

/** @cond INTERNAL_HIDDEN */

struct pinctrl_soc_reg {
	uint8_t inst;
	uint16_t idx;
	uint32_t val;
};

/** @brief Type for NXP SIUL2 pin configuration. */
typedef struct {
	struct pinctrl_soc_reg mscr;
	struct pinctrl_soc_reg imcr;
} pinctrl_soc_pin_t;

/**
 * @brief Utility macro to initialize each pin.
 *
 *
 * @param group Group node identifier.
 * @param prop Property name.
 * @param idx Property entry index.
 */
#define Z_PINCTRL_STATE_PIN_INIT(group, prop, idx)                                                 \
	{NXP_SIUL2_PINMUX_INIT(group, DT_PROP_BY_IDX(group, prop, idx))},

/**
 * @brief Utility macro to initialize the pins of one pin configuration node of a state.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 * @param idx Index of the pin configuration node in @p prop.
 * @param pin_prop Property of each group that holds its pins.
 */
#define Z_PINCTRL_STATE_GROUPS_INIT(node_id, prop, idx, pin_prop)                                  \
	DT_FOREACH_CHILD_VARGS(DT_PHANDLE_BY_IDX(node_id, prop, idx), DT_FOREACH_PROP_ELEM,        \
			       pin_prop, Z_PINCTRL_STATE_PIN_INIT)

/**
 * @brief Utility macro to initialize state pins contained in a given property.
 *
 * Every pin configuration node referenced by the property is used, so a state can
 * combine nodes, e.g. `pinctrl-0 = <&uart0_default &uart0_flow_control>;`.
 *
 * @param node_id Node identifier.
 * @param prop Property name describing state pins.
 */
#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{DT_FOREACH_PROP_ELEM_VARGS(node_id, prop, Z_PINCTRL_STATE_GROUPS_INIT, pinmux)}

/** @endcond */

#endif /* ZEPHYR_INCLUDE_DRIVERS_PINCTRL_PINCTRL_NXP_SIUL2_COMMON_H_ */
