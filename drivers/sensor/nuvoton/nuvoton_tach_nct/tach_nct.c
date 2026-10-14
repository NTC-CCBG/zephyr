/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_nct_tach

/**
 * @file tach_nct.c
 * @brief NCT Multi-Function Timer (MFT) controller driver.
 *
 * This driver initialises the MFT hardware (clock gate, Mode 5, capture-enable,
 * debounce, LOW_PWR for LFCLK) and exposes a non-sensor device that child
 * tach-input nodes share via phandle.  It does NOT implement the sensor API;
 * that is the responsibility of tach_input_nct.c.
 *
 * One instance is created per enabled "nuvoton,nct-tach" DT node.
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include "tach_nct.h"

LOG_MODULE_REGISTER(mft_nct, CONFIG_SENSOR_LOG_LEVEL);

static int mft_nct_init(const struct device *dev)
{
	const struct mft_nct_config *const config = dev->config;
	struct mft_nct_data *const data = dev->data;
	struct tach_reg *const inst = MFT_HAL_INSTANCE(dev);
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(pcc));
	int ret;

	if (!device_is_ready(clk_dev)) {
		LOG_ERR("clock control device not ready");
		return -ENODEV;
	}

	/* Enable the MFT clock gate */
	ret = clock_control_on(clk_dev, (clock_control_subsys_t)config->clk_cfg);
	if (ret < 0) {
		LOG_ERR("Turn on MFT%d clock fail %d", config->tach_channel, ret);
		return ret;
	}

	/* Read back the actual source clock frequency */
	ret = clock_control_get_rate(clk_dev,
				     (clock_control_subsys_t)config->clk_cfg,
				     &data->input_clk);
	if (ret < 0) {
		LOG_ERR("Get MFT%d clock rate error %d", config->tach_channel, ret);
		return ret;
	}

	/* Set Mode 5 (input capture) */
	SET_FIELD(inst->TMCTRL, NCT_TMCTRL_MDSEL_FIELD, NCT_TACH_MDSEL);

	/* Enable capture and input debounce - persist across pin switches */
	inst->TMCTRL |= BIT(NCT_TMCTRL_TAEN);
	inst->TCFG   |= BIT(NCT_TCFG_TADBEN);

	/* Pre-set LOW_PWR mode if the source is LFCLK */
	if (data->input_clk == NCT_TACH_LFCLK) {
		inst->TCKC |= BIT(NCT_TCKC_LOW_PWR);
	}

	/* No pin is active yet */
	data->current_pin = 0xFF;
	data->current_sample_clk = 0;

	return 0;
}

#define NCT_MFT_INIT(inst)							\
	static const struct mft_nct_config mft_cfg_##inst = {			\
		.base         = DT_INST_REG_ADDR(inst),				\
		.clk_cfg      = DT_INST_PHA(inst, clocks, clk_cfg),		\
		.tach_channel = DT_INST_PROP(inst, tach_channel),		\
	};									\
										\
	static struct mft_nct_data mft_data_##inst;				\
										\
	DEVICE_DT_INST_DEFINE(inst,						\
			      mft_nct_init,					\
			      NULL,						\
			      &mft_data_##inst,					\
			      &mft_cfg_##inst,					\
			      POST_KERNEL,					\
			      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,		\
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(NCT_MFT_INIT)
