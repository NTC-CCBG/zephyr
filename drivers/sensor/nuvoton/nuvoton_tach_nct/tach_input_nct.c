/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nuvoton_nct_tach_input

/**
 * @file tach_input_nct.c
 * @brief NCT virtual tachometer-input sensor driver.
 *
 * Each enabled "nuvoton,nct-tach-input" DT node creates one Zephyr sensor
 * device that measures the RPM on a single TA pin.  Multiple nodes may
 * reference the same MFT controller via the `mft` phandle; the driver
 * time-multiplexes the shared hardware using a spinlock.
 *
 * sample_fetch behaviour (non-blocking):
 *   - Underflow detected  -> switch to this pin, restart counter -> return  0
 *                            (caller sees 0 RPM = fan stopped)
 *   - Wrong pin active    -> switch to this pin                  -> return -EBUSY
 *   - Capture ready       -> read counter, compute capture       -> return  0
 *   - No event yet        ->                                        return -EBUSY
 *
 * channel_get always returns the last successfully captured RPM value.
 */

#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/dt-bindings/sensor/nct_tach.h>
#include <zephyr/logging/log.h>
#include "tach_nct.h"

LOG_MODULE_DECLARE(mft_nct, CONFIG_SENSOR_LOG_LEVEL);

/* Per-instance configuration (compile-time, from DT) */
struct tach_input_config {
	const struct device *mft;
	uint8_t  pin_select;
	uint32_t sample_clk;
	int      pulses_per_round;
	const struct pinctrl_dev_config *pcfg;
};

/* Per-instance runtime data */
struct tach_input_data {
	uint32_t capture; /* last valid captured count (CNT_MAX - TCRA) */
};

/* Pin-switch helper (must be called with MFT spinlock held) */
static void mft_switch_pin(struct tach_reg *inst,
			   const struct mft_nct_data *mft_data,
			   uint8_t pin, uint32_t sample_clk)
{
	/* Stop counter clock */
	SET_FIELD(inst->TCKC, NCT_TCKC_C1CSEL_FIELD, NCT_CLKSEL_DISABLED);

	/* Select new input pin */
	SET_FIELD(inst->TCFG, NCT_TCFG_MFT_IN_SEL, pin);

	/* Configure prescaler for the new sample rate */
	if (mft_data->input_clk == NCT_TACH_LFCLK) {
		/* LOW_PWR already set in mft_nct_init; no prescaler needed */
	} else {
		uint32_t prescaler = mft_data->input_clk / sample_clk;

		inst->TPRSC = (uint8_t)MIN(NCT_TACHO_PRSC_MAX, MAX(prescaler, 1u));
	}

	/* Reset counter and capture registers to max (counts down to 0) */
	inst->TCNT1 = NCT_TACHO_CNT_MAX;
	inst->TCRA  = NCT_TACHO_CNT_MAX;

	/* Clear any stale capture and underflow flags */
	inst->TECLR = BIT(NCT_TECLR_TACLR) | BIT(NCT_TECLR_TCCLR);

	/* Re-assert TAEN */
	inst->TMCTRL |= BIT(NCT_TMCTRL_TAEN);

	/* Restart counter clock */
	SET_FIELD(inst->TCKC, NCT_TCKC_C1CSEL_FIELD,
		  mft_data->input_clk == NCT_TACH_LFCLK
		  ? NCT_CLKSEL_LFCLK : NCT_CLKSEL_APBCLK);
}

/* Sensor API: sample_fetch */
static int tach_input_nct_sample_fetch(const struct device *dev,
				       enum sensor_channel chan)
{
	const struct tach_input_config *const config = dev->config;
	struct tach_input_data *const data = dev->data;
	struct mft_nct_data *const mft_data = config->mft->data;
	struct tach_reg *const inst = MFT_HAL_INSTANCE(config->mft);
	k_spinlock_key_t key;

	if (chan != SENSOR_CHAN_RPM && chan != SENSOR_CHAN_ALL) {
		return -ENOTSUP;
	}

	key = k_spin_lock(&mft_data->lock);

	/* Underflow: counter expired, no edge on current pin */
	if (IS_BIT_SET(inst->TECTRL, NCT_TECTRL_TCPND)) {
		mft_switch_pin(inst, mft_data, config->pin_select, config->sample_clk);
		mft_data->current_pin = config->pin_select;
		mft_data->current_sample_clk = config->sample_clk;
		k_spin_unlock(&mft_data->lock, key);
		data->capture = 0; /* fan stopped */
		return 0;
	}

	/* Wrong pin active: switch and ask caller to retry */
	if (mft_data->current_pin != config->pin_select) {
		mft_switch_pin(inst, mft_data, config->pin_select, config->sample_clk);
		mft_data->current_pin = config->pin_select;
		mft_data->current_sample_clk = config->sample_clk;
		k_spin_unlock(&mft_data->lock, key);
		return -EBUSY;
	}

	/* Capture ready: edge received on our pin */
	if (IS_BIT_SET(inst->TECTRL, NCT_TECTRL_TAPND)) {
		uint16_t raw = inst->TCRA;

		inst->TECLR = BIT(NCT_TECLR_TACLR) | BIT(NCT_TECLR_TCCLR);
		k_spin_unlock(&mft_data->lock, key);
		data->capture = NCT_TACHO_CNT_MAX - raw;
		return 0;
	}

	/* Not ready yet */
	k_spin_unlock(&mft_data->lock, key);
	return -EBUSY;
}

/* Sensor API: channel_get */
static int tach_input_nct_channel_get(const struct device *dev,
				      enum sensor_channel chan,
				      struct sensor_value *val)
{
	const struct tach_input_config *const config = dev->config;
	const struct tach_input_data *const data = dev->data;

    if (chan != SENSOR_CHAN_RPM) {
		return -ENOTSUP;
	}

	if (data->capture > 0) {
		/*
		 * RPM = (f * 60) / (n * TACH)
		 *   f    = sample_clk (Hz)
		 *   n    = pulses_per_round
		 *   TACH = captured count
		 */
		val->val1 = (config->sample_clk * 60U) /
			    (config->pulses_per_round * data->capture);
	} else {
		val->val1 = 0U;
	}

	val->val2 = 0U;
	return 0;
}

/* Device init: apply pinctrl, verify MFT parent is ready */
static int tach_input_nct_init(const struct device *dev)
{
	const struct tach_input_config *const config = dev->config;
	int ret;

	if (!device_is_ready(config->mft)) {
		LOG_ERR("%s: MFT controller not ready", dev->name);
		return -ENODEV;
	}

	ret = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		LOG_ERR("%s: pinctrl setup failed (%d)", dev->name, ret);
		return ret;
	}

	return 0;
}

static const struct sensor_driver_api tach_input_nct_driver_api = {
	.sample_fetch = tach_input_nct_sample_fetch,
	.channel_get  = tach_input_nct_channel_get,
};

/* Per-instance registration macro */
#define NCT_TACH_INPUT_INIT(inst)						\
	PINCTRL_DT_INST_DEFINE(inst);						\
										\
	static const struct tach_input_config tach_input_cfg_##inst = {		\
		.mft              = DEVICE_DT_GET(DT_INST_PHANDLE(inst, mft)),	\
		.pin_select       = DT_INST_PROP(inst, pin_select),		\
		.sample_clk       = DT_INST_PROP(inst, sample_clk),		\
		.pulses_per_round = DT_INST_PROP(inst, pulses_per_round),	\
		.pcfg             = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),	\
	};									\
										\
	static struct tach_input_data tach_input_data_##inst;			\
										\
	SENSOR_DEVICE_DT_INST_DEFINE(inst,					\
				     tach_input_nct_init,			\
				     NULL,					\
				     &tach_input_data_##inst,			\
				     &tach_input_cfg_##inst,			\
				     POST_KERNEL,				\
				     CONFIG_SENSOR_INIT_PRIORITY,		\
				     &tach_input_nct_driver_api);

DT_INST_FOREACH_STATUS_OKAY(NCT_TACH_INPUT_INIT)
