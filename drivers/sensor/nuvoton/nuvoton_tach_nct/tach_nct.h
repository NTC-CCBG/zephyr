/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file tach_nct.h
 * @brief Private shared definitions for the NCT MFT/tachometer driver pair.
 *
 * This header is included by both tach_nct.c (MFT controller) and
 * tach_input_nct.c (virtual tach-input sensor) so they share struct layouts
 * and register constants without exposing them to application code.
 */

#ifndef TACH_NCT_H_
#define TACH_NCT_H_

#include <zephyr/kernel.h>
#include <soc.h>

/* Counter / prescaler limits */
#define NCT_TACHO_PRSC_MAX   0xffu
#define NCT_TACHO_CNT_MAX    0xffffu

/* TCKC.C1CSEL clock-source values */
#define NCT_CLKSEL_DISABLED  0u
#define NCT_CLKSEL_APBCLK    1u
#define NCT_CLKSEL_LFCLK     4u

/* LFCLK frequency used to identify LFCLK-sourced MFTs */
#define NCT_TACH_LFCLK       32768u

/* TMCTRL.MDSEL value for Mode 5 (input capture) */
#define NCT_TACH_MDSEL       4u

/* MFT controller configuration (compatible "nuvoton,nct-tach") */
struct mft_nct_config {
	uintptr_t base;
	uint32_t  clk_cfg;
	int       tach_channel;
};

/* MFT controller runtime data - shared by all child tach-input devices */
struct mft_nct_data {
	uint32_t       input_clk;          /* actual source clock rate (Hz)    */
	struct k_spinlock lock;            /* guards register access + state   */
	uint8_t        current_pin;        /* active TA pin index; 0xFF = idle */
	uint32_t       current_sample_clk; /* sample_clk of the active pin     */
};

/* Register-base helper */
#define MFT_HAL_INSTANCE(mft_dev) \
	((struct tach_reg *)((const struct mft_nct_config *)(mft_dev)->config)->base)

#endif /* TACH_NCT_H_ */
