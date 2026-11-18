/* SPDX-License-Identifier: Apache-2.0 */

/*
 * Copyright (c) 2025 Nuvoton Technology Corporation.
 */

#ifndef ZEPHYR_DRIVERS_MISC_C2H_NCT_C2H_NCT_H_
#define ZEPHYR_DRIVERS_MISC_C2H_NCT_C2H_NCT_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <soc.h>

void rtc_write_offset(const struct device *dev, SIB_RTC_OFFSET_Enum offset, uint8_t value);
uint8_t rtc_read_offset(const struct device *dev, SIB_RTC_OFFSET_Enum offset);

#endif /* ZEPHYR_DRIVERS_MISC_C2H_NCT_C2H_NCT_H_ */
