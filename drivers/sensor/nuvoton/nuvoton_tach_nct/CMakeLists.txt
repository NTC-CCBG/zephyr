# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_TACH_NCT       tach_nct.c)
zephyr_library_sources_ifdef(CONFIG_TACH_NCT_INPUT tach_input_nct.c)
