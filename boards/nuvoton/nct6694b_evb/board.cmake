
# SPDX-License-Identifier: Apache-2.0

# --- J-Link runner (default) ---
board_runner_args(jlink "--device=nct6694b" "--speed=4000")
board_runner_args(jlink "--file=./build/zephyr/${CONFIG_KERNEL_BIN_NAME}_signed.bin")
board_runner_args(jlink "--reset-after-load")

# --- OpenOCD runner (Nuvoton fork + Raspberry Pi Debug Probe) ---
# Use: west flash --runner openocd
#      west debug --runner openocd
board_runner_args(openocd
  "--openocd=$ENV{NUVOTON_OPENOCD}/bin/openocd"
  "--openocd-search=$ENV{NUVOTON_OPENOCD}/share/openocd/scripts"
  "--cmd-reset-halt=reset halt"
  "--load-file=./build/zephyr/${CONFIG_KERNEL_BIN_NAME}_signed.bin 0x80000 bin"
  --use-elf
)

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
