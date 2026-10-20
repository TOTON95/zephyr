# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

board_runner_args(ap_bootloader "--board-id=1186" "--usb-id=26ac:1186")
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
board_runner_args(jlink "--device=STM32H743VI" "--speed=4000")

# keep first
if(NOT CONFIG_BOARD_CONTROL_N1_STM32H743XX_BARE)
  include(${ZEPHYR_BASE}/boards/common/ap_bootloader.board.cmake)
endif()
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/blackmagicprobe.board.cmake)
