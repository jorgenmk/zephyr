board_runner_args(nrfjprog "--nrf-family=NRF91" "--softreset")
board_runner_args(jlink "--device=cortex-m4" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
