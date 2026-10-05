# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

# Ported driver layer (FreeRTOS -> Zephyr adaptation of esp_driver_sdio).
zephyr_library_sources(sdio_slave_esp32c6.c)

# Espressif HAL for SDIO slave: register structs, LL ops, periph pin table.
# These ship in hal_espressif but are not compiled by its Zephyr glue.
set(ESP_HAL_SD_DIR ${ZEPHYR_HAL_ESPRESSIF_MODULE_DIR}/components/esp_hal_sd)
zephyr_library_sources(
	${ESP_HAL_SD_DIR}/sdio_slave_hal.c
	${ESP_HAL_SD_DIR}/${CONFIG_SOC_SERIES}/sdio_slave_periph.c
)
zephyr_library_include_directories(
	${ESP_HAL_SD_DIR}/include
	${ESP_HAL_SD_DIR}/${CONFIG_SOC_SERIES}/include
)
