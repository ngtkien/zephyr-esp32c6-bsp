# Copyright (c) 2026
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources_ifdef(CONFIG_ESP_C6_OTA_BOOT_PROBE boot_probe.c)
zephyr_library_sources_ifdef(CONFIG_ESP_C6_OTA_BOOT_PROBE app_desc.c)
