/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * esp_app_desc_t-compatible header placed at the very start of the
 * image's first segment (see the .esp_app_desc KEEP() at the top of
 * .iram0.text in soc/espressif/esp32c6/default.ld).
 *
 * The stock IDF 2nd-stage bootloader's esp_image_verify() — which the
 * ESP-Hosted hybrid firmware runs on every OTAEnd — requires the first
 * 4 bytes of segment 0 to be ESP_APP_DESC_MAGIC_WORD and reads
 * mmu_page_size at offsetof(esp_app_desc_t, mmu_page_size). Without
 * this block every Zephyr image is rejected as "corrupted" at OTAEnd.
 */

#include <stdint.h>
#include <zephyr/sys/util.h>

#define ESP_APP_DESC_MAGIC_WORD	0xABCD5432U

struct esp_app_desc_compat {
	uint32_t magic_word;
	uint32_t secure_version;
	uint32_t reserv1[2];
	char     version[32];
	char     project_name[32];
	char     time[16];
	char     date[16];
	char     idf_ver[32];
	uint8_t  app_elf_sha256[32];
	uint16_t min_efuse_blk_rev_full;
	uint16_t max_efuse_blk_rev_full;
	uint8_t  mmu_page_size;	/* log2; 0 = legacy 64 KiB pages */
	uint8_t  reserv3[3];
	uint32_t reserv2[18];
};

BUILD_ASSERT(sizeof(struct esp_app_desc_compat) == 256);
BUILD_ASSERT(offsetof(struct esp_app_desc_compat, mmu_page_size) == 180);

__attribute__((section(".esp_app_desc"), used))
static const struct esp_app_desc_compat c6_app_desc = {
	.magic_word = ESP_APP_DESC_MAGIC_WORD,
	.secure_version = 0,
	.version = "1.4.90+zephyr",
	.project_name = "c6_zephyr",
	.time = __TIME__,
	.date = __DATE__,
	.idf_ver = "zephyr",
	.mmu_page_size = 0,
};
