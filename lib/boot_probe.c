/*
 * Copyright (c) 2026
 * SPDX-License-Identifier: Apache-2.0
 *
 * Runtime resolution of the flash offset our image was booted from.
 *
 * Runs inside the Zephyr ESP ROM loader (map_rom_segments) — no kernel
 * APIs, no heap, only ROM functions. Mirrors the semantics of IDF's
 * esp_ota_get_boot_partition(): decode otadata, fall back to scanning
 * candidate partition offsets for a valid ESP app image.
 */

#include <stdint.h>
#include <stdbool.h>

#include <esp_rom_crc.h>
#include <bootloader_flash_priv.h>
#include <esp_flash_partitions.h>
#include <esp_app_format.h>

#include <zephyr/devicetree.h>

#define OTADATA_ADDR	DT_REG_ADDR(DT_NODELABEL(otadata_partition))
#define OTA_SEL_STRIDE	0x1000U /* entries live one flash sector apart */

#define OTA0_ADDR	DT_REG_ADDR(DT_NODELABEL(ota0_partition))
#define OTA1_ADDR	DT_REG_ADDR(DT_NODELABEL(ota1_partition))
#define BOOT_ADDR	DT_REG_ADDR(DT_NODELABEL(boot_partition))

#define OTA_APP_COUNT	2U

/* esp_ota_img_states_t subset (esp_ota_ops.h) */
#define ESP_OTA_IMG_INVALID	0x3U
#define ESP_OTA_IMG_ABORTED	0x4U

extern void __start(void);

static bool ota_sel_valid(const esp_ota_select_entry_t *s)
{
	if (s->ota_seq == UINT32_MAX || s->ota_state == ESP_OTA_IMG_INVALID ||
	    s->ota_state == ESP_OTA_IMG_ABORTED) {
		return false;
	}

	return s->crc == esp_rom_crc32_le(UINT32_MAX, (const uint8_t *)&s->ota_seq,
					sizeof(s->ota_seq));
}

static bool image_matches_self(uint32_t addr)
{
	esp_image_header_t hdr;

	if (esp_rom_flash_read(addr, &hdr, sizeof(hdr), true) != 0 ||
	    hdr.magic != ESP_IMAGE_HEADER_MAGIC ||
	    hdr.chip_id != ESP_CHIP_ID_ESP32C6) {
		return false;
	}

	/* The image entry is our own ELF entry point. */
	return hdr.entry_addr == (uint32_t)&__start;
}

uint32_t esp_c6_boot_image_offset(void)
{
	esp_ota_select_entry_t sel[2];
	bool valid[2];
	int active = -1;

	for (int i = 0; i < 2; i++) {
		if (esp_rom_flash_read(OTADATA_ADDR + i * OTA_SEL_STRIDE, &sel[i],
				       sizeof(sel[i]), true) != 0) {
			valid[i] = false;
			continue;
		}
		valid[i] = ota_sel_valid(&sel[i]);
	}

	if (valid[0] && valid[1]) {
		active = sel[0].ota_seq >= sel[1].ota_seq ? 0 : 1;
	} else if (valid[0]) {
		active = 0;
	} else if (valid[1]) {
		active = 1;
	}

	if (active >= 0) {
		switch ((sel[active].ota_seq - 1) % OTA_APP_COUNT) {
		case 0:
			return OTA0_ADDR;
		case 1:
			return OTA1_ADDR;
		}
	}

	/* otadata unusable — scan candidate offsets for our own image */
	if (image_matches_self(OTA0_ADDR)) {
		return OTA0_ADDR;
	}
	if (image_matches_self(OTA1_ADDR)) {
		return OTA1_ADDR;
	}
	if (image_matches_self(BOOT_ADDR)) {
		return BOOT_ADDR;
	}

	return BOOT_ADDR;
}
