/*
 * Copyright (c) 2025 Cognition AI
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr port of the ESP-IDF sdio_slave driver API surface
 * (driver/sdio_slave.h), for the ESP32-C6 hosted-slave role.
 */

#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <hal/sdio_slave_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Callback of the sdio slave driver invoked when a host interrupt bit
 * (0-7) is set by the host. Runs in ISR context. */
typedef void (*sdio_slave_event_cb_t)(uint8_t pos);

typedef enum {
    SDIO_SLAVE_FLAG_INTERNAL_PULLUP      = BIT(0), /**< Enable internal pullups for enabled pins */
    SDIO_SLAVE_FLAG_DEFAULT_SPEED        = BIT(1), /**< Disable the highspeed support */
    SDIO_SLAVE_FLAG_HOST_INTR_DISABLED   = BIT(2), /**< Disable D1 (host interrupt) line */
    SDIO_SLAVE_FLAG_DAT2_DISABLED        = BIT(3), /**< Disable D2 line */
} sdio_slave_flags_t;

typedef struct {
    sdio_slave_sending_mode_t   sending_mode;   /**< Sending mode. */
    sdio_slave_timing_t         timing;         /**< Timing mode. */
    uint32_t                    flags;          /**< sdio_slave_flags_t OR-ed */
    uint8_t                     send_queue_size;/**< Max queued send buffers */
    size_t                      recv_buffer_size;/**< Negotiated recv buffer size */
    sdio_slave_event_cb_t       event_cb;       /**< Host interrupt callback, ISR context */
} sdio_slave_config_t;

typedef void *sdio_slave_buf_handle_t;

/* sdio_slave_hostint_t is provided by hal/sdio_slave_types.h */
typedef uint32_t sdio_slave_hostinten_t;   /**< Host interrupt enable bits */

int  sdio_slave_initialize(const sdio_slave_config_t *config);
void sdio_slave_deinit(void);
int  sdio_slave_start(void);
int  sdio_slave_reset(void);
void sdio_slave_stop(void);

int sdio_slave_wait_int(uint8_t pos, k_timeout_t wait);
int sdio_slave_send_host_int(uint8_t pos);
uint8_t sdio_slave_read_reg(uint8_t pos);
int  sdio_slave_write_reg(uint8_t pos, uint8_t reg);
sdio_slave_hostint_t sdio_slave_get_host_intena(void);
void sdio_slave_set_host_intena(sdio_slave_hostint_t mask);
void sdio_slave_clear_host_int(sdio_slave_hostint_t mask);

int sdio_slave_send_queue(uint8_t *addr, size_t len, void *arg, k_timeout_t wait);
int sdio_slave_send_get_finished(void **out_arg, k_timeout_t wait);
int sdio_slave_transmit(uint8_t *addr, size_t len);

int  sdio_slave_recv_load_buf(sdio_slave_buf_handle_t handle);
sdio_slave_buf_handle_t sdio_slave_recv_register_buf(uint8_t *start);
int  sdio_slave_recv_unregister_buf(sdio_slave_buf_handle_t handle);
int  sdio_slave_recv(sdio_slave_buf_handle_t *handle_ret, uint8_t **out_addr,
                     size_t *out_len, k_timeout_t wait);
int  sdio_slave_recv_packet(sdio_slave_buf_handle_t *handle_ret, k_timeout_t wait);
uint8_t *sdio_slave_recv_get_buf(sdio_slave_buf_handle_t handle, size_t *len_o);

#ifdef __cplusplus
}
#endif
