/*
 * Copyright (c) 2025 Cognition AI
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zephyr port of esp_driver_sdio/sdio_slave.c (ESP-IDF v5.4) for ESP32-C6.
 * The driver logic is preserved verbatim; only the OS primitives are
 * replaced: semaphores/queues/spinlocks -> k_sem/k_msgq/k_spinlock,
 * esp_intr_alloc -> Zephyr intc_esp32, heap_caps_* -> k_heap/k_malloc.
 */

#include <string.h>
#include <errno.h>
#include <assert.h>

#include <sys/queue.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/interrupt_controller/intc_esp32.h>

#include <soc/soc_memory_layout.h>
#include <soc/soc_caps.h>
#include <soc/interrupts.h>
#include <soc/io_mux_reg.h>
#include <esp_attr.h>
#include <esp_err.h>
#include <esp_memory_utils.h>
#include <hal/sdio_slave_hal.h>
#include <hal/sdio_slave_periph.h>
#include <hal/sdio_slave_ll.h>
#include <hal/misc.h>
#include <hal/log.h>

#include "drivers/esp32c6_sdio_slave.h"

LOG_MODULE_REGISTER(sdio_slave, CONFIG_LOG_DEFAULT_LEVEL);

/* C6 DMA descriptor: 14-bit size field */
#define SDIO_SLAVE_LL_DMA_DESC_MAX_BUF_SIZE          ((1 << 14) - 1)
#define SDIO_SLAVE_DMA_DESC_MAX_BUF_SIZE_ALIGNED_DOWN \
	(SDIO_SLAVE_LL_DMA_DESC_MAX_BUF_SIZE & ~0x3U)

#define SDIO_SLAVE_CHECK(res, ret_val) do { if (!(res)) { return ret_val; } } while (0)

typedef struct recv_desc_s {
	union {
		struct {
			sdio_slave_hal_recv_desc_t hal_desc;
			uint32_t not_receiving;
		};
		struct {
			uint32_t _reserved0;
			uint32_t _reserved1;
			TAILQ_ENTRY(recv_desc_s) tail_entry;
		};
	};
} recv_desc_t;

typedef TAILQ_HEAD(recv_tailq_head_s, recv_desc_s) recv_tailq_t;

typedef struct {
	sdio_slave_config_t     config;
	sdio_slave_context_t    *hal;
	intr_handle_t           intr_handle;
	/*------- events ---------------*/
	union {
		struct k_sem events[9];         /* 0-7 host ints, 8 = recv */
		struct {
			struct k_sem _events[8];
			struct k_sem recv_event;
		};
	};
	struct k_spinlock       reg_spinlock;
	/*------- sending ---------------*/
	struct k_sem            remain_cnt;
	struct k_spinlock       write_spinlock;
	struct k_msgq           ret_queue;
	void                    *ret_queue_buf;
	/*------- receiving ---------------*/
	recv_tailq_t            recv_reg_list;
	struct k_spinlock       recv_spinlock;
} sdio_context_t;

static sdio_context_t context;

static void sdio_intr(void *);
static void sdio_intr_host(void *);
static void sdio_intr_send(void *);
static void sdio_intr_recv(void *);

static int send_flush_data(void);
static int recv_flush_data(void);

static inline void critical_enter_recv(k_spinlock_key_t *key)
{
	*key = k_spin_lock(&context.recv_spinlock);
}

static inline void critical_exit_recv(k_spinlock_key_t key)
{
	k_spin_unlock(&context.recv_spinlock, key);
}

static void deinit_context(void)
{
	context.config = (sdio_slave_config_t){0};
	context.ret_queue_buf = NULL;
	k_free(context.hal->send_desc_queue.data);
	context.hal->send_desc_queue.data = NULL;
	k_free(context.hal);
	context.hal = NULL;
}

static int init_context(const sdio_slave_config_t *config)
{
	SDIO_SLAVE_CHECK(*(uint32_t *)&context.config == 0, -EALREADY);
	context = (sdio_context_t){0};
	context.config = *config;

	context.hal = k_calloc(sizeof(sdio_slave_context_t), 1);
	if (context.hal == NULL) {
		goto no_mem;
	}

	context.hal->sending_mode = config->sending_mode;
	context.hal->timing = config->timing;
	context.hal->no_highspeed = (config->flags & SDIO_SLAVE_FLAG_DEFAULT_SPEED) ==
				  SDIO_SLAVE_FLAG_DEFAULT_SPEED;
	context.hal->send_queue_size = config->send_queue_size;
	context.hal->recv_buffer_size = config->recv_buffer_size;

	sdio_ringbuf_t *buf = &(context.hal->send_desc_queue);

	buf->size = SDIO_SLAVE_SEND_DESC_SIZE * (config->send_queue_size + 1);
	buf->data = k_aligned_alloc(4, buf->size);
	if (buf->data == NULL) {
		goto no_mem;
	}

	sdio_slave_hal_init(context.hal);

	k_sem_init(&context.recv_event, 0, K_SEM_MAX_LIMIT);
	for (int i = 0; i < 8; i++) {
		k_sem_init(&context.events[i], 0, 1);
	}
	k_sem_init(&context.remain_cnt, config->send_queue_size,
		   config->send_queue_size);

	context.ret_queue_buf = k_malloc(config->send_queue_size * sizeof(void *));
	if (context.ret_queue_buf == NULL) {
		goto no_mem;
	}
	k_msgq_init(&context.ret_queue, context.ret_queue_buf, sizeof(void *),
		    config->send_queue_size);

	TAILQ_INIT(&context.recv_reg_list);
	return 0;

no_mem:
	deinit_context();
	return -ENOMEM;
}

/* Configure one SDIO pin through the IO_MUX. func comes from
 * sdio_slave_slot_info[]. Drive strength 3 (20 mA), input enabled,
 * optional internal pullup, pulldown always off. */
static void configure_pin(int pin, uint32_t func, bool pullup)
{
	uint32_t reg = IO_MUX_GPIO0_REG + (pin * 4);

	__ASSERT(pin >= 0, "invalid sdio pin");

	SET_PERI_REG_MASK(reg, FUN_IE);
	REG_SET_FIELD(reg, MCU_SEL, func);
	REG_SET_FIELD(reg, FUN_DRV, 3);
	REG_CLR_BIT(reg, FUN_PD);
	if (pullup) {
		REG_SET_BIT(reg, FUN_PU);
	}
}

static int sdio_slave_hw_init(sdio_slave_config_t *config)
{
	const sdio_slave_slot_info_t *slot = &sdio_slave_slot_info[0];
	bool pullup = config->flags & SDIO_SLAVE_FLAG_INTERNAL_PULLUP;

	configure_pin(slot->clk_gpio, slot->func, false);
	configure_pin(slot->cmd_gpio, slot->func, pullup);
	configure_pin(slot->d0_gpio, slot->func, pullup);
	if ((config->flags & SDIO_SLAVE_FLAG_HOST_INTR_DISABLED) == 0) {
		configure_pin(slot->d1_gpio, slot->func, pullup);
	}
	if ((config->flags & SDIO_SLAVE_FLAG_DAT2_DISABLED) == 0) {
		configure_pin(slot->d2_gpio, slot->func, pullup);
	}
	configure_pin(slot->d3_gpio, slot->func, pullup);

	/* SOC_RCC_IS_INDEPENDENT on C6: direct PCR pokes, no atomic needed. */
	sdio_slave_ll_enable_bus_clock(true);
	sdio_slave_ll_reset_register();

	sdio_slave_hal_hw_init(context.hal);
	return 0;
}

static void sdio_slave_hw_deinit(void)
{
	sdio_slave_ll_enable_bus_clock(false);
}

int sdio_slave_initialize(const sdio_slave_config_t *cfg)
{
	int r;
	intr_handle_t intr_handle = NULL;
	const int flags = 0;

	r = init_context(cfg);
	SDIO_SLAVE_CHECK(r == 0, r);

	r = esp_intr_alloc_intrstatus(
		ETS_SLC0_INTR_SOURCE,
		flags,
		(uint32_t)sdio_slave_hal_get_intr_status_reg(context.hal),
		sdio_slave_ll_intr_status_mask,
		sdio_intr,
		NULL,
		&intr_handle);
	SDIO_SLAVE_CHECK(r == 0, -EIO);
	context.intr_handle = intr_handle;

	r = sdio_slave_hw_init((sdio_slave_config_t *)cfg);
	if (r != 0) {
		return r;
	}

	sdio_slave_reset();
	return 0;
}

void sdio_slave_deinit(void)
{
	sdio_slave_hw_deinit();

	recv_desc_t *temp_desc;
	recv_desc_t *desc;

	TAILQ_FOREACH_SAFE(desc, &context.recv_reg_list, tail_entry, temp_desc) {
		TAILQ_REMOVE(&context.recv_reg_list, desc, tail_entry);
		k_free(desc);
	}
	while (1) {
		desc = (recv_desc_t *)sdio_slave_hal_recv_unload_desc(context.hal);
		if (desc == NULL) {
			break;
		}
		k_free(desc);
	}
	int ret = esp_intr_free(context.intr_handle);

	__ASSERT(ret == 0, "esp_intr_free failed");
	context.intr_handle = NULL;
	deinit_context();
}

int sdio_slave_start(void)
{
	int ret;
	sdio_slave_hostint_t intr = (sdio_slave_hostint_t)UINT32_MAX;
	k_spinlock_key_t key;

	sdio_slave_hal_hostint_clear(context.hal, &intr);
	ret = sdio_slave_hal_send_start(context.hal);
	if (ret != 0) {
		return ret;
	}

	critical_enter_recv(&key);
	sdio_slave_hal_recv_start(context.hal);
	critical_exit_recv(key);

	sdio_slave_hal_set_ioready(context.hal, true);
	return 0;
}

int sdio_slave_reset(void)
{
	int err;

	err = send_flush_data();
	if (err != 0) {
		return err;
	}
	err = sdio_slave_hal_send_reset_counter(context.hal);
	if (err != 0) {
		return err;
	}
	err = recv_flush_data();
	if (err != 0) {
		return err;
	}

	k_spinlock_key_t key;

	critical_enter_recv(&key);
	sdio_slave_hal_recv_reset_counter(context.hal);
	critical_exit_recv(key);
	return 0;
}

void sdio_slave_stop(void)
{
	sdio_slave_hal_set_ioready(context.hal, false);
	sdio_slave_hal_send_stop(context.hal);
	sdio_slave_hal_recv_stop(context.hal);
}

static void sdio_intr(void *arg)
{
	sdio_intr_send(arg);
	sdio_intr_recv(arg);
	sdio_intr_host(arg);
}

/*---------------------------------------------------------------------------
 *                  Host
 *--------------------------------------------------------------------------*/
static void sdio_intr_host(void *arg)
{
	sdio_slave_ll_slvint_t int_val;

	ARG_UNUSED(arg);
	sdio_slave_hal_slvint_fetch_clear(context.hal, &int_val);
	for (int i = 0; i < 8; i++) {
		if (BIT(i) & int_val) {
			if (context.config.event_cb != NULL) {
				context.config.event_cb(i);
			}
			k_sem_give(&context.events[i]);
		}
	}
}

int sdio_slave_wait_int(uint8_t pos, k_timeout_t wait)
{
	SDIO_SLAVE_CHECK(pos < 8, -EINVAL);
	return k_sem_take(&context.events[pos], wait);
}

uint8_t sdio_slave_read_reg(uint8_t pos)
{
	return sdio_slave_hal_host_get_reg(context.hal, pos);
}

int sdio_slave_write_reg(uint8_t pos, uint8_t reg)
{
	if (pos >= 28 && pos <= 31) {
		LOG_WRN("interrupt reg, use sdio_slave_clear_int");
		return -EINVAL;
	}
	if (pos >= 64) {
		return -EINVAL;
	}

	k_spinlock_key_t key = k_spin_lock(&context.reg_spinlock);

	sdio_slave_hal_host_set_reg(context.hal, pos, reg);
	k_spin_unlock(&context.reg_spinlock, key);
	return 0;
}

sdio_slave_hostint_t sdio_slave_get_host_intena(void)
{
	sdio_slave_hostint_t host_int;

	sdio_slave_hal_hostint_get_ena(context.hal, &host_int);
	return host_int;
}

void sdio_slave_set_host_intena(sdio_slave_hostint_t mask)
{
	sdio_slave_hal_hostint_set_ena(context.hal, &mask);
}

void sdio_slave_clear_host_int(sdio_slave_hostint_t mask)
{
	sdio_slave_hal_hostint_clear(context.hal, &mask);
}

int sdio_slave_send_host_int(uint8_t pos)
{
	SDIO_SLAVE_CHECK(pos < 8, -EINVAL);
	sdio_slave_hostint_t intr = (sdio_slave_hostint_t)BIT(pos);

	sdio_slave_hal_hostint_send(context.hal, &intr);
	return 0;
}

/*---------------------------------------------------------------------------
 *                  Send
 *--------------------------------------------------------------------------*/
static void sdio_intr_send(void *arg)
{
	ARG_UNUSED(arg);
	sdio_slave_hal_send_handle_isr_invoke(context.hal);

	uint32_t returned_cnt = 0;

	if (sdio_slave_hal_send_eof_happened(context.hal)) {
		int err;

		while (1) {
			void *finished_arg;

			err = sdio_slave_hal_send_get_next_finished_arg(context.hal,
					&finished_arg, &returned_cnt);
			if (err != 0) {
				break;
			}
			__ASSERT(returned_cnt == 0, "unexpected returned_cnt");
			(void)k_msgq_put(&context.ret_queue, &finished_arg, K_NO_WAIT);
		}
		for (size_t i = 0; i < returned_cnt; i++) {
			k_sem_give(&context.remain_cnt);
		}
	}

	sdio_slave_hal_send_new_packet_if_exist(context.hal);
}

int sdio_slave_send_queue(uint8_t *addr, size_t len, void *arg, k_timeout_t wait)
{
	SDIO_SLAVE_CHECK(len > 0 && len <= SDIO_SLAVE_DMA_DESC_MAX_BUF_SIZE_ALIGNED_DOWN,
			 -EINVAL);
	SDIO_SLAVE_CHECK(esp_ptr_dma_capable(addr) && (uint32_t)addr % 4 == 0, -EINVAL);

	int cnt_ret = k_sem_take(&context.remain_cnt, wait);

	if (cnt_ret != 0) {
		return -ETIMEDOUT;
	}

	k_spinlock_key_t key = k_spin_lock(&context.write_spinlock);
	int ret = sdio_slave_hal_send_queue(context.hal, addr, len, arg);

	k_spin_unlock(&context.write_spinlock, key);
	return ret;
}

int sdio_slave_send_get_finished(void **out_arg, k_timeout_t wait)
{
	void *arg = NULL;
	int err = k_msgq_get(&context.ret_queue, &arg, wait);

	if (out_arg) {
		*out_arg = arg;
	}
	if (err != 0) {
		return -ETIMEDOUT;
	}
	return 0;
}

int sdio_slave_transmit(uint8_t *addr, size_t len)
{
	uint32_t timestamp = k_cycle_get_32();
	uint32_t ret_stamp;
	int err;

	err = sdio_slave_send_queue(addr, len, (void *)timestamp, K_FOREVER);
	if (err != 0) {
		return err;
	}
	err = sdio_slave_send_get_finished((void **)&ret_stamp, K_FOREVER);
	if (err != 0) {
		return err;
	}
	SDIO_SLAVE_CHECK(ret_stamp == timestamp, -EIO);
	return 0;
}

static int send_flush_data(void)
{
	int err;

	while (1) {
		void *finished_arg;
		uint32_t return_cnt = 0;

		err = sdio_slave_hal_send_flush_next_buffer(context.hal,
				&finished_arg, &return_cnt);
		if (err == 0) {
			(void)k_msgq_put(&context.ret_queue, &finished_arg, K_NO_WAIT);
			for (size_t i = 0; i < return_cnt; i++) {
				k_sem_give(&context.remain_cnt);
			}
		} else {
			if (err == -ENOENT) {
				err = 0;
			}
			break;
		}
	}

	if (err == -EINVAL) {
		LOG_ERR("flush data when transmission started");
	}
	return err;
}

/*---------------------------------------------------------------------------
 *                  Recv
 *--------------------------------------------------------------------------*/
#define CHECK_HANDLE_IDLE(desc) do { if (desc == NULL || !desc->not_receiving) { \
	return -EINVAL; } } while (0)

static int recv_flush_data(void)
{
	while (1) {
		if (k_sem_take(&context.recv_event, K_NO_WAIT) != 0) {
			break;
		}
		k_spinlock_key_t key;

		critical_enter_recv(&key);
		sdio_slave_hal_recv_flush_one_buffer(context.hal);
		critical_exit_recv(key);
	}
	return 0;
}

static void sdio_intr_recv(void *arg)
{
	ARG_UNUSED(arg);
	bool triggered = sdio_slave_hal_recv_done(context.hal);

	while (triggered) {
		k_spinlock_key_t key = k_spin_lock(&context.recv_spinlock);
		bool has_next_item = sdio_slave_hal_recv_has_next_item(context.hal);

		k_spin_unlock(&context.recv_spinlock, key);
		if (has_next_item) {
			k_sem_give(&context.recv_event);
			continue;
		}
		triggered = sdio_slave_hal_recv_done(context.hal);
	}
}

int sdio_slave_recv_load_buf(sdio_slave_buf_handle_t handle)
{
	recv_desc_t *desc = (recv_desc_t *)handle;
	k_spinlock_key_t key;

	CHECK_HANDLE_IDLE(desc);
	__ASSERT_NO_MSG(desc->not_receiving);

	critical_enter_recv(&key);
	TAILQ_REMOVE(&context.recv_reg_list, desc, tail_entry);
	desc->not_receiving = 0;
	sdio_slave_hal_load_buf(context.hal, &desc->hal_desc);
	critical_exit_recv(key);
	return 0;
}

sdio_slave_buf_handle_t sdio_slave_recv_register_buf(uint8_t *start)
{
	if (!(esp_ptr_dma_capable(start) && (uint32_t)start % 4 == 0)) {
		return NULL;
	}
	recv_desc_t *desc = k_aligned_alloc(4, sizeof(recv_desc_t));
	k_spinlock_key_t key;

	if (desc == NULL) {
		return NULL;
	}

	sdio_slave_hal_recv_init_desc(context.hal, &desc->hal_desc, start);
	critical_enter_recv(&key);
	TAILQ_INSERT_TAIL(&context.recv_reg_list, desc, tail_entry);
	critical_exit_recv(key);
	return desc;
}

int sdio_slave_recv(sdio_slave_buf_handle_t *handle_ret, uint8_t **out_addr,
		    size_t *out_len, k_timeout_t wait)
{
	int ret = sdio_slave_recv_packet(handle_ret, wait);

	if (ret == -EIO) {          /* ESP_ERR_NOT_FINISHED -> partial packet */
		ret = 0;
	}
	if (ret == 0) {
		recv_desc_t *desc = (recv_desc_t *)(*handle_ret);

		if (out_addr) {
			*out_addr = (uint8_t *)desc->hal_desc.buf;
		}
		if (out_len) {
			*out_len = desc->hal_desc.length;
		}
	}
	return ret;
}

int sdio_slave_recv_packet(sdio_slave_buf_handle_t *handle_ret,
			   k_timeout_t wait)
{
	SDIO_SLAVE_CHECK(handle_ret != NULL, -EINVAL);
	int err = k_sem_take(&context.recv_event, wait);
	k_spinlock_key_t key;

	if (err != 0) {
		return -ETIMEDOUT;
	}

	critical_enter_recv(&key);
	recv_desc_t *desc =
		(recv_desc_t *)sdio_slave_hal_recv_unload_desc(context.hal);

	__ASSERT_NO_MSG(desc != NULL && desc->hal_desc.owner == 0);
	TAILQ_INSERT_TAIL(&context.recv_reg_list, desc, tail_entry);
	critical_exit_recv(key);

	*handle_ret = (sdio_slave_buf_handle_t)desc;

	if (!desc->hal_desc.eof) {
		return -EIO;
	}
	return 0;
}

int sdio_slave_recv_unregister_buf(sdio_slave_buf_handle_t handle)
{
	recv_desc_t *desc = (recv_desc_t *)handle;
	k_spinlock_key_t key;

	CHECK_HANDLE_IDLE(desc);
	critical_enter_recv(&key);
	TAILQ_REMOVE(&context.recv_reg_list, desc, tail_entry);
	critical_exit_recv(key);
	k_free(desc);
	return 0;
}

uint8_t *sdio_slave_recv_get_buf(sdio_slave_buf_handle_t handle, size_t *len_o)
{
	if (handle == NULL) {
		return NULL;
	}
	recv_desc_t *desc = (recv_desc_t *)handle;

	if (len_o != NULL) {
		*len_o = desc->hal_desc.length;
	}
	return (uint8_t *)desc->hal_desc.buf;
}
