/*
 * ibus.c - FlySky iBUS serial RC protocol driver
 *
 * iBUS Frame Format (32 bytes total):
 *   Byte 0:     0x20 (frame length = 32)
 *   Byte 1:     0x40 (command: channel data)
 *   Bytes 2-29: 14 channels, each as little-endian uint16_t
 *   Bytes 30-31: Checksum (little-endian), computed as
 *                0xFFFF - sum(bytes 0..29)
 *
 * Frames arrive at 115200 baud, approximately every 7ms (~140Hz).
 */

#include "ibus.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(ibus, CONFIG_LOG_DEFAULT_LEVEL);

#define IBUS_FRAME_LEN    32
#define IBUS_HEADER_LEN   0x20
#define IBUS_HEADER_CMD   0x40
#define IBUS_CHANNEL_CENTER 1500

/* Timeout for considering the link lost */
#define IBUS_TIMEOUT_MS   500

/* Reception state machine */
static uint8_t rx_buf[IBUS_FRAME_LEN];
static uint8_t rx_idx;

/* Shared channel data (written from ISR, read from flight loop) */
static uint16_t channels[IBUS_MAX_CHANNELS];
static int64_t last_valid_frame_time;

static void ibus_process_frame(void)
{
	/* Validate header */
	if (rx_buf[0] != IBUS_HEADER_LEN || rx_buf[1] != IBUS_HEADER_CMD) {
		return;
	}

	/* Validate checksum: 0xFFFF - sum(bytes 0..29) == bytes 30..31 (LE) */
	uint16_t checksum = 0xFFFF;

	for (int i = 0; i < IBUS_FRAME_LEN - 2; i++) {
		checksum -= rx_buf[i];
	}

	uint16_t rx_checksum = (uint16_t)rx_buf[30] | ((uint16_t)rx_buf[31] << 8);

	if (checksum != rx_checksum) {
		return;
	}

	/* Extract channels (little-endian uint16_t pairs starting at byte 2) */
	for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
		uint16_t val = (uint16_t)rx_buf[2 + i * 2] |
			       ((uint16_t)rx_buf[3 + i * 2] << 8);

		/* Sanity-clamp to valid RC range */
		if (val >= 800 && val <= 2200) {
			channels[i] = val;
		}
	}

	last_valid_frame_time = k_uptime_get();
}

static void ibus_uart_isr(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uart_irq_update(dev);

	while (uart_irq_rx_ready(dev)) {
		uint8_t byte;

		if (uart_fifo_read(dev, &byte, 1) != 1) {
			break;
		}

		if (rx_idx == 0 && byte != IBUS_HEADER_LEN) {
			/* Wait for frame start byte */
			continue;
		}

		rx_buf[rx_idx++] = byte;

		if (rx_idx >= IBUS_FRAME_LEN) {
			ibus_process_frame();
			rx_idx = 0;
		}
	}
}

int ibus_init(const struct device *uart_dev)
{
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("iBUS UART device is not ready");
		return -ENODEV;
	}

	/* Initialize channels to center position */
	for (int i = 0; i < IBUS_MAX_CHANNELS; i++) {
		channels[i] = IBUS_CHANNEL_CENTER;
	}

	rx_idx = 0;
	last_valid_frame_time = 0;

	/* Configure UART interrupt-driven RX */
	uart_irq_callback_set(uart_dev, ibus_uart_isr);
	uart_irq_rx_enable(uart_dev);

	LOG_INF("iBUS receiver initialized on %s", uart_dev->name);
	return 0;
}

uint16_t ibus_channel(uint8_t ch)
{
	if (ch >= IBUS_MAX_CHANNELS) {
		return IBUS_CHANNEL_CENTER;
	}

	return channels[ch];
}

bool ibus_is_active(void)
{
	if (last_valid_frame_time == 0) {
		return false;
	}

	return (k_uptime_get() - last_valid_frame_time) < IBUS_TIMEOUT_MS;
}
