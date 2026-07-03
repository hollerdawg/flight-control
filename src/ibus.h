/*
 * ibus.h - FlySky iBUS serial RC protocol driver
 *
 * Provides interrupt-driven reception of iBUS frames on a UART device.
 * Each frame carries up to 14 RC channels (1000-2000 range).
 */

#ifndef IBUS_H_
#define IBUS_H_

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

/* Maximum number of channels in an iBUS frame */
#define IBUS_MAX_CHANNELS 14

/**
 * @brief Initialize the iBUS receiver on the given UART device.
 *
 * Configures interrupt-driven RX and begins frame reception.
 *
 * @param uart_dev Pointer to the UART device (must support interrupt-driven API).
 * @return 0 on success, negative errno on failure.
 */
int ibus_init(const struct device *uart_dev);

/**
 * @brief Get the current value of an iBUS channel.
 *
 * @param ch Channel index (0-based, 0 = CH1, up to 13 = CH14).
 * @return Channel value in the range 1000-2000, or 1500 (center) if
 *         the channel index is out of range or no valid frame has been received.
 */
uint16_t ibus_channel(uint8_t ch);

/**
 * @brief Check if the iBUS receiver is actively receiving valid frames.
 *
 * @return true if a valid frame was received within the last 500ms.
 */
bool ibus_is_active(void);

#endif /* IBUS_H_ */
