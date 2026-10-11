#ifndef LOG_H
#define LOG_H

/*
 * Non-blocking UART log on USART1 TX (PB10).
 *
 * printf()/log_printf() format into a 1 KB ring buffer and return at once;
 * DMA1 channel 4 (USART1_TX) drains it in the background. When the buffer
 * is full the new text is dropped (and counted) instead of blocking, so a
 * log line can never delay the PD stack or the heater.
 */

#include <stdint.h>

void log_init(uint32_t baud);
/* Starts the next DMA chunk if the previous one has finished. Called from
 * the main loop; cheap when idle. */
void log_poll(void);
uint32_t log_dropped(void);

#endif
