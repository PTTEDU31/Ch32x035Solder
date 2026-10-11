#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/* Drive the heater gate to its OFF level. Safe to call at any time. */
void heater_force_off(void);
/* Gate on/off. Only sense.c (SysTick ISR) may call this with on = 1. */
void heater_pin(int on);

/* Clocks, heater-off, UART, GPIO inputs, ADC, then the 1 ms SysTick that
 * runs sense_tick(), input_tick() and out_tick(). */
void board_init(void);

uint32_t millis(void);
/* Free-running 48 MHz cycle counter (SysTick CNT, wraps every 1 ms). */
uint32_t cycles_now(void);
/* Longest SysTick ISR seen, in cycles (48 = 1 us); reset on read. */
uint32_t isr_max_cycles(void);
/* Busy-wait using the SysTick counter; never use the SDK Delay_* functions,
 * they reprogram SysTick. */
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

/* One conversion. Only the SysTick ISR (sense.c) may call this: ADC1 is
 * not re-entrant. */
uint16_t adc_read(uint8_t channel);

/* Cached V_IN in mV, updated every 1 ms by the ISR. */
uint32_t vin_mv(void);

void iwdg_init(void);
void iwdg_feed(void);

#endif
