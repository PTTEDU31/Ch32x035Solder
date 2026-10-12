#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/* Drive the heater gate to its OFF level. Safe to call at any time. */
void heater_force_off(void);
/* Gate states for heater_pin(). Only sense.c (SysTick ISR) may call it with
 * anything but HEATER_OFF. */
enum { HEATER_OFF = 0, HEATER_SOLID = 1, HEATER_PWM = 2 };
/* HEATER_SOLID: PB0 is a GPIO held at the ON level (presence probe at 5 V,
 * where the bootstrap is too weak for 20 kHz edges). HEATER_PWM: PB0 is
 * handed to TIM1 CH2N (HEATER_PWM_OFF_PCT, same as SOLID without it).
 * HEATER_OFF: GPIO at the OFF level, whatever the previous state. PWM stays
 * off until heater_pwm_enable(). */
void heater_pin(int mode);
/* Called by out_init() once TIM1 CH2N is configured. */
void heater_pwm_enable(void);

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
