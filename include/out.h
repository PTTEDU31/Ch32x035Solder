#ifndef OUT_H
#define OUT_H

#include <stdint.h>

/* Backlight on TIM1_CH1 (PB9, BLK_HZ) and buzzer on TIM3_CH2 (PB5 via
 * partial remap 1, BEEP_HZ).
 *
 * The heater gate (H_PWM, PB0) is TIM1_CH2N on the same 20 kHz timer as the
 * backlight (HEATER_PWM_OFF_PCT): the 1 ms tick in sense.c decides when the
 * heater is on (TEMP must be sampled in its off window) and heater_pin()
 * hands PB0 to the timer for that window, so the gate is refreshed every
 * 50 us instead of in audible 1 ms gaps. */
void out_init(void);
/* 1 ms, from the SysTick ISR (ends beeps). */
void out_tick(void);

void backlight_set(uint8_t pct);
void beep(uint16_t ms);

#endif
