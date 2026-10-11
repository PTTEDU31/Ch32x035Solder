#ifndef OUT_H
#define OUT_H

#include <stdint.h>

/* Backlight on TIM1_CH1 (PB9, BLK_HZ) and buzzer on TIM3_CH2 (PB5 via
 * partial remap 1, BEEP_HZ), so each runs at its own frequency.
 *
 * PB0 (H_PWM) is TIM1_CH2N, but the heater is switched by the 1 ms tick in
 * sense.c (TEMP must be sampled in its off window), so CH2/CH2N stay
 * disabled and PB0 stays a plain GPIO output. */
void out_init(void);
/* 1 ms, from the SysTick ISR (ends beeps). */
void out_tick(void);

void backlight_set(uint8_t pct);
void beep(uint16_t ms);

#endif
