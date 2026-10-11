#ifndef INPUT_H
#define INPUT_H

#include <stdint.h>

typedef enum {
    KEY_NONE,
    KEY_SHORT,
    KEY_LONG,  /* held >= 800 ms, reported once while still held */
    KEY_DOUBLE,
} ui_key_t;

void input_init(void);
/* 1 ms, from the SysTick ISR. */
void input_tick(void);

/* Detents turned since the last call (signed). */
int16_t input_take_steps(void);
ui_key_t input_take_key(void);
/* millis() of the last knob or button activity. */
uint32_t input_last_activity(void);

#endif
