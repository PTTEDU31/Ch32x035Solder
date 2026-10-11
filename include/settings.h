#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>

typedef struct {
    int16_t target_c;
    int16_t sleep_c;
    int16_t boost_c;     /* boost raises the target by this much */
    uint16_t boost_s;    /* boost duration */
    uint16_t off_min;    /* minutes on the stand before heater off, 0 = never */
    uint8_t backlight;   /* % */
    uint8_t beep;        /* 0/1 */
    uint8_t fahrenheit;  /* 0/1 */
    uint8_t ui_style;    /* 0 = gauge, 1 = graph */
    uint8_t fast_wake;   /* 0/1 */
    uint8_t sleep1_min;  /* minutes at sleep_c before deep sleep */
    int16_t deep_c;      /* deep sleep temperature */
    int8_t cal_t12;      /* tip offsets, C, added to the reading */
    int8_t cal_c210;
} settings_t;

extern settings_t settings;

/* Loads the newest valid copy, or defaults. */
void settings_load(void);
/* Marks settings dirty; written later by settings_poll() when it is safe. */
void settings_touch(void);
/* Writes a dirty copy after SETTINGS_IDLE_MS without changes, with the
 * heater suspended, right after PD has been quiet. Call from the main loop. */
void settings_poll(void);

#endif
