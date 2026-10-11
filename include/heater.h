#ifndef HEATER_H
#define HEATER_H

#include <stdint.h>

#include "tips.h"

typedef enum {
    HS_NO_TIP,    /* nothing detected, PD at 5 V */
    HS_DETECT,    /* tip ID seen, waiting for 5 V + presence probe */
    HS_NEGOTIATE, /* tip known, waiting for its PD contract */
    HS_RUN,       /* controlling temperature */
    HS_SLEEP,     /* on stand: holds the sleep temperature */
    HS_OFF,       /* heater off: auto-off on the stand, or held knob */
    HS_ERROR,     /* latched until the tip is removed */
} heater_state_t;

typedef enum {
    HE_NONE,
    HE_TC_OPEN,
    HE_ID_CHANGED,
    HE_VIN,
    HE_OVERTEMP,
    HE_RUNAWAY,   /* tip did not warm up under power: sensor wrong */
    HE_NO_POWER,  /* no usable PD contract for this tip */
    HE_TEST_DONE, /* FIRST_HEAT session time is over */
    HE_OVERCURRENT, /* heater current above the limit (short?) */
} heater_err_t;

typedef struct {
    heater_state_t state;
    heater_err_t err;
    const tip_t *tip;
    int16_t temp_c;
    int16_t target_c;
    uint8_t power_pct;
    uint16_t watts_x10;   /* from measured current x V_IN x duty */
    uint8_t boost;
    uint8_t waking;       /* fast wake from sleep in progress */
    uint8_t deep_sleep;   /* second sleep stage */
    uint16_t boost_left_s;
    uint16_t sleep_left_s; /* until auto-off, 0 if off disabled */
    uint8_t off_manual;   /* HS_OFF entered with the knob (lifting does not wake) */
} heater_status_t;

void heater_init(void);
/* Call from the main loop; runs the PID loop once per heater cycle. */
void heater_poll(void);

void heater_set_target(int16_t c);
void heater_set_sleep_target(int16_t c);
/* Two-stage sleep: stage 1 holds sleep_c for stage1_s seconds, then
 * deep_c until auto-off. */
void heater_set_sleep_stages(uint16_t stage1_s, int16_t deep_c);
/* Wake from sleep with extra voltage/power for up to wake_s seconds. */
void heater_set_fast_wake(uint8_t enable);
/* Knob activity: wakes from sleep/off like lifting the iron. */
void heater_wake(void);
/* Switch the heater off now (held knob). Only the knob wakes it again. */
void heater_power_off(void);
/* Minutes on the stand before the heater switches off, 0 = never. */
void heater_set_auto_off(uint16_t minutes);
/* Boost: raises the target by boost_c for boost_s seconds and allows the
 * tip's highest voltage. */
void heater_boost(int16_t boost_c, uint16_t boost_s);

const heater_status_t *heater_status(void);
const char *heater_state_name(heater_state_t s);
const char *heater_err_name(heater_err_t e);

#endif
