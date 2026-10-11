#ifndef SENSE_H
#define SENSE_H

#include <stdint.h>

#include "config.h"

/*
 * 1 ms sampling and heater timing, run from the SysTick ISR. This module is
 * the only user of ADC1 and the only code that turns the heater on.
 *
 * Heater cycle (HEATER_PERIOD_MS): heater off for HEATER_SETTLE_MS, sample
 * TEMP, then on for the requested time if every interlock passes; the shunt
 * current is sampled during the on window.
 */

enum {
    ID_CLASS_GND,   /* T12 */
    ID_CLASS_MID,   /* resistor ID (not used yet) */
    ID_CLASS_OPEN,  /* C210, or nothing plugged in */
    ID_CLASS_UNKNOWN,
};

/* Fault bits latched by the ISR; heater stays off until sense_clear_faults(). */
#define FAULT_TC_OPEN 0x01  /* TEMP saturated: tip or handle removed */
#define FAULT_ID 0x02       /* tip ID changed */
#define FAULT_VIN 0x04      /* V_IN left the allowed window */
#define FAULT_OVERTEMP 0x08 /* TEMP above the hard limit */
#define FAULT_OVERCURRENT 0x10 /* heater current above the limit */

typedef struct {
    volatile uint16_t temp_raw; /* last off-window TEMP, average of 8 */
    volatile uint16_t cur_raw;  /* last on-window shunt reading, 0 if none */
    volatile uint16_t vin_mv;
    volatile uint16_t id_raw;
    volatile uint8_t id_class;   /* debounced */
    volatile uint8_t on_ms;      /* on time applied in the last cycle */
    volatile uint8_t stand;      /* debounced, 1 = on stand */
    volatile uint8_t faults;
    volatile uint32_t cycle;     /* increments at each TEMP sample */
    volatile uint16_t t_early[HEATER_SETTLE_MS]; /* TEMP 1..n ms into the off window (settle check) */
    volatile uint16_t probe_cur;  /* shunt reading of the last presence probe */
    volatile uint16_t cur_peak;   /* highest shunt reading in the last on window */
    volatile uint16_t probe_count;
} sense_t;

extern sense_t sense;

void sense_init(void);
void sense_tick(void);

/* Heater on time for the following cycles, in ms (clamped to the window). */
void sense_set_on_ms(uint8_t on_ms);
/* Interlocks checked before every on pulse. permit = 0 forces off. */
void sense_set_permit(int permit, uint16_t vin_min_mv, uint16_t vin_max_mv, uint16_t temp_max_raw,
                      uint8_t id_class);
/* Current limit in mA, checked every 1 ms while the heater is on. */
void sense_set_current_limit(uint16_t max_ma);
/* Force the heater off immediately and keep it off until resumed. */
void sense_suspend(int on);
void sense_clear_faults(void);

/* Presence probe: one HEATER_PROBE_MS pulse at the next cycle (subject to
 * the same interlocks). Returns 1 when done, with the shunt reading. */
void sense_probe_start(void);
int sense_probe_done(uint16_t *cur_raw);

/* Max on time per cycle in ms. */
#define SENSE_MAX_ON_MS (HEATER_PERIOD_MS - HEATER_SETTLE_MS - 2)

#endif
