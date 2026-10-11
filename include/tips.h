#ifndef TIPS_H
#define TIPS_H

#include <stdint.h>

/*
 * Tip types. Values not measured on this board are marked THEORY in tips.c
 * and must be checked during the first supervised heat.
 */

#define TC_POINTS 11 /* curve points at 0, 50, ..., 500 C above ambient */

typedef struct {
    const char *name;
    uint8_t id_class;          /* ID_CLASS_* from sense.h */
    uint8_t enabled;
    uint16_t max_mv;           /* never request more than this */
    uint16_t max_w;            /* power cap */
    uint16_t r_mohm;           /* nominal heater resistance, hot */
    uint16_t max_ma;           /* current limit for this tip */
    uint16_t max_c;            /* highest setpoint */
    uint16_t tc_uv[TC_POINTS]; /* thermocouple output vs. temperature rise */
} tip_t;

/* First enabled tip matching the ID class, or NULL. */
const tip_t *tip_for_id(uint8_t id_class);

/* Per-tip offset in C added to the reading (user calibration against a
 * tip thermometer at the working temperature). */
void tip_set_offset(const tip_t *t, int8_t offset_c);

/* Raw ADC (TEMP channel) <-> temperature in C, using the tip's curve,
 * TEMP_GAIN, AMBIENT_C and the offset. */
int16_t tip_raw_to_c(const tip_t *t, uint16_t raw);
uint16_t tip_c_to_raw(const tip_t *t, int16_t c);

#endif
