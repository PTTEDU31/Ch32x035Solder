#include "tips.h"

#include <stddef.h>

#include "config.h"
#include "sense.h"

/*
 * T12: Hakko T12 curve (as used by IronOS/KSGER), uV vs. rise above the
 *      cold junction. Heater ~8 ohm, 70 W class. 20 V fixed gives 50 W,
 *      24 V AVS (boost) gives 72 W.
 * C210: THEORY. JBC C210, heater ~2 ohm in series with the TC, ~10 uV/C
 *      (polynomial from an earlier C210 station, converted to uV). Low
 *      resistance: needs PPS/AVS 5 A, typically 9-10 V.
 * C245: THEORY, disabled. ~2.5 ohm, ~20 uV/C (eevblog measurements), 24 V
 *      class. Needs a tip-ID scheme on the board before it can be enabled.
 * C410 (JBC C470?): not defined, disabled until the user gives its data.
 *
 * Full scale at TEMP_GAIN 201 is ~16.4 mV, so T12 reads up to ~500 C rise.
 */
static const tip_t tips[] = {
    {
        .name = "T12",
        .id_class = ID_CLASS_GND,
        .enabled = 1,
        .max_mv = 24000,
        .max_w = 75,
        /* Measured: ~2.1 A at 11.6 V on the first heat (~5.5 ohm cold).
         * Datasheet T12 is ~8 ohm hot; use the lower value for the PD
         * current check so the source is never asked for too little. */
        .r_mohm = 5500,
        .max_ma = 4000,
        .max_c = 450,
        .tc_uv = {0, 1244, 2351, 3426, 4544, 5750, 7059, 8456, 9896, 11304, 12575},
    },
    {
        .name = "C210",
        .id_class = ID_CLASS_OPEN,
        .enabled = 1,
        .max_mv = 12000,
        .max_w = 40,
        .r_mohm = 2000,
        .max_ma = 5000,
        .max_c = 450,
        .tc_uv = {0, 526, 1050, 1571, 2090, 2607, 3122, 3635, 4147, 4656, 5163},
    },
    {
        .name = "C245",
        .id_class = ID_CLASS_MID,
        .enabled = 0,
        .max_mv = 24000,
        .max_w = 130,
        .r_mohm = 2500,
        .max_ma = 5000,
        .max_c = 450,
        .tc_uv = {0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000},
    },
};

static int8_t offsets[sizeof(tips) / sizeof(tips[0])];

static int8_t offset_of(const tip_t *t)
{
    return offsets[t - tips];
}

void tip_set_offset(const tip_t *t, int8_t offset_c)
{
    if (t) {
        offsets[t - tips] = offset_c;
    }
}

const tip_t *tip_for_id(uint8_t id_class)
{
    for (size_t i = 0; i < sizeof(tips) / sizeof(tips[0]); i++) {
        if (tips[i].enabled && tips[i].id_class == id_class) {
            return &tips[i];
        }
    }
    return NULL;
}

/* 64-bit: raw * 3.3e6 and uv * 201 * 4095 overflow 32 bits. */
static uint32_t raw_to_uv(uint16_t raw)
{
    return (uint32_t)((uint64_t)raw * ADC_VREF_MV * 1000U / ADC_FULL_SCALE / TEMP_GAIN);
}

static uint16_t uv_to_raw(uint32_t uv)
{
    uint64_t raw = (uint64_t)uv * TEMP_GAIN * ADC_FULL_SCALE / (ADC_VREF_MV * 1000U);

    return raw > ADC_FULL_SCALE ? ADC_FULL_SCALE : (uint16_t)raw;
}

int16_t tip_raw_to_c(const tip_t *t, uint16_t raw)
{
    uint32_t uv = raw_to_uv(raw);
    uint8_t i;

    for (i = 1; i < TC_POINTS - 1 && uv > t->tc_uv[i]; i++) {
    }
    /* Linear between points i-1 and i (extrapolates past the last one). */
    {
        int32_t u0 = t->tc_uv[i - 1], u1 = t->tc_uv[i];
        int32_t rise = (int32_t)(i - 1) * 50 + ((int32_t)uv - u0) * 50 / (u1 - u0);
        return (int16_t)(rise + AMBIENT_C + offset_of(t));
    }
}

uint16_t tip_c_to_raw(const tip_t *t, int16_t c)
{
    int32_t rise = c - AMBIENT_C - offset_of(t);
    uint8_t i;

    if (rise <= 0) {
        return 0;
    }
    i = (uint8_t)(rise / 50);
    if (i >= TC_POINTS - 1) {
        i = TC_POINTS - 2;
    }
    {
        int32_t u0 = t->tc_uv[i], u1 = t->tc_uv[i + 1];
        int32_t uv = u0 + (u1 - u0) * (rise - (int32_t)i * 50) / 50;
        return uv_to_raw((uint32_t)uv);
    }
}
