#include "sense.h"

#include <ch32x035.h>

#include "board.h"
#include "config.h"

sense_t sense;

static uint8_t ready;
static uint8_t phase;     /* ms within the heater cycle */
static uint8_t on_until;  /* phase at which the heater turns off, 0 = off */
static uint8_t heater_on;
static uint8_t probing;
static uint32_t vin_f;    /* raw << 4 */
static uint8_t id_cnt, id_pending;
static uint8_t stand_cnt;

static volatile uint8_t req_on_ms;
static volatile uint8_t permit;
static volatile uint8_t suspended;
static volatile uint16_t win_min_mv, win_max_mv, temp_max;
static volatile uint8_t id_expect;
static volatile uint8_t probe_req, probe_done;
static volatile uint16_t probe_raw;
static volatile uint16_t cur_soft_raw = 0xFFFF, cur_hard_raw = 0xFFFF;

static uint16_t ma_to_raw(uint32_t ma)
{
    uint32_t raw = ma * CUR_COUNTS_PER_A / 1000U + CUR_OFFSET_RAW;
    return raw > 4095 ? 4095 : (uint16_t)raw;
}

static void off(void)
{
    heater_pin(0);
    heater_on = 0;
    on_until = 0;
    if (probing) {
        /* Ends the probe whatever stopped it (end of pulse or interlock). */
        probing = 0;
        sense.probe_cur = probe_raw;
        sense.probe_count++;
        probe_done = 1;
    }
}

static uint8_t classify_id(uint16_t raw)
{
    if (raw <= ID_GND_MAX_RAW) {
        return ID_CLASS_GND;
    }
    if (raw >= ID_OPEN_MIN_RAW) {
        return ID_CLASS_OPEN;
    }
    return ID_CLASS_MID;
}

static uint16_t avg(uint8_t ch, uint8_t n)
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < n; i++) {
        sum += adc_read(ch);
    }
    return (uint16_t)(sum / n);
}

/* Every interlock that must hold for the heater to be (or stay) on. */
static int interlocks_ok(void)
{
    return HEATER_ENABLE && permit && !suspended && !sense.faults && sense.vin_mv >= win_min_mv &&
           sense.vin_mv <= win_max_mv && sense.id_class == id_expect;
}

void sense_init(void)
{
    sense.id_class = ID_CLASS_UNKNOWN;
    id_pending = ID_CLASS_UNKNOWN;
    cur_soft_raw = cur_hard_raw = ma_to_raw(CUR_HARD_MAX_MA);
    sense.temp_raw = 4095;
    ready = 1;
}

void sense_tick(void)
{
    uint16_t raw;

    if (!ready) {
        return;
    }

    /* V_IN every tick, IIR filtered (tau ~4 ms). */
    raw = adc_read(ADC_CH_VIN);
    vin_f = (uint32_t)((int32_t)vin_f + (((int32_t)raw << 4) - (int32_t)vin_f) / 4);
    {
        /* vin_f <= 65520: keep every intermediate below 2^32. */
        uint32_t mv16 = vin_f * ADC_VREF_MV / ADC_FULL_SCALE;
        mv16 = mv16 * ((VIN_R_TOP + VIN_R_BOT) / 100) / (VIN_R_BOT / 100);
        sense.vin_mv = (uint16_t)(mv16 >> 4);
    }

    /* Tip ID every 10 ms, 3 equal readings to change. */
    if (phase % 10 == 5) {
        uint8_t c;
        sense.id_raw = avg(ADC_CH_ID, 2);
        c = classify_id(sense.id_raw);
        if (c == id_pending) {
            if (id_cnt < 3 && ++id_cnt == 3) {
                sense.id_class = c;
            }
        } else {
            id_pending = c;
            id_cnt = 0;
        }
    }

    /* Stand switch, 50 ms debounce. */
    if (GPIO_ReadInputDataBit(SLEEP_PORT, SLEEP_PIN) == Bit_RESET) {
        if (stand_cnt < 50 && ++stand_cnt == 50) {
            sense.stand = 1;
        }
    } else {
        if (stand_cnt > 0 && --stand_cnt == 0) {
            sense.stand = 0;
        }
    }

    if (heater_on) {
        /* Drop out at once if anything goes wrong mid-pulse. */
        if (permit && sense.id_class != id_expect) {
            sense.faults |= FAULT_ID;
        }
        if (sense.vin_mv < win_min_mv || sense.vin_mv > win_max_mv) {
            sense.faults |= FAULT_VIN;
        }
        if (!interlocks_ok() && !(probing && !sense.faults && !suspended)) {
            off();
        }
    }

    /* Current every 1 ms while on, after a 1 ms turn-on delay (the gate is
     * charged through R1). Over the limit: off at once and latch. */
    if (heater_on && phase >= HEATER_SETTLE_MS + 1) {
        uint16_t c = adc_read(ADC_CH_CURRENT);
        if (c > sense.cur_peak) {
            sense.cur_peak = c;
        }
        if (phase == HEATER_SETTLE_MS + 2) {
            sense.cur_raw = c;
            if (probing) {
                probe_raw = c;
            }
        }
        if (c > cur_hard_raw) {
            sense.faults |= FAULT_OVERCURRENT; /* short circuit */
            off();
        } else if (c > cur_soft_raw && !probing) {
            sense.cur_cut++; /* end this pulse; the loop backs off */
            off();
        }
    }

    if (heater_on && phase >= on_until) {
        off();
    }

    if (phase == HEATER_SETTLE_MS) {
        uint8_t on_ms = req_on_ms;

        off(); /* belt and braces: TEMP is only valid with the heater off */
        raw = avg(ADC_CH_TEMP, 8);
        sense.temp_raw = raw;
        if (raw >= TEMP_OPEN_RAW) {
            sense.faults |= FAULT_TC_OPEN;
        }
        if (permit && raw >= temp_max) {
            sense.faults |= FAULT_OVERTEMP;
        }
        sense.cur_raw = 0;
        sense.cur_peak = 0;
        sense.cycle++;

        if (probe_req) {
            probe_req = 0;
            /* Probe ignores the main-loop on time but not the interlocks.
             * The TC must be closed (handle plugged in). */
            if (HEATER_ENABLE && !suspended && raw < TEMP_OPEN_RAW && sense.vin_mv >= win_min_mv &&
                sense.vin_mv <= win_max_mv) {
                probing = 1;
                probe_raw = 0;
                on_until = HEATER_SETTLE_MS + HEATER_PROBE_MS + 1;
                heater_on = 1;
                heater_pin(1);
            } else {
                probe_raw = 0;
                probe_done = 1;
            }
        } else if (on_ms && interlocks_ok()) {
            if (on_ms > SENSE_MAX_ON_MS) {
                on_ms = SENSE_MAX_ON_MS;
            }
            on_until = HEATER_SETTLE_MS + on_ms;
            heater_on = 1;
            heater_pin(1);
        }
        sense.on_ms = heater_on ? (uint8_t)(on_until - HEATER_SETTLE_MS) : 0;
    }

    if (++phase >= HEATER_PERIOD_MS) {
        phase = 0;
        off();
    }
}

void sense_set_on_ms(uint8_t on_ms)
{
    req_on_ms = on_ms;
}

void sense_set_permit(int p, uint16_t vin_min_mv, uint16_t vin_max_mv, uint16_t temp_max_raw, uint8_t id_class)
{
    __disable_irq();
    win_min_mv = vin_min_mv;
    win_max_mv = vin_max_mv;
    temp_max = temp_max_raw;
    id_expect = id_class;
    permit = (uint8_t)p;
    __enable_irq();
}

void sense_set_current_limit(uint16_t max_ma)
{
    uint32_t hard = (uint32_t)max_ma * CUR_SHORT_PCT / 100;

    if (hard > CUR_HARD_MAX_MA) {
        hard = CUR_HARD_MAX_MA;
    }
    if (max_ma > hard) {
        max_ma = (uint16_t)hard;
    }
    cur_soft_raw = ma_to_raw(max_ma);
    cur_hard_raw = ma_to_raw(hard);
}

void sense_suspend(int on)
{
    __disable_irq();
    suspended = (uint8_t)on;
    if (on) {
        off();
        probing = 0;
    }
    __enable_irq();
}

void sense_clear_faults(void)
{
    __disable_irq();
    sense.faults = 0;
    __enable_irq();
}

void sense_probe_start(void)
{
    __disable_irq();
    probe_done = 0;
    probe_req = 1;
    __enable_irq();
}

int sense_probe_done(uint16_t *cur_raw)
{
    if (!probe_done) {
        return 0;
    }
    *cur_raw = probe_raw;
    return 1;
}
