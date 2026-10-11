/*
 * Tip detection and temperature control (main loop side).
 *
 * The SysTick ISR (sense.c) samples and switches the heater; this file
 * decides what it is allowed to do:
 *  - tips are only detected at 5 V: on ID change or TC open the heater stops,
 *    5 V is requested, and detection starts again;
 *  - each tip asks PD for its own voltage and load resistance, and the
 *    heater only runs while V_IN matches the contract (+/- VIN_TOL_PCT) and
 *    stays under the tip's limit;
 *  - power is capped by the tip's max_w, recomputed from V_IN every cycle;
 *  - a tip that does not warm up under power latches HE_RUNAWAY.
 *
 * Control: PID on the measured temperature (derivative on measurement, so
 * setpoint steps do not kick) plus a feed-forward term: the duty needed to
 * hold a temperature is close to proportional to (T - ambient), and the
 * ratio is learned while holding. Gains were chosen with a thermal model
 * fitted to the bench runs (12 V / 150 C and 20 V / 320 C) and still need
 * tuning on the iron with a solder-joint load.
 *
 * Sleep: stage 1 at sleep_c, then deep_c, then off. Waking (lifting the
 * iron or turning the knob) allows the tip's boost voltage and power for a
 * few seconds and skips the soft start, since the source is already up.
 */
#include "heater.h"

#include <stddef.h>

#include "board.h"
#include "config.h"
#include "pd_sink.h"
#include "sense.h"

#define DETECT_STABLE_MS 300
/* On the stand (ignored during a FIRST_HEAT test session). */
#define STAND() (sense.stand && !FIRST_HEAT)
#define PD_SETTLE_MS 50
#define ERROR_RETRY_MS 1500
#define BOOST_MAX_C 100
#define WAKE_MAX_S 10          /* fast wake ends at the setpoint or after this */
#define WAKE_W_EXTRA 15        /* allowed above max_w while waking */

static heater_status_t st;
static uint32_t state_since;
static uint32_t last_cycle;
static int16_t target_c = 320;
static int16_t sleep_c = 150;
static int16_t boost_c;
static uint32_t boost_until;
static int32_t integ;          /* integrator, in 1/256 ms of on time */
static int16_t prev_temp;      /* for the derivative */
static int32_t rate_f;         /* filtered dT per cycle, C x 16 */
static uint32_t ff_k = 47;     /* hold duty per C of rise, x 1000 (learned) */
static uint8_t hold_cycles;
static uint32_t runaway_since;
static int16_t runaway_start_c;
static uint8_t probe_pending;
static uint32_t first_run_at;
static uint8_t last_out; /* on time of the previous cycle (soft start) */
static uint16_t auto_off_min = 10;
static uint32_t stand_since;
static uint16_t sleep1_s = 180;
static int16_t deep_c = 150;
static uint8_t fast_wake_en = 1;
static uint32_t wake_until;
static volatile uint8_t wake_req;

static void set_state(heater_state_t s)
{
    st.state = s;
    state_since = millis();
}

static void stop(void)
{
    last_out = 0;
    sense_set_on_ms(0);
    sense_set_permit(0, 0, 0, 0, 0);
    integ = 0;
    rate_f = 0;
    prev_temp = 0;
    hold_cycles = 0;
    runaway_since = 0;
    st.power_pct = 0;
    st.watts_x10 = 0;
}

static void fail(heater_err_t e)
{
    stop();
    st.err = e;
    pd_request(5000, 0);
    set_state(HS_ERROR);
}

static void go_no_tip(void)
{
    stop();
    st.tip = NULL;
    pd_request(5000, 0);
    sense_clear_faults();
    set_state(HS_NO_TIP);
}

/* Voltage wanted for this tip: its max_mv, but no more than needed for
 * max_w (P = V^2 / R -> V = sqrt(P * R)). */
static uint16_t tip_mv(const tip_t *t, int boost)
{
    uint32_t p_mw = (uint32_t)t->max_w * 1000;
    uint32_t v2 = p_mw * t->r_mohm / 1000; /* mV^2 / 1000 */
    uint32_t mv = 1;

    if (!HEATER_ENABLE) {
        /* Heater locked: never raise V_BUS. A gate drive that leaks (or a
         * shorted MOSFET) would otherwise heat the tip uncontrolled. */
        return 5000;
    }
    while (mv * mv / 1000 < v2 && mv < t->max_mv) {
        mv += 100;
    }
    /* Stay on the common 20 V level unless boosting: above 20 V needs EPR
     * (or a PPS that reaches 21 V), which not every charger has. */
    if (!boost && mv > 20000) {
        mv = 20000;
    }
    if (boost || mv > t->max_mv) {
        mv = t->max_mv;
    }
    if (FIRST_HEAT && mv > FIRST_HEAT_MAX_MV) {
        mv = FIRST_HEAT_MAX_MV;
    }
    return (uint16_t)mv;
}

/* Lowest V_IN the heater may run at (see HEATER_MIN_RUN_MV). While the
 * heater is locked the state machine still runs at 5 V for testing. */
#define RUN_MIN_MV (HEATER_ENABLE ? HEATER_MIN_RUN_MV : VIN_MIN_MV)

static int contract_ok(const tip_t *t)
{
    uint16_t c = pd_contract_mv();

    return pd_state() == PD_READY && c >= RUN_MIN_MV && c <= t->max_mv &&
           (millis() - pd_ready_since()) > PD_SETTLE_MS;
}

static void apply_permit(const tip_t *t, int run)
{
    uint16_t c = pd_contract_mv();
    uint32_t lo = (uint32_t)c * (100 - VIN_TOL_PCT) / 100;
    uint32_t hi = (uint32_t)c * (100 + VIN_TOL_PCT) / 100;

    if (hi > t->max_mv + t->max_mv / 20) {
        hi = t->max_mv + t->max_mv / 20;
    }
    if (lo < RUN_MIN_MV) {
        lo = RUN_MIN_MV;
    }
    sense_set_permit(run, (uint16_t)lo, (uint16_t)hi, tip_c_to_raw(t, TEMP_HARD_MAX_C), t->id_class);
    {
        /* Lowest of: tip limit, PD contract + 10 %, hard limit. */
        uint32_t lim = t->max_ma;
        uint32_t pd_ma = (uint32_t)pd_contract_ma() * 110 / 100;
        if (pd_ma && pd_ma < lim) {
            lim = pd_ma;
        }
        sense_set_current_limit((uint16_t)lim);
    }
}

static int16_t effective_target(void)
{
    int16_t t = target_c;

    if (st.state == HS_SLEEP) {
        t = st.deep_sleep ? deep_c : sleep_c;
    } else if (boost_until && (int32_t)(millis() - boost_until) < 0) {
        t = target_c + boost_c;
    } else {
        boost_until = 0;
    }
    if (t > st.tip->max_c) {
        t = st.tip->max_c;
    }
    if (FIRST_HEAT && t > FIRST_HEAT_MAX_C) {
        t = FIRST_HEAT_MAX_C;
    }
    return t;
}

/* One PID step per heater cycle. Output: on time in ms. */
static void control(void)
{
    const tip_t *t = st.tip;
    int16_t tgt = effective_target();
    int32_t err = tgt - st.temp_c;
    int32_t out, base, d;
    uint32_t vin = sense.vin_mv;
    uint32_t max_on = SENSE_MAX_ON_MS;
    uint32_t max_w = t->max_w + (wake_until ? WAKE_W_EXTRA : 0);
    int waking = wake_until != 0;

    /* Power cap: duty <= P_max * R / V^2. */
    if (vin > 1000) {
        uint32_t cap = max_w * t->r_mohm / 1000 * HEATER_PERIOD_MS * 1000 / (vin * vin / 1000);
        if (cap < max_on) {
            max_on = cap;
        }
    }

    /* Rate of change, C per cycle x16, low-pass filtered (noise ~0.5 C). */
    if (prev_temp) {
        rate_f += (((int32_t)(st.temp_c - prev_temp) << 4) - rate_f) / 3;
    }
    prev_temp = st.temp_c;

    /* Feed-forward: duty to hold tgt, from the learned ratio. In on time:
     * ff_k/1000 * (tgt - ambient) * full_power_duty, where full power is
     * V^2/R; ff_k is learned in "duty at 1 W/C" units scaled to this tip. */
    base = (int32_t)((uint64_t)ff_k * (uint32_t)(tgt > AMBIENT_C ? tgt - AMBIENT_C : 0) * t->r_mohm *
                     HEATER_PERIOD_MS / ((uint64_t)vin * vin / 1000) / 1000);

    /* Gains in ms of on time per 200 ms cycle:
     * Kp 3 ms/C (1.5 %/C), Ki 0.1 ms/C per cycle, Kd 4 ms per C/cycle. */
    d = rate_f * 4 / 16;
    out = base + err * 3 + integ / 256 - d;
    /* Conditional integration (anti-windup): only within +/-30 C, and not
     * further into a saturated output. Measured: while the tip cooled from
     * 331 C with the heater off, the integrator ran negative and then held
     * the tip 8 C low for 15 s. */
    if (err > -30 && err < 30 && !(out <= 0 && err < 0) && !(out >= (int32_t)max_on && err > 0)) {
        integ += err * 26;
    }
    /* The integrator only trims the feed-forward: never below -base. */
    if (integ < -base * 256) {
        integ = -base * 256;
    }
    if (integ > (int32_t)max_on * 128) {
        integ = (int32_t)max_on * 128;
    }
    out = base + err * 3 + integ / 256 - d;
    if (err > 30) {
        out = (int32_t)max_on; /* far below: full power */
    }
    if (out < 0) {
        out = 0;
    }
    if (out > (int32_t)max_on) {
        out = (int32_t)max_on;
    }
    /* Soft start only from a cold bus; waking keeps the contract up. */
    if (!waking && out > (int32_t)last_out + SOFTSTART_STEP_MS) {
        out = (int32_t)last_out + SOFTSTART_STEP_MS;
    }
    last_out = (uint8_t)out;
    sense_set_on_ms((uint8_t)out);

    /* Learn the hold ratio while settled: |err| < 3 C, slow drift. */
    if (err > -3 && err < 3 && rate_f > -8 && rate_f < 8 && tgt > AMBIENT_C + 50 && vin > 1000) {
        if (++hold_cycles > 25) {
            uint32_t k = (uint32_t)((uint64_t)out * ((uint64_t)vin * vin / 1000) * 1000 /
                                    ((uint32_t)(tgt - AMBIENT_C) * t->r_mohm * HEATER_PERIOD_MS));
            if (k > 5 && k < 300) {
                ff_k = (ff_k * 15 + k) / 16;
            }
            integ = integ * 15 / 16; /* the feed-forward takes over */
        }
    } else {
        hold_cycles = 0;
    }

    /* End fast wake at the setpoint. */
    if (waking && (err < 5 || (int32_t)(millis() - wake_until) >= 0)) {
        wake_until = 0;
    }

    st.target_c = tgt;
    st.power_pct = (uint8_t)(out * 100 / HEATER_PERIOD_MS);
    /* Power from the measured on-window current. */
    {
        uint32_t ma = CUR_MA(sense.cur_raw);
        if (!ma) {
            ma = vin / (t->r_mohm / 1000 ? t->r_mohm / 1000 : 1);
        }
        st.watts_x10 = (uint16_t)(vin * ma / 1000 * (uint32_t)out / HEATER_PERIOD_MS / 100);
    }

    /* Runaway: well below the setpoint and at >= half of the allowed power,
     * the tip must warm up. Near the setpoint half power only holds the
     * temperature, so the check would trip falsely there. */
    if (err > 30 && out * 2 >= (int32_t)max_on && max_on > 0 && sense.on_ms) {
        if (!runaway_since) {
            runaway_since = millis();
            runaway_start_c = st.temp_c;
        } else if (millis() - runaway_since > RUNAWAY_MS) {
            if (st.temp_c - runaway_start_c < RUNAWAY_MIN_RISE_C) {
                fail(HE_RUNAWAY);
                return;
            }
            runaway_since = 0;
        }
    } else {
        runaway_since = 0;
    }
}

void heater_init(void)
{
    go_no_tip();
}

void heater_poll(void)
{
    uint32_t now = millis();
    uint8_t faults = sense.faults;
    uint8_t id = sense.id_class;
    int tc_open = sense.temp_raw >= TEMP_OPEN_RAW;

    if (st.tip) {
        st.temp_c = tip_raw_to_c(st.tip, sense.temp_raw);
    }

    /* Faults from the ISR end any heating state. */
    if (st.state == HS_RUN || st.state == HS_SLEEP || st.state == HS_NEGOTIATE) {
        if (faults & FAULT_TC_OPEN) {
            go_no_tip();
            return;
        }
        if (faults & FAULT_ID) {
            fail(HE_ID_CHANGED);
            return;
        }
        if (faults & FAULT_OVERTEMP) {
            fail(HE_OVERTEMP);
            return;
        }
        if (faults & FAULT_OVERCURRENT) {
            fail(HE_OVERCURRENT);
            return;
        }
        if (faults & FAULT_VIN) {
            /* V_IN moved (PD renegotiation or source trouble): retry. */
            stop();
            sense_clear_faults();
            set_state(HS_NEGOTIATE);
            return;
        }
    }

    switch (st.state) {
    case HS_NO_TIP:
        /* The ID pin says which tip; the TC must also read (closed and not
         * saturated) before any power is applied. */
        if (!tc_open && id != ID_CLASS_UNKNOWN && tip_for_id(id)) {
            st.err = HE_NONE;
            set_state(HS_DETECT);
            probe_pending = 0;
        }
        break;

    case HS_DETECT: {
        const tip_t *t = tip_for_id(id);
        uint16_t cur;
        if (tc_open || !t) {
            go_no_tip();
            break;
        }
        /* Stable ID at 5 V before anything else. */
        if (now - state_since < DETECT_STABLE_MS || pd_contract_mv() > 5500 || pd_state() != PD_READY) {
            if (pd_state() == PD_IDLE && pd_contract_mv() == 0 && now - state_since > DETECT_STABLE_MS) {
                /* No PD source (or a plain 5 V port): the probe still runs at 5 V. */
            } else {
                break;
            }
        }
        if (!probe_pending) {
            /* Presence probe at 5 V: a heater draws current. */
            sense_set_permit(0, VIN_MIN_MV, 5800, 4095, t->id_class);
            sense_probe_start();
            probe_pending = 1;
            break;
        }
        if (!sense_probe_done(&cur)) {
            break;
        }
        probe_pending = 0;
        if (!HEATER_ENABLE || cur >= CUR_PRESENT_RAW) {
            st.tip = t;
            pd_request(tip_mv(t, 0), t->r_mohm);
            set_state(HS_NEGOTIATE);
        } else {
            /* TC closed but no heater current: try again later. */
            set_state(HS_DETECT);
        }
        break;
    }

    case HS_NEGOTIATE:
        if (id != st.tip->id_class) {
            go_no_tip();
            break;
        }
        pd_request(tip_mv(st.tip, boost_until != 0 || wake_until != 0), st.tip->r_mohm);
        if (contract_ok(st.tip)) {
            sense_clear_faults();
            apply_permit(st.tip, 1);
            set_state(STAND() ? HS_SLEEP : HS_RUN);
        } else if (now - state_since > 5000 && pd_state() != PD_NEGOTIATING) {
            fail(HE_NO_POWER);
        }
        break;

    case HS_RUN:
    case HS_SLEEP:
        if (FIRST_HEAT) {
            /* One session per power-up: never restarts after the timeout. */
            if (!first_run_at) {
                first_run_at = now | 1;
            } else if (now - first_run_at > FIRST_HEAT_TIMEOUT_S * 1000UL) {
                fail(HE_TEST_DONE);
                break;
            }
        }
        if (!contract_ok(st.tip)) {
            /* Bus is moving (new contract) or gone: hold off. */
            stop();
            set_state(HS_NEGOTIATE);
            break;
        }
        apply_permit(st.tip, 1);
        pd_request(tip_mv(st.tip, boost_until != 0 || wake_until != 0), st.tip->r_mohm);
        if (STAND() && st.state == HS_RUN && !wake_req) {
            set_state(HS_SLEEP);
            stand_since = now;
            st.deep_sleep = 0;
            wake_until = 0;
        } else if ((!STAND() || wake_req) && st.state == HS_SLEEP) {
            if (fast_wake_en) {
                wake_until = (now + WAKE_MAX_S * 1000UL) | 1;
            }
            if (wake_req) {
                stand_since = now; /* knob wake on the stand restarts sleep */
            }
            set_state(HS_RUN);
        } else if (st.state == HS_SLEEP && !st.deep_sleep && now - stand_since > (uint32_t)sleep1_s * 1000UL) {
            st.deep_sleep = 1;
        } else if (st.state == HS_SLEEP && auto_off_min &&
                   now - stand_since > (uint32_t)auto_off_min * 60000UL) {
            /* Left on the stand: switch off, back to 5 V. */
            stop();
            pd_request(5000, 0);
            set_state(HS_OFF);
            break;
        }
        if (wake_req && st.state == HS_RUN && now - state_since > 60000UL) {
            wake_req = 0; /* knob wake on the stand lasts one minute */
        }
        if (!STAND()) {
            wake_req = 0;
        }
        if (sense.cycle != last_cycle) {
            last_cycle = sense.cycle;
            control();
        }
        st.sleep_left_s = 0;
        if (st.state == HS_SLEEP && auto_off_min) {
            uint32_t end = stand_since + (uint32_t)auto_off_min * 60000UL;
            st.sleep_left_s = (uint16_t)((int32_t)(end - now) > 0 ? (end - now) / 1000 : 0);
        }
        break;

    case HS_OFF:
        /* Lifting the iron, the knob, or a different tip starts over. */
        if (!STAND() || wake_req || id != st.tip->id_class) {
            go_no_tip();
        }
        break;

    case HS_ERROR:
        /* Leaves on tip removal (or a different tip); temporary errors retry. */
        if ((tc_open && id == ID_CLASS_OPEN) || (st.tip && id != st.tip->id_class && id != ID_CLASS_UNKNOWN)) {
            go_no_tip();
        } else if ((st.err == HE_NO_POWER || st.err == HE_VIN) && now - state_since > ERROR_RETRY_MS) {
            go_no_tip();
        }
        break;
    }
}

void heater_set_target(int16_t c)
{
    target_c = c;
}

void heater_set_sleep_target(int16_t c)
{
    sleep_c = c;
}

void heater_set_auto_off(uint16_t minutes)
{
    auto_off_min = minutes;
}

void heater_set_sleep_stages(uint16_t stage1_s, int16_t c)
{
    sleep1_s = stage1_s;
    deep_c = c;
}

void heater_set_fast_wake(uint8_t enable)
{
    fast_wake_en = enable;
}

void heater_wake(void)
{
    if (st.state == HS_SLEEP || st.state == HS_OFF) {
        wake_req = 1;
    }
}

void heater_boost(int16_t c, uint16_t s)
{
    if (c > BOOST_MAX_C) {
        c = BOOST_MAX_C;
    }
    boost_c = c;
    boost_until = s ? millis() + (uint32_t)s * 1000 : 0;
}

const heater_status_t *heater_status(void)
{
    st.boost = boost_until != 0;
    st.waking = wake_until != 0;
    st.boost_left_s = boost_until ? (uint16_t)((int32_t)(boost_until - millis()) > 0 ? (boost_until - millis()) / 1000 : 0)
                                  : 0;
    if (!st.tip) {
        st.temp_c = 0;
        st.target_c = target_c;
    }
    return &st;
}

const char *heater_state_name(heater_state_t s)
{
    static const char *const n[] = {"NO TIP", "DETECT", "PD", "RUN", "SLEEP", "OFF", "ERROR"};
    return n[s];
}

const char *heater_err_name(heater_err_t e)
{
    static const char *const n[] = {"", "TC OPEN", "TIP CHANGED", "V_IN", "OVERTEMP", "NO HEAT", "NO POWER", "TEST DONE",
                                    "OVERCURRENT"};
    return n[e];
}
