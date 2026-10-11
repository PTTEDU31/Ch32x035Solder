#include "input.h"

#include <ch32x035.h>

#include "board.h"
#include "config.h"

#define LONG_MS 800
#define DOUBLE_MS 300
#define DEBOUNCE_MS 20

static volatile int16_t steps;
static volatile uint8_t key;
static volatile uint32_t activity;

static uint8_t ab_last;
static int8_t sub;
static uint8_t sw_cnt, sw_down, long_sent, clicks;
static uint16_t sw_ms, gap_ms;

/* Gray code transition table: index = old << 2 | new. */
static const int8_t qdec[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

static uint8_t read_ab(void)
{
    uint32_t in = GPIO_ReadInputData(ENC_PORT);
    return (uint8_t)(((in & ENC_A_PIN) ? 2 : 0) | ((in & ENC_B_PIN) ? 1 : 0));
}

void input_init(void)
{
    ab_last = read_ab();
}

void input_tick(void)
{
    uint8_t ab = read_ab();
    uint32_t now = millis();

    if (ab != ab_last) {
        sub += qdec[(ab_last << 2) | ab];
        ab_last = ab;
        if (sub >= ENC_DIV || sub <= -ENC_DIV) {
            int8_t d = sub > 0 ? 1 : -1;
            steps += ENC_REVERSE ? -d : d;
            sub = 0;
            activity = now;
        }
    }

    /* Push switch: active low, debounced. */
    if (GPIO_ReadInputDataBit(ENC_PORT, ENC_SW_PIN) == Bit_RESET) {
        if (sw_cnt < DEBOUNCE_MS) {
            sw_cnt++;
        }
    } else if (sw_cnt) {
        sw_cnt--;
    }

    if (!sw_down && sw_cnt == DEBOUNCE_MS) {
        sw_down = 1;
        sw_ms = 0;
        long_sent = 0;
        activity = now;
    } else if (sw_down && sw_cnt == 0) {
        sw_down = 0;
        if (!long_sent) {
            clicks++;
            gap_ms = 0;
        }
    }

    if (sw_down) {
        if (sw_ms < 0xFFFF) {
            sw_ms++;
        }
        if (!long_sent && sw_ms >= LONG_MS) {
            long_sent = 1;
            clicks = 0;
            key = KEY_LONG;
        }
    } else if (clicks) {
        if (++gap_ms >= DOUBLE_MS || clicks >= 2) {
            key = clicks >= 2 ? KEY_DOUBLE : KEY_SHORT;
            clicks = 0;
        }
    }
}

int16_t input_take_steps(void)
{
    int16_t s;

    __disable_irq();
    s = steps;
    steps = 0;
    __enable_irq();
    return s;
}

ui_key_t input_take_key(void)
{
    ui_key_t k;

    __disable_irq();
    k = (ui_key_t)key;
    key = KEY_NONE;
    __enable_irq();
    return k;
}

uint32_t input_last_activity(void)
{
    return activity;
}
