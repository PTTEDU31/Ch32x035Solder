/*
 * Settings in two 256 B flash pages, written alternately with a sequence
 * number and CRC, so a power loss mid-write keeps the older copy.
 *
 * Fast erase/program stalls the CPU (no interrupts, no GoodCRC to PD
 * messages) for a few ms: writes happen only when settings have been idle,
 * the heater is suspended and PD has just finished an exchange.
 */
#include "settings.h"

#include <ch32x035.h>
#include <stddef.h>
#include <string.h>

#include "board.h"
#include "config.h"
#include "pd_sink.h"
#include "sense.h"

#define MAGIC 0x53543033UL /* "ST03" */
#define SETTINGS_IDLE_MS 3000
#define PD_QUIET_WINDOW_MS 150 /* write within this time after a PD message */

typedef struct {
    uint32_t magic;
    uint32_t seq;
    settings_t s;
    uint32_t crc;
} record_t;

settings_t settings;

static uint32_t seq;
static uint8_t dirty;
static uint32_t touched;
static uint32_t page_buf[64];

static const settings_t defaults = {
    .target_c = 320,
    .boost_c = 50,
    .boost_s = 30,
    .off_min = 10,
    .backlight = 80,
    .beep = 1,
    .fahrenheit = 0,
    .ui_style = 0,
    .fast_wake = 1,
    .sleep1_min = 3,
    .sleep_c = 200,
    .deep_c = 150,
    .off_screen = 1,
};

static uint32_t crc32(const void *data, uint32_t len)
{
    const uint8_t *p = data;
    uint32_t c = 0xFFFFFFFFUL;

    while (len--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) {
            c = (c >> 1) ^ (0xEDB88320UL & -(c & 1));
        }
    }
    return ~c;
}

static const record_t *valid(uint32_t addr)
{
    const record_t *r = (const record_t *)addr;

    if (r->magic != MAGIC || r->crc != crc32(r, offsetof(record_t, crc))) {
        return NULL;
    }
    return r;
}

static void sanitize(settings_t *s)
{
    if (s->target_c < 100 || s->target_c > 450) {
        s->target_c = defaults.target_c;
    }
    if (s->sleep_c < 100 || s->sleep_c > 300) {
        s->sleep_c = defaults.sleep_c;
    }
    if (s->boost_c < 0 || s->boost_c > 100) {
        s->boost_c = defaults.boost_c;
    }
    if (s->boost_s < 5 || s->boost_s > 120) {
        s->boost_s = defaults.boost_s;
    }
    if (s->off_min > 60) {
        s->off_min = defaults.off_min;
    }
    if (s->backlight < 5 || s->backlight > 100) {
        s->backlight = defaults.backlight;
    }
    s->beep = !!s->beep;
    s->fahrenheit = !!s->fahrenheit;
    s->ui_style = s->ui_style > 1 ? 0 : s->ui_style;
    s->fast_wake = !!s->fast_wake;
    if (s->sleep1_min > 30) {
        s->sleep1_min = defaults.sleep1_min;
    }
    if (s->deep_c < 100 || s->deep_c > 300) {
        s->deep_c = defaults.deep_c;
    }
    if (s->cal_t12 < -50 || s->cal_t12 > 50) {
        s->cal_t12 = 0;
    }
    if (s->cal_c210 < -50 || s->cal_c210 > 50) {
        s->cal_c210 = 0;
    }
    if (s->off_screen > 3) {
        s->off_screen = 1;
    }
}

void settings_load(void)
{
    const record_t *a = valid(SETTINGS_ADDR_A);
    const record_t *b = valid(SETTINGS_ADDR_B);
    const record_t *r = a;

    if (b && (!a || (int32_t)(b->seq - a->seq) > 0)) {
        r = b;
    }
    if (r) {
        settings = r->s;
        seq = r->seq;
    } else {
        settings = defaults;
        seq = 0;
    }
    sanitize(&settings);
}

void settings_touch(void)
{
    dirty = 1;
    touched = millis();
}

static void write_now(void)
{
    record_t *r = (record_t *)page_buf;
    uint32_t addr = (seq & 1) ? SETTINGS_ADDR_A : SETTINGS_ADDR_B;

    memset(page_buf, 0xFF, sizeof(page_buf));
    r->magic = MAGIC;
    r->seq = seq + 1;
    r->s = settings;
    r->crc = crc32(r, offsetof(record_t, crc));

    sense_suspend(1);
    heater_force_off();
    __disable_irq();
    FLASH_ROM_ERASE(addr, 256);
    FLASH_ROM_WRITE(addr, page_buf, 256);
    __enable_irq();
    sense_suspend(0);

    if (valid(addr)) {
        seq++;
    }
}

void settings_poll(void)
{
    uint32_t now = millis();

    if (!dirty || now - touched < SETTINGS_IDLE_MS) {
        return;
    }
    /* Right after a PD exchange the source is not expecting anything for a
     * while (or there is no PD source at all). */
    if (pd_state() != PD_IDLE && now - pd_last_rx() > PD_QUIET_WINDOW_MS) {
        if (now - touched < SETTINGS_IDLE_MS + 15000) {
            return;
        }
    }
    dirty = 0;
    write_now();
}
