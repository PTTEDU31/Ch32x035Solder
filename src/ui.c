/*
 * User interface, ST7789 240x320 portrait. Two main screen styles:
 *
 *  GAUGE: tip / PD chips, a 270 degree ring coloured blue -> amber -> red
 *         with a setpoint tick, big anti-aliased temperature and state word,
 *         SET card, power bar with %, W and V_IN.
 *  GRAPH: same chips, big temperature on the left with the setpoint and
 *         state, a 60 s temperature history (with the setpoint line), and
 *         the power bar.
 *
 * Drawing is incremental (no framebuffer): every element keeps what it last
 * drew and only changed parts are sent; the ring only redraws the segments
 * whose colour changes. Long drawing yields to PD through lcd_init's hook.
 *
 * Knob: turn = target +/-5 C (wakes from sleep), click = boost, long press
 * = menu. In the menu: turn = change, click = next item, double click or
 * long press = exit.
 */
#include "ui.h"

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "config.h"
#include "fonts.h"
#include "heater.h"
#include "input.h"
#include "lcd.h"
#include "out.h"
#include "pd_sink.h"
#include "sense.h"
#include "settings.h"
#include "tips.h"

/* ---- palette ------------------------------------------------------------ */
#define C_BG RGB(10, 12, 16)
#define C_CARD RGB(24, 28, 36)
#define C_TRACK RGB(36, 42, 52)
#define C_FG RGB(236, 240, 245)
#define C_DIM RGB(110, 118, 132)
#define C_FAINT RGB(70, 76, 88)
#define C_BLUE RGB(80, 160, 255)
#define C_AMBER RGB(255, 170, 40)
#define C_RED RGB(255, 80, 56)
#define C_GREEN RGB(70, 220, 120)
#define C_PD RGB(120, 190, 255)

/* ---- layout ------------------------------------------------------------- */
#define CHIP_Y 8
#define CHIP_H 22
#define RING_CX 120
#define RING_CY 132
#define RING_R 88
#define RING_W 10
#define RING_SEGS 54 /* 5 degrees each over 270 degrees */
#define SET_Y 236
#define SET_H 32
#define BAR_Y 278
#define BAR_H 10
#define FOOT_Y 294

#define GRAPH_X 8
#define GRAPH_Y 128
#define GRAPH_W 224
#define GRAPH_H 92
#define GRAPH_PTS 112 /* one point per 2 px, sampled every 0.5 s: 56 s */

typedef enum {
    MENU_OFF,
    MENU_STYLE,
    MENU_SLEEP1,
    MENU_SLEEP1_MIN,
    MENU_DEEP,
    MENU_OFF_MIN,
    MENU_FAST_WAKE,
    MENU_BOOST_C,
    MENU_BOOST_S,
    MENU_CAL,
    MENU_BL,
    MENU_BEEP,
    MENU_UNIT,
    MENU_COUNT
} menu_t;

static menu_t menu;
static uint8_t full_redraw = 1;
static uint32_t last_draw, last_graph;

/* What is on screen now. */
static struct {
    char chip_tip[12];
    char chip_pd[24];
    char big[8];
    uint16_t big_fg;
    char word[12];
    uint16_t word_fg;
    char set[24];
    char extra[16];
    char foot[32];
    uint16_t bar_px;
    uint16_t bar_color;
    uint16_t seg[RING_SEGS];
    int16_t tick_seg;
    char m_name[24];
    char m_val[24];
} shown;

static uint8_t graph_t[GRAPH_PTS]; /* temperature / 2, 0 = no data */
static uint8_t graph_n;

/* ---- helpers ------------------------------------------------------------ */
static int16_t to_unit(int16_t c)
{
    return settings.fahrenheit ? (int16_t)(c * 9 / 5 + 32) : c;
}

static const char *unit(void)
{
    return settings.fahrenheit ? FONT_MID_DEG "F" : FONT_MID_DEG "C";
}

/* Blue -> amber -> red over 0..255. */
static uint16_t heat_color(uint16_t t)
{
    if (t > 255) {
        t = 255;
    }
    if (t < 128) {
        return lcd_mix(C_AMBER, C_BLUE, (uint8_t)(t * 15 / 127));
    }
    return lcd_mix(C_RED, C_AMBER, (uint8_t)((t - 128) * 15 / 127));
}

static uint16_t temp_frac(int16_t c)
{
    int32_t f = (int32_t)(c - 25) * 255 / (450 - 25);
    return (uint16_t)(f < 0 ? 0 : f > 255 ? 255 : f);
}

/* Text in a fixed box, right/left/centre aligned, only if changed. */
enum { AL_LEFT, AL_CENTER, AL_RIGHT };

static void mid_box(char *cache, size_t cache_len, const char *s, uint16_t x, uint16_t y, uint16_t w, uint8_t align,
                    uint16_t fg, uint16_t bg)
{
    uint16_t tw, tx;

    if (!full_redraw && strcmp(cache, s) == 0) {
        return;
    }
    tw = lcd_mid_width(s);
    if (tw > w) {
        tw = w;
    }
    tx = align == AL_LEFT ? x : align == AL_CENTER ? (uint16_t)(x + (w - tw) / 2) : (uint16_t)(x + w - tw);
    lcd_fill(x, y, tx - x, FONT_MID_H, bg);
    lcd_mid(tx, y, s, fg, bg);
    lcd_fill(tx + tw, y, x + w - tx - tw, FONT_MID_H, bg);
    strncpy(cache, s, cache_len - 1);
    cache[cache_len - 1] = 0;
}

static void small_box(char *cache, size_t cache_len, const char *s, uint16_t x, uint16_t y, uint16_t w, uint8_t align,
                      uint16_t fg, uint16_t bg)
{
    uint16_t tw = (uint16_t)strlen(s) * FONT_SMALL_W, tx;

    if (!full_redraw && strcmp(cache, s) == 0) {
        return;
    }
    if (tw > w) {
        tw = w;
    }
    tx = align == AL_LEFT ? x : align == AL_CENTER ? (uint16_t)(x + (w - tw) / 2) : (uint16_t)(x + w - tw);
    lcd_fill(x, y, tx - x, FONT_SMALL_H, bg);
    lcd_text(tx, y, s, fg, bg, 1);
    lcd_fill(tx + tw, y, x + w - tx - tw, FONT_SMALL_H, bg);
    strncpy(cache, s, cache_len - 1);
    cache[cache_len - 1] = 0;
}

/* ---- ring --------------------------------------------------------------- */
/* Integer sin/cos in 1/256, by table over 0..90 degrees in 5 degree steps. */
static const uint8_t sin_tab[19] = {0, 22, 44, 66, 88, 108, 128, 147, 165, 181, 196, 210, 222, 232, 241, 247, 252, 255, 255};

static int16_t isin(int16_t deg)
{
    deg %= 360;
    if (deg < 0) {
        deg += 360;
    }
    if (deg <= 90) {
        return sin_tab[deg / 5];
    }
    if (deg <= 180) {
        return sin_tab[(180 - deg) / 5];
    }
    if (deg <= 270) {
        return -sin_tab[(deg - 180) / 5];
    }
    return -sin_tab[(360 - deg) / 5];
}

static int16_t icos(int16_t deg)
{
    return isin(deg + 90);
}

/* Segment i covers 135 + 5i .. 140 + 5i degrees (screen y points down). Each
 * is drawn as short radial strokes at 1 degree steps. */
static void ring_segment(uint8_t i, uint16_t color)
{
    for (int16_t a = 135 + i * 5; a < 140 + i * 5; a++) {
        /* Finer angle: interpolate the 5 degree table. */
        int16_t a0 = a - a % 5, f = a % 5;
        int32_t c = (icos(a0) * (5 - f) + icos(a0 + 5) * f) / 5;
        int32_t s = (isin(a0) * (5 - f) + isin(a0 + 5) * f) / 5;
        for (int16_t r = RING_R - RING_W / 2; r <= RING_R + RING_W / 2; r++) {
            int16_t x = (int16_t)(RING_CX + c * r / 256);
            int16_t y = (int16_t)(RING_CY + s * r / 256);
            lcd_pixel((uint16_t)x, (uint16_t)y, color);
            lcd_pixel((uint16_t)x + 1, (uint16_t)y, color);
        }
        lcd_yield();
    }
}

static void ring_tick(int16_t seg, uint16_t color)
{
    int16_t a = 135 + seg * 5 + 2;
    int16_t a0 = a - a % 5, f = a % 5;
    int32_t c = (icos(a0) * (5 - f) + icos(a0 + 5) * f) / 5;
    int32_t s = (isin(a0) * (5 - f) + isin(a0 + 5) * f) / 5;

    for (int16_t r = RING_R - RING_W / 2 - 8; r <= RING_R + RING_W / 2 + 6; r++) {
        int16_t x = (int16_t)(RING_CX + c * r / 256);
        int16_t y = (int16_t)(RING_CY + s * r / 256);
        lcd_pixel((uint16_t)x, (uint16_t)y, color);
        lcd_pixel((uint16_t)x + 1, (uint16_t)y, color);
        lcd_pixel((uint16_t)x, (uint16_t)y + 1, color);
    }
}

static void draw_ring(int16_t temp_c, int16_t set_c, uint16_t solid)
{
    uint16_t lit = (uint16_t)(temp_frac(temp_c) * RING_SEGS / 255);
    int16_t tick = (int16_t)(temp_frac(set_c) * (RING_SEGS - 1) / 255);

    for (uint8_t i = 0; i < RING_SEGS; i++) {
        uint16_t c = i < lit ? (solid ? solid : heat_color((uint16_t)(i * 255 / (RING_SEGS - 1)))) : C_TRACK;
        if (full_redraw || shown.seg[i] != c) {
            ring_segment(i, c);
            shown.seg[i] = c;
            lcd_yield();
        }
    }
    if (full_redraw || shown.tick_seg != tick) {
        if (shown.tick_seg >= 0 && !full_redraw) {
            /* Erase the old tick by redrawing its ring segment and the
             * background just outside it. */
            ring_tick(shown.tick_seg, C_BG);
            ring_segment((uint8_t)shown.tick_seg, shown.seg[shown.tick_seg]);
        }
        ring_tick(tick, C_FG);
        shown.tick_seg = tick;
    }
}

/* ---- graph -------------------------------------------------------------- */
static void graph_push(int16_t c)
{
    uint8_t v = (uint8_t)(c < 0 ? 0 : c > 508 ? 254 : c / 2);

    if (v == 0) {
        v = 1;
    }
    if (graph_n < GRAPH_PTS) {
        graph_t[graph_n++] = v;
    } else {
        memmove(graph_t, graph_t + 1, GRAPH_PTS - 1);
        graph_t[GRAPH_PTS - 1] = v;
    }
}

static void draw_graph(int16_t set_c)
{
    /* Scale: 50 .. 450 C onto the graph height. */
    uint16_t set_y = (uint16_t)(GRAPH_Y + GRAPH_H - 1 - (uint32_t)(set_c > 50 ? set_c - 50 : 0) * (GRAPH_H - 1) / 400);

    lcd_round_rect(GRAPH_X, GRAPH_Y, GRAPH_W, GRAPH_H, 8, C_CARD, C_BG);
    for (uint8_t i = 0; i < graph_n; i++) {
        int16_t c = (int16_t)graph_t[i] * 2;
        uint16_t h = (uint16_t)((uint32_t)(c > 50 ? c - 50 : 0) * (GRAPH_H - 1) / 400);
        uint16_t x = (uint16_t)(GRAPH_X + 4 + i * 2);
        uint16_t col = heat_color(temp_frac(c));
        if (x + 1 >= GRAPH_X + GRAPH_W - 4) {
            break;
        }
        if (h > GRAPH_H - 6) {
            h = GRAPH_H - 6;
        }
        /* Filled area, faded, with a bright top line. */
        lcd_vline(x, GRAPH_Y + GRAPH_H - 3 - h, h, lcd_mix(col, C_CARD, 5));
        lcd_vline(x + 1, GRAPH_Y + GRAPH_H - 3 - h, h, lcd_mix(col, C_CARD, 5));
        lcd_fill(x, GRAPH_Y + GRAPH_H - 4 - h, 2, 2, col);
    }
    /* Setpoint line, dashed. */
    for (uint16_t x = GRAPH_X + 4; x < GRAPH_X + GRAPH_W - 4; x += 6) {
        lcd_fill(x, set_y, 3, 1, C_FG);
    }
}

/* ---- main screen -------------------------------------------------------- */
static const char *kind_str(pd_kind_t k)
{
    static const char *const n[] = {"", "", " PPS", " AVS"};
    return n[k];
}

static void draw_chips(const heater_status_t *h)
{
    char buf[24];

    if (full_redraw) {
        lcd_round_rect(8, CHIP_Y, 64, CHIP_H, 10, C_CARD, C_BG);
        lcd_round_rect(78, CHIP_Y, 154, CHIP_H, 10, C_CARD, C_BG);
    }
    small_box(shown.chip_tip, sizeof(shown.chip_tip), h->tip ? h->tip->name : "--", 14, CHIP_Y + 3, 52, AL_CENTER,
              C_FG, C_CARD);
    if (pd_contract_mv()) {
        snprintf(buf, sizeof(buf), "%u.%uV %u.%uA%s%s", pd_contract_mv() / 1000, pd_contract_mv() % 1000 / 100,
                 pd_contract_ma() / 1000, pd_contract_ma() % 1000 / 100, kind_str(pd_contract_kind()),
                 pd_epr_active() ? " EPR" : "");
    } else {
        snprintf(buf, sizeof(buf), "%s", pd_state() == PD_NEGOTIATING ? "PD ..." : "no PD");
    }
    small_box(shown.chip_pd, sizeof(shown.chip_pd), buf, 84, CHIP_Y + 3, 142, AL_CENTER, C_PD, C_CARD);
}

/* State word and colours. */
static void state_look(const heater_status_t *h, const char **word, uint16_t *fg, uint16_t *ring_solid)
{
    int16_t diff = h->temp_c - h->target_c;

    *ring_solid = 0;
    switch (h->state) {
    case HS_RUN:
        if (h->boost) {
            *word = "BOOST";
            *fg = C_RED;
        } else if (h->waking) {
            *word = "WAKE";
            *fg = C_AMBER;
        } else if (diff > -6 && diff < 6) {
            *word = "READY";
            *fg = C_GREEN;
        } else if (diff <= -6) {
            *word = "HEATING";
            *fg = C_AMBER;
        } else {
            *word = "COOLING";
            *fg = C_BLUE;
        }
        break;
    case HS_SLEEP:
        *word = h->deep_sleep ? "DEEP SLEEP" : "SLEEP";
        *fg = C_BLUE;
        *ring_solid = C_BLUE;
        break;
    case HS_OFF:
        *word = "OFF";
        *fg = C_DIM;
        *ring_solid = C_DIM;
        break;
    case HS_ERROR:
        *word = heater_err_name(h->err);
        *fg = C_RED;
        break;
    case HS_NO_TIP:
        *word = (sense.id_class != ID_CLASS_UNKNOWN && sense.temp_raw >= TEMP_OPEN_RAW) ? "TC HIGH" : "NO TIP";
        *fg = C_DIM;
        break;
    default:
        *word = heater_state_name(h->state);
        *fg = C_DIM;
        break;
    }
}

static void draw_big_temp(const heater_status_t *h, uint16_t x, uint16_t y, uint16_t fg)
{
    char buf[8];
    int show = h->tip && h->state != HS_NO_TIP && h->state != HS_DETECT;

    if (show) {
        int16_t t = to_unit(h->temp_c);
        snprintf(buf, sizeof(buf), "%3d", t < 0 ? 0 : t > 999 ? 999 : t);
    } else {
        snprintf(buf, sizeof(buf), "---");
    }
    if (!full_redraw && strcmp(shown.big, buf) == 0 && shown.big_fg == fg) {
        return;
    }
    lcd_big(x, y, buf, fg, C_BG);
    strcpy(shown.big, buf);
    shown.big_fg = fg;
}

static void draw_set_card(const heater_status_t *h, uint16_t y)
{
    char buf[24], extra[16];
    int16_t set = h->state == HS_SLEEP ? h->target_c : (h->boost ? h->target_c : settings.target_c);

    if (full_redraw) {
        lcd_round_rect(8, y, 224, SET_H, 12, C_CARD, C_BG);
        lcd_text(20, y + 8, "SET", C_DIM, C_CARD, 1);
    }
    snprintf(buf, sizeof(buf), "%d%s", to_unit(set), unit());
    mid_box(shown.set, sizeof(shown.set), buf, 60, y + 5, 100, AL_CENTER, C_FG, C_CARD);

    extra[0] = 0;
    if (h->boost) {
        snprintf(extra, sizeof(extra), "+%d %us", settings.boost_c, h->boost_left_s);
    } else if (h->state == HS_SLEEP && h->sleep_left_s) {
        snprintf(extra, sizeof(extra), "off %um", (h->sleep_left_s + 59) / 60);
    }
    small_box(shown.extra, sizeof(shown.extra), extra, 160, y + 8, 64, AL_RIGHT,
              h->boost ? C_RED : C_DIM, C_CARD);
}

static void draw_power(const heater_status_t *h)
{
    char buf[32];
    uint16_t vin = (uint16_t)vin_mv();
    uint16_t w = (uint16_t)(224UL * h->power_pct / 100);
    uint16_t col = heat_color((uint16_t)(h->power_pct * 255 / 100));

    if (full_redraw || w != shown.bar_px || col != shown.bar_color) {
        lcd_round_rect(8, BAR_Y, 224, BAR_H, 5, C_TRACK, C_BG);
        if (w >= 10) {
            lcd_round_rect(8, BAR_Y, w, BAR_H, 5, col, C_TRACK);
        }
        shown.bar_px = w;
        shown.bar_color = col;
    }
    snprintf(buf, sizeof(buf), "%3u%%   %3u.%uW   %2u.%uV", h->power_pct, h->watts_x10 / 10, h->watts_x10 % 10,
             vin / 1000, vin % 1000 / 100);
    small_box(shown.foot, sizeof(shown.foot), buf, 8, FOOT_Y, 224, AL_CENTER, C_FG, C_BG);
}

static void draw_gauge(const heater_status_t *h)
{
    const char *word;
    uint16_t fg, solid;
    int16_t ring_t = (h->tip && h->state != HS_NO_TIP && h->state != HS_DETECT) ? h->temp_c : 0;

    state_look(h, &word, &fg, &solid);
    draw_chips(h);
    draw_ring(ring_t, h->state == HS_SLEEP ? h->target_c : settings.target_c, solid);
    /* Big digits: white when ready, otherwise the state colour. */
    draw_big_temp(h, RING_CX - 3 * FONT_BIG_W / 2, RING_CY - 40, fg == C_GREEN ? C_FG : fg);
    mid_box(shown.word, sizeof(shown.word), word, RING_CX - 60, RING_CY + 26, 120, AL_CENTER, fg, C_BG);
    draw_set_card(h, SET_Y);
    draw_power(h);
}

static void draw_graph_screen(const heater_status_t *h)
{
    const char *word;
    uint16_t fg, solid;

    state_look(h, &word, &fg, &solid);
    draw_chips(h);
    draw_big_temp(h, 8, 44, fg == C_GREEN ? C_FG : fg);
    if (full_redraw) {
        lcd_mid(160, 46, unit(), C_DIM, C_BG);
    }
    mid_box(shown.word, sizeof(shown.word), word, 160, 76, 76, AL_LEFT, fg, C_BG);
    if (full_redraw || millis() - last_graph >= 500) {
        last_graph = millis();
        if (h->tip && h->state != HS_NO_TIP && h->state != HS_DETECT) {
            graph_push(h->temp_c);
        }
        draw_graph(h->state == HS_SLEEP ? h->target_c : settings.target_c);
    }
    draw_set_card(h, SET_Y);
    draw_power(h);
}

/* ---- menu --------------------------------------------------------------- */
static const char *menu_name(menu_t m)
{
    static const char *const n[] = {"",           "Screen",    "Sleep temp", "Sleep time", "Deep sleep",
                                    "Auto off",   "Fast wake", "Boost +",    "Boost time", "Tip offset",
                                    "Backlight",  "Beep",      "Unit"};
    return n[m];
}

static int8_t *cal_ptr(void)
{
    const heater_status_t *h = heater_status();
    if (h->tip && h->tip->id_class == ID_CLASS_OPEN) {
        return &settings.cal_c210;
    }
    return &settings.cal_t12;
}

static void menu_value(menu_t m, char *buf, size_t len)
{
    switch (m) {
    case MENU_STYLE:
        snprintf(buf, len, "%s", settings.ui_style ? "Graph" : "Gauge");
        break;
    case MENU_SLEEP1:
        snprintf(buf, len, "%d%s", to_unit(settings.sleep_c), unit());
        break;
    case MENU_SLEEP1_MIN:
        snprintf(buf, len, "%u min", settings.sleep1_min);
        break;
    case MENU_DEEP:
        snprintf(buf, len, "%d%s", to_unit(settings.deep_c), unit());
        break;
    case MENU_OFF_MIN:
        if (settings.off_min) {
            snprintf(buf, len, "%u min", settings.off_min);
        } else {
            snprintf(buf, len, "never");
        }
        break;
    case MENU_FAST_WAKE:
        snprintf(buf, len, "%s", settings.fast_wake ? "on" : "off");
        break;
    case MENU_BOOST_C:
        snprintf(buf, len, "+%d%s", settings.boost_c, FONT_MID_DEG "C");
        break;
    case MENU_BOOST_S:
        snprintf(buf, len, "%u s", settings.boost_s);
        break;
    case MENU_CAL: {
        const heater_status_t *h = heater_status();
        snprintf(buf, len, "%s %+d%s", h->tip && h->tip->id_class == ID_CLASS_OPEN ? "C210" : "T12", *cal_ptr(),
                 FONT_MID_DEG "C");
        break;
    }
    case MENU_BL:
        snprintf(buf, len, "%u %%", settings.backlight);
        break;
    case MENU_BEEP:
        snprintf(buf, len, "%s", settings.beep ? "on" : "off");
        break;
    case MENU_UNIT:
        snprintf(buf, len, "%s", settings.fahrenheit ? FONT_MID_DEG "F" : FONT_MID_DEG "C");
        break;
    default:
        buf[0] = 0;
        break;
    }
}

static int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static void apply_settings(void)
{
    heater_set_sleep_target(settings.sleep_c);
    heater_set_sleep_stages((uint16_t)settings.sleep1_min * 60, settings.deep_c);
    heater_set_auto_off(settings.off_min);
    heater_set_fast_wake(settings.fast_wake);
    tip_set_offset(tip_for_id(ID_CLASS_GND), settings.cal_t12);
    tip_set_offset(tip_for_id(ID_CLASS_OPEN), settings.cal_c210);
}

static void menu_adjust(menu_t m, int16_t d)
{
    switch (m) {
    case MENU_STYLE:
        settings.ui_style = !settings.ui_style;
        break;
    case MENU_SLEEP1:
        settings.sleep_c = (int16_t)clampi(settings.sleep_c + d * 10, 100, 300);
        break;
    case MENU_SLEEP1_MIN:
        settings.sleep1_min = (uint8_t)clampi(settings.sleep1_min + d, 0, 30);
        break;
    case MENU_DEEP:
        settings.deep_c = (int16_t)clampi(settings.deep_c + d * 10, 100, 300);
        break;
    case MENU_OFF_MIN:
        settings.off_min = (uint16_t)clampi(settings.off_min + d, 0, 60);
        break;
    case MENU_FAST_WAKE:
        settings.fast_wake = !settings.fast_wake;
        break;
    case MENU_BOOST_C:
        settings.boost_c = (int16_t)clampi(settings.boost_c + d * 10, 10, 100);
        break;
    case MENU_BOOST_S:
        settings.boost_s = (uint16_t)clampi(settings.boost_s + d * 5, 5, 120);
        break;
    case MENU_CAL: {
        int8_t *p = cal_ptr();
        *p = (int8_t)clampi(*p + d, -50, 50);
        break;
    }
    case MENU_BL:
        settings.backlight = (uint8_t)clampi(settings.backlight + d * 5, 5, 100);
        backlight_set(settings.backlight);
        break;
    case MENU_BEEP:
        settings.beep = !settings.beep;
        break;
    case MENU_UNIT:
        settings.fahrenheit = !settings.fahrenheit;
        break;
    default:
        break;
    }
    apply_settings();
    settings_touch();
}

static void draw_menu(void)
{
    char buf[24], pos[12];

    if (full_redraw) {
        lcd_fill(0, 0, LCD_W, LCD_H, C_BG);
        lcd_mid(16, 14, "Settings", C_PD, C_BG);
        lcd_round_rect(8, 96, 224, 120, 14, C_CARD, C_BG);
        lcd_text(16, 292, "turn: change  click: next", C_FAINT, C_BG, 1);
        lcd_text(16, 304, "hold: exit", C_FAINT, C_BG, 1);
        memset(&shown, 0, sizeof(shown));
    }
    if (full_redraw) {
        snprintf(pos, sizeof(pos), "%u/%u", menu, MENU_COUNT - 1);
        lcd_text(200, 18, pos, C_DIM, C_BG, 1);
    }
    mid_box(shown.m_name, sizeof(shown.m_name), menu_name(menu), 24, 116, 192, AL_LEFT, C_DIM, C_CARD);
    menu_value(menu, buf, sizeof(buf));
    mid_box(shown.m_val, sizeof(shown.m_val), buf, 24, 160, 192, AL_LEFT, C_FG, C_CARD);
    /* Help text only on the calibration item; the area is already clear
     * after a full redraw (each item change is one). */
    if (full_redraw && menu == MENU_CAL) {
        lcd_text(16, 228, "reading + offset = real tip", C_DIM, C_BG, 1);
        lcd_text(16, 244, "temperature (tip thermometer)", C_DIM, C_BG, 1);
    }
}

/* ---- input -------------------------------------------------------------- */
static void click(void)
{
    if (settings.beep) {
        beep(15);
    }
}

static void handle_input(void)
{
    int16_t d = input_take_steps();
    ui_key_t k = input_take_key();

    if (menu == MENU_OFF) {
        if (d) {
            const heater_status_t *h = heater_status();
            if (h->state == HS_SLEEP || h->state == HS_OFF) {
                heater_wake(); /* first turn only wakes */
            } else {
                settings.target_c = (int16_t)clampi(settings.target_c + d * 5, 100, 450);
                heater_set_target(settings.target_c);
                settings_touch();
            }
        }
        if (k == KEY_SHORT) {
            const heater_status_t *h = heater_status();
            click();
            if (h->state == HS_SLEEP || h->state == HS_OFF) {
                heater_wake();
            } else {
                heater_boost(settings.boost_c, h->boost ? 0 : settings.boost_s);
            }
        } else if (k == KEY_LONG) {
            click();
            menu = MENU_STYLE;
            full_redraw = 1;
        }
    } else {
        if (d) {
            menu_adjust(menu, d);
        }
        if (k == KEY_SHORT) {
            click();
            menu = (menu_t)(menu + 1);
            if (menu == MENU_COUNT) {
                menu = MENU_OFF;
            }
            full_redraw = 1;
        } else if (k == KEY_DOUBLE || k == KEY_LONG) {
            click();
            menu = MENU_OFF;
            full_redraw = 1;
        }
    }
}

/* ---- entry points ------------------------------------------------------- */
void ui_init(void)
{
    backlight_set(settings.backlight);
    full_redraw = 1;
    shown.tick_seg = -1;
}

void ui_poll(void)
{
    static menu_t drawn_menu = MENU_COUNT;
    static uint8_t drawn_style = 0xFF;
    const heater_status_t *h;

    handle_input();
    if (millis() - last_draw < 100 && !full_redraw) {
        return;
    }
    last_draw = millis();

    if ((menu == MENU_OFF) != (drawn_menu == MENU_OFF) || settings.ui_style != drawn_style) {
        full_redraw = 1;
    }
    h = heater_status();
    if (menu == MENU_OFF) {
        if (full_redraw) {
            lcd_fill(0, 0, LCD_W, LCD_H, C_BG);
            memset(&shown, 0, sizeof(shown));
            shown.tick_seg = -1;
        }
        if (settings.ui_style) {
            draw_graph_screen(h);
        } else {
            draw_gauge(h);
        }
    } else {
        draw_menu();
    }
    drawn_menu = menu;
    drawn_style = settings.ui_style;
    full_redraw = 0;
}
