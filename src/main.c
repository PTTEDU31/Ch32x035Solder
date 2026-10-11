#include <ch32x035.h>
#include <stdio.h>

#include "board.h"
#include "config.h"
#include "heater.h"
#include "input.h"
#include "lcd.h"
#include "log.h"
#include "out.h"
#include "pd_sink.h"
#include "sense.h"
#include "settings.h"
#include "ui.h"

void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/* Longest gap between two background() calls, in ms: the PD stack only
 * runs there, so this must stay well under ~25 ms. */
static uint32_t bg_last, bg_gap_max;
static uint8_t gap_stage;
/* Longest single run of each main-loop step, ms (to find what blocks). */
static uint32_t t_ui, t_set, t_st;

/* Everything that must keep running, also from inside long LCD draws. */
static void background(void)
{
    uint32_t now = millis();

    if (bg_last && now - bg_last > bg_gap_max) {
        bg_gap_max = now - bg_last;
        gap_stage = ui_stage;
    }
    bg_last = now;
    pd_poll();
    heater_poll();
    log_poll();
    iwdg_feed();
}

#if STATUS_UART
static void status_line(void)
{
    static uint32_t next;
    const heater_status_t *h;
    uint8_t code, arg;

    while (pd_event(&code, &arg)) {
        printf("EV %u %02x\r\n", code, arg);
    }
    if ((int32_t)(millis() - next) < 0) {
        return;
    }
    next = millis() + 1000;
    h = heater_status();
    printf("%lu %s tip=%s err=%s T=%d/%d raw=%u cur=%u on=%u P=%u%% vin=%u pd=%u/%u kind=%d epr=%d id=%umV/%s st=%u f=%02x pb0=%u/%u e=%u/%u/%u ix=%lu pc=%u#%u I=%lumA pk=%lumA isr=%luus gap=%lums drop=%lu ui=%lu set=%lu st=%lu gs=%u\r\n",
           (unsigned long)millis(),
           /* ID sees a tip but TEMP is saturated: P+ is not at 0 V. */
           (h->state == HS_NO_TIP && sense.id_class != ID_CLASS_UNKNOWN && sense.temp_raw >= TEMP_OPEN_RAW)
               ? "TC_HIGH"
               : heater_state_name(h->state),
           h->tip ? h->tip->name : "-",
           heater_err_name(h->err), h->temp_c, h->target_c, sense.temp_raw, sense.cur_raw, sense.on_ms, h->power_pct,
           sense.vin_mv, pd_contract_mv(), pd_contract_ma(), (int)pd_contract_kind(), pd_epr_active(),
           (unsigned)((uint32_t)sense.id_raw * ADC_VREF_MV / ADC_FULL_SCALE),
           sense.id_class == ID_CLASS_GND ? "T12" : sense.id_class == ID_CLASS_OPEN ? "C210/open" : "?", sense.stand, sense.faults, (unsigned)(GPIOB->OUTDR & 1), (unsigned)(GPIOB->INDR & 1),
           sense.t_early[4], sense.t_early[12], sense.t_early[HEATER_SETTLE_MS - 1],
           /* Expected shunt reading if Q5 is fully on: V/R in counts. */
           h->tip ? (unsigned long)((uint32_t)sense.vin_mv * CUR_COUNTS_PER_A / h->tip->r_mohm) : 0UL,
           sense.probe_cur, sense.probe_count, (unsigned long)CUR_MA(sense.cur_raw),
           (unsigned long)CUR_MA(sense.cur_peak), (unsigned long)(isr_max_cycles() / 48),
           (unsigned long)bg_gap_max, (unsigned long)log_dropped(), (unsigned long)t_ui, (unsigned long)t_set,
           (unsigned long)t_st, gap_stage);
    bg_gap_max = 0;
    t_ui = t_set = t_st = 0;
}
#endif

int main(void)
{
    board_init();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    printf("\r\nCH32 soldering station, heater %s\r\n", HEATER_ENABLE ? "ENABLED" : "LOCKED");
    if (FIRST_HEAT) {
        printf("TEST SESSION: max %u mV, %u C, %u s, stand ignored\r\n", FIRST_HEAT_MAX_MV, FIRST_HEAT_MAX_C,
               FIRST_HEAT_TIMEOUT_S);
    }

    /* PD first: chargers stop sending capabilities after a while. */
    pd_init();
    settings_load();
    heater_init();
    heater_set_target(settings.target_c);
    heater_set_sleep_target(settings.sleep_c);
#ifdef UI_STYLE_FORCE
    settings.ui_style = UI_STYLE_FORCE; /* bench: pick a screen without the knob */
#endif
    heater_set_auto_off(settings.off_min);
    heater_set_sleep_stages((uint16_t)settings.sleep1_min * 60, settings.deep_c);
    heater_set_fast_wake(settings.fast_wake);
    tip_set_offset(tip_for_id(ID_CLASS_GND), settings.cal_t12);
    tip_set_offset(tip_for_id(ID_CLASS_OPEN), settings.cal_c210);
    iwdg_init();

    out_init();
    lcd_init(background);
    ui_init();
    if (settings.beep) {
        beep(60);
    }

    while (1) {
        uint32_t t0;
        background();
        t0 = millis();
        ui_poll();
        if (millis() - t0 > t_ui) {
            t_ui = millis() - t0;
        }
        t0 = millis();
        settings_poll();
        if (millis() - t0 > t_set) {
            t_set = millis() - t0;
        }
#if STATUS_UART
        t0 = millis();
        status_line();
        if (millis() - t0 > t_st) {
            t_st = millis() - t0;
        }
#endif
    }
}

void NMI_Handler(void) {}

void HardFault_Handler(void)
{
    heater_force_off();
    NVIC_SystemReset();
}
