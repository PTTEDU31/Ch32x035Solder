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

/* Everything that must keep running, also from inside long LCD draws. */
static void background(void)
{
    uint32_t now = millis();

    if (bg_last && now - bg_last > bg_gap_max) {
        bg_gap_max = now - bg_last;
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
    printf("%lu %s %s %s T=%d/%d P=%u%% I=%lumA V=%u pd=%u/%u%s st=%u f=%02x isr=%luus gap=%lums cut=%u pk=%lu\r\n",
           (unsigned long)millis(), heater_state_name(h->state), h->tip ? h->tip->name : "-", heater_err_name(h->err),
           h->temp_c, h->target_c, h->power_pct, (unsigned long)CUR_MA(sense.cur_raw), sense.vin_mv,
           pd_contract_mv(), pd_contract_ma(), pd_epr_active() ? " EPR" : "", sense.stand, sense.faults,
           (unsigned long)(isr_max_cycles() / 48), (unsigned long)bg_gap_max, sense.cur_cut,
           (unsigned long)CUR_MA(sense.cur_peak));
    bg_gap_max = 0;
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
        background();
        ui_poll();
        settings_poll();
#if STATUS_UART
        status_line();
#endif
    }
}

void NMI_Handler(void) {}

void HardFault_Handler(void)
{
    heater_force_off();
    NVIC_SystemReset();
}
