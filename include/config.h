#ifndef CONFIG_H
#define CONFIG_H

/*
 * Board configuration for the CH32X035G8U6 soldering station.
 *
 * Assumes the reworked board (see project notes): Q5 fixed, R16 removed,
 * heater drive default-OFF in reset, V_IN divider trimmed.
 * Values marked ASSUMPTION must be matched to the real board.
 */

/* ---- Build mode ---------------------------------------------------------- */
/* 1 = normal use. 0 = the heater gate is never driven on (bench work on
 * the board); everything else still runs. Verified on hardware with T12:
 * 12 V / 150 C and 20 V / 320 C. */
#ifndef HEATER_ENABLE
#define HEATER_ENABLE 1
#endif

/* First supervised heat: low voltage, low setpoint, automatic stop. Build
 * with -DHEATER_ENABLE=1 -DFIRST_HEAT=1, watch the iron, then go back to
 * the normal build. */
#ifndef FIRST_HEAT
#define FIRST_HEAT 0
#endif
/* Limits of a test session; override with -D for later test steps. The
 * stand switch is ignored in a test session so the setpoint is reached. */
#ifndef FIRST_HEAT_MAX_MV
#define FIRST_HEAT_MAX_MV 12000
#endif
#ifndef FIRST_HEAT_MAX_C
#define FIRST_HEAT_MAX_C 150
#endif
#ifndef FIRST_HEAT_TIMEOUT_S
#define FIRST_HEAT_TIMEOUT_S 90
#endif

#define UART_BAUD 115200
/* One status line per second on UART1 TX (PB10), sent by DMA from a ring
 * buffer, so it never blocks. Build with -DSTATUS_UART=0 to turn it off. */
#ifndef STATUS_UART
#define STATUS_UART 1
#endif

/* ---- Pins (QFN28) -------------------------------------------------------- */
/* Heater gate drive, PB0 (pin 13). */
#define HEATER_PORT GPIOB
#define HEATER_PIN GPIO_Pin_0
/* ASSUMPTION: 1 = PB0 low turns the heater on (reworked default-off drive). */
#define HEATER_ACTIVE_LOW 1

#define ADC_CH_TEMP ADC_Channel_0    /* PA0, pin 5: thermocouple amplifier */
#define ADC_CH_CURRENT ADC_Channel_1 /* PA1, pin 6: shunt amplifier */
#define ADC_CH_VIN ADC_Channel_2     /* PA2, pin 7: V_IN divider */
#define ADC_CH_ID ADC_Channel_13     /* PC3, pin 4: tip ID */

#define SLEEP_PORT GPIOB /* PB8, pin 19: stand switch, GND = on stand */
#define SLEEP_PIN GPIO_Pin_8

#define ENC_PORT GPIOB
#define ENC_A_PIN GPIO_Pin_3  /* pin 14 */
#define ENC_B_PIN GPIO_Pin_4  /* pin 15 */
#define ENC_SW_PIN GPIO_Pin_6 /* pin 17, EC11 push switch */
/* Quadrature transitions per detent (EC11: 4 or 2). */
#define ENC_DIV 4
#define ENC_REVERSE 0

/* Buzzer: pin 16 bonds PB1 and PB5; driven by TIM3_CH2 (partial remap 1)
 * on PB5, PB1 is left as an input so the two never fight. */
#define BEEP_PORT GPIOB
#define BEEP_PIN GPIO_Pin_5
#define BEEP_PIN_BONDED GPIO_Pin_1
#define BEEP_HZ 2700
/* LCD backlight (PWM_BLK): PB9, pin 20, TIM1_CH1. */
#define BLK_PORT GPIOB
#define BLK_PIN GPIO_Pin_9
#define BLK_HZ 20000

#define LCD_RES_PIN GPIO_Pin_3 /* PA3 */
#define LCD_DC_PIN GPIO_Pin_4  /* PA4 */
#define LCD_SCK_PIN GPIO_Pin_5 /* PA5, SPI1 SCK */
#define LCD_SDA_PIN GPIO_Pin_7 /* PA7, SPI1 MOSI */
/* ST7796S 480x320 landscape. This module needs display inversion on
 * (with 0 the dark background showed up white). */
#define LCD_INVERT 1
/* MADCTL: MV (row/column exchange) for landscape | BGR. Try 0xE8 (MY|MX|MV)
 * to rotate 180 degrees, or drop 0x08 if red and blue are swapped. */
#define LCD_MADCTL 0x28

/* ---- Analog scaling ------------------------------------------------------ */
#define ADC_VREF_MV 3300
#define ADC_FULL_SCALE 4095

/* V_IN divider (R29 pot trimmed so this ratio reads correctly). */
#define VIN_R_TOP 100000UL
#define VIN_R_BOT 10000UL

/* Thermocouple amplifier U2.1: gain 1 + R3/R4 = 1 + 200k/1k. */
#define TEMP_GAIN 201
/* At or above this the TC input is open (no handle / no tip). */
#define TEMP_OPEN_RAW 4000
/* No cold-junction sensor on the board: assumed handle temperature. */
#define AMBIENT_C 25

/* Shunt R15 = 1 mOhm into U2.2: input divider R18/(R9+R18) = 200k/210k,
 * gain 1 + R10/R11 = 1 + 499k/10k = 50.9 -> 48.5 mV/A, ~60 counts/A,
 * full scale ~68 A. Idle reads ~3 counts (amplifier offset).
 * First heat measured 126 counts at 11.6 V: ~2.1 A, i.e. ~5.5 ohm. */
#define CUR_COUNTS_PER_A 60
#define CUR_OFFSET_RAW 3
#define CUR_MA(raw) ((raw) > CUR_OFFSET_RAW ? (uint32_t)((raw) - CUR_OFFSET_RAW) * 1000U / CUR_COUNTS_PER_A : 0U)
/* Presence probe at 5 V: clearly above the offset (~0.4 A). The bootstrap
 * only gives Vgs ~4.5 V at 5 V, so the probe current is below V/R. */
#define CUR_PRESENT_RAW 25
/* Current limiting, checked every 1 ms while the heater is on:
 * - soft limit = lower of the tip limit and the PD contract current: a
 *   pulse that exceeds it is cut short (no error), the control loop then
 *   lowers the voltage and the duty so the current stays below it;
 * - hard limit = soft x CUR_SHORT_PCT % or CUR_HARD_MAX_MA: only a short
 *   circuit gets there; it latches FAULT_OVERCURRENT. */
#define CUR_HARD_MAX_MA 6000
#define CUR_SHORT_PCT 150

/* Tip ID (PC3): ~0.6 V = T12, 3.3 V (open) = C210. Thresholds sit halfway
 * between the levels so divider tolerance and noise do not matter:
 * below 1.5 V -> T12, above 2.5 V -> C210, in between -> unknown/other. */
#define ID_MV_TO_RAW(mv) ((mv) * ADC_FULL_SCALE / ADC_VREF_MV)
#define ID_GND_MAX_RAW ID_MV_TO_RAW(1500)
#define ID_OPEN_MIN_RAW ID_MV_TO_RAW(2500)

/* ---- Heater timing ------------------------------------------------------- */
#define HEATER_PERIOD_MS 200 /* control cycle */
/* Off time before the TEMP sample. During a pulse the TC input is clamped
 * (D2, 3.3 V) and the input filter has to recover. Measured on the first
 * heat: still saturated at 1 ms, ~2160 counts too high at 5 ms, ~280 at
 * 11 ms, i.e. tau ~3 ms. 25 ms leaves ~8 tau (< 1 count). */
#define HEATER_SETTLE_MS 25
/* Soft start: the on time may grow by at most this much per cycle, so the
 * load on the charger ramps up over ~2 s instead of stepping to full power.
 * Each pulse still draws V/R; the PD contract is chosen so that V/R stays
 * within the source current. */
#define SOFTSTART_STEP_MS 20
#define HEATER_PROBE_MS 2    /* presence probe pulse */

/* ---- USB-PD -------------------------------------------------------------- */
/* The board has external 5.1k Rd on CC1/CC2 (R12/R13), needed so a charger
 * powers us before the MCU runs. Set to 1 only if those are removed. */
#ifndef PD_INTERNAL_RD
#define PD_INTERNAL_RD 0
#endif
/* Highest voltage ever requested. Above 20 V needs an EPR source and a
 * 5 A EPR cable, and every part on V_BUS rated for it (see README). */
#define PD_MAX_MV 28000
#define PD_ALLOW_PPS 1
#define PD_ALLOW_EPR 1
/* Operational power announced when entering EPR mode, in W. */
#define PD_EPR_WATTS 140
/* Some sources (Lenovo Legion 140W) only set the EPR capable bit after
 * entry; try EPR whenever more than 20 V is wanted. */
#ifndef PD_EPR_FORCE_TRY
#define PD_EPR_FORCE_TRY 1
#endif

/* ---- Safety limits ------------------------------------------------------- */
#define VIN_MIN_MV 4500
/* Bootstrap gate drive (C1 + SS54, 12 V zener): Vgs ~= V_IN - 0.5 V up to
 * the clamp. Below ~8 V the IRFH7084 is not fully enhanced, so the heater
 * only runs (beyond the 2 ms presence probe) at or above this. */
#define HEATER_MIN_RUN_MV 8000
/* Allowed V_IN deviation from the PD contract while heating, percent. */
#define VIN_TOL_PCT 10
#define TEMP_HARD_MAX_C 480
/* Runaway check: with >= half power for this long the tip must warm up by
 * at least RUNAWAY_MIN_RISE_C, otherwise the sensor is wrong/inverted. */
#define RUNAWAY_MS 4000
#define RUNAWAY_MIN_RISE_C 8

/* ---- Settings storage ---------------------------------------------------- */
/* Two 256 B pages just below the 62 KB code limit. platformio.ini lowers
 * board_upload.maximum_size so the linker cannot place code here. */
#define SETTINGS_ADDR_A 0x0800F600UL
#define SETTINGS_ADDR_B 0x0800F700UL

/* Watchdog: LSI ~40 kHz / 32, reload 1500 -> ~1.2 s. */
#define IWDG_RELOAD 1500

#endif
