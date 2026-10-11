# CH32X035 soldering station firmware

PlatformIO project (noneos-sdk) for the CH32X035G8U6 T12/C210 station.

Branches: `main` = ST7789 240x320 portrait, `st7796-480x320` = ST7796S
480x320 landscape (this branch).

## Status

| Part | State |
|---|---|
| USB-PD 3.0 sink: fixed + PPS | verified on hardware: 5/9/15/20 V fixed, 13.3/18/21 V PPS, PPS refresh held 40 s |
| USB-PD 3.1 EPR (fixed 28 V, AVS 15-28 V) | verified on hardware with Lenovo Legion 140W: EPR entry, 28 V fixed, 24 V AVS, keep-alive |
| Idle at 5 V without a tip, no watchdog reset | verified on hardware |
| Tip detect / heater control / PI / soft start / current limit | verified on hardware with T12 at 12 V: detect, 1 ms current trip armed, soft start, holds 150 C +/-2 C after ~25 s (first-heat build) |
| LCD ST7789 UI, encoder, stand switch, buzzer, backlight | old UI shows on the panel (user); new UI (SPI DMA) runs, look not yet confirmed |
| PID + feed-forward, anti-windup | verified at 20 V / 320 C: holds 316-320 C after the windup fix |
| Timing | measured: SysTick ISR <= 133 us, PD/heater background gap 2-9 ms (limit ~25 ms), log drops 0 |
| Two-stage sleep, fast wake, buzzer on TIM3 | compiled, not verified on the iron |
| Settings in flash, watchdog | compiled; watchdog runs without resets at idle |

### Bring-up order still to do

1. Look at the LCD: does it show the main screen (status bar, `---`, SET 320C,
   footer `NO TIP (HEATER LOCKED)`)? If colours are inverted set
   `LCD_INVERT 0`; if nothing shows, the panel may need SPI mode 0.
2. Turn the knob (SET changes by 5), click (BOOST), long press (menu), put
   the iron on the stand.
3. Plug in a T12 with `HEATER_ENABLE 0`: the UART line should show
   `tip=T12`, the PD contract and `raw=` dropping below 4000. Heat the tip
   with another iron: `raw` must rise. If it falls, the thermocouple sign is
   inverted and must be fixed in hardware before going further.
4. Supervised first heat: build with `-DHEATER_ENABLE=1`, setpoint 150 C,
   stay at the bench. Check `HEATER_ACTIVE_LOW`, the temperature, and that
   `NO HEAT` (runaway) does not trigger.

## Before running above 20 V (EPR)

28 V needs an EPR source (140 W class) and a 5 A EPR-rated e-marked cable.
Every part on V_BUS / V_IN must be rated for it:

- SY8303 buck input maximum
- C16, C17, C18, C6 voltage ratings
- R1 / D1 dissipation on the Q5 gate drive
- amplifier input protection on P+

Lower `PD_MAX_MV` or set `PD_ALLOW_EPR 0` until these are checked.

## Bench note

When the board runs from the WCH-Link 3.3 V with a charger attached, the
3.3 V rail back-feeds V_IN to about 2.9 V through the buck. Some chargers
wait for V_BUS < 0.8 V after a hard reset and then stay off: unplug and
replug the charger. On USB-C power alone this does not happen.

## Tips (values in src/tips.c)

| Tip | ID | PD request | Notes |
|---|---|---|---|
| T12 | ID = GND | up to 24 V (AVS) / 20 V fixed, ~8 ohm | Hakko curve, max 75 W |
| C210 | ID open | PPS/AVS 5 A, ~9-10 V, ~2 ohm | THEORY: ~10 uV/C, verify |
| C245 | (no ID yet) | 24 V, ~2.5 ohm | disabled, needs an ID scheme |
| C410 | ? | ? | not defined |

Thermocouple amplifier gain 201 (full scale ~16.4 mV). No cold-junction
sensor: `AMBIENT_C` is assumed. The shunt amplifier saturates at ~1.6 A, so
current is used for presence only; power is computed from V_IN and the
nominal heater resistance.

## Temperature control

PID once per 200 ms heater cycle (heater off 25 ms, TEMP sample, then on up
to 173 ms) plus feed-forward:

- Feed-forward: the duty that holds a temperature is ~proportional to
  (T - ambient); the ratio is learned while holding (|error| < 3 C) and
  applied immediately on setpoint changes and wake-up.
- P 1.5 %/C, I only within +/-30 C of the setpoint (anti-windup), D on the
  filtered measurement (reacts to a joint pulling heat out, no kick on
  setpoint steps). Gains picked with a thermal model fitted to the bench
  runs; tune on the iron with a real solder joint.
- Soft start (on time +20 ms per cycle) only from a cold bus.

## Sleep and wake

- Stage 1: `sleep_c` (default 200 C) for `sleep1_min` (default 3 min),
  then deep sleep at `deep_c` (150 C), then off after `off_min` (10 min).
- Wake on lifting the iron or turning/clicking the knob. Fast wake allows
  the tip's boost voltage (24 V on EPR/AVS sources) and +15 W for up to
  10 s or until the setpoint, without soft start.
- Model estimate to 320 C at 20 V: ~10.6 s from 200 C, ~13.6 s from 150 C.

## Calibration

The board has no cold-junction sensor and the CH32X035 has no internal
temperature sensor (its only internal ADC channel is Vrefint). The reading
assumes `AMBIENT_C` (25 C) at the handle, plus a per-tip offset set in the
menu ("Tip offset"): hold the working temperature, measure the tip with a
tip thermometer (e.g. Hakko FG-100 / 191 style), and enter the difference.
This is how most T12 stations are calibrated (single- or multi-point offset
against a tip thermometer). For cold-junction compensation add an NTC near
the handle connector on a free ADC pin.

## Display

ST7796S 480x320 landscape over SPI (same pins as the ST7789: SCK PA5, MOSI
PA7, DC PA4, RES PA3, backlight PB9; CS tied low). `LCD_MADCTL` (0x28 =
landscape + BGR) and `LCD_INVERT` in config.h adjust rotation, colour order
and inversion for a given module.

Two styles (menu "Screen"): **Gauge** (ring with the temperature inside on
the left, chips / SET / power cards on the right) and **Graph** (2x
temperature, chips and SET at the top, ~114 s history with the setpoint
line and power bar below). Both show
tip / PD contract chips, the SET card (boost time left or minutes to
auto-off), and a power bar with %, W (from measured current) and V_IN.

## CPU and peripherals

| Block | How |
|---|---|
| SysTick 1 ms ISR | ADC (polled, ~10-50 us), heater gate, encoder, stand, beep end |
| USB-PD | its own DMA; GoodCRC in the USBPD ISR, policy in the main loop |
| LCD SPI1 | 16-bit DMA (DMA1 ch3): fills repeat one word, glyphs go through two 480-pixel line buffers |
| UART log USART1 | DMA (DMA1 ch4) from a 1 KB ring buffer; printf never blocks, overflow is dropped and counted. On by default (`-DSTATUS_UART=0` to disable) |
| TIM1 CH1 | backlight PWM 20 kHz (PB9) |
| TIM3 CH2 | buzzer 2.7 kHz (PB5, remap 1; PB1 bonded on the same pin is an input) |
| Heater | GPIO from the 1 ms ISR, 200 ms cycle with a 25 ms off window for the TEMP sample |

The status line reports `isr=` (longest SysTick ISR, us),
`gap=` (longest time between two runs of the PD/heater background, ms; must
stay well under ~25 ms) and `drop=` (log bytes dropped).

## Heater drive (N-FET high side with bootstrap)

Q5 IRFH7084 (N-channel) is the high-side switch. Its gate is pulled up by
R1 = 2.2 k and boosted by C1 (470 nF) charged through SS54 while the heater
is off; D1 (12 V zener) clamps Vgs. Q2 pulls the gate low: H_PWM high =
heater off.

- The bootstrap needs an off phase before every on phase. The firmware
  cycle is 100 ms with at least 5 ms off (TEMP sample), and the heater is
  off while idle, so C1 is always charged. No fast PWM is needed; higher
  frequencies only add switching loss in Q5 (slow gate through R1) and
  zener dissipation.
- With H_PWM low but C1 never charged (no prior off phase) Q5 only
  half-conducts: tip ~= V_IN - Vgs(th), ~1.3 V at 5 V. This is what is seen
  when the MCU is in reset and PB0 floats.
- At 5 V Vgs is only ~4.5 V: the heater only runs at V_IN >=
  `HEATER_MIN_RUN_MV` (8 V). 5 V is used only for the 2 ms presence probe.
- R1 dissipates V_IN^2 / 2.2 k while the heater is off: 0.18 W at 20 V,
  0.36 W at 28 V. Use a 0805/1206 part rated for that (or raise R1).
- **Recommended:** 10 k pull-up from H_PWM (PB0) to 3.3 V so the heater is
  off while the MCU is in reset or being flashed. Without it, a reset
  during a 20-28 V contract leaves Q5 half on (several W in Q5, tip heating
  uncontrolled) until the firmware runs again.

## Assumptions to match to the board

- `HEATER_ACTIVE_LOW 1`: H_PWM low = heater on (Q2 off, gate pulled up by R1).
- `VIN_R_TOP`/`VIN_R_BOT`: V_IN divider ratio (pot trimmed so 9 V reads 9.0 V).
