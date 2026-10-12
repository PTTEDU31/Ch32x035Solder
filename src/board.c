#include "board.h"

#include <ch32x035.h>
#include <stdio.h>

#include "config.h"
#include "input.h"
#include "log.h"
#include "out.h"
#include "sense.h"

#define TICKS_PER_MS (48000000UL / 1000)
#define TICKS_PER_US (48000000UL / 1000000)

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

static volatile uint32_t ms_ticks;
static volatile uint32_t isr_max;

void SysTick_Handler(void)
{
    uint32_t t0 = (uint32_t)SysTick->CNT, t1;

    SysTick->SR = 0;
    ms_ticks++;
    sense_tick();
    input_tick();
    out_tick();
    t1 = (uint32_t)SysTick->CNT;
    t1 = t1 >= t0 ? t1 - t0 : t1 + TICKS_PER_MS - t0;
    if (t1 > isr_max) {
        isr_max = t1;
    }
}

uint32_t cycles_now(void)
{
    return (uint32_t)SysTick->CNT;
}

uint32_t isr_max_cycles(void)
{
    uint32_t m = isr_max;
    isr_max = 0;
    return m;
}

#if HEATER_PWM_OFF_PCT
_Static_assert(HEATER_ACTIVE_LOW, "heater PWM drives PB0 low = on");
_Static_assert(HEATER_PIN == GPIO_Pin_0, "heater PWM is TIM1_CH2N on PB0");
#endif

static volatile uint8_t gate_pwm_ready; /* TIM1 CH2N configured (out_init) */
static uint8_t gate_af;                 /* PB0 currently belongs to the timer */

static void gate_mode(GPIOMode_TypeDef mode)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.GPIO_Pin = HEATER_PIN;
    gpio.GPIO_Mode = mode;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HEATER_PORT, &gpio);
}

void heater_pwm_enable(void)
{
    gate_pwm_ready = 1;
}

/* The OFF level is always written first and every off path ends in plain
 * GPIO mode, so no path can leave the timer driving the gate. The mode only
 * changes at the start and end of an on window (read-modify-write of
 * GPIOB->CFGLR, nothing else touches PB0..7 configuration at run time). */
void heater_pin(int mode)
{
    if (mode == HEATER_PWM && HEATER_PWM_OFF_PCT && gate_pwm_ready) {
        if (!gate_af) {
            gate_af = 1;
            gate_mode(GPIO_Mode_AF_PP);
        }
        return;
    }
    if (gate_af) {
        gate_af = 0;
        gate_mode(GPIO_Mode_Out_PP);
    }
    /* Solid on, or off, as a GPIO; PWM requested before the timer is up
     * falls back to solid on (as before the PWM existed). */
#if HEATER_ACTIVE_LOW
    if (mode != HEATER_OFF) {
        GPIO_ResetBits(HEATER_PORT, HEATER_PIN);
    } else {
        GPIO_SetBits(HEATER_PORT, HEATER_PIN);
    }
#else
    if (mode != HEATER_OFF) {
        GPIO_SetBits(HEATER_PORT, HEATER_PIN);
    } else {
        GPIO_ResetBits(HEATER_PORT, HEATER_PIN);
    }
#endif
}

void heater_force_off(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    heater_pin(HEATER_OFF);
    gpio.GPIO_Pin = HEATER_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(HEATER_PORT, &gpio);
}

static void systick_init(void)
{
    SysTick->CTLR = 0;
    SysTick->SR = 0;
    SysTick->CNT = 0;
    SysTick->CMP = TICKS_PER_MS - 1;
    /* STE | STIE | STCLK (HCLK) | STRE (auto reload) | INIT */
    SysTick->CTLR = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5);
    NVIC_SetPriority(SysTick_IRQn, 0x80);
    NVIC_EnableIRQ(SysTick_IRQn);
}

uint32_t millis(void)
{
    return ms_ticks;
}

void delay_us(uint32_t us)
{
    uint32_t last = (uint32_t)SysTick->CNT;
    uint32_t left = us * TICKS_PER_US;

    while (left) {
        uint32_t now = (uint32_t)SysTick->CNT;
        uint32_t step = now >= last ? now - last : now + TICKS_PER_MS - last;
        last = now;
        left = step >= left ? 0 : left - step;
    }
}

void delay_ms(uint32_t ms)
{
    while (ms--) {
        delay_us(1000);
    }
}

static void inputs_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    gpio.GPIO_Pin = SLEEP_PIN | ENC_A_PIN | ENC_B_PIN | ENC_SW_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    GPIO_Init(GPIOC, &gpio);
}

static void adc_init(void)
{
    ADC_InitTypeDef adc = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    ADC_DeInit(ADC1);
    ADC_CLKConfig(ADC1, ADC_CLK_Div6);
    adc.ADC_Mode = ADC_Mode_Independent;
    adc.ADC_ScanConvMode = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &adc);
    ADC_Cmd(ADC1, ENABLE);
}

uint16_t adc_read(uint8_t channel)
{
    ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_11Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
    }
    return ADC_GetConversionValue(ADC1);
}

uint32_t vin_mv(void)
{
    return sense.vin_mv;
}

void iwdg_init(void)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(IWDG_Prescaler_32);
    IWDG_SetReload(IWDG_RELOAD);
    IWDG_ReloadCounter();
    IWDG_Enable();
}

void iwdg_feed(void)
{
    /* Only while the 1 ms ISR is alive: it owns the heater. */
    static uint32_t last;
    uint32_t now = ms_ticks;

    if (now != last) {
        last = now;
        IWDG_ReloadCounter();
    }
}

void board_init(void)
{
    heater_force_off();
    SystemCoreClockUpdate();
    log_init(UART_BAUD);
    inputs_init();
    adc_init();
    sense_init();
    input_init();
    systick_init();
}
