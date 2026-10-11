#include "out.h"

#include <ch32x035.h>

#include "config.h"

/* Both timers count at 48 MHz / 6 = 8 MHz. */
#define TIM_CLK 8000000UL
#define BLK_PERIOD (TIM_CLK / BLK_HZ)
#define BEEP_PERIOD (TIM_CLK / BEEP_HZ)

static volatile uint16_t beep_left;

static void timer_base(TIM_TypeDef *tim, uint32_t period)
{
    TIM_TimeBaseInitTypeDef tb = {0};

    tb.TIM_Prescaler = (uint16_t)(48000000UL / TIM_CLK - 1);
    tb.TIM_Period = (uint16_t)(period - 1);
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(tim, &tb);
}

void out_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    TIM_OCInitTypeDef oc = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM3, ENABLE);

    /* PB1 is bonded to PB5 on pin 16: input only. */
    gpio.GPIO_Pin = BEEP_PIN_BONDED;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &gpio);

    gpio.GPIO_Pin = BLK_PIN | BEEP_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = 0;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;

    /* Backlight: TIM1 CH1 only. Break input disabled (BDTR.BKE = 0). */
    timer_base(TIM1, BLK_PERIOD);
    TIM_OC1Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    /* Buzzer: TIM3 CH2, silent (0 % duty) until beep(). */
    timer_base(TIM3, BEEP_PERIOD);
    TIM_OC2Init(TIM3, &oc);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    TIM_Cmd(TIM3, ENABLE);
}

void backlight_set(uint8_t pct)
{
    if (pct > 100) {
        pct = 100;
    }
    TIM1->CH1CVR = (uint16_t)(BLK_PERIOD * pct / 100);
}

void beep(uint16_t ms)
{
    beep_left = ms;
    TIM3->CH2CVR = BEEP_PERIOD / 2;
}

void out_tick(void)
{
    if (beep_left && --beep_left == 0) {
        TIM3->CH2CVR = 0;
    }
}
