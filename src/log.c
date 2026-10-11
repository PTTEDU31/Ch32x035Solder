#include "log.h"

#include <ch32x035.h>
#include <stddef.h>
#include <string.h>

#define LOG_BUF 1024 /* power of two */
#define LOG_DMA DMA1_Channel4
#define LOG_DMA_TC DMA1_FLAG_TC4

static uint8_t buf[LOG_BUF];
static volatile uint16_t head;  /* next write */
static volatile uint16_t tail;  /* start of data not yet handed to DMA */
static volatile uint16_t inflight; /* bytes in the running DMA transfer */
static volatile uint32_t dropped;

void log_init(uint32_t baud)
{
    GPIO_InitTypeDef gpio = {0};
    USART_InitTypeDef us = {0};
    DMA_InitTypeDef dma = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_10;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &gpio);

    us.USART_BaudRate = baud;
    us.USART_WordLength = USART_WordLength_8b;
    us.USART_StopBits = USART_StopBits_1;
    us.USART_Parity = USART_Parity_No;
    us.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    us.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &us);
    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART1, ENABLE);

    DMA_DeInit(LOG_DMA);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DATAR;
    dma.DMA_MemoryBaseAddr = (uint32_t)buf;
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    dma.DMA_BufferSize = 0;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_Low;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(LOG_DMA, &dma);
}

void log_poll(void)
{
    uint16_t h, t, n;

    if (inflight) {
        if (!(DMA1->INTFR & LOG_DMA_TC)) {
            return; /* still sending */
        }
        DMA1->INTFCR = LOG_DMA_TC;
        tail = (uint16_t)((tail + inflight) & (LOG_BUF - 1));
        inflight = 0;
    }
    h = head;
    t = tail;
    if (h == t) {
        return;
    }
    /* Contiguous chunk up to the end of the buffer. */
    n = h > t ? (uint16_t)(h - t) : (uint16_t)(LOG_BUF - t);
    LOG_DMA->CFGR &= ~DMA_CFGR1_EN;
    LOG_DMA->MADDR = (uint32_t)&buf[t];
    LOG_DMA->CNTR = n;
    inflight = n;
    LOG_DMA->CFGR |= DMA_CFGR1_EN;
}

uint32_t log_dropped(void)
{
    return dropped;
}

/* newlib hook: printf() ends up here. Only called from the main loop. */
__attribute__((used)) int _write(int fd, char *s, int size)
{
    uint16_t h = head;
    uint16_t free_bytes = (uint16_t)((tail - h - 1) & (LOG_BUF - 1));

    (void)fd;
    if ((uint16_t)size > free_bytes) {
        dropped += (uint32_t)size;
        return size;
    }
    for (int i = 0; i < size; i++) {
        buf[h] = (uint8_t)s[i];
        h = (uint16_t)((h + 1) & (LOG_BUF - 1));
    }
    head = h;
    log_poll();
    return size;
}


/* Heap for newlib (printf). The SDK debug.c is not built (it has its own
 * blocking _write), so this comes from here; same as the SDK version. */
__attribute__((used)) void *_sbrk(ptrdiff_t incr)
{
    extern char _end[];
    extern char _heap_end[];
    static char *curbrk = _end;

    if ((curbrk + incr < _end) || (curbrk + incr > _heap_end)) {
        return (void *)-1;
    }
    curbrk += incr;
    return curbrk - incr;
}
