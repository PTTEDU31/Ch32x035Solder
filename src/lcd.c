/*
 * ST7796S 480x320 landscape over SPI1 (PA5 SCK, PA7 MOSI), DC on PA4, RES on
 * PA3, no CS (CS tied low on the module).
 * Modules without CS need SPI mode 3. No framebuffer: everything is drawn
 * directly, in strips, calling the yield hook between strips.
 *
 * Pixel data goes out by DMA (DMA1 channel 3 = SPI1_TX) with SPI in 16-bit
 * mode: fills repeat one word from memory without incrementing, glyphs are
 * rendered into one of two line buffers while DMA sends the other.
 * Commands and their parameters stay byte-wise (8-bit mode, polled).
 */
#include "lcd.h"

#include <ch32x035.h>

#include "board.h"
#include "config.h"
#include "fonts.h"

#define STRIP_PIXELS 2400 /* ~1 ms at 24 MHz */
#define LCD_DMA DMA1_Channel3
#define LCD_DMA_TC DMA1_FLAG_TC3
#define PIXBUF 480 /* pixels per buffer (two lines), x2 buffers = 1.9 KB */

static lcd_yield_fn yield_fn;
static uint16_t pixbuf[2][PIXBUF];
static uint8_t pixsel;
static uint16_t pixn;
static uint8_t dma_busy;
static uint16_t fill_word;

static inline void spi_wait(void)
{
    while (!(SPI1->STATR & SPI_I2S_FLAG_TXE)) {
    }
}

static void dma_wait(void)
{
    if (dma_busy) {
        while (!(DMA1->INTFR & LCD_DMA_TC)) {
        }
        DMA1->INTFCR = LCD_DMA_TC;
        LCD_DMA->CFGR &= ~DMA_CFGR1_EN;
        dma_busy = 0;
    }
}

static inline void spi_done(void)
{
    dma_wait();
    spi_wait();
    while (SPI1->STATR & SPI_I2S_FLAG_BSY) {
    }
}

/* SPI frame size; only changed while idle. */
static void spi_16bit(int on)
{
    uint16_t c = SPI1->CTLR1;
    if (!!(c & SPI_CTLR1_DFF) == !!on) {
        return;
    }
    spi_done();
    SPI1->CTLR1 = (uint16_t)(c & ~SPI_CTLR1_SPE);
    SPI1->CTLR1 = (uint16_t)((on ? (c | SPI_CTLR1_DFF) : (c & ~SPI_CTLR1_DFF)) & ~SPI_CTLR1_SPE);
    SPI1->CTLR1 |= SPI_CTLR1_SPE;
}

/* Start sending n 16-bit words from p (inc = 1) or n copies of *p. */
static void dma_start(const uint16_t *p, uint16_t n, int inc)
{
    dma_wait();
    spi_16bit(1);
    LCD_DMA->CFGR = DMA_DIR_PeripheralDST | DMA_PeripheralDataSize_HalfWord | DMA_MemoryDataSize_HalfWord |
                    DMA_Priority_Medium | (inc ? DMA_MemoryInc_Enable : 0);
    LCD_DMA->PADDR = (uint32_t)&SPI1->DATAR;
    LCD_DMA->MADDR = (uint32_t)p;
    LCD_DMA->CNTR = n;
    LCD_DMA->CFGR |= DMA_CFGR1_EN;
    dma_busy = 1;
}

/* Buffered pixel writer: px_put() fills the current buffer, px_flush()
 * hands it to DMA and switches to the other one. */
static void px_flush(void)
{
    if (pixn) {
        dma_start(pixbuf[pixsel], pixn, 1);
        pixsel ^= 1;
        pixn = 0;
    }
}

static inline void px_put(uint16_t c)
{
    pixbuf[pixsel][pixn++] = c;
    if (pixn == PIXBUF) {
        px_flush();
    }
}

static inline void spi8(uint8_t b)
{
    spi_wait();
    *(volatile uint8_t *)&SPI1->DATAR = b;
}

static void cmd(uint8_t c)
{
    px_flush();
    spi_16bit(0);
    spi_done();
    GPIO_ResetBits(GPIOA, LCD_DC_PIN);
    spi8(c);
    spi_done();
    GPIO_SetBits(GPIOA, LCD_DC_PIN);
}

static void data(uint8_t d)
{
    spi8(d);
}

/* Commands while drawing stay in 16-bit SPI mode: the word 0x00cc is sent
 * with DC low, i.e. NOP (0x00) followed by the command, and the CASET/RASET
 * parameters are 16-bit values anyway. This avoids switching the SPI frame
 * size (and waiting for idle) several times per window. */
static void cmd16(uint8_t c)
{
    px_flush();
    spi_16bit(1);
    spi_done();
    GPIO_ResetBits(GPIOA, LCD_DC_PIN);
    SPI1->DATAR = c;
    spi_done();
    GPIO_SetBits(GPIOA, LCD_DC_PIN);
}

static inline void data16(uint16_t d)
{
    spi_wait();
    SPI1->DATAR = d;
}

static void window(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    cmd16(0x2A);
    data16(x);
    data16((uint16_t)(x + w - 1));
    cmd16(0x2B);
    data16(y);
    data16((uint16_t)(y + h - 1));
    cmd16(0x2C);
}

/* Pixels always go through the buffer (sent by DMA, 16-bit frames). */
static inline void px(uint16_t c)
{
    px_put(c);
}

static void yield(void)
{
    px_flush();
    spi_done();
    if (yield_fn) {
        yield_fn();
    }
}

void lcd_init(lcd_yield_fn y)
{
    GPIO_InitTypeDef gpio = {0};
    SPI_InitTypeDef spi = {0};

    yield_fn = y;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_SPI1, ENABLE);

    gpio.GPIO_Pin = LCD_SCK_PIN | LCD_SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);
    gpio.GPIO_Pin = LCD_RES_PIN | LCD_DC_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);

    spi.SPI_Direction = SPI_Direction_1Line_Tx;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(SPI1, &spi);
    SPI_Cmd(SPI1, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
    SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

    /* Hardware reset, then the ST7796S sequence (as used by TFT_eSPI).
     * Delays are split so PD keeps running. */
    GPIO_SetBits(GPIOA, LCD_DC_PIN);
    GPIO_ResetBits(GPIOA, LCD_RES_PIN);
    for (int i = 0; i < 10; i++) {
        delay_ms(1);
        yield();
    }
    GPIO_SetBits(GPIOA, LCD_RES_PIN);
    for (int i = 0; i < 120; i++) {
        delay_ms(1);
        yield();
    }
    cmd(0x01); /* software reset */
    for (int i = 0; i < 120; i++) {
        delay_ms(1);
        yield();
    }
    cmd(0x11); /* sleep out */
    for (int i = 0; i < 120; i++) {
        delay_ms(1);
        yield();
    }
    cmd(0xF0); /* command set control: enable extension part I */
    data(0xC3);
    cmd(0xF0); /* part II */
    data(0x96);
    cmd(0x36); /* MADCTL */
    data(LCD_MADCTL);
    cmd(0x3A); /* 16 bpp */
    data(0x55);
    cmd(0xB4); /* 1-dot inversion */
    data(0x01);
    cmd(0xB6); /* display function control */
    data(0x80);
    data(0x02);
    data(0x3B);
    cmd(0xE8); /* display output ctrl adjust */
    data(0x40);
    data(0x8A);
    data(0x00);
    data(0x00);
    data(0x29);
    data(0x19);
    data(0xA5);
    data(0x33);
    cmd(0xC1); /* power control 2 */
    data(0x06);
    cmd(0xC2); /* power control 3 */
    data(0xA7);
    cmd(0xC5); /* VCOM */
    data(0x18);
    {
        static const uint8_t gp[14] = {0xF0, 0x09, 0x0B, 0x06, 0x04, 0x15, 0x2F,
                                       0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B};
        static const uint8_t gn[14] = {0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B,
                                       0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B};
        cmd(0xE0); /* positive gamma */
        for (int i = 0; i < 14; i++) {
            data(gp[i]);
        }
        cmd(0xE1); /* negative gamma */
        for (int i = 0; i < 14; i++) {
            data(gn[i]);
        }
    }
    cmd(0xF0); /* disable extension part I */
    data(0x3C);
    cmd(0xF0); /* part II */
    data(0x69);
    cmd(LCD_INVERT ? 0x21 : 0x20);
    cmd(0x13); /* normal display */
    lcd_fill(0, 0, LCD_W, LCD_H, 0);
    cmd(0x29); /* display on */
    spi_done();
}

void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    uint32_t n = (uint32_t)w * h;

    if (!w || !h) {
        return;
    }
    window(x, y, w, h);
    fill_word = color;
    /* DMA repeats one word; the CPU only steps in between strips. The
     * window stays open while yielding (yield never touches the LCD). */
    while (n) {
        uint16_t k = n > STRIP_PIXELS ? STRIP_PIXELS : (uint16_t)n;
        dma_start(&fill_word, k, 0);
        n -= k;
        if (n) {
            dma_wait();
            if (yield_fn) {
                yield_fn();
            }
        }
    }
    spi_done();
}

uint16_t lcd_text(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale)
{
    if (scale < 1) {
        scale = 1;
    }
    for (; *s; s++) {
        uint8_t c = (uint8_t)*s;
        const uint8_t *g;
        if (c < FONT_SMALL_FIRST || c > FONT_SMALL_LAST) {
            c = '?';
        }
        if (x + FONT_SMALL_W * scale > LCD_W) {
            break;
        }
        g = &font8x16[(c - FONT_SMALL_FIRST) * FONT_SMALL_H];
        window(x, y, FONT_SMALL_W * scale, FONT_SMALL_H * scale);
        for (uint8_t row = 0; row < FONT_SMALL_H; row++) {
            for (uint8_t sy = 0; sy < scale; sy++) {
                for (uint8_t col = 0; col < FONT_SMALL_W; col++) {
                    uint16_t color = (g[row] & (0x80 >> col)) ? fg : bg;
                    for (uint8_t sx = 0; sx < scale; sx++) {
                        px(color);
                    }
                }
            }
        }
        x += FONT_SMALL_W * scale;
        yield();
    }
    spi_done();
    return x;
}

void lcd_yield(void)
{
    yield();
}

void lcd_begin(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    window(x, y, w, h);
}

void lcd_push(uint16_t c)
{
    px(c);
}

void lcd_end(void)
{
    spi_done();
}

uint16_t lcd_mix(uint16_t fg, uint16_t bg, uint8_t a)
{
    uint16_t r, g, b;

    if (a >= 15) {
        return fg;
    }
    if (a == 0) {
        return bg;
    }
    r = (uint16_t)((((fg >> 11) & 0x1F) * a + ((bg >> 11) & 0x1F) * (15 - a)) / 15);
    g = (uint16_t)((((fg >> 5) & 0x3F) * a + ((bg >> 5) & 0x3F) * (15 - a)) / 15);
    b = (uint16_t)(((fg & 0x1F) * a + (bg & 0x1F) * (15 - a)) / 15);
    return (uint16_t)((r << 11) | (g << 5) | b);
}

/* 16-entry palette from bg to fg for 4 bpp glyphs. */
static void palette(uint16_t *pal, uint16_t fg, uint16_t bg)
{
    for (uint8_t i = 0; i < 16; i++) {
        pal[i] = lcd_mix(fg, bg, i);
    }
}

uint16_t lcd_big(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    uint16_t pal[16];

    palette(pal, fg, bg);
    for (; *s; s++) {
        int idx;
        if (*s >= '0' && *s <= '9') {
            idx = *s - '0';
        } else if (*s == '-') {
            idx = FONT_BIG_DASH;
        } else {
            lcd_fill(x, y, FONT_BIG_W, FONT_BIG_H, bg);
            x += FONT_BIG_W;
            continue;
        }
        {
            const uint8_t *g = &font_big[idx * (FONT_BIG_W / 2) * FONT_BIG_H];
            window(x, y, FONT_BIG_W, FONT_BIG_H);
            for (uint16_t i = 0; i < (FONT_BIG_W / 2) * FONT_BIG_H; i++) {
                px(pal[g[i] >> 4]);
                px(pal[g[i] & 0x0F]);
                if (i % ((FONT_BIG_W / 2) * 10) == (FONT_BIG_W / 2) * 10 - 1) {
                    yield(); /* window stays open across the pause */
                }
            }
        }
        yield();
        x += FONT_BIG_W;
    }
    spi_done();
    return x;
}

uint16_t lcd_big2(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    uint16_t pal[16];
    const uint16_t rb = FONT_BIG_W / 2; /* bytes per glyph row */

    palette(pal, fg, bg);
    for (; *s; s++) {
        int idx;
        if (*s >= '0' && *s <= '9') {
            idx = *s - '0';
        } else if (*s == '-') {
            idx = FONT_BIG_DASH;
        } else {
            lcd_fill(x, y, FONT_BIG_W * 2, FONT_BIG_H * 2, bg);
            x += FONT_BIG_W * 2;
            continue;
        }
        {
            const uint8_t *g = &font_big[idx * rb * FONT_BIG_H];
            window(x, y, FONT_BIG_W * 2, FONT_BIG_H * 2);
            for (uint16_t row = 0; row < FONT_BIG_H; row++) {
                const uint8_t *r = &g[row * rb];
                for (uint8_t rep = 0; rep < 2; rep++) {
                    /* Horizontal 2x with the in-between pixel interpolated
                     * for a smoother edge. */
                    for (uint16_t i = 0; i < rb; i++) {
                        uint8_t a = r[i] >> 4, b = r[i] & 0x0F;
                        uint8_t n = i + 1 < rb ? (r[i + 1] >> 4) : b;
                        px(pal[a]);
                        px(pal[(a + b) >> 1]);
                        px(pal[b]);
                        px(pal[(b + n) >> 1]);
                    }
                }
                if ((row & 3) == 3) {
                    yield(); /* window stays open across the pause */
                }
            }
        }
        yield();
        x += FONT_BIG_W * 2;
    }
    spi_done();
    return x;
}

static uint8_t mid_index(char c)
{
    uint8_t u = (uint8_t)c;
    if (u < FONT_MID_FIRST || u > FONT_MID_LAST) {
        u = '?';
    }
    return (uint8_t)(u - FONT_MID_FIRST);
}

uint16_t lcd_mid_width(const char *s)
{
    uint16_t w = 0;

    for (; *s; s++) {
        w += font_mid_w[mid_index(*s)];
    }
    return w;
}

uint16_t lcd_mid(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg)
{
    uint16_t pal[16];

    palette(pal, fg, bg);
    for (; *s; s++) {
        uint8_t i = mid_index(*s);
        uint8_t w = font_mid_w[i];
        const uint8_t *g = &font_mid[font_mid_off[i]];
        if (x + w > LCD_W) {
            break;
        }
        window(x, y, w, FONT_MID_H);
        for (uint16_t n = 0; n < (uint16_t)(w / 2) * FONT_MID_H; n++) {
            px(pal[g[n] >> 4]);
            px(pal[g[n] & 0x0F]);
        }
        x += w;
        yield();
    }
    spi_done();
    return x;
}

void lcd_pixel(uint16_t x, uint16_t y, uint16_t c)
{
    window(x, y, 1, 1);
    px(c);
}

void lcd_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t c)
{
    if (!h) {
        return;
    }
    window(x, y, 1, h);
    while (h--) {
        px(c);
    }
}

void lcd_round_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint16_t color, uint16_t bg)
{
    /* Middle band, then r rows top and bottom with trimmed, blended ends. */
    lcd_fill(x, y + r, w, h - 2 * r, color);
    for (uint8_t i = 0; i < r; i++) {
        /* Distance of this row from the corner centre. */
        int32_t dy = r - i;
        int32_t dx2 = (int32_t)r * r - dy * dy;
        uint8_t inset = 0, edge = 15;
        if (dx2 <= 0) {
            inset = r;
        } else {
            int32_t dx = 0;
            while ((dx + 1) * (dx + 1) <= dx2) {
                dx++;
            }
            inset = (uint8_t)(r - dx);
            edge = (uint8_t)(15 * (dx2 - dx * dx) / (2 * dx + 1));
        }
        for (int k = 0; k < 2; k++) {
            uint16_t yy = k ? (uint16_t)(y + h - 1 - i) : (uint16_t)(y + i);
            if (inset > 0) {
                lcd_fill(x, yy, inset - 1, 1, bg);
                lcd_pixel(x + inset - 1, yy, lcd_mix(color, bg, edge));
                lcd_pixel(x + w - inset, yy, lcd_mix(color, bg, edge));
                lcd_fill(x + w - inset + 1, yy, inset - 1, 1, bg);
            }
            lcd_fill(x + inset, yy, w - 2 * inset, 1, color);
        }
        yield();
    }
}
