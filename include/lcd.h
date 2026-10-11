#ifndef LCD_DRV_H
#define LCD_DRV_H

#include <stdint.h>

/* ST7796S, 480x320 landscape. */
#define LCD_W 480
#define LCD_H 320

#define RGB(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))

/* Long drawing operations call this between strips so the USB-PD stack
 * keeps its response deadlines. */
typedef void (*lcd_yield_fn)(void);

void lcd_init(lcd_yield_fn yield);
void lcd_fill(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
/* 8x16 text, scale 1..3. Returns the x after the text. */
uint16_t lcd_text(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg, uint8_t scale);
/* Large anti-aliased digits ('0'-'9', '-', ' '), FONT_BIG_W x FONT_BIG_H. */
uint16_t lcd_big(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg);
/* Same digits drawn at 2x (96x120 cells): no extra flash. */
uint16_t lcd_big2(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg);
/* Medium anti-aliased proportional text (FONT_MID_H high). */
uint16_t lcd_mid(uint16_t x, uint16_t y, const char *s, uint16_t fg, uint16_t bg);
uint16_t lcd_mid_width(const char *s);
/* Filled rectangle with rounded corners (radius r), corners blended to bg. */
void lcd_round_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t r, uint16_t color, uint16_t bg);
/* Mix two RGB565 colours, a = 0..15 weight of fg. */
uint16_t lcd_mix(uint16_t fg, uint16_t bg, uint8_t a);
/* One pixel / vertical line (no yield). */
void lcd_pixel(uint16_t x, uint16_t y, uint16_t c);
void lcd_vline(uint16_t x, uint16_t y, uint16_t h, uint16_t c);
/* Let the background work run (PD) between small drawing steps. */
void lcd_yield(void);
/* Streamed drawing: open a window, push w*h pixels row by row (sent by
 * DMA through the line buffers), then end. */
void lcd_begin(uint16_t x, uint16_t y, uint16_t w, uint16_t h);
void lcd_push(uint16_t c);
void lcd_end(void);

#endif
