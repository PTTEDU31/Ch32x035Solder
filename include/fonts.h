#ifndef FONTS_H
#define FONTS_H

#include <stdint.h>

#define FONT_SMALL_W 8
#define FONT_SMALL_H 16
#define FONT_SMALL_FIRST 32
#define FONT_SMALL_LAST 126

/* Big digits: '0'..'9' then '-', 4 bpp (two pixels per byte, high nibble
 * first), fixed FONT_BIG_W x FONT_BIG_H cells. */
#define FONT_BIG_W 48
#define FONT_BIG_H 60
#define FONT_BIG_DASH 10

/* Medium proportional font, 4 bpp: codes 32..127, 127 = degree sign. */
#define FONT_MID_H 22
#define FONT_MID_FIRST 32
#define FONT_MID_LAST 127
#define FONT_MID_DEG "\x7f"

extern const uint8_t font8x16[];
extern const uint8_t font_big[];
extern const uint8_t font_mid[];
extern const uint8_t font_mid_w[];
extern const uint16_t font_mid_off[];

#endif
