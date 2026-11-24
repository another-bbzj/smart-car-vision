/*********************************************************************************************************************
* 文件名称          My_Lcd
* 功能说明          按行显示中英文混合文字（UTF-8，ASCII 10x20 + 中文 20x20），说明见 My_Lcd.h
*
* 修改记录
* 日期              备注
* 2026-09-27        first version
* 2026-11-24        支持 UTF-8 20x20 汉字字模与 32 列单元差分刷新
********************************************************************************************************************/
#include "NEUQ_lib_include.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "My_Lcd.h"
#include "My_Lcd_FontZh.h"

extern LCD200_Config_Struct LCD200_Config;
extern uint16_t LCD200_Buff[4096];

#define CELL_CONT (0xFFFFU)

// 每种样式的字色和底色（RGB888，库会自动转成屏幕用的 RGB565），顺序与 UI_Style_Enum 一致
static const uint32_t s_pen_color[UI_STYLE_NUM]  = {LCD_BLACK, LCD_WHITE, LCD_WHITE, LCD_BLACK, DARK_GREEN, LCD_RED, DARK_GREY};
static const uint32_t s_back_color[UI_STYLE_NUM] = {LCD_WHITE, DARK_BLUE, LCD_BLUE,  LCD_YELLOW, LCD_WHITE, LCD_WHITE, LCD_WHITE};

// 屏幕内容缓存：每行 32 个逻辑半角列；全角汉字首列存 Unicode，次列存 CELL_CONT
static uint16_t s_cells[UI_ROWS][UI_COLS];
static uint8_t  s_style[UI_ROWS];
static uint8_t  s_valid[UI_ROWS];        // 0 表示这一行的缓存不可信（刚上电 / 被清过），下次必须整行重画

static const uint8_t *find_zh_glyph(uint16_t unicode)
{
    unsigned lo = 0U;
    unsigned hi = MY_LCD_ZH_COUNT;
    while (lo < hi) {
        unsigned mid = lo + ((hi - lo) >> 1);
        uint16_t key = s_my_lcd_zh_font[mid].unicode;
        if (key == unicode) {
            return s_my_lcd_zh_font[mid].bitmap;
        }
        if (key < unicode) {
            lo = mid + 1U;
        } else {
            hi = mid;
        }
    }
    return NULL;
}

static void draw_zh20(uint16_t x, uint16_t y, const uint8_t *bmp)
{
    uint16_t pen = LCD200_Config.Pen_Color;
    uint16_t bg  = LCD200_Config.Background_Color;
    uint16_t idx = 0U;
    uint8_t row, col;

    if (bmp != NULL) {
        for (row = 0U; row < MY_LCD_ZH_HEIGHT; row++) {
            const uint8_t *r = bmp + (unsigned)row * MY_LCD_ZH_STRIDE;
            for (col = 0U; col < MY_LCD_ZH_WIDTH; col++) {
                uint8_t bit = (uint8_t)(r[col >> 3] & (0x80U >> (col & 7U)));
                LCD200_Buff[idx++] = bit ? pen : bg;
            }
        }
    } else {
        for (row = 0U; row < MY_LCD_ZH_HEIGHT; row++) {
            for (col = 0U; col < MY_LCD_ZH_WIDTH; col++) {
                int border = (row == 2U || row == 17U || col == 2U || col == 17U) &&
                             (row >= 2U && row <= 17U && col >= 2U && col <= 17U);
                LCD200_Buff[idx++] = border ? pen : bg;
            }
        }
    }

    NEUQ_LCD200_SetAddress(x, y,
                           (uint16_t)(x + MY_LCD_ZH_WIDTH - 1U),
                           (uint16_t)(y + MY_LCD_ZH_HEIGHT - 1U));
    NEUQ_LCD200_WriteBuff(LCD200_Buff, (uint16_t)(MY_LCD_ZH_WIDTH * MY_LCD_ZH_HEIGHT));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化屏幕
// 备注信息     NEUQ_LCD200_init 内部会调用 printf 打印调试信息，所以之前必须先关闭调试打印或初始化串口
//-------------------------------------------------------------------------------------------------------------------
void My_Lcd_init(void)
{
    NEUQ_LCD200_init();                         // 横屏、白底清屏、打开背光
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font20);    // 10x20 字体，一行 32 半角列 / 16 汉字
    My_Lcd_invalidate();
}

void My_Lcd_invalidate(void)
{
    uint8_t row;
    for (row = 0U; row < UI_ROWS; row++) {
        s_valid[row] = 0U;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示一整行（支持 ASCII + UTF-8 中文混排）
// 参数说明     row     行号 0 ~ UI_ROWS-1
// 参数说明     style   样式，见 UI_Style_Enum
// 参数说明     fmt     和 printf 一样的 UTF-8 格式字符串
// 备注信息     1. 先解码为 32 个逻辑半角列（汉字占 2 列、ASCII 占 1 列，尾部补空格覆盖旧内容）
//              2. 样式变化或未初始化时整行重画；否则逐列差分对比，只重画变化单元，无残影不闪屏
//-------------------------------------------------------------------------------------------------------------------
void My_Lcd_print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
{
    char     line[UI_LINE_BYTES];
    uint16_t cells[UI_COLS];
    va_list  args;
    int      len;
    uint8_t  col, full;
    uint16_t y = (uint16_t)(row * UI_ROW_PITCH);
    const uint8_t *p;

    if (row >= UI_ROWS || style >= UI_STYLE_NUM) {
        return;
    }

    line[0] = '\0';
    va_start(args, fmt);
    len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    if (len < 0) {
        line[0] = '\0';
    }

    col = 0U;
    p = (const uint8_t *)line;
    while (*p != '\0' && col < UI_COLS) {
        if (*p < 0x80U) {
            uint8_t ch = (*p >= 32U && *p <= 126U) ? *p : (uint8_t)' ';
            cells[col++] = (uint16_t)ch;
            p++;
        } else if ((p[0] & 0xE0U) == 0xC0U && (p[1] & 0xC0U) == 0x80U) {
            uint16_t cp = (uint16_t)(((uint16_t)(p[0] & 0x1FU) << 6) |
                                     (uint16_t)(p[1] & 0x3FU));
            if (col + 2U > UI_COLS) break;
            cells[col++] = cp;
            cells[col++] = CELL_CONT;
            p += 2;
        } else if ((p[0] & 0xF0U) == 0xE0U && (p[1] & 0xC0U) == 0x80U && (p[2] & 0xC0U) == 0x80U) {
            uint16_t cp = (uint16_t)(((uint16_t)(p[0] & 0x0FU) << 12) |
                                     ((uint16_t)(p[1] & 0x3FU) << 6) |
                                     (uint16_t)(p[2] & 0x3FU));
            if (col + 2U > UI_COLS) break;
            cells[col++] = cp;
            cells[col++] = CELL_CONT;
            p += 3;
        } else {
            cells[col++] = (uint16_t)' ';
            p++;
        }
    }
    while (col < UI_COLS) {
        cells[col++] = (uint16_t)' ';
    }

    NEUQ_LCD200_SetPenColor(s_pen_color[style]);
    NEUQ_LCD200_SetBackColor(s_back_color[style]);

    full = (!s_valid[row] || s_style[row] != (uint8_t)style) ? 1U : 0U;
    col = 0U;
    while (col < UI_COLS) {
        uint16_t c = cells[col];
        if (c > 0x7FU && c != CELL_CONT && (col + 1U) < UI_COLS) {
            if (full || s_cells[row][col] != c || s_cells[row][col + 1U] != CELL_CONT) {
                draw_zh20((uint16_t)(col * ASCII_Font20.Width), y, find_zh_glyph(c));
                s_cells[row][col] = c;
                s_cells[row][col + 1U] = CELL_CONT;
            }
            col += 2U;
        } else {
            uint8_t ch = (c >= 32U && c <= 126U) ? (uint8_t)c : (uint8_t)' ';
            if (full || s_cells[row][col] != (uint16_t)ch) {
                NEUQ_LCD200_DisplayChar((uint16_t)(col * ASCII_Font20.Width), y, ch);
                s_cells[row][col] = (uint16_t)ch;
            }
            col += 1U;
        }
    }

    s_style[row] = (uint8_t)style;
    s_valid[row] = 1U;
}
