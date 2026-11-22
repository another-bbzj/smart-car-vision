/*
 * ============================================================================
 * 文件名称：Drv_Lcd.c
 * ============================================================================
 * 模块功能
 * ----------------------------------------------------------------------------
 * 文本行缓存 + 增量刷新：
 *   1. Lcd_Init()       ：初始化屏幕并清空行缓存；
 *   2. Lcd_Print()      ：格式化一行文字，逐字符与缓存比较，只重画变化的字符；
 *   3. Lcd_Invalidate() ：丢掉缓存，下次整行重画（切页/复位后用）。
 *
 * 行缓存的意义：
 *   屏幕刷新是整机最慢的操作（毫秒级），菜单每 120 ms 刷 10 行。
 *   缓存把“每行要显示什么”留在 RAM 里，没变的字符一个像素都不发。
 *
 * 函数列表
 *   void Lcd_Init(void)
 *   void Lcd_Invalidate(void)
 *   void Lcd_Print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
 * ============================================================================
 */
#include "NEUQ_lib_include.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "Drv_Lcd.h"

// 每种样式的字色和底色（RGB888，库会自动转成屏幕用的 RGB565），顺序与 UI_Style_Enum 一致
static const uint32_t s_pen_color[UI_STYLE_NUM]  = {LCD_BLACK, LCD_WHITE, LCD_WHITE, LCD_BLACK, DARK_GREEN, LCD_RED, DARK_GREY};
static const uint32_t s_back_color[UI_STYLE_NUM] = {LCD_WHITE, DARK_BLUE, LCD_BLUE,  LCD_YELLOW, LCD_WHITE, LCD_WHITE, LCD_WHITE};

// 屏幕内容缓存：每一行现在显示的文字和样式
static char    s_text[LCD_ROWS][LCD_COLS + 1];
static uint8_t s_style[LCD_ROWS];
static uint8_t s_valid[LCD_ROWS];        // 0 表示这一行的缓存不可信（刚上电 / 被清过），下次必须整行重画

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化屏幕
// 备注信息     NEUQ_LCD200_init 内部会调用 printf 打印调试信息，所以之前必须先 NEUQ_Debug_init 初始化串口，
//              否则 printf 会一直等待一个没打开的串口
//-------------------------------------------------------------------------------------------------------------------
/*
 * 函数名称：Lcd_Init
 * 函数功能：初始化屏幕并把行缓存清空。
 * 参数说明：无。
 * 返回参数：无。
 * 备注信息：初始化由 NEUQ_LCD200 完成；行缓存清空后第一次 Lcd_Print 会整行重画，
 *           避免把上电前屏幕上的随机内容当成“已经显示过”。
 */
void Lcd_Init(void)
{
    NEUQ_LCD200_init();                         // 横屏、白底清屏、打开背光
    NEUQ_LCD200_SetAsciiFont(&ASCII_Font20);    // 10x20 字体，一行正好 32 个字
    Lcd_Invalidate();
}

void Lcd_Invalidate(void)
{
    uint8_t row;
    for (row = 0; row < LCD_ROWS; row++) {
        s_valid[row] = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示一整行
// 参数说明     row     行号 0 ~ LCD_ROWS-1
// 参数说明     style   样式，见 UI_Style_Enum
// 参数说明     fmt     和 printf 一样的格式字符串
// 备注信息     1. 先把内容格式化成正好 32 个字符（不足补空格），这样新内容一定能完全覆盖旧内容
//              2. 样式变了（比如光标移到这一行）就整行重画；样式没变就逐个字符比较，只画变了的字符
//-------------------------------------------------------------------------------------------------------------------
void Lcd_Print(uint8_t row, UI_Style_Enum style, const char *fmt, ...)
{
    char    line[LCD_COLS + 1];
    va_list args;
    int     len;
    uint8_t col;
    uint16_t y = (uint16_t)(row * LCD_ROW_PITCH);

    if (row >= LCD_ROWS || style >= UI_STYLE_NUM) {
        return;
    }

    // 1. 格式化并补齐到 32 个字符；字库里只有可见 ASCII 字符，其他字符一律换成空格
    va_start(args, fmt);
    len = vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    for (col = 0; col < LCD_COLS; col++) {
        if ((int)col >= len || (uint8_t)line[col] < 32U || (uint8_t)line[col] > 126U) {
            line[col] = ' ';
        }
    }
    line[LCD_COLS] = '\0';

    NEUQ_LCD200_SetPenColor(s_pen_color[style]);
    NEUQ_LCD200_SetBackColor(s_back_color[style]);

    // 2. 样式变了或缓存不可信：整行重画
    if (!s_valid[row] || s_style[row] != (uint8_t)style) {
        NEUQ_LCD200_DisplayString(0, y, line);
        memcpy(s_text[row], line, sizeof(line));
        s_style[row] = (uint8_t)style;
        s_valid[row] = 1;
        return;
    }

    // 3. 样式没变：只重画内容不同的字符（字宽 ASCII_Font20.Width = 10 像素）
    for (col = 0; col < LCD_COLS; col++) {
        if (line[col] != s_text[row][col]) {
            NEUQ_LCD200_DisplayChar((uint16_t)(col * ASCII_Font20.Width), y, (uint8_t)line[col]);
            s_text[row][col] = line[col];
        }
    }
}
