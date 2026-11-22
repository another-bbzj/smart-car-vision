#ifndef DRV_LCD_H
#define DRV_LCD_H
/*
 * ============================================================================
 * 文件名称：Drv_Lcd.h
 * ============================================================================
 * 模块功能：基于 NEUQ_LCD200 的“按行显示文字”，横屏 320x240，10 行 × 32 列。
 *
 *   外设资源：ST7789V2 并口屏，初始化与像素写入由 NEUQ_LCD200 完成。
 *
 *   运行视角
 *     菜单每 120 ms 只重画“变了的那一行里变了的那几个字符”：
 *     每行缓存上次显示的内容，逐字符比较后才发送像素，所以刷新数值不闪。
 *
 *   为什么不用 LCD200_DisplayString 整行重画：
 *     一个 10×20 字符要传 200 像素，整页重画几十毫秒并伴随闪屏；
 *     按字符增量刷新通常只传几个字符，快且不闪。
 * ============================================================================
 */

#include <stdint.h>

#define LCD_ROWS             (10)        // 行数
#define LCD_COLS             (32)        // 每行字符数（320 像素 / 字宽 10）
#define LCD_ROW_PITCH        (24)        // 行距，单位像素（字高 20 + 行间隔 4）

typedef enum {                          // 行的显示样式（字色 / 底色）
    UI_STYLE_NORMAL = 0,                // 黑字白底：普通内容
    UI_STYLE_TITLE,                     // 白字深蓝底：页面标题
    UI_STYLE_SELECT,                    // 白字蓝底：光标所在行
    UI_STYLE_EDIT,                      // 黑字黄底：正在编辑的参数
    UI_STYLE_GOOD,                      // 深绿字：正常 / 运行中
    UI_STYLE_WARN,                      // 红字：警告 / 请等待
    UI_STYLE_HINT,                      // 灰字：操作提示
    UI_STYLE_NUM
} UI_Style_Enum;

void Lcd_Init(void);                                                    // 初始化屏幕（白底）
void Lcd_Print(uint8_t row, UI_Style_Enum style, const char *fmt, ...); // 显示一整行：不足补空格，超出截断
void Lcd_Invalidate(void);                                              // 忘掉缓存，下次显示时每行整行重画

#endif
