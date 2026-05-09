#ifndef __ST7735_H__
#define __ST7735_H__

#include "main.h"
#include <stdint.h>

#define ST7735_WIDTH                    128U
#define ST7735_HEIGHT                   160U

#define ST7735_X_OFFSET                 0U
#define ST7735_Y_OFFSET                 0U

#define ST7735_FONT_WIDTH               8U
#define ST7735_FONT_HEIGHT              16U

#define ST7735_COLOR_WHITE              0xFFFFU
#define ST7735_COLOR_BLACK              0x0000U
#define ST7735_COLOR_BLUE               0x001FU
#define ST7735_COLOR_RED                0xF800U
#define ST7735_COLOR_GREEN              0x07E0U
#define ST7735_COLOR_CYAN               0x07FFU
#define ST7735_COLOR_MAGENTA            0xF81FU
#define ST7735_COLOR_YELLOW             0xFFE0U

void ST7735_Init(void);
void ST7735_SetBackLight(uint8_t enable);
void ST7735_FillScreen(uint16_t color);
void ST7735_FillRect(uint16_t x,
                     uint16_t y,
                     uint16_t width,
                     uint16_t height,
                     uint16_t color);
void ST7735_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void ST7735_DrawChar16(uint16_t x,
                       uint16_t y,
                       char ch,
                       uint16_t pointColor,
                       uint16_t backColor);
void ST7735_DrawString16(uint16_t x,
                         uint16_t y,
                         const char *pStr,
                         uint16_t pointColor,
                         uint16_t backColor);
uint8_t ST7735_DrawString16Dma(uint16_t x,
                               uint16_t y,
                               const char *pStr,
                               uint16_t pointColor,
                               uint16_t backColor);
void ST7735_Process(void);
uint8_t ST7735_IsBusy(void);

#endif
