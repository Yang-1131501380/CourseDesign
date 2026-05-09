#include "ui_text.h"
#include "st7735.h"

void UiText_Init(void)
{
    ST7735_Init();
}

void UiText_Clear(uint16_t color)
{
    ST7735_FillScreen(color);
}

void UiText_ShowString(uint16_t x,
                       uint16_t y,
                       const char *pStr,
                       uint16_t pointColor,
                       uint16_t backColor)
{
    (void)ST7735_DrawString16Dma(x, y, pStr, pointColor, backColor);
}

void UiText_Process(void)
{
    ST7735_Process();
}

uint8_t UiText_IsBusy(void)
{
    return ST7735_IsBusy();
}
