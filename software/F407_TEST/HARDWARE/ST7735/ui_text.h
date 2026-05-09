#ifndef __UI_TEXT_H__
#define __UI_TEXT_H__

#include "st7735.h"

void UiText_Init(void);
void UiText_Clear(uint16_t color);
void UiText_ShowString(uint16_t x,
                       uint16_t y,
                       const char *pStr,
                       uint16_t pointColor,
                       uint16_t backColor);
uint8_t UiText_TryShowString(uint16_t x,
                             uint16_t y,
                             const char *pStr,
                             uint16_t pointColor,
                             uint16_t backColor);
void UiText_Process(void);
uint8_t UiText_IsBusy(void);

#endif
