#include "st7735.h"
#include "spi.h"
#include "Font.h"
#include "rtthread.h"

#define ST7735_TEXT_LINE_BYTES          (ST7735_WIDTH * ST7735_FONT_HEIGHT * 2U)
#define ST7735_TEXT_MAX_CHARS           ((ST7735_WIDTH / ST7735_FONT_WIDTH) * \
                                         (ST7735_HEIGHT / ST7735_FONT_HEIGHT))
#define ST7735_TEXT_QUEUE_SIZE          4U

#define ST7735_CMD_SWRESET              0x01U
#define ST7735_CMD_SLPOUT               0x11U
#define ST7735_CMD_INVON                0x21U
#define ST7735_CMD_DISPON               0x29U
#define ST7735_CMD_CASET                0x2AU
#define ST7735_CMD_RASET                0x2BU
#define ST7735_CMD_RAMWR                0x2CU
#define ST7735_CMD_MADCTL               0x36U
#define ST7735_CMD_COLMOD               0x3AU
#define ST7735_CMD_FRMCTR1              0xB1U
#define ST7735_CMD_FRMCTR2              0xB2U
#define ST7735_CMD_FRMCTR3              0xB3U
#define ST7735_CMD_INVCTR               0xB4U
#define ST7735_CMD_DISSET5              0xB6U
#define ST7735_CMD_PWCTR1               0xC0U
#define ST7735_CMD_PWCTR2               0xC1U
#define ST7735_CMD_PWCTR3               0xC2U
#define ST7735_CMD_PWCTR4               0xC3U
#define ST7735_CMD_PWCTR5               0xC4U
#define ST7735_CMD_VMCTR1               0xC5U
#define ST7735_CMD_GMCTRP1              0xE0U
#define ST7735_CMD_GMCTRN1              0xE1U

static void ST7735_Select(void);
static void ST7735_Unselect(void);
static void ST7735_WriteCommand(uint8_t cmd);
static void ST7735_WriteData(const uint8_t *pData, uint16_t len);
static void ST7735_Reset(void);
static void ST7735_SetWindow(uint16_t x0,
                             uint16_t y0,
                             uint16_t x1,
                             uint16_t y1);
static void ST7735_WaitIdle(void);
static uint8_t ST7735_AsyncPrepareLine(uint8_t bufIndex);
static uint8_t ST7735_AsyncStartDma(uint8_t bufIndex);
static uint8_t ST7735_StartQueuedText(void);
static void ST7735_CopyText(char *pDst, const char *pSrc);
static void ST7735_AsyncWritePixel(uint8_t *pBuf,
                                   uint16_t width,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t color);
static void ST7735_AsyncRenderChar(uint8_t *pBuf,
                                   uint16_t width,
                                   uint16_t x,
                                   char ch,
                                   uint16_t pointColor);

typedef struct _st7735AsyncText {
    uint8_t buf[2][ST7735_TEXT_LINE_BYTES];
    char text[ST7735_TEXT_MAX_CHARS + 1U];
    uint16_t startX;
    uint16_t nextY;
    uint16_t dmaY;
    uint16_t dmaX;
    uint16_t dmaWidth;
    uint16_t pointColor;
    uint16_t backColor;
    uint16_t textIndex;
    uint16_t bufX[2];
    uint16_t bufWidth[2];
    uint8_t activeBuf;
    uint8_t preparedBuf;
    uint8_t preparedValid;
    uint8_t dmaBusy;
    uint8_t taskActive;
} ST7735_ASYNC_TEXT_S;

typedef struct _st7735TextReq {
    char text[ST7735_TEXT_MAX_CHARS + 1U];
    uint16_t x;
    uint16_t y;
    uint16_t pointColor;
    uint16_t backColor;
} ST7735_TEXT_REQ_S;

static ST7735_ASYNC_TEXT_S g_st7735AsyncText;
static ST7735_TEXT_REQ_S g_st7735TextQueue[ST7735_TEXT_QUEUE_SIZE];
static uint8_t g_st7735TextHead;
static uint8_t g_st7735TextTail;
static uint8_t g_st7735TextCount;

static void ST7735_Select(void)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void ST7735_Unselect(void)
{
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void ST7735_WriteCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    ST7735_Select();
    (void)HAL_SPI_Transmit(&hspi2, &cmd, 1U, 100U);
    ST7735_Unselect();
}

static void ST7735_WriteData(const uint8_t *pData, uint16_t len)
{
    if ((pData == NULL) || (len == 0U))
    {
        return;
    }

    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    ST7735_Select();
    (void)HAL_SPI_Transmit(&hspi2, (uint8_t *)pData, len, 100U);
    ST7735_Unselect();
}

static void ST7735_Reset(void)
{
    HAL_GPIO_WritePin(LCD_SDO_RST_GPIO_Port,
                      LCD_SDO_RST_Pin,
                      GPIO_PIN_RESET);
    rt_thread_mdelay(20U);
    HAL_GPIO_WritePin(LCD_SDO_RST_GPIO_Port,
                      LCD_SDO_RST_Pin,
                      GPIO_PIN_SET);
    rt_thread_mdelay(20U);
}

static void ST7735_WaitIdle(void)
{
    while ((g_st7735AsyncText.dmaBusy != 0U) ||
           (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY))
    {
        rt_thread_yield();
    }
}

static void ST7735_SetWindow(uint16_t x0,
                             uint16_t y0,
                             uint16_t x1,
                             uint16_t y1)
{
    uint8_t data[4];

    x0 += ST7735_X_OFFSET;
    x1 += ST7735_X_OFFSET;
    y0 += ST7735_Y_OFFSET;
    y1 += ST7735_Y_OFFSET;

    ST7735_WriteCommand(ST7735_CMD_CASET);
    data[0] = (uint8_t)(x0 >> 8);
    data[1] = (uint8_t)(x0 & 0xFFU);
    data[2] = (uint8_t)(x1 >> 8);
    data[3] = (uint8_t)(x1 & 0xFFU);
    ST7735_WriteData(data, 4U);

    ST7735_WriteCommand(ST7735_CMD_RASET);
    data[0] = (uint8_t)(y0 >> 8);
    data[1] = (uint8_t)(y0 & 0xFFU);
    data[2] = (uint8_t)(y1 >> 8);
    data[3] = (uint8_t)(y1 & 0xFFU);
    ST7735_WriteData(data, 4U);

    ST7735_WriteCommand(ST7735_CMD_RAMWR);
}

void ST7735_SetBackLight(uint8_t enable)
{
    HAL_GPIO_WritePin(LCD_BLK_GPIO_Port,
                      LCD_BLK_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ST7735_Init(void)
{
    uint8_t data[16];

    g_st7735AsyncText.preparedValid = 0U;
    g_st7735AsyncText.dmaBusy = 0U;
    g_st7735AsyncText.taskActive = 0U;
    g_st7735TextHead = 0U;
    g_st7735TextTail = 0U;
    g_st7735TextCount = 0U;

    ST7735_SetBackLight(0U);
    ST7735_Reset();

    ST7735_WriteCommand(ST7735_CMD_SWRESET);
    rt_thread_mdelay(120U);

    ST7735_WriteCommand(ST7735_CMD_SLPOUT);
    rt_thread_mdelay(120U);

    data[0] = 0x01U;
    data[1] = 0x2CU;
    data[2] = 0x2DU;
    ST7735_WriteCommand(ST7735_CMD_FRMCTR1);
    ST7735_WriteData(data, 3U);

    ST7735_WriteCommand(ST7735_CMD_FRMCTR2);
    ST7735_WriteData(data, 3U);

    data[0] = 0x01U;
    data[1] = 0x2CU;
    data[2] = 0x2DU;
    data[3] = 0x01U;
    data[4] = 0x2CU;
    data[5] = 0x2DU;
    ST7735_WriteCommand(ST7735_CMD_FRMCTR3);
    ST7735_WriteData(data, 6U);

    data[0] = 0x07U;
    ST7735_WriteCommand(ST7735_CMD_INVCTR);
    ST7735_WriteData(data, 1U);

    data[0] = 0xA2U;
    data[1] = 0x02U;
    data[2] = 0x84U;
    ST7735_WriteCommand(ST7735_CMD_PWCTR1);
    ST7735_WriteData(data, 3U);

    data[0] = 0xC5U;
    ST7735_WriteCommand(ST7735_CMD_PWCTR2);
    ST7735_WriteData(data, 1U);

    data[0] = 0x0AU;
    data[1] = 0x00U;
    ST7735_WriteCommand(ST7735_CMD_PWCTR3);
    ST7735_WriteData(data, 2U);

    data[0] = 0x8AU;
    data[1] = 0x2AU;
    ST7735_WriteCommand(ST7735_CMD_PWCTR4);
    ST7735_WriteData(data, 2U);

    data[0] = 0x8AU;
    data[1] = 0xEEU;
    ST7735_WriteCommand(ST7735_CMD_PWCTR5);
    ST7735_WriteData(data, 2U);

    data[0] = 0x0EU;
    ST7735_WriteCommand(ST7735_CMD_VMCTR1);
    ST7735_WriteData(data, 1U);

    ST7735_WriteCommand(ST7735_CMD_INVON);

    data[0] = 0x05U;
    ST7735_WriteCommand(ST7735_CMD_COLMOD);
    ST7735_WriteData(data, 1U);

    data[0] = 0xC8U;
    ST7735_WriteCommand(ST7735_CMD_MADCTL);
    ST7735_WriteData(data, 1U);

    data[0] = 0x02U;
    data[1] = 0x1CU;
    data[2] = 0x07U;
    data[3] = 0x12U;
    data[4] = 0x37U;
    data[5] = 0x32U;
    data[6] = 0x29U;
    data[7] = 0x2DU;
    data[8] = 0x29U;
    data[9] = 0x25U;
    data[10] = 0x2BU;
    data[11] = 0x39U;
    data[12] = 0x00U;
    data[13] = 0x01U;
    data[14] = 0x03U;
    data[15] = 0x10U;
    ST7735_WriteCommand(ST7735_CMD_GMCTRP1);
    ST7735_WriteData(data, 16U);

    data[0] = 0x03U;
    data[1] = 0x1DU;
    data[2] = 0x07U;
    data[3] = 0x06U;
    data[4] = 0x2EU;
    data[5] = 0x2CU;
    data[6] = 0x29U;
    data[7] = 0x2DU;
    data[8] = 0x2EU;
    data[9] = 0x2EU;
    data[10] = 0x37U;
    data[11] = 0x3FU;
    data[12] = 0x00U;
    data[13] = 0x00U;
    data[14] = 0x02U;
    data[15] = 0x10U;
    ST7735_WriteCommand(ST7735_CMD_GMCTRN1);
    ST7735_WriteData(data, 16U);

    ST7735_WriteCommand(ST7735_CMD_DISPON);
    rt_thread_mdelay(20U);

    ST7735_SetBackLight(1U);
    ST7735_FillScreen(ST7735_COLOR_BLACK);
}

void ST7735_FillRect(uint16_t x,
                     uint16_t y,
                     uint16_t width,
                     uint16_t height,
                     uint16_t color)
{
    uint32_t pixelCount;
    uint32_t remain;
    uint32_t chunkPixels;
    uint32_t i;
    uint8_t hi;
    uint8_t lo;
    uint8_t buf[128];

    ST7735_WaitIdle();

    if ((width == 0U) || (height == 0U))
    {
        return;
    }

    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
    {
        return;
    }

    if ((x + width) > ST7735_WIDTH)
    {
        width = (uint16_t)(ST7735_WIDTH - x);
    }

    if ((y + height) > ST7735_HEIGHT)
    {
        height = (uint16_t)(ST7735_HEIGHT - y);
    }

    hi = (uint8_t)(color >> 8);
    lo = (uint8_t)(color & 0xFFU);

    for (i = 0; i < sizeof(buf); i += 2U)
    {
        buf[i] = hi;
        buf[i + 1U] = lo;
    }

    pixelCount = (uint32_t)width * (uint32_t)height;
    remain = pixelCount;

    ST7735_SetWindow(x, y, (uint16_t)(x + width - 1U), (uint16_t)(y + height - 1U));

    while (remain > 0U)
    {
        chunkPixels = remain;
        if (chunkPixels > (sizeof(buf) / 2U))
        {
            chunkPixels = sizeof(buf) / 2U;
        }

        ST7735_WriteData(buf, (uint16_t)(chunkPixels * 2U));
        remain -= chunkPixels;
    }
}

void ST7735_FillScreen(uint16_t color)
{
    ST7735_FillRect(0U, 0U, ST7735_WIDTH, ST7735_HEIGHT, color);
}

void ST7735_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
    ST7735_FillRect(x, y, 1U, 1U, color);
}

void ST7735_DrawChar16(uint16_t x,
                       uint16_t y,
                       char ch,
                       uint16_t pointColor,
                       uint16_t backColor)
{
    uint8_t line;
    uint8_t bit;
    uint16_t index;

    if ((ch < ' ') || (ch > '~'))
    {
        ch = '?';
    }

    index = (uint16_t)(ch - ' ') * 16U;

    for (line = 0U; line < 16U; line++)
    {
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((asc16[index + line] & (0x80U >> bit)) != 0U)
            {
                ST7735_DrawPoint((uint16_t)(x + bit), y + line, pointColor);
            }
            else
            {
                ST7735_DrawPoint((uint16_t)(x + bit), y + line, backColor);
            }
        }
    }
}

void ST7735_DrawString16(uint16_t x,
                         uint16_t y,
                         const char *pStr,
                         uint16_t pointColor,
                         uint16_t backColor)
{
    uint16_t curX;
    char ch;

    if (pStr == NULL)
    {
        return;
    }

    curX = x;
    while (*pStr != '\0')
    {
        ch = *pStr;
        if (ch == '\r')
        {
            pStr++;
            continue;
        }

        if (ch == '\n')
        {
            curX = x;
            y += 16U;
            pStr++;
            continue;
        }

        if ((curX + 8U) > ST7735_WIDTH)
        {
            curX = x;
            y += 16U;
        }

        if ((y + 16U) > ST7735_HEIGHT)
        {
            break;
        }

        ST7735_DrawChar16(curX, y, ch, pointColor, backColor);
        curX += 8U;
        pStr++;
    }
}

static void ST7735_AsyncWritePixel(uint8_t *pBuf,
                                   uint16_t width,
                                   uint16_t x,
                                   uint16_t y,
                                   uint16_t color)
{
    uint32_t index;

    index = ((uint32_t)y * width + x) * 2U;
    pBuf[index] = (uint8_t)(color >> 8);
    pBuf[index + 1U] = (uint8_t)(color & 0xFFU);
}

static void ST7735_AsyncRenderChar(uint8_t *pBuf,
                                   uint16_t width,
                                   uint16_t x,
                                   char ch,
                                   uint16_t pointColor)
{
    uint8_t line;
    uint8_t bit;
    uint16_t index;

    if ((ch < ' ') || (ch > '~'))
    {
        ch = '?';
    }

    index = (uint16_t)(ch - ' ') * ST7735_FONT_HEIGHT;
    for (line = 0U; line < ST7735_FONT_HEIGHT; line++)
    {
        for (bit = 0U; bit < ST7735_FONT_WIDTH; bit++)
        {
            if ((asc16[index + line] & (0x80U >> bit)) != 0U)
            {
                ST7735_AsyncWritePixel(pBuf,
                                       width,
                                       (uint16_t)(x + bit),
                                       line,
                                       pointColor);
            }
        }
    }
}

static uint8_t ST7735_AsyncPrepareLine(uint8_t bufIndex)
{
    uint8_t *pBuf;
    uint16_t pixelIndex;
    uint16_t fillPixels;
    uint16_t curX;
    uint16_t lineWidth;
    uint16_t scanIndex;
    uint8_t hi;
    uint8_t lo;
    char ch;

    if ((g_st7735AsyncText.taskActive == 0U) ||
        (g_st7735AsyncText.nextY >= ST7735_HEIGHT))
    {
        return 0U;
    }

    pBuf = g_st7735AsyncText.buf[bufIndex];
    hi = (uint8_t)(g_st7735AsyncText.backColor >> 8);
    lo = (uint8_t)(g_st7735AsyncText.backColor & 0xFFU);

    curX = g_st7735AsyncText.startX;
    scanIndex = g_st7735AsyncText.textIndex;
    while (g_st7735AsyncText.text[scanIndex] != '\0')
    {
        ch = g_st7735AsyncText.text[scanIndex];
        if (ch == '\r')
        {
            scanIndex++;
            continue;
        }

        if (ch == '\n')
        {
            break;
        }

        if ((curX + ST7735_FONT_WIDTH) > ST7735_WIDTH)
        {
            break;
        }

        curX += ST7735_FONT_WIDTH;
        scanIndex++;
    }

    lineWidth = (uint16_t)(curX - g_st7735AsyncText.startX);
    if (lineWidth == 0U)
    {
        while (g_st7735AsyncText.text[g_st7735AsyncText.textIndex] == '\r')
        {
            g_st7735AsyncText.textIndex++;
        }

        if (g_st7735AsyncText.text[g_st7735AsyncText.textIndex] == '\n')
        {
            g_st7735AsyncText.textIndex++;
            g_st7735AsyncText.nextY =
                (uint16_t)(g_st7735AsyncText.nextY + ST7735_FONT_HEIGHT);
            return ST7735_AsyncPrepareLine(bufIndex);
        }

        return 0U;
    }

    fillPixels = (uint16_t)(lineWidth * ST7735_FONT_HEIGHT);
    for (pixelIndex = 0U; pixelIndex < fillPixels; pixelIndex++)
    {
        pBuf[pixelIndex * 2U] = hi;
        pBuf[pixelIndex * 2U + 1U] = lo;
    }

    curX = 0U;
    while (g_st7735AsyncText.text[g_st7735AsyncText.textIndex] != '\0')
    {
        ch = g_st7735AsyncText.text[g_st7735AsyncText.textIndex];
        if (ch == '\r')
        {
            g_st7735AsyncText.textIndex++;
            continue;
        }

        if (ch == '\n')
        {
            g_st7735AsyncText.textIndex++;
            break;
        }

        if ((curX + ST7735_FONT_WIDTH) > ST7735_WIDTH)
        {
            break;
        }

        ST7735_AsyncRenderChar(pBuf,
                               lineWidth,
                               curX,
                               ch,
                               g_st7735AsyncText.pointColor);
        curX += ST7735_FONT_WIDTH;
        g_st7735AsyncText.textIndex++;
    }

    g_st7735AsyncText.bufX[bufIndex] = g_st7735AsyncText.startX;
    g_st7735AsyncText.bufWidth[bufIndex] = lineWidth;
    g_st7735AsyncText.dmaX = g_st7735AsyncText.startX;
    g_st7735AsyncText.dmaY = g_st7735AsyncText.nextY;
    g_st7735AsyncText.dmaWidth = lineWidth;
    g_st7735AsyncText.nextY =
        (uint16_t)(g_st7735AsyncText.nextY + ST7735_FONT_HEIGHT);

    return 1U;
}

static uint8_t ST7735_AsyncStartDma(uint8_t bufIndex)
{
    HAL_StatusTypeDef ret;
    uint16_t dataLen;

    g_st7735AsyncText.dmaX = g_st7735AsyncText.bufX[bufIndex];
    g_st7735AsyncText.dmaWidth = g_st7735AsyncText.bufWidth[bufIndex];

    ST7735_SetWindow(g_st7735AsyncText.dmaX,
                     g_st7735AsyncText.dmaY,
                     (uint16_t)(g_st7735AsyncText.dmaX +
                                g_st7735AsyncText.dmaWidth - 1U),
                     (uint16_t)(g_st7735AsyncText.dmaY +
                                ST7735_FONT_HEIGHT - 1U));

    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
    ST7735_Select();

    dataLen = (uint16_t)(g_st7735AsyncText.dmaWidth *
                         ST7735_FONT_HEIGHT * 2U);
    ret = HAL_SPI_Transmit_DMA(&hspi2,
                               g_st7735AsyncText.buf[bufIndex],
                               dataLen);
    if (ret != HAL_OK)
    {
        ST7735_Unselect();
        g_st7735AsyncText.dmaBusy = 0U;
        g_st7735AsyncText.taskActive = 0U;
        g_st7735AsyncText.preparedValid = 0U;
        return 0U;
    }

    g_st7735AsyncText.activeBuf = bufIndex;
    g_st7735AsyncText.dmaBusy = 1U;
    return 1U;
}

static uint8_t ST7735_StartQueuedText(void)
{
    ST7735_TEXT_REQ_S *pReq;
    uint8_t started;

    if (g_st7735TextCount == 0U)
    {
        return 0U;
    }

    pReq = &g_st7735TextQueue[g_st7735TextHead];
    g_st7735TextHead = (uint8_t)((g_st7735TextHead + 1U) % ST7735_TEXT_QUEUE_SIZE);
    g_st7735TextCount--;

    ST7735_CopyText(g_st7735AsyncText.text, pReq->text);
    g_st7735AsyncText.startX = pReq->x;
    g_st7735AsyncText.nextY = pReq->y;
    g_st7735AsyncText.pointColor = pReq->pointColor;
    g_st7735AsyncText.backColor = pReq->backColor;
    g_st7735AsyncText.textIndex = 0U;
    g_st7735AsyncText.preparedValid = 0U;
    g_st7735AsyncText.taskActive = 1U;

    if (ST7735_AsyncPrepareLine(0U) == 0U)
    {
        g_st7735AsyncText.taskActive = 0U;
        return 0U;
    }

    started = ST7735_AsyncStartDma(0U);
    if (started == 0U)
    {
        return 0U;
    }

    if (ST7735_AsyncPrepareLine(1U) != 0U)
    {
        g_st7735AsyncText.preparedBuf = 1U;
        g_st7735AsyncText.preparedValid = 1U;
    }

    return 1U;
}

static void ST7735_CopyText(char *pDst, const char *pSrc)
{
    uint16_t i;

    if ((pDst == NULL) || (pSrc == NULL))
    {
        return;
    }

    for (i = 0U; i < ST7735_TEXT_MAX_CHARS; i++)
    {
        pDst[i] = pSrc[i];
        if (pSrc[i] == '\0')
        {
            return;
        }
    }

    pDst[ST7735_TEXT_MAX_CHARS] = '\0';
}

uint8_t ST7735_DrawString16Dma(uint16_t x,
                               uint16_t y,
                               const char *pStr,
                               uint16_t pointColor,
                               uint16_t backColor)
{
    ST7735_TEXT_REQ_S *pReq;

    if ((pStr == NULL) || (pStr[0] == '\0') ||
        (x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
    {
        return 0U;
    }

    if (g_st7735TextCount >= ST7735_TEXT_QUEUE_SIZE)
    {
        return 0U;
    }

    pReq = &g_st7735TextQueue[g_st7735TextTail];
    ST7735_CopyText(pReq->text, pStr);
    pReq->x = x;
    pReq->y = y;
    pReq->pointColor = pointColor;
    pReq->backColor = backColor;
    g_st7735TextTail = (uint8_t)((g_st7735TextTail + 1U) % ST7735_TEXT_QUEUE_SIZE);
    g_st7735TextCount++;

    if (g_st7735AsyncText.taskActive == 0U)
    {
        ST7735_WaitIdle();
        (void)ST7735_StartQueuedText();
    }

    return 1U;
}

uint8_t ST7735_IsBusy(void)
{
    return g_st7735AsyncText.taskActive;
}

void ST7735_Process(void)
{
    if ((g_st7735AsyncText.taskActive == 0U) &&
        (g_st7735TextCount > 0U))
    {
        ST7735_WaitIdle();
        (void)ST7735_StartQueuedText();
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    uint8_t nextBuf;

    if ((hspi != &hspi2) || (g_st7735AsyncText.dmaBusy == 0U))
    {
        return;
    }

    ST7735_Unselect();
    g_st7735AsyncText.dmaBusy = 0U;

    if (g_st7735AsyncText.preparedValid == 0U)
    {
        g_st7735AsyncText.taskActive = 0U;
        (void)ST7735_StartQueuedText();
        return;
    }

    nextBuf = g_st7735AsyncText.preparedBuf;
    g_st7735AsyncText.preparedValid = 0U;
    if (ST7735_AsyncStartDma(nextBuf) == 0U)
    {
        return;
    }

    nextBuf = (uint8_t)(1U - nextBuf);
    if (ST7735_AsyncPrepareLine(nextBuf) != 0U)
    {
        g_st7735AsyncText.preparedBuf = nextBuf;
        g_st7735AsyncText.preparedValid = 1U;
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
    if ((hspi != &hspi2) || (g_st7735AsyncText.taskActive == 0U))
    {
        return;
    }

    ST7735_Unselect();
    g_st7735AsyncText.preparedValid = 0U;
    g_st7735AsyncText.dmaBusy = 0U;
    g_st7735AsyncText.taskActive = 0U;
    (void)ST7735_StartQueuedText();
}
