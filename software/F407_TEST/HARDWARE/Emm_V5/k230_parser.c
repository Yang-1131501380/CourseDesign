#include "k230_parser.h"

#include "uart_multi.h"

#include <string.h>

static int k230_parser_hex_value(char ch, uint8_t *pValue)
{
    if (pValue == NULL)
    {
        return -1;
    }

    if ((ch >= '0') && (ch <= '9'))
    {
        *pValue = (uint8_t)(ch - '0');
        return 0;
    }

    if ((ch >= 'A') && (ch <= 'F'))
    {
        *pValue = (uint8_t)(ch - 'A' + 10);
        return 0;
    }

    if ((ch >= 'a') && (ch <= 'f'))
    {
        *pValue = (uint8_t)(ch - 'a' + 10);
        return 0;
    }

    return -1;
}

static int k230_parser_parse_hex_byte(const char *pText, uint8_t *pValue)
{
    uint8_t high;
    uint8_t low;

    if ((pText == NULL) || (pValue == NULL) ||
        (pText[0] == '\0') || (pText[1] == '\0') || (pText[2] != '\0'))
    {
        return -1;
    }

    if ((k230_parser_hex_value(pText[0], &high) != 0) ||
        (k230_parser_hex_value(pText[1], &low) != 0))
    {
        return -1;
    }

    *pValue = (uint8_t)((high << 4) | low);
    return 0;
}

static int k230_parser_parse_i32(const char *pText, int32_t *pValue)
{
    int32_t sign;
    int32_t value;

    if ((pText == NULL) || (pValue == NULL) || (*pText == '\0'))
    {
        return -1;
    }

    sign = 1;
    value = 0;
    if (*pText == '-')
    {
        sign = -1;
        ++pText;
    }

    if (*pText == '\0')
    {
        return -1;
    }

    while (*pText != '\0')
    {
        if ((*pText < '0') || (*pText > '9'))
        {
            return -1;
        }
        value = (value * 10) + (int32_t)(*pText - '0');
        ++pText;
    }

    *pValue = value * sign;
    return 0;
}

static int k230_parser_parse_float(const char *pText, float *pValue)
{
    int32_t sign;
    uint32_t integerPart;
    uint32_t fracPart;
    uint32_t fracScale;
    uint8_t hasDigit;

    if ((pText == NULL) || (pValue == NULL) || (*pText == '\0'))
    {
        return -1;
    }

    sign = 1;
    integerPart = 0U;
    fracPart = 0U;
    fracScale = 1U;
    hasDigit = 0U;
    if (*pText == '-')
    {
        sign = -1;
        ++pText;
    }

    while ((*pText >= '0') && (*pText <= '9'))
    {
        hasDigit = 1U;
        integerPart = (integerPart * 10U) + (uint32_t)(*pText - '0');
        ++pText;
    }

    if (*pText == '.')
    {
        ++pText;
        while ((*pText >= '0') && (*pText <= '9'))
        {
            hasDigit = 1U;
            if (fracScale < 1000000U)
            {
                fracPart = (fracPart * 10U) + (uint32_t)(*pText - '0');
                fracScale *= 10U;
            }
            ++pText;
        }
    }

    if ((hasDigit == 0U) || (*pText != '\0'))
    {
        return -1;
    }

    *pValue = ((float)integerPart + ((float)fracPart / (float)fracScale)) *
              (float)sign;
    return 0;
}

static int k230_parser_next_field(char **ppCursor, char **ppField)
{
    char *pComma;

    if ((ppCursor == NULL) || (ppField == NULL) || (*ppCursor == NULL))
    {
        return -1;
    }

    *ppField = *ppCursor;
    pComma = strchr(*ppCursor, ',');
    if (pComma != NULL)
    {
        *pComma = '\0';
        *ppCursor = pComma + 1;
    }
    else
    {
        *ppCursor = NULL;
    }

    return 0;
}

static int k230_parser_parse_line(K230_PARSER_S *pParser,
                                  char *pLine,
                                  uint32_t nowMs)
{
    char *pStar;
    char *pCursor;
    char *pField;
    char *pToken;
    int32_t value[7];
    float distCm;
    uint8_t sum;
    uint8_t frameSum;
    uint8_t i;

    if ((pParser == NULL) || (pLine == NULL) ||
        (strncmp(pLine, "$K230,", 6U) != 0))
    {
        return -1;
    }

    pStar = strchr(pLine, '*');
    if (pStar == NULL)
    {
        return -1;
    }

    sum = 0U;
    for (pToken = pLine + 1; pToken < pStar; ++pToken)
    {
        sum ^= (uint8_t)(*pToken);
    }

    if ((k230_parser_parse_hex_byte(pStar + 1, &frameSum) != 0) ||
        (sum != frameSum))
    {
        return -1;
    }

    *pStar = '\0';
    pCursor = pLine + 1;
    if ((k230_parser_next_field(&pCursor, &pField) != 0) ||
        (strcmp(pField, "K230") != 0))
    {
        return -1;
    }

    for (i = 0U; i < 5U; ++i)
    {
        if ((k230_parser_next_field(&pCursor, &pField) != 0) ||
            (k230_parser_parse_i32(pField, &value[i]) != 0))
        {
            return -1;
        }
    }

    if ((k230_parser_next_field(&pCursor, &pField) != 0) ||
        (k230_parser_parse_float(pField, &distCm) != 0))
    {
        return -1;
    }

    for (i = 5U; i < 7U; ++i)
    {
        if ((k230_parser_next_field(&pCursor, &pField) != 0) ||
            (k230_parser_parse_i32(pField, &value[i]) != 0))
        {
            return -1;
        }
    }

    if (pCursor != NULL)
    {
        return -1;
    }

    pParser->target.valid = (uint8_t)value[0];
    pParser->target.cx = (int32_t)value[1];
    pParser->target.cy = (int32_t)value[2];
    pParser->target.dx = (int32_t)value[3];
    pParser->target.dy = (int32_t)value[4];
    pParser->target.width = (int32_t)value[5];
    pParser->target.height = (int32_t)value[6];
    pParser->target.dist_cm = distCm;
    pParser->target.tick_ms = nowMs;
    pParser->target.seq++;

    if (pParser->target_cb != NULL)
    {
        pParser->target_cb(&pParser->target, pParser->p_user);
    }

    return 0;
}

void K230Parser_Init(K230_PARSER_S *pParser,
                     UartMultiChannelId uartCh,
                     K230ParserTargetCallback targetCb,
                     K230ParserUartCallback uartCb,
                     void *pUser)
{
    if (pParser == NULL)
    {
        return;
    }

    memset(pParser, 0, sizeof(*pParser));
    pParser->uart_ch = uartCh;
    pParser->target_cb = targetCb;
    pParser->uart_cb = uartCb;
    pParser->p_user = pUser;
}

void K230Parser_Process(K230_PARSER_S *pParser, uint32_t nowMs)
{
    uint8_t data[K230_PARSER_RX_CHUNK];
    uint16_t len;
    uint16_t i;
    uint32_t err;

    if (pParser == NULL)
    {
        return;
    }

    err = UartMulti_GetUartErrorCount(pParser->uart_ch);
    if (err != pParser->last_uart_err)
    {
        pParser->last_uart_err = err;
        pParser->line_len = 0U;
        (void)UartMulti_Read(pParser->uart_ch, data, sizeof(data));
        if (pParser->uart_cb != NULL)
        {
            pParser->uart_cb(err, pParser->p_user);
        }
        return;
    }

    do
    {
        len = UartMulti_Read(pParser->uart_ch, data, sizeof(data));
        for (i = 0U; i < len; ++i)
        {
            if ((data[i] == '\r') || (data[i] == '\n'))
            {
                if (pParser->line_len > 0U)
                {
                    pParser->line[pParser->line_len] = '\0';
                    (void)k230_parser_parse_line(pParser, pParser->line,
                                                 nowMs);
                    pParser->line_len = 0U;
                }
            }
            else if (pParser->line_len < (K230_PARSER_LINE_LEN - 1U))
            {
                pParser->line[pParser->line_len++] = (char)data[i];
            }
            else
            {
                pParser->line_len = 0U;
            }
        }
    } while (len == sizeof(data));
}

void K230Parser_GetTarget(const K230_PARSER_S *pParser,
                          track_target_t *pTarget)
{
    if ((pParser == NULL) || (pTarget == NULL))
    {
        return;
    }

    *pTarget = pParser->target;
}
