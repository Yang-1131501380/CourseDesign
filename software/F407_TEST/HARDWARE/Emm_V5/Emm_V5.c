#include "Emm_V5.h"

#include <string.h>

static int32_t emm_v5_be_i32(const uint8_t *pData)
{
    int32_t value;

    value = (int32_t)(((uint32_t)pData[0] << 24) |
                      ((uint32_t)pData[1] << 16) |
                      ((uint32_t)pData[2] << 8) |
                      (uint32_t)pData[3]);

    return value;
}

static void emm_v5_consume(EmmV5Parser *pParser, uint16_t len)
{
    if (len >= pParser->length)
    {
        pParser->length = 0U;
        return;
    }

    memmove(pParser->buffer, &pParser->buffer[len], pParser->length - len);
    pParser->length = (uint16_t)(pParser->length - len);
}

static uint16_t emm_v5_frame_len(const uint8_t *pData, uint16_t len)
{
    uint16_t i;

    if (len < 4U)
    {
        return 0U;
    }

    if (pData[1] == 0x36U)
    {
        return (len >= 8U) ? 8U : 0U;
    }

    for (i = 3U; i < len; ++i)
    {
        if (pData[i] == 0x6BU)
        {
            return (uint16_t)(i + 1U);
        }
    }

    return 0U;
}

EmmV5IoStatus Emm_V5_WriteRawOn(UartMultiChannelId id,
                                const uint8_t *pData,
                                uint16_t len)
{
    UartMultiStatus status;

    if ((id >= UART_MULTI_CH_COUNT) || (pData == NULL) || (len == 0U))
    {
        return EMM_V5_IO_ERROR;
    }

    status = UartMulti_Write(id, pData, len);
    if (status == UART_MULTI_OK)
    {
        return EMM_V5_IO_OK;
    }

    return (status == UART_MULTI_BUSY) ? EMM_V5_IO_BUSY : EMM_V5_IO_ERROR;
}

uint16_t Emm_V5_ReadRawFrom(UartMultiChannelId id,
                            uint8_t *pBuffer,
                            uint16_t len)
{
    if ((id >= UART_MULTI_CH_COUNT) || (pBuffer == NULL))
    {
        return 0U;
    }

    return UartMulti_Read(id, pBuffer, len);
}

void Emm_V5_ParserInit(EmmV5Parser *pParser, UartMultiChannelId id)
{
    if (pParser == NULL)
    {
        return;
    }

    memset(pParser, 0, sizeof(*pParser));
    pParser->channel = id;
}

EmmV5ParseResult Emm_V5_ParserPoll(EmmV5Parser *pParser,
                                   EmmV5Frame *pFrame)
{
    uint16_t len;
    uint8_t *pData;

    if ((pParser == NULL) || (pFrame == NULL))
    {
        return EMM_V5_PARSE_INVALID_FRAME;
    }

    len = emm_v5_frame_len(pParser->buffer, pParser->length);
    if (len == 0U)
    {
        if (pParser->length >= EMM_V5_PARSER_BUFFER_SIZE)
        {
            pParser->length = 0U;
            return EMM_V5_PARSE_BUFFER_OVERFLOW;
        }

        return EMM_V5_PARSE_NEED_MORE_DATA;
    }

    pData = pParser->buffer;
    memset(pFrame, 0, sizeof(*pFrame));
    pFrame->addr = pData[0];
    pFrame->command = pData[1];
    if (pData[len - 1U] != 0x6BU)
    {
        emm_v5_consume(pParser, 1U);
        return EMM_V5_PARSE_INVALID_FRAME;
    }

    if (pData[1] == 0x00U)
    {
        pFrame->type = EMM_V5_FRAME_TYPE_ERROR;
    }
    else if (pData[1] == 0x36U)
    {
        pFrame->type = EMM_V5_FRAME_TYPE_SYS_PARAM;
        pFrame->data.sys_param.param = S_CPOS;
        pFrame->data.sys_param.value_i32 = emm_v5_be_i32(&pData[3]);
        if (pData[2] != 0U)
        {
            pFrame->data.sys_param.value_i32 =
                -pFrame->data.sys_param.value_i32;
        }
    }
    else
    {
        pFrame->type = (pData[2] == (uint8_t)EMM_V5_STATUS_REACHED) ?
                       EMM_V5_FRAME_TYPE_REACHED : EMM_V5_FRAME_TYPE_ACK;
        pFrame->status = (EmmV5StatusCode)pData[2];
    }

    emm_v5_consume(pParser, len);

    return EMM_V5_PARSE_OK;
}

void Emm_V5_Read_Sys_Params(UartMultiChannelId ch,
                            uint8_t addr,
                            SysParams_t s)
{
    uint8_t cmd[3];

    if (s != S_CPOS)
    {
        return;
    }

    cmd[0] = addr;
    cmd[1] = 0x36U;
    cmd[2] = 0x6BU;
    (void)Emm_V5_WriteRawOn(ch, cmd, sizeof(cmd));
}
