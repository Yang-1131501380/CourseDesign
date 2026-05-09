#ifndef __EMM_V5_H
#define __EMM_V5_H

#include "uart_multi.h"
#include <stdint.h>

#define EMM_V5_FRAME_MAX_LEN        16U
#define EMM_V5_PARSER_BUFFER_SIZE   64U

typedef enum {
    S_CPOS = 15
} SysParams_t;

typedef enum {
    EMM_V5_IO_OK = 0,
    EMM_V5_IO_ERROR,
    EMM_V5_IO_BUSY
} EmmV5IoStatus;

typedef enum {
    EMM_V5_PARSE_OK = 0,
    EMM_V5_PARSE_NO_FRAME,
    EMM_V5_PARSE_NEED_MORE_DATA,
    EMM_V5_PARSE_INVALID_FRAME,
    EMM_V5_PARSE_BUFFER_OVERFLOW
} EmmV5ParseResult;

typedef enum {
    EMM_V5_FRAME_TYPE_NONE = 0,
    EMM_V5_FRAME_TYPE_ACK,
    EMM_V5_FRAME_TYPE_ERROR,
    EMM_V5_FRAME_TYPE_REACHED,
    EMM_V5_FRAME_TYPE_SYS_PARAM
} EmmV5FrameType;

typedef enum {
    EMM_V5_STATUS_REACHED = 0x9F
} EmmV5StatusCode;

typedef struct {
    SysParams_t param;
    int32_t value_i32;
} EmmV5SysParamData;

typedef struct {
    uint8_t addr;
    uint8_t command;
    EmmV5FrameType type;
    EmmV5StatusCode status;
    union {
        EmmV5SysParamData sys_param;
    } data;
} EmmV5Frame;

typedef struct {
    UartMultiChannelId channel;
    uint16_t length;
    uint8_t buffer[EMM_V5_PARSER_BUFFER_SIZE];
} EmmV5Parser;

EmmV5IoStatus Emm_V5_WriteRawOn(UartMultiChannelId id,
                                const uint8_t *pData,
                                uint16_t len);
uint16_t Emm_V5_ReadRawFrom(UartMultiChannelId id,
                            uint8_t *pBuffer,
                            uint16_t len);
void Emm_V5_ParserInit(EmmV5Parser *pParser, UartMultiChannelId id);
EmmV5ParseResult Emm_V5_ParserPoll(EmmV5Parser *pParser,
                                   EmmV5Frame *pFrame);
void Emm_V5_Read_Sys_Params(UartMultiChannelId ch,
                            uint8_t addr,
                            SysParams_t s);

#endif
