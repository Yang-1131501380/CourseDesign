#ifndef __K230_PARSER_H
#define __K230_PARSER_H

#include "uart_multi.h"
#include "emm_v5_msg.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define K230_PARSER_LINE_LEN     80U
#define K230_PARSER_RX_CHUNK     32U

typedef void (*K230ParserTargetCallback)(const track_target_t *pTarget,
                                         void *pUser);
typedef void (*K230ParserUartCallback)(uint32_t err, void *pUser);

typedef struct {
    UartMultiChannelId uart_ch;
    char line[K230_PARSER_LINE_LEN];
    uint16_t line_len;
    uint32_t last_uart_err;
    track_target_t target;
    K230ParserTargetCallback target_cb;
    K230ParserUartCallback uart_cb;
    void *p_user;
} K230_PARSER_S;

void K230Parser_Init(K230_PARSER_S *pParser,
                     UartMultiChannelId uartCh,
                     K230ParserTargetCallback targetCb,
                     K230ParserUartCallback uartCb,
                     void *pUser);
void K230Parser_Process(K230_PARSER_S *pParser, uint32_t nowMs);
void K230Parser_GetTarget(const K230_PARSER_S *pParser,
                          track_target_t *pTarget);

#ifdef __cplusplus
}
#endif

#endif
