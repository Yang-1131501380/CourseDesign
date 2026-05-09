#ifndef __UART_MULTI_H__
#define __UART_MULTI_H__

#include "kfifo.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART_MULTI_OK = 0,
    UART_MULTI_ERROR = 1,
    UART_MULTI_BUSY = 2,
    UART_MULTI_TIMEOUT = 3
} UartMultiStatus;

typedef enum {
    UART_MULTI_CH1 = 0,
    UART_MULTI_CH2,
    UART_MULTI_CH6,
    UART_MULTI_CH_COUNT
} UartMultiChannelId;

typedef enum {
    UART_MULTI_PARITY_NONE = 0,
    UART_MULTI_PARITY_EVEN,
    UART_MULTI_PARITY_ODD
} UartMultiParity;

typedef struct {
    uint32_t baudrate;
    UartMultiParity parity;
    uint8_t data_bits;
    uint8_t stop_bits;
} UartMultiConfig;

typedef void (*UartMultiRxCallback)(UartMultiChannelId id, void *context,
                                    uint8_t in_isr);

typedef struct {
    UartMultiStatus (*start_rx)(void *context, uint8_t *buffer,
                                uint16_t size);
    UartMultiStatus (*start_tx)(void *context, uint8_t *buffer,
                                uint16_t size);
    void (*stop_rx)(void *context);
    void (*stop_tx)(void *context);
    UartMultiStatus (*set_config)(void *context,
                                  const UartMultiConfig *config);
} UartMultiPlatformOps;

typedef struct {
    void *platform_context;
    const UartMultiPlatformOps *ops;
    uint8_t *rx_dma_buffer;
    uint16_t rx_dma_size;
    uint8_t *rx_fifo_buffer;
    void *rx_fifo_seq_buffer;
    uint32_t rx_fifo_size;
    uint8_t *tx_dma_buffer;
    uint16_t tx_dma_size;
    uint8_t *tx_fifo_buffer;
    void *tx_fifo_seq_buffer;
    uint32_t tx_fifo_size;
} UartMultiInitConfig;

typedef struct {
    void *platform_context;
    const UartMultiPlatformOps *ops;
    uint8_t *rx_dma_buffer;
    uint16_t rx_dma_size;
    uint16_t last_rx_pos;
    kfifo_t rx_fifo;
    volatile uint32_t rx_fifo_overflow_count;
    uint8_t *tx_dma_buffer;
    uint16_t tx_dma_size;
    kfifo_t tx_fifo;
#if (KFIFO_USE_ATOMIC != 0)
    rt_atomic_t tx_busy;
#else
    uint8_t tx_busy;
#endif
    volatile uint32_t tx_fifo_overflow_count;
    volatile uint32_t uart_error_count;
    UartMultiRxCallback rx_callback;
    void *rx_callback_context;
} UartMultiChannel;

UartMultiStatus UartMulti_InitChannel(UartMultiChannelId id,
                                      const UartMultiInitConfig *config);
UartMultiChannel *UartMulti_GetChannel(UartMultiChannelId id);
UartMultiStatus UartMulti_SetConfig(UartMultiChannelId id,
                                    const UartMultiConfig *config);

uint16_t UartMulti_Read(UartMultiChannelId id, uint8_t *buffer, uint16_t len);
uint16_t UartMulti_Available(UartMultiChannelId id);
UartMultiStatus UartMulti_Write(UartMultiChannelId id, const uint8_t *data,
                                uint16_t len);
UartMultiStatus UartMulti_WriteString(UartMultiChannelId id, const char *str);
void UartMulti_SetRxCallback(UartMultiChannelId id,
                             UartMultiRxCallback callback, void *context);
uint32_t UartMulti_GetRxOverflowCount(UartMultiChannelId id);
uint32_t UartMulti_GetTxOverflowCount(UartMultiChannelId id);
uint32_t UartMulti_GetUartErrorCount(UartMultiChannelId id);
int UartMulti_Printf(UartMultiChannelId id, const char *format, ...);

void UartMulti_OnRxEvent(UartMultiChannelId id, uint16_t curr_pos,
                         uint8_t in_isr);
void UartMulti_OnTxDone(UartMultiChannelId id);
void UartMulti_OnError(UartMultiChannelId id);

#ifdef __cplusplus
}
#endif

#endif
