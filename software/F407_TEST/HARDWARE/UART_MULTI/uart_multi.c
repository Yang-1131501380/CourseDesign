#include "uart_multi.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static UartMultiChannel g_uart_channels[UART_MULTI_CH_COUNT];

static void UartMulti_NotifyRx(UartMultiChannelId id,
                               UartMultiChannel *channel,
                               uint8_t in_isr)
{
    if (channel->rx_callback != NULL) {
        channel->rx_callback(id, channel->rx_callback_context, in_isr);
    }
}

static uint8_t UartMulti_TryAcquireTx(UartMultiChannel *channel)
{
#if (KFIFO_USE_ATOMIC != 0)
    rt_atomic_t expected = 0;

    return rt_atomic_compare_exchange_strong(&channel->tx_busy, &expected, 1U);
#else
    if (channel->tx_busy != 0U) {
        return 0U;
    }

    channel->tx_busy = 1U;
    return 1U;
#endif
}

static void UartMulti_ReleaseTx(UartMultiChannel *channel)
{
#if (KFIFO_USE_ATOMIC != 0)
    rt_atomic_store(&channel->tx_busy, 0U);
#else
    channel->tx_busy = 0U;
#endif
}

static UartMultiStatus UartMulti_StartRx(UartMultiChannel *channel)
{
    if ((channel == NULL) || (channel->ops == NULL) ||
        (channel->ops->start_rx == NULL)) {
        return UART_MULTI_ERROR;
    }

    channel->last_rx_pos = 0U;
    return channel->ops->start_rx(channel->platform_context,
                                  channel->rx_dma_buffer,
                                  channel->rx_dma_size);
}

static UartMultiStatus UartMulti_KickTx(UartMultiChannel *channel)
{
    uint32_t tx_len;
    UartMultiStatus status;

    if ((channel == NULL) || (channel->ops == NULL) ||
        (channel->ops->start_tx == NULL)) {
        return UART_MULTI_ERROR;
    }

    if (UartMulti_TryAcquireTx(channel) == 0U) {
        return UART_MULTI_OK;
    }

    tx_len = kfifo_out(&channel->tx_fifo, channel->tx_dma_buffer,
                       channel->tx_dma_size);
    if (tx_len == 0U) {
        UartMulti_ReleaseTx(channel);
        return UART_MULTI_OK;
    }

    status = channel->ops->start_tx(channel->platform_context,
                                    channel->tx_dma_buffer, (uint16_t)tx_len);
    if (status != UART_MULTI_OK) {
        UartMulti_ReleaseTx(channel);
    }

    return status;
}

UartMultiStatus UartMulti_InitChannel(UartMultiChannelId id,
                                      const UartMultiInitConfig *config)
{
    UartMultiChannel *channel;

    if ((id >= UART_MULTI_CH_COUNT) || (config == NULL) ||
        (config->ops == NULL) || (config->rx_dma_buffer == NULL) ||
        (config->rx_fifo_buffer == NULL) || (config->tx_dma_buffer == NULL) ||
        (config->tx_fifo_buffer == NULL)) {
        return UART_MULTI_ERROR;
    }

    channel = &g_uart_channels[id];
    memset(channel, 0, sizeof(*channel));

    channel->platform_context = config->platform_context;
    channel->ops = config->ops;
    channel->rx_dma_buffer = config->rx_dma_buffer;
    channel->rx_dma_size = config->rx_dma_size;
    channel->tx_dma_buffer = config->tx_dma_buffer;
    channel->tx_dma_size = config->tx_dma_size;

    kfifo_init(&channel->rx_fifo, config->rx_fifo_buffer,
               config->rx_fifo_seq_buffer, config->rx_fifo_size);
    kfifo_init(&channel->tx_fifo, config->tx_fifo_buffer,
               config->tx_fifo_seq_buffer, config->tx_fifo_size);

    return UartMulti_StartRx(channel);
}

UartMultiChannel *UartMulti_GetChannel(UartMultiChannelId id)
{
    if (id >= UART_MULTI_CH_COUNT) {
        return NULL;
    }

    return &g_uart_channels[id];
}

UartMultiStatus UartMulti_SetConfig(UartMultiChannelId id,
                                    const UartMultiConfig *config)
{
    UartMultiChannel *channel = UartMulti_GetChannel(id);
    uint8_t tx_was_busy;

    if ((channel == NULL) || (channel->ops == NULL) ||
        (channel->ops->set_config == NULL) || (config == NULL)) {
        return UART_MULTI_ERROR;
    }

    tx_was_busy = channel->tx_busy != 0U;

    if (channel->ops->stop_rx != NULL) {
        channel->ops->stop_rx(channel->platform_context);
    }

    if (channel->ops->stop_tx != NULL) {
        channel->ops->stop_tx(channel->platform_context);
    }

    if (channel->ops->set_config(channel->platform_context, config) !=
        UART_MULTI_OK) {
        return UART_MULTI_ERROR;
    }

    if (UartMulti_StartRx(channel) != UART_MULTI_OK) {
        return UART_MULTI_ERROR;
    }

    if (tx_was_busy != 0U) {
        UartMulti_ReleaseTx(channel);
        (void)UartMulti_KickTx(channel);
    }

    return UART_MULTI_OK;
}

uint16_t UartMulti_Read(UartMultiChannelId id, uint8_t *buffer, uint16_t len)
{
    UartMultiChannel *channel = UartMulti_GetChannel(id);

    if ((channel == NULL) || (buffer == NULL) || (len == 0U)) {
        return 0U;
    }

    return (uint16_t)kfifo_out(&channel->rx_fifo, buffer, len);
}

uint16_t UartMulti_Available(UartMultiChannelId id)
{
    UartMultiChannel *channel = UartMulti_GetChannel(id);

    if (channel == NULL) {
        return 0U;
    }

    return (uint16_t)kfifo_len(&channel->rx_fifo);
}

UartMultiStatus UartMulti_Write(UartMultiChannelId id, const uint8_t *data,
                                uint16_t len)
{
    UartMultiChannel *channel = UartMulti_GetChannel(id);
    uint32_t written;

    if ((channel == NULL) || (data == NULL) || (len == 0U)) {
        return UART_MULTI_ERROR;
    }

    written = kfifo_in(&channel->tx_fifo, data, len);
    if (written != len) {
        channel->tx_fifo_overflow_count++;
        return UART_MULTI_BUSY;
    }

    return UartMulti_KickTx(channel);
}

UartMultiStatus UartMulti_WriteString(UartMultiChannelId id, const char *str)
{
    if (str == NULL) {
        return UART_MULTI_ERROR;
    }

    return UartMulti_Write(id, (const uint8_t *)str, (uint16_t)strlen(str));
}

int UartMulti_Printf(UartMultiChannelId id, const char *format, ...)
{
    va_list args;
    char buffer[256];
    int len;

    if ((format == NULL) || (id >= UART_MULTI_CH_COUNT)) {
        return -1;
    }

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if ((len <= 0) || (len >= (int)sizeof(buffer))) {
        return -1;
    }

    return (UartMulti_Write(id, (const uint8_t *)buffer, (uint16_t)len) ==
            UART_MULTI_OK) ? len : -1;
}

void UartMulti_SetRxCallback(UartMultiChannelId id,
                             UartMultiRxCallback callback, void *context)
{
    UartMultiChannel *channel = UartMulti_GetChannel(id);

    if (channel == NULL) {
        return;
    }

    channel->rx_callback = callback;
    channel->rx_callback_context = context;
}

void UartMulti_OnRxEvent(UartMultiChannelId id, uint16_t curr_pos,
                         uint8_t in_isr)
{
    UartMultiChannel *channel;
    uint16_t last_pos;
    uint32_t copied;
    uint8_t has_rx_event = 0U;

    if (id >= UART_MULTI_CH_COUNT) {
        return;
    }

    channel = &g_uart_channels[id];
    last_pos = channel->last_rx_pos;
    if (curr_pos == last_pos) {
        return;
    }

    if (curr_pos > last_pos) {
        copied = kfifo_in(&channel->rx_fifo,
                          &channel->rx_dma_buffer[last_pos],
                          (uint32_t)(curr_pos - last_pos));
        if (copied != (uint32_t)(curr_pos - last_pos)) {
            channel->rx_fifo_overflow_count++;
        }
        has_rx_event = 1U;
    } else {
        copied = kfifo_in(&channel->rx_fifo,
                          &channel->rx_dma_buffer[last_pos],
                          (uint32_t)(channel->rx_dma_size - last_pos));
        if (copied != (uint32_t)(channel->rx_dma_size - last_pos)) {
            channel->rx_fifo_overflow_count++;
        }

        copied = kfifo_in(&channel->rx_fifo, &channel->rx_dma_buffer[0],
                          curr_pos);
        if (copied != curr_pos) {
            channel->rx_fifo_overflow_count++;
        }
        has_rx_event = 1U;
    }

    channel->last_rx_pos = curr_pos;
    if (has_rx_event != 0U) {
        UartMulti_NotifyRx(id, channel, in_isr);
    }
}

void UartMulti_OnTxDone(UartMultiChannelId id)
{
    UartMultiChannel *channel;

    if (id >= UART_MULTI_CH_COUNT) {
        return;
    }

    channel = &g_uart_channels[id];
    UartMulti_ReleaseTx(channel);
    (void)UartMulti_KickTx(channel);
}

void UartMulti_OnError(UartMultiChannelId id)
{
    UartMultiChannel *channel;

    if (id >= UART_MULTI_CH_COUNT) {
        return;
    }

    channel = &g_uart_channels[id];
    channel->uart_error_count++;
    (void)UartMulti_StartRx(channel);
}

uint32_t UartMulti_GetRxOverflowCount(UartMultiChannelId id)
{
    UartMultiChannel *channel = UartMulti_GetChannel(id);

    if (channel == NULL) {
        return 0U;
    }

    return channel->rx_fifo_overflow_count;
}

uint32_t UartMulti_GetTxOverflowCount(UartMultiChannelId id)
{
    UartMultiChannel *channel = UartMulti_GetChannel(id);

    if (channel == NULL) {
        return 0U;
    }

    return channel->tx_fifo_overflow_count;
}

uint32_t UartMulti_GetUartErrorCount(UartMultiChannelId id)
{
    UartMultiChannel *channel = UartMulti_GetChannel(id);

    if (channel == NULL) {
        return 0U;
    }

    return channel->uart_error_count;
}
