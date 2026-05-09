#include "uart_multi_stm32.h"
#include <string.h>

/* CH1 / USART1: 日志口，发送为主，保留少量接收能力 */
#define UART1_RX_DMA_SIZE   64U
#define UART1_RX_FIFO_SIZE  128U
#define UART1_TX_DMA_SIZE   64U
#define UART1_TX_FIFO_SIZE  128U

/* CH2 / USART2: 步进电机控制口，命令短小，强调低延迟与确定性 */
#define UART2_RX_DMA_SIZE   128U
#define UART2_RX_FIFO_SIZE  256U
#define UART2_TX_DMA_SIZE   128U
#define UART2_TX_FIFO_SIZE  256U

/* CH6 / USART6: K230 通信口，双向通信，提升收发缓冲余量 */
#define UART6_RX_DMA_SIZE   128U
#define UART6_RX_FIFO_SIZE  256U
#define UART6_TX_DMA_SIZE   128U
#define UART6_TX_FIFO_SIZE  256U

typedef struct {
    UART_HandleTypeDef *huart;
} UartMultiStm32Context;

static UartMultiStm32Context g_uart1_context;
static UartMultiStm32Context g_uart2_context;
static UartMultiStm32Context g_uart6_context;

static uint8_t g_uart1_rx_dma[UART1_RX_DMA_SIZE];
static uint8_t g_uart2_rx_dma[UART2_RX_DMA_SIZE];
static uint8_t g_uart6_rx_dma[UART6_RX_DMA_SIZE];

static uint8_t g_uart1_rx_fifo[UART1_RX_FIFO_SIZE];
static uint8_t g_uart2_rx_fifo[UART2_RX_FIFO_SIZE];
static uint8_t g_uart6_rx_fifo[UART6_RX_FIFO_SIZE];
#if (KFIFO_USE_ATOMIC != 0)
static rt_atomic_t g_uart1_rx_fifo_seq[UART1_RX_FIFO_SIZE];
static rt_atomic_t g_uart2_rx_fifo_seq[UART2_RX_FIFO_SIZE];
static rt_atomic_t g_uart6_rx_fifo_seq[UART6_RX_FIFO_SIZE];
#endif

static uint8_t g_uart1_tx_dma[UART1_TX_DMA_SIZE];
static uint8_t g_uart2_tx_dma[UART2_TX_DMA_SIZE];
static uint8_t g_uart6_tx_dma[UART6_TX_DMA_SIZE];

static uint8_t g_uart1_tx_fifo[UART1_TX_FIFO_SIZE];
static uint8_t g_uart2_tx_fifo[UART2_TX_FIFO_SIZE];
static uint8_t g_uart6_tx_fifo[UART6_TX_FIFO_SIZE];
#if (KFIFO_USE_ATOMIC != 0)
static rt_atomic_t g_uart1_tx_fifo_seq[UART1_TX_FIFO_SIZE];
static rt_atomic_t g_uart2_tx_fifo_seq[UART2_TX_FIFO_SIZE];
static rt_atomic_t g_uart6_tx_fifo_seq[UART6_TX_FIFO_SIZE];
#endif

static UartMultiStm32Context *UartMultiStm32_GetContext(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return &g_uart1_context;
        case UART_MULTI_CH2:
            return &g_uart2_context;
        case UART_MULTI_CH6:
            return &g_uart6_context;
        default:
            return NULL;
    }
}

static uint8_t *UartMultiStm32_GetRxDmaBuffer(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return g_uart1_rx_dma;
        case UART_MULTI_CH2:
            return g_uart2_rx_dma;
        case UART_MULTI_CH6:
            return g_uart6_rx_dma;
        default:
            return NULL;
    }
}

static uint16_t UartMultiStm32_GetRxDmaSize(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return UART1_RX_DMA_SIZE;
        case UART_MULTI_CH2:
            return UART2_RX_DMA_SIZE;
        case UART_MULTI_CH6:
            return UART6_RX_DMA_SIZE;
        default:
            return 0U;
    }
}

static uint8_t *UartMultiStm32_GetRxFifoBuffer(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return g_uart1_rx_fifo;
        case UART_MULTI_CH2:
            return g_uart2_rx_fifo;
        case UART_MULTI_CH6:
            return g_uart6_rx_fifo;
        default:
            return NULL;
    }
}

static uint32_t UartMultiStm32_GetRxFifoSize(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return UART1_RX_FIFO_SIZE;
        case UART_MULTI_CH2:
            return UART2_RX_FIFO_SIZE;
        case UART_MULTI_CH6:
            return UART6_RX_FIFO_SIZE;
        default:
            return 0U;
    }
}

static uint8_t *UartMultiStm32_GetTxDmaBuffer(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return g_uart1_tx_dma;
        case UART_MULTI_CH2:
            return g_uart2_tx_dma;
        case UART_MULTI_CH6:
            return g_uart6_tx_dma;
        default:
            return NULL;
    }
}

static uint16_t UartMultiStm32_GetTxDmaSize(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return UART1_TX_DMA_SIZE;
        case UART_MULTI_CH2:
            return UART2_TX_DMA_SIZE;
        case UART_MULTI_CH6:
            return UART6_TX_DMA_SIZE;
        default:
            return 0U;
    }
}

static uint8_t *UartMultiStm32_GetTxFifoBuffer(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return g_uart1_tx_fifo;
        case UART_MULTI_CH2:
            return g_uart2_tx_fifo;
        case UART_MULTI_CH6:
            return g_uart6_tx_fifo;
        default:
            return NULL;
    }
}

static uint32_t UartMultiStm32_GetTxFifoSize(UartMultiChannelId id)
{
    switch (id) {
        case UART_MULTI_CH1:
            return UART1_TX_FIFO_SIZE;
        case UART_MULTI_CH2:
            return UART2_TX_FIFO_SIZE;
        case UART_MULTI_CH6:
            return UART6_TX_FIFO_SIZE;
        default:
            return 0U;
    }
}

static void *UartMultiStm32_GetRxSeqBuffer(UartMultiChannelId id)
{
#if (KFIFO_USE_ATOMIC != 0)
    switch (id) {
        case UART_MULTI_CH1:
            return g_uart1_rx_fifo_seq;
        case UART_MULTI_CH2:
            return g_uart2_rx_fifo_seq;
        case UART_MULTI_CH6:
            return g_uart6_rx_fifo_seq;
        default:
            return NULL;
    }
#else
    (void)id;
    return NULL;
#endif
}

static void *UartMultiStm32_GetTxSeqBuffer(UartMultiChannelId id)
{
#if (KFIFO_USE_ATOMIC != 0)
    switch (id) {
        case UART_MULTI_CH1:
            return g_uart1_tx_fifo_seq;
        case UART_MULTI_CH2:
            return g_uart2_tx_fifo_seq;
        case UART_MULTI_CH6:
            return g_uart6_tx_fifo_seq;
        default:
            return NULL;
    }
#else
    (void)id;
    return NULL;
#endif
}

static void UartMultiStm32_ClearDmaFlags(DMA_HandleTypeDef *hdma)
{
    if (hdma == NULL) {
        return;
    }

    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_DME_FLAG_INDEX(hdma));
    __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_FE_FLAG_INDEX(hdma));
}

static void UartMultiStm32_DisableDmaStream(DMA_Stream_TypeDef *stream)
{
    if (stream == NULL) {
        return;
    }

    CLEAR_BIT(stream->CR, DMA_SxCR_EN);
    while ((stream->CR & DMA_SxCR_EN) != 0U) {
    }
}

static void UartMultiStm32_ClearUartIdleFlag(USART_TypeDef *instance)
{
    volatile uint32_t tmp;

    tmp = instance->SR;
    tmp = instance->DR;
    (void)tmp;
}

static UartMultiStatus UartMultiStm32_StartRx(void *context, uint8_t *buffer,
                                              uint16_t size)
{
    UartMultiStm32Context *stm32_context = context;
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma;

    if ((stm32_context == NULL) || (stm32_context->huart == NULL) ||
        (buffer == NULL) || (size == 0U)) {
        return UART_MULTI_ERROR;
    }

    huart = stm32_context->huart;
    if (huart->hdmarx == NULL) {
        return UART_MULTI_ERROR;
    }

    hdma = huart->hdmarx;
    UartMultiStm32_DisableDmaStream(hdma->Instance);
    UartMultiStm32_ClearDmaFlags(hdma);

    hdma->Instance->PAR = (uint32_t)&huart->Instance->DR;
    hdma->Instance->M0AR = (uint32_t)buffer;
    hdma->Instance->NDTR = size;
    MODIFY_REG(hdma->Instance->CR,
               DMA_SxCR_DIR | DMA_SxCR_CIRC | DMA_SxCR_PINC |
                   DMA_SxCR_MINC | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
                   DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE,
               DMA_SxCR_CIRC | DMA_SxCR_MINC | DMA_SxCR_TCIE |
                   DMA_SxCR_HTIE | DMA_SxCR_TEIE);

    SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart->Instance->CR1, USART_CR1_IDLEIE);
    UartMultiStm32_ClearUartIdleFlag(huart->Instance);
    SET_BIT(hdma->Instance->CR, DMA_SxCR_EN);

    return UART_MULTI_OK;
}

static UartMultiStatus UartMultiStm32_StartTx(void *context, uint8_t *buffer,
                                              uint16_t size)
{
    UartMultiStm32Context *stm32_context = context;
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma;

    if ((stm32_context == NULL) || (stm32_context->huart == NULL) ||
        (buffer == NULL) || (size == 0U)) {
        return UART_MULTI_ERROR;
    }

    huart = stm32_context->huart;
    if (huart->hdmatx == NULL) {
        return UART_MULTI_ERROR;
    }

    hdma = huart->hdmatx;
    UartMultiStm32_DisableDmaStream(hdma->Instance);
    UartMultiStm32_ClearDmaFlags(hdma);
    hdma->Instance->PAR = (uint32_t)&huart->Instance->DR;
    hdma->Instance->M0AR = (uint32_t)buffer;
    hdma->Instance->NDTR = size;
    MODIFY_REG(hdma->Instance->CR,
               DMA_SxCR_DIR | DMA_SxCR_CIRC | DMA_SxCR_PINC |
                   DMA_SxCR_MINC | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE |
                   DMA_SxCR_TCIE | DMA_SxCR_HTIE | DMA_SxCR_TEIE,
               DMA_SxCR_DIR_0 | DMA_SxCR_MINC | DMA_SxCR_TCIE |
                   DMA_SxCR_TEIE);
    SET_BIT(huart->Instance->CR3, USART_CR3_DMAT);
    SET_BIT(hdma->Instance->CR, DMA_SxCR_EN);

    return UART_MULTI_OK;
}

static void UartMultiStm32_StopRx(void *context)
{
    UartMultiStm32Context *stm32_context = context;

    if ((stm32_context == NULL) || (stm32_context->huart == NULL)) {
        return;
    }

    CLEAR_BIT(stm32_context->huart->Instance->CR1, USART_CR1_IDLEIE);
    CLEAR_BIT(stm32_context->huart->Instance->CR3, USART_CR3_DMAR);
    if (stm32_context->huart->hdmarx != NULL) {
        UartMultiStm32_DisableDmaStream(stm32_context->huart->hdmarx->Instance);
        UartMultiStm32_ClearDmaFlags(stm32_context->huart->hdmarx);
    }
}

static void UartMultiStm32_StopTx(void *context)
{
    UartMultiStm32Context *stm32_context = context;

    if ((stm32_context == NULL) || (stm32_context->huart == NULL)) {
        return;
    }

    CLEAR_BIT(stm32_context->huart->Instance->CR3, USART_CR3_DMAT);
    if (stm32_context->huart->hdmatx != NULL) {
        UartMultiStm32_DisableDmaStream(stm32_context->huart->hdmatx->Instance);
        UartMultiStm32_ClearDmaFlags(stm32_context->huart->hdmatx);
    }
}

static UartMultiStatus UartMultiStm32_SetConfig(void *context,
                                                const UartMultiConfig *config)
{
    UartMultiStm32Context *stm32_context = context;
    UART_HandleTypeDef *huart;

    if ((stm32_context == NULL) || (stm32_context->huart == NULL) ||
        (config == NULL) || (config->data_bits != 8U)) {
        return UART_MULTI_ERROR;
    }

    huart = stm32_context->huart;
    if (HAL_UART_DeInit(huart) != HAL_OK) {
        return UART_MULTI_ERROR;
    }

    huart->Init.BaudRate = config->baudrate;
    switch (config->parity) {
        case UART_MULTI_PARITY_NONE:
            huart->Init.Parity = UART_PARITY_NONE;
            huart->Init.WordLength = UART_WORDLENGTH_8B;
            break;
        case UART_MULTI_PARITY_EVEN:
            huart->Init.Parity = UART_PARITY_EVEN;
            huart->Init.WordLength = UART_WORDLENGTH_9B;
            break;
        case UART_MULTI_PARITY_ODD:
            huart->Init.Parity = UART_PARITY_ODD;
            huart->Init.WordLength = UART_WORDLENGTH_9B;
            break;
        default:
            return UART_MULTI_ERROR;
    }

    switch (config->stop_bits) {
        case 1U:
            huart->Init.StopBits = UART_STOPBITS_1;
            break;
        case 2U:
            huart->Init.StopBits = UART_STOPBITS_2;
            break;
        default:
            return UART_MULTI_ERROR;
    }

    return (HAL_UART_Init(huart) == HAL_OK) ? UART_MULTI_OK :
                                             UART_MULTI_ERROR;
}

static const UartMultiPlatformOps g_uart_multi_stm32_ops = {
    UartMultiStm32_StartRx,
    UartMultiStm32_StartTx,
    UartMultiStm32_StopRx,
    UartMultiStm32_StopTx,
    UartMultiStm32_SetConfig
};

UartMultiStatus UartMultiStm32_InitChannel(UartMultiChannelId id,
                                           UART_HandleTypeDef *huart)
{
    UartMultiStm32Context *context = UartMultiStm32_GetContext(id);
    UartMultiInitConfig config;

    if ((context == NULL) || (huart == NULL)) {
        return UART_MULTI_ERROR;
    }

    memset(&config, 0, sizeof(config));
    context->huart = huart;
    config.platform_context = context;
    config.ops = &g_uart_multi_stm32_ops;
    config.rx_dma_buffer = UartMultiStm32_GetRxDmaBuffer(id);
    config.rx_dma_size = UartMultiStm32_GetRxDmaSize(id);
    config.rx_fifo_buffer = UartMultiStm32_GetRxFifoBuffer(id);
    config.rx_fifo_seq_buffer = UartMultiStm32_GetRxSeqBuffer(id);
    config.rx_fifo_size = UartMultiStm32_GetRxFifoSize(id);
    config.tx_dma_buffer = UartMultiStm32_GetTxDmaBuffer(id);
    config.tx_dma_size = UartMultiStm32_GetTxDmaSize(id);
    config.tx_fifo_buffer = UartMultiStm32_GetTxFifoBuffer(id);
    config.tx_fifo_seq_buffer = UartMultiStm32_GetTxSeqBuffer(id);
    config.tx_fifo_size = UartMultiStm32_GetTxFifoSize(id);

    return UartMulti_InitChannel(id, &config);
}

UartMultiChannelId UartMultiStm32_GetIdByHandle(UART_HandleTypeDef *huart)
{
    uint32_t i;
    UartMultiStm32Context *context;

    for (i = 0U; i < UART_MULTI_CH_COUNT; ++i) {
        context = UartMultiStm32_GetContext((UartMultiChannelId)i);
        if ((context != NULL) && (context->huart == huart)) {
            return (UartMultiChannelId)i;
        }
    }

    return UART_MULTI_CH_COUNT;
}

void UartMultiStm32_UartIrqHandler(UartMultiChannelId id)
{
    UartMultiStm32Context *context = UartMultiStm32_GetContext(id);
    UART_HandleTypeDef *huart;
    uint16_t curr_pos;
    uint32_t sr;

    if ((context == NULL) || (context->huart == NULL)) {
        return;
    }

    huart = context->huart;
    sr = huart->Instance->SR;
    if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U) {
        UartMultiStm32_ClearUartIdleFlag(huart->Instance);
        UartMulti_OnError(id);
        sr = huart->Instance->SR;
    }

    if ((sr & USART_SR_IDLE) != 0U) {
        UartMultiStm32_ClearUartIdleFlag(huart->Instance);
        curr_pos = (uint16_t)(UartMultiStm32_GetRxDmaSize(id) -
                              huart->hdmarx->Instance->NDTR);
        UartMulti_OnRxEvent(id, curr_pos, 1U);
    }
}

void UartMultiStm32_RxDmaIrqHandler(UartMultiChannelId id)
{
    UartMultiStm32Context *context = UartMultiStm32_GetContext(id);
    DMA_HandleTypeDef *hdma;
    uint16_t curr_pos;

    if ((context == NULL) || (context->huart == NULL) ||
        (context->huart->hdmarx == NULL)) {
        return;
    }

    hdma = context->huart->hdmarx;
    if (((hdma->Instance->CR & DMA_SxCR_HTIE) != 0U) &&
        (__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma)) !=
         RESET)) {
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_HT_FLAG_INDEX(hdma));
        curr_pos = (uint16_t)(UartMultiStm32_GetRxDmaSize(id) -
                              hdma->Instance->NDTR);
        UartMulti_OnRxEvent(id, curr_pos, 1U);
    }

    if (((hdma->Instance->CR & DMA_SxCR_TCIE) != 0U) &&
        (__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma)) !=
         RESET)) {
        __HAL_DMA_CLEAR_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma));
        curr_pos = (uint16_t)(UartMultiStm32_GetRxDmaSize(id) -
                              hdma->Instance->NDTR);
        UartMulti_OnRxEvent(id, curr_pos, 1U);
    }

    if (((hdma->Instance->CR & DMA_SxCR_TEIE) != 0U) &&
        (__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma)) !=
         RESET)) {
        UartMultiStm32_ClearDmaFlags(hdma);
        UartMulti_OnError(id);
    }
}

void UartMultiStm32_TxDmaIrqHandler(UartMultiChannelId id)
{
    UartMultiStm32Context *context = UartMultiStm32_GetContext(id);
    DMA_HandleTypeDef *hdma;

    if ((context == NULL) || (context->huart == NULL) ||
        (context->huart->hdmatx == NULL)) {
        return;
    }

    hdma = context->huart->hdmatx;
    if (((hdma->Instance->CR & DMA_SxCR_TCIE) != 0U) &&
        (__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TC_FLAG_INDEX(hdma)) !=
         RESET)) {
        UartMultiStm32_StopTx(context);
        UartMulti_OnTxDone(id);
        return;
    }

    if (((hdma->Instance->CR & DMA_SxCR_TEIE) != 0U) &&
        (__HAL_DMA_GET_FLAG(hdma, __HAL_DMA_GET_TE_FLAG_INDEX(hdma)) !=
         RESET)) {
        UartMultiStm32_StopTx(context);
        UartMulti_OnError(id);
    }
}

void UartMultiStm32_UartIrqHandlerByHandle(UART_HandleTypeDef *huart)
{
    UartMultiChannelId id = UartMultiStm32_GetIdByHandle(huart);

    if (id < UART_MULTI_CH_COUNT) {
        UartMultiStm32_UartIrqHandler(id);
    }
}

void UartMultiStm32_RxDmaIrqHandlerByHandle(UART_HandleTypeDef *huart)
{
    UartMultiChannelId id = UartMultiStm32_GetIdByHandle(huart);

    if (id < UART_MULTI_CH_COUNT) {
        UartMultiStm32_RxDmaIrqHandler(id);
    }
}

void UartMultiStm32_TxDmaIrqHandlerByHandle(UART_HandleTypeDef *huart)
{
    UartMultiChannelId id = UartMultiStm32_GetIdByHandle(huart);

    if (id < UART_MULTI_CH_COUNT) {
        UartMultiStm32_TxDmaIrqHandler(id);
    }
}
