#ifndef __UART_MULTI_STM32_H__
#define __UART_MULTI_STM32_H__

#include "uart_multi.h"
#include "usart.h"

#ifdef __cplusplus
extern "C" {
#endif

UartMultiStatus UartMultiStm32_InitChannel(UartMultiChannelId id,
                                           UART_HandleTypeDef *huart);
UartMultiChannelId UartMultiStm32_GetIdByHandle(UART_HandleTypeDef *huart);
void UartMultiStm32_UartIrqHandler(UartMultiChannelId id);
void UartMultiStm32_RxDmaIrqHandler(UartMultiChannelId id);
void UartMultiStm32_TxDmaIrqHandler(UartMultiChannelId id);
void UartMultiStm32_UartIrqHandlerByHandle(UART_HandleTypeDef *huart);
void UartMultiStm32_RxDmaIrqHandlerByHandle(UART_HandleTypeDef *huart);
void UartMultiStm32_TxDmaIrqHandlerByHandle(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
