#ifndef _DMA_INIT_H_
#define _DMA_INIT_H_
#include "sys.h"
#include "stm32h7xx_hal.h"

/* 公开两个句柄，供 usart.c 里 __HAL_LINKDMA */
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

/* 初始化函数，须在 uart_init() 调用一次 */
void DMA_USART2_RX_Init(UART_HandleTypeDef *huart);
void DMA_USART1_TX_Init(UART_HandleTypeDef *huart);

#endif
