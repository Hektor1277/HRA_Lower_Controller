#include "dma.h"

/* --- USART2  RX  (数据口，Circular) -------------------*/
DMA_HandleTypeDef hdma_usart2_rx;
void DMA_USART2_RX_Init(UART_HandleTypeDef *huart)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX; // **H7 新写法**
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart2_rx);

    __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/* --- USART1  TX  (调试口，Normal) ---------------------*/
DMA_HandleTypeDef hdma_usart1_tx;
void DMA_USART1_TX_Init(UART_HandleTypeDef *huart)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_usart1_tx.Instance = DMA1_Stream7;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL; // 1 报文发一次
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart1_tx);

    __HAL_LINKDMA(huart, hdmatx, hdma_usart1_tx);

    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}
