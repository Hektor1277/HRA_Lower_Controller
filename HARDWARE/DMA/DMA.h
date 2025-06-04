#ifndef __DMA_H
#define __DMA_H

#include "stm32h7xx_hal.h"

extern DMA_HandleTypeDef UART1RxDMA_Handler;  // DMA¾ä±ú

void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx);

#endif /* __DMA_H */
