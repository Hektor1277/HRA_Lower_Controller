/* =========================================================
 *  serial.h  ——  双串口 + DMA-RX(IDLE) 驱动（HAL v1.8.x）
 *  ---------------------------------------------------------
 *  USART1 : 调试口（IT RX/TX）
 *  USART2 : 数据口（DMA RX + IDLE，只 RX）
 *  --------------------------------------------------------
 *  Usage:
 *      ▸ Call MX_USART_Init(230400, 230400);          // anywhere after HAL_Init()
 *      ▸ Nothing else required – callbacks & IRQs do the job.
 * =========================================================*/
#ifndef SERIAL_H
#define SERIAL_H

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart_ex.h"
#include "stm32h7xx_hal_def.h"
#include "stm32h7xx_hal_dma.h"
#include "core_cm7.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

//================== 系统模式选择 ==================
// 0 = 调试模式:   系统低频运行，用于系统调试，此时发送全部调试信息
// 1 = 运行模式:   系统高频运行，用于实际控制，仅发送高层信息
#define OPERATING_MODE 1 // 系统运行模式切换标志位, 0=调试模式, 1=运行模式
#define SEND_DETAIL 0    // 调试模式下发送详细调试信息 (!OPERATING_MODE)
#define DEBUG_ECHO 0     // 可选调试：1=回显 USART2 RX 原始字节到调试口；0=完全关闭回显
//=================================================

//================ DMA配置区 ======================
#define USE_DMA 1        /* 0=全部 IT，1=USART2 用 DMA RX */
#define USE_FREERTOS 0   /* 1=在 ISR 中使用 FreeRTOS API */
#define RX2_DMA_SZ 1024u /* 缓冲区扩大至 1024 (32B 对齐)   */
//================================================

//================ 句柄 & 统计量 ================
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
#if USE_DMA
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
#endif
#define TERM_UART huart1 // 调试接口，发送调试信息并接收PC 指令，IT RX + DMA TX */
#define DATA_UART huart2 // 数据接口，接收高频数据帧，DMA RX ONLY   */

extern uint32_t fifo_overflow_count;     // FIFO 满导致的数据丢弃
extern uint32_t dma_fifo_overflow_count; // uart_write_dma 丢弃数据
//================ 句柄 & 统计量  ================

//================ DMA 环形缓冲区（无 Cache） ================
#if USE_DMA
extern ALIGN_32BYTES(__attribute__((section(".dma_nc"))) uint8_t rx2_dma_buf[RX2_DMA_SZ]);
#endif
//================ DMA 环形缓冲区（无 Cache） ================

//================ FIFO（TX for UART1） ================
#define UART1_TXE_ENABLE() __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE)
#define UART1_TXE_DISABLE() __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE)

// 使用临界区保护 FIFO 操作
#define CRITICAL_SECTION_START() __disable_irq()
#define CRITICAL_SECTION_END() __enable_irq()

// FIFO 缓冲区大小
#define TX_FIFO_SZ 8192 // 必须为 2^n
// FIFO 缓冲区指针
extern uint16_t tx_head;
extern uint16_t tx_tail;
// FIFO 缓冲区定义
extern uint8_t tx_fifo[TX_FIFO_SZ];
//================ FIFO（TX for UART1） ================

//================ 函数声明 ================
void MX_USART_Init(uint32_t baud1, uint32_t baud2);
void USART_SendFormatted(UART_HandleTypeDef *huart, const char *format, ...);      // printf-style短文本发送(阻塞)
void USART_SendFormatted_DMA(const char *fmt, ...);                                // 短文本发送(DMA, 非阻塞)
size_t uart_write_dma(UART_HandleTypeDef *huart, const uint8_t *data, size_t len); // 块数据发送(DMA, 非阻塞)
//================ 函数声明 ================

//================ 内联函数 ================
static inline uint16_t fifo_cnt(void)
{
    return (uint16_t)(tx_head - tx_tail);
}

static inline bool fifo_room(uint16_t n)
{
    return fifo_cnt() + n <= TX_FIFO_SZ;
}

/* 单字节写入（调试情况下可能还会被少量使用） */
static inline bool fifo_put(uint8_t c)
{
    CRITICAL_SECTION_START();
    if (fifo_cnt() >= TX_FIFO_SZ)
    {
        CRITICAL_SECTION_END();
        return false;
    }
    tx_fifo[tx_head++ & (TX_FIFO_SZ - 1u)] = c;
    CRITICAL_SECTION_END();
    return true;
}

/* [FIX-3] 一次检查空间 + 批量 memcpy —— 防止 len 次进出临界区 */
static inline bool fifo_put_multi(const uint8_t *src, uint16_t n)
{
    CRITICAL_SECTION_START();
    if (!fifo_room(n))
    {
        CRITICAL_SECTION_END();
        return false;
    }

    uint16_t pos = tx_head & (TX_FIFO_SZ - 1u);
    uint16_t last = TX_FIFO_SZ - pos;
    if (last >= n)
    {
        memcpy(&tx_fifo[pos], src, n);
    }
    else
    {
        memcpy(&tx_fifo[pos], src, last);
        memcpy(&tx_fifo[0], src + last, n - last);
    }
    tx_head += n;
    CRITICAL_SECTION_END();
    return true;
}

//================ 内联函数 ================

#endif /* SERIAL_H */
