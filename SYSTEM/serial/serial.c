/**************************************************************************
 *  serial.c  ——  实现文件
 *  HAL v1.8.x  •  Cortex-M7  •  STM32H7xx
 **************************************************************************/
#include "serial.h"
#include "protocol.h"
#if USE_FREERTOS
#include "FreeRTOS.h"
#endif
#include <stdio.h>
#include <string.h>
#include "stm32h7xx_hal_dma.h" // HAL_DMA_GetCounter 的声明

/* =========================================================
 * 1. 声明区
 * =======================================================*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
#if USE_DMA
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;

// FIFO 溢出计数
uint32_t fifo_overflow_count = 0;
uint32_t dma_fifo_overflow_count = 0;

/* 数据口 DMA 环形缓冲区唯一定义点（32B 对齐） */
ALIGN_32BYTES(__attribute__((section(".dma_nc"))) uint8_t rx2_dma_buf[RX2_DMA_SZ]);
// volatile uint32_t rx2_prev_pos = 0;
#else
static uint8_t rx2_byte; /* IT 模式用 */
#endif

static uint8_t rx1_byte; /* USART1 IT RX */

/* DMA */
/* 放 .dma_nc，避免 Cache 一致性问题 */
static volatile bool dma_inflight = false; // DMA “在飞” 互斥锁，严格禁止并发启动下一条 DMA

static volatile uint16_t prev_ndtr = RX2_DMA_SZ; /* 上一次进入 ISR 时的 NDTR */
static uint16_t dma_tail = 0;                    /* 环形读取尾指针（0~RX2_DMA_SZ-1） */

/* 调试控制台缓冲区 */
#if (OPERATING_MODE == 0)
#define DBG_CMD_MAX_LEN 64
static uint8_t dbg_rx_buf[DBG_CMD_MAX_LEN];
static uint16_t dbg_rx_idx = 0;
static volatile bool dbg_cmd_ready = false;
#endif

/* =========================================================
 * 2. 调试口 TX 环形 FIFO  (2^n 大小)
 * =======================================================*/
/* 调试 FIFO 同样放 non‑cache 区域  */
uint8_t tx_fifo[TX_FIFO_SZ] __attribute__((section(".dma_nc"))) __attribute__((aligned(32)));
uint16_t tx_head = 0, tx_tail = 0;

/* =========================================================
 * 3. GPIO & UART 基础初始化
 * =======================================================*/
static void gpio_init(GPIO_TypeDef *port,
                      uint16_t tx, uint16_t rx, uint8_t af)
{
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = af;

    g.Pin = tx;
    HAL_GPIO_Init(port, &g);
    g.Pin = rx;
    HAL_GPIO_Init(port, &g);
}

static void uart_basic(UART_HandleTypeDef *h,
                       USART_TypeDef *inst, uint32_t baud)
{
    h->Instance = inst;
    h->Init.BaudRate = baud;
    h->Init.WordLength = UART_WORDLENGTH_8B;
    h->Init.StopBits = UART_STOPBITS_1;
    h->Init.Parity = UART_PARITY_NONE;
    h->Init.Mode = UART_MODE_TX_RX;
    h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    h->Init.OverSampling = UART_OVERSAMPLING_16;
    h->Init.FIFOMode = UART_FIFOMODE_DISABLE;
    HAL_UART_Init(h);

    /* 禁用 FIFO 功能 → 与 v1.8 一致 */
    CLEAR_BIT(h->Instance->CR1, USART_CR1_FIFOEN);
}

/* =========================================================
 * 4. 公共初始化接口
 * =======================================================*/
void MX_USART_Init(uint32_t baud1, uint32_t baud2)
{
    /* --- 外设时钟 --*/
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_USART2_CLK_ENABLE();
#if USE_DMA
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();
#endif

    /* --- USART1 (调试口) --- */
    gpio_init(GPIOA, GPIO_PIN_9, GPIO_PIN_10, GPIO_AF7_USART1); // PA9 TX, PA10 RX
    uart_basic(&huart1, USART1, baud1);

    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(3, 3, 1)); // 调试口 RX优先级 3,1
    NVIC_EnableIRQ(USART1_IRQn);

    HAL_UART_Receive_IT(&huart1, &rx1_byte, 1); /* RX 仍 IT */
    CLEAR_BIT(USART1->CR3, USART_CR3_EIE);      //    只保留 IDLE | RXNE | TC，关掉 EIE

    /* DMA 模式下彻底不用 TXE 中断 */
#if USE_DMA
    UART1_TXE_DISABLE();
#else
    UART1_TXE_ENABLE();
#endif

#if USE_DMA
    /* TX DMA：DMA2 Stream2 → USART1_TX（CubeMX 默认映射） */
    hdma_usart1_tx.Instance = DMA2_Stream2;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    HAL_DMA_Init(&hdma_usart1_tx);
    __HAL_LINKDMA(&huart1, hdmatx, hdma_usart1_tx);

    NVIC_SetPriority(DMA2_Stream2_IRQn, NVIC_EncodePriority(3, 2, 1)); // 调试口 TX 完成优先级 2,1
    NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    UART1_TXE_DISABLE(); /* 禁用 TXE 中断，DMA 传输时不需要 */
#endif

    /* --- USART2 (数据口) --- */
    gpio_init(GPIOA, GPIO_PIN_2, GPIO_PIN_3, GPIO_AF7_USART2); // PA2 TX, PA3 RX
    uart_basic(&huart2, USART2, baud2);

    NVIC_SetPriority(USART2_IRQn, NVIC_EncodePriority(3, 1, 0)); // 数据口优先级 1,0
    NVIC_EnableIRQ(USART2_IRQn);

#if USE_DMA /*  DMA + IDLE --------------------------*/
    hdma_usart2_rx.Instance = DMA1_Stream5;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_USART2_RX;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_HIGH;
    HAL_DMA_Init(&hdma_usart2_rx);
    __HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);

    NVIC_SetPriority(DMA1_Stream5_IRQn, NVIC_EncodePriority(3, 3, 0)); // 数据口 RX 错误优先级 3,0
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    /* 启动环形 DMA */
    HAL_UART_Receive_DMA(&huart2, rx2_dma_buf, RX2_DMA_SZ);
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT | DMA_IT_TC);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

    /* 启用 DMA 错误中断，用于 TEIF 处理  */
    __HAL_DMA_ENABLE_IT(&hdma_usart2_rx, DMA_IT_TE);
#else /* 纯 IT RX */
    HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
#endif
}

/* =========================================================
 * 5. 发送启动函数
 * =======================================================*/

static void uart1_start_dma_tx(void)
{
#if USE_DMA
    if (huart1.gState != HAL_UART_STATE_READY)
        return;

    CRITICAL_SECTION_START();
    if (dma_inflight || !fifo_cnt())
    {
        CRITICAL_SECTION_END();
        return;
    }
    /* [FIX-5] 在临界区内把 inflight 设为 true 再启动 DMA，避免竞态 */
    dma_inflight = true;

    uint16_t tail = tx_tail & (TX_FIFO_SZ - 1u);
    uint16_t head = tx_head & (TX_FIFO_SZ - 1u);
    uint16_t len = (head > tail) ? (head - tail) : (TX_FIFO_SZ - tail);

    // __DMB(); /* 可选：如遇到非常规一致性问题可打开 */
    HAL_UART_Transmit_DMA(&huart1, &tx_fifo[tail], len);

    CRITICAL_SECTION_END();
#endif
}

/* =========================================================
 * 6. printf-风格发送（阻塞）
 * =======================================================*/
// 格式化：printf-style小文本发送
// 典型长度 < 200 B, 用于快速打印单行
void USART_SendFormatted(UART_HandleTypeDef *huart, const char *format, ...)
{
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len <= 0)
        return;

    // 发送指定长度的数据到 UART，每次发送不超过缓冲区大小 - 1 字节
    int bytesSent = 0;
    while (bytesSent < len)
    {
        // 计算本次发送的字节数，不超过剩余数据量和缓冲区可用空间
        size_t remaining = (size_t)(len - bytesSent);
        size_t chunkSize = (remaining > sizeof(buf) - 1) ? sizeof(buf) - 1 : remaining;

        // 发送数据并检查返回值
        HAL_UART_Transmit(huart, (uint8_t *)(buf + bytesSent), chunkSize, HAL_MAX_DELAY);
        bytesSent += chunkSize;
    }
}

/* =========================================================
 * 7. DMA 非阻塞发送
 * =======================================================*/
// 格式化短报文发送
void USART_SendFormatted_DMA(const char *fmt, ...)
{
    char tmp[256];
    va_list ap;
    va_start(ap, fmt);
    int len = vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (len <= 0)
        return;

    /* 直接复用 uart_write_dma —— 不够 1KB，非常快 */
    uart_write_dma(&TERM_UART, (const uint8_t *)tmp, (size_t)len);
}

// 非格式化长报文发送（DMA, 非阻塞双缓冲）
size_t uart_write_dma(UART_HandleTypeDef *huart, const uint8_t *data, size_t len)
{
#if !USE_DMA
    HAL_UART_Transmit(huart, (uint8_t *)data, len, HAL_MAX_DELAY);
    return len;
#else
    size_t queued_total = 0;

    while (len)
    {
        /* 这一次尽量全写，但如果空间不足则部分写入 */
        uint32_t room = TX_FIFO_SZ - fifo_cnt();
        if (room == 0)
        {
            dma_fifo_overflow_count += len;
#if (OPERATING_MODE == 0)
            USART_SendFormatted(huart, "\r\n[FIFO FULL]\r\n");
#endif
            break;
        }

        uint32_t chunk = (len > room) ? room : (uint32_t)len;

        if (!fifo_put_multi(data, (uint16_t)chunk))
        {
            /* 正常不会走到这里，因为前面刚算了 room。
             * 留作防御，也算作溢出 */
            dma_fifo_overflow_count += chunk;
            break;
        }

        data += chunk;
        len -= chunk;
        queued_total += chunk;
    }

    uart1_start_dma_tx();
    return queued_total;
#endif
}

/* =========================================================
 * 8. 中断服务
 * =======================================================*/

// USART-1  IRQHandler
// — ORE / RXNE / TXE / DMA-Tx 完成
void USART1_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
        __HAL_UART_CLEAR_OREFLAG(&huart1);

#if !USE_DMA
    /* 只有非 DMA 版本才会用 TXE 逐字节发 */
    if (__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) &&
        __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
    {
        if (fifo_cnt())
            huart1.Instance->TDR = tx_fifo[tx_tail++ & (TX_FIFO_SZ - 1u)];
        else
            UART1_TXE_DISABLE();
    }
#endif

    HAL_UART_IRQHandler(&huart1);
}

/* ---------- USART2 : DMA + IDLE ------------- */

void USART2_IRQHandler(void)
{
#if USE_FREERTOS
    BaseType_t xHigherPTWoken = pdFALSE;
#endif

#if USE_DMA
    uint16_t curr = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    if (curr == 0)
        curr = RX2_DMA_SZ;
    uint16_t len = (prev_ndtr >= curr) ? (prev_ndtr - curr)
                                       : (prev_ndtr + RX2_DMA_SZ - curr);
    prev_ndtr = curr;

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE) &&
        __HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(&huart2);
        volatile uint8_t dummy = huart2.Instance->RDR;

        if (len)
        {
            proto_ringbuf_push(&rx2_dma_buf[dma_tail], len);

#if DEBUG_ECHO
            /* 如果打开回显，这里使用批量入队 API */
            if (!fifo_put_multi(&rx2_dma_buf[dma_tail], len))
            {
                fifo_overflow_count += len;
            }
            else
            {
                uart1_start_dma_tx();
            }
#endif
            dma_tail = (dma_tail + len) & (RX2_DMA_SZ - 1);
        }
    }
#endif

    HAL_UART_IRQHandler(&huart2);

#if USE_FREERTOS
    portYIELD_FROM_ISR(xHigherPTWoken);
#endif
}

/* ---------- DMA2 Stream2 : TEIF 错误处理  -------- */

#if USE_DMA
void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
}

/* =========================================================
 * USART1 - Tx DMA 完成回调
 * — 若 FIFO 顶部是 0xFE 描述符，则继续发下一块
 * =======================================================*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance != USART1)
        return;

    // __DMB(); /* [FIX-8] 可选 memory barrier */

    CRITICAL_SECTION_START();
    tx_tail += h->TxXferSize;
    CRITICAL_SECTION_END();

    dma_inflight = false;
    uart1_start_dma_tx();
}

#endif

/* ---------- DMA1 Stream5 : TEIF 错误处理  -------- */
#if USE_DMA
void DMA1_Stream5_IRQHandler(void)
{
    if (__HAL_DMA_GET_FLAG(&hdma_usart2_rx, DMA_FLAG_TEIF1_5))
    {
        __HAL_DMA_CLEAR_FLAG(&hdma_usart2_rx, DMA_FLAG_TEIF1_5);

        /* ① 关闭 DMA */
        __HAL_DMA_DISABLE(&hdma_usart2_rx);

        /* ② 复位计数器为满缓冲（RX2_DMA_SZ）并重新启动 */
        __HAL_DMA_SET_COUNTER(&hdma_usart2_rx, RX2_DMA_SZ);
        __HAL_DMA_ENABLE(&hdma_usart2_rx);

        /* ③ 同步软件游标 —— 复位为“空”（tail 指向下一写入位）   */
        prev_ndtr = RX2_DMA_SZ; /* 上一次 NDTR = RX2_DMA_SZ             */
        dma_tail = 0;           /* 环形尾指针归零                 */

        rb_reset(&rb); /* 可选：若需要把 ring-buf 清零    */
    }
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
}
#endif

/* HAL 回调: RX 完成：快速入队，再尝试发出                                   */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == USART1)
    {
        /* 简单 PC->调试口的回显：也走 FIFO+DMA */
        uint8_t ch = rx1_byte;
        HAL_UART_Receive_IT(&huart1, &rx1_byte, 1); // 重新开启接收

#if (OPERATING_MODE == 0)
        /* -----------------------------------------------------------
         * 功能 A: 文本指令解析 (#FAN:...), 单个驱动器直接控制
         * ----------------------------------------------------------- */
        if (!dbg_cmd_ready) // 如果上一条指令还没处理完，先暂不接收新指令防止覆盖
        {
            if (ch == '\n' || ch == '\r')
            {
                if (dbg_rx_idx > 0) // 只有缓冲区有数据才置标志位
                {
                    dbg_cmd_ready = true;
                }
            }
            else if (dbg_rx_idx < DBG_CMD_MAX_LEN - 1)
            {
                dbg_rx_buf[dbg_rx_idx++] = ch;
            }
            else
            {
                // 缓冲区溢出，重置
                dbg_rx_idx = 0;
            }
        }

        /* -----------------------------------------------------------
         * 功能 B: 将字节注入协议层环形缓冲区
         * -----------------------------------------------------------
         * 1. proto_poll() 在主循环里时刻监视 rb (RingBuffer)。
         * 2. 如果我们通过串口1发送了标准的 90字节 数据帧 (0xAA 0xBB...)，
         *    这里会逐字节推入 rb。
         * 3. 当 rb 攒够一帧后，proto_poll 就会成功提取，并触发
         *    Position_Controller -> PGD -> Set_Fan_PWM 的全流程。
         * ----------------------------------------------------------- */
        proto_ringbuf_push(&ch, 1);

#endif // OPERATING_MODE == 0

#if USE_DMA
        if (fifo_put_multi(&ch, 1))
        {
            uart1_start_dma_tx();
        }
        else
        {
#if (OPERATING_MODE == 0)
            USART_SendFormatted(&TERM_UART, "\r\n[FIFO FULL]\r\n");
#endif
            fifo_overflow_count++;
        }
#else
        if (h->Instance == USART2)
        {
            uint8_t ch = rx2_byte;
            HAL_UART_Receive_IT(&huart2, &rx2_byte, 1);
            if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
            {
                huart1.Instance->TDR = ch;
                UART1_TXE_ENABLE();
            }
            else
            {
                fifo_put(ch);
                UART1_TXE_ENABLE();
            }
        }
#endif
    }
}

/*  ORE / 其他 UART 错误回调 —— 重新启动 DMA */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if USE_DMA
    if (huart->Instance == USART2 &&
        (huart->ErrorCode & (HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE |
                             HAL_UART_ERROR_NE | HAL_UART_ERROR_PE)))
    {
        __HAL_DMA_DISABLE(huart->hdmarx);
        __HAL_DMA_SET_COUNTER(huart->hdmarx, RX2_DMA_SZ);
        __HAL_DMA_ENABLE(huart->hdmarx);

        prev_ndtr = RX2_DMA_SZ;
        dma_tail = 0;
        rb_reset(&rb);

        huart->ErrorCode = HAL_UART_ERROR_NONE;
    }
#endif
}

/* =========================================================
 * 9. 调试命令轮询处理函数
 * =======================================================*/
#if (OPERATING_MODE == 0) // 仅在调试模式编译
/* 内部辅助：解析并执行指令 */
static void Process_Debug_Command(char *cmd_str)
{
    int id = -1;
    int duty_int = 0;

    // 解析格式: #FAN:id,duty
    // 示例: #FAN:0,50  或 #FAN:99,10
    if (sscanf(cmd_str, "#FAN:%d,%d", &id, &duty_int) == 2)
    {
        // 1. 安全限幅 (MIN_FAN_Duty_Rate% - MAX_FAN_Duty_Rate%)
        float duty = (float)duty_int;
        if (duty < MIN_FAN_Duty_Rate)
            duty = MIN_FAN_Duty_Rate;
        if (duty > MAX_FAN_Duty_Rate)
            duty = MAX_FAN_Duty_Rate;

        // 2. 映射 ID 到 FanControl 结构体
        // ID 映射表 (参考 Set_Fan_PWM 中的顺序)
        // 0:LX_p, 1:LX_n, 2:RX_p, 3:RX_n, 4:FY_p, 5:FY_n
        // 6:AY_p, 7:AY_n, 8:LZ_p, 9:LZ_n, 10:RZ_p, 11:RZ_n

        bool update_all = (id == 99);
        bool valid_id = true;

        if (update_all || id == 0)
            Fan_Control_duty_rate.control_LX_p = duty;
        if (update_all || id == 1)
            Fan_Control_duty_rate.control_LX_n = duty;
        if (update_all || id == 2)
            Fan_Control_duty_rate.control_RX_p = duty;
        if (update_all || id == 3)
            Fan_Control_duty_rate.control_RX_n = duty;
        if (update_all || id == 4)
            Fan_Control_duty_rate.control_FY_p = duty;
        if (update_all || id == 5)
            Fan_Control_duty_rate.control_FY_n = duty;
        if (update_all || id == 6)
            Fan_Control_duty_rate.control_AY_p = duty;
        if (update_all || id == 7)
            Fan_Control_duty_rate.control_AY_n = duty;
        if (update_all || id == 8)
            Fan_Control_duty_rate.control_LZ_p = duty;
        if (update_all || id == 9)
            Fan_Control_duty_rate.control_LZ_n = duty;
        if (update_all || id == 10)
            Fan_Control_duty_rate.control_RZ_p = duty;
        if (update_all || id == 11)
            Fan_Control_duty_rate.control_RZ_n = duty;

        if (!update_all && (id < 0 || id > 11))
            valid_id = false;

        // 3. 应用 PWM 到底层硬件
        if (valid_id)
        {
            Set_Fan_PWM(&Fan_Control_duty_rate);
            USART_SendFormatted_DMA("\r\n[CMD] Set Fan ID:%d -> Duty:%.1f%% OK\r\n", id, duty);
        }
        else
        {
            USART_SendFormatted_DMA("\r\n[CMD] Invalid Fan ID (0-11, 99=All)\r\n");
        }
    }
    else
    {
        USART_SendFormatted_DMA("\r\n[CMD] Syntax Error. Use: #FAN:id,duty\r\n");
    }
}

/* 公共接口：在 main 中轮询 */
void Debug_Console_Poll(void)
{
    if (dbg_cmd_ready)
    {
        // 确保字符串以 null 结尾
        dbg_rx_buf[dbg_rx_idx] = '\0';
        Process_Debug_Command((char *)dbg_rx_buf);

        // 清除状态，准备接收下一条
        dbg_rx_idx = 0;
        dbg_cmd_ready = false;
    }
}
#endif
