#ifndef _USART_H
#define _USART_H

#include "DMA.h"
#include "sys.h"
#include "stdio.h"
#include "ringbuf.h"
#include "Silde_Mode_Controller.h"
#include <stdbool.h>
#include <math.h> /* fabsf() 用于 big_jump */

/************************************************************
0  1  2..5     6..13          14..85           86..87  88 89
┌──┬──┬────────┬──────────────┬────────────────┬──────┬──┬──┐
│AA│BB│sequence│ unix_time_ns │ 72 B payload   │ CRC16│CC│DD│
└──┴──┴────────┴──────────────┴────────────────┴──────┴──┴──┘
           ↑Big - endian↑
*************************************************************/

// 数据帧格式定义
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0xBB
#define FRAME_FOOTER_1 0xCC
#define FRAME_FOOTER_2 0xDD

#define FRAME_LEN 90u /* 2+4+8+72+2+2 */
#define PAYLOAD_LEN 72u

// PC 指令缓冲
#define PC_CMD_LEN 64u

//================== 模式选择 ==================
// 0 = 调试模式:   系统低频运行，用于系统调试，此时发送全部调试信息
// 1 = 运行模式:   系统高频运行，用于实际控制，仅发送高层信息
//=============================================
#define OPERATING_MODE 0              // 系统运行模式切换标志位
#define SEND_DETAIL (!OPERATING_MODE) // 调试模式发送完整 72 字段

// UART 句柄宏
extern UART_HandleTypeDef UART1_Handler;
extern UART_HandleTypeDef UART2_Handler;

#define DATA_UART UART2_Handler // 数据接口，接收高频数据帧，DMA RX ONLY   */
#define TERM_UART UART1_Handler // 调试接口，发送调试信息并接收PC 指令，IT RX + DMA TX */

// 全局调试量
extern volatile uint32_t g_frame_seq_id;  /* 最近一帧的序号 */
extern volatile uint64_t g_frame_time_ns; /* 最近一帧的 unix ns */
extern volatile float g_frame_latency;    /* 最近一帧的延迟（ms） */
extern volatile uint32_t g_seq_gap;       /* 最近两帧的序号间隔（gap） */
extern volatile uint32_t report_tick;     /* 报告周期触发计时器 */
extern const float THRESHOLD;             /* =120.0f */

// 状态标志
extern bool frame_fault;
extern bool is_first_frame;

// 错误计数器
extern volatile u32 frame_error_count;    // 帧错误计数器
extern volatile u32 checksum_error_count; // 帧校验和错误计数器
extern volatile u32 data_anomaly_count;   // 数据帧异常计数器
extern volatile uint64_t lost_pkt_count;  // 丢包计数器

// 串口初始化函数声明
void uart_init(u32 bound);

// 阻塞发送格式化字符串（短报文）
void USART_SendFormatted(UART_HandleTypeDef *huart, const char *format, ...);

// DMA非阻塞格式化发送（长报文）
void USART_SendFormatted_DMA(UART_HandleTypeDef *huart, const char *format, ...);

// 低层 Tx API（主循环内直接调用）
void dbg_tx_dma(UART_HandleTypeDef *huart, const char *msg, uint16_t len);

// 帧提取：若成功则填充 payload72/seq/ns 并返回 true
bool frame_extract(uint8_t payload[72],
                   uint32_t *seq,
                   uint64_t *unix_ns);

// 数据解析并存入结构体 + 统计延迟/丢包
void parse_data(const uint8_t payload[72],
                uint32_t seq_id,
                uint64_t unix_ns,
                ControllerInput *ctrl_input,
                ControllerInput *prev_ctrl_input);

// 串口数据解析函数声明
void send_info(UART_HandleTypeDef *huart);

// 静态内联函数声明
static inline int16_t be_to_i16(const uint8_t *p) __attribute__((always_inline));
static inline float scale_i16(int16_t v) __attribute__((always_inline));
static inline bool big_jump(float now, float prev) __attribute__((always_inline));
static inline int16_t be_to_i16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]); // 大端 → 小端
}

static inline float scale_i16(int16_t v) { return v * 0.001f; }

static inline bool big_jump(float now, float prev)
{
    return fabsf(now - prev) > THRESHOLD;
}

#endif
