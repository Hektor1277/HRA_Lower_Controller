#ifndef _USART_H
#define _USART_H

#include "sys.h"
#include "stdio.h"
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

#define SEQ_LENGTH 4                                                                       // 4字节，数据序号
#define TIME_LENGTH 8                                                                      // 8字节，时间戳
#define DATA_LENGTH 72                                                                     // 72字节，36个int16_t类型数据
#define CHECKSUM_LENGTH 2                                                                  // 2字节，校验和长度
#define TOTAL_FRAME_LEN (2 + SEQ_LENGTH + TIME_LENGTH + DATA_LENGTH + CHECKSUM_LENGTH + 2) // 帧头(2) + 序号(4) + 时间戳(8) + 数据(72) + 帧尾(2) = 90字节，总帧长度

//================== 模式选择 ==================
// 0 = 调试模式:   UART1 既收数据又打印调试
// 1 = 运行模式:   UART2 收数据, UART1 打印调试
//=============================================
#define OPERATING_MODE 1              // 系统运行模式切换标志位
#define SEND_DETAIL (!OPERATING_MODE) // 调试模式发送完整 72 字段
extern volatile uint32_t report_tick;

extern UART_HandleTypeDef UART1_Handler;
extern UART_HandleTypeDef UART2_Handler;

#if OPERATING_MODE
#define DATA_UART UART2_Handler // DATA_UART 调试时为串口2，实际运行时为串口1
#define TERM_UART UART1_Handler // TERM_UART 始终为串口1
#else
#define DATA_UART UART1_Handler
#define TERM_UART UART1_Handler
#endif

// 缓冲区定义
#define PC_CMD_LEN 128    // 定义PC命令接收缓冲区长度
#define USART_REC_LEN 128 // 接收缓冲长度（包括帧头、数据、校验和、帧尾）
#define RXBUFFERSIZE 1    // 串口接收缓冲区大小

/* 过滤阈值（向前声明，真正定义在 .c 中，编译期常量） */
extern const float THRESHOLD; /* =120.0f */

/* 供解析函数写入 / 调试函数读取 -----------------------*/
extern volatile uint32_t g_frame_seq_id;  /* 最近一帧的序号 */
extern volatile uint64_t g_frame_time_ns; /* 最近一帧的 unix ns */
extern volatile float g_frame_latency;    /* 最近一帧的延迟（ms） */
extern volatile uint32_t g_seq_gap;       /* 最近两帧的序号间隔（gap） */

// 串口初始化函数声明
void uart_init(u32 bound);

// 串口发送格式化字符串
void USART_SendFormatted(UART_HandleTypeDef *huart, const char *format, ...);

// 串口数据解析函数声明
void send_info(UART_HandleTypeDef *huart);
void parse_data(ControllerInput *ctrl_input, ControllerInput *prev_ctrl_input); // 修改为处理int16_t类型数据

// 静态内联函数声明
static inline int16_t be_to_i16(const uint8_t *p) __attribute__((always_inline));
static inline float be_i16_to_f(const uint8_t *p) __attribute__((always_inline));
static inline bool big_jump(float now, float prev) __attribute__((always_inline));
static inline int16_t be_to_i16(const uint8_t *p)
{
    return (int16_t)((p[0] << 8) | p[1]); // 大端 → 小端
}

static inline float be_i16_to_f(const uint8_t *p)
{
    return (float)((int16_t)((p[0] << 8) | p[1])) * 0.001f;
}

static inline bool big_jump(float now, float prev)
{
    return fabsf(now - prev) > THRESHOLD;
}

#endif
