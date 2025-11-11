/************************************************************
0  1  2..5     6..13          14..85           86..87  88 89
┌──┬──┬────────┬──────────────┬────────────────┬──────┬──┬──┐
│AA│BB│sequence│ unix_time_ns │ 72 B payload   │ CRC16│CC│DD│
└──┴──┴────────┴──────────────┴────────────────┴──────┴──┴──┘
           ↑Big - endian↑
*************************************************************/

#ifndef _PROTOCOL_H
#define _PROTOCOL_H

#include "stm32h7xx_hal.h"
#include "ringbuf.h"
#include <stdint.h>
#include "stdio.h"
#include "Silde_Mode_Controller.h"
#include <stdbool.h>
#include <math.h>
#include "config.h"

//================== 协议常量 ===================
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0xBB
#define FRAME_FOOTER_1 0xCC
#define FRAME_FOOTER_2 0xDD
#define FRAME_LEN 90u
#define PAYLOAD_LEN 72u
//================== 协议常量 ===================

//================== 声明区 =================
// 全局调试量
extern volatile uint32_t g_frame_seq_id;  /* 最近一帧的序号 */
extern volatile uint64_t g_frame_time_ns; /* 最近一帧的 unix ns */
extern volatile float g_frame_latency;    /* 最近一帧的延迟（ms） */
extern volatile uint32_t g_seq_gap;       /* 最近两帧的序号间隔（gap） */
extern volatile uint32_t report_tick;     /* 报告周期触发计时器 */

// 错误计数器
extern volatile uint32_t data_anomaly_count; // 数据帧异常计数器
extern volatile uint64_t lost_pkg_count;     // 丢包计数器
extern volatile uint64_t crc_failed_count;   // CRC 校验未通过计数器

// 过滤阈值
// extern const float anomaly_threshold; /* =3.0f */
// 状态标志
extern bool frame_fault;
extern bool is_first_frame;
//================== 声明区 =================

//============= 对上位机的业务 API ==============
void proto_init_buffers(uint8_t *dma_mem, uint16_t cap);
void proto_ringbuf_push(uint8_t *p, uint16_t len);
void proto_poll(void);
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

void send_info(UART_HandleTypeDef *huart); // 串口数据解析函数声明
//============= 对上位机的业务 API ==============

//============= 静态内联函数 ==============
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
    return fabsf(now - prev) > anomaly_threshold;
}
//============= 静态内联函数 ==============

#endif /* _PROTOCOL_H */
