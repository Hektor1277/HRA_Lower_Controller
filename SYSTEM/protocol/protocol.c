#include "protocol.h"
#include "serial.h"
#include "Fan.h"
#include "timer.h"
#include "PGD.h"
#include "WDG.h"
#include <string.h>

/* ============= 声明区 ==============*/
// 全局调试量
volatile uint32_t g_frame_seq_id = 0;
volatile uint64_t g_frame_time_ns = 0;
volatile float g_frame_latency = 0.f;
volatile uint32_t g_seq_gap = 0;
// 错误计数器
volatile uint64_t lost_pkt_count = 0;
volatile uint32_t frame_error_count = 0;
volatile uint32_t data_anomaly_count = 0;
// 数据帧状态标志
bool frame_fault = false;       // 数据帧滤波标志位，0为正常，1为未通过滤波
bool is_first_frame = true;     // 标志是否是第一次接收数据
const float THRESHOLD = 120.0f; // 过滤阈值
// 1024 B 环形缓冲，本身存在 D2 non-cache 区域
static uint8_t proto_mem[RX2_DMA_SZ] __attribute__((section(".dma_nc")));
ringbuf_t rb = {
    .buf = proto_mem,
    .cap = RX2_DMA_SZ,
    .head = 0,
    .tail = 0,
};
/* ============= 声明区 ==============*/

/* ============= 工具函数 ==============*/
// CRC
static uint16_t crc16_be_acc(const uint8_t *p, uint16_t n)
{
    uint16_t c = 0;
    while (n)
    {
        c += (uint16_t)(*p++) << 8 | *p++;
        n -= 2;
    }
    return c;
}

// 初始化环形缓冲区
void proto_init_buffers(uint8_t *dma_mem, uint16_t cap)
{
    rb_init(&rb, dma_mem, cap);
}

// 快速写入环形缓冲区：一次或两次 memcpy；不做逐字节 while
void proto_ringbuf_push(uint8_t *p, uint16_t len)
{
    /* 假设上层保证 len <= cap（本工程中 IDLE 每次推进长度远小于 1024） */
    uint16_t head = rb.head;
    uint16_t cap = rb.cap;

    uint16_t first = (cap - head < len) ? (cap - head) : len;
    memcpy(rb.buf + head, p, first);
    memcpy(rb.buf, p + first, len - first);

    rb.head = (head + len) % cap;

    /* 若有需要，可在此加入对“写满覆盖”的统计。但当前我们在 frame_extract 中做 CRC 过滤/统计。*/
}

// 轮询环形缓冲区，提取完整帧
void proto_poll(void)
{
    uint8_t payload[PAYLOAD_LEN];
    uint32_t seq;
    uint64_t ts;
    static uint32_t last_seq = 0;
    bool got = false;

    while (frame_extract(payload, &seq, &ts))
        got = true; /* 抛弃旧帧，只留最新一帧 */

    if (!got || seq == last_seq) /* 没有新帧/序号没变 → 直接返回 */
        return;

    last_seq = seq;
    parse_data(payload, seq, ts, &ctrl_input, &prev_ctrl_input);

#if OPERATING_MODE == 0

    // 调用位置和姿态控制器计算控制输出
    Position_Controller(&ctrl_input, &ctrl_output);
    Attitude_Controller(&ctrl_input, &ctrl_output);

    // 调用底层风扇控制函数，根据控制输出计算风扇转速并输出相应PWM信号
    Fan_Rotation_Control(&ctrl_output, &Fan_desire_Speed, &Fan_Control_duty_rate);

    while (TERM_UART.gState != HAL_UART_STATE_READY)
        ;
    send_info(&TERM_UART); /* 只打印一次 */
#endif
}

/* ============= 工具函数 ==============*/

/* ============= 数据协议业务函数 ==============*/
// 帧提取
bool frame_extract(uint8_t payload[72], uint32_t *seq, uint64_t *ts)
{
    uint8_t tmp[FRAME_LEN]; // FRAME_LEN = 90
    while (rb_size(&rb) >= FRAME_LEN)
    {
        rb_peek(&rb, tmp, FRAME_LEN);

        if (tmp[0] == FRAME_HEADER_1 && tmp[1] == FRAME_HEADER_2 &&
            tmp[88] == FRAME_FOOTER_1 && tmp[89] == FRAME_FOOTER_2)
        {
            // 校验数据段：tmp[14] ~ tmp[85]，共 72 字节
            uint16_t expected = (tmp[86] << 8) | tmp[87];
            uint16_t computed = crc16_be_acc(tmp + 14, 72);

#if SEND_DETAIL
            static uint32_t last_crc_us = 0;
            uint32_t now = HAL_GetTick(); // 1 ms tick
            if (now - last_crc_us > 10)
            { // ≥10 ms 再打印
                USART_SendFormatted_DMA("\r\nExpected CRC: 0x%04X, Computed CRC: 0x%04X\r\n", expected, computed);
                last_crc_us = now;
            }
#endif

            if (computed == expected)
            {
                // 提取帧序号
                *seq = (tmp[2] << 24) | (tmp[3] << 16) | (tmp[4] << 8) | tmp[5];

                // 提取时间戳
                *ts = 0;
                for (int i = 0; i < 8; i++)
                {
                    *ts = (*ts << 8) | tmp[6 + i];
                }

                // 提取 payload
                memcpy(payload, tmp + 14, PAYLOAD_LEN);

                // 提取成功则，移动 tail
                rb_pop(&rb, FRAME_LEN);
                return true;
            }
            else
            {
                frame_error_count++;
            }
        }
        // 滑窗前移，容忍噪声字节
        rb_pop(&rb, 1);
    }
    return false;
}

// 解析数据帧 (Frame Apply)
void parse_data(const uint8_t payload[72],
                uint32_t seq, uint64_t unix_ns,
                ControllerInput *ctrl_input,
                ControllerInput *prev_ctrl_input)
{
    /* 1) 延迟 & 丢包统计 */
    if (is_first_frame)
    {
        is_first_frame = false;
    }
    else
    {
        g_seq_gap = (seq > g_frame_seq_id) ? seq - g_frame_seq_id - 1 : 0;
        lost_pkt_count += g_seq_gap;
        g_frame_latency = (float)(unix_ns - g_frame_time_ns) / 1e6f; /* ms */
    }
    g_frame_seq_id = seq;
    g_frame_time_ns = unix_ns;

    /* 2) 72 B→ControllerInput (12×vec3) */
    const uint8_t *p = payload;
    for (int i = 0; i < 12; i++)
    {
        float *dst = ((float *)ctrl_input) + 3 * i;
        for (int k = 0; k < 3; k++, p += 2)
        {
            int16_t raw = be_to_i16(p);
            float val = scale_i16(raw);

            /* 跳变滤波 */
            float prev_val = ((float *)prev_ctrl_input)[3 * i + k];
            if (big_jump(val, prev_val))
            {
                frame_fault = true;
                data_anomaly_count++;
                val = prev_val;
            }
            dst[k] = val;
        }
    }
    memcpy(prev_ctrl_input, ctrl_input, sizeof(ControllerInput));
    frame_fault = false;
}

// 发送调试信息
void send_info(UART_HandleTypeDef *huart)
{
#define OUT_SZ 2048
    static char outbuf[OUT_SZ];
    int off = 0;

    /* 发送侧速率限制（比如 50ms 以上才发一次），
     * 防止“数据端瞬间多帧 + 主线程也调用 send_info”把 TX FIFO 顶爆。
     * 如不想限速，把这一段 #if 0 掉。 */
    static uint32_t last_info_ms = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_info_ms < 50)
        return;
    last_info_ms = now;

    // 1) Bridge + Frame 系统状态, 错误计数 & 帧元数据
    off += snprintf(outbuf + off, OUT_SZ - off,
                    "\r\n=================== Bridge Status ===================\r\n"
                    "Frame Errors      : %lu\r\n"
                    "Data Anomalies    : %lu\r\n"
                    "Lost Packets      : %llu\r\n"

                    "\r\n=================== Bridge Status ===================\r\n"
                    "\r\n==================== Frame Meta ====================\r\n"
                    "Current Seq-ID    : %lu\r\n"
                    "Unix-time (ns)    : %llu\r\n"
                    "dt to prev (ms)   : %.3f\r\n"
                    "Seq Gap           : %lu"
                    "\r\n==================== Frame Meta ====================\r\n",
                    frame_error_count, data_anomaly_count, lost_pkt_count, (unsigned long)g_frame_seq_id, (unsigned long long)g_frame_time_ns, g_frame_latency, g_seq_gap);

    uint32_t isr_avg = isr_acc / isr_cnt;
    uint32_t pgd_avg = pgd_cnt ? pgd_acc_cycles / pgd_cnt : 0;
    off += snprintf(outbuf + off, OUT_SZ - off,
                    "\r\n=================== System Status ===================\r\n"
                    "ISR max          : %lu\r\n"
                    "ISR avg          : %lu\r\n"
                    "ISR cnt          : %lu\r\n"
                    "PGD max (cyc)   : %lu\r\n"
                    "PGD avg (cyc)   : %lu\r\n"
                    "PGD timeout        : %lu\r\n"
                    "Soft resets      : %lu\r\n"
                    "TX FIFO Drops     : %lu\r\n"
                    "DMA Queue Drops   : %lu"
                    "\r\n=================== System Status ===================\r\n",
                    isr_max, isr_avg, isr_cnt, pgd_max_cycles, pgd_avg, pgd_timeout_cnt, sw_reset_cnt, (unsigned long)fifo_overflow_count, (unsigned long)dma_fifo_overflow_count);

#if SEND_DETAIL
    // 2) Parsed Payload 解析后的数据
    off += snprintf(outbuf + off, OUT_SZ - off,
                    "\r\n================== Parsed Payload ==================\r\n"
                    "Desired Pos : %.3f, %.3f, %.3f\r\n"
                    "Desired Vel : %.3f, %.3f, %.3f\r\n"
                    "Desired Acc : %.3f, %.3f, %.3f\r\n"
                    "Desired Ang : %.3f, %.3f, %.3f\r\n"
                    "Desired Ang Vel : %.3f, %.3f, %.3f\r\n"
                    "Desired Ang Acc : %.3f, %.3f, %.3f\r\n"
                    "Position : %.3f, %.3f, %.3f\r\n"
                    "Velocity : %.3f, %.3f, %.3f\r\n"
                    "Acceleration : %.3f, %.3f, %.3f\r\n"
                    "Angles : %.3f, %.3f, %.3f\r\n"
                    "Angular Vel : %.3f, %.3f, %.3f\r\n"
                    "Angular Acc : %.3f, %.3f, %.3f"
                    "\r\n================== Parsed Payload ==================\r\n",
                    ctrl_input.desired_position[0], ctrl_input.desired_position[1], ctrl_input.desired_position[2],
                    ctrl_input.desired_velocity[0], ctrl_input.desired_velocity[1], ctrl_input.desired_velocity[2],
                    ctrl_input.desired_acceleration[0], ctrl_input.desired_acceleration[1], ctrl_input.desired_acceleration[2],
                    ctrl_input.desired_angles[0], ctrl_input.desired_angles[1], ctrl_input.desired_angles[2],
                    ctrl_input.desired_angular_velocity[0], ctrl_input.desired_angular_velocity[1], ctrl_input.desired_angular_velocity[2],
                    ctrl_input.desired_angular_acceleration[0], ctrl_input.desired_angular_acceleration[1], ctrl_input.desired_angular_acceleration[2],
                    ctrl_input.position[0], ctrl_input.position[1], ctrl_input.position[2],
                    ctrl_input.velocity[0], ctrl_input.velocity[1], ctrl_input.velocity[2],
                    ctrl_input.acceleration[0], ctrl_input.acceleration[1], ctrl_input.acceleration[2],
                    ctrl_input.angles[0], ctrl_input.angles[1], ctrl_input.angles[2],
                    ctrl_input.angular_velocity[0], ctrl_input.angular_velocity[1], ctrl_input.angular_velocity[2],
                    ctrl_input.angular_acceleration[0], ctrl_input.angular_acceleration[1], ctrl_input.angular_acceleration[2]);

    // 3) Control 控制器输出
    off += snprintf(outbuf + off, OUT_SZ - off,
                    "\r\n================== Control Output ===================\r\n"
                    "Thrust : [%.3f, %.3f, %.3f]\r\n"
                    "Torque : [%.3f, %.3f, %.3f]"
                    "\r\n================== Control Output ===================\r\n",
                    ctrl_output.thrust[0], ctrl_output.thrust[1], ctrl_output.thrust[2],
                    ctrl_output.torque[0], ctrl_output.torque[1], ctrl_output.torque[2]);

    // 4) Fan 风扇输出
    off += snprintf(outbuf + off, OUT_SZ - off,
                    "\r\n================== Fan Output ===================\r\n"
                    "Fan duty rate in module 1(LT): LX+: %.3f, FY+: %.3f, LZ+: %.3f\r\n"
                    "Fan duty rate in module 2(RT): RX-: %.3f, AY-: %.3f, RZ+: %.3f\r\n"
                    "Fan duty rate in module 3(LB): LX-: %.3f, AY+: %.3f, LZ-: %.3f\r\n"
                    "Fan duty rate in module 4(RB): RX+: %.3f, FY-: %.3f, RZ-: %.3f\r\n"
                    "Desired Fan speed in module 1(LT): LX+: %.3f, FY+: %.3f, LZ+: %.3f\r\n"
                    "Desired Fan speed in module 2(RT): RX-: %.3f, AY-: %.3f, RZ+: %.3f\r\n"
                    "Desired Fan speed in module 3(LB): LX-: %.3f, AY+: %.3f, LZ-: %.3f\r\n"
                    "Desired Fan speed in module 4(RB): RX+: %.3f, FY-: %.3f, RZ-: %.3f"
                    "\r\n================== Fan Output ===================\r\n",
                    Fan_Control_duty_rate.control_LX_p, Fan_Control_duty_rate.control_FY_p, Fan_Control_duty_rate.control_LZ_p,
                    Fan_Control_duty_rate.control_RX_n, Fan_Control_duty_rate.control_AY_n, Fan_Control_duty_rate.control_RZ_p,
                    Fan_Control_duty_rate.control_LX_n, Fan_Control_duty_rate.control_AY_p, Fan_Control_duty_rate.control_LZ_n,
                    Fan_Control_duty_rate.control_RX_p, Fan_Control_duty_rate.control_FY_n, Fan_Control_duty_rate.control_RZ_n,
                    Fan_desire_Speed.omega_LX_p, Fan_desire_Speed.omega_FY_p, Fan_desire_Speed.omega_LZ_p,
                    Fan_desire_Speed.omega_RX_n, Fan_desire_Speed.omega_AY_n, Fan_desire_Speed.omega_RZ_p,
                    Fan_desire_Speed.omega_LX_n, Fan_desire_Speed.omega_AY_p, Fan_desire_Speed.omega_LZ_n,
                    Fan_desire_Speed.omega_RX_p, Fan_desire_Speed.omega_FY_n, Fan_desire_Speed.omega_RZ_n);

#endif // SEND_DETAIL

    /*拼接所有调试信息, 只调用一次 DMA 发送 */
    uart_write_dma(huart, (uint8_t *)outbuf, off);
}
