#include "sys.h"
#include "main.h"
#include "timer.h"
#include "serial.h"
#include "protocol.h"
#include "Silde_Mode_Controller.h"
#include "delay.h"
#include "Fan.h"
#include <string.h>

#define ENABLE_USART 1
#define ENABLE_TIMER 1
#define ENABLE_CONTROLLER 1

volatile uint32_t report_tick = 0;
const uint32_t REPORT_INTERVAL = 200; // 100Hz下为2s

int main(void)
{
    MPU_Config_DMA_NC();                  // 配置 DMA 环形缓冲区为非 Cache
    proto_init_buffers(rx2_dma_buf, 512); // 初始化协议缓冲区
    Cache_Enable();                       // 打开L1-Cache
    HAL_Init();                           // 初始化HAL库
    Stm32_Clock_Init(192, 5, 2, 2);       // 设置时钟
    delay_init(480);                      // 延时初始化
    DWT_Enable();                         // 启用 DWT 计数器
    // SCB_DisableDCache();            // 关闭D-Cache

#if ENABLE_USART
    MX_USART_Init(230400, 230400); // 初始化USART1和USART2

#if !OPERATING_MODE
    USART_SendFormatted_DMA("\r\n[DEBUG-MODE] Running in low frequency. Send detailed information.\r\n");
#else
    USART_SendFormatted_DMA("\r\n[OPERATING-MODE] Running in high frequency. Send essential information.\r\n");
#endif
#endif

#if ENABLE_TIMER
#if !OPERATING_MODE
    USART_SendFormatted_DMA("\r\n[DEBUG-MODE] Disable Timer6, response data frame when received.\r\n");
#else
    TIM6_Init(); // 初始化TIM6
    USART_SendFormatted_DMA("\r\n[OPERATING-MODE] Enable Timer6, Controller running at 100Hz.\r\n");
#endif
    PWM_Init(); // PWM初始化
#endif

#if ENABLE_CONTROLLER
    // 定义控制器输入输出, 风扇转速和占空比结构体
    extern ControllerInput ctrl_input;
    extern ControllerInput prev_ctrl_input;
    extern ControllerOutput ctrl_output;
    extern FanSpeed Fan_desire_Speed;
    extern FanControl Fan_Control_duty_rate;

    USART_SendFormatted_DMA("\r\nController Initialized.\r\n");

    // 初始化参数并初始化控制器
    InitConstantsToFloat();                    // 初始化常量为浮点数
    InitController(&ctrl_input, &ctrl_output); // 初始化控制器

#endif

    // 主循环
    while (1)
    {
#if ENABLE_USART
        proto_poll(); // 解析并更新 ctrl_input
#if OPERATING_MODE
        if (ctrl_buf[ctrl_r].valid)
        {
            ctrl_buf[ctrl_r].valid = false;
            ControllerOutput local_out = ctrl_buf[ctrl_r].data; // 复制到局部
            Calculate_Fan_Speed(&local_out, &Fan_desire_Speed); // 无告警调用
            ctrl_r ^= 1u;
        }
        if (dbg_flag) // 触发定时器中断
        {
            dbg_flag = 0;                         // 重置标志位
            if (++report_tick >= REPORT_INTERVAL) // 每REPORT_INTERVAL次中断发送一次调试信息
            {
                report_tick = 0;
                send_info(&TERM_UART); // 在主循环打印调试信息
            }
        }
#endif
#endif
    }
}
