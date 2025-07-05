#include "sys.h"
#include "main.h"
#include "timer.h"
#include "Silde_Mode_Controller.h"
#include "usart.h"
#include "delay.h"
#include "Fan.h"
#include <string.h>

#define ENABLE_USART 1
#define ENABLE_TIMER 1
#define ENABLE_CONTROLLER 1

const uint32_t REPORT_INTERVAL = 200; // 100Hz下为2s

int main(void)
{
    Cache_Enable();                 // 打开L1-Cache
    HAL_Init();                     // 初始化HAL库
    Stm32_Clock_Init(192, 5, 2, 2); // 设置时钟
    delay_init(480);                // 延时初始化

#if ENABLE_USART
    uart_init(230400); // 串口初始化

#if !OPERATING_MODE
    USART_SendFormatted(&TERM_UART, "\r\n[DEBUG-MODE] UART1 send & receive\r\n");
#else
    USART_SendFormatted(&TERM_UART, "\r\n[OPERATING-MODE] UART2 receive data, UART1 debug output\r\n");
#endif
#endif

#if ENABLE_TIMER
#if !OPERATING_MODE
    USART_SendFormatted(&TERM_UART, "\r\n[DEBUG-MODE] Debug data frames responded via UART1, with non-fixed frequency.\r\n");
#else
    TIM6_Init(); // 初始化TIM6
    USART_SendFormatted(&TERM_UART, "\r\n[OPERATING-MODE] Control responses are executed at a frequency of 100Hz.\r\n");
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

    USART_SendFormatted(&TERM_UART, "\r\nController Initialized\r\n");

    // 初始化参数并初始化控制器
    InitConstantsToFloat();                    // 初始化常量为浮点数
    InitController(&ctrl_input, &ctrl_output); // 初始化控制器

#endif
    // 主循环
    while (1)
    {
#if ENABLE_USART
        uint8_t payload[72];
        uint32_t seq;
        uint64_t ts;

        if (frame_extract(payload, &seq, &ts))
            parse_data(payload, seq, ts, &ctrl_input, &prev_ctrl_input); // 解析数据帧

#if OPERATING_MODE
        if (dbg_flag) // 触发定时器中断
        {
            dbg_flag = 0;                         // 重置标志位
            if (++report_tick >= REPORT_INTERVAL) // 每1000次中断发送一次调试信息
            {
                report_tick = 0;
                send_info(&TERM_UART); // 在主循环打印调试信息
            }
        }
#endif
        // 其他非关键性任务可以在这里执行，例如监控状态或处理其他事件
        // times++;
        // if (times % 200 == 0)
        // {
        //     USART_SendFormatted("Waiting for data...\r\n"); // 定期发送调试信息，以确认程序运行正常
        // }
        // delay_ms(10);
#endif
    }
}
