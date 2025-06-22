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

static volatile uint32_t report_tick = 0;
#if OPERATING_MODE
const uint32_t REPORT_INTERVAL = 1000; // 10 s
#else
const uint32_t REPORT_INTERVAL = 5; // 每5帧
#endif

// 调试用
float duty_cycle_1[12] = {50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0};
float duty_cycle_2[12] = {10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 5.0, 95.0};
float duty_cycle_3[12] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0};
float duty_cycle_4[12] = {15.5, 45.2, 75.8, 32.1, 87.4, 52.5, 19.0, 68.3, 90.9, 22.7, 40.6, 60.5};
float duty_cycle_5[12] = {99.9, 0.1, 50.5, 25.5, 75.3, 10.0, 90.0, 5.0, 80.0, 20.0, 100.0, 0.0};
// 调试用

int main(void)
{
    Cache_Enable();                 // 打开L1-Cache
    HAL_Init();                     // 初始化HAL库
    Stm32_Clock_Init(192, 5, 2, 2); // 设置时钟
    delay_init(480);                // 延时初始化

#if ENABLE_USART
    uart_init(115200); // 串口初始化
    // u16 times = 0;     // 计时

#if !OPERATING_MODE
    USART_SendFormatted(&TERM_UART,
                        "\r\n[DEBUG-MODE] 单口调试模式已启用: UART1 收+发\r\n");
#else
    USART_SendFormatted(&TERM_UART,
                        "\r\n[OPERATING] 双口运行模式已启用: UART2 收, UART1 调试输出\r\n");
#endif
#endif

#if ENABLE_TIMER
    TIM6_Init(); // 初始化TIM6，10ms周期，适当的预分频
    PWM_Init();  // PWM初始化
    Set_Fan_PWM(&Fan_Control_duty_rate);
#if !OPERATING_MODE
    USART_SendFormatted(&TERM_UART,
                        "\r\n[DEBUG-MODE] 低频调试模式已启用: 以1Hz频率执行控制响应(或其他频率, 具体请查看定时器头文件)\r\n");
#else
    USART_SendFormatted(&TERM_UART,
                        "\r\n[OPERATING] 高频运行模式已启用: 以100Hz频率执行控制响应(或其他频率, 具体请查看定时器头文件)\r\n");
#endif
#endif

#if ENABLE_CONTROLLER
    // 定义控制器输入输出, 风扇转速和占空比结构体
    extern ControllerInput ctrl_input;
    extern ControllerInput prev_ctrl_input;
    extern ControllerOutput ctrl_output;
    extern FanSpeed Fan_desire_Speed;
    extern FanControl Fan_Control_duty_rate;

    USART_SendFormatted(&TERM_UART,
                        "\r\n控制器初始化\r\n");

    // 初始化参数并初始化控制器
    InitConstantsToFloat();                    // 初始化常量为浮点数
    InitController(&ctrl_input, &ctrl_output); // 初始化控制器

#endif
    // 主循环
    while (1)
    {
#if ENABLE_USART
        if (dbg_flag) // 触发定时器中断
        {
            dbg_flag = 0; // 重置标志位
            if (++report_tick >= REPORT_INTERVAL)
            {
                report_tick = 0;
                send_info(&TERM_UART); // 统一在主循环打印调试信息
            }
        }
        // 其他非关键性任务可以在这里执行，例如监控状态或处理其他事件
        // times++;
        // if (times % 200 == 0)
        // {
        //     USART_SendFormatted("Waiting for data...\r\n"); // 定期发送调试信息，以确认程序运行正常
        // }
        // delay_ms(10);
#endif

#if ENABLE_CONTROLLER

#endif
    }
}
