#include "sys.h"
#include "main.h"
#include "timer.h"
#include "Silde_Mode_Controller.h"
#include "usart.h"
#include "delay.h"
#include "Fan.h"
#include <string.h>

#define ENABLE_USART_TEST 1
#define ENABLE_TIMER 1
#define ENABLE_CONTROLLER 1

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

#if ENABLE_USART_TEST
    uart_init(115200); // 串口初始化
    // u16 times = 0;     // 计时
#endif

#if ENABLE_TIMER
    TIM6_Init(); // 初始化TIM6，10ms周期，适当的预分频
    PWM_Init();  // PWM初始化
    Set_Fan_PWM(&Fan_Control_duty_rate);
#endif

#if ENABLE_CONTROLLER
    // 定义控制器输入输出, 风扇转速和占空比结构体
    extern ControllerInput ctrl_input;
    extern ControllerInput prev_ctrl_input;
    extern ControllerOutput ctrl_output;
    extern FanSpeed Fan_desire_Speed;
    extern FanControl Fan_Control_duty_rate;

    // 初始化参数并初始化控制器
    InitConstantsToFloat();                    // 初始化常量为浮点数
    InitController(&ctrl_input, &ctrl_output); // 初始化控制器

#endif
    // 主循环
    while (1)
    {
#if ENABLE_USART_TEST
        // 如果 UART1 的状态不是 READY 或没有处于接收中断模式，重新开启接收中断
        if (HAL_UART_GetState(&UART1_Handler) == HAL_UART_STATE_READY && !(USART_RX_STA & 0x8000))
        {
            HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);
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
