#include "timer.h"
#include "Silde_Mode_Controller.h"
#include "usart.h"
#include <string.h>
#include <math.h>

// 全局控制器输入输出变量的定义
extern ControllerInput ctrl_input;
extern ControllerInput prev_ctrl_input;
extern ControllerOutput ctrl_output;
TIM_HandleTypeDef TIM3_Handler; // 定时器句柄

// 通用定时器3中断初始化, 定时器3在APB1上，APB1的定时器时钟为200MHz
// arr：自动重装值。
// psc：时钟预分频数
// 定时器溢出时间计算方法: Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率, 单位:Mhz
// 这里使用的是定时器3!(定时器3挂在APB1上，时钟为HCLK/2)
void TIM3_Init(u16 arr, u16 psc)
{
    TIM3_Handler.Instance = TIM3;                             // 通用定时器3
    TIM3_Handler.Init.Prescaler = psc;                        // 分频
    TIM3_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;       // 向上计数器
    TIM3_Handler.Init.Period = arr;                           // 自动装载值
    TIM3_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频因子
    HAL_TIM_Base_Init(&TIM3_Handler);

    HAL_TIM_Base_Start_IT(&TIM3_Handler); // 使能定时器3和定时器3更新中断：TIM_IT_UPDATE
}

// 定时器底册驱动，开启时钟，设置中断优先级
// 此函数会被HAL_TIM_Base_Init()函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();           // 使能TIM3时钟
        HAL_NVIC_SetPriority(TIM3_IRQn, 1, 3); // 设置中断优先级，抢占优先级1，子优先级3
        HAL_NVIC_EnableIRQ(TIM3_IRQn);         // 开启TIM3中断
    }
}

// 定时器3中断服务函数
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM3_Handler);
}

// 定时器3中断服务函数调用
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&TIM3_Handler))
    {

        // 调用当前周期控制输入结构体，输入位置和姿态控制器计算控制输出
        Position_Controller(&ctrl_input, &ctrl_output);
        Attitude_Controller(&ctrl_input, &ctrl_output);

        // 应用控制器输出
        FanControl(&ctrl_output);
    }
}
