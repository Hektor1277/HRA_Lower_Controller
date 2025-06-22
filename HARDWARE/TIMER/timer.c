#include "timer.h"
#include "Silde_Mode_Controller.h"
#include "Fan.h"
#include "usart.h"
#include <string.h>
#include <math.h>

// 定义控制器输入输出, 风扇转速和占空比结构体
extern ControllerInput ctrl_input;
extern ControllerInput prev_ctrl_input;
extern ControllerOutput ctrl_output;
extern FanSpeed Fan_desire_Speed;
extern FanControl Fan_Control_duty_rate;

volatile uint8_t dbg_flag = 0; // 用于控制调试输出的标志位

// 定时器自动重装值arr和时钟预分频数psc
uint16_t CTRL_ARR = ctrl_arr; // 运行模式控制周期时长为：Tout=((CTRL_ARR+1)*(CTRL_PSC+1))/Ft =10ms. Ft=定时器工作频率, 单位:Mhz; 调试模式为1000ms.
uint16_t CTRL_PSC = ctrl_psc;
uint16_t PWM_ARR = 959; // PWM输出频率为：Ft=((TIM_arr+1)*(TIM_psc+1))/Tout 25kHz.
uint16_t PWM_PSC = 9;

TIM_HandleTypeDef TIM6_Handler; // 基本定时器6句柄，用作控制周期定时器

TIM_HandleTypeDef TIM3_Handler; // 定时器3句柄
TIM_HandleTypeDef TIM4_Handler; // 定时器4句柄
TIM_HandleTypeDef TIM5_Handler; // 定时器5句柄
TIM_HandleTypeDef TIM8_Handler; // 定时器8句柄

TIM_OC_InitTypeDef TIM3_CHHandler; // 定时器3通道句柄
TIM_OC_InitTypeDef TIM4_CHHandler; // 定时器4通道句柄
TIM_OC_InitTypeDef TIM5_CHHandler; // 定时器5通道句柄
TIM_OC_InitTypeDef TIM8_CHHandler; // 定时器8通道句柄

// ⭐定时器6中断服务函数调用, 控制器主控制循环
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&TIM6_Handler))
    {
        dbg_flag = 1; // 一行即可
        // 定时器中断触发调用控制器/风扇计算
        // 调用当前周期控制输入结构体，输入位置和姿态控制器计算控制输出
        Position_Controller(&ctrl_input, &ctrl_output);
        Attitude_Controller(&ctrl_input, &ctrl_output);

        // 调用底层风扇控制函数，根据控制输出计算风扇转速并输出相应PWM信号
        Fan_Rotation_Control(&ctrl_output, &Fan_desire_Speed, &Fan_Control_duty_rate);
    }
}

// 基本定时器6中断初始化，用作控制周期定时器
// arr：自动重装值。
// psc：时钟预分频数
// 定时器溢出时间计算方法: Tout=((arr+1)*(psc+1))/Ft us.
// Ft=定时器工作频率, 单位:Mhz
void TIM6_Init(void)
{
    TIM6_Handler.Instance = TIM6;                             // 基本定时器6
    TIM6_Handler.Init.Prescaler = CTRL_PSC;                   // 分频
    TIM6_Handler.Init.CounterMode = TIM_COUNTERMODE_UP;       // 向上计数器
    TIM6_Handler.Init.Period = CTRL_ARR;                      // 自动装载值
    TIM6_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频因子

    HAL_TIM_Base_Init(&TIM6_Handler);

    HAL_TIM_Base_Start_IT(&TIM6_Handler); // 使能定时器和更新中断：TIM_IT_UPDATE
}

// 定时器底册驱动，开启时钟，设置中断优先级
// 此函数会被 HAL_TIM_Base_Init() 函数调用
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        __HAL_RCC_TIM6_CLK_ENABLE();               // 使能 TIM6 时钟
        HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 1); // 设置中断优先级，抢占优先级 1，子优先级 1
        HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);         // 开启 TIM6 中断
    }
}

// 定时器6中断服务函数
void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&TIM6_Handler);
}

// PWM输出定时器 PWM 部分初始化函数
void TIMx_PWM_Init(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfigOC)
{
    htim->Init.Prescaler = PWM_PSC;                    // 定时器分频
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;       // 向上计数模式
    htim->Init.Period = PWM_ARR;                       // 自动重装载值
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频因子

    HAL_TIM_PWM_Init(htim); // 初始化PWM

    sConfigOC->OCMode = TIM_OCMODE_PWM1;               // 模式选择PWM1
    sConfigOC->Pulse = (uint32_t)round(PWM_ARR * 0.1); // 设置初始比较值，四舍五入为 10%
    sConfigOC->OCPolarity = TIM_OCPOLARITY_HIGH;       // 输出比较极性为高
}

// 初始化所有PWM通道
void PWM_Init(void)
{

    TIM3_Handler.Instance = TIM3;
    TIMx_PWM_Init(&TIM3_Handler, &TIM3_CHHandler);
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler, &TIM3_CHHandler, TIM_CHANNEL_1); // PA6、7和PB0、1对应通道为1、2、3、4
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler, &TIM3_CHHandler, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler, &TIM3_CHHandler, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&TIM3_Handler, &TIM3_CHHandler, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&TIM3_Handler, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM3_Handler, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM3_Handler, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM3_Handler, TIM_CHANNEL_4);

    TIM4_Handler.Instance = TIM4;
    TIMx_PWM_Init(&TIM4_Handler, &TIM4_CHHandler);
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &TIM4_CHHandler, TIM_CHANNEL_2); // PB7、8、9对应通道为2、3、4
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &TIM4_CHHandler, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&TIM4_Handler, &TIM4_CHHandler, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM4_Handler, TIM_CHANNEL_4);

    TIM5_Handler.Instance = TIM5;
    TIMx_PWM_Init(&TIM5_Handler, &TIM5_CHHandler);
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler, &TIM5_CHHandler, TIM_CHANNEL_1); // PH10、11对应通道为1、2
    HAL_TIM_PWM_ConfigChannel(&TIM5_Handler, &TIM5_CHHandler, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM5_Handler, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM5_Handler, TIM_CHANNEL_2);

    TIM8_Handler.Instance = TIM8;
    TIMx_PWM_Init(&TIM8_Handler, &TIM8_CHHandler);
    HAL_TIM_PWM_ConfigChannel(&TIM8_Handler, &TIM8_CHHandler, TIM_CHANNEL_1); // PI5、6、7对应通道为1、2、3
    HAL_TIM_PWM_ConfigChannel(&TIM8_Handler, &TIM8_CHHandler, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&TIM8_Handler, &TIM8_CHHandler, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&TIM8_Handler, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&TIM8_Handler, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&TIM8_Handler, TIM_CHANNEL_3);
}

// PWM输出定时器底层驱动，时钟使能，引脚配置
// 此函数会被 HAL_TIM_PWM_Init() 调用
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_Initure;

    if (htim->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();                    // 使能 TIM3 时钟
        __HAL_RCC_GPIOA_CLK_ENABLE();                   // 开启 GPIOA 时钟
        __HAL_RCC_GPIOB_CLK_ENABLE();                   // 开启 GPIOB 时钟
        GPIO_Initure.Pin = GPIO_PIN_6 | GPIO_PIN_7;     // PA6, PA7
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;            // 复用推挽输出
        GPIO_Initure.Pull = GPIO_PULLUP;                // 上拉
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 高速
        GPIO_Initure.Alternate = GPIO_AF2_TIM3;         // 复用为 TIM3_CH1, CH2
        HAL_GPIO_Init(GPIOA, &GPIO_Initure);

        GPIO_Initure.Pin = GPIO_PIN_0 | GPIO_PIN_1; // PB0, PB1
        HAL_GPIO_Init(GPIOB, &GPIO_Initure);
    }
    else if (htim->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        GPIO_Initure.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9; // PB7, PB8, PB9
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_Initure.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_Initure);
    }
    else if (htim->Instance == TIM5)
    {
        __HAL_RCC_TIM5_CLK_ENABLE();
        __HAL_RCC_GPIOH_CLK_ENABLE();
        GPIO_Initure.Pin = GPIO_PIN_10 | GPIO_PIN_11; // PH10, PH11
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;
        GPIO_Initure.Pull = GPIO_PULLUP;
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_Initure.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOH, &GPIO_Initure);
    }
    else if (htim->Instance == TIM8)
    {
        __HAL_RCC_TIM8_CLK_ENABLE();                             // 使能 TIM8 时钟
        __HAL_RCC_GPIOI_CLK_ENABLE();                            // 开启 GPIOI 时钟
        GPIO_Initure.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7; // PI5, PI6, PI7
        GPIO_Initure.Mode = GPIO_MODE_AF_PP;                     // 复用推挽输出
        GPIO_Initure.Pull = GPIO_PULLUP;                         // 上拉
        GPIO_Initure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;          // 高速
        GPIO_Initure.Alternate = GPIO_AF3_TIM8;                  // 复用为 TIM8_CH1, CH2, CH3
        HAL_GPIO_Init(GPIOI, &GPIO_Initure);
    }
}

// 设置 TIM 通道的占空比
void TIM_SetCompare(TIM_HandleTypeDef *htim, uint32_t Channel, float duty_rate)
{
    uint32_t pulse = (uint32_t)round((duty_rate / 100.0f) * (htim->Init.Period + 1));
    __HAL_TIM_SET_COMPARE(htim, Channel, pulse);
}
