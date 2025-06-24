#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "usart.h"
#include "Silde_Mode_Controller.h"

#define ctrl_arr 999  // 计数器自动重装载值
#define ctrl_psc 2399 // 分频

extern volatile uint8_t dbg_flag; // 用于控制调试输出的标志位

extern TIM_HandleTypeDef TIM3_Handler; // 定时器句柄

// 定时器自动重装值arr和时钟预分频数psc
extern uint16_t CTRL_ARR; // 控制周期时长为：Tout=((CTRL_ARR+1)*(CTRL_PSC+1))/Ft =10ms. Ft=定时器工作频率, 单位:Mhz
extern uint16_t CTRL_PSC;
extern uint16_t PWM_ARR; // PWM输出频率为：Ft=((PWM_ARR+1)*(PWM_PSC+1))/Tout 25kHz.
extern uint16_t PWM_PSC;

// 函数声明
void TIM6_Init(void); // 定时器3初始化函数
// void Set_Fan_PWM(float duty_rate[12]); // 设置风扇PWM输出占空比
void TIMx_PWM_Init(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef *sConfigOC);      // 定时器PWM初始化函数
void PWM_Init(void);                                                             // PWM初始化函数
void TIM_SetCompare(TIM_HandleTypeDef *htim, uint32_t Channel, float duty_rate); // 设置定时器比较值

#endif
