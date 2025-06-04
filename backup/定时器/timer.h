#ifndef _TIMER_H
#define _TIMER_H
#include "sys.h"
#include "Silde_Mode_Controller.h"

extern TIM_HandleTypeDef TIM3_Handler; // 定时器句柄

// 函数声明
void TIM3_Init(u16 arr, u16 psc); // 定时器3初始化函数

#endif
