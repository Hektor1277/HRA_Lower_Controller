#ifndef WATCHDOG_H
#define WATCHDOG_H
#include "stm32h7xx_hal.h"

extern TIM_HandleTypeDef htim7;
extern IWDG_HandleTypeDef hiwdg;

extern uint32_t sw_reset_cnt; // 软复位次数

extern volatile uint8_t pgd_abort_flag;
extern volatile uint8_t ctrl_abort_flag; /* 供 TIM6 ISR 检测 */

/* 30 ms 软件看门狗 by TIM7 */
void SoftWDG_Init(void); /* 开启 TIM7 30 ms 周期 */
void SoftWDG_Kick(void); /* 每帧调用喂狗        */

/* 150 ms 硬件 IWDG */
void IWDG_Init(void);
void IWDG_Kick(void);

#endif
