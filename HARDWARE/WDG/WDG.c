#include "WDG.h"
#include "Fan.h" /* 用到安全停机函数 */
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_iwdg.h"

TIM_HandleTypeDef htim7;
IWDG_HandleTypeDef hiwdg;

/* --- 软件看门狗 ------------------------------------------------ */
volatile uint8_t pgd_abort_flag = 0;  /* 通知 PGD 立即退出 */
volatile uint8_t ctrl_abort_flag = 0; /* TIM6 控制环退出 */

uint32_t sw_reset_cnt = 0;

void SoftWDG_Init(void)
{
    __HAL_RCC_TIM7_CLK_ENABLE();
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 2399U; // 240MHz/(2399+1) = 100000Hz
    htim7.Init.Period = 2999U;    // (2999+1)/100000 = 30 ms
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 时钟分频因子
    HAL_TIM_Base_Init(&htim7);
    HAL_TIM_Base_Start(&htim7);
    __HAL_TIM_SET_COUNTER(&htim7, 0); // 防止上电计数残留
    HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

void SoftWDG_Kick(void)
{
    __HAL_TIM_SET_COUNTER(&htim7, 0U); /* 重新装载 0 */
}

void TIM7_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE))
    {
        __HAL_TIM_CLEAR_IT(&htim7, TIM_IT_UPDATE);

        /* --- 软复位动作：安全停车 & 记录故障计数 ---------------- */
        ++sw_reset_cnt;
#if SEND_DETAIL
        USART_SendFormatted_DMA("Soft WDG reset! cnt=%lu\r\n", sw_reset_cnt);
#endif
        pgd_abort_flag = 1;  // 设置 PGD 终止标志
        ctrl_abort_flag = 1; // 终止 TIM6 控制器
        /* 让主循环继续运行并喂 IWDG；如长时间卡死则 IWDG 复位 */
    }
}

/* --- 硬件 IWDG ------------------------------------------------ */

void IWDG_Init(void)
{
    hiwdg.Instance = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_64; // 64 预分频
    hiwdg.Init.Reload = 75;                   // Tout=((4*2^Prescaler)*Reload) /32 = 64*75/32k = 0.150 s
    hiwdg.Init.Window = IWDG_WINDOW_DISABLE;  // 关闭窗口功能
    HAL_IWDG_Init(&hiwdg);
}

void IWDG_Kick(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}
