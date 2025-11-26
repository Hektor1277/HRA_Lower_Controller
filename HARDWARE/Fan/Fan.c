#include "Fan.h"
#include "timer.h"
#include "serial.h"
#include "PGD.h"
#include <math.h>

/////////////////////////////////////////////////////////////////////////////////////////////
//
// 风扇底层驱动程序，实现控制器所计算的推力、扭矩到底层风扇的驱动PWM信号
// 整个底层风扇控制分为两个步骤: 期望推力扭矩到各风扇的控制分配; 各风扇期望推力到实际PWM信号的占空比。
// 控制分配, 控制器计算的期望推力和力矩是针对整个机器人而言, 对于具体的驱动器, 也就是风扇, 还需要进行控制分配。
// 控制分配的方式: 通过控制分配矩阵进行计算, 控制分配矩阵则与风扇的物理布局有关。
// 矩阵形式如下所示, 具体推导参见相关文件：
//                                                                                                                          [ ϖ^2_LX+ ]
//                                                                                                                          [ ϖ^2_LX- ]
//                                                                                                                          [ ϖ^2_RX+ ]
// [ Fx^b  ]   [-c_T      c_T     -c_T    c_T       0        0        0        0       0         0        0        0      ] [ ϖ^2_RX- ]
// [ Fy^b  ]   [ 0        0        0       0       -c_T      c_T     -c_T      c_T      0        0        0        0      ] [ ϖ^2_FY+ ]
// [ Fz^b  ] = [ 0        0        0       0        0        0        0        0       -c_T      c_T     -c_T      c_T    ]*[ ϖ^2_FY- ]
// [ T_Φ^b ]   [ 0        0        0       0        D * c_T  D * c_T -D * c_T -D * c_T -D * c_T  D * c_T  D * c_T -D * c_T] [ ϖ^2_AY+ ]
// [ T_Θ^b ]   [-D * c_T -D * c_T  D * c_T D * c_T  0        0        0        0        D * c_T  D * c_T -D * c_T -D * c_T] [ ϖ^2_AY- ]
// [ T_Ψ^b ]   [ D * c_T -D * c_T -D * c_T D * c_T -D * c_T  D * c_T  D * c_T -D * c_T  0        0        0        0      ] [ ϖ^2_LZ+ ]
//                                                                                                                          [ ϖ^2_LZ- ]
//                                                                                                                          [ ϖ^2_RZ+ ]
//                                                                                                                          [ ϖ^2_RZ- ]
// 硬编码原始矩阵(行主序)
// static const double M_origin[72] = {
//     -c_T, c_T, -c_T, c_T, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0, -c_T, c_T, -c_T, c_T, 0.0, 0.0, 0.0, 0.0,
//     0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -c_T, c_T, -c_T, c_T,
//     0.0, 0.0, 0.0, 0.0, D_val *c_T, D_val *c_T, -D_val *c_T, -D_val *c_T, -D_val *c_T, D_val *c_T, D_val *c_T, -D_val *c_T,
//     -D_val *c_T, -D_val *c_T, D_val *c_T, D_val *c_T, 0.0, 0.0, 0.0, 0.0, D_val *c_T, D_val *c_T, -D_val *c_T, -D_val *c_T,
//     D_val *c_T, -D_val *c_T, -D_val *c_T, D_val *c_T, -D_val *c_T, D_val *c_T, D_val *c_T, -D_val *c_T, 0.0, 0.0, 0.0, 0.0};
//
// 推力到占空比的流程是：根据输入的各风扇期望推力F_i, 计算各风扇转速的平方, 开方后得到期望转速，再根据公式(2)计算出油门信号，最后输出到各风扇。
// 两个重要公式: 推力和转速的平方曲线; 转速和油门信号(PWM信号占空比)曲线。由这两个公式即可实现期望推力输出到风扇控制。
//          F_i = c_T * ω_i ^ 2     (1)             F_i为第i个风扇的推力，单位为N; c_T为实验测量数据, 称拉力系数; ω_i为第i个风扇的转速。
//          ω_i = c_R * σ_i + ω_b   (2)             σ_i为油门信号，范围为0~1, ω_b为基准转速。
//
//
// 根据实验所测得数据:c_T = 4.375e-08, c_R =  235.1, ω_b = 589.2
/////////////////////////////////////////////////////////////////////////////////////////////

// 定义控制器输入输出结构体
extern ControllerInput ctrl_input;
extern ControllerInput prev_ctrl_input;
extern ControllerOutput ctrl_output;
volatile CtrlBuf ctrl_buf[2] = {0};
volatile uint8_t ctrl_w = 0, ctrl_r = 0;
// 定义风扇转速和占空比结构体
FanSpeed Fan_desire_Speed = {0};
FanControl Fan_Control_duty_rate = {0};

extern TIM_HandleTypeDef htim3; // 定时器3句柄
extern TIM_HandleTypeDef htim4; // 定时器4句柄
extern TIM_HandleTypeDef htim5; // 定时器5句柄
extern TIM_HandleTypeDef htim8; // 定时器8句柄

extern TIM_OC_InitTypeDef TIM3_CHHandler; // 定时器3通道句柄
extern TIM_OC_InitTypeDef TIM4_CHHandler; // 定时器4通道句柄
extern TIM_OC_InitTypeDef TIM5_CHHandler; // 定时器5通道句柄
extern TIM_OC_InitTypeDef TIM8_CHHandler; // 定时器8通道句柄

uint32_t solution_accepted_cnt = 0; // 接受解计数

// 定义底层风扇控制前向通道
void Fan_Rotation_Control(ControllerOutput *output, FanSpeed *fan_speed, FanControl *duty_rate)
{
    // 计算风扇转速
    Calculate_Fan_Speed(output, fan_speed);
    // 计算各风扇PWM信号占空比
    Calculate_Duty_Rate(fan_speed, duty_rate);
    // 设置各风扇对应通道PWM输出占空比
    Set_Fan_PWM(duty_rate);
#if SEND_DETAIL
    USART_SendFormatted_DMA("Fan Speed calculated and PWM duty cycle set.\r\n");
#endif
}

// 定义计算风扇转速的函数
void Calculate_Fan_Speed(ControllerOutput *output, FanSpeed *fan_speed)
{

    //  对方程进行归一化, 使得求解数值稳定
    //  F/Fmax = (M / (Fmax * cT)) * (cT*ω)
    //  从output中取出期望F向量(6x1), 并进行归一化: Fx,Fy,Fz,Tphi,Ttheta,Tpsi
    double F_origin[6];                   // 从输出结构体中提取 F 向量
    double F_norm[6];                     // 归一化的 F 向量
    double F_proj[6];                     // 用于存储归一化后的 F 向量在控制效率矩阵列空间的投影
    static double PGD_solution[12] = {0}; // 用于存储 PGD 解
    float de_norm_PGD[12] = {0};          // 用于存储反归一化后的 PGD 解
    int F_status = 0;                     // 存储 F 是否在列空间中的状态
    int accept_solution = 0;              // 存储是否接受解的状态

#if SEND_DETAIL
    USART_SendFormatted_DMA("PGD input Thrust: [%.5f, %.5f, %.5f]\r\n", output->thrust[0], output->thrust[1], output->thrust[2]);
    USART_SendFormatted_DMA("PGD input Torque: [%.5f, %.5f, %.5f]\r\n", output->torque[0], output->torque[1], output->torque[2]);
#endif

    for (int i = 0; i < 3; i++) // 从控制器输出合力中提取 F 向量
    {
        F_origin[i] = output->thrust[i];     // 单位N,取值范围[-0.4,0.4]
        F_origin[i + 3] = output->torque[i]; // 单位N·m,取值范围[-0.0568,0.0568]
    }

#if SEND_DETAIL
    USART_SendFormatted_DMA("PGD input F_norm:\r\n[");
#endif

    /***********************************************************************
     * 双极值归一化说明
     *  1. 力 (Fx,Fy,Fz)   归一化因子：FMAX_FORCE = 2*FMAX  ≈ 0.40 N
     *  2. 力矩(Tφ,Tθ,Tψ)  归一化因子：FMAX_TORQUE = 4*D*FMAX ≈ 0.0568 N·m
     *
     * 这样六维 F_norm 均落在 [-1,1]，PGD 目标函数的各分量权重相近，
     * 既提升数值条件，也使残差阈值在力/力矩两类通道上具有一致物理意义。
     * 反归一化仅作用于推力→转速的平方关系，与本缩放互不耦合。
     **********************************************************************/

    for (int i = 0; i < 6; i++) // 归一化 F 向量, 力/力矩分别除以各自极值，保证六维量纲一致
    {
        F_norm[i] = F_origin[i] / (i < 3 ? (FMAX_FORCE) : (FMAX_TORQUE)); // 单位N,取值范围[-0.4,0.4] → [-1, 1]; 单位N·m,取值范围[-0.0568,0.0568] → [-1, 1]
#if SEND_DETAIL
        USART_SendFormatted_DMA(" %.3f ", F_norm[i]);
#endif
    }
#if SEND_DETAIL
    USART_SendFormatted_DMA("]\r\n");
#endif
    project_target(F_norm, F_proj); // 将归一化后的 F 向量投影至控制效率矩阵列空间
#if SEND_DETAIL
    print_vector("Projected Target F_proj", F_proj, M_ROWS);
#endif

    // Step 2: 判断 F 是否在列空间中, 根据是否在列空间中来决定接收解的残差阈值
    F_status = is_in_column_space(F_norm, F_proj);

    // Step 3: 使用投影梯度下降法求解
#if SEND_DETAIL
    USART_SendFormatted_DMA("\r\nRunning Projected Gradient Descent...\r\n");
#endif
    uint32_t pgd_start = DWT->CYCCNT;

    projected_gradient_descent(F_proj, PGD_solution);

    uint32_t dur = CYCLES_ELAPSED(DWT->CYCCNT, pgd_start);
    if (dur > pgd_max_cycles)
        pgd_max_cycles = dur;
    pgd_acc_cycles += dur;
    ++pgd_cnt;

    // Step 4: 计算解的残差, 并根据残差判断是否接受解
#if SEND_DETAIL
    USART_SendFormatted_DMA("\r\nVerifying PGD Solution...\r\n");
#endif
    compute_residual(F_norm, PGD_solution, F_status, &accept_solution);

    if (accept_solution) // 通过解的残差判断是否接受解, 并打印最终解向量 x
    {
        solution_accepted_cnt++;
        Denormalize_solution(PGD_solution, de_norm_PGD);
        accept_PGD_solution(fan_speed, de_norm_PGD);
    }
}

void accept_PGD_solution(FanSpeed *fan_speed, float *de_norm_PGD)
{
    // 若通过检验, 则接受PGD解, 将反归一化后的PGD解存入fan_speed结构体
    fan_speed->omega_LX_p = de_norm_PGD[0];
    fan_speed->omega_LX_n = de_norm_PGD[1];
    fan_speed->omega_RX_p = de_norm_PGD[2];
    fan_speed->omega_RX_n = de_norm_PGD[3];
    fan_speed->omega_FY_p = de_norm_PGD[4];
    fan_speed->omega_FY_n = de_norm_PGD[5];
    fan_speed->omega_AY_p = de_norm_PGD[6];
    fan_speed->omega_AY_n = de_norm_PGD[7];
    fan_speed->omega_LZ_p = de_norm_PGD[8];
    fan_speed->omega_LZ_n = de_norm_PGD[9];
    fan_speed->omega_RZ_p = de_norm_PGD[10];
    fan_speed->omega_RZ_n = de_norm_PGD[11];
}

float Calculate_Single_Duty(float fan_speed)
{
    float duty;

    if (fan_speed <= Turning_point_speed_1)
    {
        duty = (fan_speed - ω_b_L) / c_R_L;
    }
    else if (fan_speed <= Turning_point_speed_2)
    {
        duty = (fan_speed - ω_b_M) / c_R_M;
    }
    else
    {
        duty = (fan_speed - ω_b_H) / c_R_H;
    }

    return (float)(custom_fmin(MAX_FAN_Duty_Rate, custom_fmax(MIN_FAN_Duty_Rate, duty)));
}

void Calculate_Duty_Rate(FanSpeed *fan_speed, FanControl *duty_rate)
{
    // 根据公式(2)计算占空比, 并限制在0.1~0.9之间。然后放大100倍, 作为占空比(10-90%)
    duty_rate->control_LX_p = Calculate_Single_Duty(fan_speed->omega_LX_p);
    duty_rate->control_LX_n = Calculate_Single_Duty(fan_speed->omega_LX_n);
    duty_rate->control_RX_p = Calculate_Single_Duty(fan_speed->omega_RX_p);
    duty_rate->control_RX_n = Calculate_Single_Duty(fan_speed->omega_RX_n);
    duty_rate->control_FY_p = Calculate_Single_Duty(fan_speed->omega_FY_p);
    duty_rate->control_FY_n = Calculate_Single_Duty(fan_speed->omega_FY_n);
    duty_rate->control_AY_p = Calculate_Single_Duty(fan_speed->omega_AY_p);
    duty_rate->control_AY_n = Calculate_Single_Duty(fan_speed->omega_AY_n);
    duty_rate->control_LZ_p = Calculate_Single_Duty(fan_speed->omega_LZ_p);
    duty_rate->control_LZ_n = Calculate_Single_Duty(fan_speed->omega_LZ_n);
    duty_rate->control_RZ_p = Calculate_Single_Duty(fan_speed->omega_RZ_p);
    duty_rate->control_RZ_n = Calculate_Single_Duty(fan_speed->omega_RZ_n);
}

// 控制风扇占空比，接收 12 个风扇的占空比(MIN_FAN_Duty_Rate%-MAX_FAN_Duty_Rate%), 并输出相应PWM波
// PWM输出引脚分配: 用于输出PWM信号的引脚按风扇模块分布于P1P2, 并按核心模块朝向进行分配, 当前引脚分配为:
// PI5, 6, 7为左上模块(编号1, LX+, FY+, LZ+) PB7, 8, 9为左下模块(编号3, LX-, AY+, LZ-); PB0, 1和PA7为右上模块(编号2, RX-, AY-, RZ+)PA6和PH10, 11为右下模块(编号4, RX+, FY-, RZ-)
// 结合定时器引脚分配: 定时器3为PA6, 7和PB0, 1; 定时器4为PB7, 8, 9; 定时器5为PH10, 11; 定时器8为PI5, 6, 7
void Set_Fan_PWM(FanControl *duty_rate)
{
    // 设置左上模块(编号1, LX+, FY+, LZ+, 对应PI5, 6, 7)的三个通道的占空比
    TIM_SetCompare(&htim8, TIM_CHANNEL_1, duty_rate->control_LX_p); // 定时器8通道1为PI5, 对应左上模块的LX+
    TIM_SetCompare(&htim8, TIM_CHANNEL_2, duty_rate->control_FY_p); // 定时器8通道2为PI6, 对应左上模块的FY+
    TIM_SetCompare(&htim8, TIM_CHANNEL_3, duty_rate->control_LZ_p); // 定时器8通道3为PI7, 对应左上模块的LZ+

    // 设置右上模块(编号2, RX-, AY-, RZ+, 对应PA7和PB0, 1)三个通道的占空比
    TIM_SetCompare(&htim3, TIM_CHANNEL_2, duty_rate->control_RX_n); // 定时器3通道2为PA7, 对应右上模块的RX-
    TIM_SetCompare(&htim3, TIM_CHANNEL_3, duty_rate->control_AY_n); // 定时器3通道3为PB0, 对应右上模块的AY-
    TIM_SetCompare(&htim3, TIM_CHANNEL_4, duty_rate->control_RZ_p); // 定时器3通道4为PB1, 对应右上模块的RZ+

    // 设置左下模块(编号3, LX-, AY+, LZ-, 对应PB7, 8, 9)三个通道的占空比
    TIM_SetCompare(&htim4, TIM_CHANNEL_2, duty_rate->control_LX_n); // 定时器4通道2为PB7, 对应左下模块的LX-
    TIM_SetCompare(&htim4, TIM_CHANNEL_3, duty_rate->control_AY_p); // 定时器4通道3为PB8, 对应左下模块的AY+
    TIM_SetCompare(&htim4, TIM_CHANNEL_4, duty_rate->control_LZ_n); // 定时器4通道4为PB9, 对应左下模块的LZ-

    // 设置右下模块(编号4, RX+, FY-, RZ-, 对应PA6和PH10, 11)三个通道的占空比
    TIM_SetCompare(&htim5, TIM_CHANNEL_2, duty_rate->control_RX_p); // 定时器5通道2为PH11, 对应右下模块的RX+
    TIM_SetCompare(&htim5, TIM_CHANNEL_1, duty_rate->control_FY_n); // 定时器5通道1为PH10, 对应右下模块的FY-
    TIM_SetCompare(&htim3, TIM_CHANNEL_1, duty_rate->control_RZ_n); // 定时器3通道1为PA6, 对应右下模块的RZ-
}
