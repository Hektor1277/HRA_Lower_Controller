#ifndef _FAN_H
#define _FAN_H
#include "sys.h"
#include "Silde_Mode_Controller.h"
#include <stdbool.h>
#include "config.h"

// 双极值归一化(含安全系数)
#define FMAX_FORCE 2 * FMAX * 1.1      // 单轴最大推力 (2*FMAX) * 1.1 (N)0.44
#define FMAX_TORQUE 4 * D * FMAX * 1.1 // 单轴最大力矩 (4*D*FMAX) * 1.1 (N·m)0.0625

extern uint32_t solution_accepted_cnt; // 接受解计数

// 风扇拉力-占空比分段拟合参数
static const double c_T = Fan_Tension_Coefficient; // 拉力系数, 由于其值较大, 因此(1)的计算使用double以保证精度
static const float c_R_H = HS_Slope;               //(2)式的计算使用float即可保证足够精度
static const float ω_b_H = HS_Intercept;
static const float c_R_M = MS_Slope;
static const float ω_b_M = MS_Intercept;
static const float c_R_L = LS_Slope;
static const float ω_b_L = LS_Intercept;
static const double D_val = D;

// 定义风扇转速结构体
typedef struct
{
    // 期望值
    float omega_LX_p; // 标签LX+风扇转速
    float omega_LX_n; // 标签LX-风扇转速
    float omega_RX_p; // 标签RX+风扇转速
    float omega_RX_n; // 标签RX-风扇转速
    float omega_FY_p; // 标签FY+风扇转速
    float omega_FY_n; // 标签FY-风扇转速
    float omega_AY_p; // 标签AY+风扇转速
    float omega_AY_n; // 标签AY-风扇转速
    float omega_LZ_p; // 标签LZ+风扇转速
    float omega_LZ_n; // 标签LZ-风扇转速
    float omega_RZ_p; // 标签RZ+风扇转速
    float omega_RZ_n; // 标签RZ-风扇转速
} FanSpeed;

// 定义风扇PWM输出占空比结构体
typedef struct
{
    // 期望值
    float control_LX_p; // 标签LX+风扇PWM占空比
    float control_LX_n; // 标签LX-风扇PWM占空比
    float control_RX_p; // 标签RX+风扇PWM占空比
    float control_RX_n; // 标签RX-风扇PWM占空比
    float control_FY_p; // 标签FY+风扇PWM占空比
    float control_FY_n; // 标签FY-风扇PWM占空比
    float control_AY_p; // 标签AY+风扇PWM占空比
    float control_AY_n; // 标签AY-风扇PWM占空比
    float control_LZ_p; // 标签LZ+风扇PWM占空比
    float control_LZ_n; // 标签LZ-风扇PWM占空比
    float control_RZ_p; // 标签RZ+风扇PWM占空比
    float control_RZ_n; // 标签RZ-风扇PWM占空比
} FanControl;

// 双缓冲结构
typedef struct
{
    ControllerOutput data;
    volatile bool valid;
} CtrlBuf;

extern volatile CtrlBuf ctrl_buf[2];
extern volatile uint8_t ctrl_w, ctrl_r;

extern FanSpeed Fan_desire_Speed;
extern FanControl Fan_Control_duty_rate;

// 函数声明
void Fan_Rotation_Control(ControllerOutput *output, FanSpeed *fan_speed, FanControl *duty_rate); // 底层风扇控制前向通道
void Calculate_Fan_Speed(ControllerOutput *output, FanSpeed *fan_speed);                         // 计算风扇转速
void accept_PGD_solution(FanSpeed *fan_speed, float *de_norm_PGD);
void Calculate_Duty_Rate(FanSpeed *fan_speed, FanControl *duty_rate); // 计算风扇PWM信号占空比
float Calculate_Single_Duty(float fan_speed);                         // 计算单个风扇占空比
void Set_Fan_PWM(FanControl *FanControl);                             // 设置各通道PWM输出占空比

#endif
