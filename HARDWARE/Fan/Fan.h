#ifndef _FAN_H
#define _FAN_H
#include "sys.h"
#include "Silde_Mode_Controller.h"
#include <stdbool.h>

// 风扇参数定义
#define Fan_Tension_Coefficient 4.3882e-10 // 拉力系数c_T
#define LS_Slope 411.1852                  // 10-20占空比线性拟合斜率
#define LS_Intercept -3370.9               // 10-20占空比线性拟合截距
#define Turning_point_speed_1 4882         // 20占空比交界转速
#define MS_Slope 299.1938                  // 20-40占空比线性拟合斜率
#define MS_Intercept -1101.7               // 20-40占空比线性拟合截距
#define Turning_point_speed_2 10885        // 40占空比交界转速
#define HS_Slope 217.8695                  // 40-90占空比线性拟合斜率
#define HS_Intercept 2170.9                // 40-90占空比线性拟合截距

#define MAX_SPEED 22000.0
#define MAX_SPEED_SQUARE 484000000.0 // 最大转速平方值, 单位为(RPM)^2. 所使用的风扇额定转速为22000RPM

#define NUM_FANS 12  // 风扇数量
#define NUM_FORCES 6 // 力和力矩分量数

#define LEARNING_RATE 0.1 // 初始学习率
#define TOLERANCE 1e-7    // 收敛阈值
#define MAX_ITERATIONS 10 // 最大迭代次数

// 本课题机器人风扇参数, 数据由实验测得
static const double c_T = Fan_Tension_Coefficient; // 拉力系数, 由于其值较大, 因此(1)的计算使用double以保证精度
static const float c_R_H = HS_Slope;               //(2)式的计算使用float即可保证足够精度
static const float ω_b_H = HS_Intercept;
static const float c_R_M = MS_Slope;
static const float ω_b_M = MS_Intercept;
static const float c_R_L = LS_Slope;
static const float ω_b_L = LS_Intercept;
static const double D_val = D;

// // 参数定义
// extern const double c_T; // 拉力系数, 由于其值较大, 因此(1)的计算使用double以保证精度
// extern const float c_R;  //(2)式的计算使用float即可保证足够精度
// extern const float ω_b;

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
