#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "sys.h"
#include "config.h"
// #include "arm_math.h" // 引入CMSIS-DSP库

// 定义控制器输入结构体，包括期望和实际值
typedef struct
{
    // 期望值
    float desired_position[3];             // 期望x, y, z 位置
    float desired_velocity[3];             // 期望x, y, z 速度
    float desired_acceleration[3];         // 期望x, y, z 加速度
    float desired_angles[3];               // 期望phi, theta, psi 角度
    float desired_angular_velocity[3];     // 期望角速度
    float desired_angular_acceleration[3]; // 期望角加速度

    // 实际值
    float position[3];             // 实际x, y, z 位置
    float velocity[3];             // 实际x, y, z 速度
    float acceleration[3];         // 实际x, y, z 加速度
    float angles[3];               // 实际phi, theta, psi 角度
    float angular_velocity[3];     // 实际角速度
    float angular_acceleration[3]; // 实际角加速度
} ControllerInput;

// 定义控制器输出结构体
typedef struct
{
    float thrust[3]; // x, y, z 推力
    float torque[3]; // phi, theta, psi 力矩
} ControllerOutput;

// 声明控制器输入输出结构体
extern ControllerInput ctrl_input;
extern ControllerInput prev_ctrl_input;
extern ControllerOutput ctrl_output;
extern float Rsb[3][3]; // 全局变量旋转矩阵 Rsb

// 函数声明
void Position_Controller(const ControllerInput *input, ControllerOutput *output); // 位置滑模控制器
void Attitude_Controller(const ControllerInput *input, ControllerOutput *output); // 姿态滑模控制器
void InitConstantsToFloat(void);                                                  // 初始化常数为浮点数
void update_Rsb(float phi, float theta, float psi);                               // 更新旋转矩阵 Rsb
void InitController(ControllerInput *input, ControllerOutput *output);            // 初始化控制器输入输出

// 饱和函数
static inline float sat(float s, float delta)
{
    return (s > delta) ? 1.0f : (s < -delta) ? -1.0f
                                             : s / delta;
}

// 自定义 fmin 和 fmax 函数的声明
// float custom_fmin(float x, float y);
static inline float custom_fmin(float x, float y) { return (x < y) ? x : y; }
// float custom_fmax(float x, float y);
static inline float custom_fmax(float x, float y) { return (x > y) ? x : y; }

#endif /* CONTROLLER_H */
