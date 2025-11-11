#include "Silde_Mode_Controller.h"
#include "Fan.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "sys.h"
#include "config.h"
// #include "arm_math.h" // 引入 CMSIS-DSP 库

// 定义控制器输入输出, 风扇转速和占空比结构体
ControllerInput ctrl_input = {0};
ControllerInput prev_ctrl_input = {0};
ControllerOutput ctrl_output = {0};
extern FanSpeed Fan_desire_Speed;
extern FanControl Fan_Control_duty_rate;

float Rsb[3][3] = {0}; // 全局变量旋转矩阵 Rsb

// 定义浮点数常量
static float mass_f, ixx_f, iyy_f, izz_f, fmax_f, d_f;
static float epsilon_x_f, epsilon_y_f, epsilon_z_f;
static float epsilon_phi_f, epsilon_theta_f, epsilon_psi_f;
static float p_c_f, a_c_f, p_delta_f, a_delta_f;

bool current_robot_config = CURRENT_CONFIGURATION;

// 位置滑模控制器实现
void Position_Controller(const ControllerInput *input, ControllerOutput *output)
{
    // 提取输入信号
    float pxd = input->desired_position[0], dpxd = input->desired_velocity[0], ddpxd = input->desired_acceleration[0]; // 期望位置、速度、加速度
    float pyd = input->desired_position[1], dpyd = input->desired_velocity[1], ddpyd = input->desired_acceleration[1];
    float pzd = input->desired_position[2], dpzd = input->desired_velocity[2], ddpzd = input->desired_acceleration[2];

    // if (current_robot_config == 0) // 机器人构型为地面实验（3DOF）时
    // {
    //     // 将z轴期望位置、速度、加速度设为0
    //     pzd = 0, dpzd = 0, ddpzd = 0;
    // }

    float px = input->position[0], dpx = input->velocity[0]; // 实际位置、速度
    float py = input->position[1], dpy = input->velocity[1];
    float pz = input->position[2], dpz = input->velocity[2];

    // if (current_robot_config == 0) // 机器人构型为地面实验（3DOF）时
    // {
    //     // 将z轴实际位置、速度设为0
    //     pz = 0, dpz = 0;
    // }

    float phi = input->angles[0]; // 实际角度
    float theta = input->angles[1];
    float psi = input->angles[2];

    // if (current_robot_config == 0) // 机器人构型为地面实验（3DOF）时
    // {
    //     // 将phi, theta设为0
    //     phi = 0, theta = 0;
    // }

    // 计算位置和速度误差
    float e_x = pxd - px, de_x = dpxd - dpx;
    float e_y = pyd - py, de_y = dpyd - dpy;
    float e_z = pzd - pz, de_z = dpzd - dpz;

    // 滑模面 s = c * 位置误差 + 速度误差
    float s_x = p_c_f * e_x + de_x;
    float s_y = p_c_f * e_y + de_y;
    float s_z = p_c_f * e_z + de_z;

    // 声明滑模控制向量 A
    float A_x, A_y, A_z;

    // 声明机体系推力
    float F_bd[3];

    // 更新旋转矩阵 Rsb
    update_Rsb(phi, theta, psi);

    // 更新滑模控制向量 A
    A_x = p_c_f * de_x + ddpxd + epsilon_x_f * sat(s_x, p_delta_f);
    A_y = p_c_f * de_y + ddpyd + epsilon_y_f * sat(s_y, p_delta_f);
    A_z = p_c_f * de_z + ddpzd + epsilon_z_f * sat(s_z, p_delta_f);

    // 使用旋转矩阵Rsb的转置计算控制律: F_bd = m * (Rsb)^T * A
    F_bd[0] = mass_f * (Rsb[0][0] * A_x + Rsb[1][0] * A_y + Rsb[2][0] * A_z);
    F_bd[1] = mass_f * (Rsb[0][1] * A_x + Rsb[1][1] * A_y + Rsb[2][1] * A_z);
    F_bd[2] = mass_f * (Rsb[0][2] * A_x + Rsb[1][2] * A_y + Rsb[2][2] * A_z);

    output->thrust[0] = custom_fmin(custom_fmax(F_bd[0], -2 * fmax_f), 2 * fmax_f);
    output->thrust[1] = custom_fmin(custom_fmax(F_bd[1], -2 * fmax_f), 2 * fmax_f);
    output->thrust[2] = custom_fmin(custom_fmax(F_bd[2], -2 * fmax_f), 2 * fmax_f);

    if (current_robot_config == 0) // 机器人构型为地面实验（3DOF）时
    {
        output->thrust[2] = 0; // 限制 F_zbd 为0
    }
}

// 姿态滑模控制器实现
void Attitude_Controller(const ControllerInput *input, ControllerOutput *output)
{
    // 提取输入信号
    float phid = input->desired_angles[0], dphid = input->desired_angular_velocity[0], ddphid = input->desired_angular_acceleration[0]; // 期望角度、角速度、角加速度
    float thetad = input->desired_angles[1], dthetad = input->desired_angular_velocity[1], ddthetad = input->desired_angular_acceleration[1];
    float psid = input->desired_angles[2], dpsid = input->desired_angular_velocity[2], ddpsid = input->desired_angular_acceleration[2];

    // if (current_robot_config == 0) // 机器人构型为地面实验（3DOF）时
    // {
    //     // 将期望phi, theta设为0
    //     phid = 0, dphid = 0, ddphid = 0;
    //     thetad = 0, dthetad = 0, ddthetad = 0;
    // }

    float phi = input->angles[0], dphi = input->angular_velocity[0]; // 实际角度、角速度
    float theta = input->angles[1], dtheta = input->angular_velocity[1];
    float psi = input->angles[2], dpsi = input->angular_velocity[2];

    // if (current_robot_config == 0) // 机器人构型为地面实验（3DOF）时
    // {
    //     // 将实际phi, theta设为0
    //     phi = 0, dphi = 0;
    //     theta = 0, dtheta = 0;
    // }

    // 计算姿态角度和角速度误差
    float e_phi = phid - phi, de_phi = dphid - dphi;
    float e_theta = thetad - theta, de_theta = dthetad - dtheta;
    float e_psi = psid - psi, de_psi = dpsid - dpsi;

    // 滑模面 s = c * 角度误差 + 角速度误差
    float s_phi = a_c_f * e_phi + de_phi;
    float s_theta = a_c_f * e_theta + de_theta;
    float s_psi = a_c_f * e_psi + de_psi;

    // 控制律计算
    // 姿态控制律：根据滑模面的符号决定控制力矩的方向，并与角速度成比例
    float T_phi = ixx_f * (epsilon_phi_f * sat(s_phi, a_delta_f) + a_c_f * de_phi + ddphid) - dtheta * dpsi * (iyy_f - izz_f);
    float T_theta = iyy_f * (epsilon_theta_f * sat(s_theta, a_delta_f) + a_c_f * de_theta + ddthetad) - dpsi * dphi * (izz_f - ixx_f);
    float T_psi = izz_f * (epsilon_psi_f * sat(s_psi, a_delta_f) + a_c_f * de_psi + ddpsid) - dphi * dtheta * (ixx_f - iyy_f);

    // 对期望输出力矩进行约束, 单个风扇最大推力约为2.7N, 则单个风扇最大力矩为 F * D。同向最多四个风扇工作。
    output->torque[0] = custom_fmin(custom_fmax(T_phi, -4 * fmax_f * d_f), 4 * fmax_f * d_f);   // 限制 T_xbd
    output->torque[1] = custom_fmin(custom_fmax(T_theta, -4 * fmax_f * d_f), 4 * fmax_f * d_f); // 限制 T_ybd
    output->torque[2] = custom_fmin(custom_fmax(T_psi, -4 * fmax_f * d_f), 4 * fmax_f * d_f);   // 限制 T_zbd

    if (current_robot_config == 0) // 机器人构型为地面实验（3DOF）时
    {
        output->torque[0] = 0; // 限制 T_xbd为0
        output->torque[1] = 0; // 限制 T_ybd为0
    }
}

// 初始化常量为浮点数
void InitConstantsToFloat(void)
{
    mass_f = (float)MASS;
    ixx_f = (float)IXX;
    iyy_f = (float)IYY;
    izz_f = (float)IZZ;
    fmax_f = (float)FMAX;
    d_f = (float)D;

    epsilon_x_f = (float)epsilon_x;
    epsilon_y_f = (float)epsilon_y;
    epsilon_z_f = (float)epsilon_z;

    epsilon_phi_f = (float)epsilon_phi;
    epsilon_theta_f = (float)epsilon_theta;
    epsilon_psi_f = (float)epsilon_psi;

    p_c_f = (float)p_c;
    a_c_f = (float)a_c;

    p_delta_f = (float)p_delta;
    a_delta_f = (float)a_delta;
}

// 更新旋转矩阵 Rsb
void update_Rsb(float phi, float theta, float psi)
{
    float cos_psi = cosf(psi);
    float sin_psi = sinf(psi);
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float cos_phi = cosf(phi);
    float sin_phi = sinf(phi);

    float cos_psi_cos_theta = cos_psi * cos_theta;
    float cos_psi_sin_theta = cos_psi * sin_theta;
    float sin_psi_cos_theta = sin_psi * cos_theta;
    float sin_psi_sin_theta = sin_psi * sin_theta;

    Rsb[0][0] = cos_psi_cos_theta;
    Rsb[0][1] = cos_psi_sin_theta * sin_phi - sin_psi * cos_phi;
    Rsb[0][2] = cos_psi_sin_theta * cos_phi + sin_psi * sin_phi;
    Rsb[1][0] = sin_psi_cos_theta;
    Rsb[1][1] = sin_psi_sin_theta * sin_phi + cos_psi * cos_phi;
    Rsb[1][2] = sin_psi_sin_theta * cos_phi - cos_psi * sin_phi;
    Rsb[2][0] = -sin_theta;
    Rsb[2][1] = cos_theta * sin_phi;
    Rsb[2][2] = cos_theta * cos_phi;
}

// 初始化控制器输入输出的函数，并热启动风扇（调用时进行结构体清零）
void InitController(ControllerInput *input, ControllerOutput *output)
{
    // 清零输入输出结构体
    memset(input, 0, sizeof(ControllerInput));
    memset(output, 0, sizeof(ControllerOutput));

    // 风扇热启动
    Fan_Rotation_Control(&ctrl_output, &Fan_desire_Speed, &Fan_Control_duty_rate);
}
