#ifndef _CONFIG_H
#define _CONFIG_H

#include <stdbool.h>

//================== 系统运行模式选择 ==================
// 0 = 调试模式:   系统低频运行，用于系统调试，此时发送全部调试信息
// 1 = 运行模式:   系统高频运行，用于实际控制，仅发送高层信息
#define OPERATING_MODE 1    // 系统运行模式切换标志位, 0=调试模式, 1=运行模式
#define SEND_DETAIL 0       // 调试模式下发送详细调试信息 (!OPERATING_MODE)
#define DEBUG_ECHO 0        // 可选调试：1=回显 USART2 RX 原始字节到调试口；0=完全关闭回显
#define CRC_DEBUG_ENABLED 0 // 设置为1启用CRC调试信息，设置为0关闭
//=====================================================

//================ 串口模式切换及相关配置 =============
#define USE_DMA 1              // 0=全部 IT，1=USART2 用 DMA RX
#define RX2_DMA_SZ 1024u       // DMA缓冲区 1024 (32B 对齐)
#define anomaly_threshold 3.0f // 跳变滤波过滤阈值
//===================================================

// #define LEARNING_RATE 0.1 // 初始学习率
// #define TOLERANCE 1e-7    // 收敛阈值
// #define MAX_ITERATIONS 10 // 最大迭代次数
//================PGD相关配置================
// 矩阵的行列数（固定）
#define M_ROWS 6
#define M_COLS 12
// 求解阈值
#define PROJ_ERR_TOL 1e-5  // 投影误差阈值
#define PGD_RES_TOL 5e-3   // 残差阈值
#define BTL_MIN_ALPHA 1e-4 // 线搜索最小步长
#define PGD_GRAD_TOL 1e-6  // 梯度阈值
#define PGD_OBJ_TOL 1e-7   // 目标函数阈值
#define PGD_UPD_TOL 1e-6   // 更新阈值
// 早停/步长刷新判据
#define PGD_STALL_ITERS 2    // 连续多少次认为“停滞”
#define PGD_DF_RATIO_THR 0.4 // Δf 相对下降不足 40 % 视为停滞
#define PGD_LS_MAX 15        // 固定线搜索最大步数
#define PGD_LS_BETA 0.8      // 衰减因子 β
#define PGD_HOT_START 1      // 打开热启动：用上一帧解作初值

#define x_max 0.21238888           // 解向量元素上界约束
#define PGD_BUDGET_CYCLES 2400000U //((480M / 1000) * 5) = 2400000 = 5 ms
#define PGD_MAX_ITER 180           // PGD最大迭代次数
//===========================================

//================机器人物理参数================
#define ROBOT_CONFIGURATION_6DOF 1 // 6自由度机器人
#define ROBOT_CONFIGURATION_3DOF 0 // 3自由度地面实验

extern bool current_robot_config; // 当前机器人构型

#define CURRENT_CONFIGURATION ROBOT_CONFIGURATION_3DOF

#if CURRENT_CONFIGURATION == ROBOT_CONFIGURATION_6DOF
// 6自由度机器人参数
#define MASS 12.71  // 机器人质量 (kg)
#define IXX 0.05648 // x轴惯性矩 (kg·m²)
#define IYY 0.04622 // y轴惯性矩 (kg·m²)
#define IZZ 0.02497 // z轴惯性矩 (kg·m²)
#define FMAX 0.2    // 单个风扇的最大推力 (N)
#define D 0.071     // 风扇到中心的距离 (m)
#else
// 3自由度地面实验参数
#define MASS 12.71  // 机器人质量 (kg)
#define IXX 0.05648 // x轴惯性矩 (kg·m²)
#define IYY 0.04622 // y轴惯性矩 (kg·m²)
#define IZZ 0.02497 // z轴惯性矩 (kg·m²)
#define FMAX 0.2    // 单个风扇的最大推力 (N)
#define D 0.071     // 风扇到中心的距离 (m)
#endif

#define NUM_FANS 12  // 风扇数量
#define NUM_FORCES 6 // 力和力矩分量数
//===========================================

// ================风扇物理参数================
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
//===========================================

//================滑模控制器参数================
#define epsilon_x 1.6 // 位置控制滑模面增益
#define epsilon_y 1.6
#define epsilon_z 1.6

#define epsilon_phi 1.6 // 姿态控制滑模面增益
#define epsilon_theta 1.6
#define epsilon_psi 1.6

#define p_c 1.3 // 位置控制增益
#define a_c 1.3

#define p_delta 1.0 // 位置饱和函数范围参数
#define a_delta 1.0 // 姿态饱和函数范围参数
//===========================================

//================其他配置================
#define USE_FREERTOS 0 // 1=在 ISR 中使用 FreeRTOS API
//===========================================

#endif
