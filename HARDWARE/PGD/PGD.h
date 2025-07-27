#ifndef PGD_H
#define PGD_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>

// 矩阵的行列数（固定）
#define M_ROWS 6
#define M_COLS 12
#define MAX_ITER 200
#define Res_TOL 1e-2 // 残差阈值
#define PRINT_INTERVAL 100
#define x_max 0.21238888 // 解向量元素上界约束

// 硬编码的矩阵 M 和伪逆矩阵 M_pseudo
extern const double M[M_ROWS * M_COLS];
extern const double M_pseudo[M_COLS * M_ROWS];

#define PGD_BUDGET_CYCLES ((SystemCoreClock / 1000U) * 3) /* ≈ 3 ms */

typedef struct // PGD 静态工作区
{
    double g[M_COLS];
    double x_prev[M_COLS];
    double x_new[M_COLS];
    double Mx[M_ROWS];
} PGD_Work;

extern PGD_Work pgd_ws; // PGD.c 中定义

// 函数声明
void print_vector(const char *name, const double *vec, int size);                         // 打印向量
void project_target(const double *F, double *F_proj);                                     // 投影操作：计算 F_proj = M * M_pseudo * F
int is_in_column_space(const double *F, const double *F_proj);                            // 判断 F 是否在列空间中
double compute_objective(const double *F_proj, const double *x);                          // 计算目标函数 f(x) = 0.5 * ||F_proj - Mx||^2
double backtracking_line_search(const double *F_proj, const double *x, const double *g);  // 动态步长：回溯线搜索
void Denormalize_solution(const double *PGD_solution, float *de_norm_PGD);                // 反归一化PGD解
void compute_gradient(const double *F_proj, const double *x, double *g);                  // 计算梯度 g = -M^T * (F_proj - Mx)
int compute_residual(const double *F, const double *x, int status, int *accept_solution); // 验证残差并打印详细信息
void projected_gradient_descent(const double *F_proj, double *x);                         // 投影梯度下降法
#endif                                                                                    // PGD_H
