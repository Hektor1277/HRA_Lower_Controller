#include "PGD.h"
#include "Fan.h"
#include "usart.h"

// 硬编码的矩阵 M
const double M[M_ROWS * M_COLS] = {
    -2.5, 2.5, -2.5, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, -2.5, 2.5, -2.5, 2.5, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.5, 2.5, -2.5, 2.5,
    0.0, 0.0, 0.0, 0.0, 0.1875, 0.1875, -0.1875, -0.1875, -0.1875, 0.1875, 0.1875, -0.1875,
    -0.1875, -0.1875, 0.1875, 0.1875, 0.0, 0.0, 0.0, 0.0, 0.1875, 0.1875, -0.1875, -0.1875,
    0.1875, -0.1875, -0.1875, 0.1875, -0.1875, 0.1875, 0.1875, -0.1875, 0.0, 0.0, 0.0, 0.0};

// 硬编码的伪逆矩阵 M^+
const double M_pseudo[M_COLS * M_ROWS] = {
    -0.1000, 0.0000, -0.0000, 0.0000, -0.6667, 0.6667,
    0.1000, 0.0000, -0.0000, 0.0000, -0.6667, -0.6667,
    -0.1000, -0.0000, 0.0000, -0.0000, 0.6667, -0.6667,
    0.1000, 0.0000, 0.0000, 0.0000, 0.6667, 0.6667,
    0.0000, -0.1000, -0.0000, 0.6667, -0.0000, -0.6667,
    0.0000, 0.1000, -0.0000, 0.6667, 0.0000, 0.6667,
    0.0000, -0.1000, 0.0000, -0.6667, 0.0000, 0.6667,
    0.0000, 0.1000, 0.0000, -0.6667, -0.0000, -0.6667,
    0.0000, 0.0000, -0.1000, -0.6667, 0.6667, -0.0000,
    -0.0000, 0.0000, 0.1000, 0.6667, 0.6667, -0.0000,
    0.0000, -0.0000, -0.1000, 0.6667, -0.6667, 0.0000,
    -0.0000, 0.0000, 0.1000, -0.6667, -0.6667, 0.0000};

// 打印向量
void print_vector(const char *name, const double *vec, int size)
{
    USART_SendFormatted("%s:\r\n", name);
    for (int i = 0; i < size; i++)
    {
        USART_SendFormatted("%8.6f ", vec[i]);
    }
    USART_SendFormatted("\r\n");
}

// 辅助函数：计算 Mx
static void compute_Mx(const double *x, double *Mx)
{
    for (int i = 0; i < M_ROWS; i++)
    {
        Mx[i] = 0.0;
        for (int j = 0; j < M_COLS; j++)
        {
            Mx[i] += M[i * M_COLS + j] * x[j];
        }
    }
}

// 辅助函数：投影到约束范围内
static void apply_constraints(double *x)
{
    for (int i = 0; i < M_COLS; i++)
    {
        if (x[i] < 0.0)
        {
            x[i] = 0.0;
        }
        if (x[i] > x_max)
        {
            x[i] = x_max;
        }
    }
}

// 内存分配检查函数
static void *checked_calloc(size_t num, size_t size)
{
    void *ptr = calloc(num, size);
    if (!ptr)
    {
        // 使用USART发送错误信息
        USART_SendFormatted("Memory allocation failed\r\n");
        return NULL;
    }
    return ptr;
}

// 投影操作：计算 F_proj = M * M_pseudo * F
void project_target(const double *F, double *F_proj)
{
    for (int i = 0; i < M_ROWS; i++)
    {
        F_proj[i] = 0.0;
        for (int j = 0; j < M_COLS; j++)
        {
            for (int k = 0; k < M_ROWS; k++)
            {
                F_proj[i] += M[i * M_COLS + j] * M_pseudo[j * M_ROWS + k] * F[k];
            }
        }
    }
}

// 验证残差并打印详细信息
int compute_residual(const double *F, const double *x, int status, int *accept_solution)
{
    double *Mx = (double *)calloc(M_ROWS, sizeof(double));
    double residual = 0.0;
    int all_elements_within_tolerance = 1; // 用于标记是否所有元素都满足条件

    if (!Mx)
    {
        USART_SendFormatted("Memory allocation failed for Mx.\r\n");
        *accept_solution = 0;
        return -1; // 表示错误
    }

    // 计算 Mx
    compute_Mx(x, Mx);

    // 打印初始 F 和 PGD解计算得到的F = Mx
    USART_SendFormatted("Original F:\r\n");
    for (int i = 0; i < M_ROWS; i++)
    {
        USART_SendFormatted("%8.4f ", F[i]);
    }
    USART_SendFormatted("\r\n");

    USART_SendFormatted("Computed Mx:\r\n");
    for (int i = 0; i < M_ROWS; i++)
    {
        USART_SendFormatted("%8.4f ", Mx[i]);
    }
    USART_SendFormatted("\r\n");

    // 打印每一项的残差
    USART_SendFormatted("Residual for each element (F[i] - Mx[i]):\r\n");
    for (int i = 0; i < M_ROWS; i++)
    {
        double diff = F[i] - Mx[i];
        USART_SendFormatted("Residual[%d]: %8.4f ", i, diff);

        if (status == 1)
        {                            // F 处于列空间，使用严格阈值
            residual += diff * diff; // 计算总残差平方和
        }
        else if (status == 0)
        {                                        // F 不处于列空间，按 10% 容差判断
            double tolerance = fabs(F[i]) * 0.1; // 容差为初始 F 的 10%
            if (fabs(diff) > tolerance)
            {
                all_elements_within_tolerance = 0; // 如果某个元素超出容差，标记为不接受
            }
        }
    }
    USART_SendFormatted("\r\n");

    if (status == 1)
    {                              // F 处于列空间
        residual = sqrt(residual); // 计算残差的平方根
        USART_SendFormatted("Total Residual (F - Mx): %f\r\n", residual);

        if (residual < Res_TOL)
        {
            USART_SendFormatted("Residual is within tolerance.\r\n");
            *accept_solution = 1; // 接受解
        }
        else
        {
            USART_SendFormatted("Residual exceeds tolerance.\r\n");
            *accept_solution = 0; // 不接受解
        }
    }
    else if (status == 0)
    { // F 不处于列空间
        if (all_elements_within_tolerance)
        {
            USART_SendFormatted("All elements are within 10%% tolerance.\r\n");
            *accept_solution = 1; // 接受解
        }
        else
        {
            USART_SendFormatted("Some elements exceed 10%% tolerance.\r\n");
            *accept_solution = 0; // 不接受解
        }
    }

    free(Mx); // 释放动态内存
    return 0; // 表示成功
}

// 反归一化PGD解
void Denormalize_solution(const double *PGD_solution, float *de_norm_PGD)
{
    // 反归一化PGD解
    for (int i = 0; i < M_COLS; i++)
    {
        de_norm_PGD[i] = (float)sqrt(PGD_solution[i] / c_T);
    }

    USART_SendFormatted("PGD Solution:\r\n");

    for (int i = 0; i < M_COLS; i++)
    {
        USART_SendFormatted("%10.4f ", de_norm_PGD[i]);
        if ((i + 1) % 6 == 0)
            USART_SendFormatted("\r\n");
    }
    USART_SendFormatted("\r\n");
}

// 判断 F 是否在列空间中
int is_in_column_space(const double *F, const double *F_proj)
{
    double diff = 0.0;

    // 计算投影误差范数
    for (int i = 0; i < M_ROWS; i++)
    {
        diff += (F[i] - F_proj[i]) * (F[i] - F_proj[i]);
    }
    diff = sqrt(diff);

    USART_SendFormatted("Projection Error Norm: %f\r\n", diff); // 打印误差范数

    if (diff < 1e-5) // 判断误差是否小于阈值
    {
        USART_SendFormatted("F is in the column space of M.\r\n");
        return 1;
    }
    else
    {
        USART_SendFormatted("F is NOT in the column space of M.\r\n");
        return 0;
    }
}

// 计算梯度 g = -M^T * (F_proj - Mx)
void compute_gradient(const double *F_proj, const double *x, double *g)
{
    double *Mx = (double *)calloc(M_ROWS, sizeof(double)); // 用于存储 Mx

    // 计算 Mx
    for (int i = 0; i < M_ROWS; i++)
    {
        for (int j = 0; j < M_COLS; j++)
        {
            Mx[i] += M[i * M_COLS + j] * x[j];
        }
    }

    // 计算梯度 g = -M^T * (F_proj - Mx)
    for (int j = 0; j < M_COLS; j++)
    {
        g[j] = 0.0;
        for (int i = 0; i < M_ROWS; i++)
        {
            g[j] -= M[i * M_COLS + j] * (F_proj[i] - Mx[i]);
        }
    }

    free(Mx);
}

// 计算目标函数 f(x) = 0.5 * ||F_proj - Mx||^2
double compute_objective(const double *F_proj, const double *x)
{
    double *Mx = (double *)checked_calloc(M_ROWS, sizeof(double));

    compute_Mx(x, Mx);

    double f = 0.0;
    for (int i = 0; i < M_ROWS; i++)
    {
        double diff = F_proj[i] - Mx[i];
        f += diff * diff;
    }
    free(Mx);
    return 0.5 * f;
}

// 动态步长：回溯线搜索
double backtracking_line_search(const double *F_proj, const double *x, const double *g)
{
    double alpha = 1.0;  // 初始步长
    double beta = 0.8;   // 步长缩小比例
    double sigma = 1e-4; // Armijo 条件参数

    double *x_new = (double *)checked_calloc(M_COLS, sizeof(double));

    double f_old = compute_objective(F_proj, x);

    while (1)
    {
        for (int i = 0; i < M_COLS; i++)
        {
            x_new[i] = x[i] - alpha * g[i];
        }

        apply_constraints(x_new);

        double f_new = compute_objective(F_proj, x_new);

        double grad_dot = 0.0;
        for (int i = 0; i < M_COLS; i++)
        {
            grad_dot += g[i] * (x_new[i] - x[i]);
        }

        if (f_new <= f_old + sigma * alpha * grad_dot)
        {
            break;
        }

        alpha *= beta;
        if (alpha < 1e-8)
        {
            USART_SendFormatted("Step size too small, stopping line search.\r\n");
            break;
        }
    }

    free(x_new);
    return alpha;
}

// 改进的 PGD 算法：动态步长
// 投影梯度下降法
void projected_gradient_descent(const double *F_proj, double *x)
{
    double *g = (double *)malloc(M_COLS * sizeof(double));
    double *x_prev = (double *)malloc(M_COLS * sizeof(double)); // 用于存储上一轮的解
    double f_old = compute_objective(F_proj, x);                // 上一轮目标函数值
    double grad_norm, max_update, delta_f;

    for (int iter = 0; iter < MAX_ITER; iter++)
    {
        // 保存当前解到 x_prev
        for (int i = 0; i < M_COLS; i++)
        {
            x_prev[i] = x[i];
        }

        // 计算梯度
        compute_gradient(F_proj, x, g);

        // 动态步长：回溯线搜索
        double alpha = backtracking_line_search(F_proj, x, g);

        // 更新解 x = x - alpha * g
        for (int i = 0; i < M_COLS; i++)
        {
            x[i] -= alpha * g[i];
        }

        // 投影到约束范围内
        apply_constraints(x);

        // 计算梯度范数
        grad_norm = 0.0;
        for (int i = 0; i < M_COLS; i++)
        {
            grad_norm += g[i] * g[i];
        }
        grad_norm = sqrt(grad_norm);

        // 计算目标函数值变化
        double f_new = compute_objective(F_proj, x);
        delta_f = fabs(f_new - f_old);
        f_old = f_new;

        // 计算解的最大更新量
        max_update = 0.0;
        for (int i = 0; i < M_COLS; i++)
        {
            double diff = fabs(x[i] - x_prev[i]);
            if (diff > max_update)
            {
                max_update = diff;
            }
        }

        // 输出调试信息
        if (iter % PRINT_INTERVAL == 0)
        {
            USART_SendFormatted("Iteration %d, Objective Value: %f, Grad Norm: %e, Max Update: %e, Step Size: %f\r\n",
                                iter, f_new, grad_norm, max_update, alpha);
        }

        // 早停条件
        if (grad_norm < 1e-6)
        {
            USART_SendFormatted("Converged at iteration %d with gradient norm: %e\r\n", iter, grad_norm);
            break;
        }
        if (delta_f < 1e-8)
        {
            USART_SendFormatted("Stopped at iteration %d due to small objective change: %e\r\n", iter, delta_f);
            break;
        }
        if (max_update < 1e-6)
        {
            USART_SendFormatted("Stopped at iteration %d due to small update: %e\r\n", iter, max_update);
            break;
        }
    }

    free(g);
    free(x_prev);
}
