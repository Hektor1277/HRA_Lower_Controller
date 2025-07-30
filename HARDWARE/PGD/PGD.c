#include "PGD.h"
#include "Fan.h"
#include "serial.h"
#include "stm32h7xx.h"
#include <string.h>

PGD_Work pgd_ws;              /* 全局唯一工作区 */
uint32_t pgd_timeout_cnt = 0; /* PGD 超时计数器 */
uint32_t pgd_max_cycles = 0, pgd_acc_cycles = 0, pgd_cnt = 0;
double pgd_last_solution[M_COLS] = {0};

extern volatile uint8_t pgd_abort_flag;

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
    USART_SendFormatted_DMA("%s:\r\n", name);
    for (int i = 0; i < size; i++)
    {
        USART_SendFormatted_DMA("%8.6f ", vec[i]);
    }
    USART_SendFormatted_DMA("\r\n");
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

// 投影操作：计算 F_proj = M * M_pseudo * F
void project_target(const double *F, double *F_proj)
{
    double y[M_COLS];
    /* y = M⁺ F   (12×6 ×6×1) */
    for (int j = 0; j < M_COLS; ++j)
    {
        double s = 0.0;
        for (int k = 0; k < M_ROWS; ++k)
        {
            s += M_pseudo[j * M_ROWS + k] * F[k];
        }
        y[j] = s;
    }
    /* F_proj = M y  (6×12 ×12×1) */
    for (int i = 0; i < M_ROWS; ++i)
    {
        double s = 0.0;
        for (int j = 0; j < M_COLS; ++j)
        {
            s += M[i * M_COLS + j] * y[j];
        }
        F_proj[i] = s;
    }
}

// 验证残差并打印详细信息
int compute_residual(const double *F, const double *x, int status, int *accept_solution)
{
    /* 1) Mx = M·x → 存入 pgd_ws.Mx */
    compute_Mx(x, pgd_ws.Mx);

    double residual = 0.0;
    int all_ok = 1; /* 10 % 容差用 */

#if SEND_DETAIL
    USART_SendFormatted_DMA("\r\nResidual check:\r\n");
#endif
    for (int i = 0; i < M_ROWS; ++i)
    {
        double diff = F[i] - pgd_ws.Mx[i];
#if SEND_DETAIL
        USART_SendFormatted_DMA("  [%d] F=%.4f  Mx=%.4f  diff=%.4f\r\n",
                                i, F[i], pgd_ws.Mx[i], diff);
#endif
        if (status == 1)
        {                            // F 处于列空间，使用严格阈值
            residual += diff * diff; // 计算总残差平方和
        }
        else if (status == 0)
        {                                        // F 不处于列空间，按 10% 容差判断
            double tolerance = fabs(F[i]) * 0.1; // 容差为初始 F 的 10%
            if (fabs(diff) > tolerance)
            {
                all_ok = 0; // 如果某个元素超出容差，标记为不接受
            }
        }
    }

    if (status == 1)
    {                              // F 处于列空间
        residual = sqrt(residual); // 计算残差的平方根
#if SEND_DETAIL
        USART_SendFormatted_DMA("Total Residual (F - Mx): %f\r\n", residual);
#endif
        if (residual < PGD_RES_TOL)
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Residual is within tolerance. Solution accepted.\r\n");
#endif
            *accept_solution = 1; // 接受解
        }
        else
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Residual exceeds tolerance. Solution rejected.\r\n");
#endif
            *accept_solution = 0; // 不接受解
        }
    }
    else if (status == 0)
    { // F 不处于列空间
        if (all_ok)
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("All elements are within 10%% tolerance. Solution accepted.\r\n");
#endif
            *accept_solution = 1; // 接受解
        }
        else
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Some elements exceed 10%% tolerance. Solution rejected.\r\n");
#endif
            *accept_solution = 0; // 不接受解
        }
    }

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
#if SEND_DETAIL
    USART_SendFormatted_DMA("PGD Solution:\r\n");

    for (int i = 0; i < M_COLS; i++)
    {
        USART_SendFormatted_DMA("%10.4f ", de_norm_PGD[i]);
        if ((i + 1) % 6 == 0)
            USART_SendFormatted_DMA("\r\n");
    }
    USART_SendFormatted_DMA("\r\n");
#endif
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

#if SEND_DETAIL
    USART_SendFormatted_DMA("Projection Error Norm: %f\r\n", diff); // 打印误差范数
#endif

    if (diff < PROJ_ERR_TOL) // 判断误差是否小于阈值
    {
#if SEND_DETAIL
        USART_SendFormatted_DMA("F is in the column space of M.\r\n");
#endif
        return 1;
    }
    else
    {
#if SEND_DETAIL
        USART_SendFormatted_DMA("F is NOT in the column space of M.\r\n");
#endif
        return 0;
    }
}

// 计算梯度 g = -M^T * (F_proj - Mx)
void compute_gradient(const double *F_proj, const double *x, double *g)
{
    // 1) Mx = M·x 存入工作区
    for (int i = 0; i < M_ROWS; ++i)
    {
        double sum = 0.0;
        for (int j = 0; j < M_COLS; ++j)
            sum += M[i * M_COLS + j] * x[j];
        pgd_ws.Mx[i] = sum;
    }

    // 2) g = -Mᵀ(F_proj-Mx)
    for (int j = 0; j < M_COLS; ++j)
    {
        double acc = 0.0;
        for (int i = 0; i < M_ROWS; ++i)
            acc -= M[i * M_COLS + j] * (F_proj[i] - pgd_ws.Mx[i]);
        g[j] = acc;
    }
}

// 计算目标函数 f(x) = 0.5 * ||F_proj - Mx||^2
double compute_objective(const double *F_proj, const double *x)
{
    /* 复用上一步 Mx ；若调用次序不保证，可再计算一次 */
    compute_Mx(x, pgd_ws.Mx);

    double f = 0.0;
    for (int i = 0; i < M_ROWS; ++i)
    {
        double diff = F_proj[i] - pgd_ws.Mx[i];
        f += diff * diff;
    }
    return 0.5 * f;
}

// 回溯线搜索
double backtracking_line_search(const double *F_proj, const double *x, const double *g)
{
    double alpha = 1.0;        // 初始步长
    double beta = PGD_LS_BETA; // 步长缩放
    double sigma = 1e-4;       // Armijo 参数
    int iter = 0;

    double f_old = compute_objective(F_proj, x);

    while (1)
    {
        // 1. 试步 x_new = x - α g 并投影到约束
        for (int i = 0; i < M_COLS; ++i)
        {
            pgd_ws.x_new[i] = x[i] - alpha * g[i];
        }
        apply_constraints(pgd_ws.x_new);

        // 2. 计算 f_new 与 g·d
        double f_new = compute_objective(F_proj, pgd_ws.x_new);
        double gd = 0.0;
        for (int i = 0; i < M_COLS; ++i)
        {
            gd += g[i] * (pgd_ws.x_new[i] - x[i]);
        }

        // 3. Armijo 判据
        if (f_new <= f_old + sigma * alpha * gd)
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Step size accepted at iteration %d, alpha = %f\r\n", iter, alpha);
#endif
            break; // 满足 Armijo 条件，返回步长
        }

        // 4. 收缩步长继续
        alpha *= beta;
        if (alpha < BTL_MIN_ALPHA)
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Step size too small, stopping line search. Current iteration %d\r\n", iter);
#endif
            break;
        }
        if (iter >= PGD_LS_MAX)
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Maximum iterations reached, stopping line search.\r\n");
#endif
            break;
        }
        iter++;
    }
    return alpha;
}

// 改进的 PGD 算法：动态步长
// 投影梯度下降法
void projected_gradient_descent(const double *F_proj, double *x)
{
    uint32_t start = DWT->CYCCNT;
    double *g = pgd_ws.g;
    double *x_prev = pgd_ws.x_prev;
    int iter;

    double grad_norm, max_update, delta_f;
    double prev_delta_f = 1e9;
    int stall_counter = 0;

#if PGD_HOT_START                    // 热启动
    for (int i = 0; i < M_COLS; ++i) // 用上一帧解作初值并快速投影到约束
        x[i] = pgd_last_solution[i];
    apply_constraints(x);
#else
    memset(x, 0, sizeof(double) * M_COLS);
#endif

    // 上一轮目标函数值
    double f_old = compute_objective(F_proj, x);
    double alpha = 1.0; // 初始步长

    for (iter = 0; iter < PGD_MAX_ITER; ++iter)
    {
        // 检查是否触发软件看门狗
        if (pgd_abort_flag)
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("PGD abort by soft-WDG\r\n");
#endif
            pgd_abort_flag = 0;
            break; // 立即退出 PGD
        }

        // 检查是否超时
        if (CYCLES_EXPIRED(DWT->CYCCNT, start, PGD_BUDGET_CYCLES)) // 超时判定
        {
            pgd_timeout_cnt++;
#if SEND_DETAIL
            USART_SendFormatted_DMA("PGD timeout -> pseudo inverse\r\n");
#endif
            /* 兜底：伪逆 + 裁剪 */
            for (int j = 0; j < M_COLS; ++j)
            {
                double s = 0.0;
                for (int k = 0; k < M_ROWS; ++k)
                    s += M_pseudo[j * M_ROWS + k] * F_proj[k];
                x[j] = (s < 0) ? 0 : ((s > x_max) ? x_max : s);
            }
            break;
        }

        // 保存当前解到 x_prev
        for (int i = 0; i < M_COLS; ++i)
        {
            x_prev[i] = x[i];
        }

        // 计算梯度
        compute_gradient(F_proj, x, g);

        // 判断是否需要更新步长：①前5帧迭代；②连续 PGD_STALL_ITERS 次 Δf 下降不足阈值
        if (iter < 5 || stall_counter >= PGD_STALL_ITERS)
        {
            alpha = backtracking_line_search(F_proj, x, g);
            stall_counter = 0; // 重置
        }

        // 更新解 x = x - alpha * g
        for (int i = 0; i < M_COLS; ++i)
        {
            x[i] -= alpha * g[i];
        }

        // 投影到约束范围内
        apply_constraints(x);

        // 计算梯度范数
        grad_norm = 0.0;
        for (int i = 0; i < M_COLS; ++i)
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
        for (int i = 0; i < M_COLS; ++i)
        {
            double d = fabs(x[i] - x_prev[i]);
            if (d > max_update)
            {
                max_update = d;
            }
        }

        // 停滞检测
        if (delta_f < PGD_DF_RATIO_THR * prev_delta_f && iter >= 5)
            ++stall_counter;
        else
            stall_counter = 0;
        prev_delta_f = delta_f;

        double relax = 1.0 + 0.1 * iter / PGD_MAX_ITER; // 迭代越后越宽
        // 早停条件
        if (grad_norm < PGD_GRAD_TOL * relax) // 梯度范数小于阈值
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Converged at iteration %d with gradient norm: %e\r\n", iter, grad_norm);
#endif
            memcpy(pgd_last_solution, x, sizeof(double) * M_COLS); // 热启动缓存
            return;
        }
        else if (delta_f < PGD_OBJ_TOL * relax && grad_norm < PGD_GRAD_TOL * 5) // 目标函数变化小于阈值
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Stopped at iteration %d due to small objective change: %e\r\n", iter, delta_f);
#endif
            memcpy(pgd_last_solution, x, sizeof(double) * M_COLS); // 热启动缓存
            return;
        }
        else if (max_update < PGD_UPD_TOL * relax) // 更新量小于阈值
        {
#if SEND_DETAIL
            USART_SendFormatted_DMA("Stopped at iteration %d due to small update: %e\r\n", iter, max_update);
#endif
            memcpy(pgd_last_solution, x, sizeof(double) * M_COLS); // 热启动缓存
            return;
        }
    }
#if SEND_DETAIL
    USART_SendFormatted_DMA("Maximum iterations %d reached, PGD solver stopped\r\n", iter);
#endif
    memcpy(pgd_last_solution, x, sizeof(double) * M_COLS); // 热启动缓存
}
