#include "Silde_Mode_Controller.h"
#include "Fan.h"
#include "usart.h"
#include "delay.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

// 控制器变量
extern ControllerInput ctrl_input;
extern ControllerInput prev_ctrl_input; // 上一个周期的控制输入，用于处理延迟或噪声
extern ControllerOutput ctrl_output;
extern FanSpeed Fan_desire_Speed;
extern FanControl Fan_Control_duty_rate;

// 串口句柄
UART_HandleTypeDef UART1_Handler; // UART1句柄
UART_HandleTypeDef UART2_Handler; // UART2句柄

// 串口缓冲区定义
static uint16_t pc_rx_sta = 0, data_rx_sta = 0;
static uint8_t pc_rx_buf[PC_CMD_LEN];
static uint8_t pc_aRxBuffer[RXBUFFERSIZE];
static uint8_t data_rx_buf[USART_REC_LEN];
static uint8_t data_aRxBuffer[RXBUFFERSIZE];

bool frame_fault = false;	// 数据帧滤波标志位，0为正常，1为未通过滤波
bool is_first_frame = true; // 标志是否是第一次接收数据

const float THRESHOLD = 120.0f; // 设定一个阈值，例如允许的最大变化量

// 错误计数器
static volatile u32 frame_error_count = 0;	  // 帧错误计数器
static volatile u32 checksum_error_count = 0; // 帧校验和错误计数器
static volatile u32 data_anomaly_count = 0;	  // 数据帧异常计数器
static volatile uint64_t lost_pkt_count = 0;  // 丢包计数器

// 调试全局量定义
volatile u32 g_frame_seq_id = 0;
volatile uint64_t g_frame_time_ns = 0;
volatile float g_frame_latency = 0.0f;
volatile u32 g_seq_gap = 0;

// 用于计算帧间延迟的变量
static uint64_t prev_unix_ns = 0; /* 仅用于计算 inter-frame latency */
static u32 prev_seq_id = 0;

void USART_SendFormatted(UART_HandleTypeDef *huart, const char *format, ...)
{
	char buf[256];
	va_list args;
	va_start(args, format);
	int len = vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	if (len <= 0)
		return;

	int sent = 0;
	while (sent < len)
	{
		int chunk = (len - sent > sizeof(buf) - 1) ? sizeof(buf) - 1 : (len - sent);
		HAL_UART_Transmit(huart, (uint8_t *)(buf + sent), chunk, HAL_MAX_DELAY);
		sent += chunk;
	}
}

// 初始化IO 串口1
// bound:波特率
void uart_init(u32 bound)
{
#if OPERATING_MODE // 根据运行模式决定DATA_UART是UART2还是UART1
	DATA_UART.Instance = USART2;
#else												// 调试模式：UART1 既收数据又打印
	DATA_UART.Instance = USART1;
#endif												// DATA_UART调试时为串口1，实际运行时为串口2
	DATA_UART.Init.BaudRate = bound;				// 波特率
	DATA_UART.Init.WordLength = UART_WORDLENGTH_8B; // 字长为8位数据格式
	DATA_UART.Init.StopBits = UART_STOPBITS_1;		// 一个停止位
	DATA_UART.Init.Parity = UART_PARITY_NONE;		// 无奇偶校验位
	DATA_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无硬件流控
	DATA_UART.Init.Mode = UART_MODE_TX_RX;			// 收发模式
	HAL_UART_Init(&DATA_UART);						// HAL_UART_Init()使能DATA_UART

#if OPERATING_MODE // 运行模式下，TERM_UART为串口1用于调试输出
	// TERM_UART 仅在运行模式初始化 (波特率一致)
	TERM_UART = (UART_HandleTypeDef){
		.Instance = USART1,
		.Init = DATA_UART.Init};
	HAL_UART_Init(&TERM_UART);
#endif

/* 启动首包接收 —— 根据模式选择缓冲区 ------------------------------ */
#if !OPERATING_MODE
	HAL_UART_Receive_IT(&DATA_UART, data_aRxBuffer, RXBUFFERSIZE);
#else
	HAL_UART_Receive_IT(&DATA_UART, data_aRxBuffer, RXBUFFERSIZE);
	HAL_UART_Receive_IT(&TERM_UART, pc_aRxBuffer, RXBUFFERSIZE);
#endif
}

// UART底层初始化，时钟使能，引脚配置，中断配置
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_Initure;

	if (huart->Instance == USART1) // 如果是串口1，进行串口1 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();  // 使能GPIOA时钟
		__HAL_RCC_USART1_CLK_ENABLE(); // 使能USART1时钟

		GPIO_Initure.Pin = GPIO_PIN_9;			   // PA9
		GPIO_Initure.Mode = GPIO_MODE_AF_PP;	   // 复用推挽输出
		GPIO_Initure.Pull = GPIO_PULLUP;		   // 上拉
		GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH; // 高速
		GPIO_Initure.Alternate = GPIO_AF7_USART1;  // 复用为USART1
		HAL_GPIO_Init(GPIOA, &GPIO_Initure);	   // 初始化PA9

		GPIO_Initure.Pin = GPIO_PIN_10;		 // PA10
		HAL_GPIO_Init(GPIOA, &GPIO_Initure); // 初始化PA10

		HAL_NVIC_EnableIRQ(USART1_IRQn);		 // 使能USART1中断通道
		HAL_NVIC_SetPriority(USART1_IRQn, 0, 1); // 抢占优先级0，子优先级1
	}
	else if (huart->Instance == USART2) // 如果是串口2，进行串口2 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_USART2_CLK_ENABLE();

		GPIO_Initure.Pin = GPIO_PIN_2; // PA2 TX
		GPIO_Initure.Mode = GPIO_MODE_AF_PP;
		GPIO_Initure.Pull = GPIO_PULLUP;
		GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;
		GPIO_Initure.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &GPIO_Initure);

		GPIO_Initure.Pin = GPIO_PIN_3; // PA3 RX
		HAL_GPIO_Init(GPIOA, &GPIO_Initure);

		HAL_NVIC_EnableIRQ(USART2_IRQn);		 // 使能USART2中断通道
		HAL_NVIC_SetPriority(USART2_IRQn, 0, 1); // 抢占优先级0，子优先级1
	}
}

// 串口接收完成回调：在 USART2 中进行数据接收处理逻辑；通过 USART1_SendFormatted() 输出所有调试信息
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// 接收数据处理逻辑
	if (huart->Instance == DATA_UART.Instance) // 如果是数据口
	{
		if ((data_rx_sta & 0x8000) == 0) // 接收未完成
		{
			if (data_rx_sta & 0x4000) // 已经接收到帧尾的第一个字节（0xCC）
			{
				if (data_aRxBuffer[0] != FRAME_FOOTER_2) // 检查是否接收到帧尾的第二个字节（0xDD）
				{
					// USART_SendFormatted("Invalid frame footer, resetting.\r\n");
					frame_error_count++;
					data_rx_sta = 0; // 接收错误，重新开始
				}
				else
				{
					data_rx_sta |= 0x8000; // 接收完成
					// USART_SendFormatted("Frame received successfully.\r\n");

					// 解析数据
					parse_data(&ctrl_input, &prev_ctrl_input); // 调用数据解析函数，处理帧尾前的数据

#if !OPERATING_MODE // 调试模式
					// 调用当前周期控制输入结构体，输入位置和姿态控制器计算控制输出
					Position_Controller(&ctrl_input, &ctrl_output);
					Attitude_Controller(&ctrl_input, &ctrl_output);
					// 调用底层风扇控制函数，根据控制输出计算风扇转速并输出相应PWM信号
					Fan_Rotation_Control(&ctrl_output, &Fan_desire_Speed, &Fan_Control_duty_rate);
					// 发送调试信息到TERM_UART
					send_info(&TERM_UART);
#endif
					data_rx_sta = 0; // 重置接收状态，以便下一次接收
				}
			}
			else // 还没有收到帧尾的第一个字节
			{
				if (data_aRxBuffer[0] == FRAME_FOOTER_1) // 如果接收到的是帧尾的第一个字节（0xCC）
				{
					data_rx_sta |= 0x4000; // 标记已收到第一个字节
				}
				else
				{
					if (data_rx_sta < USART_REC_LEN)
					{
						data_rx_buf[data_rx_sta++] = data_aRxBuffer[0]; // 保存接收到的数据到缓冲区
					}
					else
					{
						USART_SendFormatted(&TERM_UART, "Buffer overflow, resetting.\r\n");
						frame_error_count++;
						data_rx_sta = 0; // 缓冲区溢出，重新开始
					}
				}
			}
		}
		HAL_UART_Receive_IT(&DATA_UART, data_aRxBuffer, RXBUFFERSIZE); // 重新开启DATA_UART中断
	}

#if OPERATING_MODE
	else if (huart->Instance == TERM_UART.Instance)
	{
		// 简单回显或命令解析示例
		if (pc_rx_sta < PC_CMD_LEN) // 防止缓冲区溢出
		{
			pc_rx_buf[pc_rx_sta++] = pc_aRxBuffer[0];
			if (pc_rx_buf[pc_rx_sta - 1] == '\n' || pc_rx_sta >= PC_CMD_LEN) // 收到换行或缓冲区满
			{
				HAL_UART_Transmit(&TERM_UART, pc_rx_buf, pc_rx_sta, HAL_MAX_DELAY);
				pc_rx_sta = 0; // 重置接收状态
			}
		}
		else
		{
			pc_rx_sta = 0; // 缓冲区溢出，重置
		}
		HAL_UART_Receive_IT(&TERM_UART, pc_aRxBuffer, RXBUFFERSIZE);
	}
#endif
}

// 串口1中断服务程序
void USART1_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS // 如果使用操作系统（如RTOS）
	OSIntEnter();	  // 进入中断服务程序
#endif

	HAL_UART_IRQHandler(&UART1_Handler);

#if SYSTEM_SUPPORT_OS // 如果使用操作系统（如RTOS）
	OSIntExit();	  // 退出中断服务程序
#endif
}

// 串口2中断服务程序
void USART2_IRQHandler(void)
{
#if SYSTEM_SUPPORT_OS
	OSIntEnter();
#endif
	HAL_UART_IRQHandler(&UART2_Handler);
#if SYSTEM_SUPPORT_OS
	OSIntExit();
#endif
}

//
// * @brief  在 HAL_UART_RxCpltCallback() 判定“帧已收完”后调用，
// *         从 data_rx_buf 中提取最新一帧（不含 0xCC 0xDD），
// *         做 CRC/边界检查并填充 ctrl_input。
// * @note   缓冲区中 **不含** 帧尾字节，因此有效长度应为 88 B。
//

void parse_data(ControllerInput *ctrl_input, ControllerInput *prev_ctrl_input)
{
	/******** 0. 取得有效长度＝data_rx_sta & 0x3FFF ********/
	u16 buf_len = (data_rx_sta & 0x3FFF);		   // 已收字节数(不含尾)
	u16 FRAME_LEN_NO_FOOTER = TOTAL_FRAME_LEN - 2; // 90-2＝88

	if (buf_len < FRAME_LEN_NO_FOOTER)
	{ // 长度不足
		frame_error_count++;
		return;
	}

	/******** 1. 在缓存里寻找帧头 AA BB（可能存在前置噪声字节） ********/
	uint8_t *frame = NULL;
	for (int i = 0; i <= buf_len - FRAME_LEN_NO_FOOTER; ++i)
	{
		if (data_rx_buf[i] == FRAME_HEADER_1 && data_rx_buf[i + 1] == FRAME_HEADER_2)
		{
			frame = &data_rx_buf[i];
			break;
		}
	}
	if (!frame) // 没找到帧头
	{
		frame_error_count++;
		return;
	}

	/******** 2. 检查数据帧完整 88 B ********/
	if ((frame + FRAME_LEN_NO_FOOTER) > (data_rx_buf + buf_len))
	{
		frame_error_count++;
		return; // 缓冲区末端截断
	}

	/* 3. CRC16(累加和) 校验 — 范围: byte[2..85] 共 84 B */
	{
		/* ① 直接把 84 B 看成 42 个 16-bit 字，大幅减循环次数
		 * ② __REV16() 把大端字节顺序翻转成小端，自带一条 rev16 指令 */
		const uint16_t *wordBE = (const uint16_t *)(frame + 2);
		uint16_t crc = 0;
		for (int i = 0; i < 42; ++i)   /* 84 / 2 */
			crc += __REV16(wordBE[i]); /* hi<<8 | lo */
		if (crc != ((frame[86] << 8) | frame[87]))
		{
			++checksum_error_count;
			return;
		}
	}

	/******** 4. 解析 sequence_id 与 unix_time_ns（可选保存/调试） ********/
	uint32_t sequence_id = (frame[2] << 24) |
						   (frame[3] << 16) |
						   (frame[4] << 8) |
						   frame[5];

	uint64_t unix_time_ns = 0;
	for (int k = 0; k < 8; ++k)
	{
		unix_time_ns = (unix_time_ns << 8) | frame[6 + k];
	}

	/* ① 计算 inter-frame latency (ms) */
	float latency_ms = 0.f;
	if (prev_unix_ns == 0)
		latency_ms = 0.f; // 首帧无延迟
	if (prev_unix_ns)
		latency_ms = (unix_time_ns - prev_unix_ns) * 1e-6f;

	// 声明丢包 gap
	uint32_t gap = 0;

	/* 这里若 MCU 不需要，可忽略或仅用于 log/监控 */

	/******** 5. 解析 72 B payload — 从偏移 14 开始 ********/
	const uint8_t *p = frame + 14; /* 指向 payload[0] */

	__disable_irq(); // 进入临界区

	/* 72 B → 36×int16 → 12 组向量 */
	float *dst_groups[12] = {
		ctrl_input->desired_position,
		ctrl_input->desired_velocity,
		ctrl_input->desired_acceleration,
		ctrl_input->desired_angles,
		ctrl_input->desired_angular_velocity,
		ctrl_input->desired_angular_acceleration,
		ctrl_input->position,
		ctrl_input->velocity,
		ctrl_input->acceleration,
		ctrl_input->angles,
		ctrl_input->angular_velocity,
		ctrl_input->angular_acceleration};

	for (int g = 0; g < 12; ++g)
	{ /* 12 组 × 3 轴 */
		float *dst = dst_groups[g];
		dst[0] = be_i16_to_f(p);
		p += 2;
		dst[1] = be_i16_to_f(p);
		p += 2;
		dst[2] = be_i16_to_f(p);
		p += 2;
	}
	__enable_irq(); // 退出临界区

	/******** 6. 简单异常跳变滤波（保持你原先逻辑不变） ********/
	if (is_first_frame)
	{
		*prev_ctrl_input = *ctrl_input;
		prev_seq_id = sequence_id; // 首帧初始化
		prev_unix_ns = unix_time_ns;
		is_first_frame = false;
	}
	else
	{
		bool jump = false;
		for (int i = 0; i < 3 && !jump; ++i)
			jump |= big_jump(ctrl_input->desired_position[i], prev_ctrl_input->desired_position[i]) ||
					big_jump(ctrl_input->desired_velocity[i], prev_ctrl_input->desired_velocity[i]) ||
					big_jump(ctrl_input->desired_acceleration[i], prev_ctrl_input->desired_acceleration[i]) ||
					big_jump(ctrl_input->desired_angles[i], prev_ctrl_input->desired_angles[i]) ||
					big_jump(ctrl_input->desired_angular_velocity[i],
							 prev_ctrl_input->desired_angular_velocity[i]) ||
					big_jump(ctrl_input->desired_angular_acceleration[i],
							 prev_ctrl_input->desired_angular_acceleration[i]) ||
					big_jump(ctrl_input->position[i], prev_ctrl_input->position[i]) ||
					big_jump(ctrl_input->velocity[i], prev_ctrl_input->velocity[i]) ||
					big_jump(ctrl_input->acceleration[i], prev_ctrl_input->acceleration[i]) ||
					big_jump(ctrl_input->angles[i], prev_ctrl_input->angles[i]) ||
					big_jump(ctrl_input->angular_velocity[i], prev_ctrl_input->angular_velocity[i]) ||
					big_jump(ctrl_input->angular_acceleration[i], prev_ctrl_input->angular_acceleration[i]);

		if (jump) /* 异常帧——回滚不更新 prev_xxx */
		{
			++data_anomaly_count;
			*ctrl_input = *prev_ctrl_input;
		}
		else /* 正常帧——同步 prev_xxx */
		{
			/* 只有在正常帧里做 gap 统计 */
			if (prev_seq_id) /* first frame = 0 */
			{
				gap = sequence_id - prev_seq_id - 1U; /* 2^32 wrap-safe */
				lost_pkt_count += gap;
			}

			*prev_ctrl_input = *ctrl_input;
			prev_seq_id = sequence_id;
			prev_unix_ns = unix_time_ns;
		}

		/* 写入全局调试量 —— 供 send_info() 使用 */
		g_frame_seq_id = sequence_id;
		g_frame_time_ns = unix_time_ns;
		g_frame_latency = latency_ms;
		g_seq_gap = gap;
	}
}

// 发送解析后的数据用于验证
void send_info(UART_HandleTypeDef *huart)
{
	// 1. 系统状态 & 错误计数
	USART_SendFormatted(huart,
						"\r\n=================== Bridge Stats ===================\r\n"
						"Frame Errors      : %lu\r\n"
						"Checksum Errors   : %lu\r\n"
						"Data Anomalies    : %lu\r\n"
						"Lost Packets      : %lu\r\n"
						"\r\n=================== Bridge Stats ===================\r\n",
						frame_error_count, checksum_error_count,
						data_anomaly_count, lost_pkt_count);

	USART_SendFormatted(huart,
						"\r\n==================== Frame Meta ====================\r\n"
						"Seq-ID          : %lu\r\n"
						"Unix-time (ns)  : %llu\r\n"
						"Δt to prev (ms) : %.3f\r\n"
						"Seq Gap         : %lu\r\n"
						"\r\n==================== Frame Meta ====================\r\n",
						(unsigned long)g_frame_seq_id,
						(unsigned long long)g_frame_time_ns,
						g_frame_latency, g_seq_gap);

#if SEND_DETAIL
	// 3. 72 B Payload（与原版保持一致，仅排版略紧凑）
	USART_SendFormatted(huart,
						"\r\n================== Parsed Payload ==================\r\n"
						"Desired Pos        : %.3f, %.3f, %.3f\r\n"
						"Desired Vel        : %.3f, %.3f, %.3f\r\n"
						"Desired Acc        : %.3f, %.3f, %.3f\r\n"
						"Desired Ang        : %.3f, %.3f, %.3f\r\n"
						"Desired AngVel     : %.3f, %.3f, %.3f\r\n"
						"Desired AngAcc     : %.3f, %.3f, %.3f\r\n"
						"Position           : %.3f, %.3f, %.3f\r\n"
						"Velocity           : %.3f, %.3f, %.3f\r\n"
						"Acceleration       : %.3f, %.3f, %.3f\r\n"
						"Angles             : %.3f, %.3f, %.3f\r\n"
						"Angular Vel        : %.3f, %.3f, %.3f\r\n"
						"Angular Acc        : %.3f, %.3f, %.3f\r\n"
						"\r\n================== Parsed Payload ==================\r\n",
						ctrl_input.desired_position[0], ctrl_input.desired_position[1], ctrl_input.desired_position[2],
						ctrl_input.desired_velocity[0], ctrl_input.desired_velocity[1], ctrl_input.desired_velocity[2],
						ctrl_input.desired_acceleration[0], ctrl_input.desired_acceleration[1], ctrl_input.desired_acceleration[2],
						ctrl_input.desired_angles[0], ctrl_input.desired_angles[1], ctrl_input.desired_angles[2],
						ctrl_input.desired_angular_velocity[0], ctrl_input.desired_angular_velocity[1], ctrl_input.desired_angular_velocity[2],
						ctrl_input.desired_angular_acceleration[0], ctrl_input.desired_angular_acceleration[1], ctrl_input.desired_angular_acceleration[2],
						ctrl_input.position[0], ctrl_input.position[1], ctrl_input.position[2],
						ctrl_input.velocity[0], ctrl_input.velocity[1], ctrl_input.velocity[2],
						ctrl_input.acceleration[0], ctrl_input.acceleration[1], ctrl_input.acceleration[2],
						ctrl_input.angles[0], ctrl_input.angles[1], ctrl_input.angles[2],
						ctrl_input.angular_velocity[0], ctrl_input.angular_velocity[1], ctrl_input.angular_velocity[2],
						ctrl_input.angular_acceleration[0], ctrl_input.angular_acceleration[1], ctrl_input.angular_acceleration[2]);

	// 通过TERM_UART打印控制器求解输出----------------------------------------------------------------
	USART_SendFormatted(huart, "\r\n=============== Control Force/Torque ===============\r\n");
	USART_SendFormatted(huart, "Thrust: [%.3f, %.3f, %.3f]\r\n", ctrl_output.thrust[0], ctrl_output.thrust[1], ctrl_output.thrust[2]);
	USART_SendFormatted(huart, "Torque: [%.3f, %.3f, %.3f]\r\n", ctrl_output.torque[0], ctrl_output.torque[1], ctrl_output.torque[2]);
	USART_SendFormatted(huart, "\r\n=============== Control Force/Torque ===============\r\n");
	//------------------------------------------------------------------------------------------

	// 通过TERM_UART打印风扇控制信息------------------------------------------------------------------
	USART_SendFormatted(huart, "\r\n=============== Calculated Fan Speed ===============\r\n");
	USART_SendFormatted(huart, "Fan duty rate in module 1(LT): LX+: %.3f, FY+: %.3f, LZ+: %.3f\r\n", Fan_Control_duty_rate.control_LX_p, Fan_Control_duty_rate.control_FY_p, Fan_Control_duty_rate.control_LZ_p);
	USART_SendFormatted(huart, "Fan duty rate in module 2(RT): RX-: %.3f, AY-: %.3f, RZ+: %.3f\r\n", Fan_Control_duty_rate.control_RX_n, Fan_Control_duty_rate.control_AY_n, Fan_Control_duty_rate.control_RZ_p);
	USART_SendFormatted(huart, "Fan duty rate in module 3(LB): LX-: %.3f, AY+: %.3f, LZ-: %.3f\r\n", Fan_Control_duty_rate.control_LX_n, Fan_Control_duty_rate.control_AY_p, Fan_Control_duty_rate.control_LZ_n);
	USART_SendFormatted(huart, "Fan duty rate in module 4(RB): RX+: %.3f, FY-: %.3f, RZ-: %.3f\r\n", Fan_Control_duty_rate.control_RX_p, Fan_Control_duty_rate.control_FY_n, Fan_Control_duty_rate.control_RZ_n);
	USART_SendFormatted(huart, "Desired Fan speed in module 1(LT): LX+: %.3f, FY+: %.3f, LZ+: %.3f\r\n", Fan_desire_Speed.omega_LX_p, Fan_desire_Speed.omega_FY_p, Fan_desire_Speed.omega_LZ_p);
	USART_SendFormatted(huart, "Desired Fan speed in module 2(RT): RX-: %.3f, AY-: %.3f, RZ+: %.3f\r\n", Fan_desire_Speed.omega_RX_n, Fan_desire_Speed.omega_AY_n, Fan_desire_Speed.omega_RZ_p);
	USART_SendFormatted(huart, "Desired Fan speed in module 3(LB): LX-: %.3f, AY+: %.3f, LZ-: %.3f\r\n", Fan_desire_Speed.omega_LX_n, Fan_desire_Speed.omega_AY_p, Fan_desire_Speed.omega_LZ_n);
	USART_SendFormatted(huart, "Desired Fan speed in module 4(RB): RX+: %.3f, FY-: %.3f, RZ-: %.3f\r\n", Fan_desire_Speed.omega_RX_p, Fan_desire_Speed.omega_FY_n, Fan_desire_Speed.omega_RZ_n);
	USART_SendFormatted(huart, "\r\n=============== Calculated Fan Speed ===============\r\n");
	//------------------------------------------------------------------------------------------
#endif
}
