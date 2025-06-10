#include "Silde_Mode_Controller.h"
#include "Fan.h"
#include "usart.h"
#include "delay.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

// 全局变量 & 句柄
extern ControllerInput ctrl_input;
extern ControllerInput prev_ctrl_input; // 上一个周期的控制输入，用于处理延迟或噪声
extern ControllerOutput ctrl_output;
extern FanSpeed Fan_desire_Speed;
extern FanControl Fan_Control_duty_rate;

// 串口句柄
UART_HandleTypeDef UART1_Handler; // UART1句柄
UART_HandleTypeDef UART2_Handler; // UART2句柄

// 串口1 接收状态与缓冲（调试/命令）
static u8 pc_rx_buf[PC_CMD_LEN];	  // 接收缓冲，最大PC_CMD_LEN个字节
static u16 pc_rx_sta = 0;			  // 串口1接收状态标记
static u8 pc_aRxBuffer[RXBUFFERSIZE]; // 存储接收到的数据

// 串口2 接收状态与缓冲（数据）
static u8 data_rx_buf[USART_REC_LEN];	 // 接收缓冲，最大USART_REC_LEN个字节
static u16 data_rx_sta = 0;				 // 串口2接收状态标记
static u8 data_aRx2Buffer[RXBUFFERSIZE]; // HAL库USART2接收Buffer

bool frame_fault = false;		// 数据帧滤波标志位，0为正常，1为未通过滤波
bool is_first_frame = true;		// 标志是否是第一次接收数据
const float THRESHOLD = 120.0f; // 设定一个阈值，例如允许的最大变化量

// 错误计数器
static u32 frame_error_count = 0;
static u32 checksum_error_count = 0;
static u32 data_anomaly_count = 0;

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
	// USART1 初始化设置
	UART1_Handler.Instance = USART1;									 // USART1
	UART1_Handler.Init.BaudRate = bound;								 // 波特率
	UART1_Handler.Init.WordLength = UART_WORDLENGTH_8B;					 // 字长为8位数据格式
	UART1_Handler.Init.StopBits = UART_STOPBITS_1;						 // 一个停止位
	UART1_Handler.Init.Parity = UART_PARITY_NONE;						 // 无奇偶校验位
	UART1_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;					 // 无硬件流控
	UART1_Handler.Init.Mode = UART_MODE_TX_RX;							 // 收发模式
	HAL_UART_Init(&UART1_Handler);										 // HAL_UART_Init()使能UART1
	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)pc_aRxBuffer, PC_CMD_LEN); // 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量

	// USART2 初始化设置
	UART2_Handler.Instance = USART2;
	UART2_Handler.Init.BaudRate = bound;
	UART2_Handler.Init.WordLength = UART_WORDLENGTH_8B;
	UART2_Handler.Init.StopBits = UART_STOPBITS_1;
	UART2_Handler.Init.Parity = UART_PARITY_NONE;
	UART2_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UART2_Handler.Init.Mode = UART_MODE_TX_RX;
	HAL_UART_Init(&UART2_Handler); // HAL_UART_Init()使能UART2
	HAL_UART_Receive_IT(&UART2_Handler, (u8 *)data_aRx2Buffer, RXBUFFERSIZE);
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
		HAL_NVIC_SetPriority(USART1_IRQn, 1, 3); // 抢占优先级1，子优先级3
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
		HAL_NVIC_SetPriority(USART2_IRQn, 1, 3); // 抢占优先级1，子优先级3
	}
}

// 串口接收完成回调：在 USART2 中进行数据接收处理逻辑；通过 USART1_SendFormatted() 输出所有调试信息
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2) // 如果是串口2
	{
		if ((data_rx_sta & 0x8000) == 0) // 接收未完成
		{
			if (data_rx_sta & 0x4000) // 已经接收到帧尾的第一个字节（0xCC）
			{
				if (data_aRx2Buffer[0] != FRAME_FOOTER_2) // 检查是否接收到帧尾的第二个字节（0xDD）
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
					send_parsed_data(&UART1_Handler);		   // 发送解析后的数据用于验证

					// 控制器测试代码
					// 调用当前周期控制输入位置和姿态控制器计算控制输出
					Position_Controller(&ctrl_input, &ctrl_output);
					Attitude_Controller(&ctrl_input, &ctrl_output);

					// 通过串口1打印输出，用于调试和验证-----------------------------------------------------------
					USART_SendFormatted(&UART1_Handler, "Thrust: [%.3f, %.3f, %.3f]\r\n", ctrl_output.thrust[0], ctrl_output.thrust[1], ctrl_output.thrust[2]);
					USART_SendFormatted(&UART1_Handler, "Torque: [%.3f, %.3f, %.3f]\r\n", ctrl_output.torque[0], ctrl_output.torque[1], ctrl_output.torque[2]);
					//------------------------------------------------------------------------------------------

					// 调用底层风扇控制函数，根据控制输出计算风扇转速并输出相应PWM信号
					Fan_Rotation_Control(&ctrl_output, &Fan_desire_Speed, &Fan_Control_duty_rate);

					// 通过串口1打印风扇控制信息------------------------------------------------------------------
					USART_SendFormatted(&UART1_Handler, "Fan duty rate in module 1(LT): LX+: %.3f, FY+: %.3f, LZ+: %.3f\r\n", Fan_Control_duty_rate.control_LX_p, Fan_Control_duty_rate.control_FY_p, Fan_Control_duty_rate.control_LZ_p);
					USART_SendFormatted(&UART1_Handler, "Fan duty rate in module 2(RT): RX-: %.3f, AY-: %.3f, RZ+: %.3f\r\n", Fan_Control_duty_rate.control_RX_n, Fan_Control_duty_rate.control_AY_n, Fan_Control_duty_rate.control_RZ_p);
					USART_SendFormatted(&UART1_Handler, "Fan duty rate in module 3(LB): LX-: %.3f, AY+: %.3f, LZ-: %.3f\r\n", Fan_Control_duty_rate.control_LX_n, Fan_Control_duty_rate.control_AY_p, Fan_Control_duty_rate.control_LZ_n);
					USART_SendFormatted(&UART1_Handler, "Fan duty rate in module 4(RB): RX+: %.3f, FY-: %.3f, RZ-: %.3f\r\n", Fan_Control_duty_rate.control_RX_p, Fan_Control_duty_rate.control_FY_n, Fan_Control_duty_rate.control_RZ_n);
					USART_SendFormatted(&UART1_Handler, "Desired Fan speed in module 1(LT): LX+: %.3f, FY+: %.3f, LZ+: %.3f\r\n", Fan_desire_Speed.omega_LX_p, Fan_desire_Speed.omega_FY_p, Fan_desire_Speed.omega_LZ_p);
					USART_SendFormatted(&UART1_Handler, "Desired Fan speed in module 2(RT): RX-: %.3f, AY-: %.3f, RZ+: %.3f\r\n", Fan_desire_Speed.omega_RX_n, Fan_desire_Speed.omega_AY_n, Fan_desire_Speed.omega_RZ_p);
					USART_SendFormatted(&UART1_Handler, "Desired Fan speed in module 3(LB): LX-: %.3f, AY+: %.3f, LZ-: %.3f\r\n", Fan_desire_Speed.omega_LX_n, Fan_desire_Speed.omega_AY_p, Fan_desire_Speed.omega_LZ_n);
					USART_SendFormatted(&UART1_Handler, "Desired Fan speed in module 4(RB): RX+: %.3f, FY-: %.3f, RZ-: %.3f\r\n", Fan_desire_Speed.omega_RX_p, Fan_desire_Speed.omega_FY_n, Fan_desire_Speed.omega_RZ_n);
					//------------------------------------------------------------------------------------------

					data_rx_sta = 0; // 重置接收状态，以便下一次接收
				}
			}
			else // 还没有收到帧尾的第一个字节
			{
				if (data_aRx2Buffer[0] == FRAME_FOOTER_1) // 如果接收到的是帧尾的第一个字节（0xCC）
				{
					data_rx_sta |= 0x4000; // 标记已收到第一个字节
				}
				else
				{
					if (data_rx_sta < USART_REC_LEN)
					{
						data_rx_buf[data_rx_sta++] = data_aRx2Buffer[0]; // 保存接收到的数据到缓冲区
					}
					else
					{
						USART_SendFormatted(&UART1_Handler, "Buffer overflow, resetting.\r\n");
						frame_error_count++;
						data_rx_sta = 0; // 缓冲区溢出，重新开始
					}
				}
			}
		}
		HAL_UART_Receive_IT(&UART2_Handler, (u8 *)data_aRx2Buffer, RXBUFFERSIZE); // 重新开启中断
	}
	else if (huart->Instance == USART1)
	{
		// 简单回显或命令解析示例
		if (pc_rx_sta < PC_CMD_LEN) // 防止缓冲区溢出
		{
			pc_rx_buf[pc_rx_sta++] = pc_aRxBuffer[0];
			if (pc_rx_buf[pc_rx_sta - 1] == '\n' || pc_rx_sta >= PC_CMD_LEN) // 收到换行或缓冲区满
			{
				HAL_UART_Transmit(&UART1_Handler, pc_rx_buf, pc_rx_sta, HAL_MAX_DELAY);
				pc_rx_sta = 0; // 重置接收状态
			}
		}
		else
		{
			pc_rx_sta = 0; // 缓冲区溢出，重置
		}
		HAL_UART_Receive_IT(&UART1_Handler, pc_aRxBuffer, RXBUFFERSIZE); // 重新开启中断
	}
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

// 解析并滤波二进制数据帧，将解析后的数据赋值给控制输入结构体
void parse_data(ControllerInput *ctrl_input, ControllerInput *prev_ctrl_input)
{
	u16 data_length = (data_rx_sta & 0x3FFF); // 去除接收状态标志，获取有效数据长度

	// 预期的总帧长度：帧头(2) + 数据(72) + 校验和(2) + 帧尾(2) = 78字节
	u16 expected_length = 2 + DATA_LENGTH + CHECKSUM_LENGTH + 2;

	if (data_length < expected_length)
	{
		frame_error_count++;
		return; // 数据长度不足，直接返回
	}

	// 获取有效数据的起始位置(从帧头开始)
	u8 *valid_data = NULL;
	for (int i = 0; i <= (data_length - expected_length); i++)
	{
		if (data_rx_buf[i] == FRAME_HEADER_1 && data_rx_buf[i + 1] == FRAME_HEADER_2)
		{
			valid_data = &data_rx_buf[i];
			break;
		}
	}

	if (!valid_data)
	{
		frame_error_count++;
		return; // 未找到有效帧头
	}

	// 检查帧尾
	if (valid_data[expected_length - 2] != FRAME_FOOTER_1 ||
		valid_data[expected_length - 1] != FRAME_FOOTER_2)
	{
		frame_error_count++;
		return; // 帧尾不正确
	}

	// 计算校验和(仅计算数据部分)
	uint16_t checksum = 0;
	for (int i = 2; i < (DATA_LENGTH + 2); i += 2)
	{
		checksum += (valid_data[i] << 8) | valid_data[i + 1];
	}

	uint16_t received_checksum = (valid_data[DATA_LENGTH + 2] << 8) | valid_data[DATA_LENGTH + 3];

	if (checksum != received_checksum)
	{
		checksum_error_count++;
		return; // 校验和错误
	}

	// 为保证更新ctrl_input时安全，在更新时关闭中断响应
	__disable_irq();

	// 校验通过，遍历数据，解析并存储到控制输入结构体
	for (int i = 0; i < 3; i++)
	{
		ctrl_input->desired_position[i] = (float)((int16_t)((valid_data[2 + i * 2] << 8) | valid_data[2 + i * 2 + 1]) / 1000.0f);
		ctrl_input->desired_velocity[i] = (float)((int16_t)((valid_data[2 + 6 + i * 2] << 8) | valid_data[2 + 6 + i * 2 + 1]) / 1000.0f);
		ctrl_input->desired_acceleration[i] = (float)((int16_t)((valid_data[2 + 12 + i * 2] << 8) | valid_data[2 + 12 + i * 2 + 1]) / 1000.0f);
		ctrl_input->desired_angles[i] = (float)((int16_t)((valid_data[2 + 18 + i * 2] << 8) | valid_data[2 + 18 + i * 2 + 1]) / 1000.0f);
		ctrl_input->desired_angular_velocity[i] = (float)((int16_t)((valid_data[2 + 24 + i * 2] << 8) | valid_data[2 + 24 + i * 2 + 1]) / 1000.0f);
		ctrl_input->desired_angular_acceleration[i] = (float)((int16_t)((valid_data[2 + 30 + i * 2] << 8) | valid_data[2 + 30 + i * 2 + 1]) / 1000.0f);
		ctrl_input->position[i] = (float)((int16_t)((valid_data[2 + 36 + i * 2] << 8) | valid_data[2 + 36 + i * 2 + 1]) / 1000.0f);
		ctrl_input->velocity[i] = (float)((int16_t)((valid_data[2 + 42 + i * 2] << 8) | valid_data[2 + 42 + i * 2 + 1]) / 1000.0f);
		ctrl_input->acceleration[i] = (float)((int16_t)((valid_data[2 + 48 + i * 2] << 8) | valid_data[2 + 48 + i * 2 + 1]) / 1000.0f);
		ctrl_input->angles[i] = (float)((int16_t)((valid_data[2 + 54 + i * 2] << 8) | valid_data[2 + 54 + i * 2 + 1]) / 1000.0f);
		ctrl_input->angular_velocity[i] = (float)((int16_t)((valid_data[2 + 60 + i * 2] << 8) | valid_data[2 + 60 + i * 2 + 1]) / 1000.0f);
		ctrl_input->angular_acceleration[i] = (float)((int16_t)((valid_data[2 + 66 + i * 2] << 8) | valid_data[2 + 66 + i * 2 + 1]) / 1000.0f);
	}

	__enable_irq();

	if (is_first_frame) // 是否第一次接收数据
	{
		// 第一次接收到有效数据时，跳过滤波，将当前输入直接备份
		*prev_ctrl_input = *ctrl_input;
		is_first_frame = false; // 标志为已处理
	}
	else
	{
		// 滤波器：检查是否存在异常突变
		frame_fault = false;
		for (int i = 0; i < 3; i++)
		{
			if (fabsf(ctrl_input->desired_position[i] - prev_ctrl_input->desired_position[i]) > THRESHOLD ||
				fabsf(ctrl_input->desired_velocity[i] - prev_ctrl_input->desired_velocity[i]) > THRESHOLD ||
				fabsf(ctrl_input->desired_acceleration[i] - prev_ctrl_input->desired_acceleration[i]) > THRESHOLD ||
				fabsf(ctrl_input->desired_angles[i] - prev_ctrl_input->desired_angles[i]) > THRESHOLD ||
				fabsf(ctrl_input->desired_angular_velocity[i] - prev_ctrl_input->desired_angular_velocity[i]) > THRESHOLD ||
				fabsf(ctrl_input->desired_angular_acceleration[i] - prev_ctrl_input->desired_angular_acceleration[i]) > THRESHOLD ||
				fabsf(ctrl_input->position[i] - prev_ctrl_input->position[i]) > THRESHOLD ||
				fabsf(ctrl_input->velocity[i] - prev_ctrl_input->velocity[i]) > THRESHOLD ||
				fabsf(ctrl_input->acceleration[i] - prev_ctrl_input->acceleration[i]) > THRESHOLD ||
				fabsf(ctrl_input->angles[i] - prev_ctrl_input->angles[i]) > THRESHOLD ||
				fabsf(ctrl_input->angular_velocity[i] - prev_ctrl_input->angular_velocity[i]) > THRESHOLD ||
				fabsf(ctrl_input->angular_acceleration[i] - prev_ctrl_input->angular_acceleration[i]) > THRESHOLD)
			{
				frame_fault = true;
				break;
			}
		}

		if (!frame_fault)
		{
			// 数据正常，将当前输入备份为上一个周期的数据
			*prev_ctrl_input = *ctrl_input;
		}
		else
		{
			// 若存在异常，使用上一个周期的数据
			data_anomaly_count++;
			*ctrl_input = *prev_ctrl_input;
			frame_fault = false;
		}
	}
}

// 发送解析后的数据用于验证
void send_parsed_data(UART_HandleTypeDef *huart)
{
	USART_SendFormatted(huart, "=== System Status ===\r\n");
	USART_SendFormatted(huart, "Frame Errors: %lu\r\n", frame_error_count);
	USART_SendFormatted(huart, "Checksum Errors: %lu\r\n", checksum_error_count);
	USART_SendFormatted(huart, "Data Anomalies: %lu\r\n\r\n", data_anomaly_count);

	USART_SendFormatted(huart, "Parsed Data:\r\n");
	USART_SendFormatted(huart, "Desired Position: %.3f, %.3f, %.3f\r\n",
						ctrl_input.desired_position[0], ctrl_input.desired_position[1], ctrl_input.desired_position[2]);
	USART_SendFormatted(huart, "Desired Velocity: %.3f, %.3f, %.3f\r\n",
						ctrl_input.desired_velocity[0], ctrl_input.desired_velocity[1], ctrl_input.desired_velocity[2]);
	USART_SendFormatted(huart, "Desired Acceleration: %.3f, %.3f, %.3f\r\n",
						ctrl_input.desired_acceleration[0], ctrl_input.desired_acceleration[1], ctrl_input.desired_acceleration[2]);
	USART_SendFormatted(huart, "Desired Angles: %.3f, %.3f, %.3f\r\n",
						ctrl_input.desired_angles[0], ctrl_input.desired_angles[1], ctrl_input.desired_angles[2]);
	USART_SendFormatted(huart, "Desired Angular Velocity: %.3f, %.3f, %.3f\r\n",
						ctrl_input.desired_angular_velocity[0], ctrl_input.desired_angular_velocity[1], ctrl_input.desired_angular_velocity[2]);
	USART_SendFormatted(huart, "Desired Angular Acceleration: %.3f, %.3f, %.3f\r\n",
						ctrl_input.desired_angular_acceleration[0], ctrl_input.desired_angular_acceleration[1], ctrl_input.desired_angular_acceleration[2]);
	USART_SendFormatted(huart, "Position: %.3f, %.3f, %.3f\r\n",
						ctrl_input.position[0], ctrl_input.position[1], ctrl_input.position[2]);
	USART_SendFormatted(huart, "Velocity: %.3f, %.3f, %.3f\r\n",
						ctrl_input.velocity[0], ctrl_input.velocity[1], ctrl_input.velocity[2]);
	USART_SendFormatted(huart, "Acceleration: %.3f, %.3f, %.3f\r\n",
						ctrl_input.acceleration[0], ctrl_input.acceleration[1], ctrl_input.acceleration[2]);
	USART_SendFormatted(huart, "Angles: %.3f, %.3f, %.3f\r\n",
						ctrl_input.angles[0], ctrl_input.angles[1], ctrl_input.angles[2]);
	USART_SendFormatted(huart, "Angular Velocity: %.3f, %.3f, %.3f\r\n",
						ctrl_input.angular_velocity[0], ctrl_input.angular_velocity[1], ctrl_input.angular_velocity[2]);
	USART_SendFormatted(huart, "Angular Acceleration: %.3f, %.3f, %.3f\r\n",
						ctrl_input.angular_acceleration[0], ctrl_input.angular_acceleration[1], ctrl_input.angular_acceleration[2]);
}