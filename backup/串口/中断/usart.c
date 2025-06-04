#include "usart.h"
#include "Silde_Mode_Controller.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>
#include "delay.h"

extern ControllerInput ctrl_input;
extern ControllerInput prev_ctrl_input; // 上一个周期的控制输入，用于处理延迟或噪声

#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0xBB
#define FRAME_FOOTER_1 0xCC
#define FRAME_FOOTER_2 0xDD
#define DATA_LENGTH 72	  // 72字节，36个int16_t类型数据
#define CHECKSUM_LENGTH 2 // 校验和长度，2字节

u8 USART_RX_BUF[USART_REC_LEN];	  // 接收缓冲
u16 USART_RX_STA = 0;			  // 接收状态标记
u8 aRxBuffer[RXBUFFERSIZE];		  // HAL库使用的串口接收缓冲
UART_HandleTypeDef UART1_Handler; // UART句柄
bool frame_fault = false;		  // 数据帧滤波标志位，0为正常，1为未通过滤波
bool is_first_frame = true;		  // 标志是否是第一次接收数据
const float THRESHOLD = 120.0f;	  // 设定一个阈值，例如允许的最大变化量

// 加入以下代码,支持printf函数,而不需要选择use MicroLIB
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#if 1
// #pragma import(__use_no_semihosting)
//  标准库需要的支持函数
struct __FILE
{
	int handle;
};

FILE __stdout;
// 定义_sys_exit()以避免使用半主机模式
void _sys_exit(int x)
{
	x = x;
}
// 重定义fputc函数
int fputc(int ch, FILE *f)
{
	while ((USART1->ISR & 0X40) == 0)
		; // 循环发送,直到发送完毕
	USART1->TDR = (u8)ch;
	return ch;
}
#endif

UART_HandleTypeDef UART1_Handler; // UART句柄

// 初始化IO 串口1
// bound:波特率
void uart_init(u32 bound)
{
	// UART 初始化设置
	UART1_Handler.Instance = USART1;					// USART1
	UART1_Handler.Init.BaudRate = bound;				// 波特率
	UART1_Handler.Init.WordLength = UART_WORDLENGTH_8B; // 字长为8位数据格式
	UART1_Handler.Init.StopBits = UART_STOPBITS_1;		// 一个停止位
	UART1_Handler.Init.Parity = UART_PARITY_NONE;		// 无奇偶校验位
	UART1_Handler.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无硬件流控
	UART1_Handler.Init.Mode = UART_MODE_TX_RX;			// 收发模式
	HAL_UART_Init(&UART1_Handler);						// HAL_UART_Init()会使能UART1

	HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE); // 该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
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
		HAL_NVIC_SetPriority(USART1_IRQn, 3, 3); // 抢占优先级3，子优先级3
	}
}

// 串口接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) // 如果是串口1
	{
		if ((USART_RX_STA & 0x8000) == 0) // 接收未完成
		{
			if (USART_RX_STA & 0x4000) // 已经接收到帧尾的第一个字节（0xCC）
			{
				if (aRxBuffer[0] != FRAME_FOOTER_2) // 检查是否接收到帧尾的第二个字节（0xDD）
				{
					// printf("Invalid frame footer, resetting.\r\n");
					USART_RX_STA = 0; // 接收错误，重新开始
				}
				else
				{
					USART_RX_STA |= 0x8000; // 接收完成
					// printf("Frame received successfully.\r\n");

					// 解析数据
					parse_data(&ctrl_input, &prev_ctrl_input); // 调用数据解析函数，处理帧尾前的数据
					// send_parsed_data();						   // 发送解析后的数据用于验证

					// 控制器测试代码
					// 判断数据有效性，并调用控制器计算当前周期控制信号

					// 调用当前周期控制输入位置和姿态控制器计算控制输出
					Position_Controller(&ctrl_input, &ctrl_output);
					Attitude_Controller(&ctrl_input, &ctrl_output);

					// 打印输出，用于调试和验证
					printf("Thrust: [%.3f, %.3f, %.3f]\r\n", ctrl_output.thrust[0], ctrl_output.thrust[1], ctrl_output.thrust[2]);
					printf("Torque: [%.3f, %.3f, %.3f]\r\n", ctrl_output.torque[0], ctrl_output.torque[1], ctrl_output.torque[2]);

					USART_RX_STA = 0; // 重置接收状态，以便下一次接收
				}
			}
			else // 还没有收到帧尾的第一个字节
			{
				if (aRxBuffer[0] == FRAME_FOOTER_1) // 如果接收到的是帧尾的第一个字节（0xCC）
				{
					USART_RX_STA |= 0x4000; // 标记已收到第一个字节
				}
				else
				{
					USART_RX_BUF[USART_RX_STA & 0X3FFF] = aRxBuffer[0]; // 保存接收到的数据到缓冲区
					USART_RX_STA++;
					if (USART_RX_STA > (USART_REC_LEN - 1))
					{
						// printf("Buffer overflow, resetting.\r\n");
						USART_RX_STA = 0; // 接收数据错误，重新开始接收
					}
				}
			}
		}
		HAL_UART_Receive_IT(&UART1_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE); // 重新开启中断
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

// 解析并滤波二进制数据帧，将解析后的数据赋值给控制输入结构体
void parse_data(ControllerInput *ctrl_input, ControllerInput *prev_ctrl_input)
{
	u16 data_length = (USART_RX_STA & 0x3FFF); // 去除接收状态标志，获取有效数据长度

	// 预期的总帧长度：帧头（2字节） + 数据段（72字节） + 校验和（2字节）
	u16 expected_length = 2 + DATA_LENGTH + CHECKSUM_LENGTH;

	if (data_length >= expected_length)
	{
		// 获取有效数据的起始位置
		u8 *valid_data = USART_RX_BUF + (data_length - expected_length);

		// // 打印有效数据
		// for (int i = 0; i < DATA_LENGTH; i++)
		// {
		// 	printf("Byte %d: 0x%02X ", i, valid_data[i]);
		// }

		// 检测帧头是否正确
		if (valid_data[0] == FRAME_HEADER_1 && valid_data[1] == FRAME_HEADER_2)
		{
			// 计算校验和，确保数据的完整性
			uint16_t checksum = 0;
			for (int i = 2; i < (DATA_LENGTH + 2); i += 2)
			{
				uint16_t value = (valid_data[i] << 8) | valid_data[i + 1];
				checksum += value;
			}

			// printf("checksum is %04X\r\n", checksum);
			uint16_t received_checksum = (valid_data[DATA_LENGTH + 2] << 8) | valid_data[DATA_LENGTH + 3];

			if (checksum == received_checksum)
			{
				// 校验通过，遍历数据，解析并存储到控制输入结构体
				for (int i = 0; i < 3; i++)
				{
					// 原始单位为m，缩放因子为1000，表示小数点后三位，为mm级精度。
					ctrl_input->desired_position[i] = (float)((int16_t)((valid_data[2 + i * 2] << 8) | valid_data[2 + i * 2 + 1])) / 1000.0f;
					ctrl_input->desired_velocity[i] = (float)((int16_t)((valid_data[2 + 6 + i * 2] << 8) | valid_data[2 + 6 + i * 2 + 1])) / 1000.0f;
					ctrl_input->desired_acceleration[i] = (float)((int16_t)((valid_data[2 + 12 + i * 2] << 8) | valid_data[2 + 12 + i * 2 + 1])) / 1000.0f;
					ctrl_input->desired_angles[i] = (float)((int16_t)((valid_data[2 + 18 + i * 2] << 8) | valid_data[2 + 18 + i * 2 + 1])) / 1000.0f;
					ctrl_input->desired_angular_velocity[i] = (float)((int16_t)((valid_data[2 + 24 + i * 2] << 8) | valid_data[2 + 24 + i * 2 + 1])) / 1000.0f;
					ctrl_input->desired_angular_acceleration[i] = (float)((int16_t)((valid_data[2 + 30 + i * 2] << 8) | valid_data[2 + 30 + i * 2 + 1])) / 1000.0f;
					ctrl_input->position[i] = (float)((int16_t)((valid_data[2 + 36 + i * 2] << 8) | valid_data[2 + 36 + i * 2 + 1])) / 1000.0f;
					ctrl_input->velocity[i] = (float)((int16_t)((valid_data[2 + 42 + i * 2] << 8) | valid_data[2 + 42 + i * 2 + 1])) / 1000.0f;
					ctrl_input->acceleration[i] = (float)((int16_t)((valid_data[2 + 48 + i * 2] << 8) | valid_data[2 + 48 + i * 2 + 1])) / 1000.0f;
					ctrl_input->angles[i] = (float)((int16_t)((valid_data[2 + 54 + i * 2] << 8) | valid_data[2 + 54 + i * 2 + 1])) / 1000.0f;
					ctrl_input->angular_velocity[i] = (float)((int16_t)((valid_data[2 + 60 + i * 2] << 8) | valid_data[2 + 60 + i * 2 + 1])) / 1000.0f;
					ctrl_input->angular_acceleration[i] = (float)((int16_t)((valid_data[2 + 66 + i * 2] << 8) | valid_data[2 + 66 + i * 2 + 1])) / 1000.0f;
				}

				if (is_first_frame) // 是否第一次接收数据
				{
					// 第一次接收到有效数据时，跳过滤波，将当前输入直接备份
					memcpy(prev_ctrl_input, ctrl_input, sizeof(ControllerInput));
					is_first_frame = false; // 标志为已处理
				}
				else
				{
					// 滤波器：检查是否存在异常突变
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
							// 数据异常，标志位翻转
							frame_fault = true;
						}
					}

					if (!frame_fault) // 数据标志位检测
					{
						// 数据正常，将当前输入备份为上一个周期的数据
						*prev_ctrl_input = *ctrl_input;
						// printf("Data parsed and stored successfully.\r\n");
						return;
					}
					else
					{
						// 若存在异常，使用上一个周期的数据
						*ctrl_input = *prev_ctrl_input;
						// printf("data anomaly.\r\n");
						// 重置异常标志
						frame_fault = false;
					}
				}
			}

			else
			{
				// 如果校验和不正确，则保持上一个周期的输入
				// printf("Checksum verification failed. Using previous control input.\r\n");
			}
		}
		else
		{
			// 如果帧头不正确，则保持上一个周期的输入
			// printf("Frame header incorrect. Using previous control input.\r\n");
		}
	}
	else
	{
		// 如果数据帧有效长度不足，则保持上一个周期的输入
		// printf("Incomplete frame received. Using previous control input.\r\n");
	}
}

// 发送解析后的数据用于验证
void send_parsed_data(void)
{
	char buffer[512];
	int len = snprintf(buffer, sizeof(buffer),
					   "Parsed Data:\r\n"
					   "Desired Position: %.3f, %.3f, %.3f\r\n"
					   "Desired Velocity: %.3f, %.3f, %.3f\r\n"
					   "Desired Acceleration: %.3f, %.3f, %.3f\r\n"
					   "Desired Angles: %.3f, %.3f, %.3f\r\n"
					   "Desired Angular Velocity: %.3f, %.3f, %.3f\r\n"
					   "Desired Angular Acceleration: %.3f, %.3f, %.3f\r\n"
					   "Position: %.3f, %.3f, %.3f\r\n"
					   "Velocity: %.3f, %.3f, %.3f\r\n"
					   "Acceleration: %.3f, %.3f, %.3f\r\n"
					   "Angles: %.3f, %.3f, %.3f\r\n"
					   "Angular Velocity: %.3f, %.3f, %.3f\r\n"
					   "Angular Acceleration: %.3f, %.3f, %.3f\r\n",
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
	HAL_UART_Transmit(&UART1_Handler, (uint8_t *)buffer, len, 1000);
}
