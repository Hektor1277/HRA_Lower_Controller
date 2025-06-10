#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"
#include "Silde_Mode_Controller.h"
#include <stdbool.h>

// 缓冲区定义
#define PC_CMD_LEN 128   // 定义PC命令接收缓冲区长度
#define USART_REC_LEN 80 // 定义最大接收字节数（包括帧头、数据、校验和、帧尾）
#define RXBUFFERSIZE 1   // 缓存大小

#define EN_USART1_RX 1 // 使能（1）/禁止（0）串口1接收

// 帧格式定义
#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0xBB
#define FRAME_FOOTER_1 0xCC
#define FRAME_FOOTER_2 0xDD
#define DATA_LENGTH 72    // 72字节，36个int16_t类型数据
#define CHECKSUM_LENGTH 2 // 校验和长度，2字节

// 串口发送格式化字符串
void USART_SendFormatted(UART_HandleTypeDef *huart, const char *format, ...);

// 串口初始化函数声明
void uart_init(u32 bound);
void send_parsed_data(UART_HandleTypeDef *huart);
void parse_data(ControllerInput *ctrl_input, ControllerInput *prev_ctrl_input); // 修改为处理int16_t类型数据

#endif
