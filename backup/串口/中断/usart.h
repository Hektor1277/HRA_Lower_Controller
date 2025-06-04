#ifndef _USART_H
#define _USART_H
#include "sys.h"
#include "stdio.h"
#include "Silde_Mode_Controller.h"
#include <stdbool.h>

#define USART_REC_LEN 80 // 定义最大接收字节数（包括帧头、数据、校验和、帧尾）
#define EN_USART1_RX 1   // 使能（1）/禁止（0）串口1接收

extern u8 USART_RX_BUF[USART_REC_LEN];   // 接收缓冲，最大USART_REC_LEN个字节
extern u16 USART_RX_STA;                 // 接收状态标记
extern UART_HandleTypeDef UART1_Handler; // UART句柄

#define RXBUFFERSIZE 1             // 缓存大小
extern u8 aRxBuffer[RXBUFFERSIZE]; // HAL库USART接收Buffer

// 串口初始化函数声明
void uart_init(u32 bound);
void send_parsed_data(void);
void parse_data(ControllerInput *ctrl_input, ControllerInput *prev_ctrl_input); // 修改为处理int16_t类型数据

#endif
