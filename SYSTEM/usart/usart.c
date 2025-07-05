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

/* ==================== 用户可调宏 ==================================== */
#define RB_CAPACITY (3 * FRAME_LEN) /* 足够缓存 3 帧 */
#define DBG_TX_PERIOD_MS 5000u		/* 调试信息刷新间隔 */
/* ==================================================================== */

/* ===================== DMA BEGIN: 全局缓冲 ===================*/
static uint8_t dma_rx_mem[RB_CAPACITY];
static RingBuf_t rb;
static uint32_t dbg_tick = 0;
/* ===================== DMA END ==============================*/

static uint8_t pc_rx_buf[PC_CMD_LEN];
static uint8_t pc_rx_sta = 0; /* 收到 '\n' 时置 1 */

// 串口句柄
UART_HandleTypeDef UART1_Handler; // UART1句柄
UART_HandleTypeDef UART2_Handler; // UART2句柄

// DMA句柄
extern DMA_HandleTypeDef hdma_usart2_rx; // 串口2 DMA接收句柄
extern DMA_HandleTypeDef hdma_usart1_tx; // 串口1 DMA发送句柄

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
static volatile uint32_t report_tick = 0;

// 用于计算帧间延迟的变量
static uint64_t prev_unix_ns = 0; /* 仅用于计算 inter-frame latency */
static u32 prev_seq_id = 0;

/******************* 发送族 ***********************/

// 等待 UART DMA 传输完成
static void uart_dma_wait_idle(UART_HandleTypeDef *h)
{
	while (h->gState != HAL_UART_STATE_READY)
		;
}

// 阻塞式发送，用于短报文发送
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

// DMA非阻塞发送，用于长报文发送
void USART_SendFormatted_DMA(UART_HandleTypeDef *huart, const char *format, ...)
{
	char buf[256];
	va_list ap;
	va_start(ap, format);
	int len = vsnprintf(buf, sizeof(buf), format, ap);
	va_end(ap);
	if (len <= 0)
		return;

	uart_dma_wait_idle(huart); /* 若上一次还没发完则等待 */
	HAL_UART_Transmit_DMA(huart, (uint8_t *)buf, len);
}

// 低层 DMA 发送
void dbg_tx_dma(UART_HandleTypeDef *huart, const char *msg, uint16_t len)
{
	uart_dma_wait_idle(huart);
	HAL_UART_Transmit_DMA(huart, (uint8_t *)msg, len);
}
/******************* 发送族 ***********************/

/******************* 初始化 ***********************/
// 串口初始化函数
void uart_init(u32 bound)
{
	// 数据口初始化
	DATA_UART.Instance = USART2;
	DATA_UART.Init.BaudRate = bound;				// 波特率
	DATA_UART.Init.WordLength = UART_WORDLENGTH_8B; // 字长为8位数据格式
	DATA_UART.Init.StopBits = UART_STOPBITS_1;		// 一个停止位
	DATA_UART.Init.Parity = UART_PARITY_NONE;		// 无奇偶校验位
	DATA_UART.Init.HwFlowCtl = UART_HWCONTROL_NONE; // 无硬件流控
	DATA_UART.Init.Mode = UART_MODE_TX_RX;			// 收发模式
	HAL_UART_Init(&DATA_UART);						// HAL_UART_Init()使能DATA_UART
	DMA_USART2_RX_Init(&DATA_UART);

	// 调试口初始化
	TERM_UART = (UART_HandleTypeDef){
		.Instance = USART1,
		.Init = DATA_UART.Init};
	HAL_UART_Init(&TERM_UART);
	DMA_USART1_TX_Init(&TERM_UART);
	HAL_UART_Receive_IT(&TERM_UART, pc_rx_buf, 1);

	uart_dma_wait_idle(&TERM_UART);
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
/******************* 初始化 ***********************/

/******************* 回调 *************************/
// IDLE 事件回调，仅更新 head 指针
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart->Instance == DATA_UART.Instance)
		rb_push(&rb, Size); /* 只移动 head 指针 */
}

// 串口接收错误回调函数：处理接收错误，清除错误标志并重新启动 DMA 接收
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != DATA_UART.Instance)
		return;
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE | UART_FLAG_FE))
	{
		__HAL_UART_CLEAR_OREFLAG(huart);
		/* 重新启动 DMA 到 IDLE — 2 µs 内可恢复 */
		HAL_UARTEx_ReceiveToIdle_DMA(&DATA_UART, dma_rx_mem, RB_CAPACITY);
	}
}

// 调试串口接收完成回调函数：当前使用回显作为占位符，并启动下一次接收
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance != TERM_UART.Instance)
		return;

	uint8_t ch = pc_rx_buf[pc_rx_sta];		   /* 刚收到的字节已在缓冲 */
	HAL_UART_Transmit(&TERM_UART, &ch, 1, 20); /* 立即回显 */

	if (ch == '\n' || ch == '\r')
		pc_rx_sta = 0; /* 重置索引 */
	else if (pc_rx_sta < PC_CMD_LEN - 1)
		pc_rx_sta++; /* 继续缓存 */

	/* 重新启动下一字节接收 */
	HAL_UART_Receive_IT(&TERM_UART, &pc_rx_buf[pc_rx_sta], 1);
}
/******************* 回调 *************************/

/******************** 数据帧操作 ******************/
// CRC
static uint16_t crc16_be_acc(const uint8_t *p, uint16_t n)
{
	uint16_t c = 0;
	while (n)
	{
		c += (uint16_t)(*p++) << 8 | *p++;
		n -= 2;
	}
	return c;
}

// 帧提取
bool frame_extract(uint8_t payload[72],
				   uint32_t *seq_id,
				   uint64_t *unix_ns)
{
	uint8_t tmp[FRAME_LEN];
	while (rb_size(&rb) >= FRAME_LEN)
	{
		rb_peek(&rb, tmp, FRAME_LEN);
		// 帧头/尾/CRC 校验
		if (tmp[0] == FRAME_HEADER_1 && tmp[1] == FRAME_HEADER_2 &&
			tmp[88] == FRAME_FOOTER_1 && tmp[89] == FRAME_FOOTER_2 &&
			crc16_be_acc(tmp + 2, 84) == ((tmp[86] << 8) | tmp[87]))
		{
			*seq_id = (tmp[2] << 24) | (tmp[3] << 16) | (tmp[4] << 8) | tmp[5];
			uint64_t t = 0;
			for (int i = 0; i < 8; ++i)
				t = (t << 8) | tmp[6 + i];
			*unix_ns = t;
			memcpy(payload, tmp + 14, PAYLOAD_LEN);
			rb_pop(&rb, FRAME_LEN);
			return true;
		}
		rb_pop(&rb, 1); // 滑窗前移，容忍噪声字节
		frame_error_count++;
	}
	return false;
}
/******************** 数据帧操作 ******************/

/******************* 解析 (Frame Apply) **********/
void parse_data(const uint8_t payload[72],
				uint32_t seq, uint64_t unix_ns,
				ControllerInput *ctrl_input,
				ControllerInput *prev_ctrl_input)
{
	/* 1) 延迟 & 丢包统计 */
	if (is_first_frame)
	{
		is_first_frame = false;
	}
	else
	{
		g_seq_gap = (seq > g_frame_seq_id) ? seq - g_frame_seq_id - 1 : 0;
		lost_pkt_count += g_seq_gap;
		g_frame_latency = (float)(unix_ns - g_frame_time_ns) / 1e6f; /* ms */
	}
	g_frame_seq_id = seq;
	g_frame_time_ns = unix_ns;

	/* 2) 72 B→ControllerInput (12×vec3) */
	const uint8_t *p = payload;
	for (int i = 0; i < 12; i++)
	{
		float *dst = ((float *)ctrl_input) + 3 * i;
		for (int k = 0; k < 3; k++, p += 2)
		{
			int16_t raw = be_to_i16(p);
			float val = scale_i16(raw);

			/* 跳变滤波 */
			float prev_val = ((float *)prev_ctrl_input)[3 * i + k];
			if (big_jump(val, prev_val))
			{
				frame_fault = true;
				data_anomaly_count++;
				val = prev_val;
			}
			dst[k] = val;
		}
	}
	memcpy(prev_ctrl_input, ctrl_input, sizeof(ControllerInput));
	frame_fault = false;
#if !OPERATING_MODE
	send_info(&TERM_UART);
#endif
}
/******************* 解析 (Frame Apply) **********/

// 发送解析后的数据用于验证
void send_info(UART_HandleTypeDef *huart)
{
	// 系统状态, 错误计数 & 帧元数据
	static char debug_buf[1024];
	int len = sprintf(debug_buf,
					  "\r\n=================== Bridge Stats ===================\r\n"
					  "Frame Errors      : %lu\r\n"
					  "Data Anomalies    : %lu\r\n"
					  "Lost Packets      : %llu"
					  "\r\n=================== Bridge Stats ===================\r\n"
					  "\r\n==================== Frame Meta ====================\r\n"
					  "Current Seq-ID    : %lu\r\n"
					  "Unix-time (ns)    : %llu\r\n"
					  "dt to prev (ms)   : %.3f\r\n"
					  "Seq Gap           : %lu"
					  "\r\n==================== Frame Meta ====================\r\n",
					  frame_error_count, data_anomaly_count, lost_pkt_count,
					  (unsigned long)g_frame_seq_id, (unsigned long long)g_frame_time_ns, g_frame_latency, g_seq_gap);
	USART_SendFormatted_DMA(huart, "%s", debug_buf);

// 调试模式发送详细信息
#if SEND_DETAIL
	// 解析后的数据
	static char frame_buf[1024];
	int detail_len = sprintf(frame_buf,
							 "\r\n================== Parsed Payload ==================\r\n"
							 "Desired Pos : %.3f, %.3f, %.3f\r\n"
							 "Desired Vel : %.3f, %.3f, %.3f\r\n"
							 "Desired Acc : %.3f, %.3f, %.3f\r\n"
							 "Desired Ang : %.3f, %.3f, %.3f\r\n"
							 "Desired Ang Vel : %.3f, %.3f, %.3f\r\n"
							 "Desired Ang Acc : %.3f, %.3f, %.3f\r\n"
							 "Position : %.3f, %.3f, %.3f\r\n"
							 "Velocity : %.3f, %.3f, %.3f\r\n"
							 "Acceleration : %.3f, %.3f, %.3f\r\n"
							 "Angles : %.3f, %.3f, %.3f\r\n"
							 "Angular Vel : %.3f, %.3f, %.3f\r\n"
							 "Angular Acc : %.3f, %.3f, %.3f"
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
	USART_SendFormatted_DMA(huart, "%s", frame_buf);

	// 控制输出
	static char control_buf[1024];
	int control_len = sprintf(control_buf,
							  "\r\n================== Control Output ===================\r\n"
							  "Thrust : [%.3f, %.3f, %.3f]\r\n"
							  "Torque : [%.3f, %.3f, %.3f]"
							  "\r\n================== Control Output ===================\r\n",
							  ctrl_output.thrust[0], ctrl_output.thrust[1], ctrl_output.thrust[2],
							  ctrl_output.torque[0], ctrl_output.torque[1], ctrl_output.torque[2]);
	USART_SendFormatted_DMA(huart, "%s", control_buf);

	// 风扇输出
	static char fan_buf[1024];
	int fan_len = sprintf(fan_buf,
						  "\r\n================== Fan Output ===================\r\n"
						  "Fan duty rate in module 1(LT): LX+: %.3f, FY+: %.3f, LZ+: %.3f\r\n"
						  "Fan duty rate in module 2(RT): RX-: %.3f, AY-: %.3f, RZ+: %.3f\r\n"
						  "Fan duty rate in module 3(LB): LX-: %.3f, AY+: %.3f, LZ-: %.3f\r\n"
						  "Fan duty rate in module 4(RB): RX+: %.3f, FY-: %.3f, RZ-: %.3f\r\n"
						  "Desired Fan speed in module 1(LT): LX+: %.3f, FY+: %.3f, LZ+: %.3f\r\n"
						  "Desired Fan speed in module 2(RT): RX-: %.3f, AY-: %.3f, RZ+: %.3f\r\n"
						  "Desired Fan speed in module 3(LB): LX-: %.3f, AY+: %.3f, LZ-: %.3f\r\n"
						  "Desired Fan speed in module 4(RB): RX+: %.3f, FY-: %.3f, RZ-: %.3f"
						  "\r\n================== Fan Output ===================\r\n",
						  Fan_Control_duty_rate.control_LX_p, Fan_Control_duty_rate.control_FY_p, Fan_Control_duty_rate.control_LZ_p,
						  Fan_Control_duty_rate.control_RX_n, Fan_Control_duty_rate.control_AY_n, Fan_Control_duty_rate.control_RZ_p,
						  Fan_Control_duty_rate.control_LX_n, Fan_Control_duty_rate.control_AY_p, Fan_Control_duty_rate.control_LZ_n,
						  Fan_Control_duty_rate.control_RX_p, Fan_Control_duty_rate.control_FY_n, Fan_Control_duty_rate.control_RZ_n,
						  Fan_desire_Speed.omega_LX_p, Fan_desire_Speed.omega_FY_p, Fan_desire_Speed.omega_LZ_p,
						  Fan_desire_Speed.omega_RX_n, Fan_desire_Speed.omega_AY_n, Fan_desire_Speed.omega_RZ_p,
						  Fan_desire_Speed.omega_LX_n, Fan_desire_Speed.omega_AY_p, Fan_desire_Speed.omega_LZ_n,
						  Fan_desire_Speed.omega_RX_p, Fan_desire_Speed.omega_FY_n, Fan_desire_Speed.omega_RZ_n);
	USART_SendFormatted_DMA(huart, "%s", fan_buf);
#endif
}
