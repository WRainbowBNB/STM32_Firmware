#ifndef __SM_H__
#define __SM_H__

#include "main.h"
#include "stdio.h"
#include "string.h"
#include "usart.h"
#include "tim.h"

#define HEADER1 0x5A
#define HEADER2 0xA5

typedef struct{
	uint8_t *buf;
	uint16_t buf_size;
	uint16_t old_pos;
  uint16_t line_pos; 		// 当前行数据位置
	uint8_t line_temp[128]; // 用于存储一行数据
} RXBUFFER;

typedef union{
	float f;
	uint8_t u4[4];
} FLOAT_UNION;

void SM_UART(RXBUFFER *rx_buf);
void stm32_to_ros2(void);
uint8_t Read_Key(void);

extern RXBUFFER rx_buf;
extern uint8_t rx_buffer[64];

#endif 
