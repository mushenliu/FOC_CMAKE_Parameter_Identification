#ifndef FOC_MY_JUSTFLOAT_H
#define FOC_MY_JUSTFLOAT_H

#include "main.h"

//JustFloat协议帧尾
#define JUSTFLOAT_FRAME_TAIL 0x7F800000u
//最大变量数
#define JUSTFLOAT_MAX_VAR 16

//串口句柄指针
static UART_HandleTypeDef *jf_huart = NULL;

//变量列表和变量数
static float *jf_var_list[JUSTFLOAT_MAX_VAR];
static uint8_t jf_var_cnt = 0;

//实际发送数据：变量+帧尾
static uint32_t jf_tx_buf[JUSTFLOAT_MAX_VAR + 1];

//初始化JustFloat函数
void JUSTFLOAT_Init(void);

//绑定串口
void JUSTFLOAT_BindUart(UART_HandleTypeDef *huart);

//添加新变量
void JUSTFLOAT_AddData(float *Data);

//发送变量列表
void JUSTFLOAT_SendData(void);

#endif
