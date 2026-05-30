#ifndef FOC_MY_TLE5012B_H
#define FOC_MY_TLE5012B_H

#include "main.h"

//命令字
#define CMD_READ_ANGLE_VALUE		0x8021  //读角度寄存器
#define CMD_READ_SPEED_VALUE		0x8031  //读角速度寄存器
#define CMD_READ_MOD2     0x8081  // 读配置寄存器 MOD2
#define CMD_READ_MOD3     0x8091  // 读配置寄存器 MOD3
#define CMD_WRITE_MOD2    0x0081  // 写配置寄存器 MOD2
#define CMD_WRITE_MOD3    0x0091  // 写配置寄存器 MOD3

//产生或取消片选信号宏函数定义
#define SPI_CS_ENABLE HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
#define SPI_CS_DISABLE HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

//读数据
uint16_t TLE5012B_SPI_Read(uint16_t Order_Word);

//写数据
uint16_t TLE5012B_SPI_Write(uint16_t Order_Word, uint16_t WriteData);

//读角度值
double TLE5012B_Angle(void);

//读角速度值
double TLE5012B_Speed(void);

#endif //FOC_MY_TLE5012B_H
