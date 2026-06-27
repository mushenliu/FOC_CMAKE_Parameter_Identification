#ifndef FOC_MY_CONTROL_H
#define FOC_MY_CONTROL_H

#include "main.h"
#include "MY_CALLBACK.h"

//克拉克变换、帕克变换、反克拉克变换、反帕克变换
void Clarke_Trans(float A, float B, float C, float *alpha, float *beta);

void Inv_Clarke_Trans(float alpha, float beta, float *A, float *B, float *C);

void Park_Trans(float alpha, float beta, float Sin, float Cos, float *D, float *Q);

void Inv_Park_Trans(float D, float Q, float Sin, float Cos, float *alpha, float *beta);

//使用DSP浮点计算三角函数
void DSP_Float_Calc_SinCos(float theta, float *Sin, float *Cos);

//SVPWM计算法计算三相占空比
void SVPWM_Calculation(float* Ud, float* Uq, float Sin, float Cos, float U_svpwm_max,float Udc,
    float* Duty_A, float* Duty_B,float* Duty_C);

//将三相占空比转换为CCR值并更新TIM
void Set_CCR(float Duty_A, float Duty_B, float Duty_C);

// 最高阶次5的线性控制器
void Discrete_Controller(Discrete_Controller_Struct *Controller);

//电流环控制函数
void Current_Control();

//速度环控制函数
void Speed_Control();

// 最高阶次5的线性控制器初始化函数
void Controller_Init(float a1, float a2, float a3, float a4, float a5, float b0,
                     float b1, float b2, float b3, float b4, float b5, float Default_Set,
                     Discrete_Controller_Struct *Controller);

#endif //FOC_MY_CONTROL_H
