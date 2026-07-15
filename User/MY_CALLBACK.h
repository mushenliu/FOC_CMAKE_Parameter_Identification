#ifndef FOC_MY_CALLBACK_H
#define FOC_MY_CALLBACK_H
#include "tim.h"

#include "main.h"

//数学常数
#define SQRT3 1.7320508065

//电机参数
#define MOTOR_POLE_PAIRS 14
//相电阻，单位Ω
#define MOTOR_R 5.70125
//D轴同步电感，单位H
#define MOTOR_Ld 0.00186152
//永磁体磁链，单位Wb
#define MOTOR_Psi 0.00965548
//Q轴同步电感，单位H
#define MOTOR_Lq 0.00194861

//速度环一阶低通滤波系数
#define Speed_Filter 0.1

// D轴电流环线性控制器参数
#define D_a1 -1.123
#define D_a2 0.1227
#define D_a3 0
#define D_a4 0
#define D_a5 0
#define D_b0 5.998
#define D_b1 0.6373
#define D_b2 -5.361
#define D_b3 0
#define D_b4 0
#define D_b5 0

// Q轴电流环线性控制器参数
#define Q_a1 -1.123
#define Q_a2 0.1227
#define Q_a3 0
#define Q_a4 0
#define Q_a5 0
#define Q_b0 6.808
#define Q_b1 0.7233
#define Q_b2 -6.085
#define Q_b3 0
#define Q_b4 0
#define Q_b5 0

// 速度环线性控制器参数
#define Speed_a1 -1
#define Speed_a2 0
#define Speed_a3 0
#define Speed_a4 0
#define Speed_a5 0
#define Speed_b0 0.001002
#define Speed_b1 -0.0009975
#define Speed_b2 0
#define Speed_b3 0
#define Speed_b4 0
#define Speed_b5 0


//速度环和电流环采样周期，单位s
#define Ts_Current ((__HAL_TIM_GET_AUTORELOAD(&htim1)+1)/170000000.0) * 2
#define Ts_Speed    ((__HAL_TIM_GET_AUTORELOAD(&htim2)+1)/170000000.0)

//默认母线电压，单位V
#define U_DC_Default 12

//默认给定电流，单位A
#define ID_Target_Default 0
#define IQ_Target_Default 0
//默认给定转速，单位rpm
#define Speed_Target_Default 0
//默认驱动电压，单位V
#define Ud_Default 0
#define Uq_Default 0

//参数辨识标志位
//1——相电阻，D轴方向
//2——相电阻，Q轴方向
//3——伪随机辨识Ld同步电感
//4——永磁体磁链辨识
//5——伪随机辨识Lq同步电感
//6——D轴开环方波响应模型验证
//7——Q轴开环方波响应模型验证
//8——D轴闭环方波响应模型验证
//9——Q轴闭环方波响应模型验证
//10——伪随机辨识速度环被控对象
//11——速度环开环方波响应模型验证
#define Identification_Mode_Default 10

//速度环输出限幅（电流环给定限幅）
#define Speed_Output_Limit 0.5

//PRBS序列参数
//级数
#define PRBS_N 11
//幅值
/*
辨识D轴电感时用1即可，
辨识Q轴电感和速度环时为了获取较高的信噪比选用2
*/
#define PRBS_A 2
//工作点
/*
辨识D、Q轴电感时分别在-4、-2、0、2、4上辨识
速度环由于在0V附近转速不断在零速附近变化，非线性特性极强，因此只在-4、-2、2、4上辨识即可
*/
#define PRBS_Work_Point 4
//周期数
#define PRBS_n 1
//移位寄存器初始值(非0)
#define LFSR_INIT 0x0001

//方波信号参数
//方波起始值
#define Square_Start 0
//方波结束值
/*
若进行DQ轴开环验证，则是设定Ud和Uq的值，可以大一点，如3~5
若进行DQ轴闭环验证，则是设定电流给定值，需要小一些，如0.25~0.5，超出0.5容易发生控制器饱和引入非线性，不利于仿真分析
*/
#define Square_End -5
//方波开始序号
#define Square_Start_Index 1000
//方波长度
/*
若进行DQ轴开环验证、D轴的闭环验证或速度环开环验证，可以大一些，如50~100
若进行Q轴的闭环验证，需要保证长度足够短使得在此期间电机可以视为不转动，需要短一些，如5~10
*/
#define Square_Period 50

// 最高阶次5的线性控制器结构体
// 控制器传递函数为C(z)=( b0*z^5 + b1*z^4 + b2*z^3 + b3*z^2 + b4*z + b5) / (z^5 + a1*z^4 + a2*z^3 + a3*z^2 + a4*z + a5)
// 对应的差分方程为y(k) = -a1*y(n-1) - a2*y(n-2) - a3*y(n-3) - a4*y(n-4) - a5*y(n-5) +
//                       b0*u(n) + b1*u(n-1) + b2*u(n-2) + b3*u(n-3) + b4*u(n-4) + b5*u(n-5)
// 使用增量式实现，则等价控制器传递函数为(1-z^{-1})C(z) = ΔY(z)/U(z)
// 对应的增量式差分方程为
// Δy(k)=y(k)-y(k-1) = 
//     -a1*Δy(n-1) - a2*Δy(n-2) - a3*Δy(n-3) - a4*Δy(n-4) - a5*Δy(n-5) +
//     b0*u(n) + (b1-b0)*u(n-1) + (b2-b1)*u(n-2) + (b3-b2)*u(n-3) + (b4-b3)*u(n-4) + (b5-b4)*u(n-5) - b5*u(n-6)
typedef struct {
    float a1, a2, a3, a4, a5;
    float b0, b1, b2, b3, b4, b5;
    float Error_Record[6];
    float Output_Delta_Record[5];
    float Error_Now;
    float Output_Delta_Now;
    float Setvalue;
} Discrete_Controller_Struct;

//ADC注入组转换完成中断回调函数——获取电流值
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc);

//TIM1CH4比较中断回调函数——获取机械角度和机械角速度值，电流环控制入口
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);

//TIM2更新终端回调函数——速度环入口
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

//UART接收完成中断——更改指令值
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

#endif
