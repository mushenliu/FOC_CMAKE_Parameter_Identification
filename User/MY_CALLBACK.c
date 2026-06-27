#include "MY_CALLBACK.h"
#include "MY_JUSTFLOAT.h"
#include "MY_CONTROL.h"
#include "MY_TLE5012B.h"
#include "stdlib.h"
#include "adc.h"
#include "usart.h"

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    //ADC原始采样值
    extern uint32_t ADC_Data[3];
    //ADC转换后三相电流值
    extern float Current_abc[3];
    //ADC零位偏置
    extern float ADC1_ZERO;
    extern float ADC2_ZERO;
    //由于ADC1需要采集两个数据（其中有一个电压数据），ADC2只需要采集一个数据，因此ADC1总会比ADC2更晚进入中断
    if (hadc == &hadc1) {
        HAL_GPIO_WritePin(Test_GPIO_Port,Test_Pin, GPIO_PIN_SET);
        // ADC_Data[0] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
        // ADC_Data[1] = HAL_ADCEx_InjectedGetValue(&hadc2, ADC_INJECTED_RANK_1);
        // ADC_Data[2] = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
        ADC_Data[0] = hadc1.Instance->JDR1;
        ADC_Data[1] = hadc2.Instance->JDR1;
        ADC_Data[2] = hadc1.Instance->JDR2;
        //野值剔除
        Current_abc[0] = -((ADC_Data[0] / 4096.0) * 3.3 - ADC1_ZERO) * 2;
        Current_abc[1] = -((ADC_Data[1] / 4096.0) * 3.3 - ADC2_ZERO) * 2;
        Current_abc[2] = -Current_abc[0] - Current_abc[1];
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
    if ((htim->Instance == TIM1) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)) {
        //电流环运行标志位
        extern bool Current_Control_Flag;
        extern float theta_m;
        theta_m = TLE5012B_Angle();
        //如果电流环未运行，才进行电流环
        if (Current_Control_Flag == false)
        {
            Current_Control();
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    extern float theta_m;
    extern float theta_m_last;
    extern float n_m;
    extern float n_m_last;
    extern float Udc;
    extern float U_svpwm_max;
    extern uint32_t ADC_Data[3];
    if (theta_m - theta_m_last < 10 && theta_m - theta_m_last > -10) {
        n_m = (theta_m - theta_m_last) / (Ts_Speed * 6);
    }
    n_m = (1-Speed_Filter) * n_m_last + Speed_Filter * n_m;
    n_m_last = n_m;
    theta_m_last = theta_m;
    Udc = ADC_Data[2] *0.00686;
    U_svpwm_max = Udc / SQRT3;
    Speed_Control();
    JUSTFLOAT_SendData();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    extern uint8_t UART_Buffer[100];
    extern float Order;
    extern uint16_t UART_Length;
    extern Discrete_Controller_Struct D_Controller, Q_Controller, Speed_Controller;
    UART_Length = Size;
    if (UART_Length > 0 && UART_Length < 100)
    {
        UART_Buffer[UART_Length] = '\0';
        Order = atof((const char *)UART_Buffer);
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART_Buffer, 100);
    // 以下三个选择其一进行设置
    //  D_Controller.Setvalue = Order;
    //  Q_Controller.Setvalue = Order;
    Speed_Controller.Setvalue = Order;
}