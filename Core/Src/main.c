/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "opamp.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MY_CALLBACK.h"
#include "MY_CONTROL.h"
#include "MY_JUSTFLOAT.h"
#include "MY_TLE5012B.h"
#include "stdbool.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// ADC采样原始数据
uint32_t ADC_Data[3] = {0, 0, 0};
// ADC采样转换后电流数据
float Current_abc[3] = {0, 0, 0};
// 机械角度
float theta_m = 0;
float theta_m_last = 0;
// 电角度
float theta_e = 0;
float Sin_theta_e = 0;
float Cos_theta_e = 0;
// 机械角速度
float n_m = 0;
// 速度环低通滤波
float n_m_last = 0;
// 零位偏置
float ADC1_ZERO = 0;
float ADC2_ZERO = 0;
float Angel_ZERO = 0;
// 两相静止坐标系电流
float I_alpha = 0;
float I_beta = 0;
// 同步旋转坐标系电流
float I_d = 0;
float I_q = 0;
// SVPWM得到的三相占空比
float Duty_A = 0;
float Duty_B = 0;
float Duty_C = 0;
// 母线电压
float Udc = U_DC_Default;
//SVPWM最大电压
float U_svpwm_max = U_DC_Default / SQRT3;
// DQ轴驱动电压
float U_d = Ud_Default;
float U_q = Uq_Default;
// DQ轴给定电流
float I_d_Target = ID_Target_Default;
float I_q_Target = IQ_Target_Default;
// 电流-速度环线性控制器
Discrete_Controller_Struct D_Controller, Q_Controller, Speed_Controller;
// 指令接收全局变量
uint8_t UART_Buffer[50];
float Order = 0;
uint16_t UART_Length = 0;
// 状态计数器
int counter = 0;
// 参数辨识标志位
int Identification_Mode = Identification_Mode_Default;
// /* 本原多项式反馈位(对应x^11 + x^9 + 1，位索引从0开始) */
const uint8_t taps[] = {9, 0}; // 第10位和第1位(索引9和0)
const uint8_t tap_cnt = sizeof(taps) / sizeof(taps[0]);
/* 定义存储数组（大小为周期长度×周期数） */
uint16_t m_seq[((1 << PRBS_N) - 1) * PRBS_n];
float Output_D[((1 << PRBS_N) - 1) * PRBS_n];
float Output_Q[((1 << PRBS_N) - 1) * PRBS_n];
float Output_theta_e[((1 << PRBS_N) - 1) * PRBS_n];

// 伪随机辨识结束标志位
bool HK_END = false;
//电流环运行标志位
bool Current_Control_Flag = false;

// 永磁体磁链
float Psi = 0;
float Psi_Win[10] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_OPAMP1_Init();
  MX_OPAMP2_Init();
  MX_OPAMP3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

    // M序列生成
    if (Identification_Mode == 3 || Identification_Mode == 5)
    {
        /* 定义存储数组（大小为周期长度×周期数） */
        uint32_t lfsr = LFSR_INIT;
        uint32_t period = (1 << PRBS_N) - 1;
        uint32_t seq_idx = 0; // 数组索引
        for (uint8_t cyc = 0; cyc < PRBS_n; cyc++)
        {
            for (uint32_t i = 0; i < period; i++)
            {
                /* 计算反馈位 */
                uint32_t fb = 0;
                for (uint8_t t = 0; t < tap_cnt; t++)
                {
                    fb ^= (lfsr >> taps[t]) & 0x01;
                }
                /* 存储序列位到数组（0或1） */
                m_seq[seq_idx++] = (lfsr & 0x01);
                /* 更新移位寄存器 */
                lfsr = (lfsr >> 1) | (fb << (PRBS_N - 1));
            }
        }
    }
    else if (Identification_Mode == 6 || Identification_Mode == 7 
      || Identification_Mode == 8 || Identification_Mode == 9)
    {
        uint32_t period = ((1 << PRBS_N) - 1) * PRBS_n;
        uint32_t seq_idx = 0; // 数组索引
        for (seq_idx = 0; seq_idx < period; seq_idx++)
        {
            if(seq_idx < Step_Start_Index)
            {
                m_seq[seq_idx] = 0;
            }
            else
            {
                m_seq[seq_idx] = 1;
            }
        }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, UART_Buffer, 100);
   // 线性控制器结构体初始化
    Controller_Init(D_a1, D_a2, D_a3, D_a4, D_a5, D_b0, D_b1, D_b2, D_b3, D_b4, D_b5,
                    ID_Target_Default, &D_Controller);
    Controller_Init(Q_a1, Q_a2, Q_a3, Q_a4, Q_a5, Q_b0, Q_b1, Q_b2, Q_b3, Q_b4, Q_b5,
                    IQ_Target_Default, &Q_Controller);
    Controller_Init(Speed_a1, Speed_a2, Speed_a3, Speed_a4, Speed_a5,
                    Speed_b0, Speed_b1, Speed_b2, Speed_b3, Speed_b4, Speed_b5, Speed_Target_Default, &Speed_Controller);
    // 使能模拟运算放大器
    HAL_OPAMP_Start(&hopamp1);
    HAL_OPAMP_Start(&hopamp2);
    HAL_OPAMP_Start(&hopamp3);

    // 开启两个ADC校验
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    for (int i = 0; i < 10; i++)
    {
        // ADC零位检测
        HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
        HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK)
        {
            ADC1_ZERO += HAL_ADC_GetValue(&hadc1) / 4096.0 * 3.3;
        }
        HAL_ADC_Start(&hadc2);
        if (HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY) == HAL_OK)
        {
            ADC2_ZERO += HAL_ADC_GetValue(&hadc2) / 4096.0 * 3.3;
        }
    }
    ADC1_ZERO = ADC1_ZERO / 10.0;
    ADC2_ZERO = ADC2_ZERO / 10.0;
    // 使能ADC注入组转换
    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    // 使能定时器
    HAL_TIM_Base_Start(&htim1);
    // 三相PWM输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    // 编码器零位校准
    // int k = 0;
    // Ud = 12;
    // Uq = 0;
    // // 基于I闭环控制模型抽象的编码器零位矫正
    // float wm_0_90[2] = {0};
    // for (int i = 0; i < 2; i++)
    // {
    //     // 读取起始机械角度
    //     theta_last = TLE5012B_Angle();
    //     // 给定Ud情况下转动
    //     for (int j = 0; j < 50; j++)
    //     {
    //         theta_e = TLE5012B_Angle();
    //         theta_e = theta_e * MOTOR_POLE_PAIRS + i * 90;
    //         DSP_Float_Calc_SinCos(theta_e, &Sin, &Cos);
    //         SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
    //         Set_CCR(Duty_A, Duty_B, Duty_C);
    //     }
    //     // 读取结束机械角度
    //     theta = TLE5012B_Angle();
    //     wm_0_90[i] = theta - theta_last;
    //     Ud = 12;
    // }
    // if (wm_0_90[0] * wm_0_90[1] < 0)
    // {
    //     // 零位位于第一、第三象限
    //     if (wm_0_90[1] < 0)
    //     {
    //         // 零位位于第三象限
    //         Angel_ZERO = 180;
    //     }
    //     else if (wm_0_90[1] > 0)
    //     {
    //         // 零位位于第一象限
    //         Angel_ZERO = 0;
    //     }
    // }
    // else if (wm_0_90[0] * wm_0_90[1] > 0)
    // {
    //     // 零位位于第二、第四象限
    //     if (wm_0_90[1] < 0)
    //     {
    //         // 零位位于第二象限
    //         Angel_ZERO = 90;
    //     }
    //     else if (wm_0_90[1] > 0)
    //     {
    //         // 零位位于第四象限
    //         Angel_ZERO = 270;
    //     }
    // }
    // while (k < 3)
    // {
    //     theta_last = TLE5012B_Angle();
    //     for (int i = 0; i < 100; i++)
    //     {
    //         theta_e = TLE5012B_Angle();
    //         theta_e = theta_e * MOTOR_POLE_PAIRS + Angel_ZERO;
    //         DSP_Float_Calc_SinCos(theta_e, &Sin, &Cos);
    //         SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
    //         Set_CCR(Duty_A, Duty_B, Duty_C);
    //     }
    //     theta = TLE5012B_Angle();
    //     wm = (1 - Speed_Filter) * (theta - theta_last) + Speed_Filter * wm;
    //     Ud = 12;
    //     if (wm < 0.000001 && wm > -0.000001)
    //     {
    //         k++;
    //     }
    //     else
    //     {
    //         k = 0;
    //         Angel_ZERO -= 5 * wm;
    //     }
    // }
    // theta = 0;
    // wm = 0;
    // theta_last = 0;
    // wm_last = 0;
    // theta_e = 0;
    // // Angel_ZERO = 90;
    // Ud = Ud_Default;
    // Uq = Uq_Default;

    int k = 0;
    U_d = U_svpwm_max;
    while(k<10)
    {
      //A相
      Set_CCR(0.9,0.1,0.1);
      HAL_Delay(500);
      theta_e = TLE5012B_Angle() * MOTOR_POLE_PAIRS;
      theta_e = fmod(theta_e, 360);
      theta_e<0?theta_e+=360:theta_e;
      Angel_ZERO += 360 - theta_e;
      //B相
      Set_CCR(0.1,0.9,0.1);
      HAL_Delay(500);
      theta_e = TLE5012B_Angle() * MOTOR_POLE_PAIRS - 120;
      theta_e = fmod(theta_e, 360);
      theta_e<0?theta_e+=360:theta_e;
      Angel_ZERO += 360 - theta_e;  
      //C相
      Set_CCR(0.1,0.1,0.9);
      HAL_Delay(500);
      theta_e = TLE5012B_Angle() * MOTOR_POLE_PAIRS + 120;
      theta_e = fmod(theta_e, 360);
      theta_e<0?theta_e+=360:theta_e;
      Angel_ZERO += 360 - theta_e;
      //计算零位
      Angel_ZERO = Angel_ZERO / 3;
      //验证
      theta_m_last = TLE5012B_Angle();
      for (int i = 0; i < 100; i++)
      {
        theta_e = TLE5012B_Angle();
        theta_e = theta_e * MOTOR_POLE_PAIRS + Angel_ZERO;
        DSP_Float_Calc_SinCos(theta_e, &Sin_theta_e, &Cos_theta_e);
        SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
        Set_CCR(Duty_A, Duty_B, Duty_C);
      }
      theta_m = TLE5012B_Angle();
      n_m = (1 - Speed_Filter) * (theta_m - theta_m_last) + Speed_Filter * n_m;
      U_d = 0;
      if (n_m < 0.5 && n_m > -0.5)
      {
        break; 
      }
      else {
        k++;
        Angel_ZERO = 0;
      }
    }

    // 电流环控制中断
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
    // 速度环控制中断
    HAL_TIM_Base_Start_IT(&htim2);

    // 初始化
    JUSTFLOAT_Init();
    // 绑定串口
    JUSTFLOAT_BindUart(&huart4);
    // 注册变量表
    JUSTFLOAT_AddData(&theta_m);
    JUSTFLOAT_AddData(&theta_e);
    JUSTFLOAT_AddData(&n_m);
    JUSTFLOAT_AddData(&Udc);
    JUSTFLOAT_AddData(&I_d);
    JUSTFLOAT_AddData(&I_q);
    // JUSTFLOAT_AddData(&Duty_A);
    // JUSTFLOAT_AddData(&Duty_B);
    // JUSTFLOAT_AddData(&Duty_C);
    JUSTFLOAT_AddData(&Angel_ZERO);
    JUSTFLOAT_AddData(&U_d);
    JUSTFLOAT_AddData(&U_q);
    JUSTFLOAT_AddData(&D_Controller.Setvalue);
    JUSTFLOAT_AddData(&Q_Controller.Setvalue);
    JUSTFLOAT_AddData(&Speed_Controller.Setvalue);
    JUSTFLOAT_AddData(&Current_abc[0]);
    JUSTFLOAT_AddData(&Current_abc[1]);
    JUSTFLOAT_AddData(&Current_abc[2]);
    JUSTFLOAT_AddData(&Psi);
    

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
