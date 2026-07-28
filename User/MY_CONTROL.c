#include "MY_CONTROL.h"
#include "MY_CALLBACK.h"
#include "MY_JUSTFLOAT.h"
#include "arm_math.h"
#include "stdbool.h"
#include "tim.h"

void Clarke_Trans(float A, float B, float C, float* alpha, float* beta)
{
    // *alpha = 2 / 3.0 * (A - 0.5 * B - 0.5 * C);
    // *beta = 1 / 3.0 * (SQRT3 * B - SQRT3 * C);
    arm_clarke_f32(A, B, alpha, beta);
}

void Inv_Clarke_Trans(float alpha, float beta, float* A, float* B, float* C)
{
    // *A = alpha;
    // *B = -0.5 * alpha + SQRT3 / 2.0 * beta;
    // *C = -0.5 * alpha - SQRT3 / 2.0 * beta;
    arm_inv_clarke_f32(alpha, beta, A, B);
    *C = 0 - *A - *B;
}

void Park_Trans(float alpha, float beta, float Sin, float Cos, float* D, float* Q)
{
    // DSP_Float_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // DSP_Fixed_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // CORDIC_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // *D = alpha * cos_theta + beta * sin_theta;
    // *Q = -alpha * sin_theta + beta * cos_theta;
    arm_park_f32(alpha, beta, D, Q, Sin, Cos);
}

void Inv_Park_Trans(float D, float Q, float Sin, float Cos, float* alpha, float* beta)
{
    // DSP_Fixed_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // CORDIC_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    //  *alpha = D * cos_theta - Q * sin_theta;
    //  *beta = D * sin_theta + Q * cos_theta;
    arm_inv_park_f32(D, Q, alpha, beta, Sin, Cos);
}

void DSP_Float_Calc_SinCos(float theta, float* Sin, float* Cos)
{
    arm_sin_cos_f32(theta, Sin, Cos);
}

void SVPWM_Calculation(float* Ud, float* Uq, float Sin, float Cos, float U_svpwm_max, float Udc,
                       float* Duty_A, float* Duty_B, float* Duty_C)
{
    float U_alpha, U_beta = 0;
    float U_ABC[3] = {0};
    float U_max, U_min = 0;
    float U_0 = 0;
    float u_mag = 0;
    if (U_svpwm_max == 0)
    {
        return;
    }
    arm_sqrt_f32((*Ud) * (*Ud) + (*Uq) * (*Uq), &u_mag);
    if (u_mag > U_svpwm_max)
    {
        u_mag = U_svpwm_max / u_mag;
    }
    else
    {
        u_mag = 1;
    }
    *Ud = (*Ud) * u_mag;
    *Uq = (*Uq) * u_mag;
    Inv_Park_Trans(*Ud, *Uq, Sin, Cos, &U_alpha, &U_beta);
    Inv_Clarke_Trans(U_alpha, U_beta, &U_ABC[0], &U_ABC[1], &U_ABC[2]);
    arm_max_f32(U_ABC, 3, &U_max, NULL);
    arm_min_f32(U_ABC, 3, &U_min, NULL);
    U_0 = -0.5 * (U_max + U_min);
    *Duty_A = 0.5 + (U_0 + U_ABC[0]) / Udc;
    *Duty_B = 0.5 + (U_0 + U_ABC[1]) / Udc;
    *Duty_C = 0.5 + (U_0 + U_ABC[2]) / Udc;
}

void Set_CCR(float Duty_A, float Duty_B, float Duty_C)
{
    int ARR = __HAL_TIM_GET_AUTORELOAD(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ARR * Duty_A);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ARR * Duty_B);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ARR * Duty_C);
}

void Discrete_Controller(Discrete_Controller_Struct* Controller)
{
    float Temp_Output_Delta = 0;
    Temp_Output_Delta -= Controller->a1 * Controller->Output_Delta_Record[0];
    Temp_Output_Delta -= Controller->a2 * Controller->Output_Delta_Record[1];
    Temp_Output_Delta -= Controller->a3 * Controller->Output_Delta_Record[2];
    Temp_Output_Delta -= Controller->a4 * Controller->Output_Delta_Record[3];
    Temp_Output_Delta -= Controller->a5 * Controller->Output_Delta_Record[4];

    Temp_Output_Delta += Controller->b0 * Controller->Error_Now;
    Temp_Output_Delta += (Controller->b1 - Controller->b0) * Controller->Error_Record[0];
    Temp_Output_Delta += (Controller->b2 - Controller->b1) * Controller->Error_Record[1];
    Temp_Output_Delta += (Controller->b3 - Controller->b2) * Controller->Error_Record[2];
    Temp_Output_Delta += (Controller->b4 - Controller->b3) * Controller->Error_Record[3];
    Temp_Output_Delta += (Controller->b5 - Controller->b4) * Controller->Error_Record[4];
    Temp_Output_Delta -= Controller->b5 * Controller->Error_Record[5];

    Controller->Output_Delta_Now = Temp_Output_Delta;

    Controller->Output_Delta_Record[4] = Controller->Output_Delta_Record[3];
    Controller->Output_Delta_Record[3] = Controller->Output_Delta_Record[2];
    Controller->Output_Delta_Record[2] = Controller->Output_Delta_Record[1];
    Controller->Output_Delta_Record[1] = Controller->Output_Delta_Record[0];
    Controller->Output_Delta_Record[0] = Temp_Output_Delta;

    Controller->Error_Record[5] = Controller->Error_Record[4];
    Controller->Error_Record[4] = Controller->Error_Record[3];
    Controller->Error_Record[3] = Controller->Error_Record[2];
    Controller->Error_Record[2] = Controller->Error_Record[1];
    Controller->Error_Record[1] = Controller->Error_Record[0];
    Controller->Error_Record[0] = Controller->Error_Now;
}

void Current_Control()
{
    // 机械角度
    extern float theta_m;
    extern float Angel_ZERO;
    // 电角度
    extern float theta_e;
    // 三相自然坐标系
    extern float Current_abc[3];
    // 两相静止坐标
    extern float I_alpha;
    extern float I_beta;
    // 同步旋转坐标系
    extern float I_d;
    extern float I_q;
    // 三相占空比
    extern float Duty_A;
    extern float Duty_B;
    extern float Duty_C;
    // DQ轴电压
    extern float U_d;
    extern float U_q;
    // 母线电压
    extern float Udc;
    // SVPWM最大电压
    extern float U_svpwm_max;
    // DQ轴电流控制器（按模式所需）
#if Identification_Mode_Default == 8  || Identification_Mode_Default == 10 || \
    Identification_Mode_Default == 11 || Identification_Mode_Default == 12 || \
    Identification_Mode_Default == 13
    extern Discrete_Controller_Struct D_Controller;
#endif
#if Identification_Mode_Default == 9  || Identification_Mode_Default == 12 || \
    Identification_Mode_Default == 13
    extern Discrete_Controller_Struct Q_Controller;
#endif
    // 三角函数
    extern float Sin_theta_e;
    extern float Cos_theta_e;
    // 电流环运行标志位
    extern bool Current_Control_Flag;

    // ── 模式相关变量 ──
    // 状态计数器（模式 3,5,6,7,8,9 在电流环中使用）
#if Identification_Mode_Default == 3  || Identification_Mode_Default == 5  || \
    Identification_Mode_Default == 6  || Identification_Mode_Default == 7  || \
    Identification_Mode_Default == 8  || Identification_Mode_Default == 9
    extern int counter;
#endif
    // 伪随机辨识结束标志位（模式 3,5,6,7,8,9 使用）
#if Identification_Mode_Default == 3  || Identification_Mode_Default == 5  || \
    Identification_Mode_Default == 6  || Identification_Mode_Default == 7  || \
    Identification_Mode_Default == 8  || Identification_Mode_Default == 9
    extern bool HK_END;
#endif
    // M序列和输出序列（仅模式 3,5,6,7,8,9 在电流环中使用）
#if Identification_Mode_Default == 3 || Identification_Mode_Default == 5 || \
    Identification_Mode_Default == 6 || Identification_Mode_Default == 7 || \
    Identification_Mode_Default == 8 || Identification_Mode_Default == 9
    extern uint16_t m_seq[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_D[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_Q[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_theta_e[((1 << PRBS_N) - 1) * PRBS_n];
#endif
    // 电角速度 and 前馈变量（模式 10~13 使用）
#if Identification_Mode_Default >= 10
    extern float we;
    extern float E_D_ff;
    extern float E_D_ff_last;
#endif
#if Identification_Mode_Default == 12 || Identification_Mode_Default == 13
    extern float E_Q_ff;
    extern float E_Q_ff_last;
#endif

    // 置位标志位
    Current_Control_Flag = true;
    // 计算电角度
    theta_e = theta_m * MOTOR_POLE_PAIRS + Angel_ZERO;
    theta_e = fmod(theta_e, 360);
    theta_e < 0 ? theta_e += 360 : theta_e;
    // 计算三角函数
    DSP_Float_Calc_SinCos(theta_e, &Sin_theta_e, &Cos_theta_e);
    // 计算电流
    Clarke_Trans(Current_abc[0], Current_abc[1], Current_abc[2], &I_alpha, &I_beta);
    Park_Trans(I_alpha, I_beta, Sin_theta_e, Cos_theta_e, &I_d, &I_q);

    // ── 模式分支（编译期选择）──
#if Identification_Mode_Default == 3
    // 3——伪随机辨识Ld同步电感
    {
        if (HK_END == false)
        {
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n) + 10000)
            {
                U_d = PRBS_Work_Point;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            }
            else if (counter < 10000)
            {
                counter++;
                U_d = PRBS_Work_Point;
                SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B,
                                  &Duty_C);
                Set_CCR(Duty_A, Duty_B, Duty_C);
            }
            else if (counter >= 10000)
            {
                if (m_seq[counter - 10000] == 0)
                {
                    U_d = PRBS_Work_Point + PRBS_A;
                }
                else if (m_seq[counter - 10000] == 1)
                {
                    U_d = PRBS_Work_Point - PRBS_A;
                }
                Output_D[counter - 10000] = I_d;
                Output_Q[counter - 10000] = I_q;
                Output_theta_e[counter - 10000] = theta_e;
                counter++;
                SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
                Set_CCR(Duty_A, Duty_B, Duty_C);
            }
        }
    }

#elif Identification_Mode_Default == 5
    // 5——伪随机辨识Lq同步电感
    {
        if (HK_END == false)
        {
            if (counter == (((1 << PRBS_N) - 1) * PRBS_n) + 10000)
            {
                U_q = PRBS_Work_Point;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            }
            else if (counter < 10000)
            {
                counter++;
                U_q = PRBS_Work_Point;
                SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B,
                                  &Duty_C);
                Set_CCR(Duty_A, Duty_B, Duty_C);
            }
            else if (counter >= 10000)
            {
                if (m_seq[counter - 10000] == 0)
                {
                    U_q = PRBS_Work_Point + PRBS_A;
                }
                else
                {
                    U_q = PRBS_Work_Point - PRBS_A;
                }
                Output_D[counter - 10000] = I_d;
                Output_Q[counter - 10000] = I_q;
                Output_theta_e[counter - 10000] = theta_e;
                counter++;
                SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
                Set_CCR(Duty_A, Duty_B, Duty_C);
            }
        }
    }

#elif Identification_Mode_Default == 6
    // 6——D轴开环方波响应模型验证
    {
        if (HK_END == false)
        {
            if (counter == (((1 << PRBS_N) - 1) * PRBS_n))
            {
                U_d = (Square_Start + Square_End) / 2.0;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    U_d = Square_Start;
                }
                else
                {
                    U_d = Square_End;
                }
                Output_D[counter] = I_d;
                Output_Q[counter] = I_q;
                Output_theta_e[counter] = theta_e;
                counter++;
                SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
                Set_CCR(Duty_A, Duty_B, Duty_C);
            }
        }
    }

#elif Identification_Mode_Default == 7
    // 7——Q轴开环方波响应模型验证
    {
        if (HK_END == false)
        {
            if (counter == (((1 << PRBS_N) - 1) * PRBS_n))
            {
                U_q = (Square_Start + Square_End) / 2.0;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    U_q = Square_Start;
                }
                else
                {
                    U_q = Square_End;
                }
                Output_D[counter] = I_d;
                Output_Q[counter] = I_q;
                Output_theta_e[counter] = theta_e;
                counter++;
                SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
                Set_CCR(Duty_A, Duty_B, Duty_C);
            }
        }
    }

#elif Identification_Mode_Default == 8
    // 8——D轴闭环方波响应模型验证
    {
        if (HK_END == false)
        {
            if (counter == ((1 << PRBS_N) - 1) * PRBS_n)
            {
                D_Controller.Setvalue = (Square_Start + Square_End) / 2.0;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    D_Controller.Setvalue = Square_Start;
                }
                else
                {
                    D_Controller.Setvalue = Square_End;
                }
                D_Controller.Error_Now = D_Controller.Setvalue - I_d;
                Discrete_Controller(&D_Controller);
                U_d += D_Controller.Output_Delta_Now;
                Output_D[counter] = I_d;
                Output_Q[counter] = I_q;
                Output_theta_e[counter] = theta_e;
                counter++;
                SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
                Set_CCR(Duty_A, Duty_B, Duty_C);
            }
        }
    }

#elif Identification_Mode_Default == 9
    // 9——Q轴闭环方波响应模型验证
    {
        if (HK_END == false)
        {
            if (counter == ((1 << PRBS_N) - 1) * PRBS_n)
            {
                Q_Controller.Setvalue = (Square_Start + Square_End) / 2.0;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    Q_Controller.Setvalue = Square_Start;
                }
                else
                {
                    Q_Controller.Setvalue = Square_End;
                }
                Q_Controller.Error_Now = Q_Controller.Setvalue - I_q;
                Discrete_Controller(&Q_Controller);
                U_q += Q_Controller.Output_Delta_Now;
                Output_D[counter] = I_d;
                Output_Q[counter] = I_q;
                Output_theta_e[counter] = theta_e;
                counter++;
                SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
                Set_CCR(Duty_A, Duty_B, Duty_C);
            }
        }
    }

#elif Identification_Mode_Default == 10 || Identification_Mode_Default == 11
    // 10——伪随机辨识速度环被控对象
    // 11——速度环开环方波响应模型验证
    {
        D_Controller.Error_Now = 0 - I_d;
        Discrete_Controller(&D_Controller);
        U_d += D_Controller.Output_Delta_Now;
        // 电流环前馈解耦（增量式）
        E_D_ff = -we * MOTOR_Lq * I_q;
        U_d += E_D_ff - E_D_ff_last;
        E_D_ff_last = E_D_ff;
        SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
        Set_CCR(Duty_A, Duty_B, Duty_C);
    }

#elif Identification_Mode_Default == 12 || Identification_Mode_Default == 13
    // 12——速度环闭环方波响应模型验证
    // 13——双闭环运行
    {
        Q_Controller.Error_Now = Q_Controller.Setvalue - I_q;
        Discrete_Controller(&Q_Controller);
        U_q += Q_Controller.Output_Delta_Now;
        D_Controller.Error_Now = D_Controller.Setvalue - I_d;
        Discrete_Controller(&D_Controller);
        U_d += D_Controller.Output_Delta_Now;
        // 反电动势前馈解耦（增量式）
        E_D_ff = -we * MOTOR_Lq * I_q;
        E_Q_ff =  we * (MOTOR_Ld * I_d + MOTOR_Psi);
        U_d += E_D_ff - E_D_ff_last;
        U_q += E_Q_ff - E_Q_ff_last;
        E_D_ff_last = E_D_ff;
        E_Q_ff_last = E_Q_ff;
        SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
        Set_CCR(Duty_A, Duty_B, Duty_C);
    }

#else
    // 模式 1,2,4 — 仅 SVPWM 输出，电压由 Speed_Control() 设定
    {
        SVPWM_Calculation(&U_d, &U_q, Sin_theta_e, Cos_theta_e, U_svpwm_max, Udc, &Duty_A, &Duty_B, &Duty_C);
        Set_CCR(Duty_A, Duty_B, Duty_C);
    }
#endif

    // 重置标志位
    Current_Control_Flag = false;
    HAL_GPIO_WritePin(Test_GPIO_Port, Test_Pin, GPIO_PIN_RESET);
}

void Speed_Control()
{
    // DQ轴电流控制器
    extern Discrete_Controller_Struct D_Controller;
    extern Discrete_Controller_Struct Q_Controller;
    // 速度环控制器
    extern Discrete_Controller_Struct Speed_Controller;
    // 当前角速度
    extern float n_m;
    // DQ轴驱动电压
    extern float U_d;
    extern float U_q;
    // DQ轴电流
    extern float I_d;
    extern float I_q;
    // 电角度
    extern float theta_e;

    // ── 模式相关变量 ──
    // 状态计数器（模式 1~12 使用）
#if Identification_Mode_Default != 13
    extern int counter;
#endif
    // 伪随机辨识结束标志位（模式 3,5,6,7,8,9 使用）
#if Identification_Mode_Default == 3  || Identification_Mode_Default == 5  || \
    Identification_Mode_Default == 6  || Identification_Mode_Default == 7  || \
    Identification_Mode_Default == 8  || Identification_Mode_Default == 9
    extern bool HK_END;
#endif
    // M序列和输出序列（模式 3,5,6,7,8,9 使用）
#if Identification_Mode_Default == 3  || Identification_Mode_Default == 5  || \
    Identification_Mode_Default == 6  || Identification_Mode_Default == 7  || \
    Identification_Mode_Default == 8  || Identification_Mode_Default == 9  || \
    Identification_Mode_Default == 10 || Identification_Mode_Default == 11 || \
    Identification_Mode_Default == 12
    extern uint16_t m_seq[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_D[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_Q[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_theta_e[((1 << PRBS_N) - 1) * PRBS_n];
#endif
    // 永磁体磁链（模式 4 使用）
#if Identification_Mode_Default == 4
    extern float Psi;
    extern float Psi_Win[10];
#endif

    // ── 模式分支（编译期选择）──
#if Identification_Mode_Default == 1
    // 1——相电阻，D轴方向
    {
        if (counter == 0)
        {
            U_d = -6;
        }
        if (counter == 301)
        {
            U_d += 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
    }

#elif Identification_Mode_Default == 2
    // 2——相电阻，Q轴方向
    {
        if (counter == 0)
        {
            U_q = -6;
        }
        if (counter == 301)
        {
            U_q += 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
    }

#elif Identification_Mode_Default == 3
    // 3——伪随机辨识Ld同步电感（回放阶段）
    {
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                U_d = PRBS_Work_Point;
                I_d = 0;
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    U_d = PRBS_Work_Point + PRBS_A;
                }
                else
                {
                    U_d = PRBS_Work_Point - PRBS_A;
                }
                I_d = Output_D[counter];
                I_q = Output_Q[counter];
                theta_e = Output_theta_e[counter];
                counter++;
            }
        }
    }

#elif Identification_Mode_Default == 4
    // 4——永磁体磁链辨识
    {
        if (counter == 0)
        {
            U_q = -6;
        }
        if (counter == 301)
        {
            U_q += 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
        float we = n_m * MOTOR_POLE_PAIRS * 2 * PI / 60.0;
        if (we != 0)
        {
            for (int i = 1; i < 10; i++)
            {
                Psi_Win[i] = Psi_Win[i - 1];
            }
            Psi_Win[0] = (U_q - I_q * MOTOR_R - MOTOR_Ld * we * I_d) / we;
            Psi = 0;
            for (int i = 0; i < 10; i++)
            {
                Psi += Psi_Win[i];
            }
            Psi = Psi / 10.0;
        }
    }

#elif Identification_Mode_Default == 5
    // 5——伪随机辨识Lq同步电感（回放阶段）
    {
        if (HK_END == true)
        {
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                U_q = PRBS_Work_Point;
                I_q = 0;
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    U_q = PRBS_Work_Point + PRBS_A;
                }
                else if (m_seq[counter] == 1)
                {
                    U_q = PRBS_Work_Point - PRBS_A;
                }
                I_d = Output_D[counter];
                I_q = Output_Q[counter];
                theta_e = Output_theta_e[counter];
                counter++;
            }
        }
    }

#elif Identification_Mode_Default == 6
    // 6——D轴开环方波响应模型验证（回放阶段）
    {
        if (HK_END == true)
        {
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                U_d = (Square_Start + Square_End) / 2.0;
                I_d = 0;
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    U_d = Square_Start;
                }
                else
                {
                    U_d = Square_End;
                }
                I_d = Output_D[counter];
                I_q = Output_Q[counter];
                theta_e = Output_theta_e[counter];
                counter++;
            }
        }
    }

#elif Identification_Mode_Default == 7
    // 7——Q轴开环方波响应模型验证（回放阶段）
    {
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                U_q = (Square_Start + Square_End) / 2.0;
                I_q = 0;
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    U_q = Square_Start;
                }
                else
                {
                    U_q = Square_End;
                }
                I_d = Output_D[counter];
                I_q = Output_Q[counter];
                theta_e = Output_theta_e[counter];
                counter++;
            }
        }
    }

#elif Identification_Mode_Default == 8
    // 8——D轴闭环方波响应模型验证（回放阶段）
    {
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                D_Controller.Setvalue = (Square_Start + Square_End) / 2.0;
                I_d = 0;
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    D_Controller.Setvalue = Square_Start;
                }
                else
                {
                    D_Controller.Setvalue = Square_End;
                }
                I_d = Output_D[counter];
                I_q = Output_Q[counter];
                theta_e = Output_theta_e[counter];
                counter++;
            }
        }
    }

#elif Identification_Mode_Default == 9
    // 9——Q轴闭环方波响应模型验证（回放阶段）
    {
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                Q_Controller.Setvalue = (Square_Start + Square_End) / 2.0;
                I_q = 0;
            }
            else
            {
                if (m_seq[counter] == 0)
                {
                    Q_Controller.Setvalue = Square_Start;
                }
                else
                {
                    Q_Controller.Setvalue = Square_End;
                }
                I_d = Output_D[counter];
                I_q = Output_Q[counter];
                theta_e = Output_theta_e[counter];
                counter++;
            }
        }
    }

#elif Identification_Mode_Default == 10
    // 10——伪随机辨识速度环被控对象
    {
        if (counter >= (((1 << PRBS_N) - 1) * PRBS_n) + 1000)
        {
            U_q = PRBS_Work_Point;
        }
        else if (counter < 1000)
        {
            U_q = PRBS_Work_Point;
        }
        else if (counter >= 1000)
        {
            if (m_seq[counter - 1000] == 0)
            {
                U_q = PRBS_Work_Point + PRBS_A;
            }
            else if (m_seq[counter - 1000] == 1)
            {
                U_q = PRBS_Work_Point - PRBS_A;
            }
        }
        counter++;
    }

#elif Identification_Mode_Default == 11
    // 11——速度环开环方波响应模型验证
    {
        if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
        {
            U_q = (Square_Start + Square_End) / 2.0;
        }
        else
        {
            if (m_seq[counter] == 0)
            {
                U_q = Square_Start;
            }
            else if (m_seq[counter] == 1)
            {
                U_q = Square_End;
            }
            counter++;
        }
    }

#elif Identification_Mode_Default == 12
    // 12——速度环闭环方波响应模型验证
    {
        if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
        {
            Speed_Controller.Setvalue = (Square_Start + Square_End) / 2.0;
        }
        else
        {
            if (m_seq[counter] == 0)
            {
                Speed_Controller.Setvalue = Square_Start;
            }
            else if (m_seq[counter] == 1)
            {
                Speed_Controller.Setvalue = Square_End;
            }
            counter++;
        }
        Speed_Controller.Error_Now = Speed_Controller.Setvalue - n_m;
        Discrete_Controller(&Speed_Controller);
        Q_Controller.Setvalue += Speed_Controller.Output_Delta_Now;
    }

#elif Identification_Mode_Default == 13
    // 13——双闭环运行
    {
        // 速度环给定限幅
        if (Speed_Controller.Setvalue < -Speed_Target_Limit)
        {
            Speed_Controller.Setvalue = -Speed_Target_Limit;
        }
        else if (Speed_Controller.Setvalue > Speed_Target_Limit)
        {
            Speed_Controller.Setvalue = Speed_Target_Limit;
        }
        Speed_Controller.Error_Now = Speed_Controller.Setvalue - n_m;
        Discrete_Controller(&Speed_Controller);
        Q_Controller.Setvalue += Speed_Controller.Output_Delta_Now;
        if (Q_Controller.Setvalue > Speed_Output_Limit)
        {
            Q_Controller.Setvalue = Speed_Output_Limit;
        }
        else if (Q_Controller.Setvalue < -Speed_Output_Limit)
        {
            Q_Controller.Setvalue = -Speed_Output_Limit;
        }
    }
#endif
}

void Controller_Init(float a1, float a2, float a3, float a4, float a5, float b0,
                     float b1, float b2, float b3, float b4, float b5, float Default_Set,
                     Discrete_Controller_Struct* Controller)
{
    Controller->a1 = a1;
    Controller->a2 = a2;
    Controller->a3 = a3;
    Controller->a4 = a4;
    Controller->a5 = a5;
    Controller->b0 = b0;
    Controller->b1 = b1;
    Controller->b2 = b2;
    Controller->b3 = b3;
    Controller->b4 = b4;
    Controller->b5 = b5;
    Controller->Setvalue = Default_Set;
}
