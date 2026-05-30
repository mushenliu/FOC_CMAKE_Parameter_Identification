#include "MY_CONTROL.h"
#include "MY_CALLBACK.h"
#include "MY_JUSTFLOAT.h"
#include "arm_math.h"
#include "stdbool.h"
#include "tim.h"

void Clarke_Trans(float A, float B, float C, float *alpha, float *beta)
{
    // *alpha = 2 / 3.0 * (A - 0.5 * B - 0.5 * C);
    // *beta = 1 / 3.0 * (SQRT3 * B - SQRT3 * C);
    arm_clarke_f32(A, B, alpha, beta);
}

void Inv_Clarke_Trans(float alpha, float beta, float *A, float *B, float *C)
{
    // *A = alpha;
    // *B = -0.5 * alpha + SQRT3 / 2.0 * beta;
    // *C = -0.5 * alpha - SQRT3 / 2.0 * beta;
    arm_inv_clarke_f32(alpha, beta, A, B);
    *C = 0 - *A - *B;
}

void Park_Trans(float alpha, float beta, float Sin, float Cos, float *D, float *Q)
{
    // DSP_Float_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // DSP_Fixed_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // CORDIC_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // *D = alpha * cos_theta + beta * sin_theta;
    // *Q = -alpha * sin_theta + beta * cos_theta;
    arm_park_f32(alpha, beta, D, Q, Sin, Cos);
}

void Inv_Park_Trans(float D, float Q, float Sin, float Cos, float *alpha, float *beta)
{
    // DSP_Fixed_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    // CORDIC_Calc_SinCos(theta_e, &sin_theta, &cos_theta);
    //  *alpha = D * cos_theta - Q * sin_theta;
    //  *beta = D * sin_theta + Q * cos_theta;
    arm_inv_park_f32(D, Q, alpha, beta, Sin, Cos);
}

void DSP_Float_Calc_SinCos(float theta, float *Sin, float *Cos)
{
    arm_sin_cos_f32(theta, Sin, Cos);
}

void SVPWM_Calculation(float *Ud, float *Uq, float Sin, float Cos, float Udc, float *Duty_A, float *Duty_B,
                       float *Duty_C)
{
    float U_alpha, U_beta = 0;
    float U_ABC[3] = {0};
    float U_max, U_min = 0;
    float U_0 = 0;
    uint32_t p = 0;
    float u_mag = 0;
    if (Udc == 0)
    {
        return;
    }
    arm_sqrt_f32((*Ud) * (*Ud) + (*Uq) * (*Uq), &u_mag);
    if (u_mag > Udc / SQRT3)
    {
        u_mag = Udc / SQRT3 / u_mag;
    }
    else
    {
        u_mag = 1;
    }
    *Ud = (*Ud) * u_mag;
    *Uq = (*Uq) * u_mag;
    Inv_Park_Trans(*Ud, *Uq, Sin, Cos, &U_alpha, &U_beta);
    Inv_Clarke_Trans(U_alpha, U_beta, &U_ABC[0], &U_ABC[1], &U_ABC[2]);
    arm_max_f32(U_ABC, 3, &U_max, &p);
    arm_min_f32(U_ABC, 3, &U_min, &p);
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

void Discrete_Controller(Discrete_Controller_Struct *Controller)
{
    float Temp_Output = 0;

    Temp_Output -= Controller->a1 * Controller->Output_Record[0];
    Temp_Output -= Controller->a2 * Controller->Output_Record[1];
    Temp_Output -= Controller->a3 * Controller->Output_Record[2];
    Temp_Output -= Controller->a4 * Controller->Output_Record[3];
    Temp_Output -= Controller->a5 * Controller->Output_Record[4];

    Temp_Output += Controller->b0 * Controller->Error_Now;
    Temp_Output += Controller->b1 * Controller->Error_Record[0];
    Temp_Output += Controller->b2 * Controller->Error_Record[1];
    Temp_Output += Controller->b3 * Controller->Error_Record[2];
    Temp_Output += Controller->b4 * Controller->Error_Record[3];
    Temp_Output += Controller->b5 * Controller->Error_Record[4];

    Controller->Output_Now = Temp_Output;

    Controller->Output_Record[4] = Controller->Output_Record[3];
    Controller->Output_Record[3] = Controller->Output_Record[2];
    Controller->Output_Record[2] = Controller->Output_Record[1];
    Controller->Output_Record[1] = Controller->Output_Record[0];
    Controller->Output_Record[0] = Controller->Output_Now;

    Controller->Error_Record[4] = Controller->Error_Record[3];
    Controller->Error_Record[3] = Controller->Error_Record[2];
    Controller->Error_Record[2] = Controller->Error_Record[1];
    Controller->Error_Record[1] = Controller->Error_Record[0];
    Controller->Error_Record[0] = Controller->Error_Now;
}

void Current_Control()
{
    // 机械角度
    extern float theta;
    extern float Angel_ZERO;
    extern float theta_last;
    // 电角度
    extern float theta_e;
    //机械角速度
    extern float wm;
    // 三相自然坐标系
    extern float Current_abc[3];
    // 两相静止坐标
    extern float alpha;
    extern float beta;
    // 同步旋转坐标系
    extern float D;
    extern float Q;
    // 三相占空比
    extern float Duty_A;
    extern float Duty_B;
    extern float Duty_C;
    // DQ轴电压
    extern float Ud;
    extern float Uq;
    // 母线电压
    extern float Udc;
    // DQ轴电流控制器
    extern Discrete_Controller_Struct D_Controller;
    extern Discrete_Controller_Struct Q_Controller;
    // 三角函数
    extern float Sin;
    extern float Cos;
    // 参数辨识标志位
    extern int Identification_Mode;
    // 伪随机辨识结束标志位
    extern bool HK_END;
    // M序列和输出序列
    extern uint16_t m_seq[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_D[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_Q[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_theta_e[((1 << PRBS_N) - 1) * PRBS_n];
    // 状态计数器
    extern int counter;

    // 计算电角度
    theta_e = theta * MOTOR_POLE_PAIRS + Angel_ZERO;
    while (theta_e > 360)
    {
        theta_e -= 360;
    }
    while (theta_e < 0)
    {
        theta_e += 360;
    }
    // 计算三角函数
    DSP_Float_Calc_SinCos(theta_e, &Sin, &Cos);
    // 计算电流
    Clarke_Trans(Current_abc[0], Current_abc[1], Current_abc[2], &alpha, &beta);
    Park_Trans(alpha, beta, Sin, Cos, &D, &Q);
    switch (Identification_Mode)
    {
    // 5——伪随机辨识Ld同步电感
    case 5: {
        if (HK_END == false)
        {
            if (counter == (((1 << PRBS_N) - 1) * PRBS_n) + 10000)
            {
                Ud = PRBS_Work_Point;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
                break;
            }
            if (counter < 10000)
            {
                counter++;
                Ud = PRBS_Work_Point;
                SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
                // 设定CCR值
                Set_CCR(Duty_A, Duty_B, Duty_C);
                break;
            }
            else if (m_seq[counter - 10000] == 0)
            {
                Ud = PRBS_Work_Point + PRBS_A;
            }
            else
            {
                Ud = PRBS_Work_Point - PRBS_A;
            }
            Output_D[counter - 10000] = D;
            Output_Q[counter - 10000] = Q;
            Output_theta_e[counter - 10000] = theta_e;
            counter++;
            SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
            // 设定CCR值
            Set_CCR(Duty_A, Duty_B, Duty_C);
            break;
        }
        break;
    }
    // 6,7——永磁体磁链辨识，正向和负向
    case 6: {
        /*D_PID.Error_Now = 0 - D;
        Discrete_PID_Controller(&D_PID);
        Ud = D_PID.Output_Now;*/
        SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
        // 设定CCR值
        Set_CCR(Duty_A, Duty_B, Duty_C);
        break;
    }
    case 7: {
        /*D_PID.Error_Now = 0 - D;
         Discrete_PID_Controller(&D_PID);
         Ud = D_PID.Output_Now;*/
        SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
        // 设定CCR值
        Set_CCR(Duty_A, Duty_B, Duty_C);
        break;
    }
    // 8——伪随机辨识Lq同步电感
    case 8: {
        if (HK_END == false)
        {
            if (counter == (((1 << PRBS_N) - 1) * PRBS_n) + 10000)
            {
                Uq = PRBS_Work_Point;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
                break;
            }
            if (counter < 10000)
            {
                counter++;
                Uq = PRBS_Work_Point;
                SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
                // 设定CCR值
                Set_CCR(Duty_A, Duty_B, Duty_C);
                break;
            }
            else if (m_seq[counter - 10000] == 0)
            {
                Uq = PRBS_Work_Point + PRBS_A;
            }
            else
            {
                Uq = PRBS_Work_Point - PRBS_A;
            }
            D_Controller.Error_Now = 0 - D;
            Discrete_Controller(&D_Controller);
            Ud = D_Controller.Output_Now;
            Output_D[counter - 10000] = D;
            Output_Q[counter - 10000] = Q;
            Output_theta_e[counter - 10000] = theta_e;
            counter++;
            SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
            // 设定CCR值
            Set_CCR(Duty_A, Duty_B, Duty_C);
            break;
        }
        break;
    }
    // 9——D轴单位阶跃模型验证
    case 9: {
        if (HK_END == false)
        {
            if (counter == (((1 << PRBS_N) - 1) * PRBS_n))
            {
                Ud = PRBS_Work_Point;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
                break;
            }
            else if (m_seq[counter] == 0)
            {
                Ud = Step_Start;
            }
            else
            {
                Ud = Step_End;
            }
            Output_D[counter] = D;
            Output_Q[counter] = Q;
            Output_theta_e[counter] = theta_e;
            counter++;
            SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
            // 设定CCR值
            Set_CCR(Duty_A, Duty_B, Duty_C);
            break;
        }
        break;
    }
    // 10——Q轴单位阶跃模型验证
    case 10: {
        if (HK_END == false)
        {
            if (counter == (((1 << PRBS_N) - 1) * PRBS_n))
            {
                Uq = 0;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
                break;
            }
            else if (m_seq[counter] == 0)
            {
                Uq = Step_Start;
            }
            else
            {
                Uq = Step_End;
            }
            Output_D[counter] = D;
            Output_Q[counter] = Q;
            Output_theta_e[counter] = theta_e;
            counter++;
            SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
            // 设定CCR值
            Set_CCR(Duty_A, Duty_B, Duty_C);
            break;
        }
        break;
    }
    // 11——D轴闭环阶跃模型验证
    case 11:{
        if (HK_END == false)
        {
            if (counter == ((1 << PRBS_N) - 1) * PRBS_n)
            {
                D_Controller.Setvalue = Step_Start;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
                break;
            }
            else if (m_seq[counter] == 0)
            {
                D_Controller.Setvalue = Step_Start;
            }
            else
            {
                D_Controller.Setvalue = Step_End;
            }
            D_Controller.Error_Now = D_Controller.Setvalue - D;
            Discrete_Controller(&D_Controller);
            Ud = D_Controller.Output_Now;
            // // 电流环前馈解耦
            // float we = wm * 0.10472 * MOTOR_POLE_PAIRS;
            // Ud -= we * MOTOR_Ld * Q;
            // Uq += we * (MOTOR_Lq * D + MOTOR_Psi);
            Output_D[counter] = D;
            Output_Q[counter] = Q;
            Output_theta_e[counter] = theta_e;
            counter++;
            SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
            // 设定CCR值
            Set_CCR(Duty_A, Duty_B, Duty_C);
            break;
        }
        break;
    }
    // 12——Q轴闭环阶跃模型验证
    case 12:{
        if (HK_END == false)
        {
            if (counter == ((1 << PRBS_N) - 1) * PRBS_n)
            {
                Q_Controller.Setvalue = Step_Start;
                HK_END = true;
                counter = 0;
                HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
                break;
            }
            else if (m_seq[counter] == 0)
            {
                Q_Controller.Setvalue = Step_Start;
            }
            else
            {
                Q_Controller.Setvalue = Step_End;
            }
            Q_Controller.Error_Now = Q_Controller.Setvalue - Q;
            Discrete_Controller(&Q_Controller);
            Uq = Q_Controller.Output_Now;
            // 电流环前馈解耦
            float we = wm * 0.10472 * MOTOR_POLE_PAIRS;
            Ud -= we * MOTOR_Ld * Q;
            Uq += we * (MOTOR_Lq * D + MOTOR_Psi);
            Output_D[counter] = D;
            Output_Q[counter] = Q;
            Output_theta_e[counter] = theta_e;
            counter++;
            SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
            // 设定CCR值
            Set_CCR(Duty_A, Duty_B, Duty_C);
            break;
        }
        break;
    }
    default: {
        SVPWM_Calculation(&Ud, &Uq, Sin, Cos, Udc, &Duty_A, &Duty_B, &Duty_C);
        // 设定CCR值
        Set_CCR(Duty_A, Duty_B, Duty_C);
        break;
    }
    }
    HAL_GPIO_WritePin(Test_GPIO_Port,Test_Pin, GPIO_PIN_RESET); 
}

void Speed_Control()
{
    // DQ轴电流控制器
    extern Discrete_Controller_Struct D_Controller;
    extern Discrete_Controller_Struct Q_Controller;
    // 速度环控制器
    extern Discrete_Controller_Struct Speed_Controller;
    // 当前角速度
    extern float wm;
    // 状态计数器
    extern int counter;
    // DQ轴驱动电压
    extern float Ud;
    extern float Uq;
    // 参数辨识标志位
    extern int Identification_Mode;
    // 伪随机辨识结束标志位
    extern bool HK_END;
    // M序列和输出序列
    extern uint16_t m_seq[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_D[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_Q[((1 << PRBS_N) - 1) * PRBS_n];
    extern float Output_theta_e[((1 << PRBS_N) - 1) * PRBS_n];
    // 永磁体磁链
    extern float Psi;
    extern float Psi_Win[10];
    // DQ轴电流
    extern float D;
    extern float Q;
    // 电角度
    extern float theta_e;

    switch (Identification_Mode)
    {
    // 1——相电阻，Ud正向
    case 1: {
        if (counter == 0)
        {
            Ud = 0.5;
        }
        if (counter == 501)
        {
            Ud += 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
        break;
    }
    // 2——相电阻，Ud负向
    case 2: {
        if (counter == 0)
        {
            Ud = -0.5;
        }
        if (counter == 501)
        {
            Ud -= 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
        break;
    }
    // 3——相电阻，Uq正向
    case 3: {
        if (counter == 0)
        {
            Uq = 0.5;
        }
        if (counter == 501)
        {
            Uq += 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
        break;
    }
    // 4——相电阻，Uq负向
    case 4: {
        if (counter == 0)
        {
            Uq = -0.5;
        }
        if (counter == 501)
        {
            Uq -= 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
        break;
    }
    // 5——伪随机辨识Ld同步电感
    case 5: {
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                Ud = PRBS_Work_Point;
                D = 0;
                break;
            }
            if (m_seq[counter] == 0)
            {
                Ud = PRBS_Work_Point + PRBS_A;
            }
            else
            {
                Ud = PRBS_Work_Point - PRBS_A;
            }
            D = Output_D[counter];
            Q = Output_Q[counter];
            theta_e = Output_theta_e[counter];
            counter++;
            break;
        }
        break;
    }
    // 6——永磁体磁链辨识，正向
    case 6: {
        if (counter == 0)
        {
            Uq = 1;
        }
        if (counter == 1001)
        {
            Uq += 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
        float we = wm * MOTOR_POLE_PAIRS * 2 * PI / 60.0;
        if (we == 0)
        {
            break;
        }
        for (int i = 1; i < 50; i++)
        {
            Psi_Win[i] = Psi_Win[i - 1];
        }
        Psi_Win[0] = (Uq - Q * MOTOR_R - MOTOR_Ld * we * D) / we;
        Psi = 0;
        for (int i = 0; i < 50; i++)
        {
            Psi += Psi_Win[i];
        }
        Psi = Psi / 50.0;
        break;
    }
    // 7——永磁体磁链辨识，负向
    case 7: {
        if (counter == 0)
        {
            Uq = -1;
        }
        if (counter == 1001)
        {
            Uq -= 0.1;
            counter = 1;
        }
        else
        {
            counter++;
        }
        float we = wm * MOTOR_POLE_PAIRS * 2 * PI / 60.0;
        if (we == 0)
        {
            break;
        }
        for (int i = 1; i < 10; i++)
        {
            Psi_Win[i] = Psi_Win[i - 1];
        }
        Psi_Win[0] = (Uq - Q * MOTOR_R - MOTOR_Ld * we * D) / we;
        Psi = 0;
        for (int i = 0; i < 10; i++)
        {
            Psi += Psi_Win[i];
        }
        Psi = Psi / 10.0;
        break;
    }
    // 8——伪随机辨识Lq同步电感
    case 8: {
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                Uq = PRBS_Work_Point;
                Q = 0;
                break;
            }
            if (m_seq[counter] == 0)
            {
                Uq = PRBS_Work_Point + PRBS_A;
            }
            else if (m_seq[counter] == 1)
            {
                Uq = PRBS_Work_Point - PRBS_A;
            }
            D = Output_D[counter];
            Q = Output_Q[counter];
            theta_e = Output_theta_e[counter];
            counter++;
            break;
        }
        break;
    }
    // 9——D轴单位阶跃模型验证
    case 9: {
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                Ud = 0;
                D = 0;
                break;
            }
            else if (m_seq[counter] == 0)
            {
                Ud = Step_Start;
            }
            else
            {
                Ud = Step_End;
            }
            D = Output_D[counter];
            Q = Output_Q[counter];
            theta_e = Output_theta_e[counter];
            counter++;
            break;
        }
        break;
    }
    // 10——Uq轴单位阶跃模型验证
    case 10: {
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                Uq = 0;
                Q = 0;
                break;
            }
            else if (m_seq[counter] == 0)
            {
                Uq = Step_Start;
            }
            else
            {
                Uq = Step_End;
            }
            D = Output_D[counter];
            Q = Output_Q[counter];
            theta_e = Output_theta_e[counter];
            counter++;
            break;
        }
        break;
    }
    // 11——D轴闭环阶跃模型验证
    case 11:{
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                D_Controller.Setvalue = Step_Start;
                D = 0;
                break;
            }
            if (m_seq[counter] == 0)
            {
                D_Controller.Setvalue = Step_Start;
            }
            else
            {
                D_Controller.Setvalue = Step_End;
            }
            D = Output_D[counter];
            Q = Output_Q[counter];
            theta_e = Output_theta_e[counter];
            counter++;
            break;
        }
        break;
    }
    // 12——Q轴闭环阶跃模型验证
    case 12:{
        if (HK_END == true)
        {
            HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_4);
            if (counter >= (((1 << PRBS_N) - 1) * PRBS_n))
            {
                Q_Controller.Setvalue = Step_Start;
                Q = 0;
                break;
            }
            if (m_seq[counter] == 0)
            {
                Q_Controller.Setvalue = Step_Start;
            }
            else
            {
                Q_Controller.Setvalue = Step_End;
            }
            D = Output_D[counter];
            Q = Output_Q[counter];
            theta_e = Output_theta_e[counter];
            counter++;
            break;
        }
        break;
    }
}
}

void Controller_Init(float a1, float a2, float a3, float a4, float a5, float b0,
                     float b1, float b2, float b3, float b4, float b5, float Default_Set,
                     Discrete_Controller_Struct *Controller)
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
    
