#include "PID.h"

PID_TypeDef PID_Left;
PID_TypeDef PID_Right;

void PID_Init(void)
{
    // === 之前的 0.5 太小了，推不动 ===
    // 因为误差变小了 (只有4)，所以 Kp 要给大一点才能算出足够的 PWM
    
    // 建议直接给到 10.0 甚至 15.0
    // 算一下：误差 4 * 10.0 = 40 (PWM)，这个力度刚好起步！
    
    PID_Left.Kp = 8.0;   
    PID_Left.Ki = 1;    // 先别加积分，只用比例让它走起来
    PID_Left.Kd = 0.0;
    
    PID_Right.Kp = 8.0;
    PID_Right.Ki = 1;
    PID_Right.Kd = 0.0;
    
    PID_Left.Integral = 0.0;
    PID_Right.Integral = 0.0;
}


float PID_Realize(PID_TypeDef *pid, float target, float actual)
{
    pid->Target = target;
    pid->Actual = actual;
    
    pid->Err = pid->Target - pid->Actual;
    pid->Integral += pid->Err;
    
    // 简单的积分限幅，防止积分过大
    if(pid->Integral > 1000) pid->Integral = 1000;
    if(pid->Integral < -1000) pid->Integral = -1000;
    
    float output = (pid->Kp * pid->Err) + (pid->Ki * pid->Integral) + (pid->Kd * (pid->Err - pid->Err_Last));
    
    pid->Err_Last = pid->Err;
    
    return output;
}
	