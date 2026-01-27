#ifndef __PID_H
#define __PID_H

// 定义 PID 结构体 (这就是编译器找不到的东西)
typedef struct
{
    float Target;     // 目标值
    float Actual;     // 实际值
    float Err;        // 当前误差
    float Err_Last;   // 上次误差
    float Kp, Ki, Kd; // 比例、积分、微分系数
    float Voltage;    // 计算出的输出值
    float Integral;   // 积分累计值
} PID_TypeDef;

// 声明外部变量，让 main.c 和 Timer.c 也能用
extern PID_TypeDef PID_Left;
extern PID_TypeDef PID_Right;

// 函数声明
void PID_Init(void);
float PID_Realize(PID_TypeDef *pid, float target, float actual);

#endif
