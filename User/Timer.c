#include "stm32f10x.h"
#include "Timer.h"
#include "Encoder.h"
#include "Motor.h"
#include "PID.h"

// 调试变量
volatile int32_t PID_Heartbeat = 0; 
extern float Target_Speed_L;
extern float Target_Speed_R;

void Timer1_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    TIM_InternalClockConfig(TIM1);
    
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    
    // ============================================
    // 【修改点】提速到 10ms (100Hz)
    // 72MHz / 7200 = 10kHz
    // 10000 / 100 = 100Hz (10ms)
    // ============================================
    TIM_TimeBaseInitStructure.TIM_Period = 100 - 1;   // 改成 100
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1; 
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);
    
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    TIM_Cmd(TIM1, ENABLE);
}

void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        PID_Heartbeat++;
        
        int16_t Speed_L = Encoder_Get_Left();
        int16_t Speed_R = Encoder_Get_Right();
        
        // PID 计算
        float PWM_L = PID_Realize(&PID_Left, Target_Speed_L, (float)Speed_L);
        float PWM_R = PID_Realize(&PID_Right, Target_Speed_R, (float)Speed_R);
        
        // 限幅
        if (PWM_L > 100) PWM_L = 100;
        if (PWM_L < -100) PWM_L = -100;
        if (PWM_R > 100) PWM_R = 100;
        if (PWM_R < -100) PWM_R = -100;
        
        MotorL_SetSpeed((int8_t)PWM_L);
        MotorR_SetSpeed((int8_t)PWM_R);
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}
