#include "Motor.h"

void MotorAll_Init(void)
{
    // 1. GPIO 初始化
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PWM 引脚 (PA0, PA1) -> 复用推挽 (TIM2)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 方向引脚 -> 推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    
    // 【修改点】左轮保持 PA4, PA5 不变
    // 【修改点】右轮从 PA6, PA7 改为 PA2, PA3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 2. TIM2 PWM 初始化 (保持不变)
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = 100 - 1; 
    TIM_TimeBaseStructure.TIM_Prescaler = 36 - 1; 
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    
    TIM_OC1Init(TIM2, &TIM_OCInitStructure); // PA0
    TIM_OC2Init(TIM2, &TIM_OCInitStructure); // PA1
    TIM_Cmd(TIM2, ENABLE);
}

void MotorL_SetSpeed(int8_t Speed)
{
    // 【核心修改】
    // 之前：Speed >= 0 时，Reset PA4, Set PA5
    // 现在：Speed >= 0 时，Set PA4, Reset PA5 (逻辑取反)
    
    if (Speed >= 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_4);     // 改为 Set
        GPIO_ResetBits(GPIOA, GPIO_Pin_5);   // 改为 Reset
        TIM_SetCompare1(TIM2, Speed);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_4);   // 改为 Reset
        GPIO_SetBits(GPIOA, GPIO_Pin_5);     // 改为 Set
        TIM_SetCompare1(TIM2, -Speed);
    }
}

/**
  * @brief  右轮电机控制 (软件反向版)
  * @param  Speed: 速度值 (-100 ~ 100)
  */
void MotorR_SetSpeed(int8_t Speed)
{
    // 【核心修改】
    // 之前：Speed >= 0 时，Reset PA2, Set PA3
    // 现在：Speed >= 0 时，Set PA2, Reset PA3 (逻辑取反)
    
    if (Speed >= 0)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_2);     // 改为 Set
        GPIO_ResetBits(GPIOA, GPIO_Pin_3);   // 改为 Reset
        TIM_SetCompare2(TIM2, Speed);
    }
    else
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);   // 改为 Reset
        GPIO_SetBits(GPIOA, GPIO_Pin_3);     // 改为 Set
        TIM_SetCompare2(TIM2, -Speed);
    }
}

