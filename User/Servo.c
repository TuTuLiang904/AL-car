#include "Servo.h"

void Servo_Init(void)
{
    // 1. 开启时钟：TIM3 和 GPIOA, GPIOB
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    // 配置 PA6 (给 TIM3_CH1 用)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // 配置 PB0, PB1 (给 TIM3_CH3, CH4 用)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 2. 配置 TIM3 时基 (50Hz)
    TIM_InternalClockConfig(TIM3);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 72 - 1; 
    TIM_TimeBaseInitStructure.TIM_Period = 20000 - 1; 
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0; 
    
    // 初始化 3 个通道
    TIM_OC1Init(TIM3, &TIM_OCInitStructure); // PA6 (舵机3)
    TIM_OC3Init(TIM3, &TIM_OCInitStructure); // PB0 (舵机1)
    TIM_OC4Init(TIM3, &TIM_OCInitStructure); // PB1 (舵机2)
    
    TIM_Cmd(TIM3, ENABLE);
}

void Servo_SetAngle(uint8_t id, float angle)
{
//    if (angle < 0) angle = 0;
//    if (angle > 180) angle = 180;
    
    uint16_t pwm_val = (uint16_t)(500 + (angle / 180.0) * 2000);
    
    switch(id)
    {
        case 1: TIM_SetCompare3(TIM3, pwm_val); break; // PB0
        case 2: TIM_SetCompare4(TIM3, pwm_val); break; // PB1
        case 3: TIM_SetCompare1(TIM3, pwm_val); break; // PA6 (新位置)
        default: break;
    }
}
