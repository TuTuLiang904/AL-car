#include "Encoder.h"

volatile int16_t Encoder_Left_Count = 0;
volatile int16_t Encoder_Right_Count = 0;

void Encoder_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入，抗干扰
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_12 | GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // 配置中断源
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);  // 左轮 E1A
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12); // 右轮 E2A
    
    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line6; // 左轮
    EXTI_Init(&EXTI_InitStructure);
    
    EXTI_InitStructure.EXTI_Line = EXTI_Line12; // 右轮
    EXTI_Init(&EXTI_InitStructure);
    
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
}

int16_t Encoder_Get_Left(void) {
    int16_t Speed = Encoder_Left_Count;
    Encoder_Left_Count = 0;
    return Speed;
}

int16_t Encoder_Get_Right(void) {
    int16_t Speed = Encoder_Right_Count;
    Encoder_Right_Count = 0;
    return Speed;
}

// === 左轮中断：这里做了软件反向，修正你的接线问题 ===
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_6) == 0) {
            // 原本这里是减，现在改成加 (++)
            if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 0) Encoder_Left_Count++; 
            else Encoder_Left_Count--; 
        } else {
            // 原本这里是加，现在改成减 (--)
            if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7) == 0) Encoder_Left_Count--; 
            else Encoder_Left_Count++; 
        }
        EXTI_ClearITPendingBit(EXTI_Line6);
    }
}

// === 右轮中断：保持正常逻辑 ===
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0) {
            if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0) Encoder_Right_Count--;
            else Encoder_Right_Count++;
        } else {
            if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0) Encoder_Right_Count++;
            else Encoder_Right_Count--;
        }
        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}
