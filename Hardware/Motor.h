#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"  // <--- 这一行非常关键！你是不是写漏了？

void MotorAll_Init(void);
void MotorL_SetSpeed(int8_t Speed);
void MotorR_SetSpeed(int8_t Speed);

#endif
