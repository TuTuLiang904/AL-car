#include "stm32f10x.h"
#include "Delay.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "PID.h"
#include "Timer.h"
#include "Servo.h"

// ================= 1. 机械臂角度配置 (请根据实际情况微调) =================
#define ID_YAW      1   // 腰部舵机
#define ID_PITCH    2   // 大臂舵机
#define ID_CLAW     3   // 爪子舵机

#define YAW_BACK    0     // 朝后 (初始)
#define YAW_FRONT   180    

#define PITCH_UP    0    // 抬起
#define PITCH_DOWN  65   // 放下 (去抓) - 数值越大压得越低
#define PITCH_PLACE 10    // 放置 (放车尾)

#define CLAW_OPEN   120    // 张开
#define CLAW_CLOSE  175   // 夹紧

 float Target_Speed_L;
 float Target_Speed_R;
 uint16_t Safety_Watchdog_Count;

// ================= 2. 核心原子动作函数 =================

/**
  * @brief  [动作1] 向前蠕动一步 (用于视觉逼近)
  */
void Move_Inch_Forward(void)
{
    // 1. 慢速向前
    Target_Speed_L = 10; 
    Target_Speed_R = 10;
    
    // 2. 走一小会儿 (根据你想走多远调整这个时间)
    Delay_ms(400); 
    
    // 3. 停车
    Target_Speed_L = 0; 
    Target_Speed_R = 0;
    Delay_ms(500); // 停稳
}

/**
  * @brief  [动作2] 机械臂全套抓取流程
  */
void Grip_Procedure(void)
{
    // 1. 停车防抖
    Target_Speed_L = 0; 
    Target_Speed_R = 0;
    Delay_ms(500); 

    // --- 阶段A: 伸出 ---
    Servo_SetAngle(ID_YAW, YAW_FRONT);     // 转到前面
    Delay_ms(1000);
    Servo_SetAngle(ID_CLAW, CLAW_OPEN);    // 张开
    Delay_ms(500);
    Servo_SetAngle(ID_PITCH, PITCH_DOWN);  // 下压去够
    Delay_ms(1000);

    // --- 阶段B: 抓取 ---
    Servo_SetAngle(ID_CLAW, CLAW_CLOSE);   // 闭合
    Delay_ms(800);

    // --- 阶段C: 搬运 ---
    Servo_SetAngle(ID_PITCH, PITCH_UP);    // 抬起
    Delay_ms(1000);
    Servo_SetAngle(ID_YAW, YAW_BACK);      // 转回后面
    Delay_ms(1000);
    
    // --- 阶段D: 放置 ---
    Servo_SetAngle(ID_PITCH, PITCH_PLACE); // 放低
    Delay_ms(800);
    Servo_SetAngle(ID_CLAW, CLAW_OPEN);    // 松开
    Delay_ms(500);
    
    // --- 阶段E: 复位 ---
    Servo_SetAngle(ID_PITCH, PITCH_UP);    // 抬起
    Delay_ms(600);
    Servo_SetAngle(ID_CLAW, CLAW_CLOSE);   // 闭合防撞
    Delay_ms(500);
    
    // 重置看门狗
    Safety_Watchdog_Count = 0; 
}
void Execute_Unload_Sequence(void)
{
    // === 阶段1: 从车尾把货抓起来 ===
    
    // 1. 确保朝后，并且张开爪子准备去抓背上的货
    Servo_SetAngle(ID_YAW, YAW_BACK);
    Servo_SetAngle(ID_CLAW, CLAW_OPEN);
    Delay_ms(500);

    // 2. 下探到车尾放置位 (PITCH_PLACE 是之前放货的位置)
    Servo_SetAngle(ID_PITCH, PITCH_PLACE);
    Delay_ms(800);

    // 3. 闭合爪子 (抓紧货物)
    Servo_SetAngle(ID_CLAW, CLAW_CLOSE);
    Delay_ms(500);

    // 4. 抬起大臂 (准备搬运)
    Servo_SetAngle(ID_PITCH, PITCH_UP);
    Delay_ms(600);

    // === 阶段2: 搬运到车头并卸货 ===

    // 5. 旋转 180度 到车头方向
    Servo_SetAngle(ID_YAW, YAW_FRONT);
    Delay_ms(1000); // 转大弯给多点时间

    // 6. 下放到地面 (PITCH_DOWN 是之前抓货的高度)
    Servo_SetAngle(ID_PITCH, PITCH_DOWN);
    Delay_ms(800);

    // 7. 张开爪子 (卸货!)
    Servo_SetAngle(ID_CLAW, CLAW_OPEN);
    Delay_ms(500);

    // === 阶段3: 复位 ===

    // 8. 抬起大臂
    Servo_SetAngle(ID_PITCH, PITCH_UP);
    Delay_ms(600);

    // 9. 闭合爪子 (收纳防撞)
    Servo_SetAngle(ID_CLAW, CLAW_CLOSE);
    Delay_ms(300);

    // 10. 转回车尾 (恢复初始状态)
    Servo_SetAngle(ID_YAW, YAW_BACK);
    Delay_ms(800);
    
    // 重置看门狗，防止动作时间过长导致停车逻辑误判
    Safety_Watchdog_Count = 0; 
}

// ================= 3. 主函数 =================
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    MotorAll_Init(); Encoder_Init(); Servo_Init(); 
    Serial_Init(); PID_Init(); Timer1_Init(); 
    
    // 初始姿态
    Servo_SetAngle(ID_YAW, YAW_BACK);
    Servo_SetAngle(ID_PITCH, PITCH_UP);
    Servo_SetAngle(ID_CLAW, CLAW_CLOSE);
    
    while (1)
    {
        if (Serial_RxFlag == 1) 
        {
            Safety_Watchdog_Count = 0; 
            int8_t speed_R = (int8_t)Serial_RxPacket[0];
            int8_t speed_L = (int8_t)Serial_RxPacket[1];
            uint8_t cmd    = Serial_RxPacket[2]; // 命令字

            // === 协议分发 ===
            if (cmd == 1) {
                Move_Inch_Forward(); // 收到1: 蠕动
            }
            else if (cmd == 2) {
                Grip_Procedure();    // 收到2: 抓取
            }
			else if (cmd == 3) {
                Execute_Unload_Sequence(); // 执行卸货
            }
            else {
                Target_Speed_L = (float)speed_L; // 收到0: 正常跑
                Target_Speed_R = (float)speed_R;
            }
            Serial_RxFlag = 0; 
        }
		
        else
        {
            Safety_Watchdog_Count++;
            if (Safety_Watchdog_Count > 100) { // 500ms无信号刹车
                Target_Speed_L = 0; Target_Speed_R = 0;
            }
        }
        Delay_ms(5);
    }

}

