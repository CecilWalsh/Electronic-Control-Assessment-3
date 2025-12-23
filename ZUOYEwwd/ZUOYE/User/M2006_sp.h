/*
 *  brief M2006的板级支持包
 *
 */

#ifndef ZUOYE_M2006_SP_H
#define ZUOYE_M2006_SP_H

#include "main.h"
#include "can.h"
#include "PID.h"

// 电机ID定义
#define MOTOR_ID_1 0x201
#define MOTOR_ID_2 0x202
#define MOTOR_ID_3 0x203
#define MOTOR_ID_4 0x204

// CAN发送ID定义
#define CAN_CONTROL_ID 0x200

// 电机数据结构体
typedef struct {
    int16_t angle;        // 角度
    int16_t speed;        // 速度
    int16_t current;      // 转矩电流
    int8_t temperature;   // 温度
    int16_t target_current; // 目标电流
} M2006_Motor_t;

// 电机转速控制结构体
typedef struct {
    M2006_Motor_t* motor;        // 电机数据指针
    PID_Controller_t speed_pid;   // 转速PID控制器
    float target_rpm;            // 目标转速 (RPM)
    float current_rpm;           // 当前转速 (RPM)
    uint8_t motor_id;            // 电机ID
    uint8_t enable;              // 使能标志
} M2006_SpeedControl_t;

// 函数声明
void M2006_Init(void);
void M2006_SetCurrent(uint8_t motor_id, int16_t current);
void M2006_ControlMotors(int16_t motor1_current, int16_t motor2_current, 
                         int16_t motor3_current, int16_t motor4_current);
void M2006_ProcessFeedback(CAN_HandleTypeDef* hcan, uint32_t RxFifo);
void M2006_GetMotorData(uint8_t motor_id, M2006_Motor_t* motor_data);
int16_t M2006_GetMotorSpeed(uint8_t motor_id);
float M2006_GetMotorSpeedRPM(uint8_t motor_id);

// 转速控制函数
void M2006_SpeedControlInit(M2006_SpeedControl_t* control, uint8_t motor_id, 
                           float kp, float ki, float kd);
void M2006_SetTargetSpeed(M2006_SpeedControl_t* control, float target_rpm);
void M2006_SpeedControlUpdate(M2006_SpeedControl_t* control);
void M2006_EnableSpeedControl(M2006_SpeedControl_t* control, uint8_t enable);
float M2006_GetTargetSpeed(M2006_SpeedControl_t* control);
float M2006_GetCurrentSpeed(M2006_SpeedControl_t* control);

// 调试函数
uint8_t M2006_CheckCANReceiveStatus(void);
uint32_t M2006_GetReceiveCount(void);
uint32_t M2006_GetCANErrorStatus(void);
uint8_t M2006_CheckCANBusStatus(void);

#endif //ZUOYE_M2006_SP_H