//
// Created by 33161 on 2025/12/20.
//

#ifndef ZUOYE_VOFA_H
#define ZUOYE_VOFA_H

#include "main.h"
#include "usart.h"
#include "M2006_sp.h"

// VOFA+数据缓冲区大小
#define VOFA_BUFFER_SIZE 128
#define VOFA_RX_BUFFER_SIZE 256

// 命令类型定义
typedef enum {
    CMD_SET_SPEED = 1,    // 设置转速
    CMD_SET_PID_KP,       // 设置PID参数Kp
    CMD_SET_PID_KI,       // 设置PID参数Ki
    CMD_SET_PID_KD,       // 设置PID参数Kd
    CMD_SET_PID_ALL,      // 同时设置所有PID参数
    CMD_GET_STATUS,       // 获取当前状态
    CMD_UNKNOWN           // 未知命令
} VOFA_CommandType_t;

// 命令结构体
typedef struct {
    VOFA_CommandType_t type;
    float value1;
    float value2;
    float value3;
    uint8_t valid;
} VOFA_Command_t;

// VOFA+数据结构体
typedef struct {
    float target_speed;     // 目标转速 (RPM)
    float current_speed;    // 当前转速 (RPM)
    float pid_output;       // PID输出
    float motor_current;    // 电机电流
    float kp;
    float ki;
    float kd;
    uint32_t timestamp;     // 时间戳
} VOFA_Data_t;

// 函数声明
void VOFA_Init(void);
void VOFA_SendData(float target_speed, float current_speed, float pid_output, float motor_current,float kp,float ki,float kd);
void VOFA_SendMotorData(uint8_t motor_id, float target_rpm, float current_rpm, float pid_output);
void VOFA_SendRawData(const char* data);
void VOFA_Begin(void);
void VOFA_End(void);

// 命令接收相关函数
void VOFA_StartRx(void);
void VOFA_ProcessCommand(char* rx_data, uint16_t len);
VOFA_Command_t VOFA_ParseCommand(const char* cmd_str);
void VOFA_ExecuteCommand(VOFA_Command_t* cmd);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

#endif //ZUOYE_VOFA_H