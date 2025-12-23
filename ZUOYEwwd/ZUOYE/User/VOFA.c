//
// Created by 33161 on 2025/12/20.
//

#include "VOFA.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>

// 外部变量声明 - 访问main.c中的电机控制对象
extern M2006_SpeedControl_t motor1_speed_control;

// 数据发送缓冲区
static char vofa_buffer[VOFA_BUFFER_SIZE];
static uint8_t dma_buffer[VOFA_BUFFER_SIZE];

// 接收缓冲区 - 分离DMA接收缓冲区和命令缓存
static uint8_t rx_char;  // 单字符DMA接收缓冲区
static char rx_command_buffer[VOFA_RX_BUFFER_SIZE];  // 命令缓存缓冲区
static volatile uint16_t rx_write_pos = 0;
static volatile uint8_t rx_busy = 0;

/**
 * @brief 初始化VOFA+通信
 */
void VOFA_Init(void) {
    // UART6已经在main.c中初始化
    // 启动DMA接收
    VOFA_StartRx();
    
    // 发送初始化确认信息
    HAL_Delay(100);  // 等待UART稳定
    VOFA_SendRawData("VOFA+ Command Interface Ready\r\n");
    VOFA_SendRawData("Commands: speed:xxxx, kp:x.x, ki:x.x, kd:x.x, pid:x.x,x.x,x.x\r\n");
}

/**
 * @brief 启动UART6 DMA接收
 */
void VOFA_StartRx(void) {
    if (!rx_busy) {
        rx_busy = 1;
        HAL_StatusTypeDef status = HAL_UART_Receive_DMA(&huart6, &rx_char, 1);
        if (status != HAL_OK) {
            // 如果启动失败，发送错误信息
            VOFA_SendRawData("DMA RX Start Failed\r\n");
            rx_busy = 0;
        }
    }
}

/**
 * @brief UART接收完成回调函数
 * @param huart UART句柄
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART6) {
        // 发送调试信息确认中断被调用
        static uint32_t debug_count = 0;
        if ((debug_count++ % 100) == 0) {
            VOFA_SendRawData("UART6 RX IRQ Active\r\n");
        }
        
        // 处理接收到的字符
        uint8_t received_char = rx_char;
        
        // 简单的命令解析：以换行符为命令结束
        if (received_char == '\n' || received_char == '\r') {
            if (rx_write_pos > 0) {
                rx_command_buffer[rx_write_pos] = '\0';  // 字符串结束符
                VOFA_ProcessCommand(rx_command_buffer, rx_write_pos);
                rx_write_pos = 0;
            }
        } else if (rx_write_pos < VOFA_RX_BUFFER_SIZE - 1) {
            rx_command_buffer[rx_write_pos++] = received_char;
        }
        
        // 重新启动DMA接收
        HAL_UART_Receive_DMA(&huart6, &rx_char, 1);
    }
}

/**
 * @brief 处理接收到的命令
 * @param rx_data 接收到的数据
 * @param len 数据长度
 */
void VOFA_ProcessCommand(char* rx_data, uint16_t len) {
    // 发送调试信息确认收到命令
    char debug_msg[64];
    snprintf(debug_msg, sizeof(debug_msg), "Received: %s\r\n", rx_data);
    VOFA_SendRawData(debug_msg);
    
    VOFA_Command_t cmd = VOFA_ParseCommand(rx_data);
    if (cmd.valid) {
        VOFA_ExecuteCommand(&cmd);
    } else {
        VOFA_SendRawData("Invalid command format\r\n");
    }
}

/**
 * @brief 解析命令字符串
 * @param cmd_str 命令字符串
 * @return 解析后的命令结构体
 */
VOFA_Command_t VOFA_ParseCommand(const char* cmd_str) {
    VOFA_Command_t cmd = {0};
    cmd.valid = 0;
    
    if (cmd_str == NULL || strlen(cmd_str) == 0) {
        return cmd;
    }
    
    // 支持的命令格式：
    // "speed:10000"     - 设置转速为10000
    // "kp:0.5"          - 设置Kp为0.5
    // "ki:0.1"          - 设置Ki为0.1
    // "kd:0.05"         - 设置Kd为0.05
    // "pid:0.5,0.1,0.05" - 同时设置所有PID参数
    
    char cmd_copy[64];
    strncpy(cmd_copy, cmd_str, sizeof(cmd_copy) - 1);
    cmd_copy[sizeof(cmd_copy) - 1] = '\0';
    
    char* token = strtok(cmd_copy, ":");
    if (token == NULL) {
        return cmd;
    }
    
    // 转换为小写进行比较
    for (int i = 0; token[i]; i++) {
        token[i] = tolower(token[i]);
    }
    
    if (strcmp(token, "speed") == 0) {
        cmd.type = CMD_SET_SPEED;
        token = strtok(NULL, ":");
        if (token) {
            cmd.value1 = atof(token);
            cmd.valid = 1;
        }
    } else if (strcmp(token, "kp") == 0) {
        cmd.type = CMD_SET_PID_KP;
        token = strtok(NULL, ":");
        if (token) {
            cmd.value1 = atof(token);
            cmd.valid = 1;
        }
    } else if (strcmp(token, "ki") == 0) {
        cmd.type = CMD_SET_PID_KI;
        token = strtok(NULL, ":");
        if (token) {
            cmd.value1 = atof(token);
            cmd.valid = 1;
        }
    } else if (strcmp(token, "kd") == 0) {
        cmd.type = CMD_SET_PID_KD;
        token = strtok(NULL, ":");
        if (token) {
            cmd.value1 = atof(token);
            cmd.valid = 1;
        }
    } else if (strcmp(token, "pid") == 0) {
        cmd.type = CMD_SET_PID_ALL;
        token = strtok(NULL, ":");
        if (token) {
            cmd.value1 = atof(token);  // kp
            token = strtok(NULL, ",");
            if (token) {
                cmd.value2 = atof(token);  // ki
                token = strtok(NULL, ",");
                if (token) {
                    cmd.value3 = atof(token);  // kd
                    cmd.valid = 1;
                }
            }
        }
    }
    
    return cmd;
}

/**
 * @brief 执行命令
 * @param cmd 要执行的命令
 */
void VOFA_ExecuteCommand(VOFA_Command_t* cmd) {
    if (!cmd || !cmd->valid) {
        return;
    }
    
    char response[64];
    
    switch (cmd->type) {
        case CMD_SET_SPEED:
            M2006_SetTargetSpeed(&motor1_speed_control, cmd->value1);
            snprintf(response, sizeof(response), "Speed set to: %.2f\r\n", cmd->value1);
            VOFA_SendRawData(response);
            break;
            
        case CMD_SET_PID_KP:
            PID_SetParams(&motor1_speed_control.speed_pid, cmd->value1, 
                         motor1_speed_control.speed_pid.ki, 
                         motor1_speed_control.speed_pid.kd);
            snprintf(response, sizeof(response), "Kp set to: %.3f\r\n", cmd->value1);
            VOFA_SendRawData(response);
            break;
            
        case CMD_SET_PID_KI:
            PID_SetParams(&motor1_speed_control.speed_pid, 
                         motor1_speed_control.speed_pid.kp, 
                         cmd->value1, 
                         motor1_speed_control.speed_pid.kd);
            snprintf(response, sizeof(response), "Ki set to: %.3f\r\n", cmd->value1);
            VOFA_SendRawData(response);
            break;
            
        case CMD_SET_PID_KD:
            PID_SetParams(&motor1_speed_control.speed_pid, 
                         motor1_speed_control.speed_pid.kp, 
                         motor1_speed_control.speed_pid.ki, 
                         cmd->value1);
            snprintf(response, sizeof(response), "Kd set to: %.3f\r\n", cmd->value1);
            VOFA_SendRawData(response);
            break;
            
        case CMD_SET_PID_ALL:
            PID_SetParams(&motor1_speed_control.speed_pid, 
                         cmd->value1, cmd->value2, cmd->value3);
            snprintf(response, sizeof(response), "PID set to: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", 
                    cmd->value1, cmd->value2, cmd->value3);
            VOFA_SendRawData(response);
            break;
            
        case CMD_GET_STATUS:
            snprintf(response, sizeof(response), 
                    "Status - Speed: %.2f, PID: Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n",
                    M2006_GetTargetSpeed(&motor1_speed_control),
                    motor1_speed_control.speed_pid.kp,
                    motor1_speed_control.speed_pid.ki,
                    motor1_speed_control.speed_pid.kd);
            VOFA_SendRawData(response);
            break;
            
        default:
            VOFA_SendRawData("Unknown command\r\n");
            break;
    }
}

/**
 * @brief 发送VOFA+数据帧开始标记
 */
void VOFA_Begin(void) {
    const char* start_marker = "开始\r\n";
    HAL_UART_Transmit(&huart6, (uint8_t*)start_marker, strlen(start_marker), 100);
}

/**
 * @brief 发送VOFA+数据帧结束标记
 */
void VOFA_End(void) {
    const char* end_marker = "结束\r\n";
    HAL_UART_Transmit(&huart6, (uint8_t*)end_marker, strlen(end_marker), 100);
}

/**
 * @brief 发送原始数据字符串
 * @param data 要发送的数据字符串
 */
void VOFA_SendRawData(const char* data) {
    if (data == NULL) return;
    
    uint16_t len = strlen(data);
    if (len > VOFA_BUFFER_SIZE - 1) {
        len = VOFA_BUFFER_SIZE - 1;
    }
    
    // 使用DMA发送数据
    memcpy(dma_buffer, data, len);
    dma_buffer[len] = '\r';  // 添加回车
    dma_buffer[len + 1] = '\n';  // 添加换行
    dma_buffer[len + 2] = '\0';
    
    HAL_UART_Transmit_DMA(&huart6, dma_buffer, len + 2);
}

/**
 * @brief 发送电机数据到VOFA+
 * @param target_speed 目标转速 (RPM)
 * @param current_speed 当前转速 (RPM)
 * @param pid_output PID输出值
 * @param motor_current 电机电流
 * @param kp just kp
 * @param ki of course
 * @param kd of course
 */
void VOFA_SendData(float target_speed, float current_speed, float pid_output, float motor_current,float kp,float ki,float kd) {
    // 格式化数据字符串，VOFA+支持逗号分隔的数值格式
    int len = snprintf(vofa_buffer, VOFA_BUFFER_SIZE, 
                      "%.2f,%.2f,%.2f,%.2f,%.3f,%.3f,%.3f\r\n",
                      target_speed,
                      current_speed,
                      pid_output,
                      motor_current,
                      kp,
                      ki,
                      kd);
    
    if (len > 0 && len < VOFA_BUFFER_SIZE) {
        HAL_UART_Transmit_DMA(&huart6, (uint8_t*)vofa_buffer, len);
    }
}

/**
 * @brief 发送单个电机的详细数据到VOFA+
 * @param motor_id 电机ID (1-4)
 * @param target_rpm 目标转速
 * @param current_rpm 当前转速
 * @param pid_output pid输出及参数
 */
void VOFA_SendMotorData(uint8_t motor_id, float target_rpm, float current_rpm, float pid_output) {
    // 格式化数据，包含电机ID信息
    int len = snprintf(vofa_buffer, VOFA_BUFFER_SIZE, 
                      "M%d:%.2f,%.2f,%.2f,%lu\r\n", 
                      motor_id, target_rpm, current_rpm, pid_output, HAL_GetTick());
    
    if (len > 0 && len < VOFA_BUFFER_SIZE) {
        HAL_UART_Transmit_DMA(&huart6, (uint8_t*)vofa_buffer, len);
    }
}