//
// Created by 33161 on 2025/12/17.
//

#include "M2006_sp.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_can.h"
#include "can.h"
// 电机数据数组
static M2006_Motor_t motors[4];

// CAN发送数据缓冲区
static uint8_t can_tx_data[8] = {0};

/**
 * @brief 初始化M2006电机
 */
void M2006_Init(void) {
    // 初始化电机数据
    for(int i = 0; i < 4; i++) {
        motors[i].angle = 0;
        motors[i].speed = 0;
        motors[i].current = 0;
        motors[i].temperature = 0;
        motors[i].target_current = 0;
    }
    
    // 配置CAN过滤器 - 专门针对M2006电机ID (0x201-0x204)
    CAN_FilterTypeDef filter_config;
    filter_config.FilterBank = 0;
    filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_config.FilterScale = CAN_FILTERSCALE_32BIT;
    
    // 配置过滤器接收0x201-0x204范围内的ID (电机反馈ID)
    // 标准ID左移5位放入FilterIdHigh的高位
    filter_config.FilterIdHigh = (0x201 << 5);  // 标准ID左移5位
    filter_config.FilterIdLow = 0x0000;
    // 掩码设置为0xFFE0，表示只匹配ID的高11位(标准标识符)
    // 这样可以接收0x201-0x204范围内的所有ID
    filter_config.FilterMaskIdHigh = 0xFFE0;    // 掩码：匹配高11位，忽略低5位
    filter_config.FilterMaskIdLow = 0x0000;
    
    filter_config.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter_config.FilterActivation = CAN_FILTER_ENABLE;
    filter_config.SlaveStartFilterBank = 14;
    
    if (HAL_CAN_ConfigFilter(&hcan1, &filter_config) != HAL_OK) {
        Error_Handler();
    }
    
    // 配置第二个过滤器，用于接收控制ID 0x200的响应（如果有的话）
    filter_config.FilterBank = 1;
    filter_config.FilterIdHigh = (0x200 << 5);  // 控制ID
    filter_config.FilterMaskIdHigh = 0xFFE0;     // 精确匹配0x200
    
    if (HAL_CAN_ConfigFilter(&hcan1, &filter_config) != HAL_OK) {
        Error_Handler();
    }
    
    // 启动CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    
    // 激活CAN接收中断
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }
    
    // 电机使能 - 发送唤醒命令
    HAL_Delay(100);  // 等待CAN稳定
    M2006_ControlMotors(0, 0, 0, 0);  // 发送零电流命令唤醒电机
    HAL_Delay(100);  // 等待电机响应
}

/**
 * @brief 设置单个电机电流
 * @param motor_id 电机ID (1-4)
 * @param current 目标电流 (-16384 ~ 16384)
 */
void M2006_SetCurrent(uint8_t motor_id, int16_t current) {
    if(motor_id < 1 || motor_id > 4) return;
    
    // 限制电流范围
    if(current > 16384) current = 16384;
    if(current < -16384) current = -16384;
    
    motors[motor_id-1].target_current = current;
}

/**
 * @brief 控制四个电机
 * @param motor1_current 电机1目标电流
 * @param motor2_current 电机2目标电流
 * @param motor3_current 电机3目标电流
 * @param motor4_current 电机4目标电流
 */
void M2006_ControlMotors(int16_t motor1_current, int16_t motor2_current, 
                         int16_t motor3_current, int16_t motor4_current) {
    // 设置各电机电流
    M2006_SetCurrent(1, motor1_current);
    M2006_SetCurrent(2, motor2_current);
    M2006_SetCurrent(3, motor3_current);
    M2006_SetCurrent(4, motor4_current);
    
    // 准备CAN数据
    can_tx_data[0] = (uint8_t)(motors[0].target_current >> 8);
    can_tx_data[1] = (uint8_t)(motors[0].target_current);
    can_tx_data[2] = (uint8_t)(motors[1].target_current >> 8);
    can_tx_data[3] = (uint8_t)(motors[1].target_current);
    can_tx_data[4] = (uint8_t)(motors[2].target_current >> 8);
    can_tx_data[5] = (uint8_t)(motors[2].target_current);
    can_tx_data[6] = (uint8_t)(motors[3].target_current >> 8);
    can_tx_data[7] = (uint8_t)(motors[3].target_current);
    
    // 发送CAN消息
    CAN_TxHeaderTypeDef tx_header;
    tx_header.StdId = CAN_CONTROL_ID;
    tx_header.ExtId = 0;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.IDE = CAN_ID_STD;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;
    
    uint32_t tx_mailbox;
    if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, can_tx_data, &tx_mailbox) != HAL_OK) {
        // 修复：在中断中不调用Error_Handler，直接返回
        return;
    }
}

/**
 * @brief 处理电机反馈数据
 * @param hcan CAN句柄
 * @param RxFifo 接收FIFO
 */
void M2006_ProcessFeedback(CAN_HandleTypeDef* hcan, uint32_t RxFifo) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, RxFifo, &rx_header, rx_data) == HAL_OK) {
        // 调试：接收到CAN消息
        static uint32_t receive_count = 0;
        receive_count++;
        
        // 调试：打印接收到的CAN ID
        // 可以通过LED或其他方式指示接收到数据
        
        // 根据接收ID确定电机
        uint8_t motor_index = 0;
        if (rx_header.StdId >= MOTOR_ID_1 && rx_header.StdId <= MOTOR_ID_4) {
            motor_index = rx_header.StdId - MOTOR_ID_1;
            
            // 解析电机数据
            motors[motor_index].angle = (int16_t)((rx_data[0] << 8) | rx_data[1]);
            motors[motor_index].speed = (int16_t)((rx_data[2] << 8) | rx_data[3]);
            motors[motor_index].current = (int16_t)((rx_data[4] << 8) | rx_data[5]);
            motors[motor_index].temperature = rx_data[6];
        } else {
            // 调试：接收到非电机消息，记录ID以便调试
            // 可以设置一个变量记录接收到的非预期ID
            static uint32_t last_unknown_id = 0;
            last_unknown_id = rx_header.StdId;
        }
    }
}

/**
 * @brief 获取电机数据
 * @param motor_id 电机ID (1-4)
 * @param motor_data 电机数据结构体指针
 */
void M2006_GetMotorData(uint8_t motor_id, M2006_Motor_t* motor_data) {
    if(motor_id < 1 || motor_id > 4 || motor_data == NULL) return;
    
    *motor_data = motors[motor_id-1];
}

/**
 * @brief 获取电机转速 (原始值)
 * @param motor_id 电机ID (1-4)
 * @return 电机转速 (dps)
 */
int16_t M2006_GetMotorSpeed(uint8_t motor_id) {
    if(motor_id < 1 || motor_id > 4) return 0;
    
    return motors[motor_id-1].speed;
}

/**
 * @brief 获取电机转速 (RPM)
 * @param motor_id 电机ID (1-4)
 * @return 电机转速 (RPM)
 */
float M2006_GetMotorSpeedRPM(uint8_t motor_id) {
    if(motor_id < 1 || motor_id > 4) return 0.0f;
    
    //转速就是原始值，不需要换算
    return (float)(motors[motor_id-1].speed) ;
}

// ==================== 转速PID控制函数 ====================

/**
 * @brief 初始化电机转速控制
 * @param control 转速控制结构体指针
 * @param motor_id 电机ID (1-4)
 * @param kp PID比例系数
 * @param ki PID积分系数
 * @param kd PID微分系数
 */
void M2006_SpeedControlInit(M2006_SpeedControl_t* control, uint8_t motor_id, 
                           float kp, float ki, float kd) {
    if (control == NULL || motor_id < 1 || motor_id > 4) return;
    
    control->motor = &motors[motor_id - 1];
    control->motor_id = motor_id;
    control->target_rpm = 0.0f;
    control->current_rpm = 0.0f;
    control->enable = 0;
    
    // 初始化PID控制器
    // 输出限幅：电流范围 -16384 ~ 16384
    PID_Init(&control->speed_pid, kp, ki, kd, 16384.0f, -16384.0f, 10000.0f);
}

/**
 * @brief 设置目标转速
 * @param control 转速控制结构体指针
 * @param target_rpm 目标转速 (RPM)
 */
void M2006_SetTargetSpeed(M2006_SpeedControl_t* control, float target_rpm) {
    if (control == NULL) return;
    
    control->target_rpm = target_rpm;
}

/**
 * @brief 更新转速控制
 * @param control 转速控制结构体指针
 */
void M2006_SpeedControlUpdate(M2006_SpeedControl_t* control) {
    if (control == NULL || control->enable == 0) return;
    
    // 获取当前转速
    control->current_rpm = M2006_GetMotorSpeedRPM(control->motor_id);
    
    // PID计算
    float pid_output = PID_Calculate(&control->speed_pid, control->target_rpm, control->current_rpm);
    
    // 设置电机电流
    M2006_SetCurrent(control->motor_id, (int16_t)pid_output);
    
    // 修复：只发送当前控制的电机，避免CAN总线拥塞
    static int16_t motor_currents[4] = {0};
    motor_currents[control->motor_id - 1] = (int16_t)pid_output;
    
    // 只发送非零电流的电机命令，减少CAN总线负载
    if (pid_output != 0.0f || motor_currents[control->motor_id - 1] != 0) {
        M2006_ControlMotors(motor_currents[0], motor_currents[1], 
                            motor_currents[2], motor_currents[3]);
    }
}

/**
 * @brief 使能/禁用转速控制
 * @param control 转速控制结构体指针
 * @param enable 使能标志 (1=使能, 0=禁用)
 */
void M2006_EnableSpeedControl(M2006_SpeedControl_t* control, uint8_t enable) {
    if (control == NULL) return;
    
    control->enable = enable;
    
    if (enable == 0) {
        // 禁用时停止电机
        M2006_SetCurrent(control->motor_id, 0);
        PID_Reset(&control->speed_pid);
    }
}

/**
 * @brief 获取目标转速
 * @param control 转速控制结构体指针
 * @return 目标转速 (RPM)
 */
float M2006_GetTargetSpeed(M2006_SpeedControl_t* control) {
    if (control == NULL) return 0.0f;
    return control->target_rpm;
}

/**
 * @brief 获取当前转速
 * @param control 转速控制结构体指针
 * @return 当前转速 (RPM)
 */
float M2006_GetCurrentSpeed(M2006_SpeedControl_t* control) {
    if (control == NULL) return 0.0f;
    return control->current_rpm;
}

// ==================== 调试函数 ====================

/**
 * @brief 检查CAN接收状态
 * @return 1表示接收到过数据，0表示未接收到数据
 */
uint8_t M2006_CheckCANReceiveStatus(void) {
    // 检查FIFO0中是否有待处理消息
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        return 1;
    }
    return 0;
}

/**
 * @brief 获取CAN错误状态
 * @return CAN错误状态码
 */
uint32_t M2006_GetCANErrorStatus(void) {
    return HAL_CAN_GetError(&hcan1);
}

/**
 * @brief 检查CAN总线状态
 * @return 1表示总线正常，0表示有错误
 */
uint8_t M2006_CheckCANBusStatus(void) {
    // 检查CAN总线是否关闭
    if (HAL_CAN_GetState(&hcan1) == HAL_CAN_STATE_SLEEP_ACTIVE) {
        return 1;
    }
    
    // 检查是否有错误
    uint32_t error = HAL_CAN_GetError(&hcan1);
    if (error != HAL_CAN_ERROR_NONE) {
        return 0;
    }
    
    return 1;
}
