//
// Created by 33161 on 2025/12/20.
//

#include "PID.h"
#include <math.h>

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 * @param max_output 输出限幅
 * @param min_output 输出下限
 * @param max_integral 积分限幅
 */
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, 
               float max_output, float min_output, float max_integral) {
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->target = 0.0f;
    pid->feedback = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_output;
    pid->min_output = min_output;
    pid->max_integral = max_integral;
}

/**
 * @brief PID计算
 * @param pid PID控制器结构体指针
 * @param target 目标值
 * @param feedback 反馈值
 * @return PID输出值
 */
float PID_Calculate(PID_Controller_t* pid, float target, float feedback) {
    if (pid == NULL) return 0.0f;
    
    pid->target = target;
    pid->feedback = feedback;
    pid->error = target - feedback;
    
    // 积分项计算
    pid->integral += pid->error;
    
    // 积分限幅
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }
    
    // 微分项计算
    float derivative = pid->error - pid->last_error;
    
    // PID输出计算
    pid->output = pid->kp * pid->error + 
                  pid->ki * pid->integral + 
                  pid->kd * derivative;
    
    // 输出限幅
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < pid->min_output) {
        pid->output = pid->min_output;
    }
    
    // 保存当前误差用于下次微分计算
    pid->last_error = pid->error;
    
    return pid->output;
}

/**
 * @brief 重置PID控制器
 * @param pid PID控制器结构体指针
 */
void PID_Reset(PID_Controller_t* pid) {
    if (pid == NULL) return;
    
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 设置PID参数
 * @param pid PID控制器结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
void PID_SetParams(PID_Controller_t* pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}