//
// Created by 33161 on 2025/12/20.
//

#ifndef ZUOYE_PID_H
#define ZUOYE_PID_H

#include "main.h"

// PID结构体
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float target;       // 目标值
    float feedback;     // 反馈值
    float error;        // 当前误差
    float last_error;   // 上次误差
    float integral;     // 积分累计
    float output;       // PID输出
    float max_output;   // 输出限幅
    float min_output;   // 输出下限
    float max_integral; // 积分限幅
} PID_Controller_t;

// 函数声明
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, 
               float max_output, float min_output, float max_integral);
float PID_Calculate(PID_Controller_t* pid, float target, float feedback);
void PID_Reset(PID_Controller_t* pid);
void PID_SetParams(PID_Controller_t* pid, float kp, float ki, float kd);

#endif //ZUOYE_PID_H