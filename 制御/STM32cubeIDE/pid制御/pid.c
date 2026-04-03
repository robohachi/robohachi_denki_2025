/*
 * pid.c
 *
 *  Created on: Mar 23, 2025
 *      Author: yuzuk
 */

#include "pid.h"

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float max_output, float min_output) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->target = 0.0f;
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->max_output = max_output;
    pid->min_output = min_output;
}

float PID_Calculate(PID_TypeDef *pid, float current_value) {
    // 誤差の計算
    pid->error = pid->target - current_value;

    // 積分項の計算
    pid->integral += pid->error;

    // アンチワインドアップ（積分項の制限）
    if (pid->integral > pid->max_output) {
        pid->integral = pid->max_output;
    } else if (pid->integral < pid->min_output) {
        pid->integral = pid->min_output;
    }

    // 微分項の計算
    pid->derivative = pid->error - pid->prev_error;

    // PID出力の計算
    pid->output = (pid->Kp * pid->error) + (pid->Ki * pid->integral) + (pid->Kd * pid->derivative);

    // 出力制限
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < pid->min_output) {
        pid->output = pid->min_output;
    }

    // 前回の誤差を更新
    pid->prev_error = pid->error;

    return pid->output;
}

void PID_Reset(PID_TypeDef *pid) {
    pid->error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
}

void PID_SetTarget(PID_TypeDef *pid, float target) {
    pid->target = target;
}


