/*
 * pid.h
 *
 *  Created on: Mar 23, 2025
 *      Author: yuzuk
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include "main.h"

typedef struct {
    float Kp;           // 比例ゲイン
    float Ki;           // 積分ゲイン
    float Kd;           // 微分ゲイン
    float target;       // 目標値
    float error;        // 現在の誤差
    float prev_error;   // 前回の誤差
    float integral;     // 積分値
    float derivative;   // 微分値
    float output;       // PID出力
    float max_output;   // 最大出力制限
    float min_output;   // 最小出力制限
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float max_output, float min_output);
float PID_Calculate(PID_TypeDef *pid, float current_value);
void PID_Reset(PID_TypeDef *pid);
void PID_SetTarget(PID_TypeDef *pid, float target);

#endif /* SRC_PID_H_ */
