#ifndef __PID_H
#define __PID_H
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "xunji.h"

typedef struct {
    float target_speed;    // 目标速度（设定值）
    float actual_speed;    // 实际速度（通过编码器反馈）
    float err;             // 速度误差
    float err_last;        // 上一次的速度误差
    float integral;        // 速度误差的积分项
    float Kp;              // 比例增益
    float Ki;              // 积分增益
    float Kd;              // 微分增益
    float actual_val;      // PID 控制输出（占空比）
} _pid;
extern _pid pid1,pid2;


void set_pid_target(_pid *pid, float temp_val);
float get_pid_target(_pid *pid);
void set_p_i_d(_pid *pid, float p, float i, float d);

float speed_pid_realize(_pid *pid, float actual_val);
#endif

