#include "pid.h"


_pid pid1,pid2;


void set_pid_target(_pid *pid, float temp_val)
{
  pid->target_speed = temp_val;    // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
float get_pid_target(_pid *pid)
{
  return pid->target_speed;    // 设置当前的目标值
}

void set_p_i_d(_pid *pid, float p, float i, float d)
{
  	pid->Kp = p;    // 设置比例系数 P
		pid->Ki = i;    // 设置积分系数 I
		pid->Kd = d;    // 设置微分系数 D
}
float speed_pid_realize(_pid *pid, float actual_val)
{
    // 计算目标值与实际值的误差
    pid->err = pid->target_speed - actual_val;

    // 限制误差范围，避免快速反向调整
    if ((pid->err < 1.0f) && (pid->err > -1.0f)) {
        pid->err = 0.0f;
    }

    // 积分累积
    pid->integral += pid->err;

    // 积分限幅
    if (pid->integral >= 100) {
        pid->integral = 100;
    } else if (pid->integral < -100) {
        pid->integral = -100;
    }

    // PID 算法实现
    pid->actual_val = pid->Kp * pid->err
                      + pid->Ki * pid->integral
                      + pid->Kd * (pid->err - pid->err_last);

    // 误差传递
    pid->err_last = pid->err;

    // 返回实际控制值
    return pid->actual_val;
}