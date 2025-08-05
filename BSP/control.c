#include "control.h"
#include "board.h"
#include "bsp_tb6612.h"
#include "oled_hardware_i2c.h"
#include "pid.h"
#include "xunji.h"

float line_count,zhuan_count;
int V1, V2;
int PID_V1, PID_V2;
void TIMER_1_INST_IRQHandler(void) {
  xunji_proc();
  V1 = (Motor_Get_Encoder(0) * 3000) / 660;
  V2 = (Motor_Get_Encoder(1) * 3000) / 660;
  if(line_flag==1)
  {

  line_count+=(((V1+V2)/2)*15.09)/3000;
  
  PID_V1 = speed_pid_realize(&pid1, V1);
  PID_V2 = speed_pid_realize(&pid2, V2);
  if (PID_V1 > 100)
    PID_V1 = 100;
  if (PID_V1 < -100)
    PID_V1 = -100;
  if (PID_V2 > 100)
    PID_V2 = 100;
  if (PID_V2 < -100)
    PID_V2 = -100;
  AO_Control(0, PID_V1); // A端电机转动 速度最大1000
  BO_Control(0, PID_V2);
  lc_printf("%d,%d,50,100\n",V1,V2);
  set_pid_target(&pid1, 50-Line_Num);
  set_pid_target(&pid2, 50+Line_Num);
  sprintf(text, "=%.1f ",line_count);
  OLED_ShowString(0, 4, (uint8_t *)text, 8);
  if(line_count>=count_l)
  {
    TB6612_Motor_Stop();
    line_flag=0;
  }
}
}