/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ADC.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "board.h"
#include "bsp_mpu6050.h"
#include "bsp_tb6612.h"
#include "clock.h"
#include "control.h"
#include "inv_mpu.h"
#include "oled_hardware_i2c.h"
#include "pid.h"
#include "stdio.h"
#include "ti_msp_dl_config.h"
float count_l;
int line_flag;
char text[32];
int main(void) {

  SYSCFG_DL_init();

  MPU6050_Init();

  uint8_t ret = 1;

  float pitch = 0, roll = 0, yaw = 0; // 欧拉角

  lc_printf("start\r\n");

  // DMP初始化
  while (mpu_dmp_init()) {
    lc_printf("dmp error\r\n");
    delay_ms(100);
  }

  lc_printf("Initialization Data Succeed \r\n");
  OLED_Init();
  TB6612_Motor_Stop();
  Motor_Init();
  xunji_init();

  // AO_Control(0, 50); // A端电机转动 速度最大440
  // BO_Control(0, 50);
  set_pid_target(&pid1, 50);
  set_pid_target(&pid2, 50);

  set_p_i_d(&pid1, 0.6, 0.9, 1.2);
  set_p_i_d(&pid2, 0.8, 0.8, 0.5);
  count_l = 160;
  line_flag=0;

  while (1) {

    // OLED_ShowString(0, 0, (uint8_t *)"Pitch", 8);
    // OLED_ShowString(0, 2, (uint8_t *)" Roll", 8);
    // OLED_ShowString(0, 4, (uint8_t *)"  Yaw", 8);

    // sprintf(text,"V1=%d ,V2=%d ",V1,V2);

    // OLED_ShowString(0, 6,(uint8_t *)text, 8);
    if(line_flag==0)
    {
    if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
      if((int)yaw<0)
      {
        yaw=yaw+360;
      }
      lc_printf("%d,%d,%d\n", (int)pitch, (int)roll, (int)yaw);
      sprintf(text, "pitch=%d",(int)pitch);
      OLED_ShowString(0, 0, (uint8_t *)text, 8);
      sprintf(text, "roll=%d",(int)roll);
      OLED_ShowString(0, 2, (uint8_t *)text, 8);
      sprintf(text, "yaw=%d",(int)yaw);
      OLED_ShowString(0, 4, (uint8_t *)text, 8);

}
    }
    // delay_ms(20); // 根据设置的采样率，不可设置延时过大
  }
}
