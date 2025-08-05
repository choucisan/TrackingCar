#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "oled_hardware_i2c.h"
#include "board.h"
unsigned short Anolog[8] = {0};
unsigned short white[8] = {2500, 2500, 2500, 2000, 2000, 2000, 2000, 1800};
unsigned short black[8] = {1200, 1200,1200, 600, 600, 600, 600, 400};
unsigned short Normal[8];

No_MCU_Sensor sensor;
unsigned char Digtal;
int Line_Num;
void xunji_init(void) {
  // 初始化传感器，不带黑白值
 
  No_MCU_Ganv_Sensor_Init_Frist(&sensor);
  No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
  Get_Anolog_Value(&sensor, Anolog);
  // 此时打印的ADC的值，可用通过这个ADC作为黑白值的校准
  // 也可以自己写按键逻辑完成一键校准功能
  lc_printf("Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n", Anolog[0], Anolog[1],
            Anolog[2], Anolog[3], Anolog[4], Anolog[5], Anolog[6], Anolog[7]);
  No_MCU_Ganv_Sensor_Init(&sensor, white, black);
}
void xunji_proc(void) {

  // OLED_ShowString(0, 7, (uint8_t *)"MPU6050 Demo", 8);
  // 无时基传感器常规任务，包含模拟量，数字量，归一化量
  No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
  // 有时基传感器常规任务，包含模拟量，数字量，归一化量
  //			No_Mcu_Ganv_Sensor_Task_With_tick(&sensor)
  // 获取传感器数字量结果(只有当有黑白值传入进去了之后才会有这个值！！)
  Digtal = Get_Digtal_For_User(&sensor);

  Get_Anolog_Value(&sensor, Anolog);
  // 此时打印的ADC的值，可用通过这个ADC作为黑白值的校准
  // 也可以自己写按键逻辑完成一键校准功能
  // lc_printf("Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n", Anolog[0], Anolog[1],
  //           Anolog[2], Anolog[3], Anolog[4], Anolog[5], Anolog[6], Anolog[7]);
  // sprintf(text, "%d%d%d%d%d%d%d%d", (Digtal >> 0) & 0x01, (Digtal >> 1) & 0x01,
  //         (Digtal >> 2) & 0x01, (Digtal >> 3) & 0x01, (Digtal >> 4) & 0x01,
  //         (Digtal >> 5) & 0x01, (Digtal >> 6) & 0x01, (Digtal >> 7) & 0x01);
  // OLED_ShowString(0, 0, (uint8_t *)text, 8);
  delay_ms(1);
    if((((Digtal>>3)&0x01)==0)||(((Digtal>>4)&0x01)==0))  Line_Num =  0;
	  if((((Digtal>>4)&0x01)==1)&&(((Digtal>>3)&0x01)==0))  Line_Num =  4;
	  if((((Digtal>>4)&0x01)==0)&&(((Digtal>>3)&0x01)==1))  Line_Num =  -4;	
	  if((((Digtal>>5)&0x01)==1)&&(((Digtal>>2)&0x01)==0))  Line_Num = 12;
	  if((((Digtal>>5)&0x01)==0)&&(((Digtal>>2)&0x01)==1))  Line_Num =  -12;	
	  if((((Digtal>>6)&0x01)==1)&&(((Digtal>>1)&0x01)==0))  Line_Num = 25;
    if((((Digtal>>6)&0x01)==0)&&(((Digtal>>1)&0x01)==1))  Line_Num =  -25;
    if((((Digtal>>7)&0x01)==1)&&(((Digtal>>0)&0x01)==0))  Line_Num = 40;
    if((((Digtal>>7)&0x01)==0)&&(((Digtal>>0)&0x01)==1))  Line_Num =  -40;
    if((Digtal)==0xff)  Line_Num =  0;
}