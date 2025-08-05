#ifndef _BSP_TB6612_H
#define _BSP_TB6612_H

#include "board.h"

#define AIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN1_PIN)) )
#define AIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_AIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_AIN2_PIN)) )

#define BIN1_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN1_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN1_PIN)) )
#define BIN2_OUT(X)  ( (X) ? (DL_GPIO_setPins(TB6612_PORT,TB6612_BIN2_PIN)) : (DL_GPIO_clearPins(TB6612_PORT,TB6612_BIN2_PIN)) )


typedef struct{
    int Should_Get_Encoder_Count;   // 将要获得的编码器计数
    int Obtained_Get_Encoder_Count; // 得到的编码器的计数
}Encoder;


void Motor_Init(void);
int Motor_Get_Encoder(int dir);
void TB6612_Motor_Stop(void);
void AO_Control(uint8_t dir, uint32_t speed);
void BO_Control(uint8_t dir, uint32_t speed);

#endif  /* _BSP_TB6612_H */