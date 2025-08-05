#include "bsp_tb6612.h"
#include "board.h"

static volatile Encoder Encoder_A;
static volatile Encoder Encoder_B;
/******************************************************************
 * 函 数 名 称：TB6612_Motor_Stop
 * 函 数 说 明：A端和B端电机停止
 * 函 数 形 参：无
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：无
 ******************************************************************/
void TB6612_Motor_Stop(void) {
  AIN1_OUT(1);
  AIN2_OUT(1);
  BIN1_OUT(1);
  BIN2_OUT(1);
}

/******************************************************************
 * 函 数 名 称：AO_Control
 * 函 数 说 明：A端口电机控制
 * 函 数 形 参：dir旋转方向 1正转0反转   speed旋转速度，范围（0 ~ per-1）
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：speed 0-1000
 ******************************************************************/
void AO_Control(uint8_t dir, uint32_t speed) {
  if (speed > 440 || dir > 1) {
    lc_printf("\nAO_Control parameter error!!!\r\n");
    return;
  }

  if (dir == 1) {
    AIN1_OUT(0);
    AIN2_OUT(1);
  } else {
    AIN1_OUT(1);
    AIN2_OUT(0);
  }

  DL_TimerG_setCaptureCompareValue(PWM_INST, speed, GPIO_PWM_C0_IDX);
}

/******************************************************************
 * 函 数 名 称：BO_Control
 * 函 数 说 明：B端口电机控制
 * 函 数 形 参：dir旋转方向 1正转0反转   speed旋转速度，范围（0 ~ per-1）
 * 函 数 返 回：无
 * 作       者：LCKFB
 * 备       注：speed 0-1000
 ******************************************************************/
void BO_Control(uint8_t dir, uint32_t speed) {
  if (speed > 440 || dir > 1) {
    lc_printf("\nAO_Control parameter error!!!\r\n");
    return;
  }

  if (dir == 1) {
    BIN1_OUT(0);
    BIN2_OUT(1);
  } else {
    BIN1_OUT(1);
    BIN2_OUT(0);
  }

  DL_TimerG_setCaptureCompareValue(PWM_INST, speed, GPIO_PWM_C1_IDX);
}
void Motor_Init(void) {
  // 编码器引脚外部中断
  NVIC_ClearPendingIRQ(ENCODER_INT_IRQN);
  NVIC_EnableIRQ(ENCODER_INT_IRQN);

  // 定时器中断
  NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
  NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
  NVIC_ClearPendingIRQ(TIMER_1_INST_INT_IRQN);
  NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);

  lc_printf("Motor initialized successfully\r\n");
}

/******************************************************************
 * 函 数 名 称：Motor_Get_Encoder
 * 函 数 说 明：获取编码器的值
 * 函 数 形 参：dir=0获取左轮编码器值  dir=1获取右轮编码器值
 * 函 数 返 回：返回对应的编码器值
 * 作       者：LCKFB
 * 备       注：无
 ******************************************************************/
int Motor_Get_Encoder(int dir) {
  if (!dir)
    return Encoder_A.Obtained_Get_Encoder_Count;

  return Encoder_B.Obtained_Get_Encoder_Count;
}

/*******************************************************
函数功能：外部中断模拟编码器信号
入口函数：无
返回  值：无
***********************************************************/
void GROUP1_IRQHandler(void) {
  uint32_t gpio_interrup = 0;

  // 获取中断信号
  gpio_interrup = DL_GPIO_getEnabledInterruptStatus(
      ENCODER_PORT,
      ENCODER_E1A_PIN | ENCODER_E1B_PIN | ENCODER_E2A_PIN | ENCODER_E2B_PIN);

  // encoderA
  if ((gpio_interrup & ENCODER_E1A_PIN) == ENCODER_E1A_PIN) {
    if (!DL_GPIO_readPins(ENCODER_PORT, ENCODER_E1B_PIN)) {
      Encoder_A.Should_Get_Encoder_Count--;
    } else {
      Encoder_A.Should_Get_Encoder_Count++;
    }
  } else if ((gpio_interrup & ENCODER_E1B_PIN) == ENCODER_E1B_PIN) {
    if (!DL_GPIO_readPins(ENCODER_PORT, ENCODER_E1A_PIN)) {
      Encoder_A.Should_Get_Encoder_Count++;
    } else {
      Encoder_A.Should_Get_Encoder_Count--;
    }
  }

  // encoderB
  if ((gpio_interrup & ENCODER_E2A_PIN) == ENCODER_E2A_PIN) {
    if (!DL_GPIO_readPins(ENCODER_PORT, ENCODER_E2B_PIN)) {
      Encoder_B.Should_Get_Encoder_Count--;
    } else {
      Encoder_B.Should_Get_Encoder_Count++;
    }
  } else if ((gpio_interrup & ENCODER_E2B_PIN) == ENCODER_E2B_PIN) {
    if (!DL_GPIO_readPins(ENCODER_PORT, ENCODER_E2A_PIN)) {
      Encoder_B.Should_Get_Encoder_Count++;
    } else {
      Encoder_B.Should_Get_Encoder_Count--;
    }
  }
  DL_GPIO_clearInterruptStatus(ENCODER_PORT, ENCODER_E1A_PIN | ENCODER_E1B_PIN |
                                                 ENCODER_E2A_PIN |
                                                 ENCODER_E2B_PIN);

  // ============================== 分隔线 ==============================
  // 检查是否为按键产生的中断（注意这里假设按键和编码器不在同一个PORT）

  uint32_t key_interrupt =
      DL_GPIO_getEnabledInterruptStatus(GPIO_KEY_PORT, GPIO_KEY_PIN_21_PIN);

  if ((key_interrupt & GPIO_KEY_PIN_21_PIN) == GPIO_KEY_PIN_21_PIN) {
    if (DL_GPIO_readPins(GPIO_KEY_PORT, GPIO_KEY_PIN_21_PIN) > 0) {
      DL_GPIO_togglePins(LED_PORT, LED_PIN_PIN);
      line_flag=1;
    }

    // 清除按键中断标志位
    DL_GPIO_clearInterruptStatus(GPIO_KEY_PORT, GPIO_KEY_PIN_21_PIN);
  }
}

// 电机编码器脉冲计数
void TIMER_0_INST_IRQHandler(void) {
  // Read_Quad();
  // 编码器速度计算
  // if( DL_TimerG_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO )
  // {
  /* 两个电机安装相反，所以编码器值也要相反 */
  Encoder_A.Obtained_Get_Encoder_Count = Encoder_A.Should_Get_Encoder_Count;
  Encoder_B.Obtained_Get_Encoder_Count = -Encoder_B.Should_Get_Encoder_Count;

  /* 编码器计数值清零 */
  Encoder_A.Should_Get_Encoder_Count = 0;
  Encoder_B.Should_Get_Encoder_Count = 0;
  // Read_Quad();
  // }
}
