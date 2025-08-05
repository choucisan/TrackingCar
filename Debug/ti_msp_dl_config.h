/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     40000000



/* Defines for PWM */
#define PWM_INST                                                           TIMA1
#define PWM_INST_IRQHandler                                     TIMA1_IRQHandler
#define PWM_INST_INT_IRQN                                       (TIMA1_INT_IRQn)
#define PWM_INST_CLK_FREQ                                                5000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_C0_PORT                                                   GPIOA
#define GPIO_PWM_C0_PIN                                           DL_GPIO_PIN_17
#define GPIO_PWM_C0_IOMUX                                        (IOMUX_PINCM39)
#define GPIO_PWM_C0_IOMUX_FUNC                       IOMUX_PINCM39_PF_TIMA1_CCP0
#define GPIO_PWM_C0_IDX                                      DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_C1_PORT                                                   GPIOA
#define GPIO_PWM_C1_PIN                                           DL_GPIO_PIN_16
#define GPIO_PWM_C1_IOMUX                                        (IOMUX_PINCM38)
#define GPIO_PWM_C1_IOMUX_FUNC                       IOMUX_PINCM38_PF_TIMA1_CCP1
#define GPIO_PWM_C1_IDX                                      DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                           (194U)
/* Defines for TIMER_1 */
#define TIMER_1_INST                                                     (TIMA0)
#define TIMER_1_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_1_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_1_INST_LOAD_VALUE                                           (390U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C1
#define I2C_OLED_INST_IRQHandler                                 I2C1_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C1_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOB
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_3
#define GPIO_I2C_OLED_IOMUX_SDA                                  (IOMUX_PINCM16)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                   IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOB
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_2
#define GPIO_I2C_OLED_IOMUX_SCL                                  (IOMUX_PINCM15)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                   IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           20000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_20_MHZ_115200_BAUD                                      (10)
#define UART_0_FBRD_20_MHZ_115200_BAUD                                      (54)





/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC1
#define ADC12_0_INST_IRQHandler                                  ADC1_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC1_INT_IRQn)
#define ADC12_0_ADCMEM_0                                      DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_0_REF                     DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_0_ADCMEM_0_REF_VOLTAGE_V                                       3.3
#define GPIO_ADC12_0_C0_PORT                                               GPIOA
#define GPIO_ADC12_0_C0_PIN                                       DL_GPIO_PIN_15



/* Port definition for Pin Group LED */
#define LED_PORT                                                         (GPIOB)

/* Defines for PIN: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED_PIN_PIN                                             (DL_GPIO_PIN_22)
#define LED_PIN_IOMUX                                            (IOMUX_PINCM50)
/* Port definition for Pin Group GPIO_KEY */
#define GPIO_KEY_PORT                                                    (GPIOB)

/* Defines for PIN_21: GPIOB.21 with pinCMx 49 on package pin 20 */
// pins affected by this interrupt request:["PIN_21"]
#define GPIO_KEY_INT_IRQN                                       (GPIOB_INT_IRQn)
#define GPIO_KEY_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_KEY_PIN_21_IIDX                                (DL_GPIO_IIDX_DIO21)
#define GPIO_KEY_PIN_21_PIN                                     (DL_GPIO_PIN_21)
#define GPIO_KEY_PIN_21_IOMUX                                    (IOMUX_PINCM49)
/* Port definition for Pin Group IIC_Software */
#define IIC_Software_PORT                                                (GPIOA)

/* Defines for SCL: GPIOA.1 with pinCMx 2 on package pin 34 */
#define IIC_Software_SCL_PIN                                     (DL_GPIO_PIN_1)
#define IIC_Software_SCL_IOMUX                                    (IOMUX_PINCM2)
/* Defines for SDA: GPIOA.0 with pinCMx 1 on package pin 33 */
#define IIC_Software_SDA_PIN                                     (DL_GPIO_PIN_0)
#define IIC_Software_SDA_IOMUX                                    (IOMUX_PINCM1)
/* Port definition for Pin Group TB6612 */
#define TB6612_PORT                                                      (GPIOA)

/* Defines for AIN1: GPIOA.7 with pinCMx 14 on package pin 49 */
#define TB6612_AIN1_PIN                                          (DL_GPIO_PIN_7)
#define TB6612_AIN1_IOMUX                                        (IOMUX_PINCM14)
/* Defines for AIN2: GPIOA.8 with pinCMx 19 on package pin 54 */
#define TB6612_AIN2_PIN                                          (DL_GPIO_PIN_8)
#define TB6612_AIN2_IOMUX                                        (IOMUX_PINCM19)
/* Defines for BIN1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define TB6612_BIN1_PIN                                         (DL_GPIO_PIN_12)
#define TB6612_BIN1_IOMUX                                        (IOMUX_PINCM34)
/* Defines for BIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define TB6612_BIN2_PIN                                         (DL_GPIO_PIN_13)
#define TB6612_BIN2_IOMUX                                        (IOMUX_PINCM35)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOA)

/* Defines for E1A: GPIOA.26 with pinCMx 59 on package pin 30 */
// pins affected by this interrupt request:["E1A","E1B","E2A","E2B"]
#define ENCODER_INT_IRQN                                        (GPIOA_INT_IRQn)
#define ENCODER_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODER_E1A_IIDX                                    (DL_GPIO_IIDX_DIO26)
#define ENCODER_E1A_PIN                                         (DL_GPIO_PIN_26)
#define ENCODER_E1A_IOMUX                                        (IOMUX_PINCM59)
/* Defines for E1B: GPIOA.27 with pinCMx 60 on package pin 31 */
#define ENCODER_E1B_IIDX                                    (DL_GPIO_IIDX_DIO27)
#define ENCODER_E1B_PIN                                         (DL_GPIO_PIN_27)
#define ENCODER_E1B_IOMUX                                        (IOMUX_PINCM60)
/* Defines for E2A: GPIOA.24 with pinCMx 54 on package pin 25 */
#define ENCODER_E2A_IIDX                                    (DL_GPIO_IIDX_DIO24)
#define ENCODER_E2A_PIN                                         (DL_GPIO_PIN_24)
#define ENCODER_E2A_IOMUX                                        (IOMUX_PINCM54)
/* Defines for E2B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define ENCODER_E2B_IIDX                                    (DL_GPIO_IIDX_DIO25)
#define ENCODER_E2B_PIN                                         (DL_GPIO_PIN_25)
#define ENCODER_E2B_IOMUX                                        (IOMUX_PINCM55)
/* Port definition for Pin Group Gray_Address */
#define Gray_Address_PORT                                                (GPIOB)

/* Defines for PIN_0: GPIOB.13 with pinCMx 30 on package pin 1 */
#define Gray_Address_PIN_0_PIN                                  (DL_GPIO_PIN_13)
#define Gray_Address_PIN_0_IOMUX                                 (IOMUX_PINCM30)
/* Defines for PIN_1: GPIOB.14 with pinCMx 31 on package pin 2 */
#define Gray_Address_PIN_1_PIN                                  (DL_GPIO_PIN_14)
#define Gray_Address_PIN_1_IOMUX                                 (IOMUX_PINCM31)
/* Defines for PIN_2: GPIOB.15 with pinCMx 32 on package pin 3 */
#define Gray_Address_PIN_2_PIN                                  (DL_GPIO_PIN_15)
#define Gray_Address_PIN_2_IOMUX                                 (IOMUX_PINCM32)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_SYSCTL_CLK_init(void);
void SYSCFG_DL_PWM_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_TIMER_1_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_ADC12_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
