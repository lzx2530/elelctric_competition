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


#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_MOTOR_LEFT */
#define PWM_MOTOR_LEFT_INST                                               TIMG12
#define PWM_MOTOR_LEFT_INST_IRQHandler                         TIMG12_IRQHandler
#define PWM_MOTOR_LEFT_INST_INT_IRQN                           (TIMG12_INT_IRQn)
#define PWM_MOTOR_LEFT_INST_CLK_FREQ                                    32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_LEFT_C0_PORT                                        GPIOB
#define GPIO_PWM_MOTOR_LEFT_C0_PIN                                DL_GPIO_PIN_13
#define GPIO_PWM_MOTOR_LEFT_C0_IOMUX                             (IOMUX_PINCM30)
#define GPIO_PWM_MOTOR_LEFT_C0_IOMUX_FUNC            IOMUX_PINCM30_PF_TIMG12_CCP0
#define GPIO_PWM_MOTOR_LEFT_C0_IDX                           DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_LEFT_C1_PORT                                        GPIOA
#define GPIO_PWM_MOTOR_LEFT_C1_PIN                                DL_GPIO_PIN_31
#define GPIO_PWM_MOTOR_LEFT_C1_IOMUX                              (IOMUX_PINCM6)
#define GPIO_PWM_MOTOR_LEFT_C1_IOMUX_FUNC             IOMUX_PINCM6_PF_TIMG12_CCP1
#define GPIO_PWM_MOTOR_LEFT_C1_IDX                           DL_TIMER_CC_1_INDEX

/* Defines for PWM_MOTOR_RIGHT */
#define PWM_MOTOR_RIGHT_INST                                               TIMG7
#define PWM_MOTOR_RIGHT_INST_IRQHandler                         TIMG7_IRQHandler
#define PWM_MOTOR_RIGHT_INST_INT_IRQN                           (TIMG7_INT_IRQn)
#define PWM_MOTOR_RIGHT_INST_CLK_FREQ                                   32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_RIGHT_C0_PORT                                       GPIOA
#define GPIO_PWM_MOTOR_RIGHT_C0_PIN                               DL_GPIO_PIN_28
#define GPIO_PWM_MOTOR_RIGHT_C0_IOMUX                             (IOMUX_PINCM3)
#define GPIO_PWM_MOTOR_RIGHT_C0_IOMUX_FUNC              IOMUX_PINCM3_PF_TIMG7_CCP0
#define GPIO_PWM_MOTOR_RIGHT_C0_IDX                          DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_RIGHT_C1_PORT                                       GPIOB
#define GPIO_PWM_MOTOR_RIGHT_C1_PIN                               DL_GPIO_PIN_19
#define GPIO_PWM_MOTOR_RIGHT_C1_IOMUX                            (IOMUX_PINCM45)
#define GPIO_PWM_MOTOR_RIGHT_C1_IOMUX_FUNC             IOMUX_PINCM45_PF_TIMG7_CCP1
#define GPIO_PWM_MOTOR_RIGHT_C1_IDX                          DL_TIMER_CC_1_INDEX

/* Defines for PWM_STEP_YAW */
#define PWM_STEP_YAW_INST                                                  TIMA0
#define PWM_STEP_YAW_INST_IRQHandler                            TIMA0_IRQHandler
#define PWM_STEP_YAW_INST_INT_IRQN                              (TIMA0_INT_IRQn)
#define PWM_STEP_YAW_INST_CLK_FREQ                                      32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_STEP_YAW_C0_PORT                                          GPIOA
#define GPIO_PWM_STEP_YAW_C0_PIN                                   DL_GPIO_PIN_8
#define GPIO_PWM_STEP_YAW_C0_IOMUX                               (IOMUX_PINCM19)
#define GPIO_PWM_STEP_YAW_C0_IOMUX_FUNC              IOMUX_PINCM19_PF_TIMA0_CCP0
#define GPIO_PWM_STEP_YAW_C0_IDX                             DL_TIMER_CC_0_INDEX

/* Defines for PWM_STEP_PITCH */
#define PWM_STEP_PITCH_INST                                                TIMG0
#define PWM_STEP_PITCH_INST_IRQHandler                          TIMG0_IRQHandler
#define PWM_STEP_PITCH_INST_INT_IRQN                            (TIMG0_INT_IRQn)
#define PWM_STEP_PITCH_INST_CLK_FREQ                                    32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_STEP_PITCH_C0_PORT                                        GPIOA
#define GPIO_PWM_STEP_PITCH_C0_PIN                                DL_GPIO_PIN_12
#define GPIO_PWM_STEP_PITCH_C0_IOMUX                             (IOMUX_PINCM34)
#define GPIO_PWM_STEP_PITCH_C0_IOMUX_FUNC             IOMUX_PINCM34_PF_TIMG0_CCP0
#define GPIO_PWM_STEP_PITCH_C0_IDX                           DL_TIMER_CC_0_INDEX



/* Defines for TIMER_CTRL_1KHZ */
#define TIMER_CTRL_1KHZ_INST                                             (TIMA1)
#define TIMER_CTRL_1KHZ_INST_IRQHandler                         TIMA1_IRQHandler
#define TIMER_CTRL_1KHZ_INST_INT_IRQN                           (TIMA1_INT_IRQn)
#define TIMER_CTRL_1KHZ_INST_LOAD_VALUE                                 (31999U)




/* Defines for I2C_SENSOR_BUS */
#define I2C_SENSOR_BUS_INST                                                 I2C1
#define I2C_SENSOR_BUS_INST_IRQHandler                           I2C1_IRQHandler
#define I2C_SENSOR_BUS_INST_INT_IRQN                               I2C1_INT_IRQn
#define I2C_SENSOR_BUS_BUS_SPEED_HZ                                       100000
#define GPIO_I2C_SENSOR_BUS_SDA_PORT                                       GPIOB
#define GPIO_I2C_SENSOR_BUS_SDA_PIN                                DL_GPIO_PIN_3
#define GPIO_I2C_SENSOR_BUS_IOMUX_SDA                            (IOMUX_PINCM16)
#define GPIO_I2C_SENSOR_BUS_IOMUX_SDA_FUNC               IOMUX_PINCM16_PF_I2C1_SDA
#define GPIO_I2C_SENSOR_BUS_SCL_PORT                                       GPIOB
#define GPIO_I2C_SENSOR_BUS_SCL_PIN                                DL_GPIO_PIN_2
#define GPIO_I2C_SENSOR_BUS_IOMUX_SCL                            (IOMUX_PINCM15)
#define GPIO_I2C_SENSOR_BUS_IOMUX_SCL_FUNC               IOMUX_PINCM15_PF_I2C1_SCL


/* Defines for UART_DEBUG */
#define UART_DEBUG_INST                                                    UART0
#define UART_DEBUG_INST_FREQUENCY                                       32000000
#define UART_DEBUG_INST_IRQHandler                              UART0_IRQHandler
#define UART_DEBUG_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_UART_DEBUG_RX_PORT                                            GPIOA
#define GPIO_UART_DEBUG_TX_PORT                                            GPIOA
#define GPIO_UART_DEBUG_RX_PIN                                    DL_GPIO_PIN_11
#define GPIO_UART_DEBUG_TX_PIN                                    DL_GPIO_PIN_10
#define GPIO_UART_DEBUG_IOMUX_RX                                 (IOMUX_PINCM22)
#define GPIO_UART_DEBUG_IOMUX_TX                                 (IOMUX_PINCM21)
#define GPIO_UART_DEBUG_IOMUX_RX_FUNC                  IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_DEBUG_IOMUX_TX_FUNC                  IOMUX_PINCM21_PF_UART0_TX
#define UART_DEBUG_BAUD_RATE                                            (115200)
#define UART_DEBUG_IBRD_32_MHZ_115200_BAUD                                  (17)
#define UART_DEBUG_FBRD_32_MHZ_115200_BAUD                                  (23)
/* Defines for UART_K230 */
#define UART_K230_INST                                                     UART1
#define UART_K230_INST_FREQUENCY                                        32000000
#define UART_K230_INST_IRQHandler                               UART1_IRQHandler
#define UART_K230_INST_INT_IRQN                                   UART1_INT_IRQn
#define GPIO_UART_K230_RX_PORT                                             GPIOA
#define GPIO_UART_K230_TX_PORT                                             GPIOA
#define GPIO_UART_K230_RX_PIN                                     DL_GPIO_PIN_18
#define GPIO_UART_K230_TX_PIN                                     DL_GPIO_PIN_17
#define GPIO_UART_K230_IOMUX_RX                                  (IOMUX_PINCM40)
#define GPIO_UART_K230_IOMUX_TX                                  (IOMUX_PINCM39)
#define GPIO_UART_K230_IOMUX_RX_FUNC                   IOMUX_PINCM40_PF_UART1_RX
#define GPIO_UART_K230_IOMUX_TX_FUNC                   IOMUX_PINCM39_PF_UART1_TX
#define UART_K230_BAUD_RATE                                             (115200)
#define UART_K230_IBRD_32_MHZ_115200_BAUD                                   (17)
#define UART_K230_FBRD_32_MHZ_115200_BAUD                                   (23)





/* Port definition for Pin Group GPIO_LED */
#define GPIO_LED_PORT                                                    (GPIOB)

/* Defines for LED_STATUS: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_LED_LED_STATUS_PIN                                 (DL_GPIO_PIN_26)
#define GPIO_LED_LED_STATUS_IOMUX                                (IOMUX_PINCM57)
/* Port definition for Pin Group GPIO_MOTOR */
#define GPIO_MOTOR_PORT                                                  (GPIOB)

/* Defines for DIR_LEFT: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_MOTOR_DIR_LEFT_PIN                                  (DL_GPIO_PIN_0)
#define GPIO_MOTOR_DIR_LEFT_IOMUX                                (IOMUX_PINCM12)
/* Defines for DIR_RIGHT: GPIOB.6 with pinCMx 23 on package pin 58 */
#define GPIO_MOTOR_DIR_RIGHT_PIN                                 (DL_GPIO_PIN_6)
#define GPIO_MOTOR_DIR_RIGHT_IOMUX                               (IOMUX_PINCM23)
/* Port definition for Pin Group GPIO_TURRET */
#define GPIO_TURRET_PORT                                                 (GPIOB)

/* Defines for DIR_YAW: GPIOB.12 with pinCMx 29 on package pin 64 */
#define GPIO_TURRET_DIR_YAW_PIN                                 (DL_GPIO_PIN_12)
#define GPIO_TURRET_DIR_YAW_IOMUX                                (IOMUX_PINCM29)
/* Defines for DIR_PITCH: GPIOB.15 with pinCMx 32 on package pin 3 */
#define GPIO_TURRET_DIR_PITCH_PIN                               (DL_GPIO_PIN_15)
#define GPIO_TURRET_DIR_PITCH_IOMUX                              (IOMUX_PINCM32)
/* Defines for LASER_EN: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_TURRET_LASER_EN_PIN                                (DL_GPIO_PIN_16)
#define GPIO_TURRET_LASER_EN_IOMUX                               (IOMUX_PINCM33)
/* Defines for BUZZER: GPIOB.5 with pinCMx 18 on package pin 53 */
#define GPIO_TURRET_BUZZER_PIN                                   (DL_GPIO_PIN_5)
#define GPIO_TURRET_BUZZER_IOMUX                                 (IOMUX_PINCM18)
/* Defines for LINE0: GPIOA.15 with pinCMx 37 on package pin 8 */
#define GPIO_LINE_LINE0_PORT                                             (GPIOA)
#define GPIO_LINE_LINE0_PIN                                     (DL_GPIO_PIN_15)
#define GPIO_LINE_LINE0_IOMUX                                    (IOMUX_PINCM37)
/* Defines for LINE1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_LINE_LINE1_PORT                                             (GPIOA)
#define GPIO_LINE_LINE1_PIN                                     (DL_GPIO_PIN_16)
#define GPIO_LINE_LINE1_IOMUX                                    (IOMUX_PINCM38)
/* Defines for LINE2: GPIOA.2 with pinCMx 7 on package pin 42 */
#define GPIO_LINE_LINE2_PORT                                             (GPIOA)
#define GPIO_LINE_LINE2_PIN                                      (DL_GPIO_PIN_2)
#define GPIO_LINE_LINE2_IOMUX                                     (IOMUX_PINCM7)
/* Defines for LINE3: GPIOA.3 with pinCMx 8 on package pin 43 */
#define GPIO_LINE_LINE3_PORT                                             (GPIOA)
#define GPIO_LINE_LINE3_PIN                                      (DL_GPIO_PIN_3)
#define GPIO_LINE_LINE3_IOMUX                                     (IOMUX_PINCM8)
/* Defines for LINE4: GPIOA.21 with pinCMx 46 on package pin 17 */
#define GPIO_LINE_LINE4_PORT                                             (GPIOA)
#define GPIO_LINE_LINE4_PIN                                     (DL_GPIO_PIN_21)
#define GPIO_LINE_LINE4_IOMUX                                    (IOMUX_PINCM46)
/* Defines for LINE5: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_LINE_LINE5_PORT                                             (GPIOA)
#define GPIO_LINE_LINE5_PIN                                     (DL_GPIO_PIN_24)
#define GPIO_LINE_LINE5_IOMUX                                    (IOMUX_PINCM54)
/* Defines for LINE6: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_LINE_LINE6_PORT                                             (GPIOA)
#define GPIO_LINE_LINE6_PIN                                     (DL_GPIO_PIN_25)
#define GPIO_LINE_LINE6_IOMUX                                    (IOMUX_PINCM55)
/* Defines for LINE7: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_LINE_LINE7_PORT                                             (GPIOB)
#define GPIO_LINE_LINE7_PIN                                     (DL_GPIO_PIN_21)
#define GPIO_LINE_LINE7_IOMUX                                    (IOMUX_PINCM49)
/* Port definition for Pin Group GPIO_ENCODER */
#define GPIO_ENCODER_PORT                                                (GPIOB)

/* Defines for ENC_L_B: GPIOB.1 with pinCMx 13 on package pin 48 */
// pins affected by this interrupt request:["ENC_L_B","ENC_L_A","ENC_R_A","ENC_R_B"]
#define GPIO_ENCODER_INT_IRQN                                   (GPIOB_INT_IRQn)
#define GPIO_ENCODER_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODER_ENC_L_B_IIDX                            (DL_GPIO_IIDX_DIO1)
#define GPIO_ENCODER_ENC_L_B_PIN                                 (DL_GPIO_PIN_1)
#define GPIO_ENCODER_ENC_L_B_IOMUX                               (IOMUX_PINCM13)
/* Defines for ENC_L_A: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_ENCODER_ENC_L_A_IIDX                           (DL_GPIO_IIDX_DIO20)
#define GPIO_ENCODER_ENC_L_A_PIN                                (DL_GPIO_PIN_20)
#define GPIO_ENCODER_ENC_L_A_IOMUX                               (IOMUX_PINCM48)
/* Defines for ENC_R_A: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_ENCODER_ENC_R_A_IIDX                           (DL_GPIO_IIDX_DIO24)
#define GPIO_ENCODER_ENC_R_A_PIN                                (DL_GPIO_PIN_24)
#define GPIO_ENCODER_ENC_R_A_IOMUX                               (IOMUX_PINCM52)
/* Defines for ENC_R_B: GPIOB.4 with pinCMx 17 on package pin 52 */
#define GPIO_ENCODER_ENC_R_B_IIDX                            (DL_GPIO_IIDX_DIO4)
#define GPIO_ENCODER_ENC_R_B_PIN                                 (DL_GPIO_PIN_4)
#define GPIO_ENCODER_ENC_R_B_IOMUX                               (IOMUX_PINCM17)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_LEFT_init(void);
void SYSCFG_DL_PWM_MOTOR_RIGHT_init(void);
void SYSCFG_DL_PWM_STEP_YAW_init(void);
void SYSCFG_DL_PWM_STEP_PITCH_init(void);
void SYSCFG_DL_TIMER_CTRL_1KHZ_init(void);
void SYSCFG_DL_I2C_SENSOR_BUS_init(void);
void SYSCFG_DL_UART_DEBUG_init(void);
void SYSCFG_DL_UART_K230_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
