/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOTOR_LF_Pin GPIO_PIN_0
#define MOTOR_LF_GPIO_Port GPIOF
#define MOTOR_LB_Pin GPIO_PIN_1
#define MOTOR_LB_GPIO_Port GPIOF
#define MOTOR_RF_Pin GPIO_PIN_2
#define MOTOR_RF_GPIO_Port GPIOF
#define MOTOR_RB_Pin GPIO_PIN_3
#define MOTOR_RB_GPIO_Port GPIOF
#define SERVO_PWM1_Pin GPIO_PIN_6
#define SERVO_PWM1_GPIO_Port GPIOF
#define SERVO_PWM2_Pin GPIO_PIN_7
#define SERVO_PWM2_GPIO_Port GPIOF
#define BAT_ADC_Pin GPIO_PIN_2
#define BAT_ADC_GPIO_Port GPIOA
#define SW_D_Pin GPIO_PIN_3
#define SW_D_GPIO_Port GPIOA
#define SW_B_Pin GPIO_PIN_4
#define SW_B_GPIO_Port GPIOA
#define SW_C_Pin GPIO_PIN_5
#define SW_C_GPIO_Port GPIOA
#define SW_CEN_Pin GPIO_PIN_6
#define SW_CEN_GPIO_Port GPIOA
#define SW_A_Pin GPIO_PIN_7
#define SW_A_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_14
#define BEEP_GPIO_Port GPIOF
#define OLED_CS_Pin GPIO_PIN_7
#define OLED_CS_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define OLED_RST_Pin GPIO_PIN_9
#define OLED_RST_GPIO_Port GPIOE
#define OLED_SCLK_Pin GPIO_PIN_12
#define OLED_SCLK_GPIO_Port GPIOE
#define OLED_SDI_Pin GPIO_PIN_14
#define OLED_SDI_GPIO_Port GPIOE
#define SERVO_TX1_Pin GPIO_PIN_10
#define SERVO_TX1_GPIO_Port GPIOB
#define SERVO_RX1_Pin GPIO_PIN_11
#define SERVO_RX1_GPIO_Port GPIOB
#define SERVO_RX2_Pin GPIO_PIN_12
#define SERVO_RX2_GPIO_Port GPIOB
#define SERVO_TX2_Pin GPIO_PIN_13
#define SERVO_TX2_GPIO_Port GPIOB
#define SERVO_TX3_Pin GPIO_PIN_14
#define SERVO_TX3_GPIO_Port GPIOB
#define SERVO_RX3_Pin GPIO_PIN_15
#define SERVO_RX3_GPIO_Port GPIOB
#define MAG_SWITCH_Pin GPIO_PIN_6
#define MAG_SWITCH_GPIO_Port GPIOC
#define ICM_SDA_Pin GPIO_PIN_9
#define ICM_SDA_GPIO_Port GPIOC
#define ICM_SCK_Pin GPIO_PIN_8
#define ICM_SCK_GPIO_Port GPIOA
#define AUX_GP1_Pin GPIO_PIN_1
#define AUX_GP1_GPIO_Port GPIOD
#define AUX_GP2_Pin GPIO_PIN_2
#define AUX_GP2_GPIO_Port GPIOD
#define AUX_CTS_Pin GPIO_PIN_3
#define AUX_CTS_GPIO_Port GPIOD
#define AUX_RTS_Pin GPIO_PIN_4
#define AUX_RTS_GPIO_Port GPIOD
#define AUX_TX_Pin GPIO_PIN_5
#define AUX_TX_GPIO_Port GPIOD
#define AUX_RX_Pin GPIO_PIN_6
#define AUX_RX_GPIO_Port GPIOD
#define AUX_MOSI_Pin GPIO_PIN_7
#define AUX_MOSI_GPIO_Port GPIOD
#define AUX_MISO_Pin GPIO_PIN_9
#define AUX_MISO_GPIO_Port GPIOG
#define AUX_NSS_Pin GPIO_PIN_10
#define AUX_NSS_GPIO_Port GPIOG
#define AUX_SCK_Pin GPIO_PIN_11
#define AUX_SCK_GPIO_Port GPIOG
#define AUX_GP1G12_Pin GPIO_PIN_12
#define AUX_GP1G12_GPIO_Port GPIOG
#define AUX_GP0_Pin GPIO_PIN_13
#define AUX_GP0_GPIO_Port GPIOG
#define ENC1_B_Pin GPIO_PIN_4
#define ENC1_B_GPIO_Port GPIOB
#define ENC1_A_Pin GPIO_PIN_5
#define ENC1_A_GPIO_Port GPIOB
#define ENC2_B_Pin GPIO_PIN_6
#define ENC2_B_GPIO_Port GPIOB
#define ENC2_A_Pin GPIO_PIN_7
#define ENC2_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
