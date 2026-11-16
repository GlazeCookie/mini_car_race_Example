/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_RUN_Pin GPIO_PIN_14
#define GPIO_RUN_GPIO_Port GPIOB
#define L_DIR_Pin GPIO_PIN_15
#define L_DIR_GPIO_Port GPIOB
#define L_PWM_Pin GPIO_PIN_8
#define L_PWM_GPIO_Port GPIOA
#define R_PWM_Pin GPIO_PIN_9
#define R_PWM_GPIO_Port GPIOA
#define R_DIR_Pin GPIO_PIN_10
#define R_DIR_GPIO_Port GPIOA
#define MUX_READ_Pin GPIO_PIN_12
#define MUX_READ_GPIO_Port GPIOA
#define MUX_0_Pin GPIO_PIN_15
#define MUX_0_GPIO_Port GPIOA
#define MUX_1_Pin GPIO_PIN_3
#define MUX_1_GPIO_Port GPIOB
#define L_ENCODER_B_Pin GPIO_PIN_4
#define L_ENCODER_B_GPIO_Port GPIOB
#define L_ENCODER_A_Pin GPIO_PIN_5
#define L_ENCODER_A_GPIO_Port GPIOB
#define R_ENCODER_B_Pin GPIO_PIN_6
#define R_ENCODER_B_GPIO_Port GPIOB
#define R_ENCODER_A_Pin GPIO_PIN_7
#define R_ENCODER_A_GPIO_Port GPIOB
#define MUX_2_Pin GPIO_PIN_8
#define MUX_2_GPIO_Port GPIOB
#define MUX_3_Pin GPIO_PIN_9
#define MUX_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* 故障时关闭 TIM1 两路电机输出；只在定时器时钟开启时访问外设。 */
#define MOTOR_EMERGENCY_STOP() do { \
  if ((RCC->APB2ENR & RCC_APB2ENR_TIM1EN) != 0U) { \
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E); \
    TIM1->BDTR &= ~TIM_BDTR_MOE; \
    TIM1->CCR1 = 0U; \
    TIM1->CCR2 = 0U; \
  } \
} while (0)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
