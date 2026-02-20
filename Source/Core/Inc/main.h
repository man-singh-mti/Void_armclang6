/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f7xx_hal.h"

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
#define IMU1_INT_Pin         GPIO_PIN_4
#define IMU1_INT_GPIO_Port   GPIOA
#define IMU1_INT_EXTI_IRQn   EXTI4_IRQn
#define ADC_TEMP_Pin         GPIO_PIN_5
#define ADC_TEMP_GPIO_Port   GPIOA
#define ADC_WATER2_Pin       GPIO_PIN_6
#define ADC_WATER2_GPIO_Port GPIOA
#define ADC_WATER1_Pin       GPIO_PIN_1
#define ADC_WATER1_GPIO_Port GPIOB
#define PWR_IMU_Pin          GPIO_PIN_2
#define PWR_IMU_GPIO_Port    GPIOB
#define VD_2_SCL_Pin         GPIO_PIN_10
#define VD_2_SCL_GPIO_Port   GPIOB
#define VD_2_SDA_Pin         GPIO_PIN_11
#define VD_2_SDA_GPIO_Port   GPIOB
#define IMU1_nCS_Pin         GPIO_PIN_12
#define IMU1_nCS_GPIO_Port   GPIOB
#define IMU1_SCLK_Pin        GPIO_PIN_13
#define IMU1_SCLK_GPIO_Port  GPIOB
#define IMU1_MISO_Pin        GPIO_PIN_14
#define IMU1_MISO_GPIO_Port  GPIOB
#define IMU1_MOSI_Pin        GPIO_PIN_15
#define IMU1_MOSI_GPIO_Port  GPIOB
#define DEBUG_TXD_Pin        GPIO_PIN_6
#define DEBUG_TXD_GPIO_Port  GPIOC
#define DEBUG_RXD_Pin        GPIO_PIN_7
#define DEBUG_RXD_GPIO_Port  GPIOC
#define VOID_FAULT_Pin       GPIO_PIN_9
#define VOID_FAULT_GPIO_Port GPIOA
#define VOID_S_Pin           GPIO_PIN_10
#define VOID_S_GPIO_Port     GPIOA
#define VOID_RXD_Pin         GPIO_PIN_11
#define VOID_RXD_GPIO_Port   GPIOA
#define VOID_TXD_Pin         GPIO_PIN_12
#define VOID_TXD_GPIO_Port   GPIOA
#define IMU2_nCS_Pin         GPIO_PIN_15
#define IMU2_nCS_GPIO_Port   GPIOA
#define SP_TXD_Pin           GPIO_PIN_10
#define SP_TXD_GPIO_Port     GPIOC
#define SP_RXD_Pin           GPIO_PIN_11
#define SP_RXD_GPIO_Port     GPIOC
#define IMU2_INT_Pin         GPIO_PIN_2
#define IMU2_INT_GPIO_Port   GPIOD
#define IMU2_INT_EXTI_IRQn   EXTI2_IRQn
#define IMU2_SCLK_Pin        GPIO_PIN_3
#define IMU2_SCLK_GPIO_Port  GPIOB
#define IMU2_MISO_Pin        GPIO_PIN_4
#define IMU2_MISO_GPIO_Port  GPIOB
#define IMU2_MOSI_Pin        GPIO_PIN_5
#define IMU2_MOSI_GPIO_Port  GPIOB
#define UPHOLE_TXD_Pin       GPIO_PIN_6
#define UPHOLE_TXD_GPIO_Port GPIOB
#define UPHOLE_RXD_Pin       GPIO_PIN_7
#define UPHOLE_RXD_GPIO_Port GPIOB
#define VD_1_SCL_Pin         GPIO_PIN_8
#define VD_1_SCL_GPIO_Port   GPIOB
#define VD_1_SDA_Pin         GPIO_PIN_9
#define VD_1_SDA_GPIO_Port   GPIOB

/* USER CODE BEGIN Private defines */
#define UPHOLE_GPIO_Port GPIOB
#define DEBUG_GPIO_Port  GPIOC
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
