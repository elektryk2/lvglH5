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
#include "stm32h5xx_hal.h"

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
#define K1_Pin GPIO_PIN_0
#define K1_GPIO_Port GPIOC
#define K2_Pin GPIO_PIN_1
#define K2_GPIO_Port GPIOC
#define K3_Pin GPIO_PIN_2
#define K3_GPIO_Port GPIOC
#define K4_Pin GPIO_PIN_3
#define K4_GPIO_Port GPIOC
#define TEST_PIN_Pin GPIO_PIN_2
#define TEST_PIN_GPIO_Port GPIOA
#define VAKU_Pin GPIO_PIN_4
#define VAKU_GPIO_Port GPIOA
#define NPWRDWN_Pin GPIO_PIN_4
#define NPWRDWN_GPIO_Port GPIOC
#define NWAKE_Pin GPIO_PIN_5
#define NWAKE_GPIO_Port GPIOC
#define GREAD_Pin GPIO_PIN_0
#define GREAD_GPIO_Port GPIOB
#define NTRIG_Pin GPIO_PIN_1
#define NTRIG_GPIO_Port GPIOB
#define BUZ_Pin GPIO_PIN_2
#define BUZ_GPIO_Port GPIOB
#define PWR_Pin GPIO_PIN_10
#define PWR_GPIO_Port GPIOB
#define OFFON_Pin GPIO_PIN_12
#define OFFON_GPIO_Port GPIOB
#define DSP_CS_Pin GPIO_PIN_6
#define DSP_CS_GPIO_Port GPIOC
#define DSP_RES_Pin GPIO_PIN_7
#define DSP_RES_GPIO_Port GPIOC
#define DSP_DC_Pin GPIO_PIN_8
#define DSP_DC_GPIO_Port GPIOC
#define DSP_BL_Pin GPIO_PIN_9
#define DSP_BL_GPIO_Port GPIOC
#define W4_Pin GPIO_PIN_8
#define W4_GPIO_Port GPIOA
#define W3_Pin GPIO_PIN_9
#define W3_GPIO_Port GPIOA
#define W2_Pin GPIO_PIN_10
#define W2_GPIO_Port GPIOA
#define W1_Pin GPIO_PIN_15
#define W1_GPIO_Port GPIOA
#define EN3_Pin GPIO_PIN_12
#define EN3_GPIO_Port GPIOC
#define ENW_Pin GPIO_PIN_4
#define ENW_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
