/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c (STM32H743ZGT VCM + HRTIM)
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

void HAL_HRTIM_MspPostInit(HRTIM_HandleTypeDef *hhrtim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PROCESS_LED_Pin GPIO_PIN_3
#define PROCESS_LED_GPIO_Port GPIOE
#define ADC1_IFB_P_Pin GPIO_PIN_0
#define ADC1_IFB_P_GPIO_Port GPIOA
#define ADC1_IFB_N_Pin GPIO_PIN_1
#define ADC1_IFB_N_GPIO_Port GPIOA
#define ADC2_IREF_P_Pin GPIO_PIN_6
#define ADC2_IREF_P_GPIO_Port GPIOA
#define ADC2_IREF_N_Pin GPIO_PIN_7
#define ADC2_IREF_N_GPIO_Port GPIOA
#define DRV_EN_Pin GPIO_PIN_0
#define DRV_EN_GPIO_Port GPIOB
#define DRV_EN_EXTI_IRQn EXTI0_IRQn
#define FAULT_Pin GPIO_PIN_1
#define FAULT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
