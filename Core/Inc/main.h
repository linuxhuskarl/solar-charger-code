/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef enum {LeadAcid, LeadGel, LiPoly} batt_type;
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
void save_config_eeprom(void);
void load_config_eeprom(void);
uint16_t ADC_voltage(uint16_t);
uint16_t ADC_current(uint16_t);
uint16_t ADC_current_load(uint16_t);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_DIS_Pin GPIO_PIN_13
#define CS_DIS_GPIO_Port GPIOC
#define LOAD_EN_Pin GPIO_PIN_14
#define LOAD_EN_GPIO_Port GPIOC
#define SHDN_LCD_Pin GPIO_PIN_0
#define SHDN_LCD_GPIO_Port GPIOC
#define ADC_BATT_V_Pin GPIO_PIN_1
#define ADC_BATT_V_GPIO_Port GPIOC
#define SHDN_HSCA_Pin GPIO_PIN_2
#define SHDN_HSCA_GPIO_Port GPIOC
#define SHDN_12V_Pin GPIO_PIN_3
#define SHDN_12V_GPIO_Port GPIOC
#define ADC_SOLAR_I_Pin GPIO_PIN_0
#define ADC_SOLAR_I_GPIO_Port GPIOA
#define ADC_BATT_I_Pin GPIO_PIN_1
#define ADC_BATT_I_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define ADC_LOAD_I_Pin GPIO_PIN_4
#define ADC_LOAD_I_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LCD_EN_Pin GPIO_PIN_4
#define LCD_EN_GPIO_Port GPIOC
#define ADC_SOLAR_V_Pin GPIO_PIN_0
#define ADC_SOLAR_V_GPIO_Port GPIOB
#define PWM_BUCK_Pin GPIO_PIN_10
#define PWM_BUCK_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_13
#define LCD_RS_GPIO_Port GPIOB
#define LCD_RW_Pin GPIO_PIN_14
#define LCD_RW_GPIO_Port GPIOB
#define BTN_OK_Pin GPIO_PIN_7
#define BTN_OK_GPIO_Port GPIOC
#define BTN_UP_Pin GPIO_PIN_8
#define BTN_UP_GPIO_Port GPIOA
#define BTN_DOWN_Pin GPIO_PIN_9
#define BTN_DOWN_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_10
#define LCD_D7_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define LCD_D6_Pin GPIO_PIN_3
#define LCD_D6_GPIO_Port GPIOB
#define LCD_D4_Pin GPIO_PIN_4
#define LCD_D4_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_5
#define LCD_D5_GPIO_Port GPIOB
#define BTN_BACK_Pin GPIO_PIN_6
#define BTN_BACK_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define BTN_COUNT 4
#define LCD_DELAY 100
#define LCD_MODE_COUNT 4
#define LCD_MODE_SOLAR 0
#define LCD_MODE_BATT 1
#define LCD_MODE_LOAD 2
#define LCD_MODE_MENU (LCD_MODE_COUNT-1)
#define LCD_MENU_COUNT 3
#define BATTERY_TYPE_EEPROM_ADDR 0x0000
#define BATTERY_CAPACITY_EEPROM_ADDR 0x0004

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
