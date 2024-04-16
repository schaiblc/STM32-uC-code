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
#include "stm32f0xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

struct Plant {
    int plantid; //Plant number
	int moisture_lim; //These are the parameters that come from the app. This is minimum moisture
	int light_autotime; //Auto mode time range for ON (or modified ON) time
	int light_autointensity; //PWM intensity for auto mode (DIFFERENT than manual)
	int external_light; //Flag for whether direct sunlight was present in past 5 minutes
    int light_manual; //This is only for manual mode (intensity from 0, 25%, 50%, 75%, 100%)
    int mode; //Manual or automated
    int active; //Actively monitored or deactivated pot
    int sum_sm; //Running sum of measurement values from moisture sensor
    int sum_l; //Running sum of measurement values from light sensor
    int num_measure; //Tally of number of measurements in this batch so far
    int cur_sm; //The most recent full batch of data for moisture sensor data
    int cur_l; //The most recent full batch of data for light sensor data
    int countdown_sm; // Countdown for shutting off the pump after starting
    int light_turnoff; //LED turnoff flag once timer expires for the day

    GPIO_TypeDef* GPIOx_w; //Water Control GPIO Output Port
    uint16_t GPIO_Pin_w; //Water Control GPIO Output Pin
    GPIO_TypeDef* GPIOx_l; //Light Control GPIO Output Port
    uint16_t GPIO_Pin_l; //Light Control GPIO Output Pin
    int chan_sm; //Soil Moisture ADC Channel
    int chan_l; //Light ADC Channel
};

extern struct Plant Plant1;
extern struct Plant Plant2;
extern struct Plant Plant3;
extern struct Plant Plant4;
extern struct Plant Plant5;
extern struct Plant Plant6;
extern struct Plant myplants[6];

extern volatile uint32_t millis;

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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

