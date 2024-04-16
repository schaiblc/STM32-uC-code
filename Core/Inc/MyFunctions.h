#ifndef MY_FUNCTIONS_H
#define MY_FUNCTIONS_H

#include "stm32f0xx_hal.h"
#include "main.h"

// Function prototypes
void SensorMeasure(float temp, float humidity, int waterheight[10]);
void delay(uint32_t milliseconds);
void WaterHeightMeasure(int waterheight[10]);
void processUART(uint8_t RxData[10], int start_ind, int end_ind);

#endif /* MY_FUNCTIONS_H */
