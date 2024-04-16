/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "MyFunctions.h"

static const uint8_t SHT20_I2C_ADDRESS = 0x40 << 1; // Use 8-bit address
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

uint8_t RxData[10];

int start_ind=0;
int end_ind=-1;
int new_mes=1;
int cur_ind=0;
int newf=0;



volatile uint32_t millis = 0; //Each +1 is 1/10 of a ms



/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void MX_ADC_Init(int);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */


/* Function to read humidity from SHT20 sensor */
float Read_SHT20_Humidity(void) {
    /* Send command to read humidity from SHT20 with hold master mode */
    uint8_t command[] = {0xF5}; // Command to read humidity with hold master mode, 0xE5
    HAL_I2C_Master_Transmit(&hi2c1, SHT20_I2C_ADDRESS, command, 1, HAL_MAX_DELAY);
    /* Wait for sensor to process command */
    delay(200); // Wait for sensor to process command (adjust as needed)

    /* Read humidity data from sensor */
    uint8_t humidity_raw_data[2]; // Buffer to store raw humidity data
    HAL_I2C_Master_Receive(&hi2c1, SHT20_I2C_ADDRESS, humidity_raw_data, 2, HAL_MAX_DELAY);
    /* Process humidity data */
    char buffer[50];
    uint16_t raw_humidity = (humidity_raw_data[0] << 8) | humidity_raw_data[1];
    float humidity_percent = ((125.0 * raw_humidity) / 65536.0) - 6.0;
    sprintf(buffer, "Humidity: %f\r\n", humidity_percent);
    //HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    /* Use humidity_percent as needed */
    return humidity_percent;
}


float Read_SHT20_Temperature(void) {
    /* Send command to read temperature from SHT20 with hold master mode */
    uint8_t command[] = {0xF3}; // Command to read temperature with hold master mode, 0xE3
    HAL_I2C_Master_Transmit(&hi2c1, SHT20_I2C_ADDRESS, command, 1, HAL_MAX_DELAY);

    /* Wait for sensor to process command */
    delay(200); // Wait for sensor to process command (adjust as needed)

    /* Read temperature data from sensor */
    uint8_t temperature_raw_data[2]; // Buffer to store raw temperature data
    HAL_I2C_Master_Receive(&hi2c1, SHT20_I2C_ADDRESS, temperature_raw_data, 2, HAL_MAX_DELAY);
    /* Process temperature data */
	uint16_t val = (temperature_raw_data[0] << 8) | temperature_raw_data[1];
    char buffer[50];
    float temperature_celsius = ((175.72 * val) / 65536.0) - 46.85;
    sprintf(buffer, "Temperature: %f\r\n", temperature_celsius);
    //HAL_UART_Transmit(&huart1, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    /* Use temperature_celsius as needed */
    return temperature_celsius;
}





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

//struct Plant Plant1={1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,GPIOC,GPIO_PIN_9,GPIOC,GPIO_PIN_8,0,1};
//struct Plant Plant2={2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,GPIOC,GPIO_PIN_6,GPIOC,GPIO_PIN_5,4,8};
//struct Plant Plant3={3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,GPIOA,GPIO_PIN_12,GPIOA,GPIO_PIN_11,11,10};
//struct Plant Plant4={4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,GPIOB,GPIO_PIN_12,GPIOB,GPIO_PIN_11,5,6};
//struct Plant Plant5={5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,GPIOB,GPIO_PIN_15,GPIOB,GPIO_PIN_14,7,14};
//struct Plant Plant6={6,0,0,0,0,0,0,0,0,0,0,0,0,0,0,GPIOF,GPIO_PIN_5,GPIOF,GPIO_PIN_4,12,13};
struct Plant myplants[6]={\
		{1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,GPIOC,GPIO_PIN_9,GPIOC,GPIO_PIN_8,0,1},\
		{2,0,0,0,0,0,1,0,0,0,0,0,0,0,0,GPIOC,GPIO_PIN_6,GPIOC,GPIO_PIN_5,4,8},\
		{3,0,0,0,0,0,1,0,0,0,0,0,0,0,0,GPIOA,GPIO_PIN_12,GPIOA,GPIO_PIN_11,11,10},\
		{4,0,0,0,0,0,1,0,0,0,0,0,0,0,0,GPIOB,GPIO_PIN_12,GPIOB,GPIO_PIN_11,5,6},\
		{5,0,0,0,0,0,1,0,0,0,0,0,0,0,0,GPIOB,GPIO_PIN_15,GPIOB,GPIO_PIN_14,7,14},\
		{6,0,0,0,0,0,1,0,0,0,0,0,0,0,0,GPIOF,GPIO_PIN_5,GPIOF,GPIO_PIN_4,12,13}};

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  //MX_ADC_Init(0);
  MX_I2C1_Init();
  //MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM6_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_Base_Start_IT(&htim6);

float temp;
float humidity;
int waterheight[10];

for (int k=0;k<6;k++){
	HAL_GPIO_WritePin(myplants[k].GPIOx_l, myplants[k].GPIO_Pin_l, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(myplants[k].GPIOx_w, myplants[k].GPIO_Pin_w, GPIO_PIN_RESET);
}


  while (1)
  {

//	  uint16_t sense=SensorMeasure(4,10);
//	  sprintf(buffer, "Data to send: %d\r\n", sense);
//
//	  /* Transmit data over UART */
//	  HAL_UART_Transmit(&huart2, (uint16_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
//	  HAL_Delay(2000);


//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
//	  delay(1);
//	  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_12);
//	  delay(100);
//

	  temp=Read_SHT20_Temperature();
	  humidity=Read_SHT20_Humidity();

	  for (int q=0;q<10;q++){
		  WaterHeightMeasure(waterheight);
		  SensorMeasure(temp, humidity,waterheight);
	  //	  HAL_UART_Receive(&huart2, buffer,10,100);
	  //	  delay(2000);
		  //HAL_UART_Transmit(&huart2, RxData,50,100);
		  delay(1000); //30 second delay between measures
	  }


  }
    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC_Init(int chan)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  switch(chan)
  {
  case 0:
	  sConfig.Channel = ADC_CHANNEL_0;
	  break;
  case 1:
  	  sConfig.Channel = ADC_CHANNEL_1;
  	  break;
  case 4:
  	  sConfig.Channel = ADC_CHANNEL_4;
  	  break;
  case 5:
	  sConfig.Channel = ADC_CHANNEL_5;
	  break;
  case 6:
  	  sConfig.Channel = ADC_CHANNEL_6;
  	  break;
  case 7:
  	  sConfig.Channel = ADC_CHANNEL_7;
  	  break;
  case 8:
	  sConfig.Channel = ADC_CHANNEL_8;
	  break;
  case 9:
  	  sConfig.Channel = ADC_CHANNEL_9;
  	  break;
  case 10:
	  sConfig.Channel = ADC_CHANNEL_10;
	  break;
  case 11:
	  sConfig.Channel = ADC_CHANNEL_11;
	  break;
  case 12:
  	  sConfig.Channel = ADC_CHANNEL_12;
  	  break;
  case 13:
  	  sConfig.Channel = ADC_CHANNEL_13;
  	  break;
  case 14:
  	  sConfig.Channel = ADC_CHANNEL_14;
  	  break;
  }

  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }




  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 79; //8 MHZ Clock
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(TIM6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim6) {
    if (htim6->Instance == TIM6) {
        millis=millis+1;
        if (millis==86400000){
			millis=0;
		}

        for (int k=0;k<6;k++){
        	if (myplants[k].active==1){

        		if (myplants[k].mode==0){ //Manual Mode
        			if (myplants[k].light_manual!=0){
						if (millis%10==0){
							HAL_GPIO_WritePin(myplants[k].GPIOx_l, myplants[k].GPIO_Pin_l, GPIO_PIN_SET);
						}
						if (millis%10==myplants[k].light_manual){
							HAL_GPIO_WritePin(myplants[k].GPIOx_l, myplants[k].GPIO_Pin_l, GPIO_PIN_RESET);
						}
					}
        		}
        		else if(myplants[k].mode==1){ //Automated Mode
					if (myplants[k].light_autointensity!=0 && myplants[k].light_autotime>millis){
						if (millis%10==0){
							HAL_GPIO_WritePin(myplants[k].GPIOx_l, myplants[k].GPIO_Pin_l, GPIO_PIN_SET);
							myplants[k].light_turnoff=0;
						}
						if(myplants[k].external_light==0){//No direct sunlight
							if (millis%10==myplants[k].light_autointensity){
								HAL_GPIO_WritePin(myplants[k].GPIOx_l, myplants[k].GPIO_Pin_l, GPIO_PIN_RESET);
							}
						}
						else if (myplants[k].external_light==1){//Direct sunlight
							if (millis%10==(int)((myplants[k].light_autointensity)/2)){
								HAL_GPIO_WritePin(myplants[k].GPIOx_l, myplants[k].GPIO_Pin_l, GPIO_PIN_RESET);
							}
						}
					}
					else if (myplants[k].light_autotime<millis && myplants[k].light_turnoff==0){
						myplants[k].light_turnoff=1;
						HAL_GPIO_WritePin(myplants[k].GPIOx_l, myplants[k].GPIO_Pin_l, GPIO_PIN_RESET);
					}
        		}
        	}
        }

        if (millis%1000==0 && millis!=0){ //Every second, check for watering countdown
        	for (int k=0;k<6;k++){
        		if (myplants[k].countdown_sm!=0){
        			myplants[k].countdown_sm=myplants[k].countdown_sm-1;
        			if (myplants[k].countdown_sm==0){ //Turn off the pump
        				HAL_GPIO_WritePin(myplants[k].GPIOx_w, myplants[k].GPIO_Pin_w, GPIO_PIN_RESET);
        			}
        		}

        	}
        }
        if (millis%2000){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		}
		else if ((millis%2000)%1==0){
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		}

    }
}

static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */

static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
  HAL_UART_Receive_IT(&huart1, RxData, 10);
}

static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  //HAL_UART_Receive_IT(&huart2, RxData, 10);

  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (RxData[cur_ind]!=1){

			if (new_mes==1 &&RxData[cur_ind]==33){ //Start bit of exclamation mark
				start_ind=(cur_ind)%10;
				new_mes=0;
				cur_ind=start_ind;
			}
			if(RxData[cur_ind]==58){ //Change this for interface with ESP32 (58 = colon)
				end_ind=cur_ind;
				start_ind=end_ind;
				for (int k=0;k<10;k++){
					if(RxData[start_ind]!=33){
						if(start_ind==0){
							start_ind=9;
						}
						else{
							start_ind=(start_ind-1);
						}
					}
					else{
						break;
					}
				}
				start_ind=(start_ind+1)%10;
				processUART(RxData,start_ind,end_ind);
				for (int i=0;i<10;i++){ //Clear the buffer again
					RxData[i]=1;
				}
				new_mes=1; //Next character begins a new transmission
			}
			cur_ind=(cur_ind+1)%10;
			RxData[cur_ind]=1;
		}
        HAL_UART_Receive_IT(&huart1, RxData, 10);
    }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB12 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
