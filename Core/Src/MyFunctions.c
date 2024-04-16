#include "MyFunctions.h"

void SensorMeasure(float temp, float humidity, int waterheight[10]){ //chan -> ADC CHANNEL, numval -> # 1/2 sec measures
	//uint8_t tx_buffer[27]="Welcome to BinaryUpdates!\n\r";
	//HAL_UART_Transmit(&huart1, (uint16_t *)tx_buffer, 27, HAL_MAX_DELAY);
	for (int i=0;i<6;i++){
		if (myplants[i].active==1){ //MUST BE 1
			MX_ADC_Init(myplants[i].chan_sm);
			if (HAL_ADC_Start(&hadc) != HAL_OK) {
				  Error_Handler();
			  }

			  /* Wait for ADC conversion to complete */
			  if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) != HAL_OK) {
				  Error_Handler();
			  }

			  /* Read ADC value */
			  myplants[i].sum_sm = myplants[i].sum_sm+HAL_ADC_GetValue(&hadc);
			  HAL_ADC_DeInit(&hadc);


			MX_ADC_Init(myplants[i].chan_l);
			if (HAL_ADC_Start(&hadc) != HAL_OK) {
				  Error_Handler();
			  }

			  /* Wait for ADC conversion to complete */
			  if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) != HAL_OK) {
				  Error_Handler();
			  }

			  /* Read ADC value */
			  myplants[i].sum_l = myplants[i].sum_l+HAL_ADC_GetValue(&hadc);
			  HAL_ADC_DeInit(&hadc);
			myplants[i].num_measure=myplants[i].num_measure+1;
			if (myplants[i].num_measure==10){
				myplants[i].num_measure=0;
				myplants[i].cur_sm=myplants[i].sum_sm;
				myplants[i].cur_l=myplants[i].sum_l;

				int wh=0;
				for (int y=0;y<10;y++){
					wh=wh+waterheight[y];
				}

				if (wh>10000){ //Whatever the threshold is for 1/2 full level
					wh=0;
				}
				else{
					wh=1; //Water not detected, send alert that low water level
				}

				uint8_t buffer[50]; //Add slashes to string here for ESP32 receiving
				sprintf(buffer, "::%d,%.2f,%.2f,%d,%d,%d!!",myplants[i].plantid,temp,humidity,myplants[i].sum_sm,myplants[i].sum_l,wh);
				HAL_UART_Transmit(&huart1, (uint16_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

				if (myplants[i].active==1 && myplants[i].mode==1 && myplants[i].cur_sm>myplants[i].moisture_lim){
					HAL_GPIO_WritePin(myplants[i].GPIOx_w, myplants[i].GPIO_Pin_w, GPIO_PIN_SET);
					myplants[i].countdown_sm=15;
				}
				if (myplants[i].active==1 && myplants[i].mode==1 && myplants[i].cur_l>20000){
					myplants[i].external_light=1;
				}
				else if (myplants[i].active==1 && myplants[i].mode==1 && myplants[i].cur_l<=20000){
					myplants[i].external_light=0;
				}
				myplants[i].sum_sm=0;
				myplants[i].sum_l=0;

				//ADC GOES UP TO 4095
				//TRANSMIT UART TO ESP32 FOR THAT POT NOW

			}
		}

	}

}

// Non-blocking delay function
void delay(uint32_t milliseconds) {
    uint32_t start_time = millis;
    if (start_time+milliseconds>86400000){
    	while(millis< (start_time+milliseconds)%86400000 || millis>start_time){

    	}
    }
    else{
		while ((millis - start_time) < milliseconds) {
			// Wait for delay to elapse
		}
    }

}

void WaterHeightMeasure(int waterheight[10]){
	for (int c=0;c<9;c++){
		waterheight[c]=waterheight[c+1];
	}
	MX_ADC_Init(9);
	if (HAL_ADC_Start(&hadc) != HAL_OK) {
		  Error_Handler();
	  }

	  /* Wait for ADC conversion to complete */
	  if (HAL_ADC_PollForConversion(&hadc, HAL_MAX_DELAY) != HAL_OK) {
		  Error_Handler();
	  }

	  /* Read ADC value */
	  waterheight[9]=HAL_ADC_GetValue(&hadc);
	  HAL_ADC_DeInit(&hadc);
}

void processUART(uint8_t RxData[10], int start_ind, int end_ind){
	int plant_num;
	switch(RxData[start_ind]){ //PlantID Switch Case
	case (int)'1':
			plant_num=0; //Plant =Pot1
		break;
	case (int)'2':
		plant_num=1; //Plant =Pot2
		break;
	case (int)'3':
		plant_num=2; //Plant =Pot3
		break;
	case (int)'4':
		plant_num=3; //Plant =Pot4
		break;
	case (int)'5':
		plant_num=4; //Plant =Pot5
		break;
	case (int)'6':
		plant_num=5; //Plant =Pot6
		break;
	}

	switch(RxData[(start_ind+1)%10]){ //Command Switch Case
	case (int)'1':
		//Change parameters
		int flag1=0;
		int flag2=0;
		switch(RxData[(start_ind+2)%10]){ //Moisture parameter first
		case (int)'0':
			myplants[plant_num].moisture_lim=50000;
			flag1=1;
			break;
		case (int)'1':
			myplants[plant_num].moisture_lim=22000; //These parameters may need to be changed based on testing
			flag1=1;
			break;
		case (int)'2':
			myplants[plant_num].moisture_lim=19000;
			flag1=1;
			break;
		case (int)'3':
			myplants[plant_num].moisture_lim=16000;
			flag1=1;
			break;
		}
		switch(RxData[(start_ind+3)%10]){ //Lighting parameter next
		case (int)'0':
			myplants[plant_num].light_autointensity=0;
			myplants[plant_num].light_autotime=0;
			HAL_GPIO_WritePin(myplants[plant_num].GPIOx_l, myplants[plant_num].GPIO_Pin_l, GPIO_PIN_RESET);
			flag2=1;
			break;
		case (int)'1':
			myplants[plant_num].light_autointensity=5; //These parameters may need to be changed based on testing
			myplants[plant_num].light_autotime=28800000; //8 hours
			flag2=1;
			break;
		case (int)'2':
			myplants[plant_num].light_autointensity=7;
			myplants[plant_num].light_autotime=39600000; //11 hours
			flag2=1;
			break;
		case (int)'3':
			myplants[plant_num].light_autointensity=10;
			myplants[plant_num].light_autotime=50400000; //14 hours
			flag2=1;
			break;
		}
		if (flag1==1 && flag2==1){
			myplants[plant_num].active=1; //This pot is (re)activated
		}
		break;
	case (int)'2':
		//Delete Pot
		myplants[plant_num].active=0; //Deactivate that pot
		HAL_GPIO_WritePin(myplants[plant_num].GPIOx_l, myplants[plant_num].GPIO_Pin_l, GPIO_PIN_RESET);
		break;
	case (int)'3':
		//Toggle Auto/Manual
			switch (RxData[(start_ind+2)%10]){
			case (int)'0': //Automated Mode
					if (myplants[plant_num].active==1){
						myplants[plant_num].mode=1;
						HAL_GPIO_WritePin(myplants[plant_num].GPIOx_l, myplants[plant_num].GPIO_Pin_l, GPIO_PIN_RESET);
					}
					//Other automated functions to be done elsewhere, parameters kept
				break;
			case (int)'1': //Manual Mode
					if (myplants[plant_num].active==1){
						myplants[plant_num].mode=0;
						HAL_GPIO_WritePin(myplants[plant_num].GPIOx_l, myplants[plant_num].GPIO_Pin_l, GPIO_PIN_RESET);
					}
				break;
			}
		break;
	case (int)'4':
		//Manual Only - Run Pump
			if (myplants[plant_num].mode==0 &&myplants[plant_num].active==1){ //If in Manual Mode
				HAL_GPIO_WritePin(myplants[plant_num].GPIOx_w, myplants[plant_num].GPIO_Pin_w, GPIO_PIN_SET);
				myplants[plant_num].countdown_sm=6; //Max of 6 seconds pumping water
			}
		break;
	case (int)'5':
		//Manual Only - Set Lighting Level
		if (myplants[plant_num].mode==0&&myplants[plant_num].active==1){ //If in Manual Mode
			switch(RxData[(start_ind+2)%10]){
			case (int)'0':
					myplants[plant_num].light_manual=0;
					HAL_GPIO_WritePin(myplants[plant_num].GPIOx_l, myplants[plant_num].GPIO_Pin_l, GPIO_PIN_RESET);
				break;
			case (int)'1':
					myplants[plant_num].light_manual=2;
				break;
			case (int)'2':
					myplants[plant_num].light_manual=5;
				break;
			case (int)'3':
					myplants[plant_num].light_manual=7;
				break;
			case (int)'4':
					myplants[plant_num].light_manual=10;
				break;
			}

		}

		break;
	}
}







