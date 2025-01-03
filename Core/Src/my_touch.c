#include "my_touch.h"

/*global variable in the main*/
extern ADC_HandleTypeDef hadc1;
extern ADC_ChannelConfTypeDef sConfig_read;


//int adc_read(uint32_t channel)
//{
//	ADC_ChannelConfTypeDef adc_ch;
//	adc_ch.Channel = channel;
//	adc_ch.Rank = ADC_REGULAR_RANK_1;
//	adc_ch.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
//	HAL_ADC_ConfigChannel(&adc, &adc_ch);
//
//	HAL_ADC_Start(&adc);
//	HAL_ADC_PollForConversion(&adc, 1000);
//    return HAL_ADC_GetValue(&adc);
//}

int TouchRead(float *x, float *y) {
	X_Init();
	HAL_Delay(1);
	*x = Read_axis();
	Y_Init();
	HAL_Delay(1);
	*y = Read_axis();
	if (*x >20.0 || *y > 20.0) {
		return 1;
	}
	return 0;
}

void Touch_Init() {
	/* GPIO Ports Clock Enable */
//	__HAL_RCC_GPIOH_CLK_ENABLE();
//	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

//	/* DMA controller clock enable */
//	 __HAL_RCC_DMAMUX1_CLK_ENABLE();
//	 __HAL_RCC_DMA1_CLK_ENABLE();
//
//	 /* DMA interrupt init */
//	 /* DMA1_Channel1_IRQn interrupt configuration */
//	 HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
//	 HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	 //if you already have the common init in the main, you don't need this part
//	/* ADC common Init */
//	hadc1.Instance = X_Y_ADC_INSTANCE;
//	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
//	hadc1.Init.Resolution = ADC_RESOLUTION_12B; //12bit resolution 0-4095
//	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
//	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
//	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
//	hadc1.Init.LowPowerAutoWait = DISABLE;
//	hadc1.Init.ContinuousConvMode = DISABLE;
//	hadc1.Init.NbrOfConversion = 1;
//	hadc1.Init.DiscontinuousConvMode = DISABLE;
//	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
//	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
//	hadc1.Init.DMAContinuousRequests = DISABLE;
//	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
//	hadc1.Init.OversamplingMode = DISABLE;
//	if (HAL_ADC_Init(&hadc1) != HAL_OK)
//	{
//		Error_Handler1();
//	}


	//sConfig_read.Rank = ADC_REGULAR_RANK_3;
	sConfig_read.Rank = ADC_REGULAR_RANK_1;
	sConfig_read.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
	sConfig_read.SingleDiff = ADC_SINGLE_ENDED;
	sConfig_read.OffsetNumber = ADC_OFFSET_NONE;
	sConfig_read.Offset = 0;


	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
		GPIO_InitStruct.Pin = X_NEG_PIN|Y_NEG_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

float Read_axis() {
	uint32_t tmp[50] = { 0 };
	//fix the numbers of conversions
	uint32_t buf[3] = { 0 };
	//  GPIOA->BRR=GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	 // HAL_Delay(10);
	for (int i = 0; i < 50; i++) {
		//fix the numbers of conversions
//		HAL_ADC_Start_DMA(&hadc1, buf, 3);
//		adc_read
//		HAL_Delay(1);
//		tmp[i] = buf[2];

//		HAL_ADC_Start(&hadc1);
//		HAL_Delay(1);
//		HAL_ADC_PollForConversion(&hadc1, 10);
//		HAL_Delay(1);
//		tmp[i] = HAL_ADC_GetValue(&hadc1);
//		HAL_Delay(1);
//		  HAL_ADC_Stop(&hadc1);



		  HAL_ADC_Start(&hadc1);
		 if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
		 {
			 tmp[i]  = HAL_ADC_GetValue(&hadc1);
			// HAL_Delay(50);
		 }
		 HAL_ADC_Stop(&hadc1);

		//HAL_Delay(1);
	}
	return median(tmp, 10);
}

void X_Init(void) {
	//return;
	//use X-,Y+,Y-
	//HAL_GPIO_DeInit(Y_POS_PORT, Y_POS_PIN);
	//HAL_GPIO_DeInit(Y_NEG_PORT, Y_NEG_PIN);
	//HAL_GPIO_DeInit(X_POS_PORT, X_POS_PIN);
	//HAL_GPIO_DeInit(X_NEG_PORT, X_NEG_PIN);
	MX_GPIO_InitX();
	//return;

	sConfig_read.Channel = X_POS_ADC_CHANNEL;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig_read) != HAL_OK) {
		Error_Handler1();
	}
}

void Y_Init(void) {
	//return;

	//use Y-, X+, X-
	//HAL_GPIO_DeInit(X_POS_PORT, X_POS_PIN);
	//HAL_GPIO_DeInit(X_NEG_PORT, X_NEG_PIN);
	//HAL_GPIO_DeInit(Y_POS_PORT, Y_POS_PIN);
	//HAL_GPIO_DeInit(Y_NEG_PORT, Y_NEG_PIN);
	MX_GPIO_InitY();
	sConfig_read.Channel = Y_POS_ADC_CHANNEL;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig_read) != HAL_OK) {
		Error_Handler1();
	}
}

void MX_GPIO_InitX(void) {
	//return;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };


	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Y_NEG_PORT, Y_NEG_PIN, GPIO_PIN_RESET); //Y-


	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(X_NEG_PORT, X_NEG_PIN, GPIO_PIN_SET); //

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Y_POS_PORT, Y_POS_PIN, GPIO_PIN_SET); //Y+


	//HAL_GPIO_WritePin(Y_POS_PORT, Y_POS_PIN, GPIO_PIN_RESET); //Y+

	GPIO_InitStruct.Pull = GPIO_NOPULL;
////
//////y- 0V
//	GPIO_InitStruct.Pin = Y_NEG_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	//GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(Y_NEG_PORT, &GPIO_InitStruct);
////
//////x- float
//	GPIO_InitStruct.Pin = X_NEG_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	HAL_GPIO_Init(X_NEG_PORT, &GPIO_InitStruct);

	//y+ 3v3
	GPIO_InitStruct.Pin = Y_POS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	//GPIO_InitStruct.Pull = GPIO_PULLUP;
	//GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Y_POS_PORT, &GPIO_InitStruct);
//return;

//	HAL_GPIO_WritePin(Y_POS_PORT, Y_POS_PIN, GPIO_PIN_SET); //Y+

//x+ adc
    GPIO_InitStruct.Pin = X_POS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);

}

void MX_GPIO_InitY(void) {
	//return;
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(X_NEG_PORT, X_NEG_PIN, GPIO_PIN_RESET);


	HAL_GPIO_WritePin(Y_NEG_PORT, Y_NEG_PIN, GPIO_PIN_SET); //Y-

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(X_POS_PORT, X_POS_PIN, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(X_POS_PORT, X_POS_PIN, GPIO_PIN_RESET);

//	/*Configure GPIO pin : PC2 */
//	GPIO_InitStruct.Pin = X_NEG_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//	HAL_GPIO_Init(X_NEG_PORT, &GPIO_InitStruct);
//
//	/*Configure GPIO pin : PC1 */
//	GPIO_InitStruct.Pin = Y_NEG_PIN;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//	HAL_GPIO_Init(Y_NEG_PORT, &GPIO_InitStruct);

	/*Configure GPIO pin : PC0 */
	GPIO_InitStruct.Pin = X_POS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);


//	HAL_GPIO_WritePin(X_POS_PORT, X_POS_PIN, GPIO_PIN_SET);

	//x+ adc
	    GPIO_InitStruct.Pin = Y_POS_PIN;
	    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(X_POS_PORT, &GPIO_InitStruct);

}

void Error_Handler1(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

int compare(const void *a, const void *b) {
	return (*(uint32_t*) a - *(uint32_t*) b);
}

float median(uint32_t arr[], int n) {
	qsort(arr, n, sizeof(uint32_t), compare);
	if (arr[n - 1] - arr[0] > 50)
		return 0;
	if (n % 2 == 0) {
		return ((float) arr[n / 2 - 1] + (float) arr[n / 2]) / 2.0;
	} else {
		return (float) arr[n / 2];
	}
}
