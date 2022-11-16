#include "bsp_adc.h"
#include "main.h"
#include "os.h"

extern ADC_HandleTypeDef hadc1;
adc_data_t adc_data[6];

/**
  * @brief This function gets value from a certain adc channel.
  */

static void adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
	static ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ch;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;//ADC_SAMPLETIME_1CYCLE_5;
	
	uint16_t cur_data = 0;
	float k = adc_data[ch].filter_k;
	
	if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
	{
			Error_Handler();
	}

	HAL_ADC_Start(ADCx);

	HAL_ADC_PollForConversion(ADCx, 10);
	
	cur_data = (uint16_t)HAL_ADC_GetValue(ADCx);
	
	adc_data[ch].data = k * adc_data[ch].last_data + (1 - k) * cur_data;
	
	adc_data[ch].last_data = cur_data;
}

/**
  * @brief This function gets all 5 adc channels' value.
  */

static void adc1_get_value(void * arg) {
	while(1) {
		adcx_get_chx_value(&hadc1, 0);
		adcx_get_chx_value(&hadc1, 1);
		adcx_get_chx_value(&hadc1, 2);
		adcx_get_chx_value(&hadc1, 3);
		/*!!!NO CHANNEL 4!!!*/
		adcx_get_chx_value(&hadc1, 5);
		delay(2);
	}
}

/**
  * @brief This function initializes and starts adc sampling.
  */

void Adc_Init(void) {
	for(int i = 0; i < 6; ++i) {
		adc_data[i].data = 0;
		adc_data[i].last_data = 0;
		adc_data[i].filter_k = 0.8;
	}
	
	osThreadCreate("ADC", adc1_get_value,  NULL, osPriorityAboveNormal, 64);
}
