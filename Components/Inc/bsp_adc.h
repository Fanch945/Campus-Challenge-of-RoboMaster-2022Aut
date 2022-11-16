#ifndef _COMPONENTS_BSP_ADC_H
#define _COMPONENTS_BSP_ADC_H
#include "main.h"

void Adc_Init(void);

typedef struct {
uint16_t last_data;
uint16_t data;
float filter_k;
}adc_data_t;

extern adc_data_t adc_data[6];

#endif
