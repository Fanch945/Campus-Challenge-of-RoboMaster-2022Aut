#include "components.h"

/**
  * @brief This function includes all components' initializers.
  */

void Components_Init()
{
	Controller_Init();
	PWM_Init();
	Adc_Init();
	Power_Init();
}
