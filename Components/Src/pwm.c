#include "pwm.h"

/**
  * @brief This function includes general defination for all 8 PWM channels.
  */

PWM  pwm1[PWM_MAX_NUMBER] = {		
	[0] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_1,
		.enabled     = 1
	},
	[1] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_1,
		.enabled     = 1
	},
	[2] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_2,
		.enabled     = 1
	},
	[3] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_3,
		.enabled     = 1
	},
	[4] = {
		.port        = &htim1,
		.channel     = TIM_CHANNEL_4,
		.enabled     = 1
	}
};

PWM  pwm2[PWM_MAX_NUMBER] = {		
	[0] = {
		.port        = &htim2,
		.channel     = TIM_CHANNEL_1,
		.enabled     = 1
	},
	[1] = {
		.port        = &htim2,
		.channel     = TIM_CHANNEL_1,
		.enabled     = 1
	},
	[2] = {
		.port        = &htim2,
		.channel     = TIM_CHANNEL_2,
		.enabled     = 1
	},
	[3] = {
		.port        = &htim2,
		.channel     = TIM_CHANNEL_3,
		.enabled     = 1
	},
	[4] = {
		.port        = &htim2,
		.channel     = TIM_CHANNEL_4,
		.enabled     = 1
	}
};

/**
  * @brief This function defines the end of PWM channels array.
  */

void * pwm1End = pwm1 + PWM_MAX_NUMBER;

void * pwm2End = pwm2 + PWM_MAX_NUMBER;

/**
  * @brief This function initializes all 8 PWM channels.
  */

HAL_StatusTypeDef PWM_Init(void) {
	for (PWM* pwm = pwm1; (void*) pwm != pwm1End; ++pwm) {
		if (pwm->enabled) { 
			PWM_Start(pwm);
			uint32_t pluse;
			switch (pwm->channel) {
			  case TIM_CHANNEL_1:
				  pluse = pwm->port->Instance->CCR1;
				  break;
			  case TIM_CHANNEL_2:
				  pluse = pwm->port->Instance->CCR2;
				  break;
				case TIM_CHANNEL_3:
				  pluse = pwm->port->Instance->CCR3;
				  break;
				case TIM_CHANNEL_4:
				  pluse = pwm->port->Instance->CCR4;
				  break;
		  }
			pwm->frequency = PWM_DEFAULT_FREQUENCY;
			pwm->dutyRatio = (float) pluse / (pwm->port->Instance->ARR+1);
		}
	}
	for (PWM* pwm = pwm2; (void*) pwm != pwm2End; ++pwm) {
		if (pwm->enabled) { 
			PWM_Start(pwm);
			uint32_t pluse; 
			switch (pwm->channel) {
			  case TIM_CHANNEL_1:
				  pluse = pwm->port->Instance->CCR1;
				  break;
			  case TIM_CHANNEL_2:
				  pluse = pwm->port->Instance->CCR2;
				  break;
				case TIM_CHANNEL_3:
				  pluse = pwm->port->Instance->CCR3;
				  break;
				case TIM_CHANNEL_4:
				  pluse = pwm->port->Instance->CCR4;
				  break;
		  }
			pwm->frequency = PWM_DEFAULT_FREQUENCY;
			pwm->dutyRatio = (float) pluse / (pwm->port->Instance->ARR+1);
		}
	}
	return HAL_OK;
}

/**
  * @brief This function deinitializes all 8 PWM channels.
  */

HAL_StatusTypeDef PWM_DeInit(void) {
	return HAL_OK;
}

/**
  * @brief This function starts a PWM channel.
  */

HAL_StatusTypeDef PWM_Start(const PWM * pwm) {
	return HAL_TIM_PWM_Start(pwm->port, pwm->channel);
}

/**
  * @brief This function stops a PWM channel.
  */

HAL_StatusTypeDef PWM_Stop(const PWM * pwm) {
	return HAL_TIM_PWM_Stop(pwm->port, pwm->channel);
}

/**
  * @brief This function sets a PWM channel's Auto Reload Register.
  */

inline static HAL_StatusTypeDef PWM_SetARR(const PWM * pwm, uint32_t value) {
	pwm->port->Instance->ARR = value;
	return HAL_OK;
}

/**
  * @brief This function sets a PWM channel's pulse.
  */

inline HAL_StatusTypeDef PWM_SetPulse(const PWM * pwm, uint32_t value) {
	if (pwm->channel == TIM_CHANNEL_1) pwm->port->Instance->CCR1 = value;
	if (pwm->channel == TIM_CHANNEL_2) pwm->port->Instance->CCR2 = value;
	if (pwm->channel == TIM_CHANNEL_3) pwm->port->Instance->CCR3 = value;
	if (pwm->channel == TIM_CHANNEL_4) pwm->port->Instance->CCR4 = value;
	return HAL_OK;
}

/**
  * @brief This function sets a PWM channel's duty ratio.
  */

HAL_StatusTypeDef PWM_SetDutyRatio(PWM * pwm, float ratio) {
	if (ratio < 0 || ratio > 1) return HAL_ERROR;
	pwm->dutyRatio = ratio;
	PWM_SetPulse(pwm, ratio * (pwm->port->Instance->ARR+1));
	return HAL_OK;
}

/**
  * @brief This function sets a PWM channel's fequency.
  */

HAL_StatusTypeDef PWM_SetFrequency(PWM * pwm, uint32_t frequency) {
	PWM_SetARR(pwm, PWM_DEFAULT_FREQUENCY * PWM_DEFAULT_ARR / frequency - 1);
	pwm->frequency = frequency;
	return HAL_OK;
}

/**
  * @brief This function sets a PWM channel's PulseLength.
  */

HAL_StatusTypeDef PWM_SetPulseLength(PWM * pwm, uint32_t us) {
	return PWM_SetDutyRatio(pwm, us * 1e-6 * pwm->frequency);
}

