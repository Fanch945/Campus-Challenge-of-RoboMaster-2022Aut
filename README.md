# RM2023MBF

A Framework for Magician STN32F103C8T6 Board. 2023

**IMPORTANT: READ THIS DOCUMENT BEFORE USING THIS DEVELOPING FRAMEWORK!**

## Usage:

Write thread functions containing a dead loop.

Put any thread you want in user_main.c.

Compile & Flash

## Components:

### OS

A simplified layer of FreeRTOS's thread. Out of the real-usage concern, this layer only gives thread creation method. By 

>osThreadId_t osThreadCreate(char const *name, osThreadFunc_t func, void *arg, osPriority_t priority, uint32_t stackSize)

where name can be whatever you want, as long as they are not the same as each other, func can be any void type function that has a dead loop within, priority can be selected from enum osPriority_t, and stackSize can be any number on 2's power(large than 32 is suggested).

### Controller

No functions should be used outside of this file. Users may get controller's data by global struct "control". Structure of control are the same as what in C Board.

### PWM

The most often used function of PWM should be

>HAL_StatusTypeDef PWM_SetDutyRatio(PWM * pwm, float ratio)

and 

>HAL_StatusTypeDef PWM_SetFrequency(PWM * pwm, uint32_t frequency)

These two functions are used to set a pwm channel's duty ratio and frequency.

### ADC

No functions should be used outside of this file. Users may get adc_channels' data by global struct "adc_data[ch]". Structure of adc_data[ch] is the same as what in C Board.

### GPIO_OUT

We didn't implement custom function to operate GPIO_OUT. You can simply use 

>void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState)

to operate a pin if you want.