#ifndef _COMPONENTS_OS_H
#define _COMPONENTS_OS_H

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"
#include "cmsis_armcc.h"

#define OS_TOTAL_STACK_SIZE 2048
#define OS_MAX_TASK_NUMBER 15
#define OS_MAX_EVENT_NUMBER 3

extern volatile uint32_t uwTick;

#define time() (uwTick)

osThreadId_t osThreadCreate(char const *name, osThreadFunc_t func, void *arg, osPriority_t priority, uint32_t stack_size);
osEventFlagsId_t osEventFlagsCreate(char const *name);

/**
  * @brief This function creates a universal delay function whether the osKernel is running.
  */

inline static void delay(uint32_t ms) {
  if (__get_IPSR() || osKernelGetState() != osKernelRunning) {
    HAL_Delay(ms);
  } else {
    osDelay(ms);
  }
}

/**
  * @brief This function creates a universal delay function until certain tick.
  */

inline static void delayUntil(uint32_t ticks) {
  if (__get_IPSR() || osKernelGetState() != osKernelRunning) {
    while (uwTick < ticks) ;
  } else {
    osDelayUntil(ticks);
  }
}

#endif
