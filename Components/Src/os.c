#include "os.h"
#include "FreeRTOS.h"

static uint32_t stack[OS_TOTAL_STACK_SIZE];
static StaticTask_t task[OS_MAX_TASK_NUMBER];
static StaticEventGroup_t event[OS_MAX_EVENT_NUMBER];

static uint16_t stack_c = 0;
static uint8_t task_c = 0;
static uint8_t event_c = 0;

/**
  * @brief This function handles the creation of threads.
  */

osThreadId_t osThreadCreate(char const *name, osThreadFunc_t func, void *arg, osPriority_t priority, uint32_t stackSize) {
  if (name == NULL || func == NULL) {
    return NULL;
  }
  if (stack_c + stackSize > OS_TOTAL_STACK_SIZE) {
    /* no enough global stack space, halt */
    while (1);
  }
  if (task_c == OS_MAX_TASK_NUMBER) {
    /* max task number reached, halt */
    while (1);
  }
  osThreadAttr_t attr = {
    .name = name,
    .cb_mem = task + task_c,
    .cb_size = sizeof(task[0]),
    .stack_mem = stack + stack_c,
    .stack_size = stackSize * 4,
    .priority = priority
  };
  stack_c += stackSize;
  ++task_c;
  
  return osThreadNew(func, arg, &attr);
}

/**
  * @brief This function handles the creation of event flags.
  */

osEventFlagsId_t osEventFlagsCreate(char const *name) {
  if (name == NULL) {
    return NULL;
  }
  if (event_c == OS_MAX_EVENT_NUMBER) {
    /* max event group number reached, halt */
    while (1);
  }
  osEventFlagsAttr_t attr = {
    .name = name,
    .cb_mem = event + event_c,
    .cb_size = sizeof(event[0])
  };
  ++event_c;
  
  return osEventFlagsNew(&attr);
}
