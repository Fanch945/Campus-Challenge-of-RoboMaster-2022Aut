#include "controller.h"
#include "os.h"
#include "main.h"
#include "cmsis_os2.h"

extern UART_HandleTypeDef huart1;

ControlData control = {0};
volatile uint32_t _lastRCPacketTime = 0;
volatile uint8_t rc_buf[RC_BUF_SIZE];

/*Thread ID and Flag setting*/
osThreadId_t unpackerId;
#define RC_DATA_PENDING 0x0001
/*********End setting********/

/**
  * @brief This function filtering the value within certain range.
  */

inline static void avoid_deadzone_float(float* value, float range) {
  if (-range < *value && *value < range) *value = 0;
}

/**
  * @brief This function handles the unpacking of the controller data.
  */

void unpackData(uint8_t volatile *data) {
  control.channel[0] = ((uint16_t)(((data[0] | (data[1] << 8))) & 0x7ff) - 1024) / 660.0f;
  control.channel[1] = ((uint16_t)(((data[1] >> 3) | (data[2] << 5)) & 0x7ff) - 1024) / 660.0f;
  control.channel[2] = ((uint16_t)(((data[2] >> 6) | (data[3] << 2) | data[4] << 10) & 0x7ff) - 1024) / 660.0f;
  control.channel[3] = ((uint16_t)(((data[4] >> 1) | (data[5] << 7)) & 0x7ff) - 1024) / 660.0f;
  avoid_deadzone_float(&control.channel[0], 0.05f);
  avoid_deadzone_float(&control.channel[1], 0.05f);
  avoid_deadzone_float(&control.channel[2], 0.05f);
  avoid_deadzone_float(&control.channel[3], 0.05f);
  control.triSwitch[0] = (data[5] >> 6) & 0x3;
  control.triSwitch[1] = (data[5] >> 4) & 0x3;
  control.mice.x = data[6] | (data[7] << 8);
  control.mice.y = data[8] | (data[9] << 8);
  control.mice.z = data[10] | (data[11] << 8);
  control.mice.leftButton = data[12];
  control.mice.rightButton = data[13];
  control.key = data[14] | (data[15] << 8);
	control.roller1 = data[16];
	control.roller2 = data[17];
}


/**
  * @brief This function sets thread flag as RC_DATA_PENDING.
  */

void notifyRCDataUnpacker() {
  osThreadFlagsSet(unpackerId, RC_DATA_PENDING);
}

/**
  * @brief This function starts the UART idle interrupt and DMA receive.
  */

inline static void Controller_Start_Rx() {
  __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart1, (uint8_t*) rc_buf, RC_BUF_SIZE);
}

/**
  * @brief This function is the main function of controller.
  */

void RCDataUnpacker(void *_) {
  uint32_t _lastRCUnpackTime = 0;
  Controller_Start_Rx();
  while (1) {
    control.online = (_lastRCPacketTime > _lastRCUnpackTime);
    if (control.online) {
      _lastRCUnpackTime = HAL_GetTick();
				unpackData(rc_buf);
		}
    /* the manual says frame interval is 14ms */
    osThreadFlagsWait(RC_DATA_PENDING, osFlagsWaitAny, 100);
	}
}

/**
  * @brief This function starts a new thread for controller.
  */

void Controller_Init() {
  unpackerId = osThreadCreate("RC", RCDataUnpacker, NULL, osPriorityAboveNormal, 64);
}




