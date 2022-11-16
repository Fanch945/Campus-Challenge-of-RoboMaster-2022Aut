#ifndef _COMPONENTS_CONTROLLER_H 
#define _COMPONENTS_CONTROLLER_H

#include <stdint.h>

#define RC_BUF_SIZE 40

/******************************************************************************/
/*                                                                            */
/*                         				Keyborad Keys		                            */
/*                                                                            */
/******************************************************************************/
/******************  Bit definition for  Keys ********************/
#define KEY_W      (control.key & 0x0001)
#define KEY_S      (control.key & 0x0002)
#define KEY_A      (control.key & 0x0004)
#define KEY_D      (control.key & 0x0008)
#define KEY_SHIFT  (control.key & 0x0010)
#define KEY_CTRL   (control.key & 0x0020)
#define KEY_Q      (control.key & 0x0040)
#define KEY_E      (control.key & 0x0080)
#define KEY_R      (control.key & 0x0100)
#define KEY_F      (control.key & 0x0200)
#define KEY_G      (control.key & 0x0400)
#define KEY_Z      (control.key & 0x0800)
#define KEY_X      (control.key & 0x1000)
#define KEY_C      (control.key & 0x2000)
#define KEY_V      (control.key & 0x4000)
#define KEY_B      (control.key & 0x8000)
/******************************************************************/
#define ROLLER1_MAX 0x94

/** 
  * @brief Mice Data 
  */

typedef struct _Mice {
  int16_t  x;             // [-32768, 32767] mouse VELOCITY of X axis, positive is rightward
  int16_t  y;             // [-32768, 32767] mouse VELOCITY of Y axis
  int16_t  z; 						// [-32768, 32767] mouse VELOCITY of Z axis (INOP ON CONTROLLER)
  uint8_t  leftButton;		
  uint8_t  rightButton;
} Mice;

/** 
  * @brief Remote Control Data 
  */

typedef struct _RemoteControlData {
	/*For specific definition, please refer to https://cloud.rmshtech.com/share/vDa40NAC Page 6~7*/
  float     channel[4];    // (-1, 1)    centered channel pos in percentage
  Mice      mice;
  uint16_t  key;
  uint8_t   triSwitch[2];  // 
  uint8_t   online;
	uint8_t   roller1,roller2;
} ControlData;

extern volatile uint8_t rc_buf[RC_BUF_SIZE];
extern volatile uint32_t _lastRCPacketTime;
extern ControlData control;

void Controller_Init(void);
void notifyRCDataUnpacker(void);
#endif

