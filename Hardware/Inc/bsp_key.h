#ifndef __KEY_H
#define __KEY_H

#include "stm32f1xx_hal.h"

// 按键标志
#define KEY1_PRESSED     1   // PB12 - 近光/远光/关闭循环
#define KEY2_PRESSED     2   // PB13 - 左转向灯
#define KEY3_PRESSED     3   // PB14 - 右转向灯
#define KEY4_PRESSED     4   // PB15 - 双闪(危险警示灯)
extern uint8_t KeyNum;

#endif