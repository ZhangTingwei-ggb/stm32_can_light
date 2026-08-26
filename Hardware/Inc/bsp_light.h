#ifndef __BSP_LIGHT_H
#define __BSP_LIGHT_H

#include "stm32f1xx_hal.h"

// 灯状态枚举
typedef enum {
    LOW_BEAM_ON,
    LOW_BEAM_OFF,
    HIGH_BEAM_ON,
    HIGH_BEAM_OFF,
    LIGHT_OFF,
    LEFT_TURN_ON,
    RIGHT_TURN_ON,
    Hazard_ON
} LightState;

void Bsp_Light_Init(void);

// 近光灯控制
void Bsp_Light_LowBeam_On(void);
void Bsp_Light_LowBeam_Off(void);

// 远光灯控制
void Bsp_Light_HighBeam_On(void);
void Bsp_Light_HighBeam_Off(void);

// 左转向流水灯控制
void Bsp_Light_LeftTurn_On(void);
void Bsp_Light_LeftTurn_Off(void);

// 右转向流水灯控制
void Bsp_Light_RightTurn_On(void);
void Bsp_Light_RightTurn_Off(void);

// 危险警示灯控制
void Bsp_Light_Hazard_On(void);
void Bsp_Light_Hazard_Off(void);

void Bsp_Light_Update(void);

void Bsp_AFS_Update(int32_t angle);

#endif /* __BSP_LIGHT_H */