#include "bsp_light.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_tim.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim2;

LightState currentLightState = LIGHT_OFF;
static uint8_t turnStep = 0;
static uint32_t lastUpdateTime = 0;
extern uint8_t last_close;

void Bsp_Light_Init(void)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

// 近光灯控制
void Bsp_Light_LowBeam_On(void)
{
    currentLightState = LOW_BEAM_ON;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 10);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 10);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 10);
}

void Bsp_Light_LowBeam_Off(void)
{
    currentLightState = LOW_BEAM_OFF;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

// 远光灯控制
void Bsp_Light_HighBeam_On(void)
{
    currentLightState = HIGH_BEAM_ON;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 1000);
}

void Bsp_Light_HighBeam_Off(void)
{
    currentLightState = HIGH_BEAM_OFF;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
}

// 左转向流水灯控制
void Bsp_Light_LeftTurn_On(void)
{
    currentLightState = LEFT_TURN_ON;
    turnStep = 0;
}

void Bsp_Light_LeftTurn_Off(void)
{
    currentLightState = LIGHT_OFF;
}

// 右转向流水灯控制
void Bsp_Light_RightTurn_On(void)
{
    currentLightState = RIGHT_TURN_ON;
    turnStep = 0;
}

void Bsp_Light_RightTurn_Off(void)
{
    currentLightState = LIGHT_OFF;
}

void Bsp_Light_Hazard_On(void)
{
    currentLightState = Hazard_ON;
    turnStep = 0;
}

void Bsp_Light_Hazard_Off(void)
{
    currentLightState = LIGHT_OFF;
}


// 流水灯更新函数
void Bsp_Light_Update(void)
{
    // 非流水灯状态直接返回，不干扰其他灯光
    if (currentLightState == LIGHT_OFF || 
        currentLightState == LOW_BEAM_OFF || 
        currentLightState == HIGH_BEAM_ON || 
        currentLightState == HIGH_BEAM_OFF ||
        currentLightState == LOW_BEAM_ON)
    {
        if (last_close == 1) {
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
            last_close = 0;
        }
        return;
    }

    uint32_t currentTime = HAL_GetTick();
    if (currentTime - lastUpdateTime < 150) return;

    static const uint32_t channels[4] = {
        TIM_CHANNEL_1,
        TIM_CHANNEL_2,
        TIM_CHANNEL_3,
        TIM_CHANNEL_4
    };

    lastUpdateTime = currentTime;

    for (int i = 0; i < 4; i++)
    {
        __HAL_TIM_SET_COMPARE(&htim2, channels[i], 0);
    }

    if (currentLightState == LEFT_TURN_ON)
    {
        __HAL_TIM_SET_COMPARE(&htim2, channels[2], (turnStep == 0) ? 0 : (turnStep == 1) ? 400 : (turnStep == 2) ? 1000 : 0);
        __HAL_TIM_SET_COMPARE(&htim2, channels[3], (turnStep == 0) ? 0 : (turnStep == 1) ? 0 : (turnStep == 2) ? 400 : 1000);
    }
    else if (currentLightState == RIGHT_TURN_ON)
    {
        __HAL_TIM_SET_COMPARE(&htim2, channels[0], (turnStep == 0) ? 0 : (turnStep == 1) ? 0 : (turnStep == 2) ? 400 : 1000);
        __HAL_TIM_SET_COMPARE(&htim2, channels[1], (turnStep == 0) ? 0 : (turnStep == 1) ? 400 : (turnStep == 2) ? 1000 : 0);
    }
    else if (currentLightState == Hazard_ON)
    {
        __HAL_TIM_SET_COMPARE(&htim2, channels[0], (turnStep == 0) ? 0 : (turnStep == 1) ? 0 : (turnStep == 2) ? 1000 : 1000);
        __HAL_TIM_SET_COMPARE(&htim2, channels[1], (turnStep == 0) ? 0 : (turnStep == 1) ? 0 : (turnStep == 2) ? 1000 : 1000);
        __HAL_TIM_SET_COMPARE(&htim2, channels[2], (turnStep == 0) ? 0 : (turnStep == 1) ? 0 : (turnStep == 2) ? 1000 : 1000);
        __HAL_TIM_SET_COMPARE(&htim2, channels[3], (turnStep == 0) ? 0 : (turnStep == 1) ? 0 : (turnStep == 2) ? 1000 : 1000);
    }
    turnStep = (turnStep + 1) % 4;  // 循环4步
}

void Bsp_AFS_Update(int32_t angle)
{
    if(currentLightState == LOW_BEAM_ON)
    {
        if(angle > 0)
         {
            uint16_t ch3 = (uint16_t)((angle * 1000) / 2160);
            uint16_t ch4 = (uint16_t)((angle * 1000) / 720);

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 10);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, ch3);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, ch4);
        }
        else if(angle < 0)
        {
            uint16_t ch1 = (uint16_t)((angle * 1000) / -720);
            uint16_t ch2 = (uint16_t)((angle * 1000) / -2160);

            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ch1);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, ch2);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 10);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 10);
        }
        else if(angle == 0){
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 10);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 10);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 10);
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 10);
        }   
    }
    else
    {
        return;
    }
}