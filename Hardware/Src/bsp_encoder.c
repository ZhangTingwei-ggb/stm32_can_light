#include "bsp_encoder.h"
#include "stm32f1xx_hal_tim.h"
#include <stdint.h>

extern TIM_HandleTypeDef htim3;

void Bsp_Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

int32_t Bsp_Encoder_GetCount(void)
{
    if(__HAL_TIM_GET_COUNTER(&htim3) == 0)
    {
        return 0;
    }
    else if(__HAL_TIM_GET_COUNTER(&htim3) > 0 && __HAL_TIM_GET_COUNTER(&htim3) <= 80) 
    {
        return __HAL_TIM_GET_COUNTER(&htim3)*9; 
    }
    else if (__HAL_TIM_GET_COUNTER(&htim3) > 80 && __HAL_TIM_GET_COUNTER(&htim3) <= 32767)
    {
        return 720;
    }
    else if (__HAL_TIM_GET_COUNTER(&htim3) > 32767 && __HAL_TIM_GET_COUNTER(&htim3) <= 65455)
    {
        return -720; 
    }
    else if (__HAL_TIM_GET_COUNTER(&htim3) > 65455 && __HAL_TIM_GET_COUNTER(&htim3) <= 65535)
    {
        return (__HAL_TIM_GET_COUNTER(&htim3)-65536)*9; 
    }
    
}