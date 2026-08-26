#include "bsp_key.h"
#include "stm32f103xb.h"
#include "stm32f1xx_hal_gpio.h"

extern uint8_t KeyNum = 0;
static uint32_t lastKeyTick = 0;
static const uint32_t keyDebounceMs = 300;

// HAL 外部中断统一回调
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t tick = HAL_GetTick();
    if (tick - lastKeyTick < keyDebounceMs)
    {
        return;
    }
    lastKeyTick = tick;

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    
    if(GPIO_Pin == GPIO_PIN_12)
        KeyNum = KEY1_PRESSED;
    else if(GPIO_Pin == GPIO_PIN_13)
        KeyNum = KEY2_PRESSED;
    else if(GPIO_Pin == GPIO_PIN_14)
        KeyNum = KEY3_PRESSED;
    else if(GPIO_Pin == GPIO_PIN_15)
        KeyNum = KEY4_PRESSED;
}
