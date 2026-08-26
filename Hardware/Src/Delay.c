#include "Delay.h"

void delay_init(void) {
    // 启用DWT
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint32_t us) {
    uint32_t ticks = us * (SystemCoreClock / 1000000U);
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks);
}

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

void delay_s(uint32_t s) {
    for (uint32_t i = 0; i < s; i++) {
        HAL_Delay(1000);
    }
}