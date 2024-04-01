/* DCC packets parsing low-level implementation */

#include "dcc_ll.h"
#include "stm32f1xx_hal.h"

/* Local variables -----------------------------------------------------------*/

TIM_HandleTypeDef htim2; // dcc parsing timer

/* Private prototypes --------------------------------------------------------*/

void _tim2_init(void);

/* Implementation ------------------------------------------------------------*/

void dcc_ll_init(void) {
    _tim2_init();
}

void _tim2_init(void) {
    // TIM2 increment each 1 us, interrupt on timeout
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 48; // 48 MHz / 48 -> 1 us
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 77; // 77 us
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    assert_param(HAL_TIM_Base_Init(&htim2) == HAL_OK);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void gpio_on_dcc_fall(void) {
}

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}
