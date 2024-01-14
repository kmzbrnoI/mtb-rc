#include <stdbool.h>
#include "main.h"
#include "gpio.h"
#include "mtbbus.h"
#include "railcom.h"

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Private function prototypes -----------------------------------------------*/

static void init(void);
static bool init_clock(void);
static bool init_tim1(void);
static bool init_tim2(void);

/* Implementation ------------------------------------------------------------*/

int main(void) {
    init();

    while (true) {
    }
}

/* Init ----------------------------------------------------------------------*/

void init(void) {
    HAL_Init();
    if (!init_clock())
        error_handler();

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    gpio_init();

    gpio_pin_write(pin_led_red, true);
    gpio_pin_write(pin_led_green, true);
    gpio_pin_write(pin_led_blue, true);

    if (!init_tim1())
        error_handler();
    if (!init_tim2())
        error_handler();
    if (!mtbbus_init())
        error_handler();
    if (!railcom_init())
        error_handler();

    HAL_Delay(200);

    gpio_pin_write(pin_led_red, false);
    gpio_pin_write(pin_led_green, false);
    gpio_pin_write(pin_led_blue, false);
}

bool init_clock(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        return false;

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        return false;

    /** Enables the Clock Security System
    */
    HAL_RCC_EnableCSS();

    return true;
}

bool init_tim1(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
        return false;

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
    sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
        return false;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
        return false;

    __HAL_RCC_TIM1_CLK_ENABLE();

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

    return true;
}

bool init_tim2(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_IC_InitTypeDef sConfigIC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65535;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
        return false;

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
        return false;

    if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
        return false;

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        return false;

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
        return false;

    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    return true;
}

/* System stuff --------------------------------------------------------------*/

void error_handler(void) {
    // TODO: set LEDs etc.
    __disable_irq();
    while (true);
}

// Non-maskable interrupt
void NMI_Handler(void) {
    HAL_RCC_NMI_IRQHandler();
    while (1) {
    }
}

void HardFault_Handler(void) {
    while (1) {
    }
}

void MemManage_Handler(void) {
    while (1) {
    }
}

void BusFault_Handler(void) {
    while (1) {
    }
}

void UsageFault_Handler(void) {
    while (1) {
    }
}

// System service call via SWI instruction
void SVC_Handler(void) {
}

// Debug monitor
void DebugMon_Handler(void) {
}

// Pendable request for system service
void PendSV_Handler(void) {
}

// System tick timer
void SysTick_Handler(void) {
    HAL_IncTick();
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line) {
}
#endif /* USE_FULL_ASSERT */

/* "Application" code --------------------------------------------------------*/

// TIM1 capture compare interrupt
void TIM1_CC_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

// TIM2 global interrupt.
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}
