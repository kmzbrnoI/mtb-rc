#include <stdbool.h>
#include "assert.h"
#include "main.h"
#include "gpio.h"
#include "mtbbus.h"
#include "railcom.h"

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* Private function prototypes -----------------------------------------------*/

static void init(void);
static void init_clock(void);
static void init_tim1(void);
static void init_tim2(void);

void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len); // intentionally not static
static void mtbbus_send_ack(void);
static void mtbbus_send_inputs(uint8_t message_code);
static void mtbbus_send_error(uint8_t code);
static inline void leds_update(void);

/* Implementation ------------------------------------------------------------*/

int main(void) {
    init();

    while (true) {
        mtbbus_update();
    }
}

/* Init ----------------------------------------------------------------------*/

void init(void) {
    bool success = (HAL_Init() == HAL_OK);
    assert_param(success);

    init_clock();

    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    gpio_init();

    gpio_pin_write(pin_led_red, true);
    gpio_pin_write(pin_led_green, true);
    gpio_pin_write(pin_led_blue, true);

    //init_tim1();
    init_tim2();

    //uint8_t _mtbbus_addr = io_get_addr_raw();
    //error_flags.bits.addr_zero = (_mtbbus_addr == 0);
    mtbbus_init(5, MTBBUS_SPEED_115200);
    mtbbus_on_receive = mtbbus_received;

    railcom_init();

    HAL_Delay(200);

    gpio_pin_write(pin_led_red, false);
    gpio_pin_write(pin_led_green, false);
    gpio_pin_write(pin_led_blue, false);
}

void init_clock(void) {
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
    assert_param(HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK);

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    assert_param(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) == HAL_OK);

    /** Enables the Clock Security System
    */
    HAL_RCC_EnableCSS();
}

void init_tim1(void) {
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    //TIM_IC_InitTypeDef sConfigIC = {0};

    __HAL_RCC_TIM1_CLK_ENABLE();

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 65535;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    assert_param(HAL_TIM_Base_Init(&htim1) == HAL_OK);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
    sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
    sClockSourceConfig.ClockFilter = 0;
    assert_param(HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) == HAL_OK);

    //assert_param(HAL_TIM_IC_Init(&htim2) == HAL_OK);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    assert_param(HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) == HAL_OK);

    /*sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    assert_param(HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) == HAL_OK);*/

    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void init_tim2(void) {
    // General-purpose TIM2 @ 10 ms

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 100;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 4800;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    assert_param(HAL_TIM_Base_Init(&htim2) == HAL_OK);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    assert_param(HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) == HAL_OK);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    assert_param(HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) == HAL_OK);

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    HAL_TIM_Base_Start_IT(&htim2);
}

/* System stuff --------------------------------------------------------------*/

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
    __disable_irq();

    gpio_pin_write(pin_led_green, false);
    gpio_pin_write(pin_led_blue, false);
    while (1) {
        gpio_pin_write(pin_led_red, true);
        for (size_t i = 0; i < (1U << 20); i++) { __asm__("NOP"); }
        gpio_pin_write(pin_led_red, false);
        for (size_t i = 0; i < (1U << 20); i++) { __asm__("NOP"); }
    }
}
#endif /* USE_FULL_ASSERT */

/* "Application" code --------------------------------------------------------*/

// TIM1 global interrupt
void TIM1_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim1);
}

// General-purpose TIM2 @ 10 ms
void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
    gpio_pin_toggle(pin_debug_1);
}

/* MTBbus --------------------------------------------------------------------*/

void mtbbus_received(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len) {
    //if (!initialized)
    //    return;

    //error_flags.bits.bad_mtbbus_polarity = false;
    /*if (led_gr_counter == 0) {
        io_led_green_on();
        led_gr_counter = LED_GR_ON;
    }*/
    //_delay_us(2);

    /*mtbbus_timeout = 0;
    if (mtbbus_auto_speed_in_progress)
        mtbbus_auto_speed_received();*/

    gpio_pin_toggle(pin_led_green);

    switch (command_code) {

    case MTBBUS_CMD_MOSI_MODULE_INQUIRY:
        if ((!broadcast) && (data_len >= 1)) {
            //static bool last_input_changed = false;
            //static bool last_diag_changed = false;
            //static bool first_scan = true;
            //bool last_ok = data[0] & 0x01;
            mtbbus_send_ack();
        } else { goto INVALID_MSG; }
        break;


INVALID_MSG:
    default:
        if (!broadcast)
            mtbbus_send_error(MTBBUS_ERROR_UNKNOWN_COMMAND);

    };
}

// Warning: functions below don't check mtbbus_can_fill_output_buf(), bacause
// they should be called ONLY from mtbbus_received event (as MTBbus is
// request-response based bus).

void mtbbus_send_ack(void) {
    mtbbus_output_buf[0] = 1;
    mtbbus_output_buf[1] = MTBBUS_CMD_MISO_ACK;
    mtbbus_send_buf_autolen();
}

void mtbbus_send_inputs(uint8_t message_code) {
    mtbbus_output_buf[0] = 2;
    mtbbus_output_buf[1] = message_code;
    mtbbus_output_buf[2] = 0; // TODO
    mtbbus_send_buf_autolen();
}

void mtbbus_send_error(uint8_t code) {
    mtbbus_output_buf[0] = 2;
    mtbbus_output_buf[1] = MTBBUS_CMD_MISO_ERROR;
    mtbbus_output_buf[2] = code;
    mtbbus_send_buf_autolen();
}
