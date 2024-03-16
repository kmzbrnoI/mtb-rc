/* RailCom reading implementation */

#include "railcom_ll.h"
#include "assert.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

/* Local variables -----------------------------------------------------------*/

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim4; // cutout length counter

volatile RCLLData rclldata;
volatile uint8_t uart1_input_byte;
volatile uint8_t uart2_input_byte;

/* Private prototypes --------------------------------------------------------*/

static void _rc1_init(void);
static void _rc2_init(void);
static void _tim4_init(void);
static inline bool _is_cutout(void);

void _rcll_uart_rx_complete(UART_HandleTypeDef *huart);

static void cutout_start(void);
static void cutout_end(void);
static void cutout_timeout(void);
static uint16_t cutout_length_us(void);

/* Implementation ------------------------------------------------------------*/

/* Init & deinit -------------------------------------------------------------*/

void railcom_ll_init(void) {
    _rc1_init();
    _rc2_init();
    _tim4_init();

    rclldata.ready_to_parse = false;
    rclldata.ch1.size = 0;
    rclldata.ch2.size = 0;
}

void _rc1_init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();

    huart1.Instance = USART1;
    huart1.Init.BaudRate = RC_BAUDRATE;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    assert_param(HAL_UART_Init(&huart1) == HAL_OK);

    gpio_pin_init(pin_rc1, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    assert_param(HAL_UART_RegisterCallback(&huart1, HAL_UART_RX_COMPLETE_CB_ID, _rcll_uart_rx_complete) == HAL_OK);

    // Start first reading
    assert_param(HAL_UART_Receive_IT(&huart1, (uint8_t*)&uart1_input_byte, 1) == HAL_OK);
}

void _rc2_init(void) {
}

void _tim4_init(void) {
    // TIM4 increment each 1 us, interrupt on timeout
    __HAL_RCC_TIM4_CLK_ENABLE();

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 48; // 48 MHz / 48 -> 1 us
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = RC_CUTOUT_TIMEOUT;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    assert_param(HAL_TIM_Base_Init(&htim4) == HAL_OK);

    /* TIM4 interrupt Init */
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void railcom_ll_deinit(void) {
    __HAL_RCC_USART1_CLK_DISABLE();
    gpio_pin_deinit(pin_rc1);
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    __HAL_RCC_USART2_CLK_DISABLE();
    gpio_pin_deinit(pin_rc2);
}

/* Cutout --------------------------------------------------------------------*/

void TIM4_IRQHandler(void) {
    // Cutout timeout
    HAL_TIM_IRQHandler(&htim4);
    cutout_timeout();
}

void gpio_on_cutout_change(void) {
    if (_is_cutout())
        cutout_start();
    else
        cutout_end();
}

bool _is_cutout(void) {
    return gpio_pin_read(pin_cutout);
}

void cutout_start(void) {
    TIM4->CNT = 0;
    assert_param(HAL_TIM_Base_Start_IT(&htim4) == HAL_OK);
    gpio_pin_write(pin_debug_1, true);
}

void cutout_end(void) {
    assert_param(HAL_TIM_Base_Stop(&htim4) == HAL_OK);
    gpio_pin_write(pin_debug_1, false);
}

void cutout_timeout(void) {
    assert_param(HAL_TIM_Base_Stop(&htim4) == HAL_OK);
    gpio_pin_toggle(pin_debug_2);
}

uint16_t cutout_length_us(void) {
    return TIM4->CNT;
}

/* UART RX -------------------------------------------------------------------*/

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

void _rcll_uart_rx_complete(UART_HandleTypeDef *huart) {
}
