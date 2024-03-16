/* RailCom reading implementation */

#include "railcom_ll.h"
#include "assert.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

/* Local variables -----------------------------------------------------------*/

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

volatile RCLLData rclldata;
volatile uint8_t uart1_input_byte;
volatile uint8_t uart2_input_byte;

/* Private prototypes --------------------------------------------------------*/

static void _rc1_init(void);
static void _rc2_init(void);
static inline bool _is_cutout(void);

void _rcll_uart_rx_complete(UART_HandleTypeDef *huart);

/* Implementation ------------------------------------------------------------*/

void railcom_ll_init(void) {
    _rc1_init();
    _rc2_init();

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

void railcom_ll_deinit(void) {
    __HAL_RCC_USART1_CLK_DISABLE();
    gpio_pin_deinit(pin_rc1);
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    __HAL_RCC_USART2_CLK_DISABLE();
    gpio_pin_deinit(pin_rc2);
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

bool _is_cutout(void) {
    return !gpio_pin_read(pin_cutout);
}

void gpio_on_cutout_change(void) {
    gpio_pin_toggle(pin_debug_1);
}

void _rcll_uart_rx_complete(UART_HandleTypeDef *huart) {
}
