/* RailCom reading implementation */

#include "railcom.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

/* Local variables -----------------------------------------------------------*/

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

/* Private prototypes --------------------------------------------------------*/

bool rc1_init(void);
bool rc2_init(void);

/* Implementation ------------------------------------------------------------*/

bool railcom_init(void) {
    if (!rc1_init())
        return false;
    if (!rc2_init())
        return false;

    return true;
}

bool rc1_init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 250000;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        return false;

    __HAL_RCC_USART1_CLK_ENABLE();

    gpio_pin_init(pin_rc1, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);

    /* USART1 DMA Init */
    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel5;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
        return false;

    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    return true;
}

bool rc2_init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
        return false;

    __HAL_RCC_USART2_CLK_ENABLE();

    gpio_pin_init(pin_rc2, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);

    // TODO: add missing DMA

    return true;
}

void railcom_deinit(void) {
    __HAL_RCC_USART1_CLK_DISABLE();
    gpio_pin_deinit(pin_rc1);
    HAL_DMA_DeInit(huart1.hdmarx);
    HAL_NVIC_DisableIRQ(USART1_IRQn);

    __HAL_RCC_USART2_CLK_DISABLE();
    gpio_pin_deinit(pin_rc2);
    // TODO: deinit DMA
}

void USART1_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart1);
}

void DMA1_Channel5_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}
