/* MTBbus communication implementation */

#include "mtbbus.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"

/* Local variables -----------------------------------------------------------*/

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Implementation ------------------------------------------------------------*/

bool mtbbus_init(void) {
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 115200;
    huart3.Init.WordLength = UART_WORDLENGTH_9B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    assert_param(HAL_UART_Init(&huart3) == HAL_OK);

    __HAL_RCC_USART3_CLK_ENABLE();

    gpio_pin_init(pin_mtbbus_tx, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);
    gpio_pin_init(pin_mtbbus_rx, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH, false);

    /* USART3 RX DMA Init */
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    /* USART3_RX Init */
    hdma_usart3_rx.Instance = DMA1_Channel3;
    hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_rx.Init.Mode = DMA_NORMAL;
    hdma_usart3_rx.Init.Priority = DMA_PRIORITY_HIGH;
    assert_param(HAL_DMA_Init(&hdma_usart3_rx) == HAL_OK);

    __HAL_LINKDMA(&huart3,hdmarx,hdma_usart3_rx);

    /* USART3 TX DMA Init */
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    /* USART3_TX Init */
    hdma_usart3_tx.Instance = DMA1_Channel2;
    hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart3_tx.Init.Mode = DMA_NORMAL;
    hdma_usart3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    assert_param(HAL_DMA_Init(&hdma_usart3_tx) == HAL_OK);

    __HAL_LINKDMA(&huart3,hdmatx,hdma_usart3_tx);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

void mtbbus_deinit(void) {
    __HAL_RCC_USART3_CLK_DISABLE();

    gpio_pin_deinit(pin_mtbbus_tx);
    gpio_pin_deinit(pin_mtbbus_rx);

    HAL_DMA_DeInit(huart3.hdmarx);
    HAL_DMA_DeInit(huart3.hdmatx);

    HAL_NVIC_DisableIRQ(USART3_IRQn);
}

void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart3);
}

void DMA1_Channel2_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

void DMA1_Channel3_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_usart3_rx);
}
