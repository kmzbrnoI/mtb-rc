/* MTBbus communication implementation
 *
 * This library does *not* use DMA, because we want to update CRC after each
 * received byte so we don't have to take a long time to calculate the whole
 * CRC at the end of reception. This is beneficial, because we should respond
 * to the 'inquiry' message as soon as possible.
 */

#include <string.h>
#include <stm32f1xx_ll_usart.h>
#include <stm32f1xx_ll_gpio.h>
#include <stm32f1xx_ll_bus.h>
#include "stm32_assert.h"
#include "mtbbus.h"
#include "gpio.h"
#include "crc16modbus.h"

/* Local variables -----------------------------------------------------------*/

//UART_HandleTypeDef huart3;
//DMA_HandleTypeDef hdma1ch2;

volatile bool initialized = false;

uint8_t mtbbus_output_buf[MTBBUS_OUTPUT_BUF_MAX_SIZE];
uint16_t _mtbbus_ui16_output_buf[MTBBUS_OUTPUT_BUF_MAX_SIZE];
volatile uint8_t mtbbus_output_buf_size = 0;
volatile bool sending = false;

volatile uint8_t mtbbus_input_buf[MTBBUS_INPUT_BUF_MAX_SIZE];
volatile uint8_t mtbbus_input_buf_size = 0;
volatile uint16_t mtbbus_input_byte;
volatile bool receiving = false;
volatile uint16_t received_crc = 0;
volatile uint8_t received_addr;
volatile bool received = false;
volatile bool sent = false;

volatile uint8_t mtbbus_addr;
volatile MtbBusSpeed mtbbus_speed;
void (*mtbbus_on_receive)(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len) = NULL;
void (*mtbbus_on_sent)() = NULL;

/* Local protototypes --------------------------------------------------------*/

static inline void _mtbbus_send_buf();
static inline void _mtbbus_received_ninth(uint8_t data);
static inline void _mtbbus_received_non_ninth(uint8_t data);
static uint32_t _mtbbus_speed(MtbBusSpeed speed);

//void _uart_tx_complete(UART_HandleTypeDef *huart);
//void _uart_rx_complete(UART_HandleTypeDef *huart);

/* Implementation ------------------------------------------------------------*/

void mtbbus_init(uint8_t addr, MtbBusSpeed speed) {
    mtbbus_addr = addr;
    mtbbus_speed = speed;

    LL_USART_InitTypeDef USART_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

    /* USART3_RX Init */
    /*LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_3, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_3, LL_DMA_MDATAALIGN_BYTE);*/

    /* USART3_TX Init */
    /*LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_2, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);
    LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PRIORITY_HIGH);
    LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MODE_NORMAL);
    LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PERIPH_NOINCREMENT);
    LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MEMORY_INCREMENT);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_PDATAALIGN_BYTE);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_2, LL_DMA_MDATAALIGN_BYTE);*/

    /* USART3 interrupt Init */
    //NVIC_SetPriority(USART3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
    //NVIC_EnableIRQ(USART3_IRQn);

    USART_InitStruct.BaudRate = _mtbbus_speed(speed);
    USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_9B;
    USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
    USART_InitStruct.Parity = LL_USART_PARITY_NONE;
    USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
    USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
    USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
    assert_param(LL_USART_Init(USART3, &USART_InitStruct) == SUCCESS);
    LL_USART_ConfigAsyncMode(USART3);
    LL_USART_Enable(USART3);

    gpio_pin_init(pin_mtbbus_tx, LL_GPIO_MODE_ALTERNATE, 0, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_OUTPUT_PUSHPULL);
    gpio_pin_init(pin_mtbbus_rx, LL_GPIO_MODE_FLOATING, 0, LL_GPIO_SPEED_FREQ_HIGH, 0);
    gpio_pin_init(pin_mtbbus_te, LL_GPIO_MODE_OUTPUT, 0, LL_GPIO_SPEED_FREQ_HIGH, 0);
    gpio_uart_in();

    initialized = true;
}

void mtbbus_deinit(void) {
    /*assert_param(HAL_UART_Abort_IT(&huart3) == HAL_OK);

    initialized = false;
    sending = false;
    receiving = false;

    __HAL_RCC_USART3_CLK_DISABLE();

    gpio_uart_in();
    gpio_pin_deinit(pin_mtbbus_tx);
    gpio_pin_deinit(pin_mtbbus_rx);
    gpio_pin_deinit(pin_mtbbus_te);

    assert_param(HAL_UART_UnRegisterCallback(&huart3, HAL_UART_TX_COMPLETE_CB_ID) == HAL_OK);
    assert_param(HAL_UART_UnRegisterCallback(&huart3, HAL_UART_RX_COMPLETE_CB_ID) == HAL_OK);

    HAL_NVIC_DisableIRQ(USART3_IRQn);*/

    // DMA disabling missing
}

void mtbbus_set_speed(MtbBusSpeed speed) {
    mtbbus_deinit();
    mtbbus_init(mtbbus_addr, speed);
}

void USART3_IRQHandler(void) {
    //HAL_UART_IRQHandler(&huart3);
}

void DMA1_Channel2_IRQHandler() {
    //HAL_DMA_IRQHandler(&hdma1ch2);
}

void mtbbus_update(void) {
    if (received) {
        received = false;
        if (mtbbus_on_receive != NULL)
            mtbbus_on_receive(received_addr == 0, mtbbus_input_buf[1],
                              (uint8_t*)mtbbus_input_buf+2, mtbbus_input_buf_size-3);
    }

    if (sent) {
        sent = false;
        if (mtbbus_on_sent != NULL) {
            void (*tmp)() = mtbbus_on_sent;
            mtbbus_on_sent = NULL;
            tmp();
        }
    }
}

uint32_t _mtbbus_speed(MtbBusSpeed speed) {
    switch (speed) {
    case MTBBUS_SPEED_230400: return 230400;
    case MTBBUS_SPEED_115200: return 115200;
    case MTBBUS_SPEED_57600: return 57600;
    case MTBBUS_SPEED_38400:
    default:
        return 38400;
    }
}

/* Sending -------------------------------------------------------------------*/

int mtbbus_send(uint8_t *data, uint8_t size) {
    if (!mtbbus_can_fill_output_buf())
        return 1;
    if (size > MTBBUS_OUTPUT_BUF_MAX_SIZE_USER)
        return 2;

    for (uint8_t i = 0; i < size; i++)
        mtbbus_output_buf[i] = data[i];
    mtbbus_output_buf_size = size;

    mtbbus_send_buf();
    return 0;
}

int mtbbus_send_buf() {
    if (sending)
        return 1;
    sent = false;

    size_t i = mtbbus_output_buf_size;
    uint16_t crc = crc16modbus_bytes(0, (uint8_t*)mtbbus_output_buf, mtbbus_output_buf_size);
    mtbbus_output_buf_size += 2;
    mtbbus_output_buf[i] = crc & 0xFF;
    mtbbus_output_buf[i+1] = (crc >> 8) & 0xFF;
    _mtbbus_send_buf();
    return 0;
}

int mtbbus_send_buf_autolen() {
    if (sending)
        return 1;
    if (mtbbus_output_buf[0] > MTBBUS_OUTPUT_BUF_MAX_SIZE_USER)
        return 2;
    mtbbus_output_buf_size = mtbbus_output_buf[0]+1;
    mtbbus_send_buf();
    return 0;
}

static inline void _mtbbus_send_buf() {
    // Copy ui8 to ui16, because UART uses 9-bit communication
    for (size_t i = 0; i < mtbbus_output_buf_size; i++)
        _mtbbus_ui16_output_buf[i] = mtbbus_output_buf[i]; // ninth bit is alway 0 when sending

    // All data as one transaction
    sending = true;
    gpio_uart_out();
    //HAL_StatusTypeDef result = HAL_UART_Transmit_DMA(&huart3, (uint8_t*)_mtbbus_ui16_output_buf, mtbbus_output_buf_size);
    //assert_param(result == HAL_OK);
    //mtbbus_diag.sent++;
}

void _uart_tx_complete() {
    gpio_uart_in();
    sending = false;
    sent = true;
}

bool mtbbus_can_fill_output_buf() {
    return (initialized) && (!sending);
}

/* Receiving -----------------------------------------------------------------*/

void _uart_rx_complete() {
    bool ninth = (mtbbus_input_byte >> 8) & 1;
    uint8_t data = mtbbus_input_byte & 0xFF;

    if (ninth)
        _mtbbus_received_ninth(data);
    else
        _mtbbus_received_non_ninth(data);

    // Read next byte
    //assert_param(HAL_UART_Receive_IT(&huart3, (uint8_t*)&mtbbus_input_byte, 1) == HAL_OK);
}

static inline void _mtbbus_received_ninth(uint8_t data) {
    if (received) // received data pending
        return;

    received_addr = data;

    if ((received_addr == mtbbus_addr) || (received_addr == 0)) {
        receiving = true;
        mtbbus_input_buf_size = 0;
        received_crc = crc16modbus_byte(0, received_addr);
    }
}

static inline void _mtbbus_received_non_ninth(uint8_t data) {
    if (!receiving)
        return;

    if (mtbbus_input_buf_size < MTBBUS_INPUT_BUF_MAX_SIZE) {
        if ((mtbbus_input_buf_size == 0) || (mtbbus_input_buf_size <= mtbbus_input_buf[0]))
            received_crc = crc16modbus_byte(received_crc, data);
        mtbbus_input_buf[mtbbus_input_buf_size] = data;
        mtbbus_input_buf_size++;
    }

    if (mtbbus_input_buf_size >= mtbbus_input_buf[0]+3) {
        // whole message received
        uint16_t msg_crc = (mtbbus_input_buf[mtbbus_input_buf_size-1] << 8) | (mtbbus_input_buf[mtbbus_input_buf_size-2]);
        if (received_crc == msg_crc) {
            received = true;
            //mtbbus_diag.received++;
        } else {
            //mtbbus_diag.bad_crc++;
        }

        receiving = false;
        received_crc = 0;
    }
}
