/* MTBbus communication implementation */

#include "mtbbus.h"
#include "stm32f1xx_hal.h"
#include "gpio.h"
#include "crc16modbus.h"

/* Local variables -----------------------------------------------------------*/

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

volatile uint8_t mtbbus_output_buf[MTBBUS_OUTPUT_BUF_MAX_SIZE];
volatile uint8_t mtbbus_output_buf_size = 0;
volatile uint8_t mtbbus_next_byte_to_send = 0;
volatile bool sending = false;

volatile uint8_t mtbbus_input_buf[MTBBUS_INPUT_BUF_MAX_SIZE];
volatile uint8_t mtbbus_input_buf_size = 0;
volatile bool receiving = false;
volatile uint16_t received_crc = 0;
volatile uint8_t received_addr;
volatile bool received = false;
volatile bool sent = false;

volatile uint8_t mtbbus_addr;
volatile uint8_t mtbbus_speed;
void (*mtbbus_on_receive)(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len) = NULL;
void (*mtbbus_on_sent)() = NULL;

/* Local protototypes --------------------------------------------------------*/

static void _send_next_byte();
static inline void _mtbbus_send_buf();
static inline void _mtbbus_received_ninth(uint8_t data);
static inline void _mtbbus_received_non_ninth(uint8_t data);

/* Implementation ------------------------------------------------------------*/

void mtbbus_init(uint8_t addr, MtbBusSpeed speed) {
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
    sending = true;
    mtbbus_next_byte_to_send = 0;
    uart_out();

    //while (!(UCSR0A & _BV(UDRE0)));
    //_send_next_byte();
}

static void _send_next_byte() {
    //loop_until_bit_is_set(UCSR0A, UDRE0); // wait for empty transmit buffer
    //UCSR0B &= ~_BV(TXB80); // 9 bit = 0
    //UDR0 = mtbbus_output_buf[mtbbus_next_byte_to_send];
    mtbbus_next_byte_to_send++;
}

/*ISR(USART0_TX_vect) {
    if (mtbbus_next_byte_to_send < mtbbus_output_buf_size) {
        _send_next_byte();
    } else {
        uart_in();
        sending = false;
        sent = true;
    }
}*/

bool mtbbus_can_fill_output_buf() {
    return !sending;
}

/* Receiving -----------------------------------------------------------------*/

/*ISR(USART0_RX_vect) {
    uint8_t status = UCSR0A;
    bool ninth = (UCSR0B >> 1) & 0x01;
    uint8_t data = UDR0;

    if (status & ((1<<FE0)|(1<<DOR0)|(1<<UPE0)))
        return; // return on error

    if (ninth)
        _mtbbus_received_ninth(data);
    else
        _mtbbus_received_non_ninth(data);
}*/

static inline void _mtbbus_received_ninth(uint8_t data) {
    if (received) // received data pending
        return;

    received_addr = data;

    if ((received_addr == mtbbus_addr) || (received_addr == 0)) {
        //UCSR0A &= ~_BV(MPCM0); // Receive rest of message
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
        if (received_crc == msg_crc)
            received = true;

        receiving = false;
        received_crc = 0;
        //UCSR0A |= _BV(MPCM0); // Receive only if 9. bit = 1
    }
}
