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

/* Local defines -------------------------------------------------------------*/

#define MTBBUS_USART USART3
#define MTBBUS_USART_CLOCK LL_APB1_GRP1_PERIPH_USART3

/* Local variables -----------------------------------------------------------*/

bool _initialized = false;

uint8_t mtbbus_output_buf[MTBBUS_OUTPUT_BUF_MAX_SIZE];
uint8_t mtbbus_output_buf_size = 0;
int32_t _sent_i = -1; // -1 = not sending

uint8_t _mtbbus_input_buf[MTBBUS_INPUT_BUF_MAX_SIZE];
uint8_t _mtbbus_input_buf_size = 0;
bool _receiving = false;
uint16_t _received_crc = 0;
uint8_t _received_addr;

uint8_t mtbbus_addr;
MtbBusSpeed mtbbus_speed;
void (*mtbbus_on_receive)(bool broadcast, uint8_t command_code, uint8_t *data, uint8_t data_len) = NULL;
void (*mtbbus_on_sent)() = NULL;

/* Local protototypes --------------------------------------------------------*/

static inline void _mtbbus_send_buf(void);
static inline void _mtbbus_received_ninth(uint8_t data);
static inline void _mtbbus_received_non_ninth(uint8_t data);
static uint32_t _mtbbus_speed(MtbBusSpeed speed);

void _mtbbus_tx_complete(void);
void _mtbbus_rx_complete(uint8_t data, bool ninth);
static inline bool _sending(void);

/* Implementation ------------------------------------------------------------*/

void mtbbus_init(uint8_t addr, MtbBusSpeed speed) {
    mtbbus_addr = addr;
    mtbbus_speed = speed;

    LL_USART_InitTypeDef USART_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(MTBBUS_USART_CLOCK);

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
    assert_param(LL_USART_Init(MTBBUS_USART, &USART_InitStruct) == SUCCESS);
    LL_USART_ConfigAsyncMode(MTBBUS_USART);
    LL_USART_Enable(MTBBUS_USART);

    gpio_pin_init(pin_mtbbus_tx, LL_GPIO_MODE_ALTERNATE, 0, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_OUTPUT_PUSHPULL);
    gpio_pin_init(pin_mtbbus_rx, LL_GPIO_MODE_FLOATING, 0, LL_GPIO_SPEED_FREQ_HIGH, 0);
    gpio_pin_init(pin_mtbbus_te, LL_GPIO_MODE_OUTPUT, 0, LL_GPIO_SPEED_FREQ_HIGH, 0);
    gpio_uart_in();

    _initialized = true;
}

void mtbbus_deinit(void) {
    LL_USART_Disable(MTBBUS_USART);

    _initialized = false;
    _sent_i = -1;
    _receiving = false;

    LL_APB1_GRP1_DisableClock(MTBBUS_USART_CLOCK);

    gpio_uart_in();
}

void mtbbus_set_speed(MtbBusSpeed speed) {
    mtbbus_deinit();
    mtbbus_init(mtbbus_addr, speed);
}

void mtbbus_update(void) {
    if (LL_USART_IsActiveFlag_RXNE(USART3)) { // RX not-empty
        LL_USART_ClearFlag_RXNE(USART3);
        uint16_t received = LL_USART_ReceiveData9(USART3);
        _mtbbus_rx_complete(received & 0xFF, received >> 8);
    }
    if (LL_USART_IsActiveFlag_TXE(USART3)) { // TX complete
        LL_USART_ClearFlag_RXNE(USART3);
        _mtbbus_tx_complete();
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

int mtbbus_send_buf(void) {
    if (_sending())
        return 1;

    size_t i = mtbbus_output_buf_size;
    uint16_t crc = crc16modbus_bytes(0, (uint8_t*)mtbbus_output_buf, mtbbus_output_buf_size);
    mtbbus_output_buf_size += 2;
    mtbbus_output_buf[i] = crc & 0xFF;
    mtbbus_output_buf[i+1] = (crc >> 8) & 0xFF;
    _mtbbus_send_buf();
    return 0;
}

int mtbbus_send_buf_autolen(void) {
    if (_sending())
        return 1;
    if (mtbbus_output_buf[0] > MTBBUS_OUTPUT_BUF_MAX_SIZE_USER)
        return 2;
    mtbbus_output_buf_size = mtbbus_output_buf[0]+1;
    mtbbus_send_buf();
    return 0;
}

static inline void _mtbbus_send_buf(void) {
    _sent_i = 0;
    gpio_uart_out();
    LL_USART_TransmitData9(USART3, (((uint16_t)1)<<8) | mtbbus_output_buf[0]);
}

void _mtbbus_tx_complete(void) {
    _sent_i++;

    if (_sent_i < mtbbus_output_buf_size) {
        // Transfer incomplete
        LL_USART_TransmitData9(USART3, mtbbus_output_buf[_sent_i]);
    } else {
        // Transfer complete
        gpio_uart_in();
        _sent_i = -1;
        if (mtbbus_on_sent != NULL) {
            void (*tmp)() = mtbbus_on_sent;
            mtbbus_on_sent = NULL;
            tmp();
        }
    }
}

bool mtbbus_can_fill_output_buf(void) {
    return (_initialized) && (!_sending());
}

bool _sending(void) {
    return (_sent_i != -1);
}

/* Receiving -----------------------------------------------------------------*/

void _mtbbus_rx_complete(uint8_t data, bool ninth) {
    if (ninth)
        _mtbbus_received_ninth(data);
    else
        _mtbbus_received_non_ninth(data);
}

static inline void _mtbbus_received_ninth(uint8_t data) {
    _received_addr = data;

    if ((_received_addr == mtbbus_addr) || (_received_addr == 0)) {
        _receiving = false;
        _mtbbus_input_buf_size = 0;
        _received_crc = crc16modbus_byte(0, _received_addr);
    }
}

static inline void _mtbbus_received_non_ninth(uint8_t data) {
    if (!_receiving)
        return;

    if (_mtbbus_input_buf_size < MTBBUS_INPUT_BUF_MAX_SIZE) {
        if ((_mtbbus_input_buf_size == 0) || (_mtbbus_input_buf_size <= _mtbbus_input_buf[0]))
            _received_crc = crc16modbus_byte(_received_crc, data);
        _mtbbus_input_buf[_mtbbus_input_buf_size] = data;
        _mtbbus_input_buf_size++;
    }

    if (_mtbbus_input_buf_size >= _mtbbus_input_buf[0]+3) {
        // whole message received
        uint16_t msg_crc = (_mtbbus_input_buf[_mtbbus_input_buf_size-1] << 8) | (_mtbbus_input_buf[_mtbbus_input_buf_size-2]);
        if (_received_crc == msg_crc) {
            if (mtbbus_on_receive != NULL)
                mtbbus_on_receive(_received_addr == 0, _mtbbus_input_buf[1],
                                  (uint8_t*)_mtbbus_input_buf+2, _mtbbus_input_buf_size-3);
        }

        _receiving = false;
        _received_crc = 0;
    }
}
