/* DCC packets parsing low-level implementation
 * https://www.nmra.org/sites/default/files/s-92-2004-07.pdf
 *
 * DCC signal connected to external interrupt pin
 * timing provided by Timer2
 *
 * demodulation algorithm based on measurement time after rising edge
 *  - very low CPU load, specially if noisy signal
 *  - input polarity can vary
 *
 * Procedure:
 *  1. wait for falling edge of signal
 *  2. wait 77 us
 *  3. read signal state, this is bit value
 *  4. wait for another edge
 */

#include "dcc_ll.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include "gpio.h"

/* Local variables -----------------------------------------------------------*/

TIM_HandleTypeDef htim2; // dcc parsing timer

volatile bool _intr_received = false;
volatile bool _intr_bit_value = false;
volatile DCCLLDiag dcc_ll_diag;

typedef enum {DS_preamble, DS_data, DS_startstopbit} DCC_State;

typedef struct {
    DCC_State state;
    uint8_t bit_buffer;
    unsigned bit_cnt;
    uint8_t byte_buffer[DCC_LL_RECEIVED_MAX_LEN];
    unsigned byte_cnt;
} DemodulatorState;

DemodulatorState _demod_state;

uint8_t dcc_ll_received_buf[DCC_LL_RECEIVED_MAX_LEN];
unsigned dcc_ll_received_len;
bool dcc_ll_received;

/* Private prototypes --------------------------------------------------------*/

void _tim2_init(void);

/* Implementation ------------------------------------------------------------*/

void dcc_ll_init(void) {
    _intr_received = false;
    _demod_state.state = DS_preamble;
    _demod_state.bit_cnt = 0;
    _demod_state.byte_cnt = 0;
    memset((void*)&dcc_ll_diag, 0, sizeof(dcc_ll_diag));

    _tim2_init();
    gpio_dcc_enable_fall_irq();
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
    gpio_dcc_disable_fall_irq();
    TIM2->CNT = 0;
    assert_param(HAL_TIM_Base_Start_IT(&htim2) == HAL_OK);
}

void TIM2_IRQHandler(void) {
    // timer start on falling edge -> high after 77 us = short pulse = log. 1
    if (!_intr_received)
        _intr_bit_value = gpio_pin_read(pin_dcc);
    assert_param(HAL_TIM_Base_Stop(&htim2) == HAL_OK);
    gpio_dcc_enable_fall_irq();
    HAL_TIM_IRQHandler(&htim2);
    _intr_received = true;
}

void dcc_ll_update(void) {
    if (!_intr_received)
        return;

    switch (_demod_state.state) {
    case DS_preamble:
        // Check for 10 successive log 1
        if (_intr_bit_value) {
            // log 1 in preamble -> ok
            _demod_state.bit_cnt++;
         } else {
            // log 0 in preamble -> end of preamble?
            if (_demod_state.bit_cnt >= 10) {
                // ok
                _demod_state.bit_buffer = 0;
                _demod_state.bit_cnt = 0;
                _demod_state.byte_cnt = 0;
                _demod_state.state = DS_data;
            } else {
                // error
                if (_demod_state.bit_cnt > 1) {
                    // when bit_cnt <= 1, probably not an error - probably just last edge of previous DCC packet
                    dcc_ll_diag.logical_0_preamble_soon++;
                }
                _demod_state.bit_cnt = 0;
                // remain in "preamble" state
            }
        }
        break;

    case DS_data:
        _demod_state.bit_buffer <<= 1;
        _demod_state.bit_buffer |= (_intr_bit_value & 1);
        _demod_state.bit_cnt++;
        if (_demod_state.bit_cnt >= 8) // whole byte
            _demod_state.state = DS_startstopbit;
        break;

    case DS_startstopbit: // expecting start or stop bit
        if (_intr_bit_value) {
            // log 1 = whole packet stopbit
            _demod_state.bit_cnt = 1; // log. 1 could be a first bit in the preamble
            _demod_state.byte_buffer[_demod_state.byte_cnt++] = _demod_state.bit_buffer;
            _demod_state.state = DS_preamble;

            // check XOR
            uint8_t xor = 0;
            for (unsigned i = 0; i < _demod_state.byte_cnt; i++)
                xor ^= _demod_state.byte_buffer[i];
            if (xor == 0) {
                if (!dcc_ll_received) { // copy to output buffer only if previous data already processed by higher layers
                    dcc_ll_received_len = _demod_state.byte_cnt-1;
                    memcpy(dcc_ll_received_buf, _demod_state.byte_buffer, _demod_state.byte_cnt-1);
                    dcc_ll_received = true;
                    dcc_ll_diag.packets_received++;
                }
            } else {
                dcc_ll_diag.bad_xor++;
            }
        } else {
            // log 0 = next byte startbit
            _demod_state.byte_buffer[_demod_state.byte_cnt++] = _demod_state.bit_buffer;
            _demod_state.bit_buffer = 0;
            _demod_state.bit_cnt = 0;
            _demod_state.state = DS_data;
        }
        break;
    }

    _intr_received = false;
}
