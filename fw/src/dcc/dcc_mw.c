/* DCC middle-ware implementation */

#include <string.h>
#include "dcc_mw.h"
#include "stm32f1xx_hal.h"
#include "dcc_ll.h"

/* Local variables -----------------------------------------------------------*/

uint16_t dcc_last_address;
volatile unsigned dcc_time_since_address;
DCCMWDiag dcc_mw_diag;

/* Private prototypes --------------------------------------------------------*/

#define DCC_ADDR_INVALID 0xFFFF

void _process_ll(void);
uint16_t _dcc_mf_address(uint8_t byte1, uint8_t byte2);

/* Implementation ------------------------------------------------------------*/

void dcc_mw_init(void) {
    dcc_last_address = 0;
    dcc_time_since_address = DCC_TIME_SINCE_ADDRESS_MAX;
    memset((void*)&dcc_mw_diag, 0, sizeof(dcc_mw_diag));
}

void dcc_mw_update(void) {
    if (dcc_ll_received)
        _process_ll();
}

void _process_ll(void) {
    if (dcc_ll_received_len < 3)
        goto process_end;

    uint16_t addr = _dcc_mf_address(dcc_ll_received_buf[0], dcc_ll_received_buf[1]);
    if (addr == DCC_ADDR_INVALID)
        goto process_end;

    dcc_last_address = addr;
    dcc_time_since_address = 0;
    dcc_mw_diag.mobile_reads++;

process_end:
    dcc_ll_received = false;
}

bool dcc_addr_valid(void) {
    return dcc_time_since_address < DCC_TIME_SINCE_ADDRESS_MAX;
}

uint16_t _dcc_mf_address(uint8_t byte1, uint8_t byte2) {
    // Return multifunction decoder address
    // 0xFFFF = invalid
    if (byte1 < 128)
        return byte1; // Multi-Function decoder with 7bit address
    if (byte1 < 192)
        return DCC_ADDR_INVALID; // Accessory decoder
    if (byte1 < 232)
        return (((uint16_t)byte1 & 0x7) << 8) | (uint16_t)byte2; // Multi-Function decoder with 14bit address
    return DCC_ADDR_INVALID;
}
