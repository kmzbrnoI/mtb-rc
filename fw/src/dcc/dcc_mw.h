/* DCC middle-ware
 * Address parsing
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DCC_TIME_SINCE_ADDRESS_MAX 3 // in 500 us

extern uint16_t dcc_last_address;
extern volatile unsigned dcc_time_since_address; // in 500 us

void dcc_mw_init(void);
void dcc_mw_update(void);

bool dcc_addr_valid(void);
