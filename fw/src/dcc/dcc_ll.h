/* DCC low-level signal decoding
 * Output: DCC packet
 * Code partially taken from SWexla project (author: Michal Petrilak)
 */


#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DCC_LL_RECEIVED_MAX_LEN 8

extern uint8_t dcc_ll_received_buf[DCC_LL_RECEIVED_MAX_LEN];
extern unsigned dcc_ll_received_len;
extern bool dcc_ll_received;

void dcc_ll_init(void);
void dcc_ll_update(void);

void gpio_on_dcc_fall(void);
