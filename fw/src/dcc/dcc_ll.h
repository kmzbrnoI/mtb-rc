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

typedef struct {
    uint32_t packets_received;
    uint32_t bad_xor;
    uint32_t logical_0_preamble_soon;
} DCCLLDiag;

extern volatile DCCLLDiag dcc_ll_diag;

void dcc_ll_init(void);
void dcc_ll_update(void);

void gpio_on_dcc_fall(void);
