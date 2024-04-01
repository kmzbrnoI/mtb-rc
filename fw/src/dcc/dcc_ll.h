/* DCC low-level signal decoding
 * Output: DCC packet
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void dcc_ll_init(void);

void gpio_on_dcc_fall(void);
