/* RailCom data protection */

#pragma once

#include <stdint.h>

// 4-8 decoder according to RailCom specification
// 0xFF = invalid value
extern const uint8_t rc_data_decoder[256];
