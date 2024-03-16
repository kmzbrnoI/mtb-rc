/* RailCom data protection */

#pragma once

#include <stdint.h>

#define DECODER_NACK 0xFE
#define DECODER_ACK 0xFD
#define DECODER_BUSY 0xFC
#define DECODER_NUMBER_MAX 0x3F

// 4-8 decoder according to RailCom specification
// 0xFF = invalid value
extern const uint8_t rc_data_decoder[256];
