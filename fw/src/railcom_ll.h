/* RailCom USART low-level reading
 * Output: bytes parsed by RailCom
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RC_BAUDRATE 250000
#define RC_CH1_SIZE 2 // bytes
#define RC_CH2_SIZE 5 // bytes

/* Higher layers poll rlccdata, when ready_to_parse is true, higher layer can
 * read the data. Once reading is finished, higher layer sets ready_to_parse = false,
 * which is an indication for low-level layer that next reading could be performed.
 */

typedef struct {
    struct {
        uint8_t buf[RC_CH1_SIZE];
        size_t size;
    } ch1;
    struct {
        uint8_t buf[RC_CH2_SIZE];
        size_t size;
    } ch2;
    bool ready_to_parse;
} RCLLData;

extern volatile RCLLData rclldata;

void railcom_ll_init(void);
void railcom_ll_deinit(void);
