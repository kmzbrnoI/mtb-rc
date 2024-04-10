/* RailCom middleware */

#include <string.h>
#include "railcom_mw.h"
#include "assert.h"
#include "railcom_ll.h"
#include "railcom_data_protection.h"
#include "gpio.h"
#include "dcc_mw.h"

/* Local variables -----------------------------------------------------------*/

size_t rc_mux = 0; // multiplexer range: 0, 1, 2, 3
RCCh1Buf ch1[RC_TRACKS_COUNT];
bool mux_to_change = false;

RCMWTrackDiag rc_mw_diag;
RCMWTrackDiag rc_mw_track_diag[RC_TRACKS_COUNT];

/* Private prototypes --------------------------------------------------------*/

static void _parse_ch1(size_t track, uint8_t buf[], size_t size);
static void _parse_ch2(size_t track, uint8_t buf[], size_t size);
static void _poll_ll(void);
static void _update_ch1_addrs(size_t track);
static uint16_t addr_decode(uint8_t addr1, uint8_t addr2);
static void _mux_apply(void);
static void _mux_change(void);

/* Implementation ------------------------------------------------------------*/

#define DIAG_INC(member, track) { rc_mw_diag.member++; rc_mw_track_diag[track].member++; }

/* Address detection ---------------------------------------------------------*/

void rcmw_init(void) {
    for (size_t i = 0; i < RC_TRACKS_COUNT; i++) {
        ch1[i].addr1_received_count = 0;
        ch1[i].addr2_received_count = 0;
    }
    rc_mux = 0;
    _mux_apply();
    mux_to_change = false;

    memset(&rc_mw_diag, 0, sizeof(rc_mw_diag));
    memset(&rc_mw_track_diag, 0, RC_TRACKS_COUNT*sizeof(*rc_mw_track_diag));
}

void rcmw_update(void) {
    _poll_ll();
    if ((mux_to_change) && (!rc_receiving)) {
        mux_to_change = false;
        _mux_change();
    }
}

void _poll_ll(void) {
    for (size_t i = 0; i < RC_UARTS_COUNT; i++) {
        if (rclldata[i].ready_to_parse) {
            const size_t track = 4*i + rc_mux;
            _parse_ch1(track, (uint8_t*)rclldata[i].ch1.buf, rclldata[i].ch1.size);
            _parse_ch2(track, (uint8_t*)rclldata[i].ch2.buf, rclldata[i].ch2.size);
            rclldata[i].ready_to_parse = false;
            _update_ch1_addrs(track);
        }
    }
}

void _parse_ch1(size_t track, uint8_t buf[], size_t size) {
    if (size != 2)
        return;

    uint8_t decoded[2];
    for (size_t i = 0; i < 2; i++) {
        decoded[i] = rc_data_decoder[buf[i]];
        if (decoded[i] > DECODER_NUMBER_MAX) {
            // data protection mechanism detected invalid data
            DIAG_INC(ch1_invalid_data, track);
            return;
        }
    }

    const unsigned int id = (decoded[0] >> 2) & 0x0F;
    const unsigned int data = ((decoded[0] & 3) << 6) | decoded[1];

    switch (id) {
        case RC_APP_ID_ADR_LOW:
            if (ch1[track].addr1 == data) {
                if (ch1[track].addr1_received_count < CH1_RELIAB_THRSHD)
                    ch1[track].addr1_received_count++;
            } else {
                ch1[track].addr1_received_count = 1;
                DIAG_INC(addr1_received_count_resets, track);
            }
            ch1[track].addr1 = data;
            DIAG_INC(apid_adr_low_received, track);
            break;

        case RC_APP_ID_ADR_HIGH:
            if (ch1[track].addr2 == data) {
                if (ch1[track].addr2_received_count < CH1_RELIAB_THRSHD)
                    ch1[track].addr2_received_count++;
            } else {
                ch1[track].addr2_received_count = 1;
                DIAG_INC(addr2_received_count_resets, track);
            }
            ch1[track].addr2 = data;
            DIAG_INC(apid_adr_high_received, track);
            break;
    }
}

void _update_ch1_addrs(size_t track) {
    if ((ch1[track].addr1_received_count == CH1_RELIAB_THRSHD) && (ch1[track].addr2_received_count == CH1_RELIAB_THRSHD)) {
        rca_add_or_update(track, addr_decode(ch1[track].addr1, ch1[track].addr2));
        DIAG_INC(ch1_addr_added, track);
    }
}

uint16_t addr_decode(uint8_t addr1, uint8_t addr2) {
    // See RailCom specification, chapter 5.2 ADR
    return ((addr1 >> 7) == 0) ? addr2 : (((uint16_t)(addr1 & 0x3F)) << 8) | addr2;
}

void _parse_ch2(size_t track, uint8_t buf[], size_t size) {
    if (size < 1)
        return;

    // consider answer valid iff decoding of all bytes is successful
    for (unsigned i = 0; i < size; i++) {
        if (rc_data_decoder[buf[i]] > DECODER_NUMBER_MAX) {
            DIAG_INC(ch2_invalid_data, track);
            return;
        }
    }

    if (dcc_addr_valid()) {
        rca_add_or_update(track, dcc_last_address);
        DIAG_INC(ch2_addr_added, track);
    }
}

/* Multiplexing --------------------------------------------------------------*/

void rcmw_mux_to_change(void) {
    mux_to_change = true;
}

void _mux_apply(void) {
    gpio_pin_write(pin_mult1, rc_mux&1);
    gpio_pin_write(pin_mult2, rc_mux&2);
}

void _mux_change(void) {
    rc_mux = (rc_mux+1) % 4;
    _mux_apply();
}
