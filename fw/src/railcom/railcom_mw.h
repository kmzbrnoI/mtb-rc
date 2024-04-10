/* RailCom middleware
 * Lower layer: railcom_ll
 * Higher layer: railcom_addrs
 * Functionality of the middleware:
 *  - low-layer periodic polling in main loop
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "railcom_addrs.h"

// If same addr_lo & same addr_hi is received in channel 1 for at least
// CH1_RELIAB_THRSHD, address is marked as valid. If other values of addr_*
// then previous are received, counters are reset and address is not marked
// as valid.
// Each received byte revalidating the address causes calling of rca_add_or_update.

// One DCC packet: ~ 8 ms (tested on Honza's DR5000)

#define CH1_RELIAB_THRSHD 3

typedef struct {
    uint8_t addr1;
    uint8_t addr2;
    size_t addr1_received_count;
    size_t addr2_received_count;
} RCCh1Buf;

extern RCCh1Buf ch1[RC_TRACKS_COUNT];

#define RC_APP_ID_ADR_LOW 1
#define RC_APP_ID_ADR_HIGH 2

/* Diagnostics ---------------------------------------------------------------*/

typedef struct {
    uint32_t ch1_invalid_data;
    uint32_t ch2_invalid_data;
    uint32_t addr1_received_count_resets;
    uint32_t addr2_received_count_resets;
    uint32_t apid_adr_low_received;
    uint32_t apid_adr_high_received;
    uint32_t ch2_used;
} RCMWTrackDiag;

extern RCMWTrackDiag rc_mw_diag;
extern RCMWTrackDiag rc_mw_track_diag[RC_TRACKS_COUNT];

void rcmw_init(void);
void rcmw_update(void);
void rcmw_mux_to_change(void);

